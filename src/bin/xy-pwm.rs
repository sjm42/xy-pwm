// main.rs

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![allow(unused_mut)]
#![allow(dead_code)]
// #![deny(warnings)]

extern crate alloc;
extern crate no_std_compat as std;

use panic_halt as _;

#[cfg(feature = "stm32f411")]
use stm32f4xx_hal as hal;

#[cfg(feature = "stm32f411")]
use hal::otg_fs::{UsbBus, UsbBusType, USB};

use hal::{gpio::*, prelude::*, watchdog::IndependentWatchdog};
use hal::{pac::TIM2, pac::USART1, serial, serial::config::StopBits, timer::*};

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use core::fmt::{self, Write};
use core::mem::MaybeUninit;
use cortex_m::asm;
use glam::*;
use std::prelude::v1::*;
use systick_monotonic::*;
use usb_device::prelude::*;

use xy_pwm::*;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 12 * 1024;

const CMD_OFFSET: u8 = 0x30;
const PWM_MAX: u8 = 192;

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

type UsbdSerial = usbd_serial::SerialPort<'static, UsbBusType>;

type SerialTx = serial::Tx<USART1, u8>;
pub struct MyUsbSerial {
    serial: UsbdSerial,
}

impl fmt::Write for MyUsbSerial {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| {
                if c == b'\n' {
                    self.serial.write(&[b'\r'])?;
                }
                self.serial.write(&[c]).map(|_| ())
            })
            .map_err(|_| fmt::Error)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CmdState {
    Start1 = 0xFD,
    Start2 = 0x02,
    Cmd,
    Data,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum HelloState {
    Idle,
    GoUp,
    GoDown,
}

pub struct MyPwm {
    max: u16,
    ch1: PwmChannel<TIM2, C1>,
    ch2: PwmChannel<TIM2, C2>,
    ch3: PwmChannel<TIM2, C3>,
    ch4: PwmChannel<TIM2, C4>,
}
impl MyPwm {
    pub fn set_duty(&mut self, ch: u8, value: u8) {
        let d = (((value as u32 * self.max as u32) / 256u32) as u16).max(1);
        match ch {
            1 => self.ch1.set_duty(d),
            2 => self.ch2.set_duty(d),
            3 => self.ch3.set_duty(d),
            4 => self.ch4.set_duty(d),
            _ => {}
        }
    }
}

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM1, DMA2_STREAM2, DMA2_STREAM3])]
mod app {

    use crate::*;

    #[monotonic(binds=SysTick, default=true)]
    type MyMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: MyUsbSerial,
        serial_tx: SerialTx,
        led_on: bool,
        led: ErasedPin<Output<PushPull>>,
        state: CmdState,
        cmd: u8,
        pwm: MyPwm,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        hello_state: HelloState,
        i: u8,
    }

    #[cfg(feature = "stm32f411")]
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    #[cfg(feature = "stm32f411")]
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

    static mut DRW: Option<Drawing> = None;
    static mut DRW_ITER: Option<DrawingIterator<'static>> = None;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

        let dp = cx.device;
        let rcc = dp.RCC.constrain();

        #[cfg(feature = "stm32f411")]
        let hse = 25.MHz();
        #[cfg(feature = "stm32f411")]
        let sysclk = 84.MHz();
        #[cfg(feature = "stm32f411")]
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().raw());

        let mut gpioa = dp.GPIOA.split();
        // let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();

        // On Blackpill stm32f411 user led is on PC13, active low
        #[cfg(feature = "black_pill")]
        let mut led = gpioc.pc13.into_push_pull_output().erase();
        led.set_high();

        let ser_tx_pin = gpioa.pa9.into_alternate::<7>();
        // let ser_rx_pin = gpioa.pa10.into_alternate::<7>();

        // Use 115200 bps, 8N1
        let usart1_cfg = serial::config::Config::default()
            .baudrate(115200.bps())
            .wordlength_8()
            .parity_none()
            .stopbits(StopBits::STOP1);
        let mut ser_tx = serial::Serial::tx(dp.USART1, ser_tx_pin, usart1_cfg, &clocks).unwrap();
        write!(ser_tx, "\r\n\nStarting up...\r\n").ok();

        /*
               write!(ser_tx, "* Clocks:\r\n").ok();
               write!(
                   ser_tx,
                   "  requested sysclk {sysclk:?} with hse at {hse:?}\r\n",
               )
               .ok();
               write!(
                   ser_tx,
                   "  sysclk: {sysclk:?}\r\n  hclk: {hclk:?}\r\n",
                   sysclk = clocks.sysclk(),
                   hclk = clocks.hclk()
               )
               .ok();
               write!(
                   ser_tx,
                   "  pclk1: {pclk1:?}\r\n  pclk2: {pclk2:?}\r\n",
                   pclk1 = clocks.pclk1(),
                   pclk2 = clocks.pclk2()
               )
               .ok();
               write!(
                   ser_tx,
                   "  pll48clk: {pll48clk:?}\r\n  ppre1: {ppre1:?}\r\n  ppre2: {ppre2:?}\r\n",
                   pll48clk = clocks.pll48clk(),
                   ppre1 = clocks.ppre1(),
                   ppre2 = clocks.ppre2()
               )
               .ok();
        */

        #[cfg(feature = "black_pill")]
        let pins = (
            gpioa.pa0.into_alternate(),
            gpioa.pa1.into_alternate(),
            gpioa.pa2.into_alternate(),
            gpioa.pa3.into_alternate(),
        );
        #[cfg(feature = "black_pill")]
        let (mut ch1, mut ch2, mut ch3, mut ch4) =
            Timer::new(dp.TIM2, &clocks).pwm_hz(pins, 10.kHz()).split();

        ch1.set_duty(0);
        ch2.set_duty(0);
        ch3.set_duty(0);
        ch4.set_duty(0);

        ch1.enable();
        ch2.enable();
        ch3.enable();
        ch4.enable();

        let pwm = MyPwm {
            max: ch1.get_max_duty(),
            ch1,
            ch2,
            ch3,
            ch4,
        };
        write!(ser_tx, "PWM max duty: {}\r\n", pwm.max).ok();

        // *** Begin USB setup ***

        #[cfg(feature = "stm32f411")]
        {
            let usb = USB {
                usb_global: dp.OTG_FS_GLOBAL,
                usb_device: dp.OTG_FS_DEVICE,
                usb_pwrclk: dp.OTG_FS_PWRCLK,
                pin_dm: gpioa.pa11.into_alternate(),
                pin_dp: gpioa.pa12.into_alternate(),
                hclk: clocks.hclk(),
            };
            unsafe {
                USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
            }
        }

        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Siuro Hacklab")
        .product("XY PWM")
        .serial_number("xy-pwm-42")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();
        write!(ser_tx, "USB init complete.\r\n").ok();

        unsafe {
            DRW.replace(Drawing::new());
            DRW.as_mut()
                .unwrap()
                .add(Element::new_line([-1.0, -1.0].into(), [-1.0, 1.0].into()));
            DRW.as_mut()
                .unwrap()
                .add(Element::new_line([-1.0, 1.0].into(), [1.0, 1.0].into()));
            DRW.as_mut()
                .unwrap()
                .add(Element::new_line([1.0, 1.0].into(), [1.0, -1.0].into()));
            DRW.as_mut()
                .unwrap()
                .add(Element::new_line([1.0, -1.0].into(), [-1.0, -1.0].into()));
            write!(ser_tx, "drw: {:?}\r\n", DRW.as_ref().unwrap()).ok();
            for p in DRW.as_mut().unwrap().into_iter() {
                write!(ser_tx, "{p:?}\r\n").ok();
            }
        }

        // Set "busy" pin up each 100ms, feed the watchdog
        periodic::spawn().ok();

        // Signal hello with pwm
        hello::spawn_after(2000u64.millis()).ok();

        // Start the hardware watchdog
        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(500u32.millis());
        write!(ser_tx, "Watchdog started.\r\n").ok();

        (
            Shared {
                usb_dev,
                usb_serial: MyUsbSerial { serial: usb_serial },
                serial_tx: ser_tx,
                led,
                led_on: false,
                state: CmdState::Start1,
                cmd: 0,
                pwm,
            },
            Local {
                watchdog,
                hello_state: HelloState::GoUp,
                i: HELLO_MIN,
            },
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Wait for interrupt...
            asm::wfi();
        }
    }

    // Feed the watchdog to avoid hardware reset.
    #[task(priority=1, local=[watchdog])]
    fn periodic(cx: periodic::Context) {
        cx.local.watchdog.feed();
        periodic::spawn_after(100u64.millis()).ok();
    }

    #[task(priority=3, capacity=8, shared=[led_on, led])]
    fn led_blink(cx: led_blink::Context, ms: u64) {
        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if !(*led_on) {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(ms.millis()).ok();
            }
        });
    }

    #[task(priority=3, capacity=8, shared=[led_on, led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, usb_serial, state, cmd])]
    fn usb_fs(cx: usb_fs::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut usb_serial = cx.shared.usb_serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;

        (&mut usb_dev, &mut usb_serial, &mut state, &mut cmd).lock(
            |usb_dev, usb_serial, state, cmd| {
                usb_poll(usb_dev, &mut usb_serial.serial, state, cmd);
            },
        );
    }

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        usb_serial: &mut usbd_serial::SerialPort<'static, B>,
        state: &mut CmdState,
        cmd: &mut u8,
    ) {
        if !usb_dev.poll(&mut [usb_serial]) {
            return;
        }

        let mut buf = [0u8; 4];
        if let Ok(count) = usb_serial.read(&mut buf) {
            if count < 1 {
                return;
            }
            for c in buf[0..count].iter_mut() {
                match *state {
                    CmdState::Start1 => {
                        if *c == CmdState::Start1 as u8 {
                            *state = CmdState::Start2;
                        }
                    }
                    CmdState::Start2 => {
                        if *c == CmdState::Start2 as u8 {
                            *state = CmdState::Cmd;
                        } else {
                            *state = CmdState::Start1;
                        }
                    }
                    CmdState::Cmd => {
                        *state = CmdState::Data;
                        // We cannot use TryInto trait here since we are no_std, sigh
                        if *c > CMD_OFFSET && *c < CMD_OFFSET + PWM_MAX {
                            *cmd = *c;
                        } else {
                            // Unknown Cmd
                            *state = CmdState::Start1;
                            *cmd = 0x00;
                        }
                    }
                    CmdState::Data => {
                        // higher priority will execute immediately
                        set_pwm::spawn(*cmd, *c).ok();
                        *state = CmdState::Start1;
                    }
                }
            }
        }
    }

    #[task(priority=7, capacity=8, shared=[pwm])]
    fn set_pwm(cx: set_pwm::Context, ch: u8, da: u8) {
        let mut pwm = cx.shared.pwm;

        // we only support 4 channels here
        if !(CMD_OFFSET + 1..=CMD_OFFSET + 4).contains(&ch) {
            return;
        }

        // i will be 1..4 inclusive
        let i = (ch - CMD_OFFSET) as u8;
        pwm.lock(|pwm| pwm.set_duty(i, da));
    }

    const HELLO_MIN: u8 = 1;
    const HELLO_MAX: u8 = 255;
    const HELLO_IDLE: u8 = 128;
    const HELLO_DELAY: usize = 100;

    #[task(priority=1, capacity=2, local=[hello_state, i, d: usize = 0])]
    fn hello(cx: hello::Context) {
        let hello_state = cx.local.hello_state;
        let i = cx.local.i;
        let d = cx.local.d;

        match hello_state {
            HelloState::GoUp => {
                if *i < HELLO_MAX {
                    *i += 1;
                } else {
                    *hello_state = HelloState::GoDown;
                    *d = 1;
                }
            }
            HelloState::GoDown => {
                // d is delay
                if *d > 0 && *d < HELLO_DELAY {
                    *d += 1;
                } else if *i > HELLO_MIN {
                    *i -= 1;
                    *d = 0;
                } else if *d == 0 {
                    *d = 1;
                } else {
                    *hello_state = HelloState::Idle;
                    *d = 1;
                }
            }
            HelloState::Idle => {
                // d is delay
                if *d > 0 && *d < HELLO_DELAY {
                    *d += 1;
                } else {
                    // OK we are done, in idle state
                    for ch in 1..=4u8 {
                        set_pwm::spawn(CMD_OFFSET + ch, HELLO_IDLE).ok();
                    }
                    draw::spawn_after(1000u64.millis()).ok();
                    return;
                }
            }
        }

        // Note: all do_cmd calls will block (preempt, execute immediately)
        //  because the task has higher priority.
        // This is by design and exactly what we want here.

        for ch in 1..=4u8 {
            set_pwm::spawn(CMD_OFFSET + ch, *i).ok();
        }

        led_blink::spawn(10).ok();
        hello::spawn_after(20u64.millis()).ok();
    }

    #[task(priority = 1, capacity = 2, shared=[serial_tx])]
    fn draw(cx: draw::Context) {
        let mut tx = cx.shared.serial_tx;
        unsafe {
            if DRW_ITER.is_none() {
                // tx.lock(|tx| write!(tx, "draw iter start.\r\n").ok());
                DRW_ITER.replace(DRW.as_mut().unwrap().into_iter());
                // tx.lock(|tx| write!(tx, "draw iter:\r\n{:?}\r\n", DRW_ITER.as_ref().unwrap()).ok());
                led_blink::spawn(10).ok();
            }

            if let Some(v) = DRW_ITER.as_mut().unwrap().next() {
                // we KNOW x,y are betweem -1.0 ... +1.0 here
                let pwm1 = (v.x + 1.0 * 128.0).min(255.0) as u8;
                let pwm2 = (v.y + 1.0 * 128.0).min(255.0) as u8;
                set_pwm::spawn(CMD_OFFSET + 1, pwm1).ok();
                set_pwm::spawn(CMD_OFFSET + 2, pwm2).ok();
            } else {
                // tx.lock(|tx| write!(tx, "draw iter end.\r\n").ok());
                DRW_ITER.take();
            }
        }
        draw::spawn_after(1u64.millis()).ok();
    }
}
// EOF
