// xy.rs
#![allow(dead_code)]

extern crate alloc;
extern crate no_std_compat as std;

use glam::*;
use std::vec::Vec;

const STEPSZ: f32 = 0.1;

/*
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}
impl Point2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}
*/

#[derive(Debug)]
pub enum ElementType {
    Line,
}

#[derive(Debug)]
pub struct Element {
    pub t: ElementType,
    a: Vec2,
    b: Vec2,
    step_x: f32,
    step_y: f32,
    n_step: usize,
}
impl Element {
    pub fn new_line(a: Vec2, b: Vec2) -> Self {
        let len = (b - a).length();
        let n_step = (len / STEPSZ) as usize;
        let len_x = b.x - a.x;
        let len_y = b.y - a.y;
        let step_x = len_x / n_step as f32;
        let step_y = len_y / n_step as f32;

        Self {
            t: ElementType::Line,
            a,
            b,
            step_x,
            step_y,
            n_step,
        }
    }
}

#[derive(Debug)]
pub struct Drawing {
    pub elts: Vec<Element>,
}
impl Drawing {
    pub fn new() -> Self {
        Self {
            elts: Vec::with_capacity(4),
        }
    }
    pub fn add(&mut self, e: Element) -> &mut Self {
        self.elts.push(e);
        self
    }
}
impl Default for Drawing {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> IntoIterator for &'a Element {
    type Item = Vec2;
    type IntoIter = ElementIterator<'a>;
    fn into_iter(self) -> Self::IntoIter {
        ElementIterator { elem: self, i: 0 }
    }
}

#[derive(Debug)]
pub struct ElementIterator<'a> {
    elem: &'a Element,
    i: usize,
}
impl<'a> Iterator for ElementIterator<'a> {
    type Item = Vec2;
    fn next(&mut self) -> Option<Vec2> {
        if self.i > self.elem.n_step {
            return None;
        }
        let result = Vec2::new(
            self.elem.a.x + self.i as f32 * self.elem.step_x,
            self.elem.a.y + self.i as f32 * self.elem.step_y,
        );
        self.i += 1;
        Some(result)
    }
}

impl<'a> IntoIterator for &'a Drawing {
    type Item = Vec2;
    type IntoIter = DrawingIterator<'a>;
    fn into_iter(self) -> Self::IntoIter {
        Self::IntoIter {
            drw: self,
            e_iter: None,
            n: self.elts.len(),
            i: 0,
        }
    }
}

#[derive(Debug)]
pub struct DrawingIterator<'a> {
    drw: &'a Drawing,
    e_iter: Option<ElementIterator<'a>>,
    n: usize,
    i: usize,
}
impl<'a> Iterator for DrawingIterator<'a> {
    type Item = Vec2;
    fn next(&mut self) -> Option<Vec2> {
        if self.i >= self.n {
            return None;
        }
        if self.e_iter.is_none() {
            self.e_iter.replace(self.drw.elts[self.i].into_iter());
        }

        match self.e_iter.as_mut().unwrap().next() {
            Some(v2) => Some(v2),
            None => loop {
                self.i += 1;
                if self.i >= self.n {
                    return None;
                }

                self.e_iter.replace(self.drw.elts[self.i].into_iter());
                if let Some(v2) = self.e_iter.as_mut().unwrap().next() {
                    return Some(v2);
                }
            },
        }
    }
}

// EOF
