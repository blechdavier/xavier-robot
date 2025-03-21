use std::collections::VecDeque;

use approx::relative_eq;
use assert_approx_eq::assert_approx_eq;

pub struct TimeInterpolatableBuffer<T> {
    history_size: f64,
    buffer: VecDeque<(f64, T)>,
}

impl<T: Interpolate + Clone> TimeInterpolatableBuffer<T> {
    pub fn new(history_size: f64) -> Self {
        Self {
            history_size,
            buffer: VecDeque::new(),
        }
    }
    pub fn add_sample(&mut self, time: f64, value: T) {
        let most_recent = self.buffer.back();
        if most_recent.is_some() && time < most_recent.unwrap().0 {
            panic!(
                "Time must be strictly increasing. received time {} but most recent time was {}",
                time,
                most_recent.unwrap().0
            )
        }
        while let Some((t, _)) = self.buffer.front() {
            if time - t > self.history_size {
                self.buffer.pop_front();
            } else {
                break;
            }
        }
        self.buffer.push_back((time, value));
    }
    pub fn get_value(&self, time: f64) -> Option<T> {
        let mut iter = self.buffer.iter();
        let mut lower_bound = iter.next()?; // return none if there are no values
        if lower_bound.0 > time {
            return Some(lower_bound.1.clone());
        }
        for next in iter {
            if next.0 > time {
                return Some(T::interpolate(
                    &lower_bound.1,
                    &next.1,
                    (time - lower_bound.0) / (next.0 - lower_bound.0),
                ));
            }
            lower_bound = next;
        }
        return Some(self.buffer.back().unwrap().1.clone());
    }
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

#[test]
fn test_buffer_add() {
    let mut buffer = TimeInterpolatableBuffer::new(10.0);
    buffer.add_sample(0.0, 1.0);
    buffer.add_sample(1.0, 2.0);
    buffer.add_sample(2.0, 3.0);
    assert_eq!(buffer.buffer.len(), 3);
}

#[test]
fn test_buffer_cleanup() {
    let mut buffer = TimeInterpolatableBuffer::new(2.0);
    buffer.add_sample(0.0, 1.0);
    buffer.add_sample(0.1, 1.0);
    buffer.add_sample(0.2, 1.0);
    buffer.add_sample(1.0, 2.0);
    buffer.add_sample(2.0, 3.0);
    buffer.add_sample(2.9, 3.0);
    assert_eq!(buffer.buffer.len(), 3);
}

#[test]
fn test_buffer_out_of_bounds() {
    let mut buffer = TimeInterpolatableBuffer::new(200.0);
    buffer.add_sample(-10.0, 1.0);
    buffer.add_sample(10.0, 5.0);
    assert_eq!(buffer.buffer.len(), 2);
    assert_eq!(buffer.get_value(-10000.0), Some(1.0));
    assert_eq!(buffer.get_value(10000.0), Some(5.0));
}

#[test]
fn test_interpolate() {
    let mut buffer = TimeInterpolatableBuffer::new(200.0);
    buffer.add_sample(1.0, 1.0);
    buffer.add_sample(3.0, 5.0);
    buffer.add_sample(4.0, 100.0);
    buffer.add_sample(5.0, 0.0);
    assert_approx_eq!(buffer.get_value(2.0).unwrap(), 3.0);
    assert_approx_eq!(buffer.get_value(4.2).unwrap(), 80.0);
}

pub trait Interpolate {
    fn interpolate(low: &Self, high: &Self, t: f64) -> Self;
}

impl Interpolate for f64 {
    fn interpolate(low: &f64, high: &f64, t: f64) -> f64 {
        low + (high - low) * t
    }
}
