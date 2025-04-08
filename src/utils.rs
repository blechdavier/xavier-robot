use std::{collections::BTreeMap, time::Duration};

use assert_approx_eq::assert_approx_eq;

pub struct TimeInterpolatableBuffer<T> {
    history_size: Duration,
    buffer: BTreeMap<Duration, T>,
}

impl<T: Interpolate + Clone> TimeInterpolatableBuffer<T> {
    pub fn new(history_size: Duration) -> Self {
        Self {
            history_size,
            buffer: BTreeMap::new(),
        }
    }
    pub fn add_sample(&mut self, time_since_program_start: Duration, value: T) {
        let most_recent = self.buffer.last_key_value();
        if most_recent.is_some() && time_since_program_start < *most_recent.unwrap().0 {
            panic!(
                "Time must be strictly increasing. received time {:?} but most recent time was {:?}",
                time_since_program_start,
                most_recent.unwrap().0
            )
        }
        self.clean_up(time_since_program_start);
        self.buffer.insert(time_since_program_start, value);
    }
    pub fn get_value(&self, time: Duration) -> Option<T> {
        let mut iter = self.buffer.iter();
        let mut lower_bound = iter.next()?; // return none if there are no values
        if *lower_bound.0 > time {
            return Some(lower_bound.1.clone());
        }
        for next in iter {
            if *next.0 > time {
                return Some(T::interpolate(
                    &lower_bound.1,
                    &next.1,
                    (time - *lower_bound.0).as_secs_f64() / (*next.0 - *lower_bound.0).as_secs_f64(),
                ));
            }
            lower_bound = next;
        }
        return Some(self.buffer.last_key_value().unwrap().1.clone());
    }
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
    pub fn clean_up(&mut self, current_time: Duration) {
        while let Some((t, _)) = self.buffer.first_key_value() {
            if current_time - *t > self.history_size {
                self.buffer.pop_first();
            } else {
                break;
            }
        }
    }
}

#[test]
fn test_buffer_add() {
    let mut buffer = TimeInterpolatableBuffer::new(Duration::from_secs_f64(10.0));
    buffer.add_sample(Duration::from_secs_f64(0.0), 1.0);
    buffer.add_sample(Duration::from_secs_f64(1.0), 2.0);
    buffer.add_sample(Duration::from_secs_f64(2.0), 3.0);
    assert_eq!(buffer.buffer.len(), 3);
}

#[test]
fn test_buffer_cleanup() {
    let mut buffer = TimeInterpolatableBuffer::new(Duration::from_secs_f64(2.0));
    buffer.add_sample(Duration::from_secs_f64(0.0), 1.0);
    buffer.add_sample(Duration::from_secs_f64(0.1), 1.0);
    buffer.add_sample(Duration::from_secs_f64(0.2), 1.0);
    buffer.add_sample(Duration::from_secs_f64(1.0), 2.0);
    buffer.add_sample(Duration::from_secs_f64(2.0), 3.0);
    buffer.add_sample(Duration::from_secs_f64(2.9), 3.0);
    assert_eq!(buffer.buffer.len(), 3);
}

#[test]
fn test_buffer_out_of_bounds() {
    let mut buffer = TimeInterpolatableBuffer::new(Duration::from_secs_f64(200.0));
    buffer.add_sample(Duration::from_secs_f64(1.0), 1.0);
    buffer.add_sample(Duration::from_secs_f64(10.0), 5.0);
    assert_eq!(buffer.buffer.len(), 2);
    assert_eq!(buffer.get_value(Duration::from_secs_f64(0.0)), Some(1.0)); // TODO consider if this behavior is actually what I want
    assert_eq!(buffer.get_value(Duration::from_secs_f64(10000.0)), Some(5.0));
}

#[test]
fn test_interpolate() {
    let mut buffer = TimeInterpolatableBuffer::new(Duration::from_secs_f64(200.0));
    buffer.add_sample(Duration::from_secs_f64(1.0), 1.0);
    buffer.add_sample(Duration::from_secs_f64(3.0), 5.0);
    buffer.add_sample(Duration::from_secs_f64(4.0), 100.0);
    buffer.add_sample(Duration::from_secs_f64(5.0), 0.0);
    assert_approx_eq!(buffer.get_value(Duration::from_secs_f64(2.0)).unwrap(), 3.0);
    assert_approx_eq!(buffer.get_value(Duration::from_secs_f64(4.2)).unwrap(), 80.0);
}

pub trait Interpolate {
    fn interpolate(low: &Self, high: &Self, t: f64) -> Self;
}

impl Interpolate for f64 {
    fn interpolate(low: &f64, high: &f64, t: f64) -> f64 {
        low + (high - low) * t
    }
}
