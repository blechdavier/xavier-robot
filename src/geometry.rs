use std::{
    f64::consts::PI,
    ops::{Add, AddAssign},
};

use crate::utils::Interpolate;

#[derive(Debug, Clone)]
pub struct Transform2d {
    pub x_meters: f64,
    pub y_meters: f64,
    /// 0 is in the positive x direction, negative rotation is counterclockwise, 365deg is NOT the same as 5deg for interpolation.
    pub theta_radians: f64,
}

impl Transform2d {
    pub const fn new(x_meters: f64, y_meters: f64, theta_radians: f64) -> Self {
        return Transform2d {
            x_meters,
            y_meters,
            theta_radians,
        };
    }

    pub const ZERO: Self = Transform2d::new(0.0, 0.0, 0.0);
}

impl Add for Transform2d {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        let x = rhs.x_meters * self.theta_radians.cos() - rhs.y_meters * self.theta_radians.sin();
        let y = rhs.x_meters * self.theta_radians.sin() + rhs.y_meters * self.theta_radians.cos();
        return Transform2d::new(
            self.x_meters + x,
            self.y_meters + y,
            self.theta_radians + rhs.theta_radians,
        );
    }
}

impl AddAssign for Transform2d {
    fn add_assign(&mut self, rhs: Self) {
        let x = rhs.x_meters * self.theta_radians.cos() - rhs.y_meters * self.theta_radians.sin();
        let y = rhs.x_meters * self.theta_radians.sin() + rhs.y_meters * self.theta_radians.cos();
        self.x_meters += x;
        self.y_meters += y;
        self.theta_radians += rhs.theta_radians;
    }
}

impl Interpolate for Transform2d {
    fn interpolate(low: &Self, high: &Self, t: f64) -> Self {
        // TODO use Twist2D instead of just lerping
        return Transform2d::new(
            f64::interpolate(&low.x_meters, &high.x_meters, t),
            f64::interpolate(&low.y_meters, &high.y_meters, t),
            f64::interpolate(&low.theta_radians, &high.theta_radians, t),
        );
    }
}

impl PartialEq for Transform2d {
    fn eq(&self, other: &Self) -> bool {
        let x_equal = (self.x_meters - other.x_meters).abs() < 1e-9;
        let y_equal = (self.y_meters - other.y_meters).abs() < 1e-9;
        let theta_equal = (self.theta_radians - other.theta_radians).abs() < 1e-9;
        return x_equal && y_equal && theta_equal;
    }
}

#[test]
fn test_interpolate_transform2d() {
    let pose1 = Transform2d::new(0.0, 0.0, 0.0);
    let pose2 = Transform2d::new(1.0, 2.0, 3.0);
    let interpolated = Transform2d::interpolate(&pose1, &pose2, 0.25);
    assert_eq!(interpolated, Transform2d::new(0.25, 0.5, 0.75));
}

#[test]
fn test_add_transform2d() {
    let initial = Transform2d::new(0.5, 0.2, PI / 2.0);
    let transform = Transform2d::new(1.0, 0.3, 1.0);
    assert_eq!(
        Transform2d::new(0.2, 1.2, PI / 2.0 + 1.0),
        initial + transform
    );
}

#[derive(Debug)]
pub struct Twist2d {
    pub dx: f64,
    pub dy: f64,
    pub dtheta: f64,
}

impl Twist2d {
    pub const fn new(dx: f64, dy: f64, dtheta: f64) -> Self {
        Twist2d { dx, dy, dtheta }
    }
}

impl From<Twist2d> for Transform2d {
    fn from(twist: Twist2d) -> Self {
        let s;
        let c;
        if twist.dtheta.abs() < 1e-9 {
            s = 1.0 - 1.0 / 6.0 * twist.dtheta * twist.dtheta;
            c = 0.5 * twist.dtheta;
        } else {
            s = twist.dtheta.sin() / twist.dtheta;
            c = (1.0 - twist.dtheta.cos()) / twist.dtheta;
        };
        Transform2d::new(
            twist.dx * s - twist.dy * c,
            twist.dx * c + twist.dy * s,
            twist.dtheta,
        )
    }
}
