use std::{
    f64::consts::PI, fmt::Debug, ops::{Add, AddAssign, Neg}
};

use apriltag::Pose;
use nalgebra::{Matrix3, Rotation3, Translation3, Vector3};

use assert_approx_eq::assert_approx_eq;

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

impl Neg for Transform2d {
    type Output = Transform2d;

    fn neg(self) -> Transform2d {
        Transform2d::ZERO + Transform2d::new(0.0, 0.0, -self.theta_radians) + Transform2d::new(-self.x_meters, -self.y_meters, 0.0)
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

#[test]
fn test_inverse_transform2d() {
    let transform = Transform2d::new(1.0, 0.3, 1.0);
    let inverse_transform = -transform.clone();
    assert_eq!(transform.clone() + inverse_transform.clone(), Transform2d::ZERO);
    assert_eq!(inverse_transform + transform, Transform2d::ZERO);
}

#[derive(Debug)]
pub struct Twist2d {
    pub dx: f64,
    pub dy: f64,
    pub dtheta: f64,
}

impl Twist2d {
    pub const ZERO: Self = Self {dx: 0.0, dy: 0.0, dtheta: 0.0};
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

#[derive(Clone)]
pub struct Transform3d {
    pub rotation: Rotation3<f64>,
    pub translation: Vector3<f64>
}

impl Transform3d {
    pub const fn new(rotation: Rotation3<f64>, translation: Vector3<f64>) -> Self {
        Self { rotation, translation }
    }

    /// row-major order expected.
    pub fn from_slices(rotation: &[f64; 9], translation: &[f64; 3]) -> Self {
        let rotation_mat = Matrix3::from_row_slice(rotation);
        assert!(rotation_mat.is_invertible());
        assert_approx_eq!(rotation_mat.determinant(), 1.0);
        let rotation = Rotation3::from_matrix_unchecked(rotation_mat);
        Self {
            rotation,
            translation: Vector3::from_row_slice(translation)
        }
    }

    pub fn inverse(&self) -> Self {
        let rot_inv = self.rotation.inverse();
        Self {
            rotation: rot_inv,
            translation: rot_inv * -self.translation
        }
    }

    pub fn get_yaw(&self) -> f64 {
        // self.rotation.
        todo!()
    }

    pub fn to_transform_2d(&self) -> Transform2d {
        Transform2d { x_meters: self.translation[0], y_meters: self.translation[1], theta_radians: self.get_yaw() }
    }
}

impl Add for Transform3d {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            rotation: self.rotation * rhs.rotation,
            translation: self.translation + self.rotation * rhs.translation,
        } // TODO test this
    }
}

impl Debug for Transform3d {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Transform3d").field("rotation (matrix)", &self.rotation.matrix()).field("rotation (rpy)", &self.rotation.euler_angles()).field("inverse rotation (rpy)", &self.rotation.inverse().euler_angles()).field("translation", &self.translation).finish()
    }
}

impl PartialEq for Transform3d {
    fn eq(&self, other: &Self) -> bool {
        self.rotation.matrix().iter().zip(other.rotation.matrix().iter()).all(|(a, b)| (a-b).abs() < 1e-6) &&
        self.translation.iter().zip(other.translation.iter()).all(|(a, b)| (a-b).abs() < 1e-6)
    }
}

impl From<apriltag::Pose> for Transform3d {
    fn from(pose: Pose) -> Self {
        let a = Matrix3::new(0.0,-1.0,0.0,0.0,0.0,-1.0,1.0,0.0,0.0).try_inverse().unwrap();
        let b = Matrix3::new(0.0,0.0,-1.0,1.0,0.0,0.0,0.0,1.0,0.0);
        let c = Matrix3::new(0.0,-1.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0);
        let rotation_data = Matrix3::from_row_slice(pose.rotation().data());
        let translation = Vector3::from_row_slice(pose.translation().data());
        Self::new(Rotation3::from_matrix_unchecked(b * rotation_data * c), a * translation)
    }
}