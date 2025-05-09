use std::f64::consts::PI;

use crate::geometry::{Transform2d, Twist2d};

pub trait WheelOdometry<WheelPositions> {
    fn update(&mut self, gyro_angle_radians: f64, wheel_positions: &WheelPositions);
    fn get_pose(&self) -> &Transform2d;
    fn reset_pose(
        &mut self,
        gyro_angle_radians: f64,
        wheel_positions: WheelPositions,
        pose: Transform2d,
    );
}

#[derive(Debug, Clone)]
pub struct DifferentialDriveWheelPositions {
    pub left_wheel_meters: f64,
    pub right_wheel_meters: f64,
}

impl DifferentialDriveWheelPositions {
    pub const ZERO: Self = DifferentialDriveWheelPositions {
        left_wheel_meters: 0.0,
        right_wheel_meters: 0.0,
    };
}

#[derive(Debug)]
pub struct DifferentialDriveOdometry {
    pose: Transform2d,
    wheel_separation_meters: f64,
    gyro_offset_radians: f64,
    prev_angle_radians: f64,
    prev_wheel_positions: DifferentialDriveWheelPositions,
}

impl WheelOdometry<DifferentialDriveWheelPositions> for DifferentialDriveOdometry {
    fn update(
        &mut self,
        gyro_angle_radians: f64,
        wheel_positions: &DifferentialDriveWheelPositions,
    ) {
        let angle = gyro_angle_radians - self.gyro_offset_radians;

        let l = wheel_positions.left_wheel_meters - self.prev_wheel_positions.left_wheel_meters;
        let r = wheel_positions.right_wheel_meters - self.prev_wheel_positions.right_wheel_meters;

        // let twist = Twist2d::new((l + r) / 2.0, 0.0, (r - l) / self.wheel_separation_meters); // IF NO GYRO
        let twist = Twist2d::new((l + r) / 2.0, 0.0, angle - self.prev_angle_radians);
        self.pose += Transform2d::from(twist);

        self.prev_wheel_positions.left_wheel_meters = wheel_positions.left_wheel_meters;
        self.prev_wheel_positions.right_wheel_meters = wheel_positions.right_wheel_meters;
        self.prev_angle_radians = angle;
    }

    fn get_pose(&self) -> &Transform2d {
        &self.pose
    }

    fn reset_pose(
        &mut self,
        gyro_angle_radians: f64,
        wheel_positions: DifferentialDriveWheelPositions,
        pose: Transform2d,
    ) {
        self.prev_angle_radians = pose.theta_radians;
        self.prev_wheel_positions = wheel_positions;
        self.gyro_offset_radians = pose.theta_radians - gyro_angle_radians;
        self.pose = pose;
    }
}

impl DifferentialDriveOdometry {
    pub fn new(
        wheel_separation_meters: f64,
        initial_gyro_angle: f64,
        initial_wheel_positions: DifferentialDriveWheelPositions,
    ) -> Self {
        let initial_pose = Transform2d::new(0.0, 0.0, 0.0);
        DifferentialDriveOdometry {
            prev_angle_radians: initial_pose.theta_radians,
            gyro_offset_radians: initial_pose.theta_radians - initial_gyro_angle,
            pose: initial_pose,
            wheel_separation_meters,
            prev_wheel_positions: initial_wheel_positions,
        }
    }
}

#[test]
fn test_odom_forward() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 0.0,
            right_wheel_meters: 0.0,
        },
    );
    odom.update(
        0.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 1.0,
            right_wheel_meters: 1.0,
        },
    );
    assert_eq!(odom.get_pose(), &Transform2d::new(1.0, 0.0, 0.0));
}

#[test]
fn test_odom_quarter_turn() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 0.0,
            right_wheel_meters: 0.0,
        },
    );
    odom.update(
        PI / 2.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: PI / 2.0,
            right_wheel_meters: PI / 2.0,
        },
    );
    assert_eq!(odom.get_pose(), &Transform2d::new(1.0, 1.0, PI / 2.0));
}

#[test]
fn test_odom_semicircle() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 0.0,
            right_wheel_meters: 0.0,
        },
    );
    odom.update(
        PI,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 1.0,
            right_wheel_meters: 1.0,
        },
    );
    assert_eq!(odom.get_pose(), &Transform2d::new(0.0, 2.0 / PI, PI));
}

#[test]
fn test_odom_circle() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 0.0,
            right_wheel_meters: 0.0,
        },
    );
    odom.update(
        2.0 * PI,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 1.0,
            right_wheel_meters: 1.0,
        },
    );
    assert_eq!(odom.get_pose(), &Transform2d::new(0.0, 0.0, 2.0 * PI));
}

#[test]
fn test_odom_full() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 0.0,
            right_wheel_meters: 0.0,
        },
    );
    odom.update(
        0.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: -2.0,
            right_wheel_meters: -2.0,
        },
    );
    odom.update(
        0.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 3.0,
            right_wheel_meters: 3.0,
        },
    );
    odom.update(
        PI / 2.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 3.0 + PI / 2.0,
            right_wheel_meters: 3.0 + PI / 2.0,
        },
    );
    assert_eq!(odom.get_pose(), &Transform2d::new(4.0, 1.0, PI / 2.0));
}

#[test]
fn test_reset_pose() {
    let mut odom = DifferentialDriveOdometry::new(
        0.2,
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 19.0,
            right_wheel_meters: 80.0,
        },
    );
    odom.update(
        PI / 2.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 19.0 + PI / 2.0,
            right_wheel_meters: 80.0 + PI / 2.0,
        },
    );
    odom.reset_pose(
        0.0,
        DifferentialDriveWheelPositions {
            left_wheel_meters: 19.0 + PI / 2.0,
            right_wheel_meters: 80.0 + PI / 2.0,
        },
        Transform2d::new(5.0, 1.0, 0.0),
    );
    assert_eq!(odom.get_pose().clone(), Transform2d::new(5.0, 1.0, 0.0));
    odom.update(
        0.0,
        &DifferentialDriveWheelPositions {
            left_wheel_meters: 19.0 + PI / 2.0,
            right_wheel_meters: 80.0 + PI / 2.0,
        },
    );
    assert_eq!(odom.get_pose().clone(), Transform2d::new(5.0, 1.0, 0.0));
}
