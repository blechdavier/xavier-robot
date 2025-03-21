use std::time::Instant;

use geometry::Transform2d;
use odometry::{DifferentialDriveOdometry, DifferentialDriveWheelPositions};
use pose_estimator::PoseEstimator;

mod geometry;
mod odometry;
mod pose_estimator;
mod utils;

fn main() {
    let odom = DifferentialDriveOdometry::new(0.2, 0.0, DifferentialDriveWheelPositions::ZERO);
    let mut pose_estimator = PoseEstimator::new(Transform2d::ZERO, odom);
    // assert_eq!(
    //     pose_estimator.get_estimated_pose().clone(),
    //     Transform2d::new(0.0, 0.0, 0.0)
    // )
    let start = Instant::now();
    let mut pose = &Transform2d::ZERO;
    for _ in 0..1_000_000 {
        pose_estimator.update(
            0.0,
            DifferentialDriveWheelPositions {
                left_wheel_meters: 0.1,
                right_wheel_meters: 0.1,
            },
        );
        pose = pose_estimator.get_estimated_pose();
    }
    let elapsed = start.elapsed();
    dbg!(elapsed, pose);
}
