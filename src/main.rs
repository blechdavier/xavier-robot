mod cameras;
mod odometry;
mod geometry;
mod pose_estimator;
mod utils;

use cameras::AprilTagCamera;
use geometry::Transform2d;
use nokhwa::{nokhwa_initialize, utils::Resolution};
use odometry::{DifferentialDriveOdometry, DifferentialDriveWheelPositions};
use pose_estimator::PoseEstimator;

fn main() {
    if cfg!(target_os = "macos") {
        nokhwa_initialize(|granted| {
            println!("User said {}", granted);
        });
    }

    let cam = AprilTagCamera::new("UVC Camera (046d:081b)", Resolution::new(640,480), 30);
    let odom = DifferentialDriveOdometry::new(0.2, 0.0, DifferentialDriveWheelPositions::ZERO);
    let mut pose_estimator = PoseEstimator::new(Transform2d::ZERO, odom);

    loop {
        // pose_estimator.update(gyro_angle_radians, wheel_positions); // TODO
        if let Ok(update) = cam.rx.try_recv() {
            if !update.tags.is_empty() {
                // pose_estimator.add_vision_measurement(estimated_pose, update.timestamp, std_x, std_y, std_theta); // TODO
            }
        }
    }
}