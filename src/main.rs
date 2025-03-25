mod cameras;
mod odometry;
mod geometry;
mod pose_estimator;
mod utils;
mod drivetrain;

use std::{thread, time::{Duration, Instant, SystemTime, UNIX_EPOCH}};

use apriltag::TagParams;
use cameras::AprilTagCamera;
use drivetrain::{Drivetrain, XavierBotDrivetrain, XAVIERBOT_WHEEL_SEPARATION_METERS};
use geometry::Transform2d;
use nokhwa::{nokhwa_initialize, utils::Resolution};
use odometry::{DifferentialDriveOdometry, DifferentialDriveWheelPositions};
use pose_estimator::PoseEstimator;

const DURATION_PER_FRAME: Duration = Duration::from_millis(10);

fn main() {
    if cfg!(target_os = "macos") {
        nokhwa_initialize(|granted| {
            println!("User said {}", granted);
        });
    }

    let cam = AprilTagCamera::new("UVC Camera (046d:081b)", Resolution::new(640,480), 30, TagParams{fx:805.53, fy:799.9, cx:319.42, cy:238.24, tagsize:0.1651});
    // let mut drivetrain = XavierBotDrivetrain::new("/dev/ttyACM0");
    let odom = DifferentialDriveOdometry::new(XAVIERBOT_WHEEL_SEPARATION_METERS, 0.0, DifferentialDriveWheelPositions::ZERO);
    let mut pose_estimator = PoseEstimator::new(Transform2d::ZERO, odom);

    drivetrain.reset_serial_odom_alignment();
    drivetrain.set_kp(0.6);
    
    let mut prev_frame = Instant::now();

    // drivetrain.reset_serial_odom_alignment();

    loop {
        // drivetrain.update_inputs();
        // dbg!(&drivetrain.wheel_positions);
        // pose_estimator.update(0.0, &drivetrain.wheel_positions);
        // dbg!(pose_estimator.get_estimated_pose());
        // if let Ok(update) = cam.rx.try_recv() {
        //     if !update.tags.is_empty() {
        //         println!("vision pipeline latency is {:?}.", SystemTime::now().duration_since(UNIX_EPOCH).unwrap() - Duration::from_secs_f64(update.timestamp));
        //         // pose_estimator.add_vision_measurement(estimated_pose, update.timestamp, std_x, std_y, std_theta); // TODO
        //     }
        // }
        if let Some(i) = DURATION_PER_FRAME.checked_sub(prev_frame.elapsed()) {
            thread::sleep(i);
        } else {
            eprintln!("loop overrun");
        }
        prev_frame = Instant::now();
    }
}