mod cameras;
mod odometry;
mod geometry;
mod pose_estimator;
mod utils;
mod drivetrain;

use std::{f64::consts::PI, thread, time::{Duration, Instant}};

use apriltag::TagParams;
use cameras::AprilTagCamera;
use drivetrain::{Drivetrain, XavierBotDrivetrain, XAVIERBOT_WHEEL_SEPARATION_METERS};
use geometry::{Transform2d, Transform3d};
use nalgebra::{Rotation3, Vector3};
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

    let program_start = Instant::now();
    let cam = AprilTagCamera::new("UVC Camera (046d:081b)", Resolution::new(640,480), 30, TagParams {fx:805.53, fy:799.9, cx:319.42, cy:238.24, tagsize:0.1651}, Transform3d::new(Rotation3::from_euler_angles(0.0, -0.925, PI), Vector3::new(-0.093, 0.0, 0.078)), program_start.clone());
    // let mut drivetrain = XavierBotDrivetrain::new("/dev/ttyACM0");
    let odom = DifferentialDriveOdometry::new(XAVIERBOT_WHEEL_SEPARATION_METERS, 0.0, DifferentialDriveWheelPositions::ZERO);
    let mut pose_estimator = PoseEstimator::new(Transform2d::ZERO, odom, program_start.elapsed());

    // drivetrain.reset_serial_odom_alignment();
    // drivetrain.set_kp(0.6);
    
    let mut prev_frame = Instant::now();

    // drivetrain.reset_serial_odom_alignment();

    loop {
        // drivetrain.update_inputs();
        // dbg!(&drivetrain.wheel_positions);
        // pose_estimator.update_odometry(0.0, &drivetrain.wheel_positions);
        dbg!(pose_estimator.sample_at(program_start.elapsed()).unwrap());
        if let Ok(update) = cam.rx.try_recv() {
            for tag_measurement in update.tags {
                pose_estimator.add_vision_measurement(tag_measurement.world_to_robot.to_transform_2d(), update.timestamp, 0.02, 0.02, 0.3).unwrap();
            }
        }
        if let Some(i) = DURATION_PER_FRAME.checked_sub(prev_frame.elapsed()) {
            thread::sleep(i);
        } else {
            eprintln!("loop overrun");
        }
        prev_frame = Instant::now();
    }
}