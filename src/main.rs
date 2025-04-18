mod cameras;
mod odometry;
mod geometry;
mod pose_estimator;
mod pose_graph;
mod utils;
mod drivetrain;
mod lidar;
mod ws;

use std::{f64::consts::PI, future::IntoFuture, thread, time::{Duration, Instant}};

use apriltag::TagParams;
use cameras::AprilTagCamera;
use drivetrain::{Drivetrain, XavierBotDrivetrain, XAVIERBOT_WHEEL_SEPARATION_METERS};
use geometry::{Transform2d, Transform3d, Twist2d};
use lidar::LidarEngine;
use nalgebra::{Rotation3, Vector3};
use nokhwa::{nokhwa_initialize, utils::Resolution};
use odometry::{DifferentialDriveOdometry, DifferentialDriveWheelPositions};
use pose_estimator::PoseEstimator;
use socketioxide::SocketIoBuilder;
use tokio_serial::SerialPortBuilderExt;
use icp_2d::icp_least_squares;
use tower_http::services::{ServeDir, ServeFile};
use ws::{handler, WebsocketState};

const DURATION_PER_FRAME: Duration = Duration::from_millis(10);

#[tokio::main]
async fn main() {
    if cfg!(target_os = "macos") {
        nokhwa_initialize(|granted| {
            println!("User said {}", granted);
        });
    }

    let program_start = Instant::now();
    let mut lidar = LidarEngine::new(tokio_serial::new("/dev/ttyUSB0", 115_200).open_native_async().unwrap()).await;
    // let mut drivetrain = XavierBotDrivetrain::new("/dev/ttyACM0").await;
    // let odom = DifferentialDriveOdometry::new(XAVIERBOT_WHEEL_SEPARATION_METERS, 0.0, DifferentialDriveWheelPositions::ZERO);
    // let mut pose_estimator = PoseEstimator::new(Transform2d::ZERO, odom, program_start.elapsed());

    // drivetrain.reset_serial_odom_alignment().await;
    // drivetrain.set_kp(0.6).await;
    
    let mut prev_frame = Instant::now();

    let state = WebsocketState::new();
    let (layer, io) = SocketIoBuilder::new().with_state(state.clone()).build_layer();
    io.ns("/", handler);

    let app = axum::Router::new()
        .route_service("/", ServeFile::new("/home/blech/Documents/GitHub/xavier-robot/frontend/dist/index.html"))
        .route_service("/{*wildcard}", ServeDir::new("/home/blech/Documents/GitHub/xavier-robot/frontend/dist"))
        .layer(layer);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    tokio::spawn(axum::serve(listener, app).into_future());

    // drivetrain.reset_serial_odom_alignment().await;

    loop {
        // drivetrain.update_inputs().await;
        // pose_estimator.update_odometry(drivetrain.heading, &drivetrain.wheel_positions, program_start.elapsed());
        // let world_to_robot = pose_estimator.sample_at(program_start.elapsed()).unwrap();
        // dbg!(&world_to_robot);
        // if let Ok(update) = cam.rx.try_recv() {
        //     for tag_measurement in update.tags {
        //         pose_estimator.add_vision_measurement(tag_measurement.world_to_robot.to_transform_2d(), update.timestamp, 0.02, 0.02, 0.3).unwrap();
        //     }
        // }
        // let world_to_target = Transform2d::new(1.5, 0.1, 1.0);
        // let robot_to_target = (-world_to_robot) + world_to_target;
        // let error_twist: Twist2d = robot_to_target.into();
        // drivetrain.desired_chassis_speeds = error_twist * 3.0;
        // dbg!(&drivetrain.desired_chassis_speeds);
        // if let Some(s) = &*state.cmd_vel.lock().unwrap() {
        //     dbg!(&s);
        //     drivetrain.desired_chassis_speeds = s.clone();
        // }
        // drivetrain.write_outputs().await;
        if let Some(scan) = lidar.poll().await {
            if lidar.scans.len() >= 2 {
                let second_scan = &lidar.scans[1];
                let most_recent_scan = lidar.get_most_recent_scan().unwrap();
                // println!("{:.3}",icp_least_squares(&second_scan.to_cartesian_points(), &most_recent_scan.to_cartesian_points(), 10));
                io.broadcast().emit("pointCloud", &most_recent_scan.to_cartesian_points_ws()).await;
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