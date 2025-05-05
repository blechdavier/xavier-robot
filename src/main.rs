mod odometry;
mod geometry;
mod pose_estimator;
mod pose_graph;
mod utils;
mod drivetrain;
mod lidar;
mod ws;
mod icp;

use std::{f64::consts::PI, future::IntoFuture, thread, time::Instant};

use drivetrain::{Drivetrain, XavierBotDrivetrain, XAVIERBOT_WHEEL_SEPARATION_METERS};
use geometry::{Transform2d, Transform3d, Twist2d};
use lidar::LidarEngine;
use nalgebra::{Rotation3, Vector3};
use odometry::{DifferentialDriveOdometry, DifferentialDriveWheelPositions, WheelOdometry};
use pose_estimator::PoseEstimator;
use pose_graph::LidarPoseGraph;
use socketioxide::SocketIoBuilder;
use tokio_serial::SerialPortBuilderExt;
use icp::icp_least_squares;
use tower_http::services::{ServeDir, ServeFile};
use tokio::time::{sleep, Duration};
use ws::{handler, WebsocketState};

const DURATION_PER_FRAME: Duration = Duration::from_millis(10);

#[tokio::main]
async fn main() {
    let program_start = Instant::now();
    let mut lidar = LidarEngine::new(tokio_serial::new("/dev/ttyUSB0", 115_200).open_native_async().unwrap()).await;
    let mut drivetrain = XavierBotDrivetrain::new("/dev/ttyACM0").await;
    let mut odom = DifferentialDriveOdometry::new(XAVIERBOT_WHEEL_SEPARATION_METERS, 0.0, DifferentialDriveWheelPositions::ZERO);
    let mut pose_graph = LidarPoseGraph::new();

    drivetrain.reset_serial_odom_alignment().await;
    drivetrain.set_kp(0.6).await;
    
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
    let mut prev_odom: Option<Transform2d> = None;

    drivetrain.reset_serial_odom_alignment().await;

    loop {
        drivetrain.update_inputs().await;
        odom.update(drivetrain.heading, &drivetrain.wheel_positions);
        if let Some(s) = &*state.cmd_vel.lock().unwrap() {
            drivetrain.desired_chassis_speeds = s.clone();
        }
        drivetrain.write_outputs().await;
        if let Some(scan) = lidar.poll().await {
            if let Some(prev) = prev_odom.clone() {
                let curr = odom.get_pose().clone();
                let diff = -prev + curr.clone();
                if diff.norm() > 0.1 || diff.theta_radians.abs() > 0.5 {
                    println!("adding pose graph node with diff {:?}", &diff);
                    pose_graph.add_scan(diff, scan.to_cartesian_points());
                    prev_odom = Some(curr);
                    io.broadcast().emit("poseGraphNodes", &pose_graph.backend.nodes.data.as_vec()).await.unwrap();
                    io.broadcast().emit("newPoseGraphScan", &scan.to_cartesian_points_ws()).await.unwrap();
                }
            } else if scan.points.len() > 100 { // FIXME real solution
                pose_graph.add_scan(Transform2d::ZERO, scan.to_cartesian_points());
                prev_odom = Some(Transform2d::ZERO);
            }
            io.broadcast().emit("pointCloud", &scan.to_cartesian_points_ws()).await.unwrap();
        }

        if let Some(i) = DURATION_PER_FRAME.checked_sub(prev_frame.elapsed()) {
            thread::sleep(i);
        } else {
            eprintln!("loop overrun");
        }
        prev_frame = Instant::now();
    }
}