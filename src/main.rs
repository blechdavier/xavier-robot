mod odometry;
mod geometry;
mod pose_estimator;
mod pose_graph;
mod utils;
mod drivetrain;
mod lidar;
mod ws;
mod icp;
mod paths;

use drivetrain::XAVIERBOT_WHEEL_SEPARATION_METERS;
use geometry::Transform2d;
use odometry::{DifferentialDriveOdometry, WheelOdometry};
use paths::Path;
use pose_graph::{LidarPoseGraph, PoseGraphUpdateResult};
use tokio::time::{sleep, Instant, Duration};
use ws::{DriveCommand, WsPoseGraphNode};
const DURATION_PER_FRAME: Duration = Duration::from_millis(10);

#[tokio::main]
async fn main() {
    let program_start = Instant::now();

    let (state, io) = ws::start_web_server_thread().await;
    let (scan_rx, lidar_health) = lidar::start_lidar_thread(io.clone()).await;
    let (commanded_speeds, heading, wheel_positions, drivetrain_health) = drivetrain::start_drivetrain_thread(io.clone()).await;

    let mut odom = DifferentialDriveOdometry::new(XAVIERBOT_WHEEL_SEPARATION_METERS, heading.read().unwrap().clone(), wheel_positions.read().unwrap().clone());
    let mut pose_graph = LidarPoseGraph::new();

    let mut prev_frame = program_start;
    loop {
        odom.update(*heading.read().unwrap(), &wheel_positions.read().unwrap());
        io.broadcast().emit("odom", odom.get_pose()).await.unwrap();
        dbg!(*heading.read().unwrap(), &wheel_positions.read().unwrap(), odom.get_pose());
        let mut locked = state.cmd_vel.lock().unwrap();
        match &*locked {
            DriveCommand::TeleopVelocity(s) => *commanded_speeds.lock().unwrap() = s.clone(),
            DriveCommand::PathfindToPosition(pos) => {
                let mut new_path = Path { waypoints: Vec::new() };
                new_path.waypoints.push(odom.get_pose().clone());
                new_path.waypoints.push(Transform2d::new(0.2, 0.2, 1.0));
                new_path.waypoints.push(pos.clone());
                io.broadcast().emit("path", &new_path.waypoints).await.unwrap(); // FIXME don't hold the lock here
                *locked = DriveCommand::FollowPath(new_path);
            },
            DriveCommand::FollowPath(path) => {
                let (pursuit_speeds, goal_pose) = path.pure_pursuit(odom.get_pose());
                io.broadcast().emit("pursuitPose", &goal_pose).await.unwrap();
                *commanded_speeds.lock().unwrap() = pursuit_speeds;
            }
        }
        // if let Ok(scan) = scan_rx.try_recv() {
        //     let res = pose_graph.update(odom.get_pose().clone(), scan.to_cartesian_points());
        //     match res {
        //         PoseGraphUpdateResult::Added => io.broadcast().emit("poseGraphNode", &WsPoseGraphNode{tf:odom.get_pose().clone(), scan:scan.to_cartesian_points_ws()}).await.unwrap(),
        //         PoseGraphUpdateResult::LoopClosed => io.broadcast().emit("poseGraph", &{
        //             let mut nodes = Vec::new();
        //             let coords = &pose_graph.backend.nodes;
        //             dbg!(coords.len() ,coords.len()/3);
        //             for i in 0..(coords.len()/3) {
        //                 // scuffed
        //                 nodes.push(WsPoseGraphNode{ tf: Transform2d::new(coords[3*i], coords[3*i+1], coords[3*i+2]), scan: pose_graph.node_scans[i].iter().map(|x| [x[0], x[1]]).collect::<Vec<_>>() });
        //             }
        //             nodes
        //         }).await.unwrap(),
        //         _=>{}
        //     }
        // }
        if let Some(i) = DURATION_PER_FRAME.checked_sub(prev_frame.elapsed()) {
            sleep(i).await;
        } else {
            eprintln!("loop overrun");
        }
        prev_frame = Instant::now();
    }
}