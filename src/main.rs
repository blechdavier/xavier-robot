mod odometry;
mod geometry;
mod pose_estimator;
mod pose_graph;
mod utils;
mod drivetrain;
mod lidar;
mod ws;
mod icp;

use drivetrain::DrivetrainStatus;
use lidar::LidarStatus;
use tokio::time::{sleep, Instant, Duration};
const DURATION_PER_FRAME: Duration = Duration::from_millis(10);

#[tokio::main]
async fn main() {
    let program_start = Instant::now();

    let (state, io) = ws::start_web_server_thread().await;
    let (scan_rx, lidar_health) = lidar::start_lidar_thread();
    let (commanded_speeds, heading, wheel_positions, drivetrain_health) = drivetrain::start_drivetrain_thread();

    let mut prev_frame = program_start;
    loop {
        if let Some(s) = &*state.cmd_vel.lock().unwrap() {
            *commanded_speeds.lock().unwrap() = s.clone();
        }
        io.broadcast().emit("lidarStatus",&(*lidar_health.read().unwrap() == LidarStatus::Healthy)).await.unwrap();
        io.broadcast().emit("arduinoStatus",&(*drivetrain_health.read().unwrap() == DrivetrainStatus::Healthy)).await.unwrap();
        if let Some(i) = DURATION_PER_FRAME.checked_sub(prev_frame.elapsed()) {
            sleep(i).await;
        } else {
            eprintln!("loop overrun");
        }
        prev_frame = Instant::now();
    }
}