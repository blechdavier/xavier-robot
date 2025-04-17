use std::{f64::consts::PI, thread, time::Duration};

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPort, SerialPortBuilderExt, SerialStream};

use crate::{geometry::Twist2d, odometry::DifferentialDriveWheelPositions};

const XAVIERBOT_METERS_PER_ENCODER_CLICK: f64 = 2.0 * PI * (65.0 / 2.0 / 1000.0) / 1632.0; // TODO real value
pub const XAVIERBOT_WHEEL_SEPARATION_METERS: f64 = 0.2;
const XAVIERBOT_MAX_SPEED_FEASIBLE: f64 = 0.5; // TODO real value

pub struct XavierBotDrivetrain {
    arduino: SerialStream,
    pub desired_chassis_speeds: Twist2d,
    pub heading: f64,
    pub wheel_positions: DifferentialDriveWheelPositions
}

pub trait Drivetrain<T> {
    /// should be called every frame. reads sensor data
    async fn update_inputs(&mut self);
    async fn write_outputs(&mut self);
}

impl Drivetrain<DifferentialDriveWheelPositions> for XavierBotDrivetrain {
    async fn update_inputs(&mut self) {
        while self.arduino.bytes_to_read().unwrap() >= 12 {
            let mut buf = [0; 12];
            self.arduino.read_exact(&mut buf).await.unwrap();
            // dbg!(buf);
            let left_encoder = i32::from_le_bytes(buf[0..4].try_into().unwrap());
            let right_encoder = i32::from_le_bytes(buf[4..8].try_into().unwrap());
            let yaw = f32::from_le_bytes(buf[8..12].try_into().unwrap());
            self.wheel_positions.left_wheel_meters = left_encoder as f64 * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.wheel_positions.right_wheel_meters = -(right_encoder as f64) * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.heading = -yaw as f64;
        }
    }
    async fn write_outputs(&mut self) {
        let mut left_mps = self.desired_chassis_speeds.dx - XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0 * self.desired_chassis_speeds.dtheta;
        let mut right_mps = self.desired_chassis_speeds.dx + XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0 * self.desired_chassis_speeds.dtheta;

        let norm: f64 = left_mps.abs().max(right_mps.abs()) / XAVIERBOT_MAX_SPEED_FEASIBLE;
        if norm > 1.0 {
            // desaturate wheel speeds
            left_mps /= norm;
            right_mps /= norm;
        }
        if left_mps.abs() < 0.02 {
            left_mps = 0.0;
        }
        if right_mps.abs() < 0.02 {
            right_mps = 0.0;
        }
        dbg!(left_mps, right_mps);
        let left_encoder_clicks_per_sec = (left_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        let right_encoder_clicks_per_sec = -(right_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        self.arduino.write(&[0]).await.unwrap(); // 0: send wheel velocities
        assert_eq!(self.arduino.write(&left_encoder_clicks_per_sec.to_le_bytes()).await.unwrap(), 4);
        assert_eq!(self.arduino.write(&right_encoder_clicks_per_sec.to_le_bytes()).await.unwrap(), 4);
    }
}

impl XavierBotDrivetrain {
    pub async fn new(serial_path: &str) -> Self {
        let arduino = tokio_serial::new(serial_path, 115_200).timeout(Duration::from_millis(1000)).open_native_async().expect("Failed to open port");
        arduino.clear(tokio_serial::ClearBuffer::All);
        Self { arduino, desired_chassis_speeds: Twist2d::ZERO, heading: 0.0, wheel_positions: DifferentialDriveWheelPositions::ZERO }
    }

    pub async fn reset_serial_odom_alignment(&mut self) {
        dbg!(self.arduino.write(&[2]).await.unwrap()); // 2: disable odometry sending
        thread::sleep(Duration::from_millis(250));
        self.arduino.clear(tokio_serial::ClearBuffer::Input).unwrap();
        dbg!(self.arduino.write(&[1]).await.unwrap()); // 1: enable odometry sending
    }

    pub async fn set_kp(&mut self, kp: f32) {
        self.arduino.write(&[3]).await.unwrap();
        self.arduino.write(&kp.to_le_bytes()).await.unwrap();
    }
}