use std::{f64::consts::PI, thread, time::Duration};

use serialport::{ClearBuffer, SerialPort};

use crate::{geometry::Twist2d, odometry::DifferentialDriveWheelPositions};

const XAVIERBOT_METERS_PER_ENCODER_CLICK: f64 = 2.0 * PI * (65.0 / 2.0 / 1000.0) / 1632.0; // TODO real value
pub const XAVIERBOT_WHEEL_SEPARATION_METERS: f64 = 0.2;
const XAVIERBOT_MAX_SPEED_FEASIBLE: f64 = 0.5; // TODO real value

pub struct XavierBotDrivetrain {
    arduino: Box<dyn SerialPort>,
    pub desired_chassis_speeds: Twist2d,
    pub heading: f64,
    pub wheel_positions: DifferentialDriveWheelPositions
}

pub trait Drivetrain<T> {
    /// should be called every frame. reads sensor data
    fn update_inputs(&mut self);
    fn write_outputs(&mut self);
}

impl Drivetrain<DifferentialDriveWheelPositions> for XavierBotDrivetrain {
    fn update_inputs(&mut self) {
        while self.arduino.bytes_to_read().unwrap() >= 12 {
            let mut buf = [0; 12];
            self.arduino.read_exact(&mut buf).unwrap();
            // dbg!(buf);
            let left_encoder = i32::from_le_bytes(buf[0..4].try_into().unwrap());
            let right_encoder = i32::from_le_bytes(buf[4..8].try_into().unwrap());
            let yaw = f32::from_le_bytes(buf[8..12].try_into().unwrap());
            self.wheel_positions.left_wheel_meters = left_encoder as f64 * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.wheel_positions.right_wheel_meters = -(right_encoder as f64) * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.heading = yaw as f64;
        }
    }
    fn write_outputs(&mut self) {
        // let diff = self.desired_chassis_speeds.dtheta * XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0;
        let mut left_mps = -self.heading;
        let mut right_mps = self.heading;

        let norm: f64 = left_mps.abs().max(right_mps.abs()) / XAVIERBOT_MAX_SPEED_FEASIBLE;
        if norm > 1.0 {
            // desaturate wheel speeds
            left_mps /= norm;
            right_mps /= norm;
        }
        dbg!(left_mps, right_mps);
        let left_encoder_clicks_per_sec = (left_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        let right_encoder_clicks_per_sec = -(right_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        self.arduino.write(&[0]).unwrap(); // 0: send wheel velocities
        assert_eq!(self.arduino.write(&left_encoder_clicks_per_sec.to_le_bytes()).unwrap(), 4);
        assert_eq!(self.arduino.write(&right_encoder_clicks_per_sec.to_le_bytes()).unwrap(), 4);
    }
}

impl XavierBotDrivetrain {
    pub fn new(serial_path: &str) -> Self {
        let arduino = serialport::new(serial_path, 115_200)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");
    arduino.clear(ClearBuffer::All).unwrap();
        Self { arduino, desired_chassis_speeds: Twist2d::ZERO, heading: 0.0, wheel_positions: DifferentialDriveWheelPositions::ZERO }
    }

    pub fn reset_serial_odom_alignment(&mut self) {
        dbg!(self.arduino.write(&[2]).unwrap()); // 2: disable odometry sending
        thread::sleep(Duration::from_millis(250));
        self.arduino.clear(ClearBuffer::Input).unwrap();
        dbg!(self.arduino.write(&[1]).unwrap()); // 1: enable odometry sending
    }

    pub fn set_kp(&mut self, kp: f32) {
        self.arduino.write(&[3]).unwrap();
        self.arduino.write(&kp.to_le_bytes()).unwrap();
    }
}