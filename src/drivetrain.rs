use std::{f64::consts::PI, time::Duration};

use serialport::SerialPort;

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
        // dbg!(self.arduino.bytes_to_read().unwrap());
        while self.arduino.bytes_to_read().unwrap() > 8 {
            dbg!("letsgo");
            let mut buf = [0; 8];
            self.arduino.read_exact(&mut buf).unwrap();
            let left_encoder = i32::from_le_bytes(buf[0..4].try_into().unwrap());
            let right_encoder = i32::from_le_bytes(buf[4..8].try_into().unwrap());
            self.wheel_positions.left_wheel_meters = left_encoder as f64 * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.wheel_positions.right_wheel_meters = -right_encoder as f64 * XAVIERBOT_METERS_PER_ENCODER_CLICK;
        }
    }
    fn write_outputs(&mut self) {
        todo!()
        // let diff = self.desired_chassis_speeds.dtheta * XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0;
    }
}

impl XavierBotDrivetrain {
    pub fn new(serial_path: &str) -> Self {
        let arduino = serialport::new(serial_path, 115_200)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");
        Self { arduino, desired_chassis_speeds: Twist2d::ZERO, heading: 0.0, wheel_positions: DifferentialDriveWheelPositions::ZERO }
    }
}