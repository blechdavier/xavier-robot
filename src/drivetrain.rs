use std::{
    f64::consts::PI,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex, RwLock,
    },
    thread,
    time::Duration,
};

use socketioxide::SocketIo;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{Result, SerialPort, SerialPortBuilderExt, SerialStream};

use crate::{geometry::Twist2d, odometry::DifferentialDriveWheelPositions};

const XAVIERBOT_METERS_PER_ENCODER_CLICK: f64 = 2.0 * PI * (65.0 / 2.0 / 1000.0) / 1632.0; // TODO real value
pub const XAVIERBOT_WHEEL_SEPARATION_METERS: f64 = 0.2;
const XAVIERBOT_MAX_SPEED_FEASIBLE: f64 = 0.5; // TODO real value

pub async fn start_drivetrain_thread(io: SocketIo) -> (
    Arc<Mutex<Twist2d>>,
    Arc<RwLock<f64>>,
    Arc<RwLock<DifferentialDriveWheelPositions>>,
    Arc<RwLock<DrivetrainStatus>>,
) {
    let desired_chassis_speeds = Arc::new(Mutex::new(Twist2d::ZERO));
    let heading = Arc::new(RwLock::new(0.0));
    let wheels = Arc::new(RwLock::new(DifferentialDriveWheelPositions::ZERO));
    let status = Arc::new(RwLock::new(DrivetrainStatus::Initializing));
    io.broadcast().emit("arduinoStatus",&false).await.unwrap();

    let cloned_speeds = desired_chassis_speeds.clone();
    let cloned_heading = heading.clone();
    let cloned_wheels = wheels.clone();
    let cloned_status = status.clone();

    tokio::spawn(async move {
        loop {
            let mut drivetrain;
            loop {
                match XavierBotDrivetrain::new("/dev/ttyACM0").await {
                    Ok(obj) => {drivetrain = obj; break;},
                    Err(e) => {
                        eprintln!("error initializing drivetrain: {}", e);
                        tokio::time::sleep(Duration::from_millis(10)).await;
                    },
                };
            }
            if let Err(e) = drivetrain.reset_serial_odom_alignment().await {
                eprintln!("error initializing drivetrain: {}", e);
                continue;
            }
            *status.write().unwrap() = DrivetrainStatus::Healthy;
            io.broadcast().emit("arduinoStatus",&true).await.unwrap();
            loop {
                if let Err(e) = drivetrain.update_inputs().await {
                    eprintln!("error updating drivetrain inputs: {}", e);
                    break;
                } else {
                    io.broadcast().emit("arduinoStatus",&true).await.unwrap();
                }
                {
                    drivetrain.desired_chassis_speeds = desired_chassis_speeds.lock().unwrap().clone();
                }
                {
                    *heading.write().unwrap() = drivetrain.heading;
                }
                {
                    let mut wheels = wheels.write().unwrap();
                    wheels.left_wheel_meters = drivetrain.wheel_positions.left_wheel_meters;
                    wheels.right_wheel_meters = drivetrain.wheel_positions.right_wheel_meters;
                }
                if let Err(e) = drivetrain.write_outputs().await {
                    eprintln!("error writing drivetrain outputs: {}", e);
                    break;
                };
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }
    });
    (cloned_speeds, cloned_heading, cloned_wheels, cloned_status)
}

#[derive(Debug, Clone, PartialEq)]
pub enum DrivetrainStatus {
    Initializing,
    Healthy,
    Disconnected,
}

pub struct XavierBotDrivetrain {
    arduino: SerialStream,
    pub desired_chassis_speeds: Twist2d,
    pub heading: f64,
    pub wheel_positions: DifferentialDriveWheelPositions,
}

pub trait Drivetrain<T> {
    /// should be called every frame. reads sensor data
    async fn update_inputs(&mut self) -> Result<()>;
    async fn write_outputs(&mut self) -> Result<()>;
}

impl Drivetrain<DifferentialDriveWheelPositions> for XavierBotDrivetrain {
    async fn update_inputs(&mut self) -> Result<()> {
        while self.arduino.bytes_to_read()? >= 12 {
            let mut buf = [0; 12];
            self.arduino.read_exact(&mut buf).await?;
            // dbg!(buf);
            let left_encoder = i32::from_le_bytes(buf[0..4].try_into().unwrap()); // unwrap ok -- these will always be 4 bytes :)
            let right_encoder = i32::from_le_bytes(buf[4..8].try_into().unwrap());
            let yaw = f32::from_le_bytes(buf[8..12].try_into().unwrap());
            self.wheel_positions.left_wheel_meters =
                left_encoder as f64 * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.wheel_positions.right_wheel_meters =
                -(right_encoder as f64) * XAVIERBOT_METERS_PER_ENCODER_CLICK;
            self.heading = -yaw as f64;
        }
        Ok(())
    }
    async fn write_outputs(&mut self) -> Result<()> {
        let mut left_mps = self.desired_chassis_speeds.dx
            - XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0 * self.desired_chassis_speeds.dtheta;
        let mut right_mps = self.desired_chassis_speeds.dx
            + XAVIERBOT_WHEEL_SEPARATION_METERS / 2.0 * self.desired_chassis_speeds.dtheta;

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
        // dbg!(left_mps, right_mps);
        let left_encoder_clicks_per_sec = (left_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        let right_encoder_clicks_per_sec = -(right_mps / XAVIERBOT_METERS_PER_ENCODER_CLICK) as f32;
        self.arduino.write(&[0]).await?; // 0: send wheel velocities
        assert_eq!(
            self.arduino
                .write(&left_encoder_clicks_per_sec.to_le_bytes())
                .await?,
            4
        );
        assert_eq!(
            self.arduino
                .write(&right_encoder_clicks_per_sec.to_le_bytes())
                .await?,
            4
        );
        Ok(())
    }
}

impl XavierBotDrivetrain {
    pub async fn new(serial_path: &str) -> Result<Self> {
        let arduino = tokio_serial::new(serial_path, 115_200)
            .timeout(Duration::from_millis(1000))
            .open_native_async()?;
        arduino.clear(tokio_serial::ClearBuffer::All)?;
        Ok(Self {
            arduino,
            desired_chassis_speeds: Twist2d::ZERO,
            heading: 0.0,
            wheel_positions: DifferentialDriveWheelPositions::ZERO,
        })
    }

    pub async fn reset_serial_odom_alignment(&mut self) -> Result<()> {
        dbg!(self.arduino.write(&[2]).await?); // 2: disable odometry sending
        thread::sleep(Duration::from_millis(250));
        self.arduino
            .clear(tokio_serial::ClearBuffer::Input)
            .unwrap();
        dbg!(self.arduino.write(&[1]).await?); // 1: enable odometry sending
        Ok(())
    }

    pub async fn set_kp(&mut self, kp: f32) {
        self.arduino.write(&[3]).await.unwrap();
        self.arduino.write(&kp.to_le_bytes()).await.unwrap();
    }
}
