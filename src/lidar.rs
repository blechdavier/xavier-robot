/*

// Commands without payload and response
#define SL_LIDAR_CMD_STOP                   0x25
#define SL_LIDAR_CMD_SCAN                   0x20
#define SL_LIDAR_CMD_FORCE_SCAN             0x21
#define SL_LIDAR_CMD_RESET                  0x40

// Commands with payload but no response
#define SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM   0x90 //added in fw 1.30

// Commands without payload but have response
#define SL_LIDAR_CMD_GET_DEVICE_INFO        0x50
#define SL_LIDAR_CMD_GET_DEVICE_HEALTH      0x52

#define SL_LIDAR_CMD_GET_SAMPLERATE         0x59 //added in fw 1.17

#define SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL    0xA8


// Commands with payload and have response
#define SL_LIDAR_CMD_EXPRESS_SCAN           0x82 //added in fw 1.17
#define SL_LIDAR_CMD_HQ_SCAN                0x83 //added in fw 1.24
#define SL_LIDAR_CMD_GET_LIDAR_CONF         0x84 //added in fw 1.24
#define SL_LIDAR_CMD_SET_LIDAR_CONF         0x85 //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define SL_LIDAR_CMD_SET_MOTOR_PWM          0xF0
#define SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     0xFF
*/

use std::f64::consts::PI;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};
use std::sync::{mpsc, Arc, RwLock};

use serde::Serialize;
use socketioxide::SocketIo;
use tokio_serial::{available_ports, Error, ErrorKind, SerialPort, SerialPortBuilderExt, SerialPortType};
use tokio_serial::SerialStream;

use nalgebra::Vector2;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::geometry::Transform2d;

const ROBOT_TO_LIDAR: Transform2d = Transform2d::new(-0.085, -0.01, PI / 2.0);

pub async fn start_lidar_thread(io: SocketIo) -> (Receiver<LidarScan>, Arc<RwLock<LidarStatus>>){
    let (tx, rx) = mpsc::channel::<LidarScan>();
    let lidar_status = Arc::new(RwLock::new(LidarStatus::Initializing));
    io.broadcast().emit("lidarStatus",&false).await.unwrap();
    let cloned = lidar_status.clone();

    tokio::spawn(async move {
        let mut lidar = LidarEngine::new().await;
        loop {
            match lidar.poll().await {
                Ok(Some(scan)) => {
                    if scan.points.len() > 10 { // sometimes it starts a scan and there are only like 2 points... we don't want to do scan matching w those fake scans
                        tx.send(scan.clone()).unwrap();
                    }
                    *lidar_status.write().unwrap() = LidarStatus::Healthy;
                    io.broadcast().emit("lidarStatus",&true).await.unwrap();
                }
                Ok(None) => {}
                Err(e) => {
                    eprintln!("Error in Lidar Thread: {:?}, {}", e.kind, e.description);
                    *lidar_status.write().unwrap() = LidarStatus::UnknownError;
                    io.broadcast().emit("lidarStatus",&false).await.unwrap();
                    println!("Reinitializing Lidar");
                    lidar = LidarEngine::new().await;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    });
    (rx, cloned)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LidarStatus {
    Initializing,
    Healthy,
    ProtocolError,
    UnknownError
}

pub struct LidarEngine {
    pub port: SerialStream,
    pub scan_packets: Vec<ScanPacket>,
    pub scans: Vec<LidarScan>,
}

impl LidarEngine {
    pub async fn new() -> Self {
        loop {
            let ports = available_ports().unwrap();
            let port = ports.iter().find(|port| if let SerialPortType::UsbPort(port_info) = &port.port_type {
                port_info.product == Some("CP2102 USB to UART Bridge Controller".to_string())
            } else {false});
            if let Some(port) = port {
                match tokio_serial::new(port.port_name.clone(), 115_200).open_native_async() {
                    Ok(port) => {
                        let mut engine = Self {
                            port,
                            scan_packets: Vec::new(),
                            scans: Vec::new(),
                        };
                        engine.init().await;
                        return engine;
                    }
                    Err(e) => {
                        eprintln!("Error initializing LidarEngine: {}", e);
                        if e.description == "Device or resource busy" {
                            println!("Waiting 1000ms to attempt to fix this.");
                            tokio::time::sleep(Duration::from_millis(1000)).await;
                        }
                    }
                }
            } else {
                eprintln!("No matching port found for lidar");
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn init(&mut self) {
        println!("Initializing Lidar");
        'outer: loop {
            println!("Sending stop packet");
            LidarRequest::Stop.write(&mut self.port).await.unwrap();
            while self.port.bytes_to_write().unwrap() > 0 {} // FIXME could hang here
            // clear buffer
            println!("Clearing buffer");
            dbg!(self.port.bytes_to_read().unwrap());

            self.port.clear(tokio_serial::ClearBuffer::Input).unwrap();
            println!("Sending whatever packet this is");
            self.port
                .write(&[0xa5, 0x82, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x21])
                .await
                .unwrap();
            let start_this_step = Instant::now();
            while self.port.bytes_to_read().unwrap() < 7 {
                if start_this_step.elapsed() > Duration::from_millis(500) {
                    println!("Packet ack took too long. Trying again. There are {} bytes to write.", self.port.bytes_to_write().unwrap());
                    // self.port.write(&[0, 0, 0, 0, 0]).await.unwrap();
                    continue 'outer;
                }
            }

            let mut buf = [0; 7];
            self.port.read_exact(&mut buf).await.unwrap();
            if buf == [0xa5, 0x5a, 0x84, 0x00, 0x00, 0x40, 0x84] {
                println!("Lidar initialized");
                break;
            } else {
                dbg!(buf);
                println!("Lidar initialization failed, retrying.");
            }
        }
    }

    /// See if there are new scan packets and process them accordingly in order to optionally get a new scan
    pub async fn poll(&mut self) -> tokio_serial::Result<Option<&LidarScan>> {
        if self.port.bytes_to_read()? >= 132 {
            let scan_count = self.scans.len();
            let mut buffer = [0; 132];
            self.port.read_exact(&mut buffer).await.unwrap();
            match ScanPacket::from_buffer(&buffer) {
                Ok(packet) => self.scan_packets.push(packet),
                Err(ScanPacketParseError::SyncByteMismatch) => {
                    if self.port.bytes_to_read().unwrap() > 0 {
                        // attempt to realign
                        let mut buffer = [0; 1];
                        self.port.read_exact(&mut buffer).await.unwrap();
                    }
                    todo!()
                }
                Err(ScanPacketParseError::ChecksumMismatch) => return Err(Error::new(tokio_serial::ErrorKind::Unknown, "checksum mismatch"))
            }
            if self.scan_packets.len() > 1 {
                for i in 0..32 {
                    let mut dist_q2 = [0; 3];

                    let combined_x3 =
                        self.scan_packets[self.scan_packets.len() - 2].ultra_cabins[i];

                    // unpack
                    let dist_major1 = combined_x3 & 0xFFF;
                    let mut dist_predict1 = ((combined_x3 as i32) << 10) >> 22;
                    let mut dist_predict2 = (combined_x3 as i32) >> 22;

                    let dist_major2 = if i == 31 {
                        &self.scan_packets[self.scan_packets.len() - 1].ultra_cabins[0] & 0xFFF
                    } else {
                        &self.scan_packets[self.scan_packets.len() - 2].ultra_cabins[i + 1] & 0xFFF
                    };

                    let mut scale_level1 = 0;
                    let mut scale_level2 = 0;

                    let dist_major1 = varbitscale_decode(dist_major1, &mut scale_level1);
                    let dist_major2 = varbitscale_decode(dist_major2, &mut scale_level2);

                    let mut dist_base1 = dist_major1;
                    let dist_base2 = dist_major2;

                    if dist_major1 == 0 && dist_major2 != 0 {
                        dist_base1 = dist_major2;
                        scale_level1 = scale_level2;
                    }

                    dist_q2[0] = dist_major1 << 2;
                    if dist_predict1 as u32 == 0xFFFFFE00 || dist_predict1 == 0x1FF {
                        dist_q2[1] = 0
                    } else {
                        dist_predict1 <<= scale_level1;
                        dist_q2[1] = ((dist_base1 as i32 + dist_predict1) << 2) as u32;
                    }

                    if dist_predict2 as u32 == 0xFFFFFE00 || dist_predict2 == 0x1FF {
                        dist_q2[2] = 0
                    } else {
                        dist_predict2 <<= scale_level2;
                        dist_q2[2] = ((dist_base2 as i32 + dist_predict2) << 2) as u32;
                    }

                    let start_angle_q6 =
                        self.scan_packets[self.scan_packets.len() - 2].start_angle_q6 as i32;
                    let angle_diff_q6 = (self.scan_packets[self.scan_packets.len() - 1]
                        .start_angle_q6 as i32
                        - start_angle_q6 as i32)
                        .rem_euclid(360 * 64);

                    for j in 0..3 {
                        let point = LidarPoint {
                            angle_q6: (start_angle_q6 as u16
                                + (angle_diff_q6 as f64 * (i as f64 / 32.0 + j as f64 / 96.0))
                                    as u16)
                                % (360 * 64),
                            distance_q0: dist_q2[j] >> 2,
                            index: j as u8,
                        };

                        // reject 0 distance points, they represent points which are either too far or too close to be detected
                        if point.distance_q0 != 0 {
                            self.add_point(point);
                        }
                    }
                }
            }
            if self.scans.len() >= 2 && scan_count != self.scans.len() { // TODO somehow check here if it is a full scan or not
                return Ok(Some(&self.scans[self.scans.len() - 2]));
            }
        }
        Ok(None) // no scan this time :)
    }

    fn add_point(&mut self, point: LidarPoint) {
        if self.scans.is_empty() {
            self.scans.push(LidarScan { points: Vec::new() });
        }
        let current_scan = self.scans.last_mut().unwrap();
        if current_scan.points.is_empty() {
            current_scan.points.push(point);
        } else {
            // wrap once the scan is complete
            let last_point = current_scan.points.last().unwrap();
            if last_point.angle_q6 > point.angle_q6 {
                self.scans.push(LidarScan {
                    points: vec![point],
                });
            } else {
                current_scan.points.push(point);
            }
        }
    }

    pub fn get_most_recent_scan(&self) -> Option<&LidarScan> {
        if self.scans.len() >= 2 {
            Some(&self.scans[self.scans.len() - 2])
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LidarPoint {
    pub angle_q6: u16,
    pub distance_q0: u32,
    pub index: u8,
}

impl LidarPoint {
    pub fn get_angle_degrees(&self) -> f32 {
        self.angle_q6 as f32 / 64.0
    }
    pub fn get_angle_rad(&self) -> f32 {
        self.angle_q6 as f32 * std::f32::consts::PI / 180.0 / 64.0
    }
    pub fn get_angle_rad_f64(&self) -> f64 {
        self.angle_q6 as f64 * std::f64::consts::PI / 180.0 / 64.0
    }
    pub fn to_cartesian(&self) -> Vector2<f64> {
        let angle = ROBOT_TO_LIDAR.theta_radians - self.get_angle_rad_f64();
        Vector2::new(
            self.distance_q0 as f64 * angle.cos() / 1000.0 + ROBOT_TO_LIDAR.x_meters,
            self.distance_q0 as f64 * angle.sin() / 1000.0 + ROBOT_TO_LIDAR.y_meters,
        )
    }
    pub fn to_cartesian_ws(&self) -> [f64; 2] {
        let angle = ROBOT_TO_LIDAR.theta_radians - self.get_angle_rad_f64();
        [
            self.distance_q0 as f64 * angle.cos() / 1000.0 + ROBOT_TO_LIDAR.x_meters,
            self.distance_q0 as f64 * angle.sin() / 1000.0 + ROBOT_TO_LIDAR.y_meters,
        ]
    }
}

pub struct ScanPacket {
    pub timestamp: Instant,
    pub start_bit: bool,
    pub start_angle_q6: u16,
    pub ultra_cabins: [u32; 32],
}

enum ScanPacketParseError {
    SyncByteMismatch,
    ChecksumMismatch,
}

impl ScanPacket {
    fn from_buffer(bytes: &[u8; 132]) -> Result<Self, ScanPacketParseError> {
        let timestamp = Instant::now();
        let sync = (bytes[0] & 0xF0) | (bytes[1] >> 4);
        if sync != 0xa5 {
            println!("sync byte did not match with buffer {:x?}", bytes);
            return Err(ScanPacketParseError::SyncByteMismatch);
        }
        let checksum = (bytes[0] & 0xF) | (bytes[1] << 4);
        let mut check_checksum = 0;
        for i in 2..bytes.len() {
            check_checksum ^= bytes[i];
        }
        if check_checksum != checksum {
            println!("checksum did not match");
            return Err(ScanPacketParseError::ChecksumMismatch);
        }
        let start_bit = bytes[3] & 0b1000_0000 != 0;
        let start_angle_q6 = u16::from_le_bytes([bytes[2], bytes[3] & 0b0111_1111]);

        if start_bit {
            println!("New Scan Started at {}deg", start_angle_q6 as f32 / 64.0);
        }

        let mut ultra_cabins = Vec::with_capacity(32);
        for i in 0..32 {
            let offset = i * 4;
            let cabin = u32::from_le_bytes(bytes[(4 + offset)..(8 + offset)].try_into().unwrap());
            ultra_cabins.push(cabin);
        }
        Ok(ScanPacket {
            timestamp,
            start_bit,
            start_angle_q6,
            ultra_cabins: ultra_cabins.try_into().unwrap(),
        })
    }

    fn get_start_angle_radians(&self) -> f32 {
        self.start_angle_q6 as f32 * std::f32::consts::PI / 180.0 / 64.0
    }
}

#[derive(Debug, Clone)]
pub struct LidarScan {
    pub points: Vec<LidarPoint>,
}

impl LidarScan {
    pub fn to_cartesian_points(&self) -> Vec<Vector2<f64>> {
        self.points
            .iter()
            .map(|point| point.to_cartesian())
            .collect()
    }
    pub fn to_cartesian_points_ws(&self) -> Vec<[f64; 2]> {
        self.points
            .iter()
            .map(|point| point.to_cartesian_ws())
            .collect()
    }
}
// communications
pub enum LidarRequest {
    Stop,
    Reset,
    GetDeviceInfo,
    GetDeviceHealth,
}

#[derive(Debug)]
pub enum LidarResponse {
    DeviceInfo {
        model: u8,
        firmware_minor: u8,
        firmware_major: u8,
        hardware: u8,
        serial: [u8; 16],
    },
}

impl LidarRequest {
    pub async fn write(&self, port: &mut SerialStream) -> Result<(), tokio_serial::Error> {
        match self {
            LidarRequest::Stop => {
                port.write(&[0xa5, 0x25]).await?;
            }
            LidarRequest::Reset => {
                port.write(&[0xa5, 0x40]).await?;
            }
            LidarRequest::GetDeviceInfo => {
                port.write(&[0xa5, 0x50]).await?;
            }
            LidarRequest::GetDeviceHealth => {
                port.write(&[0xa5, 0x52]).await?;
            }
        }
        Ok(())
    }
}

impl LidarResponse {
    /// doesn't work yet. this might not be necessary
    pub async fn read(port: &mut SerialStream) -> Result<Self, tokio_serial::Error> {
        let mut response_descriptor = [0; 7];
        port.read_exact(&mut response_descriptor).await?;
        assert!(response_descriptor[0] == 0xa5);
        assert!(response_descriptor[1] == 0x5a);
        // 30bit length
        // 2bit send_mode
        // 1byte data_type

        let data_type = response_descriptor[6];
        let send_mode = response_descriptor[5] & 0b11;
        let length = ((response_descriptor[5] as u32) << 8) | response_descriptor[4] as u32;
        let mut buffer = [0; 20];
        port.read_exact(&mut buffer).await?;
        let mut serial = [0; 16];
        serial.copy_from_slice(&buffer[4..20]);
        Ok(LidarResponse::DeviceInfo {
            model: buffer[0],
            firmware_minor: buffer[1],
            firmware_major: buffer[2],
            hardware: buffer[3],
            serial,
        })
    }
}

fn varbitscale_decode(scaled: u32, scale_level: &mut u32) -> u32 {
    const VBS_SCALED_BASE: [u32; 5] = [3328, 1792, 1280, 512, 0];
    const VBS_SCALED_LVL: [u32; 5] = [4, 3, 2, 1, 0];
    const VBS_TARGET_BASE: [u32; 5] = [(0x1 << 14), (0x1 << 12), (0x1 << 11), (0x1 << 9), 0];

    for i in 0..VBS_SCALED_BASE.len() {
        let remain = scaled as i32 - VBS_SCALED_BASE[i] as i32;
        if remain >= 0 {
            *scale_level = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << *scale_level) as u32;
        }
    }
    0
}

#[test]
fn test_varbitscale_decode() {
    let mut scale_level = 0;
    assert_eq!(varbitscale_decode(1000, &mut scale_level), 1488);
    assert_eq!(scale_level, 1);
    scale_level = 0;
    assert_eq!(varbitscale_decode(2000, &mut scale_level), 5760);
    assert_eq!(scale_level, 3);
    scale_level = 0;
    assert_eq!(varbitscale_decode(1500, &mut scale_level), 2928);
    assert_eq!(scale_level, 2);
    scale_level = 0;
    assert_eq!(varbitscale_decode(15000, &mut scale_level), 203136);
    assert_eq!(scale_level, 4);
    scale_level = 0;
    assert_eq!(varbitscale_decode(0, &mut scale_level), 0);
    assert_eq!(scale_level, 0);
}
