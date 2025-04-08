use std::{f64::consts::PI, sync::mpsc::{channel, Receiver}, time::{Duration, Instant}};

use apriltag::{Detection, Detector, Family, Image, Pose, TagParams};
use apriltag_image::ImageExt;
use nokhwa::{
    pixel_format::LumaFormat,
    query,
    utils::{ApiBackend, RequestedFormat, RequestedFormatType, Resolution},
    CallbackCamera,
};
use nalgebra::{Matrix3, Rotation3, Vector3};
use crate::geometry::Transform3d;

#[derive(Debug)]
pub struct TagMeasurement {
    pub id: usize,
    pub corners: [[f64; 2]; 4],
    pub world_to_robot: Transform3d
}

#[derive(Debug)]
pub struct AprilTagCameraUpdate {
    pub timestamp: Duration,
    pub tags: Vec<TagMeasurement>
}

pub struct AprilTagCamera {
    cam: CallbackCamera,
    pub rx: Receiver<AprilTagCameraUpdate>
}

// 1.334m, 0.0
const WORLD_TO_TAG: [Transform3d; 2] = [
    Transform3d::new(Rotation3::from_matrix_unchecked(Matrix3::new(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0)), Vector3::new(0.0, 0.0, 1.334)),
    Transform3d::new(Rotation3::from_matrix_unchecked(Matrix3::new(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0)), Vector3::new(0.0, -0.283, 1.466))
];

impl AprilTagCamera {
    pub fn new(name: &str, res: Resolution, fps: u32, calibration: TagParams, robot_to_camera: Transform3d, program_start: Instant) -> Self {
        let cameras = query(ApiBackend::Auto).unwrap();
        dbg!(&cameras);
        let camera_info = cameras.iter().find(|x| x.human_name() == name).unwrap();
        let format = RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate);

        let (tx, rx) = channel::<AprilTagCameraUpdate>();
        let mut cam = CallbackCamera::new(camera_info.index().clone(), format, move |buffer| {
            let timestamp = program_start.elapsed();
            let image_buffer = buffer.decode_image::<LumaFormat>().unwrap();
            let img = Image::from_image_buffer(&image_buffer);
            let mut detector = Detector::builder().add_family_bits(Family::tag_36h11(), 1).build().unwrap(); // TODO can I move this outside the thread?
            let detections = detector.detect(&img);
            let mut tags: Vec<TagMeasurement> = Vec::with_capacity(detections.len());
            for detection in &detections {
                if let Some(pose) = detection.estimate_tag_pose(&calibration) {
                    let world_to_tag = WORLD_TO_TAG[detection.id()-1].clone();
                    let camera_to_tag: Transform3d = pose.into();
                    let tag_to_camera = camera_to_tag.inverse();
                    let world_to_camera = world_to_tag + tag_to_camera;
                    let world_to_robot = world_to_camera + robot_to_camera.inverse();
                    // dbg!(&world_to_robot);
                    tags.push(TagMeasurement { id: detection.id(), corners: detection.corners(), world_to_robot });
                }
            }
            tx.send(AprilTagCameraUpdate { timestamp, tags }).unwrap();
            // ~2cm per meter stddev
        }).unwrap();
        cam.set_resolution(res).unwrap();
        cam.set_frame_rate(fps).unwrap();
        cam.open_stream().unwrap();
        Self {
            cam,
            rx
        }
    }
}
