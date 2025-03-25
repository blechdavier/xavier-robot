use std::{f64::consts::PI, sync::mpsc::{channel, Receiver}, time::{SystemTime, UNIX_EPOCH}};

use apriltag::{Detection, Detector, Family, Image, Pose, TagParams};
use apriltag_image::ImageExt;
use nokhwa::{
    pixel_format::LumaFormat,
    query,
    utils::{ApiBackend, RequestedFormat, RequestedFormatType, Resolution},
    CallbackCamera,
};
use nalgebra::{Matrix3, Rotation3, Vector3};
use crate::geometry::{Transform2d, Transform3d};

#[derive(Debug)]
pub struct TagMeasurement {
    pub id: usize,
    pub corners: [[f64; 2]; 4],
    pub robot_to_tag: Transform2d
}

#[derive(Debug)]
pub struct AprilTagCameraUpdate {
    pub timestamp: f64,
    pub tags: Vec<TagMeasurement>
}

pub struct AprilTagCamera {
    cam: CallbackCamera,
    pub rx: Receiver<AprilTagCameraUpdate>
}

// 1.334m, 0.0
const CAMERA_TO_WORLD_SPACE: Transform3d = Transform3d::new(Rotation3::from_matrix_unchecked(Matrix3::new(0.0,0.0,1.0,-1.0,0.0,0.0,0.0,-1.0,0.0)), Vector3::new(0.0,0.0,0.0));
const WORLD_TO_TAG_1: Transform3d = Transform3d::new(Rotation3::from_matrix_unchecked(Matrix3::new(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0)), Vector3::new(0.0, 0.0, 1.334));

impl AprilTagCamera {
    pub fn new(name: &str, res: Resolution, fps: u32, calibration: TagParams) -> Self {
        let cameras = query(ApiBackend::Auto).unwrap();
        dbg!(&cameras);
        let camera_info = cameras.iter().find(|x| x.human_name() == name).unwrap();
        let format = RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate);

        let (tx, rx) = channel::<AprilTagCameraUpdate>();
        let mut cam = CallbackCamera::new(camera_info.index().clone(), format, move |buffer| {
            let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
            let image_buffer = buffer.decode_image::<LumaFormat>().unwrap(); // 4.2ms
            let img = Image::from_image_buffer(&image_buffer); // 1.2ms
            let mut detector = Detector::builder().add_family_bits(Family::tag_36h11(), 1).build().unwrap(); // 100us
            let detections = detector.detect(&img);
            let mut tags: Vec<TagMeasurement> = Vec::with_capacity(detections.len());
            for detection in &detections {
                if detection.id() != 1 {continue;}
                if let Some(pose) = detection.estimate_tag_pose(&calibration) {
                    let camera_to_tag_opencv = Transform3d::from_slices(pose.rotation().data().try_into().unwrap(), pose.translation().data().try_into().unwrap());
                    dbg!(camera_to_tag_opencv);
                    // let CAMERA_TO_WORLD_SPACE = Transform3d::new(Rotation3::from_euler_angles(-PI/2.0, -PI/2.0, 0.0), Vector3::zeros());
                    // let camera_to_tag = CAMERA_TO_WORLD_SPACE.clone() + camera_to_tag_opencv.clone();
                    // let tag_to_camera = camera_to_tag.inverse();
                    // assert_eq!(camera_to_tag, tag_to_camera.inverse());
                    // let world_to_camera = WORLD_TO_TAG_1 + tag_to_camera;
                    // tags.push(TagMeasurement { id: detection.id(), corners: detection.corners(), robot_to_tag: None });
                }
            }
            tx.send(AprilTagCameraUpdate { timestamp, tags: tags }).unwrap();
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

#[test]
fn test_camera_euler_angles() {
    let identity_rotation = Rotation3::from_matrix_unchecked(Matrix3::identity());
    let unit_x = Transform3d::new(identity_rotation, Vector3::new(1.0, 0.0, 0.0));
    let unit_y = Transform3d::new(identity_rotation, Vector3::new(0.0, 1.0, 0.0));
    let unit_z = Transform3d::new(identity_rotation, Vector3::new(0.0, 0.0, 1.0));

    assert_eq!((CAMERA_TO_WORLD_SPACE + unit_x).translation, Vector3::new(0.0,-1.0,0.0));
    assert_eq!((CAMERA_TO_WORLD_SPACE + unit_y).translation, Vector3::new(0.0,0.0,-1.0));
    assert_eq!((CAMERA_TO_WORLD_SPACE + unit_z).translation, Vector3::new(1.0,0.0,0.0));
}
#[test]
fn test_valid_rotation() {
    let identity_transform = Transform3d::new(Rotation3::identity(), Vector3::new(0.0,0.0,0.0));
    assert_eq!(CAMERA_TO_WORLD_SPACE + identity_transform.clone(), CAMERA_TO_WORLD_SPACE);
    assert_eq!(identity_transform + CAMERA_TO_WORLD_SPACE, CAMERA_TO_WORLD_SPACE);
}