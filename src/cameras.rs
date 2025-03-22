use std::{sync::mpsc::{channel, Receiver}, time::{SystemTime, UNIX_EPOCH}};

use apriltag::{Detection, Detector, Family, Image};
use apriltag_image::ImageExt;
use nokhwa::{
    pixel_format::LumaFormat,
    query,
    utils::{ApiBackend, RequestedFormat, RequestedFormatType, Resolution},
    CallbackCamera,
};

#[derive(Debug)]
pub struct TagMeasurement {
    pub id: usize,
    pub corners: [[f64; 2]; 4]
}

impl From<&Detection> for TagMeasurement {
    fn from(value: &Detection) -> Self {
        Self {
            id: value.id(),
            corners: value.corners()
        }
    }
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

impl AprilTagCamera {
    pub fn new(name: &str, res: Resolution, fps: u32) -> Self {
        let cameras = query(ApiBackend::Auto).unwrap();
        let camera_info = cameras.iter().find(|x| x.human_name() == name).unwrap();
        let format = RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate);

        let (tx, rx) = channel::<AprilTagCameraUpdate>();
        let mut cam = CallbackCamera::new(camera_info.index().clone(), format, move |buffer| {
            let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
            let image_buffer = buffer.decode_image::<LumaFormat>().unwrap(); // 4.2ms
            let img = Image::from_image_buffer(&image_buffer); // 1.2ms
            let mut detector = Detector::builder().add_family_bits(Family::tag_36h11(), 1).build().unwrap(); // 100us
            let tags = detector.detect(&img).iter().map(|x| x.into()).collect::<Vec<TagMeasurement>>();
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
