use std::{
    marker::PhantomData,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::{geometry::Transform2d, odometry::WheelOdometry, utils::TimeInterpolatableBuffer};

const BUFFER_SIZE: f64 = 1.5; // seconds

pub struct PoseEstimator<T: WheelOdometry<U>, U> {
    pose_estimate: Transform2d,
    odometry: T,
    odometry_buffer: TimeInterpolatableBuffer<Transform2d>,
    vision_updates: Vec<()>, // TODO
    _marker: PhantomData<U>,
}

impl<T: WheelOdometry<U>, U> PoseEstimator<T, U> {
    pub fn new(initial_pose: Transform2d, odometry: T) -> Self {
        PoseEstimator {
            pose_estimate: initial_pose,
            odometry,
            odometry_buffer: TimeInterpolatableBuffer::new(BUFFER_SIZE),
            vision_updates: Vec::new(),
            _marker: PhantomData::<U>,
        }
    }

    pub fn update(&mut self, gyro_angle_radians: f64, wheel_positions: &U) {
        self.odometry.update(gyro_angle_radians, wheel_positions);
        let time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        self.odometry_buffer
            .add_sample(time, self.odometry.get_pose().clone());

        if self.vision_updates.is_empty() {
            self.pose_estimate = self.odometry.get_pose().clone();
        }
        // TODO update pose estimate
    }

    pub fn reset_pose(&mut self, pose: Transform2d, gyro_angle_radians: f64, wheel_positions: U) {
        self.pose_estimate = pose.clone();
        self.odometry
            .reset_pose(gyro_angle_radians, wheel_positions, pose);
        self.vision_updates.clear();
        self.odometry_buffer.clear();
    }

    pub fn add_vision_measurement(
        &mut self,
        estimated_pose: Transform2d,
        timestamp: f64,
        std_x: f64,
        std_y: f64,
        std_theta: f64,
    ) {
        todo!()
    }

    pub fn get_estimated_pose(&self) -> &Transform2d {
        &self.pose_estimate
    }
}
