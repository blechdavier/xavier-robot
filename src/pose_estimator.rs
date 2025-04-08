use std::{collections::BTreeMap, marker::PhantomData, time::Duration};

use crate::{geometry::{Transform2d, Twist2d}, odometry::WheelOdometry, utils::{Interpolate, TimeInterpolatableBuffer}};

const BUFFER_SIZE: Duration = Duration::from_secs(2);

pub struct PoseEstimator<T: WheelOdometry<U>, U> {
    odometry: T,
    odometry_buffer: TimeInterpolatableBuffer<Transform2d>,
    world_to_odom: BTreeMap<Duration, Transform2d>,
    _marker: PhantomData<U>,
}

impl<T: WheelOdometry<U>, U> PoseEstimator<T, U> {
    pub fn new(world_to_robot: Transform2d, odometry: T, time_since_program_start: Duration) -> Self {
        let mut res = PoseEstimator {
            odometry,
            odometry_buffer: TimeInterpolatableBuffer::new(BUFFER_SIZE),
            world_to_odom: BTreeMap::new(),
            _marker: PhantomData::<U>,
        };
        res.odometry_buffer.add_sample(time_since_program_start, res.odometry.get_pose().clone());
        res.reset_pose(world_to_robot, time_since_program_start);
        res
    }

    pub fn update_odometry(&mut self, gyro_angle_radians: f64, wheel_positions: &U, time_since_program_start: Duration) {
        self.odometry.update(gyro_angle_radians, wheel_positions);
        self.odometry_buffer
            .add_sample(time_since_program_start, self.odometry.get_pose().clone());
    }

    pub fn reset_pose(&mut self, world_to_robot: Transform2d, time_since_program_start: Duration) {
        let odom_to_robot = self.odometry.get_pose().clone();
        let robot_to_odom = -odom_to_robot;
        let t_world_to_odom = world_to_robot + robot_to_odom;
        if let Some((max_existing_timestamp, _)) = self.world_to_odom.last_key_value() {
            // this could crash... maybe I should handle this undefined behavior in a better way.
            assert!(*max_existing_timestamp < time_since_program_start, "tried to reset pose but there was already a more recent world_to_odom transform (maybe in the future somehow..?). Pose reset timestamp: {:?}. Max key in BTreeMap: {:?}", time_since_program_start, max_existing_timestamp);
        }
        self.world_to_odom.insert(time_since_program_start, t_world_to_odom);
    }

    pub fn add_vision_measurement(
        &mut self,
        world_to_vision_pose: Transform2d,
        timestamp: Duration,
        std_x: f64,
        std_y: f64,
        std_theta: f64,
    ) -> Result<(),()> {
        let world_to_estimated_pose = self.sample_at(timestamp)?;
        let t = 0.1; // TODO actually do this statistically :)
        let world_to_robot = Transform2d::interpolate(&world_to_estimated_pose, &world_to_vision_pose, t);
        let odom_to_robot = self.odometry_buffer.get_value(timestamp).ok_or(())?;
        let new_world_to_odom = world_to_robot + (-odom_to_robot);
        self.world_to_odom.insert(timestamp, new_world_to_odom);
        Ok(())
    }

    pub fn sample_at(&self, timestamp: Duration) -> Result<Transform2d, ()> {
        let odom_at_time = self.odometry_buffer.get_value(timestamp).ok_or(())?;
        let previous_world_to_odom = self.world_to_odom.range(Duration::ZERO..=timestamp).last().ok_or(())?; // TODO this might be guaranteed to have a value & I can unwrap it???
        Ok(previous_world_to_odom.1.clone() + odom_at_time)
    }
}
