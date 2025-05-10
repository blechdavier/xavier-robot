use std::f64::consts::PI;

use crate::geometry::{Transform2d, Twist2d};

pub struct Path {
    pub waypoints: Vec<Transform2d>,
}

impl Path {
    pub fn pure_pursuit(&self, current_position: &Transform2d) -> (Twist2d, Transform2d) {
        let curr_t = self.get_path_progress(current_position);
        let max_t = self.waypoints.len() as f64 - 1.0;
        let target_t = (curr_t + 0.1).min(max_t); // TODO look ahead a fixed distance instead of using path progress
        let target_pose = self.get_path_at_time(target_t);
        let err = -current_position.clone() + target_pose.clone();
        dbg!(&err);
        let cmd_vel = if max_t - curr_t < 0.02 { // TODO consider adding a debouncer here
            // we are already at the target position, we can now rotate to the desired angle.
            Twist2d::new(0.2 * err.x_meters, 0.2 * err.y_meters, (10.0 * ((err.theta_radians + PI).rem_euclid(2.0*PI) - PI)).max(-1.0).min(1.0))
        } else {
            // we have to drive to the target position
            let angle_err = (err.y_meters).atan2(err.x_meters);
            dbg!(angle_err);
            let forward_vel = (0.4 - 10.0 * angle_err * angle_err).max(0.0).min(0.2);
            Twist2d::new(forward_vel, 0.0, (10.0 * angle_err).min(1.0).max(-1.0))
        };
        (cmd_vel, target_pose)
    }

    pub fn get_path_at_time(&self, t: f64) -> Transform2d {
        assert!(t >= 0.0);
        let max_t = self.waypoints.len() as f64 - 1.0;
        assert!(t <= max_t);
        if t == 0.0 {
            return self.waypoints.first().unwrap().clone();
        } else if t == max_t {
            return self.waypoints.last().unwrap().clone();
        }

        let waypoint_1 = &self.waypoints[t.floor() as usize];
        let waypoint_2 = &self.waypoints[t.floor() as usize + 1];
        let x = waypoint_1.x_meters + (waypoint_2.x_meters - waypoint_1.x_meters) * t.fract();
        let y = waypoint_1.y_meters + (waypoint_2.y_meters - waypoint_1.y_meters) * t.fract();
        let theta = waypoint_1.theta_radians
            + (waypoint_2.theta_radians - waypoint_1.theta_radians) * t.fract();
        Transform2d::new(x, y, theta)
    }
    /// 0.0 is the starting waypoint position, 1.0 is the second waypoint, 2.0 is the third, etc. 1.5 is halfway between the second and third waypoints.
    pub fn get_path_progress(&self, tf: &Transform2d) -> f64 {
        assert!(self.waypoints.len() >= 2);
        let mut min_dist = f64::INFINITY;
        let mut res = -1.0; // TODO use result instead of sentinel value but this should never happen because of the assert :/
        for i in 0..(self.waypoints.len() - 1) {
            let (t, dist) = self.project_tf_onto_line_segment(&tf, i);
            if dist < min_dist {
                min_dist = dist;
                res = i as f64 + t;
            }
        }
        if res == -1.0 {
            dbg!(&self.waypoints);
        }
        assert_ne!(res, -1.0);
        res
    }
    /// (t, dist) t=0.0 is at the start of the segment, t=1.0 is at the end, and t=0.5 is halfway in between. dist is the euclidean distance to the closest point on the line segment.
    fn project_tf_onto_line_segment(&self, tf: &Transform2d, index: usize) -> (f64, f64) {
        let waypoint_1 = &self.waypoints[index];
        let waypoint_2 = &self.waypoints[index + 1];

        let dx = waypoint_2.x_meters - waypoint_1.x_meters;
        let dy = waypoint_2.y_meters - waypoint_1.y_meters;

        let t;
        if dx == 0.0 && dy == 0.0 {
            t = 0.0;
        } else {
            let dx1 = tf.x_meters - waypoint_1.x_meters;
            let dy1 = tf.y_meters - waypoint_1.y_meters;
            t = ((dx * dx1 + dy * dy1) / (dx * dx + dy * dy))
                .max(0.0)
                .min(1.0);
        }

        let px = waypoint_1.x_meters + dx * t;
        let py = waypoint_1.y_meters + dy * t;
        (t, (tf.x_meters - px).hypot(tf.y_meters - py))
    }
}

#[test]
fn test_projection() {
    let mut path = Path {
        waypoints: Vec::new(),
    };
    path.waypoints.push(Transform2d::new(1.0, 0.0, 1.0));
    path.waypoints.push(Transform2d::new(5.0, 0.0, 4.0));
    path.waypoints.push(Transform2d::new(6.0, 0.0, 4.0));
    assert_eq!(
        path.get_path_progress(&Transform2d::new(-100.0, -1.0, 2.0)),
        0.0
    );
    assert_eq!(
        path.get_path_progress(&Transform2d::new(2.0, 1.0, 2.0)),
        0.25
    );
    assert_eq!(
        path.get_path_progress(&Transform2d::new(5.62, 1.0, 2.0)),
        1.62
    );
    assert_eq!(
        path.get_path_progress(&Transform2d::new(123123.123, 1.0, 2.0)),
        2.0
    );
}