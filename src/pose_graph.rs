use std::{f64::consts::PI, time::Instant};

use nalgebra::{DMatrix, DVector, Matrix3, Vector3, Vector2};
use nalgebra_sparse::{factorization::CscCholesky, CooMatrix, CscMatrix};

use serde::Serialize;

use crate::geometry::Transform2d;
use crate::icp::icp_least_squares;
use crate::lidar::LidarScan;

#[derive(Debug)]
pub struct PoseGraphBackend {
    pub nodes: DVector<f64>,
    edges: Vec<PoseGraphEdge>,
    dirty: bool
}

impl PoseGraphBackend {
    pub fn new() -> Self {
        Self {
            nodes: DVector::zeros(3), // zeros for xyz of first node
            edges: Vec::new(),
            dirty: false
        }
    }
    pub fn add_node_with_odometry(&mut self, prev_node_to_new: Transform2d) {
        self.edges.push(PoseGraphEdge {
            i: self.nodes.len() / 3 - 1,
            j: self.nodes.len() / 3,
            i_to_j: prev_node_to_new.clone()
        });
        let world_to_prev = Transform2d::new(self.nodes[self.nodes.len() - 3], self.nodes[self.nodes.len() - 2], self.nodes[self.nodes.len() - 1]);
        let world_to_new = world_to_prev + prev_node_to_new;
        // TODO there has to be a better way than this :/
        self.nodes = self.nodes.push(world_to_new.x_meters);
        self.nodes = self.nodes.push(world_to_new.y_meters);
        self.nodes = self.nodes.push(world_to_new.theta_radians);
    }
    pub fn add_loop_closure(&mut self, i: usize, j: usize, i_to_j: Transform2d) {
        self.edges.push(PoseGraphEdge { i, j, i_to_j });
        self.dirty = true;
    }
    pub fn optimize(&mut self, max_iterations: usize) {
        for _ in 0..max_iterations {
            let mut b: DVector<f64> = DVector::zeros(self.nodes.nrows());
            let mut h: DMatrix<f64> = DMatrix::zeros(self.nodes.nrows(), self.nodes.nrows()); // TODO bench constructing using a sparse representation... would be better memory but could take more compute
            for edge in &self.edges {
                let x_i = self.nodes.get(edge.i * 3).unwrap();
                let y_i = self.nodes.get(edge.i * 3 + 1).unwrap();
                let theta_i = self.nodes.get(edge.i * 3 + 2).unwrap();
                let x_j = self.nodes.get(edge.j * 3).unwrap();
                let y_j = self.nodes.get(edge.j * 3 + 1).unwrap();
                let theta_j = self.nodes.get(edge.j * 3 + 2).unwrap();
                
                let x_ij = edge.i_to_j.x_meters;
                let y_ij = edge.i_to_j.y_meters;
                let theta_ij = edge.i_to_j.theta_radians;
                
                // compute the error function TODO add test coverage
                let unrotated_x = theta_i.cos() * (x_j-x_i) + theta_i.sin() * (y_j-y_i) - x_ij;
                let unrotated_y = -theta_i.sin() * (x_j-x_i) + theta_i.cos() * (y_j-y_i) - y_ij;
                let e_ij = Vector3::new(
                    theta_ij.cos() * unrotated_x + theta_ij.sin() * unrotated_y,
                    -theta_ij.sin() * unrotated_x + theta_ij.cos() * unrotated_y,
                    theta_j - theta_i - theta_ij
                );

                // compute the jacobians of the error function TODO add test coverage
                let unrotated_x = theta_i.cos() * (y_j - y_i) - theta_i.sin() * (x_j - x_i);
                let unrotated_y = -(theta_i).cos() * (x_j - x_i) - (theta_i).sin() * (y_j - y_i);
                let jacobian_e_wrt_i = Matrix3::from_row_slice(&[
                    -(theta_i + theta_ij).cos(), -(theta_i + theta_ij).sin(), theta_ij.cos() * unrotated_x + theta_ij.sin() * unrotated_y,
                    (theta_i + theta_ij).sin(), -(theta_i + theta_ij).cos(), -theta_ij.sin() * unrotated_x + theta_ij.cos() * unrotated_y,
                    0.0, 0.0, -1.0
                ]);
                let jacobian_e_wrt_j = Matrix3::from_row_slice(&[
                    (theta_i + theta_ij).cos(), (theta_i + theta_ij).sin(), 0.0,
                    -(theta_i + theta_ij).sin(), (theta_i + theta_ij).cos(), 0.0,
                    0.0, 0.0, 1.0
                ]);

                // compute the contributions of this constraint to the linear system
                let mut h_ii = h.view_mut((edge.i*3, edge.i*3), (3, 3));
                h_ii += jacobian_e_wrt_i.transpose() * jacobian_e_wrt_i;
                let mut h_ij = h.view_mut((edge.i*3, edge.j*3), (3, 3));
                h_ij += jacobian_e_wrt_i.transpose() * jacobian_e_wrt_j;
                let mut h_ji = h.view_mut((edge.j*3, edge.i*3), (3, 3));
                h_ji += jacobian_e_wrt_j.transpose() * jacobian_e_wrt_i;
                let mut h_jj = h.view_mut((edge.j*3, edge.j*3), (3, 3));
                h_jj += jacobian_e_wrt_j.transpose() * jacobian_e_wrt_j;
                
                // compute the coefficient vector
                let mut b_i = b.rows_mut(edge.i * 3, 3);
                b_i += jacobian_e_wrt_i.transpose() * e_ij;
                let mut b_j = b.rows_mut(edge.j * 3, 3);
                b_j += jacobian_e_wrt_j.transpose() * e_ij;
            }
            let mut h_11 = h.view_mut((0, 0), (3, 3));
            h_11 += Matrix3::identity();
            // assert_eq!(&h, &h.transpose());
            let h_csc: CscMatrix<f64> = (&h).into();
            let decomp = CscCholesky::factor(&h_csc).unwrap();
            let delta_x = decomp.solve(&-b);
            self.nodes += &delta_x;
            if delta_x.norm() < 1e-10 {
                break;
            }
        }
        self.dirty = false;
    }
}

#[derive(Debug)]
struct PoseGraphEdge {
    i: usize,
    j: usize,
    i_to_j: Transform2d
}

#[test]
fn test_optimize_pose_graph() {
    let mut pose_graph = PoseGraphBackend::new();
    pose_graph.add_node_with_odometry(Transform2d::new(2.0, 0.0, 0.0));
    for _ in 0..1500 {
        pose_graph.add_node_with_odometry(Transform2d::new(2.0, 0.0, 0.2));
    }
    let ground_truth = pose_graph.nodes.clone();
    for element in pose_graph.nodes.iter_mut().skip(3) {
        *element += 1.0;
    }
    pose_graph.optimize(10);
    assert!(pose_graph.nodes.relative_eq(&ground_truth, 1e-9, 1e-9));
}

pub struct LidarPoseGraph {
    pub backend: PoseGraphBackend,
    pub node_scans: Vec<Vec<Vector2<f64>>>
}

impl LidarPoseGraph {
    pub fn new() -> Self {
        Self { backend: PoseGraphBackend::new(), node_scans: Vec::new() }
    }
    pub fn add_scan(&mut self, odom_prev_node_to_new: Transform2d, new_scan: Vec<Vector2<f64>>) {
        if let Some(last_scan) = self.node_scans.last() {
            let icp_result = icp_least_squares(&new_scan, &last_scan, Vector3::new(odom_prev_node_to_new.x_meters, odom_prev_node_to_new.y_meters, odom_prev_node_to_new.theta_radians), 50);
            dbg!(&icp_result, odom_prev_node_to_new);
            self.backend.add_node_with_odometry(Transform2d::new(icp_result[0], icp_result[1], icp_result[2]));
        } else {
            // we can trust odometry for the first node because there's no scan to match to
            self.backend.add_node_with_odometry(odom_prev_node_to_new);
        }
        self.node_scans.push(new_scan);
    }
    // pub fn add_loop_closure(&mut self, i: usize, j: usize, i_to_j: Transform2d) {
    //     self.backend.add_loop_closure
    //     self.dirty = true;
    // }
}