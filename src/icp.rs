use lstsq::lstsq; // TODO just solve the system myself :/
use nalgebra::{Matrix2, Matrix2x3, Matrix3, Rotation2, Vector2, Vector3};
use std::f64::consts::PI;

pub fn icp_least_squares(
    p: &[Vector2<f64>],
    q: &[Vector2<f64>],
    initial_guess: Vector3<f64>,
    iterations: usize,
) -> Vector3<f64> {
    let mut x = initial_guess;
    let mut p_copy = p.to_vec();
    for _ in 0..iterations {
        let correspondences = get_correspondence_indices(&p_copy, q);
        let (h, g) = prepare_system(&x, p, q, &correspondences);
        let epsilon = 1e-6;
        let dx = lstsq(&h, &-g, epsilon).unwrap().solution;
        if dx.norm() < 1e-9 {
            break;
        }
        x += dx;
        x[2] = x[2].rem_euclid(2.0 * PI);
        let rotation = Rotation2::new(x[2]);
        let translation = Vector2::new(x[0], x[1]);
        p_copy = p
            .iter()
            .map(|point| (rotation * point) + translation)
            .collect();
    }

    x
}

fn get_correspondence_indices(p: &[Vector2<f64>], q: &[Vector2<f64>]) -> Vec<usize> {
    let mut correspondences = Vec::with_capacity(p.len());
    // closest point
    for i in 0..p.len() {
        let mut min_dist = f64::MAX;
        let mut min_index = 0;
        for j in 0..q.len() {
            let dist = (p[i] - q[j]).norm();
            if dist < min_dist {
                min_dist = dist;
                min_index = j;
            }
        }
        correspondences.push(min_index);
    }
    correspondences
}

fn error(x: &Vector3<f64>, p_point: &Vector2<f64>, q_point: &Vector2<f64>) -> Vector2<f64> {
    let rotation = Rotation2::new(x[2]);
    let translation = Vector2::new(x[0], x[1]);
    let prediction = (rotation * p_point) + translation;
    prediction - q_point
}

fn prepare_system(
    x: &Vector3<f64>,
    p: &[Vector2<f64>],
    q: &[Vector2<f64>],
    correspondences: &Vec<usize>,
) -> (Matrix3<f64>, Vector3<f64>) {
    let mut h = Matrix3::zeros();
    let mut g = Vector3::zeros();
    for (i, j) in correspondences.iter().enumerate() {
        let p_point = p[i];
        let q_point = q[*j];
        let e = error(x, &p_point, &q_point);
        let jacobian = jacobian(&p_point);
        let transposed_jacobian = jacobian.transpose();
        h += transposed_jacobian * jacobian;
        g += transposed_jacobian * e;
    }
    (h, g)
}

/// Returns the derivative of a rotation matrix with a given angle.
fn d_r(theta: f64) -> Matrix2<f64> {
    let mut d_r = Matrix2::zeros();
    d_r[(0, 0)] = -theta.sin();
    d_r[(0, 1)] = -theta.cos();
    d_r[(1, 0)] = theta.cos();
    d_r[(1, 1)] = -theta.sin();
    d_r
}

fn jacobian(p: &Vector2<f64>) -> Matrix2x3<f64> {
    let mut j = Matrix2x3::zeros();
    j[(0, 0)] = 1.0;
    j[(1, 1)] = 1.0;
    let dr_0 = d_r(0.0); // Not sure why this doesn't use theta but the python code doesn't either
    let dot = dr_0 * p;
    j[(0, 2)] = dot[0];
    j[(1, 2)] = dot[1];
    j
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_lstsq() {
        let h = Matrix3::new(
            365.0,
            0.0,
            -8044.33832,
            0.0,
            365.0,
            168755.592,
            -8044.33832,
            168755.592,
            1209088170.0,
        );

        let g = Vector3::new(12299.9634f64, 21937.3152, 22231887.2);
        let solution = lstsq(&h, &-g, 1e-6).unwrap().solution;
        let expected_solution = Vector3::new(-33.9394150, -55.0489031, -0.0109298043);
        let residuals = expected_solution - solution;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 0.01);
    }

    #[test]
    fn test_d_r() {
        let theta = PI / 2.0;
        let d_r = d_r(theta);
        let expected_d_r = Matrix2::new(-1.0, 0.0, 0.0, -1.0);
        let residuals = expected_d_r - d_r;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 1e-9);
    }

    #[test]
    fn test_jacobian() {
        let p = Vector2::new(10.0, 1111.1);
        let j = jacobian(&p);
        let expected_j = Matrix2x3::new(1.0, 0.0, -1111.1, 0.0, 1.0, 10.0);
        let residuals = expected_j - j;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 1e-9);
    }

    #[test]
    fn test_prepare_system() {
        let x = Vector3::new(-2.12131717, -4.94974141, -0.78539833);
        let p = vec![
            Vector2::new(-2.0, 5.0),
            Vector2::new(-1.36069423, 5.77490779),
            Vector2::new(-0.82379037, 6.6522175),
            Vector2::new(-0.30188094, 7.54452163),
            Vector2::new(0.31405082, 8.34280343),
            Vector2::new(1.11235019, 8.95871762),
            Vector2::new(2.12289639, 9.36238498),
            Vector2::new(3.29700515, 9.60248979),
            Vector2::new(4.51307853, 9.80062997),
            Vector2::new(5.60815375, 10.11976832),
            Vector2::new(6.42719153, 10.7149441),
            Vector2::new(6.87573776, 11.68061143),
            Vector2::new(6.9594652, 13.01109755),
            Vector2::new(6.79689487, 14.58788144),
            Vector2::new(6.59872784, 16.20026204),
            Vector2::new(6.61680328, 17.59640015),
            Vector2::new(7.07504634, 18.55237066),
            Vector2::new(8.10112206, 18.9405085),
            Vector2::new(9.67883967, 18.77700445),
            Vector2::new(11.63696034, 18.23309735),
            Vector2::new(13.68085969, 17.60341156),
            Vector2::new(15.46180552, 17.23667929),
            Vector2::new(16.66758855, 17.44510982),
            Vector2::new(17.11103155, 18.41588039),
            Vector2::new(16.79175162, 20.14937387),
            Vector2::new(15.91215285, 22.44318621),
            Vector2::new(14.83984091, 24.92971171),
            Vector2::new(14.02273143, 27.16103475),
            Vector2::new(13.87638487, 28.72159487),
            Vector2::new(14.67188698, 29.34030633),
        ];
        let q = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.09588511),
            Vector2::new(2.0, 0.33658839),
            Vector2::new(3.0, 0.59849699),
            Vector2::new(4.0, 0.72743794),
            Vector2::new(5.0, 0.59847214),
            Vector2::new(6.0, 0.16934401),
            Vector2::new(7.0, -0.49109652),
            Vector2::new(8.0, -1.21088399),
            Vector2::new(9.0, -1.75955421),
            Vector2::new(10.0, -1.91784855),
            Vector2::new(11.0, -1.55218872),
            Vector2::new(12.0, -0.6705972),
            Vector2::new(13.0, 0.55931197),
            Vector2::new(14.0, 1.83956248),
            Vector2::new(15.0, 2.81399993),
            Vector2::new(16.0, 3.16594639),
            Vector2::new(17.0, 2.71485618),
            Vector2::new(18.0, 1.48362655),
            Vector2::new(19.0, -0.28557426),
            Vector2::new(20.0, -2.17608444),
            Vector2::new(21.0, -3.69472219),
            Vector2::new(22.0, -4.39995691),
            Vector2::new(23.0, -4.02708),
            Vector2::new(24.0, -2.57555001),
            Vector2::new(25.0, -0.33160949),
            Vector2::new(26.0, 2.18486859),
            Vector2::new(27.0, 4.3404359),
            Vector2::new(28.0, 5.54740119),
            Vector2::new(29.0, 5.42239132),
        ];
        let correspondences = vec![
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
            24, 25, 26, 27, 28, 29,
        ];
        let (h, g) = prepare_system(&x, &p, &q, &correspondences);
        let expected_h = Matrix3::new(
            30.0,
            0.0,
            -462.89890748,
            0.0,
            30.0,
            242.28399215,
            -462.89890748,
            242.28399215,
            11533.37791438,
        );
        let expected_g = Vector3::new(0.00012049, 0.00010064, -0.00132618);
        let residuals = expected_h - h;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 1e-3);
        let residuals = expected_g - g;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 1e-3);
    }

    #[test]
    fn test_generate_data() {
        /*
                q =  [[ 0.          1.          2.          3.          4.          5.
           6.          7.          8.          9.         10.         11.
          12.         13.         14.         15.         16.         17.
          18.         19.         20.         21.         22.         23.
          24.         25.         26.         27.         28.         29.        ]
         [ 0.          0.09588511  0.33658839  0.59849699  0.72743794  0.59847214
           0.16934401 -0.49109652 -1.21088399 -1.75955421 -1.91784855 -1.55218872
          -0.6705972   0.55931197  1.83956248  2.81399993  3.16594639  2.71485618
           1.48362655 -0.28557426 -2.17608444 -3.69472219 -4.39995691 -4.02708
          -2.57555001 -0.33160949  2.18486859  4.3404359   5.54740119  5.42239132]]
        p =  [[-2.         -1.36069423 -0.82379037 -0.30188094  0.31405082  1.11235019
           2.12289639  3.29700515  4.51307853  5.60815375  6.42719153  6.87573776
           6.9594652   6.79689487  6.59872784  6.61680328  7.07504634  8.10112206
           9.67883967 11.63696034 13.68085969 15.46180552 16.66758855 17.11103155
          16.79175162 15.91215285 14.83984091 14.02273143 13.87638487 14.67188698]
         [ 5.          5.77490779  6.6522175   7.54452163  8.34280343  8.95871762
           9.36238498  9.60248979  9.80062997 10.11976832 10.7149441  11.68061143
          13.01109755 14.58788144 16.20026204 17.59640015 18.55237066 18.9405085
          18.77700445 18.23309735 17.60341156 17.23667929 17.44510982 18.41588039
          20.14937387 22.44318621 24.92971171 27.16103475 28.72159487 29.34030633]]
                 */
        let expected_q = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.09588511),
            Vector2::new(2.0, 0.33658839),
            Vector2::new(3.0, 0.59849699),
            Vector2::new(4.0, 0.72743794),
            Vector2::new(5.0, 0.59847214),
            Vector2::new(6.0, 0.16934401),
            Vector2::new(7.0, -0.49109652),
            Vector2::new(8.0, -1.21088399),
            Vector2::new(9.0, -1.75955421),
            Vector2::new(10.0, -1.91784855),
            Vector2::new(11.0, -1.55218872),
            Vector2::new(12.0, -0.6705972),
            Vector2::new(13.0, 0.55931197),
            Vector2::new(14.0, 1.83956248),
            Vector2::new(15.0, 2.81399993),
            Vector2::new(16.0, 3.16594639),
            Vector2::new(17.0, 2.71485618),
            Vector2::new(18.0, 1.48362655),
            Vector2::new(19.0, -0.28557426),
            Vector2::new(20.0, -2.17608444),
            Vector2::new(21.0, -3.69472219),
            Vector2::new(22.0, -4.39995691),
            Vector2::new(23.0, -4.02708),
            Vector2::new(24.0, -2.57555001),
            Vector2::new(25.0, -0.33160949),
            Vector2::new(26.0, 2.18486859),
            Vector2::new(27.0, 4.3404359),
            Vector2::new(28.0, 5.54740119),
            Vector2::new(29.0, 5.42239132),
        ];
        let expected_p = vec![
            Vector2::new(-2.0, 5.0),
            Vector2::new(-1.36069423, 5.77490779),
            Vector2::new(-0.82379037, 6.6522175),
            Vector2::new(-0.30188094, 7.54452163),
            Vector2::new(0.31405082, 8.34280343),
            Vector2::new(1.11235019, 8.95871762),
            Vector2::new(2.12289639, 9.36238498),
            Vector2::new(3.29700515, 9.60248979),
            Vector2::new(4.51307853, 9.80062997),
            Vector2::new(5.60815375, 10.11976832),
            Vector2::new(6.42719153, 10.7149441),
            Vector2::new(6.87573776, 11.68061143),
            Vector2::new(6.9594652, 13.01109755),
            Vector2::new(6.79689487, 14.58788144),
            Vector2::new(6.59872784, 16.20026204),
            Vector2::new(6.61680328, 17.59640015),
            Vector2::new(7.07504634, 18.55237066),
            Vector2::new(8.10112206, 18.9405085),
            Vector2::new(9.67883967, 18.77700445),
            Vector2::new(11.63696034, 18.23309735),
            Vector2::new(13.68085969, 17.60341156),
            Vector2::new(15.46180552, 17.23667929),
            Vector2::new(16.66758855, 17.44510982),
            Vector2::new(17.11103155, 18.41588039),
            Vector2::new(16.79175162, 20.14937387),
            Vector2::new(15.91215285, 22.44318621),
            Vector2::new(14.83984091, 24.92971171),
            Vector2::new(14.02273143, 27.16103475),
            Vector2::new(13.87638487, 28.72159487),
            Vector2::new(14.67188698, 29.34030633),
        ];
        let angle = PI / 4.0;
        let rotation_true = Rotation2::new(angle);
        let translation_true = Vector2::new(-2.0, 5.0);
        println!("Ground Truth Rotation Matrix:\n{}", rotation_true);
        println!("Ground Truth Translation Vector:\n{}", translation_true);

        // define measurements
        const NUM_MEASUREMENTS: usize = 30;
        let mut q = Vec::with_capacity(NUM_MEASUREMENTS);
        for i in 0..NUM_MEASUREMENTS {
            let x = i as f64;
            let y = 0.2 * x * (0.5 * x).sin();
            // let y = 0.0;
            q.push(Vector2::new(x, y));
        }
        let p = q
            .iter()
            .map(|point| (rotation_true * point) + translation_true)
            .collect::<Vec<_>>();
        dbg!(&q);
        dbg!(&p);

        for (p1, p2) in p.iter().zip(expected_p.iter()) {
            let residuals = p1 - p2;
            println!("Residuals: {}", residuals);
            assert!(residuals.norm() < 1e-6);
        }
        for (q1, q2) in q.iter().zip(expected_q.iter()) {
            let residuals = q1 - q2;
            println!("Residuals: {}", residuals);
            assert!(residuals.norm() < 1e-6);
        }
    }

    #[test]
    fn test_overall() {
        let p = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(3.0, 0.0),
        ];

        let rotation = Rotation2::new(0.4);
        let translation = Vector2::new(0.1, 0.1);
        let q = p
            .iter()
            .map(|point| (rotation * point) + translation)
            .collect::<Vec<_>>();

        let x = icp_least_squares(&p, &q, Vector3::zeros(), 100);
        let expected_x = Vector3::new(0.1, 0.1, 0.4);
        let residuals = x - expected_x;
        println!("Residuals: {}", residuals);
        assert!(residuals.norm() < 1e-6);
    }
}