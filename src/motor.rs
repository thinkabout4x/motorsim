use nalgebra::{matrix,vector, Matrix2, Vector2};

pub struct Motor{
    a_matrix: Matrix2<f64>,
    b_vector: Vector2<f64>,
    i_matrix: Matrix2<f64>,
    ss_vector: Vector2<f64>,
    j: f64,
    b: f64,
    l: f64,
    r: f64,
    k: f64

}

impl Motor {
    pub fn new(j: f64, b: f64, l: f64, r: f64, k: f64, ss: [f64; 2]) -> Self{
        let a_matrix = matrix![-b/j, k/j; -k/l, -r/l];
        let b_vector = vector![0.0, 1.0/l];
        let i_matrix = matrix![1.0, 0.0; 0.0, 1.0];
        let ss_vector = vector![ss[0], ss[1]];
        Self {j, b, l, r, k, a_matrix, b_vector, i_matrix, ss_vector}
    }

    pub fn get_ss_vec(self, delta: f64, voltage: f64) -> Vector2<f64>{
        let a_d_matrix = delta*self.a_matrix;
        let b_d_vector = delta*self.b_vector;
        a_d_matrix*self.ss_vector+b_d_vector*voltage
    }
}
