use nalgebra::{matrix,vector, Matrix2, Vector2};

use super::math::{Integrator, Derivative, rad_to_deg};

pub struct Motor{
    a_matrix: Matrix2<f64>,
    b_vector: Vector2<f64>,
    i_matrix: Matrix2<f64>,
    ss_vector: Vector2<f64>,
    position: Integrator,
    velocity: f64,
    acceleration: Derivative,
    torque: f64,
    k :f64
}

impl Motor {
    pub fn new(j: f64, b: f64, l: f64, r: f64, k: f64, ss: [f64; 2]) -> Self{
        let a_matrix = matrix![-b/j, k/j; -k/l, -r/l];
        let b_vector = vector![0.0, 1.0/l];
        let i_matrix = matrix![1.0, 0.0; 0.0, 1.0];
        let ss_vector = vector![ss[0], ss[1]];
        let position = Integrator::default();
        let velocity = ss_vector[0];
        let acceleration = Derivative::default();
        let torque = k*ss_vector[1];
        Self {a_matrix, b_vector, i_matrix, ss_vector, position, velocity, acceleration, torque, k}
    }

    pub fn update_state(&mut self, delta: f64, voltage: f64){
        let a_d_matrix = self.i_matrix+delta*self.a_matrix;
        let b_d_vector = delta*self.b_vector;
        self.ss_vector = a_d_matrix*self.ss_vector+b_d_vector*voltage;
        self.position.integrate(delta, self.ss_vector[0]); 
        self.velocity = self.ss_vector[0];
        self.acceleration.derivate(delta, self.ss_vector[0]);
        self.torque = self.k*self.ss_vector[1]
    }

    pub fn get_position(&self) -> f64{
        rad_to_deg(self.position.get_state())
    }
    
    pub fn get_velocity(&self) -> f64{
        self.velocity
    }

    pub fn get_acceleration(&self) -> f64{
        self.acceleration.get_state()
    }

    pub fn get_torque(&self) -> f64{
        self.torque
    }

}
