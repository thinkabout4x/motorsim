use nalgebra::{matrix,vector, Matrix2, Vector2};

use super::math::{Integrator, Derivative, rad_to_deg, rads_to_rpm};

#[derive(Copy, Clone)]
pub struct ConfigMotor {
    j: f64,
    b: f64,
    l: f64,
    r: f64,
    k: f64
}

pub struct Motor{
    a_matrix: Matrix2<f64>,
    b_vector: Vector2<f64>,
    i_matrix: Matrix2<f64>,
    ss_vector: Vector2<f64>,
    position: Integrator,
    velocity: f64,
    acceleration: Derivative,
    torque: f64,
    config: ConfigMotor
}

impl Default for ConfigMotor{
    fn default() -> Self {
        Self{j:0.00065, b:0.000024 , l:0.00073, r:0.7, k:0.057}
    }
}

impl ConfigMotor{
    pub fn set_j(&mut self) ->&mut f64{
        &mut self.j
    }

    pub fn set_b(&mut self) ->&mut f64{
        &mut self.b
    }

    pub fn set_k(&mut self) ->&mut f64{
        &mut self.k
    }

    pub fn set_r(&mut self) ->&mut f64{
        &mut self.r
    }

    pub fn set_l(&mut self) ->&mut f64{
        &mut self.l
    }
}

impl Motor {
    pub fn new(config: ConfigMotor) -> Self{
        let a_matrix = matrix![-config.b/config.j, config.k/config.j; -config.k/config.l, -config.r/config.l];
        let b_vector = vector![0.0, 1.0/config.l];
        let i_matrix = matrix![1.0, 0.0; 0.0, 1.0];
        let ss_vector = vector![0.0, 0.0];
        let position = Integrator::default();
        let velocity = ss_vector[0];
        let acceleration = Derivative::default();
        let torque = config.k*ss_vector[1];
        Self {a_matrix, b_vector, i_matrix, ss_vector, position, velocity, acceleration, torque, config}
    }

    pub fn update_state(&mut self, delta: f64, voltage: f64){
        let a_d_matrix = (delta*self.a_matrix).exp();
        let b_d_vector = self.a_matrix.try_inverse().unwrap()*(a_d_matrix-self.i_matrix)*self.b_vector;
        self.ss_vector = a_d_matrix*self.ss_vector+b_d_vector*voltage;
        self.position.integrate(delta, self.ss_vector[0]); 
        self.velocity = self.ss_vector[0];
        self.acceleration.derivate(delta, self.ss_vector[0]);
        self.torque = self.config.k*self.ss_vector[1];
    }

    pub fn reset(&mut self, config: ConfigMotor){
        self.config = config;
        self.a_matrix = matrix![-config.b/config.j, config.k/config.j; -config.k/config.l, -config.r/config.l];
        self.b_vector = vector![0.0, 1.0/config.l];
        self.i_matrix = matrix![1.0, 0.0; 0.0, 1.0];
        self.ss_vector = vector![0.0, 0.0];
        self.position = Integrator::default();
        self.velocity = self.ss_vector[0];
        self.acceleration = Derivative::default();
        self.torque = config.k*self.ss_vector[1];

    }

    pub fn get_position(&self) -> f64{
        rad_to_deg(self.position.get_state())
    }
    
    pub fn get_velocity(&self) -> f64{
        rads_to_rpm(self.velocity)
    }

    pub fn get_acceleration(&self) -> f64{
        self.acceleration.get_state()
    }

    pub fn get_torque(&self) -> f64{
        self.torque
    }

}
