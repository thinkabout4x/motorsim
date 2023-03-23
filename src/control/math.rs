pub struct Integrator {
    integral: f64,

}

pub struct Derivative {
    prev_state: f64,
    derivative: f64
}

impl Default for Integrator{
    fn default() -> Self{
        Self{
            integral: 0.0
        }
    }
}

impl Default for Derivative{
    fn default() -> Self{
        Self{
            prev_state: 0.0,
            derivative: 0.0
        }
    }
}

impl Integrator{
    pub fn integrate(&mut self, delta: f64, state: f64){
        self.integral += delta*state;
    }
    pub fn get_state(&self) -> f64{
        self.integral
    }
}

impl Derivative{
    pub fn derivate(&mut self, delta: f64, state: f64){
        self.derivative = (state-self.prev_state)/delta;
        self.prev_state = state;
    }
    pub fn get_state(&self) -> f64{
        self.derivative
    }
}

pub fn rad_to_deg(rad: f64) -> f64{
    let full_deg = rad*180.0/std::f64::consts::PI;
    full_deg % 360.0
}