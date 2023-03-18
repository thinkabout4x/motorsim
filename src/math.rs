pub struct integrator {
    integral: f64,

}

pub struct derivative {
    prev_state: f64
}

impl Default for integrator{
    fn default() -> Self{
        Self{
            integral: 0.0
        }
    }
}

impl Default for derivative{
    fn default() -> Self{
        Self{
            prev_state: 0.0
        }
    }
}

impl integrator{
    pub fn integrate(delta: f64, state: f64) -> f64{
        self.integral += delta*state;
        self.integral
    }
}

impl derivative{
    pub fn derivate(delta: f64, state: f64) -> f64{
        let derivative = (state-self.prev_state)/delta;
        self.prev_state = state;
        derivate
    }
}