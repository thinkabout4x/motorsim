mod motor;
mod math;
mod time_mod;

pub use crate::control::motor::Motor;
use self::{time_mod::Time, math::{Integrator, Derivative}};

pub struct Pid{
    kp: f64,
    kd: f64,
    ki: f64,
    integral: Integrator,
    derivative: Derivative
}

pub struct Controller{
    motor: Motor,
    pos: Vec<[f64; 2]>,
    pos_pid: Pid,
    vel: Vec<[f64; 2]>,
    vel_pid: Pid,
    acc: Vec<[f64; 2]>,
    trq: Vec<[f64; 2]>,
    trq_pid: Pid,
    time: Time,
    duration: f64
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self{
        Self {kp, ki, kd, integral: Integrator::default(), derivative: Derivative::default()}
    }

    pub fn reset(&mut self){
        self.integral = Integrator::default();
        self.derivative = Derivative::default();
    }

    pub fn get_kp(&mut self) -> &mut f64{
        &mut self.kp
    }

    pub fn get_kd(&mut self) ->&mut f64{
        &mut self.kd
    }

    pub fn get_ki(&mut self) ->&mut f64{
        &mut self.ki
    }

    pub fn generate_control(&mut self, input: f64, target: f64, delta: f64) -> f64{
        let error = target - input;
        self.derivative.derivate(delta, error);
        self.integral.integrate(delta, error);
        self.kp*error+self.kd*self.derivative.get_state()+self.ki*self.integral.get_state()
    }
}

impl Controller{
    pub fn new(motor: Motor, duration: f64) -> Self{
        let time = Time::new();
        Self {motor, time, duration,
             pos: vec![],pos_pid: Pid::new(0.0,0.0,0.0),
              vel: vec![], vel_pid: Pid::new(0.0,0.0,0.0),
              acc: vec![], trq: vec![], trq_pid: Pid::new(0.0,0.0,0.0)}
    }

    pub fn update_state(&mut self, target:f64){
        self.time.update_state(); 
        let time_from_start = self.time.get_time_from_start();
        let delta = self.time.get_delta();
        let mut input = self.pos_pid.generate_control(self.motor.get_position(), target, delta);

        if input> 24.0{
            input = 24.0;
        } 
        
        self.motor.update_state(delta, input);
        if time_from_start <= self.duration{
            self.pos.push([time_from_start, self.motor.get_position()]);
            self.vel.push([time_from_start, self.motor.get_velocity()]);
            self.acc.push([time_from_start, self.motor.get_acceleration()]);
            self.trq.push([time_from_start, self.motor.get_torque()]);
        }
    }

    pub fn reset(&mut self){
        self.time = Time::new();
        self.pos_pid.reset();
        self.vel_pid.reset();
        self.vel_pid.reset();
        self.motor.reset();
        self.pos = vec![];
        self.vel = vec![];
        self.acc = vec![];
        self.trq = vec![];
    }

    pub fn get_pos(&self) -> Vec<[f64; 2]>{
        self.pos.clone()
    }
    pub fn get_pos_pid(&mut self) -> &mut Pid{
        &mut self.pos_pid
    }
    pub fn get_vel(&self) -> Vec<[f64; 2]>{
        self.vel.clone()
    }
    pub fn get_vel_pid(&mut self) -> &mut Pid{
        &mut self.vel_pid
    }
    pub fn get_acc(&self) -> Vec<[f64; 2]>{
        self.acc.clone()
    }
    pub fn get_trq(&self) -> Vec<[f64; 2]>{
        self.trq.clone()
    }
    pub fn get_trq_pid(&mut self) -> &mut Pid{
        &mut self.trq_pid
    }
}


