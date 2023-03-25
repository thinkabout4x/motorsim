mod motor;
mod math;
mod time_mod;

pub use crate::control::motor::Motor;
use self::time_mod::Time;

pub struct Pid{
    pub kp: u32,
    pub kd: u32,
    pub ki: u32,
}

pub struct Controller{
    motor: Motor,
    pos: Vec<[f64; 2]>,
    vel: Vec<[f64; 2]>,
    acc: Vec<[f64; 2]>,
    trq: Vec<[f64; 2]>,
    time: Time,
    duration: f64
}

impl Pid {
    pub fn new(kp: u32, ki: u32, kd: u32) -> Self{
        Self { kp, ki, kd}
    }
}

impl Controller{
    pub fn new(motor: Motor, duration: f64) -> Self{
        let time = Time::new();
        Self {motor, time, duration, pos: vec![], vel: vec![], acc: vec![], trq: vec![]}
    }

    pub fn get_pos_vec_acc_trq(&mut self) -> Option<Vec<[f64; 2]>>{
        self.time.update_state(); 
        let time_from_start = self.time.get_time_from_start();
        self.motor.update_state(self.time.get_delta(), 12.0);
        if time_from_start <= self.duration{
            Some(vec![[time_from_start, self.motor.get_position()], 
            [time_from_start, self.motor.get_velocity()], 
            [time_from_start, self.motor.get_acceleration()], 
            [time_from_start, self.motor.get_torque()]])
        }
        else {
            None
        }
    }

    pub fn update_state(&mut self){
        self.time.update_state(); 
        let time_from_start = self.time.get_time_from_start();
        self.motor.update_state(self.time.get_delta(), 12.0);
        if time_from_start <= self.duration{
            self.pos.push([time_from_start, self.motor.get_position()]);
            self.vel.push([time_from_start, self.motor.get_velocity()]);
            self.acc.push([time_from_start, self.motor.get_acceleration()]);
            self.trq.push([time_from_start, self.motor.get_torque()]);
        }
    }

    pub fn get_pos(&self) -> Vec<[f64; 2]>{
        self.pos.clone()
    }
    pub fn get_vel(&self) -> Vec<[f64; 2]>{
        self.vel.clone()
    }
    pub fn get_acc(&self) -> Vec<[f64; 2]>{
        self.acc.clone()
    }
    pub fn get_trq(&self) -> Vec<[f64; 2]>{
        self.trq.clone()
    }
}

