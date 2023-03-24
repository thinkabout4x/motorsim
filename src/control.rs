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

pub struct Contoller{
    motor: Motor,
    time: Time,
    duration: f64
}

impl Pid {
    pub fn new(kp: u32, ki: u32, kd: u32) -> Self{
        Self { kp, ki, kd}
    }
}

impl Contoller{
    pub fn new(motor: Motor, duration: f64) -> Self{
        let time = Time::new();
        Self {motor, time, duration}
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
}

