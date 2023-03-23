mod motor;
mod math;
mod time_mod;
use eframe::egui::plot::{PlotPoints};

pub use crate::control::motor::Motor;
use self::time_mod::Time;

pub struct Pid{
    pub kp: u32,
    pub kd: u32,
    pub ki: u32,
}

pub struct Contoller{
    motor: Motor,
    time: Time

}

impl Pid {
    pub fn new(kp: u32, ki: u32, kd: u32) -> Self{
        Self { kp, ki, kd}
    }
}

impl Contoller{
    pub fn new(motor: Motor) -> Self{
        let time = Time::new();
        Self {motor, time}
    }

    pub fn get_pos_vec_acc(&mut self) -> Vec<[f64; 2]>{
        self.time.update_state(); 
        self.motor.update_state(self.time.get_delta(), 12.0);
        vec![[self.time.get_time_from_start(), self.motor.get_position()], [self.time.get_time_from_start(), self.motor.get_velocity()], [self.time.get_time_from_start(), self.motor.get_acceleration()]]
    }
    pub fn test(&mut self) -> PlotPoints{
        let result: PlotPoints = (0..50).map(|i | {
            
            self.motor.update_state(0.1, 12.0);
            [i as f64 +0.1, self.motor.get_velocity()]
        }).collect::<PlotPoints>();

        result

    }
}

