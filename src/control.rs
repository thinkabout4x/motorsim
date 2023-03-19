mod motor;
mod math;
mod time_mod;
use eframe::egui::plot::PlotPoints;

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
    pub fn test(&mut self) -> PlotPoints{
        let result: PlotPoints = (0..50).map(|i | {
            
            self.motor.update_state(0.1, 12.0);
            [i as f64 +0.1, self.motor.get_velocity()]
        }).collect::<PlotPoints>();

        result

    }
}

