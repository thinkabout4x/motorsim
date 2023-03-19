mod motor;
mod math;
mod time_mod;
use crate::control::motor::Motor;

pub struct Pid{
    pub kp: u32,
    pub kd: u32,
    pub ki: u32,
}

pub struct Contoller{
    motor: Motor, 

}

impl Pid {
    pub fn new(kp: u32, ki: u32, kd: u32) -> Self{
        Self { kp, ki, kd}
    }
}

impl Contoller{
    pub fn new(motor: Motor) -> Self{
        Self {motor}
    }
}

