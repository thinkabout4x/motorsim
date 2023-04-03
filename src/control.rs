mod motor;
mod math;
mod time_mod;

pub use crate::control::motor::Motor;
use self::{time_mod::Time, math::{Integrator, Derivative}};


pub enum CalibType {
    Pos,
    PosVelTrq,
    VelTrq,
    Trq,
}
#[derive(PartialEq)]
pub enum ControlType {
    Pos,
    PosVelTrq,
}

pub enum PidType {
    Pos,
    Vel,
    Trq,
}


pub struct Pid{
    kp: f64,
    kd: f64,
    ki: f64,
    integral: Integrator,
    derivative: Derivative,
    option: PidType
}


pub struct Controller{
    motor: Motor,
    target: f64,
    calib_option: Option<CalibType>,
    control_option: ControlType,
    pos: Vec<[f64; 2]>,
    pos_pid: Pid,
    vltg_bound: f64,
    vel: Vec<[f64; 2]>,
    vel_pid: Pid,
    vel_bound: f64,
    voltage: Vec<[f64; 2]>,
    trq: Vec<[f64; 2]>,
    trq_pid: Pid,
    trq_bound: f64,
    time: Time,
    duration: f64
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, option: PidType) -> Self{
        Self {kp, ki, kd, integral: Integrator::default(), derivative: Derivative::default(), option}
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

    pub fn get_option(&self) -> &PidType{
        &self.option
    }

    pub fn generate_control(&mut self, input: f64, target: f64, delta: f64, bound: f64) -> f64{
        let error = target - input;
        self.derivative.derivate(delta, error);
        self.integral.integrate(delta, error);
        let mut result = self.kp*error+self.kd*self.derivative.get_state()+self.ki*self.integral.get_state();

        if result.abs() > bound{
            if result< 0.0{
                result = -bound;
            }else{
                result = bound;
            }
        } 
        result
    }
}

impl Controller{
    pub fn new(motor: Motor, duration: f64) -> Self{
        let time = Time::new();
        Self {motor, time, duration, calib_option: None, control_option: ControlType::PosVelTrq,
             pos: vec![],pos_pid: Pid::new(0.5,10.0,0.003, PidType::Pos), target: 0.0, vltg_bound: 24.0,
              vel: vec![], vel_pid: Pid::new(0.001,0.0,0.0, PidType::Vel), vel_bound: 4000.0,
              voltage: vec![], trq: vec![], trq_pid: Pid::new(0.1,5000.0,0.0, PidType::Trq), trq_bound: 1.0}
    }

    pub fn generate_control(&mut self, delta: f64) -> f64{
        let input = match &self.calib_option{
            Some(t) =>{
                match t{
                    CalibType::Pos => {
                        let target = 180.0;
                        self.pos_pid.generate_control(self.motor.get_position(), target, delta, self.vltg_bound)
                    }
                    CalibType::PosVelTrq => {
                        let target = 180.0;
                        let vel = self.pos_pid.generate_control(self.motor.get_position(), target, delta, self.vel_bound);
                        let trq = self.vel_pid.generate_control(self.motor.get_velocity(), vel, delta, self.trq_bound);
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.vltg_bound);
                        vltg
                    }
                    CalibType::VelTrq => {
                        let target = self.vel_bound/2.;
                        let trq = self.vel_pid.generate_control(self.motor.get_velocity(), target, delta, self.trq_bound);
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.vltg_bound);
                        vltg
                    }
                    CalibType::Trq => {
                        let target = self.trq_bound/2.;
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), target, delta, self.vltg_bound);
                        vltg
                    }
                }
            }
            None => {
                match self.control_option{
                    ControlType::Pos => {
                        self.pos_pid.generate_control(self.motor.get_position(), self.target, delta, self.vltg_bound)
                    }
                    ControlType::PosVelTrq => {
                        let vel = self.pos_pid.generate_control(self.motor.get_position(), self.target, delta, self.vel_bound);
                        let trq = self.vel_pid.generate_control(self.motor.get_velocity(), vel, delta, self.trq_bound);
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.vltg_bound);
                        vltg
                    }
                }
            }
        };
        input
    }

    pub fn update_state(&mut self){
        self.time.update_state(); 
        let time_from_start = self.time.get_time_from_start();
        let delta = self.time.get_delta();
        let input = self.generate_control(delta);
        
        self.motor.update_state(delta, input);
        if time_from_start <= self.duration{
            self.pos.push([time_from_start, self.motor.get_position()]);
            self.vel.push([time_from_start, self.motor.get_velocity()]);
            self.voltage.push([time_from_start, input]);
            self.trq.push([time_from_start, self.motor.get_torque()]);
        }
    }

    pub fn reset(&mut self){
        self.time = Time::new();
        self.pos_pid.reset();
        self.vel_pid.reset();
        self.trq_pid.reset();
        self.motor.reset();
        self.pos = vec![];
        self.vel = vec![];
        self.voltage = vec![];
        self.trq = vec![];
    }

    pub fn get_motor(&mut self) -> &mut Motor{
        &mut self.motor
    }
    pub fn get_pos(&self) -> Vec<[f64; 2]>{
        self.pos.clone()
    }
    pub fn get_pos_pid(&mut self) -> &mut Pid{
        &mut self.pos_pid
    }
    pub fn get_target(&mut self) -> &mut f64{
        &mut self.target
    }
    pub fn get_vltg_bound(&mut self) -> &mut f64{
        &mut self.vltg_bound
    }
    pub fn get_vel(&self) -> Vec<[f64; 2]>{
        self.vel.clone()
    }
    pub fn get_vel_pid(&mut self) -> &mut Pid{
        &mut self.vel_pid
    }
    pub fn get_vel_bound(&mut self) -> &mut f64{
        &mut self.vel_bound
    }
    pub fn get_voltage(&self) -> Vec<[f64; 2]>{
        self.voltage.clone()
    }
    pub fn get_trq(&self) -> Vec<[f64; 2]>{
        self.trq.clone()
    }
    pub fn get_trq_pid(&mut self) -> &mut Pid{
        &mut self.trq_pid
    }
    pub fn get_trq_bound(&mut self) -> &mut f64{
        &mut self.trq_bound
    }
    pub fn get_control_option(&mut self) -> &mut ControlType{
        &mut self.control_option
    }
    pub fn get_calib_option(&mut self) -> &mut Option<CalibType>{
        &mut self.calib_option
    }
}


