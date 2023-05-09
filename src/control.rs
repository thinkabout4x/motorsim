pub mod motor;
mod math;
mod time_mod;

use std::{sync::{Mutex, Arc}, collections::VecDeque};

pub use crate::control::motor::Motor;
use self::{time_mod::Time, math::{Integrator, Derivative}, motor::ConfigMotor};

#[derive(PartialEq, Copy, Clone)]
pub enum ControlType {
    Pos,
    PosVelTrq,
}


#[derive(Clone,Copy, PartialEq)]
pub enum TypePid {
    Pos,
    Vel,
    Trq,
}


pub struct Pid{
    config: ConfigPid,
    integral: Integrator,
    derivative: Derivative,
}

#[derive(Clone,Copy)]
pub struct ConfigPid{
    kp: f64,
    ki: f64,
    kd: f64,
    option: TypePid
}

#[derive(Copy, Clone)]
pub struct ConfigController{
    vltg_bound: f64,
    vel_bound: f64,
    trq_bound: f64,
    duration: f64,
    frequency: f64,
    calib_option: Option<TypePid>,
    control_option: ControlType,
    start_flag: bool,
    end_flag: bool
}

#[derive(Copy, Clone)]
pub struct Config{
    motor: ConfigMotor,
    pid_conf: [ConfigPid; 3],
    controller: ConfigController
}

pub struct PlotPnts{
    pos: VecDeque<[f64; 2]>,
    vel: VecDeque<[f64; 2]>,
    voltage: VecDeque<[f64; 2]>,
    trq: VecDeque<[f64; 2]>,
}


pub struct Controller{
    motor: Motor,
    pos_pid: Pid,
    vel_pid: Pid,
    trq_pid: Pid,
    time: Time,
    config: ConfigController,
    plotpoints: Arc<Mutex<PlotPnts>>,
    target: Arc<Mutex<f64>>
}

impl Default for ConfigController{
    fn default() -> Self {
        Self{vltg_bound: 24., vel_bound: 4000.,trq_bound: 1., duration: 3.0, frequency: 1000., calib_option: None, control_option: ControlType::PosVelTrq, start_flag: false, end_flag: false }
    }
}


impl Default for Config{
    fn default() -> Self {
        Self{motor: ConfigMotor::default(), pid_conf: [ConfigPid::new(40.0, 1.0,1.5, TypePid::Pos),
            ConfigPid::new(0.001, 0.0,0.0, TypePid::Vel),
            ConfigPid::new(8.0, 5000.0,0.0, TypePid::Trq)],
            controller: ConfigController::default()}
    }
}

impl Default for PlotPnts{
    fn default() -> Self {
        Self{pos: vec![].into(), vel: vec![].into(), trq: vec![].into(), voltage: vec![].into() }
    }
}

impl Iterator for ConfigPid{
    fn next(&mut self) -> Option<ConfigPid> {
         Some(*self) }
    type Item = ConfigPid;
}

impl PlotPnts{
    pub fn clone_pos_as_vec(&self) -> Vec<[f64; 2]>{
        self.pos.clone().into()
    }

    pub fn clone_vel_as_vec(&self) -> Vec<[f64; 2]>{
        self.vel.clone().into()
        }

    pub fn clone_trq_as_vec(&self) -> Vec<[f64; 2]>{
        self.trq.clone().into()    
    }

    pub fn clone_voltage_as_vec(&self) -> Vec<[f64; 2]>{
        self.voltage.clone().into()
    }
    pub fn reset(&mut self){
        self.pos = vec![].into();
        self.vel = vec![].into();
        self.trq = vec![].into();
        self.voltage = vec![].into();
    }
}

impl ConfigPid{
    pub fn new(kp: f64, ki: f64, kd: f64, option: TypePid) -> Self{
        Self{ kp, ki, kd, option}
    }
    pub fn set_kp(&mut self) -> &mut f64{
        &mut self.kp
    }

    pub fn set_kd(&mut self) ->&mut f64{
        &mut self.kd
    }

    pub fn set_ki(&mut self) ->&mut f64{
        &mut self.ki
    }

    pub fn get_option(&self) ->TypePid{
        self.option
    }
}

impl ConfigController {
    pub fn set_vltg_bound(&mut self) -> &mut f64{
        &mut self.vltg_bound
    }

    pub fn set_trq_bound(&mut self) -> &mut f64{
        &mut self.trq_bound
    }

    pub fn get_trq_bound(&self) -> &f64{
        &self.trq_bound
    }

    pub fn set_vel_bound(&mut self) -> &mut f64{
        &mut self.vel_bound
    }

    pub fn get_vel_bound(&self) -> &f64{
        &self.vel_bound
    }

    pub fn get_start_flag(&self) -> &bool{
        &self.start_flag
    }

    pub fn set_start_flag(&mut self) -> &mut bool{
        &mut self.start_flag
    }

    pub fn get_end_flag(&self) -> &bool{
        &self.end_flag
    }

    pub fn set_end_flag(&mut self) -> &mut bool{
        &mut self.end_flag
    }

    pub fn get_duration(&self) -> f64{
        self.duration
    }

    pub fn set_duration(&mut self) -> &mut f64{
        &mut self.duration
    }

    pub fn set_frequency(&mut self) -> &mut f64{
        &mut self.frequency
    }

    pub fn get_frequency(&self) -> f64{
        self.frequency
    }

    pub fn set_calib_option(&mut self) -> &mut Option<TypePid>{
        &mut self.calib_option
    }

    pub fn get_calib_option(&self) -> &Option<TypePid>{
        &self.calib_option
    }

    pub fn set_control_option(&mut self) -> &mut ControlType{
        &mut self.control_option
   }

   pub fn get_control_option(&self) -> &ControlType{
    &self.control_option
   }
}
    

impl Pid {
    pub fn new(config: ConfigPid) -> Self{
        Self {config, integral: Integrator::default(), derivative: Derivative::default()}
    }

    pub fn generate_control(&mut self, input: f64, target: f64, delta: f64, bound: f64) -> f64{
        let error = target - input;
        self.derivative.derivate(delta, error);
        self.integral.integrate(delta, error);
        let mut result = self.config.kp*error+self.config.kd*self.derivative.get_state()+self.config.ki*self.integral.get_state();

        if result.abs() > bound{
            if result< 0.0{
                result = -bound;
            }else{
                result = bound;
            }
        } 
        result
    }

    pub fn reset(&mut self, config: ConfigPid){
        self.config = config;
        self.integral = Integrator::default();
        self.derivative = Derivative::default();
    }
}

impl Config{

    pub fn set_pid_conf(&mut self) -> &mut [ConfigPid; 3]{
        &mut self.pid_conf
    }

    pub fn get_pid_conf(&self) -> &[ConfigPid; 3]{
        &self.pid_conf
    }

    pub fn set_controller_conf(&mut self) -> &mut ConfigController{
        &mut self.controller
    }

    pub fn get_controller_conf(&self) -> &ConfigController{
        &self.controller
    }

    pub fn set_motor_conf(&mut self) -> &mut ConfigMotor{
        &mut self.motor
    }
}

impl Controller{
    pub fn new(config: Config, plotpoints: Arc<Mutex<PlotPnts>>, target: Arc<Mutex<f64>>) -> Self{
        let time = Time::new(config.get_controller_conf().get_frequency());
        Self {motor: Motor::new(config.motor), time, target,
             pos_pid: Pid::new(config.pid_conf[0]),
             vel_pid: Pid::new(config.pid_conf[1]),
             trq_pid: Pid::new(config.pid_conf[2]), config: config.controller, plotpoints}
    }

    pub fn reset(&mut self, config: Config){
        self.config = config.controller;
        self.motor.reset(config.motor);
        self.pos_pid.reset(config.pid_conf[0]);
        self.vel_pid.reset(config.pid_conf[1]);
        self.trq_pid.reset(config.pid_conf[2]);
        self.plotpoints.lock().unwrap().reset();
        self.time = Time::new(self.config.get_frequency());
    }

    pub fn generate_control(&mut self, delta: f64) -> f64{
        let input = match self.config.calib_option{
            Some(pid) =>{
                match pid{
                    TypePid::Pos =>{
                        match self.config.control_option{
                            ControlType::Pos => {
                                let target = 180.0;
                                self.pos_pid.generate_control(self.motor.get_position(), target, delta, self.config.vltg_bound)
                            }
                            ControlType::PosVelTrq => {
                                let target = 180.0;
                                let vel = self.pos_pid.generate_control(self.motor.get_position(), target, delta, self.config.vel_bound);
                                let trq = self.vel_pid.generate_control(self.motor.get_velocity(), vel, delta, self.config.trq_bound);
                                let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.config.vltg_bound);
                                vltg
                            } 
                        }
                    }
                    TypePid::Vel =>{
                        let target = self.config.vel_bound/2.;
                        let trq = self.vel_pid.generate_control(self.motor.get_velocity(), target, delta, self.config.trq_bound);
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.config.vltg_bound);
                        vltg
                    }
                    TypePid::Trq =>{
                        let target = self.config.trq_bound/2.;
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), target, delta, self.config.vltg_bound);
                        vltg
                    }
                }
            }
            None => {
                match self.config.control_option{
                    ControlType::Pos => {
                        self.pos_pid.generate_control(self.motor.get_position(), *(self.target.lock().unwrap()), delta, self.config.vltg_bound)
                    }
                    ControlType::PosVelTrq => {
                        let vel = self.pos_pid.generate_control(self.motor.get_position(), *(self.target.lock().unwrap()), delta, self.config.vel_bound);
                        let trq = self.vel_pid.generate_control(self.motor.get_velocity(), vel, delta, self.config.trq_bound);
                        let vltg = self.trq_pid.generate_control(self.motor.get_torque(), trq, delta, self.config.vltg_bound);
                        vltg
                    }
                }
            }
        };
        input


    }

    pub fn calculate_point(&mut self){
        self.time.update_state();
        let time_from_start = self.time.get_time_from_start();
        
        if Controller::check_point_add(&mut self.config, time_from_start){
            let delta = self.time.get_delta();
            let input = self.generate_control(delta);
            self.motor.update_state(delta, input);
            let mut points = self.plotpoints.lock().unwrap();

            if time_from_start >= self.config.duration{
                points.pos.pop_front();
                points.vel.pop_front();
                points.trq.pop_front();
                points.voltage.pop_front();
            }

            points.pos.push_back([time_from_start, self.motor.get_position()]);
            points.vel.push_back([time_from_start, self.motor.get_velocity()]);
            points.voltage.push_back([time_from_start, input]);
            points.trq.push_back([time_from_start, self.motor.get_torque()]);
        }
    }

    fn check_point_add(config: &mut ConfigController, time_from_start: f64) -> bool{
        match config.get_calib_option() {
            Some(_) =>{
                if time_from_start <= config.get_duration(){
                    return true;
                } else {
                    *config.set_start_flag() = false;
                    return false;
                }
            }
            None => {
                return true;
            }
        }
    }

    pub fn get_controller_conf(&self) -> &ConfigController{
        &self.config
    }
}


