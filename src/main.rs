pub mod control;
pub mod ui;
use std::{thread, time::{Duration}, sync::{atomic::{ Ordering}}};
use control::{Contoller, Motor};
use crate::ui::Motorsim;

fn main() {
    let motorsim = Motorsim::default();
    let pos_points = motorsim.get_pos_points();
    let vel_points = motorsim.get_vel_points();
    let acc_points = motorsim.get_acc_points();
    let trq_points = motorsim.get_trq_points();
    let thread_end_flag = motorsim.get_endstate();

    let mut controller = Contoller::new(Motor::new(0.01, 0.1, 0.5, 1.0, 0.01, [0.0, 0.0]), 10.0);

    let thread = thread::spawn(move || {

        while !thread_end_flag.load(Ordering::Relaxed){
            let vector = controller.get_pos_vec_acc_trq();
            match vector{
                Some(vector) => {
                    pos_points.lock().unwrap().push(vector[0].into());
                    vel_points.lock().unwrap().push(vector[1].into());
                    acc_points.lock().unwrap().push(vector[2].into());
                    trq_points.lock().unwrap().push(vector[3].into());
                },
                None => {}
            }
            thread::sleep(Duration::from_millis(50));
        }
        println!("calc thread end")
    });

    Motorsim::run(motorsim, 1920, 1080);

    thread.join().unwrap();
    
}