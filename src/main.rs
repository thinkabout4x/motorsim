pub mod control;
pub mod ui;
use std::{thread, time::{Duration}, sync::{atomic::{ Ordering}}};
use crate::ui::Motorsim;

fn main() {
    let motorsim = Motorsim::default();
    let thread_end_flag = motorsim.get_endstate();
    let thread_start_flag = motorsim.get_startstate();
    let controller = motorsim.get_controller();

    let thread = thread::spawn(move || {
        while !thread_end_flag.load(Ordering::Relaxed){
            if thread_start_flag.load(Ordering::Relaxed){
                controller.lock().unwrap().update_state(90.);
                thread::sleep(Duration::from_millis(1));
            }
            else{
                {}
            };
        }
        println!("calc thread end");
    });

    Motorsim::run(motorsim, 1920, 1080);

    thread.join().unwrap();
    
}