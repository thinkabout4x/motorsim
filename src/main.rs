pub mod control;
pub mod ui;
use std::{thread, time::{Duration}, sync::{atomic::{ Ordering}}};
use crate::ui::Motorsim;

fn main() {
    let motorsim = Motorsim::default();
    let thread_end_flag = motorsim.get_endstate();
    let controller = motorsim.get_controller().clone();

    let thread = thread::spawn(move || {

        while !thread_end_flag.load(Ordering::Relaxed){
            controller.lock().unwrap().update_state();
            thread::sleep(Duration::from_millis(50));
        }
        println!("calc thread end")
    });

    Motorsim::run(motorsim, 1920, 1080);

    thread.join().unwrap();
    
}