pub mod control;
pub mod ui;
use std::{thread, time::{Duration}, sync::{atomic::{ Ordering}, mpsc::{self, Sender, Receiver}}};
use control::{Controller, Config};

use crate::ui::Motorsim;

fn main() {
    let (tx, rx): (Sender<Config>, Receiver<Config>) = mpsc::channel();

    let motorsim = Motorsim::new(tx.clone());
    let thread_end_flag = motorsim.get_endstate();
    let thread_start_flag = motorsim.get_startstate();
    let plotpoints = motorsim.get_plotpoints();

    

    let thread = thread::spawn(move || {
        let mut controller = Controller::new(rx.recv().unwrap(), plotpoints);

        while !thread_end_flag.load(Ordering::Relaxed){
            if thread_start_flag.load(Ordering::Relaxed){
                match rx.try_recv(){
                    Ok(config) => {
                        controller.reset(config);
                        controller.calculate_points();
                    }
                    Err(_) => {}
                }
                thread::sleep(Duration::from_millis(1));
            }
        }
        println!("Controller thread end");
    });

    Motorsim::run(motorsim, 1920, 1080);

    thread.join().unwrap();
    
}