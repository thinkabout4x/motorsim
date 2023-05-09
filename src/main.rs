pub mod control;
pub mod ui;
use std::{thread, time::{Duration}, sync::{mpsc::{self, Sender, Receiver}}};
use control::{Controller, Config};

use crate::ui::Motorsim;

fn main() {
    let (tx, rx): (Sender<Config>, Receiver<Config>) = mpsc::channel();

    let motorsim = Motorsim::new(tx.clone());
    let plotpoints = motorsim.get_plotpoints();
    let target = motorsim.get_target();

    let thread = thread::spawn(move || {
        let mut controller = Controller::new(rx.recv().unwrap(), plotpoints, target);

        loop{
            match rx.try_recv(){
                Ok(config) => {
                    controller.reset(config);
                }
                Err(_) => {}
            }

            if !controller.get_controller_conf().get_end_flag(){
                if *(controller.get_controller_conf().get_start_flag()){
                    controller.calculate_point();
                } else {
                    thread::sleep(Duration::from_millis(100));
                }
            } else {
                break
            }
        }
        println!("Controller thread end");
    });

    Motorsim::run(motorsim, 1920, 1080);

    thread.join().unwrap();
    
}