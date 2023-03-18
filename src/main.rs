use std::thread::Thread;

mod ui;
mod time_mod;
mod motor;

fn main() -> Result<(), eframe::Error> {
    let motor = motor::Motor::new(0.01, 0.1, 0.5, 1.0, 0.01, [0.0, 0.0]);


    let motorsim = ui::Motorsim::default();
    motorsim.run(1280, 720)
    

}