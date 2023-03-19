pub mod control;
pub mod ui;
use crate::ui::Motorsim;


fn main() -> Result<(), eframe::Error> {
    // let motor = motor::Motor::new(0.01, 0.1, 0.5, 1.0, 0.01, [0.0, 0.0]);


    let motorsim = Motorsim::default();
    motorsim.run(1280, 720)
    
}