use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use eframe::egui::{self, InnerResponse, Ui};
use egui::plot::{Line, Plot, PlotPoints};
use crate::control::Motor;
use crate::control::{Pid, Controller};

pub struct Motorsim{
    angle_pid: Pid,
    speed_pid: Pid,
    torque_pid: Pid,
    controller: Arc<Mutex<Controller>>,
    endstate: Arc<AtomicBool>
}

impl Default for Motorsim {
    fn default() -> Self {
        Self {
            angle_pid : Pid::new(0,0,0),
            speed_pid : Pid::new(0,0,0),
            torque_pid : Pid::new(0,0,0),
            controller: Arc::new(Mutex::new(Controller::new(Motor::new(0.01, 0.1, 0.5, 1.0, 0.01, [0.0, 0.0]), 5.0))),
            endstate: Arc::new(AtomicBool::new(false))
        }
    }
}

impl eframe::App for Motorsim {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame){
        egui::SidePanel::right("right").show(ctx, |ui|{
            
            ui.vertical_centered(|ui|{
                ui.label("Plots");
            });

            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_pos()), ui, "Angle");
            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_vel()), ui, "Speed");
            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_acc()), ui, "Acceleration");
            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_trq()), ui, "Torque");
        });

        egui::SidePanel::left("left").show(ctx, |ui|{
            Motorsim::controller(ui, "Angle controller", &mut self.angle_pid);
            Motorsim::controller(ui, "Speed controller", &mut self.speed_pid);
            Motorsim::controller(ui, "Torque controller", &mut self.torque_pid);

        });

        ctx.request_repaint();

    }

    fn on_close_event(&mut self) -> bool {
        self.endstate.store(true, Ordering::Relaxed);
        true
    }
}

impl Motorsim {
    pub fn run(motorsim: Motorsim,  width: i32, height: i32 ) {
        tracing_subscriber::fmt::init();

        let options = eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(width as f32, height as f32)),
            ..Default::default()
        };
        eframe::run_native(
            "Motor control sim",
            options,
            Box::new(|_cc| Box::new(motorsim)),
        ).unwrap()

    }

    fn plot(points: PlotPoints, ui: &mut Ui, id: &str ) -> InnerResponse<()> {
        let line = Line::new(points);
        Plot::new(id).view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line))
    }

    fn controller(ui: &mut Ui, label: &str, pid: &mut Pid) -> InnerResponse<()> {
        ui.vertical_centered(|ui|{
            ui.label(label);
        });

        ui.group(|ui|{
            ui.horizontal(|ui| {
                ui.label("Kp :");
                ui.add(egui::DragValue::new(&mut pid.kp).speed(0.05));
                ui.label("Kd :");
                ui.add(egui::DragValue::new(&mut pid.kd).speed(0.05));
                ui.label("Ki :");
                ui.add(egui::DragValue::new(&mut pid.ki).speed(0.05));
                if ui.add(egui::Button::new("Calibrate")).clicked() {
                    
                };
            });
        })
    }

    pub fn get_controller(&self) -> Arc<Mutex<Controller>>{
        Arc::clone(&self.controller)
    }

    pub fn get_endstate(&self) -> Arc<AtomicBool>{
        Arc::clone(&self.endstate)
    }

}