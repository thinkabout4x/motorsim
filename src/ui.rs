use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use eframe::egui::plot::PlotPoint;
use eframe::egui::{self, InnerResponse, Ui};
use egui::plot::{Line, Plot, PlotPoints};
use crate::control::Pid;

pub struct Motorsim{
    angle_pid: Pid,
    speed_pid: Pid,
    torque_pid: Pid,
    pos_points: Arc<Mutex<Vec<egui::plot::PlotPoint>>>,
    vel_points: Arc<Mutex<Vec<egui::plot::PlotPoint>>>,
    acc_points: Arc<Mutex<Vec<egui::plot::PlotPoint>>>,
    endstate: Arc<AtomicBool>
}

impl Default for Motorsim {
    fn default() -> Self {
        Self {
            angle_pid : Pid::new(0,0,0),
            speed_pid : Pid::new(0,0,0),
            torque_pid : Pid::new(0,0,0),
            pos_points: Arc::new(Mutex::new(vec![[0.0,0.0].into()])),
            vel_points: Arc::new(Mutex::new(vec![[0.0,0.0].into()])),
            acc_points: Arc::new(Mutex::new(vec![[0.0,0.0].into()])),
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
            
            Motorsim::plot(self.pos_points.lock().unwrap().to_vec(), ui, "Angle");
            Motorsim::plot(self.vel_points.lock().unwrap().to_vec(), ui, "Speed");
            Motorsim::plot(self.acc_points.lock().unwrap().to_vec(), ui, "Acceleration");
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

    fn plot(points: Vec<PlotPoint>, ui: &mut Ui, id: &str ) -> InnerResponse<()> {
        let line = Line::new(PlotPoints::Owned(points));
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

    pub fn get_pos_points(&self) -> Arc<Mutex<Vec<egui::plot::PlotPoint>>>{
        Arc::clone(&self.pos_points)
    }
    pub fn get_vel_points(&self) -> Arc<Mutex<Vec<egui::plot::PlotPoint>>>{
        Arc::clone(&self.vel_points)
    }
    pub fn get_acc_points(&self) -> Arc<Mutex<Vec<egui::plot::PlotPoint>>>{
        Arc::clone(&self.acc_points)
    }

    pub fn get_endstate(&self) -> Arc<AtomicBool>{
        Arc::clone(&self.endstate)
    }

}