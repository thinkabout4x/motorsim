use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use eframe::egui::{self, InnerResponse, Ui};
use egui::plot::{Line, Plot, PlotPoints};
use crate::control::CalibType;
use crate::control::ControlType;
use crate::control::Motor;
use crate::control::{Controller};

pub struct Motorsim{
    controller: Arc<Mutex<Controller>>,
    endstate: Arc<AtomicBool>,
    startstate: Arc<AtomicBool>
}

impl Default for Motorsim {
    fn default() -> Self {
        Self {
            controller: Arc::new(Mutex::new(Controller::new(Motor::new(0.00065, 0.000024 , 0.00073, 0.7, 0.057, [0.0, 0.0]), 1.0))),
            endstate: Arc::new(AtomicBool::new(false)),
            startstate: Arc::new(AtomicBool::new(false))
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
            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_trq()), ui, "Torque");
            Motorsim::plot(PlotPoints::from(self.controller.lock().unwrap().get_voltage()), ui, "Voltage");
        });

        egui::SidePanel::left("left").show(ctx, |ui|{
            ui.vertical_centered(|ui|{
                ui.group(|ui|{
                    ui.label("Control type :");
                    ui.horizontal(|ui| {
                        ui.selectable_value(self.controller.lock().unwrap().get_control_option(), ControlType::Pos, "Pos");
                        ui.selectable_value(self.controller.lock().unwrap().get_control_option(), ControlType::PosVelTrq, "PosVelTrq");
                    });
                });
            });

            ui.vertical_centered(|ui|{
                ui.label("Angle controller");
            });
    
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("Kp :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_pos_pid().get_kp()).speed(0.05));
                    ui.label("Kd :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_pos_pid().get_kd()).speed(0.05));
                    ui.label("Ki :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_pos_pid().get_ki()).speed(0.05));
    
                    if ui.add(egui::Button::new("Calibrate")).clicked() {
                        if *(self.controller.lock().unwrap().get_control_option()) == ControlType::Pos{
                            *(self.controller.lock().unwrap().get_calib_option()) = Some(CalibType::Pos);                   
                        }
                        else{
                            *(self.controller.lock().unwrap().get_calib_option()) = Some(CalibType::PosVelTrq);
                        }
                        self.controller.lock().unwrap().reset();
                        self.startstate.store(true, Ordering::Relaxed);
                    }
                })
            });

            ui.vertical_centered(|ui|{
                ui.label("Speed controller");
            });
    
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("Kp :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_vel_pid().get_kp()).speed(0.05));
                    ui.label("Kd :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_vel_pid().get_kd()).speed(0.05));
                    ui.label("Ki :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_vel_pid().get_ki()).speed(0.05));
    
                    if ui.add(egui::Button::new("Calibrate")).clicked() {
                        self.controller.lock().unwrap().reset();
                        *(self.controller.lock().unwrap().get_calib_option()) = Some(CalibType::VelTrq);                        self.controller.lock().unwrap().reset();
                        self.startstate.store(true, Ordering::Relaxed);
                    }
                })
            });

            ui.vertical_centered(|ui|{
                ui.label("Torque controller");
            });
    
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("Kp :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_trq_pid().get_kp()).speed(0.05));
                    ui.label("Kd :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_trq_pid().get_kd()).speed(0.05));
                    ui.label("Ki :");
                    ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_trq_pid().get_ki()).speed(0.05));
    
                    if ui.add(egui::Button::new("Calibrate")).clicked() {
                        self.controller.lock().unwrap().reset();
                        *(self.controller.lock().unwrap().get_calib_option()) = Some(CalibType::Trq);
                        self.startstate.store(true, Ordering::Relaxed);
                    }
                })
            });

            ui.vertical_centered(|ui|{
                ui.label("Motor parameters");
                ui.group(|ui|{
                    ui.horizontal(|ui| {
                        ui.label("j :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_motor().get_j()).speed(0.05).max_decimals(6));
                        ui.label("b :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_motor().get_b()).speed(0.05).max_decimals(6));
                        ui.label("k :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_motor().get_k()).speed(0.05).max_decimals(6));
                        ui.label("r :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_motor().get_r()).speed(0.05).max_decimals(6));
                        ui.label("l :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_motor().get_l()).speed(0.05).max_decimals(6));
                    });
                });
            });
    
            ui.vertical_centered(|ui|{
                ui.label("Motor bounds");
                ui.group(|ui|{
                    ui.horizontal(|ui| {
                        ui.label("Vltg bound :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_vltg_bound()).speed(0.05).max_decimals(6));
                        ui.label("Vel bound :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_vel_bound()).speed(0.05).max_decimals(6));
                        ui.label("Trq bound :");
                        ui.add(egui::DragValue::new(self.controller.lock().unwrap().get_trq_bound()).speed(0.05).max_decimals(6));
                    });
                });  
            });
              

            ui.add(egui::Slider::new(self.controller.lock().unwrap().get_target(), 0.0..=360.0).text("Pos target"));

            if ui.add(egui::Button::new("Start")).clicked() {
                self.controller.lock().unwrap().reset();
                *(self.controller.lock().unwrap().get_calib_option()) = None;
                self.startstate.store(true, Ordering::Relaxed);
            }

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

    pub fn get_controller(&self) -> Arc<Mutex<Controller>>{
        Arc::clone(&self.controller)
    }

    pub fn get_endstate(&self) -> Arc<AtomicBool>{
        Arc::clone(&self.endstate)
    }

    pub fn get_startstate(&self) -> Arc<AtomicBool>{
        Arc::clone(&self.startstate)
    }

}