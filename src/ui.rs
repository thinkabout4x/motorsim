use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc::Sender;

use eframe::egui::{self,Ui};
use egui::plot::{Line, Plot, PlotPoints};
use crate::control::Config;
use crate::control::ConfigController;
use crate::control::ControlType;
use crate::control::PlotPnts;
use crate::control::TypePid;
use crate::control::motor::ConfigMotor;

pub struct Motorsim{
    config: Config,
    target: Arc<Mutex<f64>>,
    plotpoints: Arc<Mutex<PlotPnts>>,
    transmitter: Sender<Config>
}

impl eframe::App for Motorsim {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame){
        egui::SidePanel::right("right").show(ctx, |ui|{
            
            ui.vertical_centered(|ui|{
                ui.label("Plots");
            });

            let mut plotpoints = self.plotpoints.lock().unwrap();
            Motorsim::plot(&mut plotpoints, &self.config, ui);
        });

        egui::SidePanel::left("left").show(ctx, |ui|{
            ui.vertical_centered(|ui|{
                ui.group(|ui|{
                    ui.label("Control type :");
                    ui.horizontal(|ui| {
                        ui.selectable_value(self.config.set_controller_conf().set_control_option(), ControlType::Pos, "Pos");
                        ui.selectable_value(self.config.set_controller_conf().set_control_option(), ControlType::PosVelTrq, "PosVelTrq");
                    });
                });
            });

            if Motorsim::pid_ui(&mut self.config, ["Angle controller", "Speed controller", "Torque controller"] , ui){
                *(self.config.set_controller_conf().set_start_flag()) = true;
                self.transmitter.send(self.config).unwrap();
            }

            Motorsim::motor_params_ui(self.config.set_motor_conf(), ui);

            Motorsim::bounds_ui(self.config.set_controller_conf(), ui);
              
            ui.add(egui::Slider::new(&mut *(self.target.lock().unwrap()), 0.0..=360.0).text("Pos target"));

            ui.vertical_centered(|ui|{
                ui.group(|ui|{
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("Start")).clicked() {
                            *(self.config.set_controller_conf().set_calib_option()) = None;
                            *(self.config.set_controller_conf().set_start_flag()) = true;
                            self.transmitter.send(self.config).unwrap();
                        }
            
                        if ui.add(egui::Button::new("Stop")).clicked() {
                            *(self.config.set_controller_conf().set_start_flag()) = false;
                            self.transmitter.send(self.config).unwrap();
                        }
                    });
                });
            });

        });

        ctx.request_repaint();

    }

    fn on_close_event(&mut self) -> bool {
        *(self.config.set_controller_conf().set_end_flag()) = true;
        self.transmitter.send(self.config).unwrap();
        true
    }
}

impl Motorsim {
    pub fn new(tx: Sender<Config>) -> Self{
        let config = Config::default();
        tx.send(config).unwrap();
        Self {
            config: config,
            target: Arc::new(Mutex::new(180.0)),
            plotpoints: Arc::new(Mutex::new(PlotPnts::default())),
            transmitter: tx
        }
    }

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

    fn plot(points: &mut PlotPnts, config: &Config, ui: &mut Ui) {
        match config.get_controller_conf().get_calib_option(){
            Some(pid_type) =>{
                for pid in config.get_pid_conf(){
                    if pid.get_option() == *pid_type{
                        match pid.get_option(){
                            TypePid::Pos =>{
                                let line = Line::new(PlotPoints::from(points.clone_pos()));
                                let target_line = Line::new(PlotPoints::from(vec![[0.0, 180.0],[config.get_controller_conf().get_duration(), 180.0]]));
                                Plot::new("Angle").view_aspect(3.0).width(600.0).show(ui, |plot_ui| {plot_ui.line(line); plot_ui.line(target_line)});
                            },
                            TypePid::Vel =>{
                                let line = Line::new(PlotPoints::from(points.clone_vel()));
                                let value = config.get_controller_conf().get_vel_bound()/2.0;
                                let target_line = Line::new(PlotPoints::from(vec![[0.0, value],[config.get_controller_conf().get_duration(), value]]));
                                Plot::new("Speed").view_aspect(3.0).width(600.0).show(ui, |plot_ui| {plot_ui.line(line); plot_ui.line(target_line)});
                            },
                            TypePid::Trq =>{
                                let line = Line::new(PlotPoints::from(points.clone_trq()));
                                let value = config.get_controller_conf().get_trq_bound()/2.0;
                                let target_line = Line::new(PlotPoints::from(vec![[0.0, value],[config.get_controller_conf().get_duration(), value]]));
                                Plot::new("Torque").view_aspect(3.0).width(600.0).show(ui, |plot_ui| {plot_ui.line(line); plot_ui.line(target_line)});
                            }
                        }
                    } else{
                        match pid.get_option(){
                            TypePid::Pos =>{
                                let line = Line::new(PlotPoints::from(points.clone_pos()));
                                Plot::new("Angle").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
                            },
                            TypePid::Vel =>{
                                let line = Line::new(PlotPoints::from(points.clone_vel()));
                                Plot::new("Speed").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
                            },
                            TypePid::Trq =>{
                                let line = Line::new(PlotPoints::from(points.clone_trq()));
                                Plot::new("Torque").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
                            }
                        }
                    }
                }
            }
            None =>{
                let line = Line::new(PlotPoints::from(points.clone_pos()));
                Plot::new("Angle").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
                let line = Line::new(PlotPoints::from(points.clone_vel()));
                Plot::new("Speed").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
                let line = Line::new(PlotPoints::from(points.clone_trq()));
                Plot::new("Torque").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
            }
        }
        let line = Line::new(PlotPoints::from(points.clone_voltage()));
        Plot::new("Voltage").view_aspect(3.0).width(600.0).show(ui, |plot_ui| plot_ui.line(line));
    }

    fn pid_ui(config: &mut Config, label:[&str;3],  ui: &mut Ui) -> bool{
        let mut send_flag = false;
        let mut calib_option = None;
        for pid in config.set_pid_conf() {
            ui.vertical_centered(|ui|{
                ui.label(label[0]);
            });

            ui.group(|ui|{
            ui.horizontal(|ui| {
                ui.label("Kp :");
                ui.add(egui::DragValue::new(pid.set_kp()).speed(0.05));
                ui.label("Kd :");
                ui.add(egui::DragValue::new(pid.set_kd()).speed(0.05));
                ui.label("Ki :");
                ui.add(egui::DragValue::new(pid.set_ki()).speed(0.05));

                if ui.add(egui::Button::new("Calibrate")).clicked() {
                    calib_option = Some(pid.get_option());
                    send_flag = true;
                }
            })
        });
        }
        if send_flag{
            *(config.set_controller_conf().set_calib_option()) = calib_option;
        }
    send_flag
    }

    fn motor_params_ui(motor_conf: &mut ConfigMotor, ui: &mut Ui){
        ui.vertical_centered(|ui|{
            ui.label("Motor parameters");
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("j :");
                    ui.add(egui::DragValue::new(motor_conf.set_j()).speed(0.05).max_decimals(6));
                    ui.label("b :");
                    ui.add(egui::DragValue::new(motor_conf.set_b()).speed(0.05).max_decimals(6));
                    ui.label("k :");
                    ui.add(egui::DragValue::new(motor_conf.set_k()).speed(0.05).max_decimals(6));
                    ui.label("r :");
                    ui.add(egui::DragValue::new(motor_conf.set_r()).speed(0.05).max_decimals(6));
                    ui.label("l :");
                    ui.add(egui::DragValue::new(motor_conf.set_l()).speed(0.05).max_decimals(6));
                });
            });
        });
    }

    fn bounds_ui(controller_conf: &mut ConfigController, ui: &mut Ui){
        ui.vertical_centered(|ui|{
            ui.label("Motor bounds");
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("Vltg bound :");
                    ui.add(egui::DragValue::new(controller_conf.set_vltg_bound()).speed(0.05).max_decimals(6));
                    ui.label("Vel bound :");
                    ui.add(egui::DragValue::new(controller_conf.set_vel_bound()).speed(0.05).max_decimals(6));
                    ui.label("Trq bound :");
                    ui.add(egui::DragValue::new(controller_conf.set_trq_bound()).speed(0.05).max_decimals(6));
                });
            });  
        });
    }

    pub fn get_plotpoints(&self) -> Arc<Mutex<PlotPnts>>{
        Arc::clone(&self.plotpoints)
    }

    pub fn get_target(&self) -> Arc<Mutex<f64>>{
        Arc::clone(&self.target)
    }

}