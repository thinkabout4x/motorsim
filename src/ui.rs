use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc::Sender;

use eframe::egui::plot::Legend;
use eframe::egui::plot::PlotUi;
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

        egui::CentralPanel::default().show(&ctx, |ui| {
            let width = ui.available_width();
            ui.columns(4, |uis| { //4 colums hack to make custom layout
                {
                let left = &mut uis[0];
                left.vertical(|left|{
                    left.group(|left|{
                        left.label("Control type :");
                        left.horizontal(|left| {
                            left.selectable_value(self.config.set_controller_conf().set_control_option(), ControlType::Pos, "Pos");
                            left.selectable_value(self.config.set_controller_conf().set_control_option(), ControlType::PosVelTrq, "PosVelTrq");
                        });
                    });
                    if Motorsim::pid_ui(&mut self.config, ["Angle controller", "Speed controller", "Torque controller"] , left){
                        *(self.config.set_controller_conf().set_start_flag()) = true;
                        self.transmitter.send(self.config).unwrap();
                    }

                    Motorsim::motor_params_ui(self.config.set_motor_conf(), left);
                    Motorsim::bounds_ui(self.config.set_controller_conf(), left);
                    left.add(egui::Slider::new(&mut *(self.target.lock().unwrap()), 0.0..=360.0).text("Pos target"));

                    left.group(|left|{
                        left.horizontal(|left| {
                            if left.add(egui::Button::new("Start")).clicked() {
                                *(self.config.set_controller_conf().set_calib_option()) = None;
                                *(self.config.set_controller_conf().set_start_flag()) = true;
                                self.transmitter.send(self.config).unwrap();
                            }
            
                            if left.add(egui::Button::new("Stop")).clicked() {
                                *(self.config.set_controller_conf().set_start_flag()) = false;
                                self.transmitter.send(self.config).unwrap();
                            }

                            left.label("Duration, sec :");
                            left.add(egui::DragValue::new(self.config.set_controller_conf().set_duration()).speed(0.05));

                            left.label("Frequency, hz :");
                            left.add(egui::DragValue::new(self.config.set_controller_conf().set_frequency()).speed(0.05));

                        });
                    });
                });
                }
                let right = &mut uis[1];
                right.set_width(width*3./4.);
                right.group(|right|{
                    right.vertical(|right| {
                        right.label("Plots");
                        let mut plotpoints = self.plotpoints.lock().unwrap();
                        Motorsim::plot(&mut plotpoints, &self.config, right);
                        ctx.request_repaint();
                    });
                });
            });
        });
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
        //tracing_subscriber::fmt::init();

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
        let height = ui.available_height()/4.;

        let mut pos_target = Line::new(PlotPoints::from(vec![]));
        let mut vel_target = Line::new(PlotPoints::from(vec![]));
        let mut trq_target = Line::new(PlotPoints::from(vec![]));

        let pos_plot = Plot::new("Angle").height(height).include_y(0.0).legend(Legend::default());
        let vel_plot = Plot::new("Speed").height(height).include_y(0.0).legend(Legend::default());
        let trq_plot = Plot::new("Torque").height(height).include_y(0.0).legend(Legend::default());
        let voltage_plot = Plot::new("Voltage").height(height).include_y(0.0).legend(Legend::default());

        let pos_line = Line::new(PlotPoints::from(points.clone_pos_as_vec())).name("Angle, deg");
        let vel_line = Line::new(PlotPoints::from(points.clone_vel_as_vec())).name("Speed, rpm");
        let trq_line = Line::new(PlotPoints::from(points.clone_trq_as_vec())).name("Torque, N*m");
        let vltg_line = Line::new(PlotPoints::from(points.clone_voltage_as_vec())).name("Voltage, V");

        match config.get_controller_conf().get_calib_option(){
            Some(pid_type) =>{
                
                for pid in config.get_pid_conf(){
                    if pid.get_option() == *pid_type{
                        match pid.get_option(){
                            TypePid::Pos =>{
                                pos_target = Line::new(PlotPoints::from(vec![[0.0, 180.0],[config.get_controller_conf().get_duration(), 180.0]]));
                            },
                            TypePid::Vel =>{
                                let value = config.get_controller_conf().get_vel_bound()/2.0;
                                vel_target = Line::new(PlotPoints::from(vec![[0.0, value],[config.get_controller_conf().get_duration(), value]]));
                            },
                            TypePid::Trq =>{
                                let value = config.get_controller_conf().get_trq_bound()/2.0;
                                trq_target = Line::new(PlotPoints::from(vec![[0.0, value],[config.get_controller_conf().get_duration(), value]]));
                            }
                        }
                    }
                }
            }
            None => { }
        }
        pos_plot.show(ui, |plot_ui: &mut PlotUi| {plot_ui.line(pos_line); plot_ui.line(pos_target)});
        vel_plot.show(ui, |plot_ui: &mut PlotUi| {plot_ui.line(vel_line); plot_ui.line(vel_target)});
        trq_plot.show(ui, |plot_ui: &mut PlotUi| {plot_ui.line(trq_line); plot_ui.line(trq_target)});
        voltage_plot.show(ui, |plot_ui: &mut PlotUi| {plot_ui.line(vltg_line);});
    }

    fn pid_ui(config: &mut Config, label:[&str;3],  ui: &mut Ui) -> bool{
        let mut send_flag = false;
        let mut calib_option = None;
        let control_option = config.get_controller_conf().get_control_option().clone();
        for (i, pid) in config.set_pid_conf().iter_mut().enumerate() {
            ui.vertical(|ui|{
                ui.label(label[i]);
            });
            ui.group(|ui|{
            ui.horizontal(|ui| {
                ui.label("Kp :");
                ui.add(egui::DragValue::new(pid.set_kp()).speed(0.05));
                ui.label("Kd :");
                ui.add(egui::DragValue::new(pid.set_kd()).speed(0.05));
                ui.label("Ki :");
                ui.add(egui::DragValue::new(pid.set_ki()).speed(0.05));

                match control_option{
                    ControlType::Pos =>{
                        match pid.get_option(){
                            TypePid::Pos =>{ }
                            TypePid::Vel =>{
                                ui.set_enabled(false);
                            }
                            TypePid::Trq =>{
                                ui.set_enabled(false);
                            }

                        }
                    }
                    ControlType::PosVelTrq =>{ }
                }
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
        ui.vertical(|ui|{
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
        ui.vertical(|ui|{
            ui.label("Motor bounds");
            ui.group(|ui|{
                ui.horizontal(|ui| {
                    ui.label("Vltg bound, V :");
                    ui.add(egui::DragValue::new(controller_conf.set_vltg_bound()).speed(0.05).max_decimals(6));
                    ui.label("Vel bound, rpm :");
                    ui.add(egui::DragValue::new(controller_conf.set_vel_bound()).speed(0.05).max_decimals(6));
                    ui.label("Trq bound, N*m :");
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