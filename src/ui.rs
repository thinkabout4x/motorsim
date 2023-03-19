use eframe::egui::{self, InnerResponse, Ui};
use egui::plot::{Line, Plot, PlotPoints};
use crate::control::Motor;
use crate::control::Pid;
use crate::control::Contoller;


pub struct Motorsim{
    angle_pid: Pid,
    speed_pid: Pid,
    torque_pid: Pid,
}

impl Default for Motorsim {
    fn default() -> Self {
        Self {
            angle_pid : Pid::new(0,0,0),
            speed_pid : Pid::new(0,0,0),
            torque_pid : Pid::new(0,0,0),
        }
    }
}

impl eframe::App for Motorsim {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame){
        egui::SidePanel::right("right").show(ctx, |ui|{
            
            ui.vertical_centered(|ui|{
                ui.label("Plots");
            });
            
            //self.plot(ui, "Angle");
            self.plot(ui, "Speed");
            //self.plot(ui, "Torque");


        });

        egui::SidePanel::left("left").show(ctx, |ui|{
            Motorsim::controller(ui, "Angle controller", &mut self.angle_pid);
            Motorsim::controller(ui, "Speed controller", &mut self.speed_pid);
            Motorsim::controller(ui, "Torque controller", &mut self.torque_pid);

        });

    }
}

impl Motorsim {
    pub fn run(&self, width: i32, height: i32 ) -> Result<(), eframe::Error> {
        tracing_subscriber::fmt::init();

        let options = eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(width as f32, height as f32)),
            ..Default::default()
        };
        eframe::run_native(
            "Motor control sim",
            options,
            Box::new(|_cc| Box::new(Motorsim::default())),
        )
    }

    fn plot(&mut self, ui: &mut Ui, id: &str ) -> InnerResponse<()> {
        ui.label(id);
        // let sin: PlotPoints = (0..1000).map(|i| {
        //     let x = i as f64 * 0.01;
        //     [x, x.sin()]
        // }).collect();
        let mut controller = Contoller::new(Motor::new(0.01, 0.1, 0.5, 1.0, 0.01, [0.0, 0.0]));
        let sin = controller.test();
        let line = Line::new(sin);
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
}