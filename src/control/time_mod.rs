use std::{time::{Instant, Duration}};

pub struct Time{
    prev_state: Option<Duration>, 
    state: Duration,
    zero_time: Duration,
    instant:Instant,
    time_period: f64

}

impl Time {
    pub fn new(frquency: f64) -> Self{
        let instant = Instant::now();
        let zero_time = instant.elapsed();
        let state = zero_time;
        Self {zero_time, prev_state: None, state, instant, time_period: 1./frquency}
    }

    pub fn update_state(&mut self){
        self.prev_state = Some(self.state);
        self.state = self.instant.elapsed();
        while (self.state-self.prev_state.unwrap()).as_secs_f64() < self.time_period{
            self.state = self.instant.elapsed();
        }
        self.state = self.instant.elapsed();
    }

    pub fn get_delta(&self) -> f64{
        (self.state-self.prev_state.unwrap()).as_secs_f64()
    }

    pub fn get_time_from_start(&self) -> f64{
        (self.state-self.zero_time).as_secs_f64()
    }
}