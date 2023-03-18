use std::{time::{Instant, Duration}};

pub struct Time{
    prev_state: Option<Duration>, 
    state: Duration,
    zero_time:Instant

}

impl Time {
    pub fn new() -> Self{
        let a = Instant::now();
        let b = a.elapsed();
        Self {zero_time: a, prev_state: None, state: b}
    }

    pub fn get_delta(&mut self) -> f64{
        self.prev_state = Some(self.state);
        self.state = self.zero_time.elapsed();
        (self.state-self.prev_state.unwrap()).as_secs_f64()
    }
}