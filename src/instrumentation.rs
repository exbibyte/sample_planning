use std::time::{Duration, Instant};

pub struct Timer {
    t: Instant,
}

impl Default for Timer {
    fn default() -> Timer {
        Self {
            t: Instant::now(),
        }
    }
}

impl Timer {

    pub fn reset( & mut self ){
        self.t = Instant::now();
    }

    pub fn dur_ms( & mut self ) -> f64 {
        let t_milli = self.t.elapsed().as_nanos() as f64 / 1e6;
        t_milli
    }
}
