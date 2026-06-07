//! 固定周期 PID，包含积分限幅和输出限幅。

#[derive(Clone, Copy, Debug)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    output_limit: f32,
    integral: f32,
    previous_error: f32,
    initialized: bool,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            integral: 0.0,
            previous_error: 0.0,
            initialized: false,
        }
    }

    pub fn step(&mut self, setpoint: f32, feedback: f32, dt: f32) -> f32 {
        let error = setpoint - feedback;
        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        );
        let derivative = if self.initialized {
            (error - self.previous_error) / dt
        } else {
            self.initialized = true;
            0.0
        };
        self.previous_error = error;
        clamp(
            self.kp * error + self.ki * self.integral + self.kd * derivative,
            -self.output_limit,
            self.output_limit,
        )
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.initialized = false;
    }
}

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}
