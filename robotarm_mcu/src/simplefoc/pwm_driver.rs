use defmt::{debug, info};
use embassy_rp::pwm::SetDutyCycle;

pub struct PWMDriver<'a> {
    pwm0: embassy_rp::pwm::Pwm<'a>,
    pwm1: embassy_rp::pwm::PwmOutput<'a>,
    pwm2: embassy_rp::pwm::PwmOutput<'a>,

    pub voltage_limit: f32,
    pub voltage_supply: f32,

    max_duty_cycle: u16,
}

impl<'a> PWMDriver<'a> {
    pub fn new(
        pwm0: embassy_rp::pwm::Pwm<'a>,
        pwm12: embassy_rp::pwm::Pwm<'a>,
        // pwm2: embassy_rp::pwm::Pwm<'a>,
        voltage_limit: f32,
        voltage_supply: f32,
    ) -> Self {
        let max_duty_cycle = pwm0.max_duty_cycle();

        match pwm12.split() {
            (Some(pwm1), Some(pwm2)) => Self {
                pwm0,
                pwm1,
                pwm2,
                voltage_limit,
                voltage_supply,
                max_duty_cycle,
            },
            _ => panic!("Failed to split PWM slice into two channels"),
        }
    }

    pub fn enable(&mut self) {
        unimplemented!()
    }

    pub fn disable(&mut self) {
        debug!("Disabling PWM Driver");
        let _ = self.pwm0.set_duty_cycle_fully_off();
        let _ = self.pwm1.set_duty_cycle_fully_off();
        let _ = self.pwm2.set_duty_cycle_fully_off();
    }

    pub fn set_duty_cycles(&mut self, dc0: u16, dc1: u16, dc2: u16) {
        let _ = self.pwm0.set_duty_cycle(dc0);
        let _ = self.pwm1.set_duty_cycle(dc1);
        let _ = self.pwm2.set_duty_cycle(dc2);
    }

    pub fn set_duty_cycles_f32(&mut self, dc0: f32, dc1: f32, dc2: f32) {
        info!("Setting duty cycles: {}, {}, {}", dc0, dc1, dc2);

        let va = dc0.clamp(0., self.voltage_limit);
        let vb = dc1.clamp(0., self.voltage_limit);
        let vc = dc2.clamp(0., self.voltage_limit);

        let dc_a = (va / self.voltage_supply).clamp(0., 1.);
        let dc_b = (vb / self.voltage_supply).clamp(0., 1.);
        let dc_c = (vc / self.voltage_supply).clamp(0., 1.);

        if let Err(_e) = self
            .pwm0
            .set_duty_cycle((dc_a * self.max_duty_cycle as f32) as u16)
        {
            debug!("Failed to set duty cycle for PWM0");
        }
        if let Err(_e) = self
            .pwm1
            .set_duty_cycle((dc_b * self.max_duty_cycle as f32) as u16)
        {
            debug!("Failed to set duty cycle for PWM1");
        }
        if let Err(_e) = self
            .pwm2
            .set_duty_cycle((dc_c * self.max_duty_cycle as f32) as u16)
        {
            debug!("Failed to set duty cycle for PWM2");
        }
    }
}
