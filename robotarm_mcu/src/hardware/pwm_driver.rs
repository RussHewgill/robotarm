use defmt::debug;
use embassy_rp::pwm::SetDutyCycle;

pub struct PWMDriver<'a> {
    pwm0: embassy_rp::pwm::Pwm<'a>,
    pwm1: embassy_rp::pwm::Pwm<'a>,
    pwm2: embassy_rp::pwm::Pwm<'a>,
}

impl<'a> PWMDriver<'a> {
    pub fn new(
        pwm0: embassy_rp::pwm::Pwm<'a>,
        pwm1: embassy_rp::pwm::Pwm<'a>,
        pwm2: embassy_rp::pwm::Pwm<'a>,
    ) -> Self {
        Self { pwm0, pwm1, pwm2 }
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
}
