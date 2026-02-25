use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotionControlType {
    Torque,
    Velocity,
    Angle,
    VelocityOpenLoop,
}

impl MotionControlType {
    pub fn is_open_loop(&self) -> bool {
        matches!(self, MotionControlType::VelocityOpenLoop)
    }
}
