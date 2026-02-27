use anyhow::{Context, Result, anyhow, bail, ensure};
use robotarm_protocol::types::MotionControlType;
use tracing::{debug, error, info, trace, warn};

#[derive(Debug, Clone, Default)]
pub struct FocStatus {
    pub id: u8,

    pub target_pos: f64,
    pub target_vel: f64,
    pub target_voltage: f64,

    pub motion_control: Option<MotionControlType>,

    pub gear_ratio: f64,

    pub feed_forward: f64,

    pub current: f32,
    pub voltage: (f32, f32),

    pub pos: f64,
    pub angle: f64,
    pub vel: f64,

    pub vel_pid_p: f32,
    pub vel_pid_i: f32,
    pub vel_pid_d: f32,
    pub vel_pid_ramp: f32,
    pub vel_pid_limit: f64,

    pub pos_pid_p: f32,
    pub pos_pid_i: f32,
    pub pos_pid_d: f32,
    pub pos_pid_ramp: f32,
    pub pos_pid_limit: f32,

    pub lpf_angle: f32,
    pub lpf_vel: f32,
}
