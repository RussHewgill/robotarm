use k::prelude::*;
use nalgebra as na;

pub struct Arm {
    base_motor_id: u8,
    shoulder_motor_id: u8,
    elbow_motor_id: u8,
    wrist_motor_id: u8,

    kinematics: k::SerialChain<f64>,
}

impl Arm {
    pub fn new(
        base_motor_id: u8,
        shoulder_motor_id: u8,
        elbow_motor_id: u8,
        wrist_motor_id: u8,
    ) -> Self {
        let base = k::Node::new(
            k::NodeBuilder::new()
                .name("Base")
                .translation(na::Translation3::new(0.0, 0.0, 0.0))
                .joint_type(k::JointType::Rotational {
                    axis: na::Vector3::z_axis(),
                })
                .finalize(),
        );

        let shoulder = k::Node::new(
            k::NodeBuilder::new()
                .name("Shoulder")
                .translation(na::Translation3::new(0.0, 0.0, 45.0))
                .joint_type(k::JointType::Rotational {
                    axis: na::Vector3::x_axis(),
                })
                .finalize(),
        );

        let elbow = k::Node::new(
            k::NodeBuilder::new()
                .name("Elbow")
                .translation(na::Translation3::new(0.0, 0.0, 218.5))
                .joint_type(k::JointType::Rotational {
                    axis: na::Vector3::x_axis(),
                })
                .finalize(),
        );

        shoulder.set_parent(&base);
        elbow.set_parent(&shoulder);

        let tree = k::Chain::from_root(base);
        let kinematics = k::SerialChain::try_new(tree).unwrap();

        Self {
            base_motor_id,
            shoulder_motor_id,
            elbow_motor_id,
            wrist_motor_id,

            kinematics,
        }
    }
}
