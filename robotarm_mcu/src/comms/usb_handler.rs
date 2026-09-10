use defmt::{debug, error, info};

use embassy_usb::{
    Handler,
    control::{InResponse, OutResponse, Request},
    types::InterfaceNumber,
};

pub struct ControlHandler {
    pub if_num: InterfaceNumber,
}

impl Handler for ControlHandler {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        unimplemented!()
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        unimplemented!()
    }
}
