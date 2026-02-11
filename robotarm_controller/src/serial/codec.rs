use anyhow::{Context, Result, anyhow, bail, ensure};
use bytes::Buf;
use tracing::{debug, error, info, trace, warn};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

#[derive(Debug, Clone)]
pub struct SerialCodec;

impl tokio_util::codec::Decoder for SerialCodec {
    type Item = SerialLogMessage;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let n = src.len();
        match postcard::take_from_bytes_cobs(src) {
            Ok((msg, rest)) => {
                let n = n - rest.len();
                src.advance(n);
                // debug!("n0 = {}, n1 = {}, consumed = {}", n0, n1, n0 - n1);
                Ok(Some(msg))
            }
            Err(postcard::Error::DeserializeUnexpectedEnd) => Ok(None),
            Err(e) => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Deserialization error: {e}"),
            )),
        }
    }

    // fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
    //     let newline = src.as_ref().iter().position(|b| *b == b'\n');
    //     if let Some(n) = newline {
    //         let line = src.split_to(n + 1);
    //         return match str::from_utf8(line.as_ref()) {
    //             Ok(s) => Ok(Some(s.to_string())),
    //             Err(_) => Err(std::io::Error::new(
    //                 std::io::ErrorKind::Other,
    //                 "Invalid String",
    //             )),
    //         };
    //     }
    //     Ok(None)
    // }
}

impl tokio_util::codec::Encoder<SerialCommand> for SerialCodec {
    type Error = std::io::Error;

    fn encode(
        &mut self,
        _item: SerialCommand,
        _dst: &mut bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}
