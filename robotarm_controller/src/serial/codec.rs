use anyhow::{Context, Result, anyhow, bail, ensure};
use bytes::Buf;
use tracing::{debug, error, info, trace, warn};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

#[derive(Debug, Clone)]
pub struct SerialCodec {
    buf: [u8; 16384],
}

impl Default for SerialCodec {
    fn default() -> Self {
        Self { buf: [0; 16384] }
    }
}

impl tokio_util::codec::Decoder for SerialCodec {
    type Item = SerialLogMessage;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let n = src.len();
        // let mut src2 = src.clone();
        self.buf[..n].copy_from_slice(src.as_ref());
        match postcard::take_from_bytes_cobs(self.buf.as_mut()) {
            Ok((msg, rest)) => {
                let len = rest.len();
                if len > 0 {
                    // src.advance(n - rest.len());
                    src.clear();
                    // src.extend_from_slice(&self.buf[..len]);
                } else {
                    src.clear();
                }
                // let n2 = n - rest.len();
                // src.advance(n2);
                // debug!("Decoded message: {:?}", msg);
                Ok(Some(msg))
            }
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                // debug!("Incomplete message received, waiting for more data");
                // src.clear(); // Clear the buffer to avoid processing incomplete data
                Ok(None)
            }
            Err(e) => {
                // debug!("Deserialization error: {e}");
                // src.clear();
                Err(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("Deserialization error: {e}"),
                ))
            }
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
        item: SerialCommand,
        dst: &mut bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        match postcard::to_stdvec_cobs(&item) {
            Ok(buf) => {
                dst.extend_from_slice(&buf);
                Ok(())
            }
            Err(e) => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Serialization error: {e}"),
            )),
        }
    }
}
