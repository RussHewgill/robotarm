use anyhow::{Context, Result, anyhow, bail, ensure};
use bytes::Buf;
use postcard::accumulator::FeedResult;
use tracing::{debug, error, info, trace, warn};

use robotarm_protocol::{SerialCommand, SerialLogMessage};

// #[derive(Debug)]
pub struct SerialCodec {
    // buf: [u8; 16384],
    accum: postcard::accumulator::CobsAccumulator<4096>,
}

impl Default for SerialCodec {
    fn default() -> Self {
        Self {
            accum: postcard::accumulator::CobsAccumulator::new(),
        }
    }
}

impl tokio_util::codec::Decoder for SerialCodec {
    type Item = SerialLogMessage;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if src.is_empty() {
            return Ok(None);
        }

        let len = src.len();

        loop {
            match self.accum.feed::<SerialLogMessage>(&mut src[..]) {
                FeedResult::Success { data, remaining } => {
                    // advance src by the number of bytes consumed and return the deserialized message
                    let consumed = len - remaining.len();
                    src.advance(consumed);
                    return Ok(Some(data));
                }
                FeedResult::Consumed => return Ok(None),
                FeedResult::OverFull(w) => {
                    error!("Accumulator overflow");
                    panic!("Accumulator overflow");
                }
                FeedResult::DeserError(w) => {
                    // error!("Deserialization error: {len}, {}", w.len());
                    let new_len = w.len();
                    src.advance(len - new_len);
                    // skip the current message by advancing src until the next 0x00 byte
                    return Ok(None);
                }
            }
        }
    }
}

#[cfg(feature = "nope")]
impl tokio_util::codec::Decoder for SerialCodec {
    type Item = SerialLogMessage;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let n = src.len();
        if n == 0 {
            return Ok(None);
        }

        if n > self.buf.len() {
            if let Some(pos) = src.as_ref().iter().position(|&b| b == 0x00) {
                src.advance(pos + 1);
                return Ok(None);
            } else {
                src.clear();
                return Ok(None);
            }
        }

        self.buf[..n].copy_from_slice(src.as_ref());
        match postcard::take_from_bytes_cobs(&mut self.buf[..n]) {
            Ok((msg, rest)) => {
                let len = rest.len();
                if len > 0 {
                    src.advance(n - len);
                } else {
                    src.clear();
                }
                Ok(Some(msg))
            }
            Err(postcard::Error::DeserializeUnexpectedEnd) => Ok(None),
            Err(_e) => {
                if let Some(pos) = src.as_ref().iter().position(|&b| b == 0x00) {
                    src.advance(pos + 1);
                } else {
                    src.clear();
                }
                Ok(None)
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
