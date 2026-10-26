#![cfg_attr(not(test), no_std)]

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    BufferTooSmall,
    TooLong,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ConnackError {
    Malformed,
    Refused(u8),
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let end = self.pos + data.len();
        self.buf
            .get_mut(self.pos..end)
            .ok_or(Error::BufferTooSmall)?
            .copy_from_slice(data);
        self.pos = end;
        Ok(())
    }
    fn u8(&mut self, v: u8) -> Result<(), Error> {
        self.bytes(&[v])
    }
    fn u16(&mut self, v: u16) -> Result<(), Error> {
        self.bytes(&v.to_be_bytes())
    }
    fn str16(&mut self, s: &str) -> Result<(), Error> {
        let len = u16::try_from(s.len()).map_err(|_| Error::TooLong)?;
        self.u16(len)?;
        self.bytes(s.as_bytes())
    }
    /// MQTT "remaining length": 7 bits per byte, high bit = more bytes follow.
    fn remaining_len(&mut self, mut len: usize) -> Result<(), Error> {
        if len > 268_435_455 {
            return Err(Error::TooLong);
        }
        loop {
            let mut byte = (len % 128) as u8;
            len /= 128;
            if len > 0 {
                byte |= 0x80;
            }
            self.u8(byte)?;
            if len == 0 {
                return Ok(());
            }
        }
    }
}

/// CONNECT with clean session. Returns the number of bytes written.
pub fn connect(buf: &mut [u8], client_id: &str, keep_alive_s: u16) -> Result<usize, Error> {
    let mut w = Writer::new(buf);
    w.u8(0x10)?;
    w.remaining_len(10 + 2 + client_id.len())?;
    w.bytes(b"\x00\x04MQTT")?; // protocol name
    w.u8(4)?; // protocol level 4 = MQTT 3.1.1
    w.u8(0x02)?; // flags: clean session
    w.u16(keep_alive_s)?;
    w.str16(client_id)?;
    Ok(w.pos)
}

/// PUBLISH at QoS 0 (no packet id, no acknowledgement).
pub fn publish(buf: &mut [u8], topic: &str, payload: &[u8], retain: bool) -> Result<usize, Error> {
    let mut w = Writer::new(buf);
    w.u8(0x30 | retain as u8)?;
    w.remaining_len(2 + topic.len() + payload.len())?;
    w.str16(topic)?;
    w.bytes(payload)?;
    Ok(w.pos)
}

pub fn pingreq(buf: &mut [u8]) -> Result<usize, Error> {
    Writer::new(buf).bytes(&[0xC0, 0x00]).map(|_| 2)
}

pub fn disconnect(buf: &mut [u8]) -> Result<usize, Error> {
    Writer::new(buf).bytes(&[0xE0, 0x00]).map(|_| 2)
}

/// Validate the 4-byte CONNACK the broker sends back.
pub fn check_connack(p: &[u8; 4]) -> Result<(), ConnackError> {
    if p[0] != 0x20 || p[1] != 0x02 {
        return Err(ConnackError::Malformed);
    }
    match p[3] {
        0 => Ok(()),
        code => Err(ConnackError::Refused(code)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connect_bytes() {
        let mut b = [0u8; 32];
        let n = connect(&mut b, "ab", 60).unwrap();
        assert_eq!(
            &b[..n],
            &[0x10, 14, 0, 4, b'M', b'Q', b'T', b'T', 4, 0x02, 0, 60, 0, 2, b'a', b'b']
        );
    }

    #[test]
    fn publish_bytes() {
        let mut b = [0u8; 32];
        let n = publish(&mut b, "a/b", b"hi", false).unwrap();
        assert_eq!(&b[..n], &[0x30, 7, 0, 3, b'a', b'/', b'b', b'h', b'i']);
    }

    #[test]
    fn retain_sets_low_bit() {
        let mut b = [0u8; 16];
        publish(&mut b, "t", b"x", true).unwrap();
        assert_eq!(b[0], 0x31);
    }

    #[test]
    fn remaining_len_multibyte() {
        let mut b = [0u8; 4];
        let mut w = Writer::new(&mut b);
        w.remaining_len(321).unwrap();
        assert_eq!(&b[..2], &[0xC1, 0x02]);
    }

    #[test]
    fn buffer_too_small() {
        let mut b = [0u8; 3];
        assert_eq!(connect(&mut b, "ab", 60), Err(Error::BufferTooSmall));
    }

    #[test]
    fn connack() {
        assert_eq!(check_connack(&[0x20, 2, 0, 0]), Ok(()));
        assert_eq!(
            check_connack(&[0x20, 2, 0, 5]),
            Err(ConnackError::Refused(5))
        );
        assert_eq!(
            check_connack(&[0x30, 2, 0, 0]),
            Err(ConnackError::Malformed)
        );
    }
}
