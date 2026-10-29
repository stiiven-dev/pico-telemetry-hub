#![cfg_attr(not(test), no_std)]

pub const BLOB_LEN: usize = 256;
pub const SSID_MAX: usize = 32;
pub const PASS_MAX: usize = 63; // WPA2 passphrase limit

const MAGIC: [u8; 4] = *b"HUBC";
const VERSION: u8 = 1;

// Blob layout (little-endian):
//   0..4 magic | 4 version | 5 ssid_len | 6 pass_len | 7 reserved
//   8..12 seq | 12..16 broker ip | 16..18 port | 18..20 interval_s
//   20..52 ssid | 52..115 pass | 115..252 reserved | 252..256 crc32 of 0..252

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    SsidLength,
    PassLength,
    Interval,
}

#[derive(Clone, PartialEq, Eq)]
pub struct Config {
    ssid: [u8; SSID_MAX],
    ssid_len: u8,
    pass: [u8; PASS_MAX],
    pass_len: u8,
    pub broker: [u8; 4],
    pub port: u16,
    pub interval_s: u16,
}

impl Config {
    const fn empty() -> Self {
        Self {
            ssid: [0; SSID_MAX],
            ssid_len: 0,
            pass: [0; PASS_MAX],
            pass_len: 0,
            broker: [0; 4],
            port: 1883,
            interval_s: 2,
        }
    }

    pub fn new(
        ssid: &str,
        pass: &str,
        broker: [u8; 4],
        port: u16,
        interval_s: u16,
    ) -> Result<Self, Error> {
        if interval_s == 0 {
            return Err(Error::Interval);
        }
        let mut c = Self::empty();
        c.set_ssid(ssid)?;
        c.set_pass(pass)?;
        c.broker = broker;
        c.port = port;
        c.interval_s = interval_s;
        Ok(c)
    }

    pub fn set_ssid(&mut self, s: &str) -> Result<(), Error> {
        let b = s.as_bytes();
        if b.is_empty() || b.len() > SSID_MAX {
            return Err(Error::SsidLength);
        }

        self.ssid = [0; SSID_MAX];
        self.ssid[..b.len()].copy_from_slice(b);
        self.ssid_len = b.len() as u8;
        Ok(())
    }

    pub fn set_pass(&mut self, s: &str) -> Result<(), Error> {
        let b = s.as_bytes();
        if b.len() < 8 || b.len() > PASS_MAX {
            return Err(Error::PassLength);
        }
        self.pass = [0; PASS_MAX];
        self.pass[..b.len()].copy_from_slice(b);
        self.pass_len = b.len() as u8;
        Ok(())
    }

    pub fn ssid(&self) -> &str {
        core::str::from_utf8(&self.ssid[..self.ssid_len as usize]).unwrap_or("")
    }

    pub fn pass(&self) -> &str {
        core::str::from_utf8(&self.pass[..self.pass_len as usize]).unwrap_or("")
    }
}

impl core::fmt::Debug for Config {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Config")
            .field("ssid", &self.ssid())
            .field("pass", &"***")
            .field("broker", &self.broker)
            .field("port", &self.port)
            .field("interval_s", &self.interval_s)
            .finish()
    }
}

/// CRC-32 (IEEE 802.3), bitwise.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg(); // all ones if the low bit is set
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

pub fn encode(cfg: &Config, seq: u32) -> [u8; BLOB_LEN] {
    let mut b = [0u8; BLOB_LEN];
    b[0..4].copy_from_slice(&MAGIC);
    b[4] = VERSION;
    b[5] = cfg.ssid_len;
    b[6] = cfg.pass_len;
    b[8..12].copy_from_slice(&seq.to_le_bytes());
    b[12..16].copy_from_slice(&cfg.broker);
    b[16..18].copy_from_slice(&cfg.port.to_le_bytes());
    b[18..20].copy_from_slice(&cfg.interval_s.to_le_bytes());
    b[20..52].copy_from_slice(&cfg.ssid);
    b[52..115].copy_from_slice(&cfg.pass);
    let crc = crc32(&b[..252]);
    b[252..256].copy_from_slice(&crc.to_le_bytes());
    b
}

/// Returns (sequence number, config) if the blob is intact and sensible.
pub fn decode(b: &[u8]) -> Option<(u32, Config)> {
    if b.len() != BLOB_LEN || b[0..4] != MAGIC || b[4] != VERSION {
        return None;
    }
    let crc = u32::from_le_bytes(b[252..256].try_into().ok()?);
    if crc != crc32(&b[..252]) {
        return None;
    }
    let ssid_len = b[5] as usize;
    let pass_len = b[6] as usize;
    let interval_s = u16::from_le_bytes(b[18..20].try_into().ok()?);
    if ssid_len == 0
        || ssid_len > SSID_MAX
        || pass_len < 8
        || pass_len > PASS_MAX
        || interval_s == 0
    {
        return None;
    }
    let mut cfg = Config::empty();
    cfg.ssid.copy_from_slice(&b[20..52]);
    cfg.ssid_len = ssid_len as u8;
    cfg.pass.copy_from_slice(&b[52..115]);
    cfg.pass_len = pass_len as u8;
    core::str::from_utf8(&cfg.ssid[..ssid_len]).ok()?;
    core::str::from_utf8(&cfg.pass[..pass_len]).ok()?;
    cfg.broker.copy_from_slice(&b[12..16]);
    cfg.port = u16::from_le_bytes(b[16..18].try_into().ok()?);
    cfg.interval_s = interval_s;
    let seq = u32::from_le_bytes(b[8..12].try_into().ok()?);
    Some((seq, cfg))
}

/// Pick the newest valid slot: (slot index, seq, config).
pub fn pick_latest(slot0: &[u8], slot1: &[u8]) -> Option<(usize, u32, Config)> {
    match (decode(slot0), decode(slot1)) {
        (Some((s0, c0)), Some((s1, c1))) => {
            if s1 > s0 {
                Some((1, s1, c1))
            } else {
                Some((0, s0, c0))
            }
        }
        (Some((s, c)), None) => Some((0, s, c)),
        (None, Some((s, c))) => Some((1, s, c)),
        (None, None) => None,
    }
}

/// Which slot to write next, and with which sequence number.
pub fn next_write(current: Option<(usize, u32)>) -> (usize, u32) {
    match current {
        Some((slot, seq)) => (1 - slot, seq.wrapping_add(1)),
        None => (0, 1),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Config {
        Config::new("HomeNet", "hunter2hunter2", [10, 50, 214, 5], 1883, 2).unwrap()
    }

    #[test]
    fn crc_known_vector() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn round_trip() {
        let (seq, cfg) = decode(&encode(&sample(), 7)).unwrap();
        assert_eq!(seq, 7);
        assert_eq!(cfg, sample());
        assert_eq!(cfg.ssid(), "HomeNet");
    }

    #[test]
    fn erased_flash_is_invalid() {
        assert!(decode(&[0xFF; BLOB_LEN]).is_none());
    }

    #[test]
    fn any_flipped_bit_is_detected() {
        let good = encode(&sample(), 1);
        for i in 0..BLOB_LEN {
            let mut b = good;
            b[i] ^= 1;
            assert!(decode(&b).is_none(), "flip at byte {i} went unnoticed");
        }
    }

    #[test]
    fn newest_slot_wins() {
        let a = encode(&sample(), 5);
        let mut newer = sample();
        newer.port = 8883;
        let b = encode(&newer, 6);
        let (slot, seq, cfg) = pick_latest(&a, &b).unwrap();
        assert_eq!((slot, seq, cfg.port), (1, 6, 8883));
        assert_eq!(pick_latest(&b, &a).unwrap().0, 0);
    }

    #[test]
    fn corrupt_newest_falls_back_to_older() {
        let a = encode(&sample(), 5);
        let mut b = encode(&sample(), 6);
        b[30] ^= 0xFF; // simulates a write interrupted by power loss
        let (slot, seq, _) = pick_latest(&a, &b).unwrap();
        assert_eq!((slot, seq), (0, 5));
    }

    #[test]
    fn next_write_alternates() {
        assert_eq!(next_write(None), (0, 1));
        assert_eq!(next_write(Some((0, 1))), (1, 2));
        assert_eq!(next_write(Some((1, 2))), (0, 3));
    }

    #[test]
    fn validation() {
        assert_eq!(
            Config::new("", "hunter2hunter2", [0; 4], 1883, 2),
            Err(Error::SsidLength)
        );
        assert_eq!(
            Config::new("x", "short", [0; 4], 1883, 2),
            Err(Error::PassLength)
        );
        assert_eq!(
            Config::new("x", "hunter2hunter2", [0; 4], 1883, 0),
            Err(Error::Interval)
        );
        let long = "a".repeat(33);
        assert_eq!(
            Config::new(&long, "hunter2hunter2", [0; 4], 1883, 2),
            Err(Error::SsidLength)
        );
    }

    #[test]
    fn debug_hides_password() {
        let s = format!("{:?}", sample());
        assert!(!s.contains("hunter2"));
        assert!(s.contains("HomeNet"));
    }
}
