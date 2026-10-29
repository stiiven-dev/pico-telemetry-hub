use embassy_rp::flash::{Blocking, Error, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use hub_config::{encode, next_write, pick_latest, Config, BLOB_LEN};

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
const BASE: u32 = (FLASH_SIZE - 2 * ERASE_SIZE) as u32; // offset from flash start

pub type Store = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

pub struct Loaded {
    pub slot: usize,
    pub seq: u32,
    pub config: Config,
}

pub fn load(flash: &mut Store) -> Option<Loaded> {
    let mut a = [0u8; BLOB_LEN];
    let mut b = [0u8; BLOB_LEN];
    flash.blocking_read(BASE, &mut a).ok()?;
    flash.blocking_read(BASE + ERASE_SIZE as u32, &mut b).ok()?;
    pick_latest(&a, &b).map(|(slot, seq, config)| Loaded { slot, seq, config })
}

/// Write into the slot that is NOT the current one, so the old copy stays
/// valid until the new one is complete.
pub fn save(
    flash: &mut Store,
    cfg: &Config,
    current: Option<(usize, u32)>,
) -> Result<(usize, u32), Error> {
    let (slot, seq) = next_write(current);
    let off = BASE + (slot * ERASE_SIZE) as u32;
    flash.blocking_erase(off, off + ERASE_SIZE as u32)?;
    flash.blocking_write(off, &encode(cfg, seq))?;
    Ok((slot, seq))
}
