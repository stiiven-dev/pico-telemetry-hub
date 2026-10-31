#![no_std]
#![no_main]

mod config_store;
mod imu;

use core::fmt::Write as _;
use core::net::Ipv4Addr;
use core::str::FromStr;

use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode, CHIP_ID};
use config_store::{Loaded, Store};
use hub_config::Config as ConfigHub;
use motion_core::{Calibrator, ComplementaryFilter, MedianFilter, NO_READING};
use mqtt_lite::backoff::{jitter, Backoff};
use srf05::{EdgeCapture, Reading};

use cyw43::{aligned_bytes, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, Stack, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, I2C0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_rp::{bind_interrupts, dma};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_time::{block_for, with_timeout, Delay, Duration, Instant, Ticker, Timer};
use panic_persist as _;
use portable_atomic::{AtomicU8, Ordering};
use static_cell::StaticCell;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

const SSID: &str = match option_env!("WIFI_SSID") {
    Some(s) => s,
    None => "",
};
const PASS: &str = match option_env!("WIFI_PASS") {
    Some(s) => s,
    None => "",
};
const BROKER_IP: &str = match option_env!("BROKER_IP") {
    Some(s) => s,
    None => "",
};
const RANGE_MEDIAN_N: usize = 5; //must be odd
const HB_BME: u8 = 1 << 0;
const HB_IMU: u8 = 1 << 1;
const HB_RANGE: u8 = 1 << 2;
const HB_DISPLAY: u8 = 1 << 3;
const HB_UPLINK: u8 = 1 << 4;
const HB_ALL: u8 = 0b1_1111;
const WD_MAGIC: u32 = 0xB17E_0000;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0> , dma::InterruptHandler<DMA_CH1>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

type I2cBus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
type SharedI2c = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();
static ALIVE: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy, PartialEq)]
enum UplinkState {
    Down,
    Up,
}

#[derive(Clone, Copy)]
enum Measurement {
    Bme280 {
        temp_c: f32,
        rh: f32,
        press_hpa: f32,
    },
    Attitude {
        pitch_deg: f32,
        roll_deg: f32,
    },
    Range {
        mm: Option<u16>,
    },
    Uplink(UplinkState),
    Log(&'static str),
}

// 8 queued messages, 2 subscribers (uplink, display), 4 publishers (3 sensors + spare)
static TELEMETRY: PubSubChannel<CriticalSectionRawMutex, Measurement, 8, 2, 4> =
    PubSubChannel::new();
type TelemetrySub = Subscriber<'static, CriticalSectionRawMutex, Measurement, 8, 2, 4>;
type Notes = heapless::Deque<&'static str, 4>;

#[embassy_executor::task]
async fn bme280_task(i2c: SharedI2c, interval_s: u64) {
    let tx = TELEMETRY.immediate_publisher();
    let mut bme = AsyncBme280::new(i2c, Delay);
    beat(HB_BME);

    loop {
        match bme.chip_id().await {
            Err(_) => tx.publish_immediate(Measurement::Log("bme280: no i2c response")),
            Ok(id) if id != CHIP_ID => {
                tx.publish_immediate(Measurement::Log("bme280: wrong chip ID (BMP280??)"))
            }
            Ok(_) => {
                let cfg = Configuration::default()
                    .with_temperature_oversampling(Oversampling::Oversample1)
                    .with_pressure_oversampling(Oversampling::Oversample1)
                    .with_humidity_oversampling(Oversampling::Oversample1)
                    .with_sensor_mode(SensorMode::Normal);
                if bme.init().await.is_ok() && bme.set_sampling_configuration(cfg).await.is_ok() {
                    break;
                }
                tx.publish_immediate(Measurement::Log("bme280: init failed"));
            }
        }
        beat(HB_BME);
        Timer::after_secs(5).await;
    }
    Timer::after_millis(50).await;

    loop {
        match bme.read_sample().await {
            Ok(s) => match (s.temperature, s.humidity, s.pressure) {
                (Some(t), Some(h), Some(p)) => tx.publish_immediate(Measurement::Bme280 {
                    temp_c: t,
                    rh: h,
                    press_hpa: p / 100.0,
                }),
                _ => tx.publish_immediate(Measurement::Log("bme280: measurement disabled")),
            },
            Err(_) => tx.publish_immediate(Measurement::Log("bme280: read failed")),
        }
        sleep_beating(HB_BME, interval_s as u32).await;
    }
}

#[embassy_executor::task]
async fn imu_task(mut i2c: SharedI2c, mut int: Input<'static>) {
    let tx = TELEMETRY.immediate_publisher();

    loop {
        match imu::init(&mut i2c).await {
            Ok(()) => break,
            Err(imu::Error::UnexpectedWhoAmI(_)) => {
                tx.publish_immediate(Measurement::Log("imu: unexpected WHO_AM_I"))
            }
            Err(_) => tx.publish_immediate(Measurement::Log("imu: no i2c response")),
        }
        beat(HB_IMU);
        Timer::after_secs(5).await;
    }

    tx.publish_immediate(Measurement::Log("imu: calibrating, keep still and level"));
    let mut calibrator = Calibrator::<200>::new();
    let cal = loop {
        beat(HB_IMU);
        if with_timeout(Duration::from_millis(100), int.wait_for_rising_edge())
            .await
            .is_err()
        {
            tx.publish_immediate(Measurement::Log("imu: no data-ready interrupt"));
            beat(HB_IMU);
            Timer::after_secs(2).await;
            continue;
        }
        match imu::read(&mut i2c).await {
            Ok((accel, gyro)) => {
                if let Some(c) = calibrator.push(gyro, accel) {
                    break c;
                }
            }
            Err(_) => tx.publish_immediate(Measurement::Log("imu: read failed")),
        }
    };
    tx.publish_immediate(Measurement::Log("imu: calibration done"));

    let mut filter = ComplementaryFilter::new(0.98);
    let mut last = Instant::now();
    let mut n: u32 = 0;
    loop {
        beat(HB_IMU);
        if with_timeout(Duration::from_millis(100), int.wait_for_rising_edge())
            .await
            .is_err()
        {
            tx.publish_immediate(Measurement::Log("imu: no data-ready interrupt"));
            beat(HB_IMU);
            Timer::after_secs(2).await;
            continue;
        }
        let (accel, gyro) = match imu::read(&mut i2c).await {
            Ok(s) => s,
            Err(_) => {
                tx.publish_immediate(Measurement::Log("imu: read failed"));
                continue;
            }
        };
        let now = Instant::now();
        let dt = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;

        let accel_g = cal.correct_accel(accel).accel_to_g();
        let gyro_dps = cal.correct_gyro(gyro).gyro_to_dps();
        // TODO: apply the same gyro axis inversion your RTIC firmware needed
        let att = filter.update(dt, gyro_dps, accel_g);

        n += 1;
        if n % 20 == 0 {
            tx.publish_immediate(Measurement::Attitude {
                pitch_deg: att.pitch_deg,
                roll_deg: att.roll_deg,
            }); // every 20th sample at 100 Hz = 5 Hz
        }
    }
}

#[embassy_executor::task]
async fn range_task(mut trig: Output<'static>, mut echo: Input<'static>) {
    let tx = TELEMETRY.immediate_publisher();
    let mut median = MedianFilter::<RANGE_MEDIAN_N>::new();

    loop {
        beat(HB_RANGE);
        // ECHO must be idle-low before we ping
        if with_timeout(Duration::from_millis(40), echo.wait_for_low())
            .await
            .is_err()
        {
            tx.publish_immediate(Measurement::Log("range: echo stuck high"));
            Timer::after_secs(2).await;
            continue;
        }

        trig.set_high();
        block_for(Duration::from_micros(10));
        trig.set_low();

        let mut cap = EdgeCapture::new();
        let timed_out = with_timeout(Duration::from_millis(40), async {
            echo.wait_for_high().await;
            cap.on_rise(Instant::now().as_micros());
            echo.wait_for_low().await;
            cap.on_fall(Instant::now().as_micros());
        })
        .await
        .is_err();

        let sample = if timed_out {
            NO_READING
        } else {
            match cap.take_reading() {
                Ok(Reading::Mm(mm)) => mm,
                Ok(Reading::OutOfRange) | Err(_) => NO_READING,
            }
        };

        if let Some(m) = median.push(sample) {
            tx.publish_immediate(Measurement::Range {
                mm: if m == NO_READING { None } else { Some(m) },
            });
        }

        Timer::after_millis(200).await;
    }
}

#[derive(Default)]
struct Screen {
    uplink_up: bool,
    env: Option<(f32, f32, f32)>, // temp_c, rh, press_hpa
    att: Option<(f32, f32)>,      // pitch, roll
    range: Option<Option<u16>>,   // None = nothing received yet; Some(None) = out of range
}

fn put<D>(d: &mut D, y: i32, s: &str)
where
    D: DrawTarget<Color = BinaryColor>,
{
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = Text::with_baseline(s, Point::new(0, y), style, Baseline::Top).draw(d);
}

#[embassy_executor::task]
async fn display_task(i2c: SharedI2c) {
    let tx = TELEMETRY.immediate_publisher();
    let mut sub: TelemetrySub = TELEMETRY.subscriber().unwrap();

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    loop {
        beat(HB_DISPLAY);
        match display.init().await {
            Ok(()) => break,
            Err(_) => {
                tx.publish_immediate(Measurement::Log("oled: init failed"));
                Timer::after_secs(5).await;
            }
        }
    }

    let mut screen = Screen::default();
    let mut flush_failed = false;
    let mut tick = Ticker::every(Duration::from_millis(250));

    loop {
        beat(HB_DISPLAY);
        // Fold in everything that arrived since the last redraw
        while let Some(m) = sub.try_next_message_pure() {
            match m {
                Measurement::Bme280 {
                    temp_c,
                    rh,
                    press_hpa,
                } => screen.env = Some((temp_c, rh, press_hpa)),
                Measurement::Attitude {
                    pitch_deg,
                    roll_deg,
                } => screen.att = Some((pitch_deg, roll_deg)),
                Measurement::Range { mm } => screen.range = Some(mm),
                Measurement::Uplink(s) => screen.uplink_up = s == UplinkState::Up,
                Measurement::Log(_) => {}
            }
        }

        let _ = display.clear(BinaryColor::Off);
        let mut s: heapless::String<32> = heapless::String::new();

        let _ = write!(
            s,
            "hub   MQTT {}",
            if screen.uplink_up { "up" } else { "down" }
        );
        put(&mut display, 0, &s);

        s.clear();
        match screen.env {
            Some((t, rh, _)) => {
                let _ = write!(s, "{:.1}C  {:.0}%RH", t, rh);
            }
            None => {
                let _ = write!(s, "T --");
            }
        }
        put(&mut display, 12, &s);

        s.clear();
        match screen.env {
            Some((_, _, p)) => {
                let _ = write!(s, "{:.1} hPa", p);
            }
            None => {
                let _ = write!(s, "P --");
            }
        }
        put(&mut display, 24, &s);

        s.clear();
        match screen.att {
            Some((p, r)) => {
                let _ = write!(s, "pit {:+.1} rol {:+.1}", p, r);
            }
            None => {
                let _ = write!(s, "attitude --");
            }
        }
        put(&mut display, 36, &s);

        s.clear();
        match screen.range {
            Some(Some(mm)) => {
                let _ = write!(s, "dist {} mm", mm);
            }
            Some(None) => {
                let _ = write!(s, "dist out of range");
            }
            None => {
                let _ = write!(s, "dist --");
            }
        }
        put(&mut display, 48, &s);

        // Report a flush failure once, not every 250 ms
        match display.flush().await {
            Ok(()) => flush_failed = false,
            Err(_) => {
                if !flush_failed {
                    tx.publish_immediate(Measurement::Log("oled: flush failed"));
                }
                flush_failed = true;
            }
        }

        tick.next().await;
    }
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn bootsel_task(mut btn: embassy_rp::gpio::Input<'static>) {
    btn.wait_for_low().await;
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
}

#[embassy_executor::task]
async fn watchdog_task(mut wd: Watchdog) {
    wd.start(Duration::from_secs(8));
    let mut seen = 0u8;
    loop {
        Timer::after_millis(250).await;
        seen |= ALIVE.swap(0, Ordering::Relaxed);
        wd.set_scratch(0, WD_MAGIC | seen as u32);
        if seen & HB_ALL == HB_ALL {
            wd.feed(Duration::from_secs(8));
            seen = 0;
        }
    }
}

async fn blink_n(control: &mut cyw43::Control<'_>, n: u8) {
    for _ in 0..n {
        control.gpio_set(0, true).await;
        Timer::after_millis(150).await;
        control.gpio_set(0, false).await;
        Timer::after_millis(150).await;
    }
}

fn rosc_seed() -> u64 {
    let mut s = 0u64;
    for _ in 0..8 {
        s = (s << 8) | RoscRng::next_u8() as u64;
    }
    s
}

async fn send_all(socket: &mut TcpSocket<'_>, data: &[u8]) -> Result<(), ()> {
    with_timeout(Duration::from_secs(3), async {
        let mut data = data;
        while !data.is_empty() {
            match socket.write(data).await {
                Ok(0) | Err(_) => return Err(()),
                Ok(n) => data = &data[n..],
            }
        }
        Ok(())
    })
    .await
    .unwrap_or(Err(()))
}

async fn read_exact(socket: &mut TcpSocket<'_>, buf: &mut [u8]) -> Result<(), ()> {
    let mut got = 0;
    while got < buf.len() {
        match socket.read(&mut buf[got..]).await {
            Ok(0) | Err(_) => return Err(()),
            Ok(n) => got += n,
        }
    }
    Ok(())
}

async fn mqtt_session(
    socket: &mut TcpSocket<'_>,
    stage: &mut Output<'_>,
    sub: &mut TelemetrySub,
    pending: &mut Notes,
) -> Result<(), &'static str> {
    let mut pkt = [0u8; 128];

    let n = mqtt_lite::connect(&mut pkt, "pico-telemetry-hub", 60).map_err(|_| "encode connect")?;
    send_all(socket, &pkt[..n])
        .await
        .map_err(|_| "send connect")?;

    let mut ack = [0u8; 4];
    with_timeout(Duration::from_secs(3), read_exact(socket, &mut ack))
        .await
        .map_err(|_| "connack timeout")?
        .map_err(|_| "connack read")?;
    mqtt_lite::check_connack(&ack).map_err(|_| "connack refused")?;
    stage.set_high(); // GP13 on = MQTT session up
    log::info!("mqtt connected");

    while sub.try_next_message_pure().is_some() {} // 1. drain first

    let tx = TELEMETRY.immediate_publisher(); // 2. then announce
    tx.publish_immediate(Measurement::Uplink(UplinkState::Up));
    while let Some(note) = pending.pop_front() {
        tx.publish_immediate(Measurement::Log(note));
    }

    loop {
        beat(HB_UPLINK);
        let msg = match with_timeout(Duration::from_secs(3), sub.next_message_pure()).await {
            Ok(m) => m,
            Err(_) => continue, // nothing to send for 3 s: just beat again
        };

        let n = match msg {
            Measurement::Bme280 {
                temp_c,
                rh,
                press_hpa,
            } => {
                let mut p: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    p,
                    "{{\"t_c\":{:.2},\"rh\":{:.1},\"p_hpa\":{:.1}}}",
                    temp_c, rh, press_hpa
                );
                mqtt_lite::publish(&mut pkt, "hub/bme280", p.as_bytes(), false)
            }
            Measurement::Attitude {
                pitch_deg,
                roll_deg,
            } => {
                let mut p: heapless::String<64> = heapless::String::new();
                let _ = write!(p, "{{\"pitch\":{:.1},\"roll\":{:.1}}}", pitch_deg, roll_deg);
                mqtt_lite::publish(&mut pkt, "hub/imu", p.as_bytes(), false)
            }
            Measurement::Range { mm } => {
                let mut p: heapless::String<24> = heapless::String::new();
                let _ = match mm {
                    Some(v) => write!(p, "{{\"mm\":{}}}", v),
                    None => write!(p, "{{\"mm\":null}}"),
                };
                mqtt_lite::publish(&mut pkt, "hub/range", p.as_bytes(), false)
            }
            Measurement::Uplink(_) => continue,
            Measurement::Log(msg) => mqtt_lite::publish(&mut pkt, "hub/log", msg.as_bytes(), false),
        }
        .map_err(|_| "encode publish")?;
        send_all(socket, &pkt[..n])
            .await
            .map_err(|_| "send publish")?;
    }
}

async fn bring_up_network(
    control: &mut cyw43::Control<'_>,
    stack: Stack<'static>,
    cfg: &ConfigHub,
) -> Result<(), &'static str> {
    if !stack.is_link_up() {
        with_timeout(
            Duration::from_secs(4),
            control.join(cfg.ssid(), JoinOptions::new(cfg.pass().as_bytes())),
        )
        .await
        .map_err(|_| "wifi: join timeout")?
        .map_err(|_| "wifi: join failed")?;
    }
    beat(HB_UPLINK);
    with_timeout(Duration::from_secs(4), async {
        stack.wait_link_up().await;
        stack.wait_config_up().await;
    })
    .await
    .map_err(|_| "net: no dhcp lease")?;
    Ok(())
}

/// One attempt: network, TCP, MQTT session. Returns why it ended.
async fn run_once(
    control: &mut cyw43::Control<'_>,
    stack: Stack<'static>,
    cfg: &ConfigHub,
    broker: (Ipv4Address, u16),
    rx_buf: &mut [u8],
    tx_buf: &mut [u8],
    stage: &mut Output<'_>,
    sub: &mut TelemetrySub,
    pending: &mut Notes,
    session_len: &mut Option<Duration>,
) -> &'static str {
    if let Err(why) = bring_up_network(control, stack, cfg).await {
        return why;
    }
    beat(HB_UPLINK);

    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(10)));
    match with_timeout(Duration::from_secs(4), socket.connect(broker)).await {
        Ok(Ok(())) => {}
        Ok(Err(_)) => return "tcp: connect failed",
        Err(_) => return "tcp: connect timeout",
    }

    let started = Instant::now();
    // End the session as soon as either MQTT fails or the Wi-Fi link drops
    let why = match select(
        mqtt_session(&mut socket, stage, sub, pending),
        stack.wait_link_down(),
    )
    .await
    {
        Either::First(Err(why)) => why,
        Either::First(Ok(())) => "mqtt: session ended",
        Either::Second(()) => "wifi: link lost",
    };
    *session_len = Some(started.elapsed());

    socket.abort();
    let _ = with_timeout(Duration::from_secs(1), socket.flush()).await;
    why
}

fn dev_config_from_env() -> Option<ConfigHub> {
    let ip = Ipv4Addr::from_str(BROKER_IP).ok()?;
    ConfigHub::new(SSID, PASS, ip.octets(), 1883, 2).ok()
}

fn beat(bit: u8) {
    ALIVE.fetch_or(bit, Ordering::Relaxed);
}

/// Sleep for `secs`, reporting in every second.
async fn sleep_beating(bit: u8, secs: u32) {
    for _ in 0..secs {
        beat(bit);
        Timer::after_secs(1).await;
    }
}
#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut wd = Watchdog::new(p.WATCHDOG);
    let wd_note: Option<&'static str> = if wd.reset_reason() == Some(ResetReason::TimedOut) {
        let raw = wd.get_scratch(0);
        wd.set_scratch(0, 0); // consume it so it can't go stale
        let missing = HB_ALL & !(raw as u8);
        Some(if raw & 0xFFFF_0000 != WD_MAGIC {
            "boot: watchdog reset"
        } else if missing & HB_UPLINK != 0 {
            "boot: watchdog reset, uplink silent"
        } else if missing & HB_IMU != 0 {
            "boot: watchdog reset, imu silent"
        } else if missing & HB_BME != 0 {
            "boot: watchdog reset, bme280 silent"
        } else if missing & HB_RANGE != 0 {
            "boot: watchdog reset, range silent"
        } else if missing & HB_DISPLAY != 0 {
            "boot: watchdog reset, display silent"
        } else {
            "boot: watchdog reset"
        })
    } else {
        None
    };

    let mut flash = Store::new_blocking(p.FLASH);
    let mut loaded = config_store::load(&mut flash);

    // Dev convenience: credentials baked in at build time are authoritative
    // and get persisted (only when they differ from what's stored).
    if let Some(env_cfg) = dev_config_from_env() {
        let current = loaded.as_ref().map(|l| (l.slot, l.seq));
        let same = loaded.as_ref().map_or(false, |l| l.config == env_cfg);
        if !same {
            if let Ok((slot, seq)) = config_store::save(&mut flash, &env_cfg, current) {
                loaded = Some(Loaded {
                    slot,
                    seq,
                    config: env_cfg,
                });
            }
        }
    }
    let interval_s = loaded.as_ref().map_or(2, |l| l.config.interval_s) as u64;

    // GP12: reset button when pressed
    let btn = Input::new(p.PIN_12, Pull::Up);
    spawner.spawn(bootsel_task(btn).unwrap());

    // I2C
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_cfg);
    let bus: &'static I2cBus = I2C_BUS.init(Mutex::new(i2c));
    spawner.spawn(bme280_task(I2cDevice::new(bus), interval_s).unwrap());

    let imu_int = Input::new(p.PIN_16, Pull::None);
    spawner.spawn(imu_task(I2cDevice::new(bus), imu_int).unwrap());

    let trig = Output::new(p.PIN_15, Level::Low);
    let echo = Input::new(p.PIN_14, Pull::None);
    spawner.spawn(range_task(trig, echo).unwrap());

    spawner.spawn(display_task(I2cDevice::new(bus)).unwrap());

    log::info!("boot: pico-telemetry-hub");
    if let Some(msg) = panic_persist::get_panic_message_utf8() {
        log::error!("previous run panicked: {}", msg);
    }

    // GP13: high while the Wi-Fi chip is coming up; stuck high = hung in bring-up
    let mut stage = Output::new(p.PIN_13, Level::High);

    let fw = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    spawner.spawn(cyw43_task(runner).unwrap());

    control.init(clm).await;
    log::info!("cyw43 initialised");

    stage.set_low();

    let Some(Loaded { config: cfg, .. }) = loaded else {
        loop {
            blink_n(&mut control, 5).await; // no configuration at all
            Timer::after_secs(2).await;
        }
    };

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        rosc_seed(),
    );

    spawner.spawn(net_task(net_runner).unwrap());

    let mut sub: TelemetrySub = TELEMETRY.subscriber().unwrap();
    let broker = (
        Ipv4Address::new(cfg.broker[0], cfg.broker[1], cfg.broker[2], cfg.broker[3]),
        cfg.port,
    );
    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 512];
    let link_tx = TELEMETRY.immediate_publisher();
    let mut backoff = Backoff::new(1_000, 30_000);
    let mut last_ok = Instant::now();
    let mut pending = Notes::new();
    if let Some(n) = wd_note {
        let _ = pending.push_back(n);
    }

    spawner.spawn(watchdog_task(wd).unwrap()); // start guarding only now that bring-up is done

    loop {
        beat(HB_UPLINK);
        link_tx.publish_immediate(Measurement::Uplink(UplinkState::Down));

        let mut session_len = None;
        let why = run_once(
            &mut control,
            stack,
            &cfg,
            broker,
            &mut rx_buf,
            &mut tx_buf,
            &mut stage,
            &mut sub,
            &mut pending,
            &mut session_len,
        )
        .await;

        link_tx.publish_immediate(Measurement::Uplink(UplinkState::Down));
        stage.set_low();
        if pending.is_full() {
            pending.pop_front();
        }
        let _ = pending.push_back(why);
        log::warn!("uplink: {}", why);

        if matches!(session_len, Some(d) if d >= Duration::from_secs(30)) {
            backoff.reset();
            last_ok = Instant::now();
        }

        // No working session for 5 minutes: stop beating and let the watchdog reset us
        if last_ok.elapsed() > Duration::from_secs(300) {
            loop {
                Timer::after_secs(1).await; // no beat(HB_UPLINK): reset within ~8 s
            }
        }

        let delay_ms = jitter(backoff.next_delay_ms(), rosc_seed() as u32);
        let mut left = delay_ms;
        while left > 0 {
            beat(HB_UPLINK);
            let step = left.min(1000);
            Timer::after_millis(step as u64).await;
            left -= step;
        }
    }
}
