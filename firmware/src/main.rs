#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::net::Ipv4Addr;
use core::str::FromStr;

use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode, CHIP_ID};

use cyw43::{aligned_bytes, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, I2C0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_time::{with_timeout, Delay, Duration, Timer};
use panic_persist as _;
use static_cell::StaticCell;

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
const BROKER_PORT: u16 = 1883;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0> , dma::InterruptHandler<DMA_CH1>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

type I2cBus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
type SharedI2c = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

#[derive(Clone, Copy)]
enum Measurement {
    Bme280 {
        temp_c: f32,
        rh: f32,
        press_hpa: f32,
    },
    Log(&'static str),
}

// 8 queued messages, 2 subscribers (uplink, display), 4 publishers (3 sensors + spare)
static TELEMETRY: PubSubChannel<CriticalSectionRawMutex, Measurement, 8, 2, 4> =
    PubSubChannel::new();
type TelemetrySub = Subscriber<'static, CriticalSectionRawMutex, Measurement, 8, 2, 4>;

#[embassy_executor::task]
async fn bme280_task(i2c: SharedI2c) {
    let tx = TELEMETRY.immediate_publisher();
    let mut bme = AsyncBme280::new(i2c, Delay);

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
        Timer::after_secs(2).await;
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

async fn send_all(socket: &mut TcpSocket<'_>, mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        match socket.write(data).await {
            Ok(0) | Err(_) => return Err(()),
            Ok(n) => data = &data[n..],
        }
    }
    Ok(())
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
) -> Result<(), &'static str> {
    let mut pkt = [0u8; 128];

    let n = mqtt_lite::connect(&mut pkt, "pico-telemetry-hub", 60).map_err(|_| "encode connect")?;
    send_all(socket, &pkt[..n])
        .await
        .map_err(|_| "send connect")?;

    let mut ack = [0u8; 4];
    with_timeout(Duration::from_secs(5), read_exact(socket, &mut ack))
        .await
        .map_err(|_| "connack timeout")?
        .map_err(|_| "connack read")?;
    mqtt_lite::check_connack(&ack).map_err(|_| "connack refused")?;
    log::info!("mqtt connected");
    stage.set_high(); // GP13 on = MQTT session up

    while sub.try_next_message_pure().is_some() {} // drop reading queued while offline

    loop {
        let n = match sub.next_message_pure().await {
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
            Measurement::Log(msg) => mqtt_lite::publish(&mut pkt, "hub/log", msg.as_bytes(), false),
        }
        .map_err(|_| "encode publish")?;
        send_all(socket, &pkt[..n])
            .await
            .map_err(|_| "send publish")?;
    }
}

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // GP12: reset button when pressed
    let btn = Input::new(p.PIN_12, Pull::Up);
    spawner.spawn(bootsel_task(btn).unwrap());

    // I2C
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c::Config::default());
    let bus: &'static I2cBus = I2C_BUS.init(Mutex::new(i2c));
    spawner.spawn(bme280_task(I2cDevice::new(bus)).unwrap());

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
    if SSID.is_empty() {
        // built without credentials: 5 blinks, repeat forever
        loop {
            blink_n(&mut control, 5).await;
            Timer::after_secs(2).await;
        }
    }

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        rosc_seed(),
    );

    spawner.spawn(net_task(net_runner).unwrap());

    loop {
        match control.join(SSID, JoinOptions::new(PASS.as_bytes())).await {
            Ok(_) => {
                log::info!("joined '{}'", SSID);
                break;
            }
            Err(e) => {
                log::warn!("join failed: {:?}", e);
                blink_n(&mut control, 3).await;
                Timer::after_secs(2).await;
            }
        }
    }

    // Fast blink while waiting for link + DHCP lease; each blink runs to completion
    while !(stack.is_link_up() && stack.is_config_up()) {
        blink_n(&mut control, 1).await;
    }
    if let Some(cfg) = stack.config_v4() {
        log::info!("got IP {}", cfg.address);
    }
    control.gpio_set(0, true).await; // solid on = has IP

    loop {
        let ip = match Ipv4Addr::from_str(BROKER_IP) {
            Ok(ip) => ip,
            Err(_) => loop {
                // missing or invalid BROKER_IP: 7 blinks, repeat
                blink_n(&mut control, 7).await;
                Timer::after_secs(2).await;
            },
        };
        let o = ip.octets();
        let broker = (Ipv4Address::new(o[0], o[1], o[2], o[3]), BROKER_PORT);

        let mut rx_buf = [0u8; 512];
        let mut tx_buf = [0u8; 512];
        let mut sub: TelemetrySub = TELEMETRY.subscriber().unwrap();

        loop {
            log::info!("connecting to broker...");
            let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
            socket.set_timeout(Some(Duration::from_secs(10)));

            match socket.connect(broker).await {
                Ok(()) => {
                    log::info!("tcp connected");
                    if let Err(why) = mqtt_session(&mut socket, &mut stage, &mut sub).await {
                        log::warn!("mqtt session ended: {}", why);
                        blink_n(&mut control, 4).await; // 4 blinks = MQTT session failed
                        control.gpio_set(0, true).await;
                    }
                    stage.set_low();
                    socket.abort();
                    let _ = socket.flush().await;
                }
                Err(e) => {
                    log::warn!("tcp connect failed: {:?}", e);
                    blink_n(&mut control, 2).await;
                    control.gpio_set(0, true).await;
                }
            }
            Timer::after_secs(3).await;
        }
    }
}
