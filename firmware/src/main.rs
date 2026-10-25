#![no_std]
#![no_main]

use core::net::Ipv4Addr;
use core::str::FromStr;

use cyw43::{aligned_bytes, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Timer};
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
});

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

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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
            Ok(_) => break,
            Err(_) => {
                blink_n(&mut control, 3).await;
                Timer::after_secs(2).await;
            }
        }
    }

    // Fast blink while waiting for link + DHCP lease; each blink runs to completion
    while !(stack.is_link_up() && stack.is_config_up()) {
        blink_n(&mut control, 1).await;
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

        loop {
            let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
            socket.set_timeout(Some(Duration::from_secs(10)));

            match socket.connect(broker).await {
                Ok(()) => {
                    stage.set_high();
                    Timer::after_secs(5).await;
                    socket.abort();
                    let _ = socket.flush().await;
                    stage.set_low();
                }
                Err(_) => {
                    blink_n(&mut control, 2).await;
                    control.gpio_set(0, true).await;
                }
            }
            Timer::after_secs(3).await;
        }
    }
}
