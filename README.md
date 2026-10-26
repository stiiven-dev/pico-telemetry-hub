# pico-telemetry-hub

Wi-Fi-connected sensor node publishing BME280, MPU-6050, and SRF05 telemetry over MQTT — the capstone that ties every sensor from this series onto one board. No debug probe required.

<!-- TODO: hero photo of the breadboard -->
`docs/images/breadboard.jpg`

<!-- TODO: demo GIF — OLED status screen through a reconnect cycle, Grafana dashboard updating live -->
`docs/images/demo.gif`

---

## Features

- **All three prior sensors on one board**: BME280 (temp/humidity/pressure), MPU-6050 (attitude), SRF05 (distance) — reused wiring and drivers from `pico-weather-station` and `srf05-imu-station`, not rebuilt from scratch.
- **First Embassy (async) project in the series** — everything before this ran on RTIC or bare interrupts. Networking is the reason: I/O-heavy, connection-oriented work is exactly where an async executor earns its complexity over a priority-task model. See the deep-dive doc for why.
- **MQTT publishing** over Wi-Fi via `cyw43` + `embassy-net`, with exponential-backoff reconnection that survives a router reboot without manual intervention.
- **Serial provisioning console** — Wi-Fi credentials and broker config are entered over USB-serial and stored in flash, never committed to git (and CI checks for that).
- **Watchdog fed only when every task has heartbeated** — this matters more here than in any earlier project: a hung async task with no probe attached is otherwise invisible until the OLED visibly stops updating.
- **`docker-compose.yml`** spins up Mosquitto + Grafana so anyone cloning this can see live telemetry in a dashboard with one command.
- Same USB-only dev loop as every project before it: `defmt-serial` logging, `panic-persist` crash capture, no SWD probe.

## To-Do list

- [x] `cyw43` up, onboard LED blinks (finally — the one LED this whole series couldn't reach without it)
- [x] DHCP + ping, confirmed reachable on the local network
- [x] MQTT publish of one sensor (BME280, simplest to start)
- [ ] all three sensors flowing through a shared `PubSubChannel`
- [ ] serial provisioning console + flash-backed config store
- [ ] reconnection backoff + watchdog, chaos-tested (pull the router mid-run)
- [ ] Grafana dashboard + `docker-compose.yml`
- [ ] CI check: no hardcoded Wi-Fi credentials committed
- [ ] `docs/adr/` — architecture decision records
- [ ] firmware version + git hash in the MQTT birth message
- [ ] CI green
- [ ] `v1.0.0`

## Hardware

Nothing new — this project's entire sensor set is the union of the previous two:

| Part | Qty | From | Notes |
| --- | --- | --- | --- |
| Raspberry Pi Pico W (or WH) | 1 | — | RP2040 + CYW43439, Wi-Fi finally gets used |
| GY-BME280 breakout | 1 | `pico-weather-station` | 3.3V only |
| MPU-6050 (GY-521) | 1 | `srf05-imu-station` | 3.3V from the 3V3 pin, not VBUS |
| SRF05 ultrasonic rangefinder | 1 | `srf05-imu-station` | 5V, ECHO needs the level-shift divider |
| SSD1306/SSD1309 OLED | 1 | reused across every project | status screen, not paginated this time |
| Push button | 1 | reused | manual reconnect trigger + reboot-to-bootloader |
| USB micro-B cable | 1 | — | data-capable |

## Wiring

Identical to the union of `pico-weather-station`'s I²C wiring and `srf05-imu-station`'s SRF05 wiring — no new pins needed:

| Pico W pin | Signal | Notes |
| --- | --- | --- |
| GP4 (pin 6) | I²C0 SDA | shared by **three** devices now: BME280, MPU-6050, OLED |
| GP5 (pin 7) | I²C0 SCL | same three-device bus |
| GP16 (pin 21) | MPU-6050 INT | |
| GP15 (pin 20) | SRF05 TRIG | |
| GP14 (pin 19) | SRF05 ECHO, via 1kΩ/2kΩ divider | 5V signal — divider is not optional |
| GP12 (pin 16) | Button, other leg to GND | internal pull-up |
| 3V3 (pin 36) | BME280, MPU-6050, OLED VCC | |
| VBUS (pin 40) | SRF05 VCC | 5V |
| GND (pin 38) | Common ground | every module |

<!-- TODO: docs/wiring.md -->

## Quickstart

```bash
docker compose up -d          # Mosquitto broker + Grafana, one command
cargo run --release           # from firmware/, per the workspace .cargo/config.toml
./watch-defmt.sh target/thumbv6m-none-eabi/release/firmware
```

On first boot, the device has no Wi-Fi credentials and drops into the provisioning console over the same USB-serial connection:

```text
> wifi-ssid MyNetwork
> wifi-pass ********
> broker 192.168.1.50
> save
```

Credentials are written to flash, never to any file in this repo. After `save`, the device connects, starts publishing, and the OLED shows link/broker status. Open Grafana at `http://localhost:3000` to see live telemetry.

Single button press forces an immediate reconnect attempt (useful for testing backoff without waiting); three-or-more clicks reboots into the bootloader, same convention as every earlier project.

## Architecture

```text
sensor_task  ──┐
imu_task     ──┼─→ PubSubChannel<Measurement> ──┬─→ display_task (renders latest)
range_task   ──┘                                └─→ uplink_task (batches → MQTT)
                                                       ↑
net_task (cyw43 + embassy-net stack)  ─────────────────┘
console_task (CDC-ACM: wifi creds, broker, interval) → config store (flash)
watchdog_task (feeds only if all tasks heartbeat)
```

Three independent sensor tasks publish into one shared channel; two independent consumers (display, uplink) read from it without knowing or caring which sensor produced a given reading. Networking, provisioning, and watchdog supervision are each their own task. See `docs/adr/` for the reasoning behind specific choices as they're made, and ask for the deep-dive explanation of how the pieces actually fit together if the diagram alone isn't enough.

## Testing

```bash
cargo test -p motion-core      # reused from srf05-imu-station, unchanged
cargo test -p station-core     # reused from pico-weather-station, unchanged
cargo clippy --workspace --all-features -- -D warnings
cargo fmt --all -- --check
```

No new pure-logic crate in this project — the math (filters, calibration, dew point) was already built and tested in the two prior repos and is reused here as-is. What's new in this project (task orchestration, MQTT, reconnection backoff) is inherently integration-level behavior that needs real hardware and a real network to verify, so it isn't host-testable the way earlier projects' driver logic was.

## Known limitations

- No debug probe support — see [`blinky-plus`](../blinky-plus) for what that means in practice for this series.
- Telemetry payload schema is documented but not versioned yet — a future schema change would need a compatibility plan this project doesn't have.
- Chaos testing (M6) covers a router reboot; it doesn't yet cover a broker going down independently of the network link, which is a meaningfully different failure mode.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
