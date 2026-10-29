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
- **Flash-backed configuration** — Wi-Fi credentials, broker address and publish interval live in two A/B flash slots (CRC-32 + sequence number, so a power cut mid-write can't lose the config). They are supplied at build time through environment variables (`secrets.env`, git-ignored) and written to flash on first boot; later builds without those variables reuse what's stored, so release binaries contain no credentials (CI checks for that).
- **Watchdog fed only when every task has heartbeated** — this matters more here than in any earlier project: a hung async task with no probe attached is otherwise invisible until the OLED visibly stops updating.
- **`docker-compose.yml`** spins up Mosquitto + Grafana so anyone cloning this can see live telemetry in a dashboard with one command.
- **No probe, no USB serial.** Observability comes from the OLED status screen, onboard-LED patterns, the `hub/log` MQTT topic and the broker's own log. (A USB logger was tried and dropped — see *Known limitations*.) `panic-persist` still keeps the last panic message in RAM across resets.
- **Shared async I²C bus + `PubSubChannel`** — three devices (BME280, MPU-6050, OLED) share one bus through a mutex; every sensor task publishes `Measurement` values and the OLED and MQTT uplink each subscribe independently.
- **Hand-written MQTT 3.1.1 publisher** (`crates/mqtt-lite`, host-tested): CONNECT, QoS 0 PUBLISH, keep-alive. No published client crate matched `embedded-io-async` 0.7 + `embassy-net` 0.9 without a heap; see `docs/adr/`.

## To-Do list

- [x] `cyw43` up, onboard LED blinks
- [x] DHCP + ping, confirmed reachable on the local network
- [x] MQTT publish of one sensor (BME280)
- [x] all three sensors flowing through a shared `PubSubChannel` (OLED as second subscriber)
- [x] flash-backed config store (A/B slots, host-tested `hub-config`)
- [ ] reconnection backoff + watchdog, chaos-tested (pull the router mid-run)
- [ ] Grafana dashboard (`docker-compose.yml` with Mosquitto already works)
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
| GP13 | Status LED (optional) | on while an MQTT session is up |
| GP12 (pin 16) | Button, other leg to GND | internal pull-up |
| 3V3 (pin 36) | BME280, MPU-6050, OLED VCC | |
| VBUS (pin 40) | SRF05 VCC | 5V |
| GND (pin 38) | Common ground | every module |

<!-- TODO: docs/wiring.md -->

## Quickstart

```bash
docker compose up -d                       # Mosquitto broker (Grafana joins later)
cp secrets.env.example secrets.env         # edit it; it is git-ignored
source secrets.env && cargo run --release  # from firmware/, per the workspace .cargo/config.toml
```

`secrets.env.example` documents the three variables:

```bash
export WIFI_SSID="your-network"
export WIFI_PASS="your-passphrase"         # 8–63 characters (WPA2)
export BROKER_IP="192.168.1.50"            # the machine running docker compose
```

On the first flash the configuration is written to flash. After that, builds *without* these variables reuse the stored values, so release binaries carry no credentials. Building with different values overwrites the stored ones. To erase the stored configuration entirely, flash Raspberry Pi's `flash_nuke.uf2`; with no configuration the board blinks the onboard LED 5 times in a loop.

Watch the telemetry:

```bash
docker compose exec mosquitto mosquitto_sub -t 'hub/#' -v
```

| Topic | Content |
|---|---|
| `hub/bme280` | `{"t_c":..,"rh":..,"p_hpa":..}` every interval |
| `hub/imu` | `{"pitch":..,"roll":..}` at ~5 Hz |
| `hub/range` | `{"mm":..}` (null when out of range) |
| `hub/log` | status and error strings from the firmware |

Pressing the button (GP12) reboots into the bootloader so you can flash without unplugging.

## Architecture

```text
bme280_task ─┐
imu_task    ─┼─→ PubSubChannel<Measurement> ──┬─→ display_task (OLED status screen)
range_task  ─┘                                └─→ uplink (MQTT session)
                                                       ↑
net_task + cyw43_task (Wi-Fi chip + embassy-net)  ─────┘
boot: hub-config (flash A/B slots) → Wi-Fi credentials, broker, interval
watchdog_task — fed only when every task has heartbeated   (step 6)
```

Three independent sensor tasks publish into one shared channel; two independent consumers (display, uplink) read from it without knowing or caring which sensor produced a given reading. Networking, provisioning, and watchdog supervision are each their own task. See `docs/adr/` for the reasoning behind specific choices as they're made, and ask for the deep-dive explanation of how the pieces actually fit together if the diagram alone isn't enough.

## Testing

```bash
cargo test -p motion-core      # reused from srf05-imu-station, unchanged
cargo test -p mqtt-lite        # MQTT packet encoding and reconnect backoff
cargo test -p hub-config       # config format, CRC, A/B slot selection
cargo clippy --workspace --all-features -- -D warnings
cargo fmt --all -- --check
```

Two new pure-logic crates were written for this project: mqtt-lite (packet encoding, backoff policy) and hub-config (blob format, CRC, slot selection), both host-tested. What can’t be host-tested is the integration behaviour: task orchestration, the MQTT session, and reconnection, which is why step 6 has a chaos test.

## Known limitations

- No debug probe support — see [`blinky-plus`](../blinky-plus) for what that means in practice for this series.
- Telemetry payload schema is documented but not versioned yet — a future schema change would need a compatibility plan this project doesn't have.
- Chaos testing (M6) covers a router reboot; it doesn't yet cover a broker going down independently of the network link, which is a meaningfully different failure mode.
- No USB serial or USB logging. `embassy-usb-logger` worked until Wi-Fi bring-up began, then output stalled, and closing the terminal made the dev PC's USB host controller report itself dead. Not investigated further; the console was dropped and replaced by build-time provisioning.
- No runtime provisioning: changing credentials means a rebuild with new environment variables.
- The configuration is stored in plaintext flash. Anyone with physical access can read it.
- MQTT is QoS 0 publish only: no subscribe, no TLS, no authentication (the broker allows anonymous clients).
- SRF05 pulse timing is taken from async wake-up times, so scheduling jitter adds error.
- IMU calibration is repeated at every boot (2 s, board must be still).

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
