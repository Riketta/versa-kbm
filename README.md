# versa-kbm

RP2040 (Pico) composite HID device - virtual keyboard + **absolute mouse** -
commanded by the host over a vendor HID control channel. The actuation
backend for [sphinx](../sphinx) (pixel-semaphore auto-buy for AuctiMogger).

## Layout

| Crate | Kind | Purpose |
|---|---|---|
| `crates/vkbm-proto` | `no_std` lib | The wire contract: command/reply vocabulary + both encodings (binary frames for HID, ASCII lines for the CDC bench channel). Shared by firmware and host. |
| `crates/vkbm-cli` | lib + bin | Host-side client (`vkbm.exe`) for bring-up, calibration, and manual actuation. |
| `crates/vkbm-fw` | firmware | Embassy RP2040 firmware. Standalone crate: own target (`thumbv6m-none-eabi`), own lockfile. |

## Control channel

Production: HID vendor usage page `0xFF00`, 64-byte output/input reports,
`vkbm_proto::frame` layout, ack + one-resend. `dev-cdc` firmware feature adds
a CDC-ACM serial port speaking the same commands as ASCII lines
(`vkbm_proto::line`) for bench debugging.

Identity is a placeholder (Logitech Unifying-style VID/PID) until the M2
identity pass.

## Build

```sh
cargo test                       # host: proto + cli
cargo clippy --all-targets -- -D warnings
cargo +nightly fmt --all -- --check
cd crates/vkbm-fw
cargo build                      # thumbv6m-none-eabi, flashes via picotool
```

Firmware flashing uses the picotool runner wired in
`crates/vkbm-fw/.cargo/config.toml` (`cargo run` / `cargo objcopy` from that
directory).

## Status

M3 (actuation): both channels drive the actuator loop - keyboard (6KRO +
modifiers), absolute mouse, wheel, compound taps/clicks with configurable
pacing, soft killswitch (`DISARM`), `RA` release-all (works disarmed).
Bench: `vkbm move/press/key/mouse/wheel/arm/disarm/release`.
See `../PLAN.md` for the milestone plan.
