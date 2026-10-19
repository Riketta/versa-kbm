# versa-kbm

Virtual keyboard and mouse on an RP2040 (Raspberry Pi Pico). The device
enumerates as an ordinary composite HID device — a keyboard plus an
**absolute mouse** — and executes commands sent by a host application over
a vendor-defined HID control channel. Ships with a companion CLI (`vkbm`)
for bring-up, bench testing, and manual actuation.

## Why an absolute mouse

Most HID mice report *relative* motion, which Windows rescales with pointer
acceleration and which cannot address a position directly. versa-kbm's mouse
reports **absolute coordinates** (16-bit X/Y across the desktop) — the same
convention KVM-over-IP devices (PiKVM & co) use. The pointer lands exactly
where the host says, no acceleration is applied, and no convergence loop is
needed.

## Features

- Composite HID: keyboard (6-key rollover + modifiers) and absolute mouse
  (5 buttons, wheel, horizontal pan).
- Control channel: HID vendor usage page `0xFF00`, 64-byte binary frames,
  acked with sequence numbers; one automatic resend on a lost ack.
- Full command set: key down/up/tap, absolute and relative mouse movement,
  button down/up, move-and-click, wheel/pan scrolling, release-all.
- Soft killswitch: `DISARM` refuses every actuation command until `ARM`;
  board LED shows the armed state. `RELEASE` always works, even disarmed.
- Configurable pacing between report steps (human-plausible timing).
- Optional CDC-ACM bench port (`dev-cdc` feature): same commands as plain
  ASCII lines, usable from any serial terminal.

## Repository layout

| Crate | Kind | Purpose |
|---|---|---|
| `crates/vkbm-proto` | `no_std` lib | The wire contract: command vocabulary + both encodings (binary frames for HID, ASCII lines for the CDC bench channel). Shared by firmware and host. |
| `crates/vkbm-fw` | firmware | Embassy RP2040 firmware. Standalone crate: own target (`thumbv6m-none-eabi`), own lockfile. |
| `crates/vkbm-cli` | lib + bin | Host-side client library and the `vkbm` command-line tool. |

## Building

Host tooling (stable Rust):

```sh
cargo build --release -p vkbm-cli     # the vkbm CLI
cargo test                            # protocol + client tests
```

Firmware (`thumbv6m-none-eabi` target required: `rustup target add
thumbv6m-none-eabi`; picotool on PATH for flashing):

```sh
cd crates/vkbm-fw
cargo build --features dev-cdc        # dev build with the CDC bench port
cargo run --features dev-cdc          # build + flash via picotool
```

Flash a blank Pico with BOOTSEL held; afterwards `cargo run` reflashes
directly. The `dev-cdc` feature adds the bench serial port — production
builds (no features) enumerate as HID only.

## CLI usage

```sh
vkbm list                 # enumerate HID devices, mark the versa-kbm channel
vkbm ping                 # handshake + round-trip latency
vkbm info                 # firmware version
vkbm move 16384 16384     # pointer to center (coordinates are 0..=32767)
vkbm press 4096 4096      # move + left-click
vkbm key 44               # tap space (HID usage code; 41 = ESC)
vkbm mouse click right    # click a button at the current position
vkbm wheel 3              # scroll up; horizontal: vkbm wheel 0 -3
vkbm disarm               # killswitch: actuation refused until...
vkbm arm                  # ...re-armed (vkbm release lets go of everything)
```

Global overrides: `--vid`, `--pid`, `--timeout-ms`.

With the `dev-cdc` firmware feature you can also talk to the device from any
serial terminal at the bench port — one command per line (`PING`, `INFO`,
`CA 16384 16384`, `RA`, ...), replies come back as `OK` / `ERR ...`.

## Wire protocol

One command per exchange; the device acks every frame with `OK` or an
`ERR ...` code. Binary frames are exactly 64 bytes: sequence byte, command
tag, little-endian payload, zero padding — see `crates/vkbm-proto/src/frame.rs`
for the tag table and `src/line.rs` for the ASCII encoding. Both encodings are
generated from one command vocabulary and covered by shared tests.

## Device identity

The USB identity (VID/PID and strings) is defined by constants near the top
of `crates/vkbm-fw/src/main.rs`, mirrored in the CLI's discovery defaults.
By default the device poses as a generic 2.4 GHz wireless receiver
(`258a:0100`, "2.4G Wireless Receiver", no manufacturer string) — the kind
of anonymous combo dongle that ships with cheap keyboard/mouse sets.
Change the constants if your use case needs a different appearance.

## Status

Working: firmware actuation engine, killswitch, both control channels, host
CLI and protocol tests. Hardware validation is in progress.
