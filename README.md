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
  acked with sequence numbers; one automatic resend on a lost ack, and the
  device replays the cached reply for a resent sequence instead of
  re-executing it.
- Full command set: key down/up/tap, absolute and relative mouse movement,
  button down/up, move-and-click, wheel/pan scrolling, release-all, pacing.
- Soft killswitch: `DISARM` refuses every actuation command until `ARM`.
- Hardware master switch: a tap on BOOTSEL disables actuation at the device
  itself; software cannot clear it. Switching off also releases every held
  key and button. Releases (`RELEASE`, key/button up) always work, no
  matter how either killswitch is set.
- Configurable pacing between report steps (`vkbm pacing inter hold`).
- Optional CDC-ACM bench port (`dev-cdc` feature): same commands as plain
  ASCII lines, usable from any serial terminal (CRLF tolerated).

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

Firmware (toolchain and flashing setup: see [Flashing](#flashing)):

```sh
cd crates/vkbm-fw
cargo build --features dev-cdc        # dev build with the CDC bench port
cargo run --features dev-cdc          # build + flash via picotool
```

The `dev-cdc` feature adds the bench serial port — production builds (no
features) enumerate as HID only.

## Flashing

Prerequisites: the `thumbv6m-none-eabi` target (`rustup target add
thumbv6m-none-eabi`) and `picotool` on your `PATH` (if it lives elsewhere,
point the runner at it in `crates/vkbm-fw/.cargo/config.toml`).

**First flash** (blank board, or recovery): hold **BOOTSEL** while plugging
the board into USB — it comes up in the stock bootloader. Then, from
`crates/vkbm-fw`:

```sh
cargo run --features dev-cdc   # build + flash + boot the new image
```

The runner (`picotool load --update --verify --execute -t elf`, see
`crates/vkbm-fw/.cargo/config.toml`) finds the bootloader-mode board,
flashes the freshly built ELF and reboots straight into the firmware.

**Reflashing** works the same way: the running firmware cannot be
reprogrammed through the vendor channel, so always put the board into
BOOTSEL mode first — unplug, hold BOOTSEL, plug back in — then `cargo run`
again.

**Verify** with the CLI: `vkbm ping` for a handshake, `vkbm info` — a
freshly flashed board reports the firmware crate's own version
(`crates/vkbm-fw` is versioned independently of the host crates') and
`switch=on arm=on`.

## CLI usage

```sh
vkbm list                 # enumerate HID devices, mark the versa-kbm channel
vkbm ping                 # handshake + round-trip latency
vkbm info                 # firmware version, switch and arm gate state
vkbm move 16384 16384     # pointer to center (coordinates are 0..=32767)
vkbm move-rel -100 0      # pointer by a delta (anchored to the device's last position)
vkbm press 4096 4096      # move + left-click
vkbm key 44               # tap space (HID usage code; 41 = ESC)
vkbm mouse click right    # click a button at the current position
vkbm wheel 3              # scroll up; horizontal: vkbm wheel 0 -3
vkbm pacing 4 10          # step gap / button hold, in milliseconds
vkbm disarm               # killswitch: actuation refused until...
vkbm arm                  # ...re-armed (vkbm release lets go of everything)
```

Global overrides: `--vid`, `--pid` (decimal or `0x`-prefixed hex),
`--timeout-ms`.

With the `dev-cdc` firmware feature you can also talk to the device from any
serial terminal at the bench port — one command per line (`PING`, `INFO`,
`CA 16384 16384`, `RA`, ...), replies come back as `OK`, `VKBM x.y switch=..
arm=..` (for `INFO`), or `ERR ...` with a reason code.

## Hardware kill switch (BOOTSEL)

A short tap on the board's BOOTSEL button toggles the device on or off
independently of any software:

- **Off:** every actuation command is refused with `ERR DISABLED`; held
  keys and buttons are released the moment the toggle lands, and any
  actuation still queued but not yet executed is dropped at the gate (only
  a command already mid-sequence can finish). The host can see the state
  (`vkbm info` prints `switch=off`) but cannot clear it.
- **On:** the device acts normally, still subject to the soft
  `DISARM`/`ARM` gate.
- The board LED is solid only when the device can actuate (switch on and
  armed); it blinks slowly whenever either gate is closed.
- Holding BOOTSEL during power-up still enters the stock USB bootloader
  (recovery/flashing mode); only short taps while running toggle the
  switch.

BOOTSEL is wired to the flash CS line, so firmware samples it by floating
that line for a few microseconds from a RAM-resident routine (the pico-sdk
recipe); USB traffic merely pauses for that instant.

## Wire protocol

One command per exchange; the device acks every frame with `OK` or an
`ERR ...` code. Binary frames are exactly 64 bytes: sequence byte, command
tag, little-endian payload, zero padding — see `crates/vkbm-proto/src/frame.rs`
for the tag table and `src/line.rs` for the ASCII encoding. Both encodings are
generated from one command vocabulary and covered by shared tests.

The device deduplicates by sequence number: because it acks *after*
execution, a paced command can outlive the host's ack timeout, and the
automatic resend would otherwise execute twice. A repeated sequence replays
the cached reply instead. If you raise pacing above the ack timeout
(`--timeout-ms`, default 250), raise the timeout with it.

Two safeguards keep a repeated sequence from ever swallowing a fresh
command: the host seeds each process's sequence counter from clock and
process id (so a new CLI session almost never starts at the number the
last session left cached), and the device only replays a repeated
sequence within a 2 s window of its first execution (a true resend
arrives within the ack timeout, well under that window). A fresh
session's first command therefore always executes.

Absolute coordinates travel as `0..=32767` (`ABS_MAX`) per axis; values
above that are clamped by the device. Wheel and pan ticks are `i8` on
the wire, clamped by the firmware to the −127..=127 range its HID
descriptor declares.

## Device identity

The USB identity (VID/PID and strings) is defined by constants near the top
of `crates/vkbm-fw/src/main.rs`, mirrored in the CLI's discovery defaults.
By default the device poses as a generic 2.4 GHz wireless receiver
(`258a:0100`, "2.4G Wireless Receiver", no manufacturer string) — the kind
of anonymous combo dongle that ships with cheap keyboard/mouse sets.
Change the constants if your use case needs a different appearance.

## Security & limitations

- The control channel has **no authentication**: any local process that can
  open the device's HID interface can arm it and send commands. The soft
  killswitch (`DISARM`) guards against accidents, not hostile local
  software — unplug the device when that matters. The BOOTSEL hardware
  switch *does* bind hostile software (it cannot be toggled over USB), at
  the cost of physical access to flip it back.
- The USB identity is a fixed constant near the top of the firmware; it is
  not a guarantee of anything, only an appearance.
- Relative movement (`move-rel`, line `MR`) is anchored to the last position
  the device itself reported, not the OS cursor; another pointer on the
  target machine makes it land elsewhere. Prefer absolute moves.
- A key or button held across a failed command stays latched;
  `vkbm release` (line `RA`) always clears everything, even disarmed.
- Tapping an already-held key or button releases it (a tap is
  press + release).
- If the firmware itself panics, the last HID report stays latched until
  the board is replugged: the panic handler halts the core. No reachable
  panic paths are known, but a halted device cannot release anything.
- Keyboard LED output reports (caps/num-lock state from the host) are not
  consumed; the virtual keyboard does not mirror them.

## Status

Working: firmware actuation engine, soft and hardware killswitches, both
control channels, host CLI and protocol tests. Hardware validation is in
progress.
