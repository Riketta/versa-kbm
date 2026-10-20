//! `vkbm` - companion CLI for the versa-kbm RP2040 device.

use std::time::{
    Duration,
    Instant,
};

use anyhow::{
    Context,
    Result,
    bail,
};
use clap::{
    Parser,
    Subcommand,
    ValueEnum,
};
use hidapi::HidApi;
use vkbm_cli::{
    ACK_TIMEOUT_MS,
    Client,
    DEFAULT_PID,
    DEFAULT_VID,
    VENDOR_USAGE,
    VENDOR_USAGE_PAGE,
    find_device,
};
use vkbm_proto::{
    ABS_MAX,
    Button,
    Command,
};

#[derive(Parser)]
#[command(name = "vkbm", version, about = "versa-kbm companion CLI")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
    /// USB vendor id override (decimal, or hex with a 0x prefix).
    #[arg(long, global = true, default_value_t = DEFAULT_VID, value_parser = usb_id)]
    vid: u16,
    /// USB product id override (decimal, or hex with a 0x prefix).
    #[arg(long, global = true, default_value_t = DEFAULT_PID, value_parser = usb_id)]
    pid: u16,
    /// Ack timeout override, in milliseconds (`1..=60000`; the cap keeps
    /// the ack deadline arithmetic far from overflow).
    #[arg(
        long,
        global = true,
        default_value_t = ACK_TIMEOUT_MS,
        value_parser = clap::value_parser!(u64).range(1..=60_000)
    )]
    timeout_ms: u64,
}

/// Parses a USB id: decimal, or hex with a `0x` prefix (`0x258a`).
fn usb_id(s: &str) -> Result<u16, String> {
    let parsed = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16)
    } else {
        s.parse::<u16>()
    };
    parsed.map_err(|_| format!("invalid USB id {s:?} (decimal, or hex with a 0x prefix)"))
}

#[derive(Subcommand)]
enum Cmd {
    /// List HID devices, marking the versa-kbm vendor control channel.
    List,
    /// Handshake: send PING and report the round-trip latency.
    Ping,
    /// Query the firmware version.
    Info,
    /// Move the pointer to an absolute position (`0..=32767` per axis).
    Move {
        /// Absolute X.
        #[arg(value_parser = clap::value_parser!(u16).range(0..=i64::from(ABS_MAX)))]
        x: u16,
        /// Absolute Y.
        #[arg(value_parser = clap::value_parser!(u16).range(0..=i64::from(ABS_MAX)))]
        y: u16,
    },
    /// Move the pointer by a relative delta. The OS applies its pointer
    /// ballistics to relative reports, so the landed offset can differ from
    /// the raw delta; use `move` when exact placement matters.
    MoveRel {
        /// Signed X delta.
        #[arg(allow_negative_numbers = true)]
        dx: i16,
        /// Signed Y delta.
        #[arg(allow_negative_numbers = true)]
        dy: i16,
    },
    /// Move to an absolute position and left-click it.
    Press {
        /// Absolute X.
        #[arg(value_parser = clap::value_parser!(u16).range(0..=i64::from(ABS_MAX)))]
        x: u16,
        /// Absolute Y.
        #[arg(value_parser = clap::value_parser!(u16).range(0..=i64::from(ABS_MAX)))]
        y: u16,
    },
    /// Send a keyboard usage (HID usage code): tap by default.
    Key {
        /// HID keyboard usage code (e.g. 41 = ESC, 44 = space, 229 = right shift).
        usage: u8,
        /// Press and hold instead of tapping.
        #[arg(long, group = "key_mode")]
        down: bool,
        /// Release a held usage.
        #[arg(long, group = "key_mode")]
        up: bool,
    },
    /// Press, release or click a mouse button through the pointer interface
    /// that moved last: at the position `move`/`press` placed, at the
    /// current cursor after `move-rel`, at screen center right after boot
    /// (the device cannot see the OS cursor - `move` first when placement
    /// matters).
    Mouse {
        /// What to do with the button.
        action: MouseAction,
        /// Which button.
        #[arg(default_value = "left")]
        button: MouseButtonName,
    },
    /// Scroll: vertical ticks, optional horizontal (pan) ticks. The wheel
    /// rides the pointer that moved last, and only the absolute pointer has
    /// a pan axis: after a `move-rel`, a pan-carrying wheel is refused
    /// (`ERR UNDELIVERABLE`) - `move` first to re-anchor it.
    Wheel {
        /// Vertical ticks (positive = up).
        #[arg(allow_negative_numbers = true)]
        v: i8,
        /// Horizontal ticks (positive = right).
        #[arg(allow_negative_numbers = true)]
        h: Option<i8>,
    },
    /// Re-enable actuation after a disarm.
    Arm,
    /// Soft killswitch: the device refuses actuation until armed again.
    Disarm,
    /// Release every held key and button.
    Release,
    /// Set actuation pacing: gap between sequenced report steps and
    /// button-hold time, in milliseconds. High values make paced commands
    /// (`press`, `key`, `wheel`, and chunked `move-rel` streams) outlive the
    /// ack timeout; the device dedups resends by completion time, but raise
    /// `--timeout-ms` to match if you raise pacing.
    Pacing {
        /// Gap between sequenced report steps, in milliseconds.
        inter_ms: u8,
        /// Button-hold time for clicks and taps, in milliseconds.
        hold_ms: u8,
    },
    /// Toggle dry-run mode: actuation commands run the full command path and
    /// are echoed back instead of physically pressing keys, moving the
    /// pointer or clicking. Off after a reboot.
    DryRun {
        /// Turn dry-run on or off.
        state: OnOff,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum OnOff {
    /// Enable.
    On,
    /// Disable.
    Off,
}

#[derive(Clone, Copy, ValueEnum)]
enum MouseAction {
    /// Press and hold.
    Down,
    /// Release.
    Up,
    /// Press, hold for the pacing interval, release.
    Click,
}

#[derive(Clone, Copy, ValueEnum)]
enum MouseButtonName {
    /// Primary button.
    Left,
    /// Secondary button.
    Right,
    /// Middle button.
    Middle,
    /// Fourth button (back).
    Back,
    /// Fifth button (forward).
    Forward,
}

impl From<MouseButtonName> for Button {
    fn from(value: MouseButtonName) -> Self {
        match value {
            MouseButtonName::Left => Self::Left,
            MouseButtonName::Right => Self::Right,
            MouseButtonName::Middle => Self::Middle,
            MouseButtonName::Back => Self::Back,
            MouseButtonName::Forward => Self::Forward,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let timeout = Duration::from_millis(cli.timeout_ms);
    let api = HidApi::new().context("hidapi init")?;
    match cli.cmd {
        Cmd::List => list(&api, cli.vid, cli.pid),
        Cmd::Ping => ping(&api, cli.vid, cli.pid, timeout),
        Cmd::Info => info(&api, cli.vid, cli.pid, timeout),
        Cmd::Move { x, y } => act(&api, cli.vid, cli.pid, timeout, Command::MouseMoveAbs { x, y }),
        Cmd::MoveRel { dx, dy } => {
            // The device acks only after the paced <=127-count chunk stream
            // finishes, so the deadline grows with the requested delta.
            let chunks = u64::from(dx.unsigned_abs().max(dy.unsigned_abs())).div_ceil(127);
            let stream_ms = chunks.saturating_mul(CHUNK_STREAM_MS);
            act(
                &api,
                cli.vid,
                cli.pid,
                timeout.saturating_add(Duration::from_millis(stream_ms)),
                Command::MouseMoveRel { dx, dy },
            )
        }
        Cmd::Press { x, y } => act(&api, cli.vid, cli.pid, timeout, Command::ClickAbs { x, y }),
        Cmd::Key { usage, down, up } => {
            let cmd = if down {
                Command::KeyDown { usage }
            } else if up {
                Command::KeyUp { usage }
            } else {
                Command::KeyTap { usage }
            };
            act(&api, cli.vid, cli.pid, timeout, cmd)
        }
        Cmd::Mouse { action, button } => {
            let button = Button::from(button);
            let cmd = match action {
                MouseAction::Down => Command::MouseDown { button },
                MouseAction::Up => Command::MouseUp { button },
                MouseAction::Click => {
                    let mut client = open(&api, cli.vid, cli.pid)?;
                    let clicked = (|| -> Result<()> {
                        request(&mut client, Command::MouseDown { button }, timeout)?;
                        std::thread::sleep(Duration::from_millis(CLICK_HOLD_MS));
                        request(&mut client, Command::MouseUp { button }, timeout)?;
                        Ok(())
                    })();
                    if clicked.is_err() {
                        // Never leave a button latched on the target: the
                        // device keeps key/mouse state until released.
                        let _ = client.request(&Command::ReleaseAll, timeout);
                        eprintln!("click failed; sent release-all as a safety net");
                    }
                    return clicked;
                }
            };
            act(&api, cli.vid, cli.pid, timeout, cmd)
        }
        Cmd::Wheel { v, h } => {
            act(&api, cli.vid, cli.pid, timeout, Command::Wheel { v, h: h.unwrap_or(0) })
        }
        Cmd::Arm => act(&api, cli.vid, cli.pid, timeout, Command::Arm),
        Cmd::Disarm => act(&api, cli.vid, cli.pid, timeout, Command::Disarm),
        Cmd::Release => act(&api, cli.vid, cli.pid, timeout, Command::ReleaseAll),
        Cmd::Pacing { inter_ms, hold_ms } => {
            act(&api, cli.vid, cli.pid, timeout, Command::SetPacing { inter_ms, hold_ms })
        }
        Cmd::DryRun { state } => act(
            &api,
            cli.vid,
            cli.pid,
            timeout,
            Command::DryRun { enabled: matches!(state, OnOff::On) },
        ),
    }
}

/// Host-side button-hold time for `mouse click` (device pacing applies to
/// `press`/`key`; this two-step click is paced here).
const CLICK_HOLD_MS: u64 = 25;

/// Per-chunk stream cost used to size `move-rel` deadlines: the device
/// paces the <=127-count reports at `inter_ms` (4 ms default) and each
/// report waits up to one endpoint poll interval (4 ms).
const CHUNK_STREAM_MS: u64 = 8;

/// Enumerates HID devices. Infallible today, but kept `Result` so every
/// subcommand keeps the same dispatch shape in `main`.
#[allow(clippy::unnecessary_wraps)]
fn list(api: &HidApi, vid: u16, pid: u16) -> Result<()> {
    for d in api.device_list() {
        let ours = d.vendor_id() == vid
            && d.product_id() == pid
            && d.usage_page() == VENDOR_USAGE_PAGE
            && d.usage() == VENDOR_USAGE;
        let marker = if ours { "  <= versa-kbm vendor channel" } else { "" };
        println!(
            "{:04x}:{:04x} usage_page={:#06x} usage={:#04x} if={} product={:?} serial={:?}{}",
            d.vendor_id(),
            d.product_id(),
            d.usage_page(),
            d.usage(),
            d.interface_number(),
            d.product_string(),
            d.serial_number(),
            marker,
        );
    }
    Ok(())
}

fn ping(api: &HidApi, vid: u16, pid: u16, timeout: Duration) -> Result<()> {
    let mut client = open(api, vid, pid)?;
    let started = Instant::now();
    let reply = client.request(&Command::Ping, timeout)?;
    println!("ping acked in {:?}: {reply:?}", started.elapsed());
    Ok(())
}

fn info(api: &HidApi, vid: u16, pid: u16, timeout: Duration) -> Result<()> {
    use vkbm_proto::info_flags;
    let mut client = open(api, vid, pid)?;
    match client.request(&Command::Info, timeout)? {
        vkbm_proto::Reply::Info { major, minor, flags } => {
            let switch = if flags & info_flags::SWITCH_ON != 0 { "on" } else { "off" };
            let arm = if flags & info_flags::ARMED != 0 { "on" } else { "off" };
            let dry = if flags & info_flags::DRY_RUN != 0 { "on" } else { "off" };
            println!("vkbm firmware v{major}.{minor} switch={switch} arm={arm} dry={dry}");
        }
        other => bail!("unexpected reply to INFO: {other:?}"),
    }
    Ok(())
}

fn act(api: &HidApi, vid: u16, pid: u16, timeout: Duration, cmd: Command) -> Result<()> {
    let mut client = open(api, vid, pid)?;
    let reply = request(&mut client, cmd, timeout)?;
    match reply {
        vkbm_proto::Reply::Echo => {
            println!("dry-run echo: {cmd:?} (nothing actuated)");
        }
        other => println!("{other:?}"),
    }
    Ok(())
}

fn request(client: &mut Client, cmd: Command, timeout: Duration) -> Result<vkbm_proto::Reply> {
    let reply = client.request(&cmd, timeout)?;
    if let vkbm_proto::Reply::Err(code) = reply {
        let hint = match code {
            // The only gate the host cannot clear: point at the button.
            vkbm_proto::ErrorCode::Disabled => {
                " (hardware switch is off: tap BOOTSEL on the board to toggle it back on)"
            }
            _ => "",
        };
        bail!("device refused {cmd:?}: {code:?}{hint}");
    }
    Ok(reply)
}

fn open(api: &HidApi, vid: u16, pid: u16) -> Result<Client> {
    if find_device(api, vid, pid).is_none() {
        bail!("no versa-kbm vendor channel on {vid:04x}:{pid:04x} (plugged in? flashed?)");
    }
    Ok(Client::open(api, vid, pid)?)
}
