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
    Client,
    DEFAULT_PID,
    DEFAULT_VID,
    VENDOR_USAGE,
    VENDOR_USAGE_PAGE,
    find_device,
};
use vkbm_proto::{
    Button,
    Command,
};

#[derive(Parser)]
#[command(name = "vkbm", version, about = "versa-kbm companion CLI")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
    /// USB vendor id override.
    #[arg(long, global = true, default_value_t = DEFAULT_VID)]
    vid: u16,
    /// USB product id override.
    #[arg(long, global = true, default_value_t = DEFAULT_PID)]
    pid: u16,
    /// Ack timeout override, in milliseconds.
    #[arg(long, global = true, default_value_t = 250)]
    timeout_ms: u64,
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
        x: u16,
        /// Absolute Y.
        y: u16,
    },
    /// Move to an absolute position and left-click it.
    Press {
        /// Absolute X.
        x: u16,
        /// Absolute Y.
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
    /// Press, release or click a mouse button at the current position.
    Mouse {
        /// What to do with the button.
        action: MouseAction,
        /// Which button.
        #[arg(default_value = "left")]
        button: MouseButtonName,
    },
    /// Scroll: vertical ticks, optional horizontal (pan) ticks.
    Wheel {
        /// Vertical ticks (positive = up).
        v: i8,
        /// Horizontal ticks (positive = right).
        h: Option<i8>,
    },
    /// Re-enable actuation after a disarm.
    Arm,
    /// Soft killswitch: the device refuses actuation until armed again.
    Disarm,
    /// Release every held key and button.
    Release,
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
                    request(&mut client, Command::MouseDown { button }, timeout)?;
                    std::thread::sleep(Duration::from_millis(25));
                    request(&mut client, Command::MouseUp { button }, timeout)?;
                    return Ok(());
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
    }
}

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
    let mut client = open(api, vid, pid)?;
    match client.request(&Command::Info, timeout)? {
        vkbm_proto::Reply::Info { major, minor } => println!("vkbm firmware v{major}.{minor}"),
        other => bail!("unexpected reply to INFO: {other:?}"),
    }
    Ok(())
}

fn act(api: &HidApi, vid: u16, pid: u16, timeout: Duration, cmd: Command) -> Result<()> {
    let mut client = open(api, vid, pid)?;
    let reply = request(&mut client, cmd, timeout)?;
    println!("{reply:?}");
    Ok(())
}

fn request(client: &mut Client, cmd: Command, timeout: Duration) -> Result<vkbm_proto::Reply> {
    let reply = client.request(&cmd, timeout)?;
    if let vkbm_proto::Reply::Err(code) = reply {
        bail!("device refused {cmd:?}: {code:?}");
    }
    Ok(reply)
}

fn open(api: &HidApi, vid: u16, pid: u16) -> Result<Client> {
    if find_device(api, vid, pid).is_none() {
        bail!("no versa-kbm vendor channel on {vid:04x}:{pid:04x} (plugged in? flashed?)");
    }
    Ok(Client::open(api, vid, pid)?)
}
