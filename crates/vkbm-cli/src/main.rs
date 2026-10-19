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
use vkbm_proto::Command;

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
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let timeout = Duration::from_millis(cli.timeout_ms);
    let api = HidApi::new().context("hidapi init")?;
    match cli.cmd {
        Cmd::List => list(&api, cli.vid, cli.pid),
        Cmd::Ping => ping(&api, cli.vid, cli.pid, timeout),
        Cmd::Info => info(&api, cli.vid, cli.pid, timeout),
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

fn open(api: &HidApi, vid: u16, pid: u16) -> Result<Client> {
    if find_device(api, vid, pid).is_none() {
        bail!("no versa-kbm vendor channel on {vid:04x}:{pid:04x} (plugged in? flashed?)");
    }
    Ok(Client::open(api, vid, pid)?)
}
