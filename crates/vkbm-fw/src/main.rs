//! versa-kbm firmware (M1 skeleton).
//!
//! RP2040 composite USB device:
//! - interface 0: HID keyboard (6KRO, report-only for now; the M3 actuator
//!   will drive it),
//! - interface 1: HID **absolute mouse** (5 buttons, wheel, pan, 16-bit
//!   X/Y over the full desktop),
//! - interface 2: HID vendor control channel (usage page `0xFF00`) speaking
//!   the `vkbm_proto::frame` protocol,
//! - with the `dev-cdc` feature: a CDC-ACM bench port speaking the same
//!   commands as `vkbm_proto::line` ASCII.
//!
//! Identity strings are a placeholder until the M2 identity pass.

#![no_std]
#![no_main]

use core::sync::atomic::{
    AtomicBool,
    AtomicU8,
    Ordering,
};

use defmt::{
    info,
    warn,
};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{
        Driver,
        InterruptHandler,
    },
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use embassy_usb::{
    Builder,
    Config,
    class::hid::{
        Config as HidConfig,
        HidReaderWriter,
        HidWriter,
        ReportId,
        RequestHandler,
        State,
    },
    control::OutResponse,
};
use panic_probe as _;
use usbd_hid::descriptor::{
    KeyboardReport,
    SerializedDescriptor,
};
use vkbm_proto::{
    Command,
    ErrorCode,
    Reply,
    frame::{
        self,
        FRAME_LEN,
        FrameError,
    },
};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Absolute-mouse report: buttons byte, wheel, pan, 16-bit X, 16-bit Y.
const MOUSE_REPORT_LEN: usize = 7;

/// u16 twin of [`MOUSE_REPORT_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 7 fits u16
const MOUSE_REPORT_LEN_U16: u16 = MOUSE_REPORT_LEN as u16;

/// u16 twin of [`FRAME_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 64 fits u16
const FRAME_LEN_U16: u16 = FRAME_LEN as u16;

/// Firmware version reported by `Info`.
const VERSION_MAJOR: u8 = 0;
/// Firmware version reported by `Info`.
const VERSION_MINOR: u8 = 1;

/// Absolute-mouse report descriptor: 5 buttons, wheel + AC Pan (relative
/// ticks), and absolute 16-bit X/Y scaled by the OS across the desktop.
///
/// Absolute pointing devices are the PiKVM/KVM-over-IP convention: exact
/// placement, and Windows applies no pointer acceleration to them.
const MOUSE_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (1)
    0x29, 0x05, //     Usage Maximum (5)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x95, 0x05, //     Report Count (5)
    0x75, 0x01, //     Report Size (1)
    0x81, 0x02, //     Input (Data, Var, Abs)
    0x95, 0x01, //     Report Count (1)
    0x75, 0x03, //     Report Size (3)
    0x81, 0x01, //     Input (Const, padding)
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x38, //     Usage (Wheel)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x06, //     Input (Data, Var, Rel)
    0x05, 0x0C, //     Usage Page (Consumer)
    0x0A, 0x38, 0x02, // Usage (AC Pan)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x06, //     Input (Data, Var, Rel)
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x16, 0x00, 0x00, // Logical Minimum (0)
    0x26, 0xFF, 0x7F, // Logical Maximum (32767)
    0x75, 0x10, //     Report Size (16)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x02, //     Input (Data, Var, Abs)
    0xC0, //   End Collection
    0xC0, // End Collection
];

/// Vendor control channel descriptor: 64-byte input and output reports on
/// vendor usage page `0xFF00`. This is the production command path.
const VENDOR_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01, // Usage (0x01)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (0x01)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x40, //   Report Count (64)
    0x81, 0x02, //   Input (Data, Var, Abs)
    0x09, 0x01, //   Usage (0x01)
    0x91, 0x02, //   Output (Data, Var, Abs)
    0xC0, // End Collection
];

static VENDOR_EVENTS: Channel<CriticalSectionRawMutex, [u8; FRAME_LEN], 4> = Channel::new();
static ARMED: AtomicBool = AtomicBool::new(true);
static PACING_INTER_MS: AtomicU8 = AtomicU8::new(4);
static PACING_HOLD_MS: AtomicU8 = AtomicU8::new(10);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // No tasks are spawned yet; the M3 actuator may claim the second core.
    let _ = spawner;
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    let driver = Driver::new(p.USB, Irqs);

    // TODO(M2): identity pass - confirm the spoofed VID/PID and strings
    // against a real composite combo receiver.
    let mut config = Config::new(0x046d, 0xc52b);
    config.manufacturer = Some("Logitech");
    config.product = Some("USB Receiver");
    config.serial_number = Some("12863144");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // All HID/CDC state must be declared before `builder`: the classes
    // register their state borrows inside it, so the states must drop first
    // (Rust drops in reverse declaration order).
    let mut kb_state = State::new();
    let mut mouse_state = State::new();
    let mut vendor_state = State::new();
    let mut vendor_handler = VendorHandler;
    #[cfg(feature = "dev-cdc")]
    let mut cdc_state = embassy_usb::class::cdc_acm::State::new();

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 256];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Keyboard: report handle only; keys are sent by the M3 actuator task.
    let kb_config = HidConfig {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 8,
        max_packet_size: 8,
    };
    let kb = HidWriter::<_, 8>::new(&mut builder, &mut kb_state, kb_config);

    // Absolute mouse: report handle only; moves are sent by the M3 actuator.
    let mouse_config = HidConfig {
        report_descriptor: MOUSE_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 4,
        max_packet_size: MOUSE_REPORT_LEN_U16,
    };
    let mouse = HidWriter::<_, MOUSE_REPORT_LEN>::new(&mut builder, &mut mouse_state, mouse_config);

    // Vendor control channel: production command path.
    let vendor_config = HidConfig {
        report_descriptor: VENDOR_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 1,
        max_packet_size: FRAME_LEN_U16,
    };
    let vendor = HidReaderWriter::<_, FRAME_LEN, FRAME_LEN>::new(
        &mut builder,
        &mut vendor_state,
        vendor_config,
    );
    let (vendor_reader, mut vendor_writer) = vendor.split();

    #[cfg(feature = "dev-cdc")]
    use embassy_usb::class::cdc_acm::CdcAcmClass;
    #[cfg(feature = "dev-cdc")]
    let cdc = CdcAcmClass::new(&mut builder, &mut cdc_state, 64);

    // Keep the report handles alive until the M3 actuator claims them; the
    // interfaces are registered in the builder by `new` above.
    let _ = &kb;
    let _ = &mouse;

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let reader_fut = vendor_reader.run(false, &mut vendor_handler);

    let logic_fut = async {
        loop {
            let raw = VENDOR_EVENTS.receive().await;
            let reply = match frame::decode(&raw) {
                Ok((_, cmd)) => handle(cmd),
                Err(FrameError::UnknownTag) => Reply::Err(ErrorCode::UnknownCommand),
                Err(_) => Reply::Err(ErrorCode::BadPayload),
            };
            let mut out = [0u8; FRAME_LEN];
            frame::encode_reply(raw[0], &reply, &mut out);
            if let Err(e) = vendor_writer.write(&out).await {
                warn!("vendor reply failed: {:?}", e);
            }
        }
    };

    #[cfg(feature = "dev-cdc")]
    let cdc_fut = cdc::run(cdc);

    info!("versa-kbm up (M1 skeleton)");

    #[cfg(feature = "dev-cdc")]
    join(usb_fut, join(reader_fut, join(logic_fut, cdc_fut))).await;
    #[cfg(not(feature = "dev-cdc"))]
    join(usb_fut, join(reader_fut, logic_fut)).await;
}

/// The USB interrupt handler task for CDC-ACM (`dev-cdc` builds).
#[cfg(feature = "dev-cdc")]
mod cdc {
    use defmt::warn;
    use embassy_rp::usb::Driver;
    use embassy_usb::class::cdc_acm::CdcAcmClass;
    use vkbm_proto::{
        ErrorCode,
        Reply,
        line::{
            self,
            LineError,
            MAX_LINE_LEN,
        },
    };

    use super::{
        USB,
        handle,
    };

    /// Worst-case reply size (line plus newline plus slack).
    const OUT_LEN: usize = MAX_LINE_LEN + 2;

    /// Serves ASCII command lines until the device is unplugged.
    pub async fn run<'a>(mut class: CdcAcmClass<'a, Driver<'a, USB>>) {
        let mut line = [0u8; MAX_LINE_LEN + 1];
        let mut len;
        loop {
            class.wait_connection().await;
            len = 0;
            loop {
                let mut pkt = [0u8; 64];
                let n = match class.read_packet(&mut pkt).await {
                    Ok(n) => n,
                    Err(e) => {
                        warn!("cdc read failed: {:?}", e);
                        break;
                    }
                };
                let mut overflow = false;
                for &b in pkt.iter().take(n) {
                    if b == b'\n' {
                        if overflow {
                            respond(&mut class, &Reply::Err(ErrorCode::BadPayload)).await;
                        } else {
                            let (used, _) = line.split_at(len);
                            match line::decode(used) {
                                Ok(cmd) => respond(&mut class, &handle(cmd)).await,
                                Err(LineError::Empty) => {}
                                Err(_) => {
                                    respond(&mut class, &Reply::Err(ErrorCode::BadPayload)).await;
                                }
                            }
                        }
                        len = 0;
                        overflow = false;
                    } else if let Some(slot) = line.split_at_mut(len).1.first_mut() {
                        *slot = b;
                        len += 1;
                    } else {
                        overflow = true;
                    }
                }
            }
        }
    }

    async fn respond<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>, reply: &Reply) {
        let mut out = [0u8; OUT_LEN];
        // encode_reply cannot overflow OUT_LEN for any reply; the fallback
        // only exists so the failure path stays honest.
        let len = if let Ok(s) = line::encode_reply(reply, &mut out) {
            s.len()
        } else {
            let fallback = b"ERR BAD-PAYLOAD";
            for (d, b) in out.iter_mut().zip(fallback.iter()) {
                *d = *b;
            }
            fallback.len()
        };
        if let Some(slot) = out.get_mut(len) {
            *slot = b'\n';
        }
        let (head, _) = out.split_at_mut(len + 1);
        if let Err(e) = class.write_packet(head).await {
            warn!("cdc write failed: {:?}", e);
        }
    }
}

/// Dispatches one decoded command to the shared M1 logic. Actuation commands
/// are recognized but refused (`NotImplemented`) until the M3 actuator lands.
fn handle(cmd: Command) -> Reply {
    let actuation = !matches!(
        cmd,
        Command::Ping
            | Command::Info
            | Command::Arm
            | Command::Disarm
            | Command::ReleaseAll
            | Command::SetPacing { .. }
    );
    if actuation && !ARMED.load(Ordering::Relaxed) {
        return Reply::Err(ErrorCode::Disarmed);
    }
    match cmd {
        // `ReleaseAll` is a no-op until the M3 actuator gives it something to
        // release; the arms will split when it grows its own reply path.
        Command::Ping | Command::ReleaseAll => Reply::Ok,
        Command::Info => Reply::Info { major: VERSION_MAJOR, minor: VERSION_MINOR },
        Command::Arm => {
            ARMED.store(true, Ordering::Relaxed);
            Reply::Ok
        }
        Command::Disarm => {
            ARMED.store(false, Ordering::Relaxed);
            Reply::Ok
        }
        Command::SetPacing { inter_ms, hold_ms } => {
            PACING_INTER_MS.store(inter_ms, Ordering::Relaxed);
            PACING_HOLD_MS.store(hold_ms, Ordering::Relaxed);
            Reply::Ok
        }
        // M3: the actuator consumes these; `MouseMoveAbs`/`ClickAbs` clamp
        // coordinates to `vkbm_proto::ABS_MAX` at the report boundary.
        Command::KeyDown { .. }
        | Command::KeyUp { .. }
        | Command::KeyTap { .. }
        | Command::MouseMoveAbs { .. }
        | Command::MouseMoveRel { .. }
        | Command::MouseDown { .. }
        | Command::MouseUp { .. }
        | Command::ClickAbs { .. }
        | Command::Wheel { .. } => Reply::Err(ErrorCode::NotImplemented),
    }
}

/// Feeds host output reports from the vendor OUT endpoint into the command
/// queue. embassy-usb delivers interrupt-OUT data through this callback.
struct VendorHandler;

impl RequestHandler for VendorHandler {
    fn get_report(&mut self, _id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        if matches!(id, ReportId::Out(_)) {
            let mut frame = [0u8; FRAME_LEN];
            let n = data.len().min(FRAME_LEN);
            let (dst, _) = frame.split_at_mut(n);
            for (d, s) in dst.iter_mut().zip(data.iter()) {
                *d = *s;
            }
            if VENDOR_EVENTS.try_send(frame).is_err() {
                // The host client resends after its ack timeout, so dropping
                // is safe; log it for bench visibility.
                warn!("vendor queue full; frame dropped (host will resend)");
            }
        }
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}

    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }
}
