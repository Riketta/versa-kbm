//! versa-kbm firmware — RP2040 composite HID virtual keyboard/mouse.
//!
//! RP2040 composite USB device:
//! - interface 0: HID keyboard (6KRO, modifier-aware),
//! - interface 1: HID **absolute mouse** (5 buttons, wheel, pan, 16-bit
//!   X/Y over the full desktop),
//! - interface 2: HID **relative mouse** (5 buttons, wheel, 8-bit rel X/Y
//!   through the OS pointer stack, ballistics included),
//! - interface 3: HID vendor control channel (usage page `0xFF00`) speaking
//!   the `vkbm_proto::frame` protocol,
//! - with the `dev-cdc` feature: a CDC-ACM bench port speaking the same
//!   commands as `vkbm_proto::line` ASCII.
//!
//! Both command channels feed one actuator loop that owns the latched
//! keyboard and the per-interface button latches, and sequences compound
//! commands (`KeyTap`, `ClickAbs`, `Wheel`, chunked `MouseMoveRel`) with the
//! configured pacing. New presses ride whichever pointer moved last, so a
//! click always lands where the host drove the cursor; each press is owned
//! by the interface it was pressed on - moves never carry another pointer's
//! buttons, and a release always rides its owner. `Disarm` (soft killswitch)
//! refuses actuation until `Arm`; `ReleaseAll` always works, even disarmed.
//! A tap on the
//! board's BOOTSEL button is a hardware master switch that disables actuation
//! outright - software can neither clear it nor override the release it
//! forces.
//!
//! The USB identity (VID/PID and strings) is a constant near the top of
//! `main` — see the README's "Device identity" section.

#![no_std]
#![no_main]

use core::sync::atomic::{
    AtomicBool,
    AtomicU8,
    Ordering,
    compiler_fence,
};

use defmt::{
    info,
    warn,
};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    Peri,
    bind_interrupts,
    gpio::{
        Level,
        Output,
    },
    peripherals::{
        PIN_25,
        USB,
    },
    usb::{
        Driver,
        InterruptHandler,
    },
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    signal::Signal,
};
use embassy_time::Timer;
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
    ABS_MAX,
    Button,
    Command,
    ErrorCode,
    Reply,
    chunks,
    dedup::SeqDedup,
    frame::{
        self,
        FRAME_LEN,
        FrameError,
    },
    info_flags,
    keyboard::KeyboardState,
    pointer::PointerLatches,
    routing,
    semver_major,
    semver_minor,
};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Absolute-mouse report: buttons byte, wheel, pan, 16-bit X, 16-bit Y.
const MOUSE_REPORT_LEN: usize = 7;

/// Relative-mouse report: buttons byte, wheel, 8-bit X delta, 8-bit Y delta.
const REL_MOUSE_REPORT_LEN: usize = 4;

/// Keyboard input report: modifier, reserved, 6 keycodes.
const KEYBOARD_REPORT_LEN: usize = 8;

/// u16 twin of [`KEYBOARD_REPORT_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 8 fits u16
const KEYBOARD_REPORT_LEN_U16: u16 = KEYBOARD_REPORT_LEN as u16;

/// u16 twin of [`MOUSE_REPORT_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 7 fits u16
const MOUSE_REPORT_LEN_U16: u16 = MOUSE_REPORT_LEN as u16;

/// u16 twin of [`REL_MOUSE_REPORT_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 4 fits u16
const REL_MOUSE_REPORT_LEN_U16: u16 = REL_MOUSE_REPORT_LEN as u16;

/// u16 twin of [`FRAME_LEN`] for descriptor config fields.
#[allow(clippy::cast_possible_truncation)] // 64 fits u16
const FRAME_LEN_U16: u16 = FRAME_LEN as u16;

/// Firmware version reported by `Info`, derived from the crate version so
/// the two cannot drift (`"0.2.0"` -> major 0, minor 2). The minor is
/// parsed in full, so two-digit minors (0.10.0+) survive the wire's u8.
const VERSION_MAJOR: u8 = semver_major(env!("CARGO_PKG_VERSION"));
/// Firmware minor version (see [`VERSION_MAJOR`]).
const VERSION_MINOR: u8 = semver_minor(env!("CARGO_PKG_VERSION"));

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

/// Relative-mouse report descriptor: 5 buttons, wheel, and 8-bit relative
/// X/Y - the bog-standard mouse collection a generic rodent reports. These
/// deltas go through the OS pointer stack (acceleration ballistics apply),
/// which is what makes closed-loop targeting monitor-independent: absolute
/// reports are normalized to the primary display, relative ones are not.
const REL_MOUSE_REPORT_DESCRIPTOR: &[u8] = &[
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
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Var, Rel)
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
/// Soft killswitch gate: `Disarm` closes it, `Arm` reopens it.
static ARMED: AtomicBool = AtomicBool::new(true);
/// Hardware master switch gate: toggled by a BOOTSEL tap at the board. While
/// off, actuation is refused and no host command can switch it back on.
static ENABLED: AtomicBool = AtomicBool::new(true);
/// Dry-run mode: actuation commands reply [`Reply::Echo`] and skip the
/// physical reports. Session-scoped: off after a reboot.
static DRY_RUN: AtomicBool = AtomicBool::new(false);
static PACING_INTER_MS: AtomicU8 = AtomicU8::new(4);
static PACING_HOLD_MS: AtomicU8 = AtomicU8::new(10);

/// Who is waiting for an actuation reply.
#[derive(Clone, Copy)]
enum Caller {
    /// The vendor-frame dispatch loop.
    Vendor,
    /// The CDC bench line loop (only constructed in `dev-cdc` builds).
    #[allow(dead_code)]
    Cdc,
    /// The hardware-switch task: its forced release is fire-and-forget.
    Switch,
}

static ACT_CMD: Channel<CriticalSectionRawMutex, (Caller, Command), 8> = Channel::new();
static VENDOR_REPLY: Signal<CriticalSectionRawMutex, Reply> = Signal::new();
static CDC_REPLY: Signal<CriticalSectionRawMutex, Reply> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    spawner.must_spawn(led_task(p.PIN_25));
    spawner.must_spawn(switch_task());
    let driver = Driver::new(p.USB, Irqs);

    // Device identity: poses as a generic vendor 2.4 GHz combo receiver —
    // anonymous cheap-OEM gear, nothing to diff against a spec sheet.
    // The CLI's discovery defaults (vkbm-cli/src/lib.rs) must match.
    let mut config = Config::new(0x258a, 0x0100);
    config.manufacturer = None;
    config.product = Some("2.4G Wireless Receiver");
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // HID/CDC state is declared before `builder`: Rust drops locals in
    // reverse declaration order, so the states drop LAST - after the USB
    // classes that borrow them are created inside `builder`. That drop
    // ordering is what the class borrows require.
    let mut kb_state = State::new();
    let mut mouse_state = State::new();
    let mut rel_mouse_state = State::new();
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

    let kb_config = HidConfig {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 8,
        max_packet_size: KEYBOARD_REPORT_LEN_U16,
    };
    let mut kb = HidWriter::<_, KEYBOARD_REPORT_LEN>::new(&mut builder, &mut kb_state, kb_config);

    let mouse_config = HidConfig {
        report_descriptor: MOUSE_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 4,
        max_packet_size: MOUSE_REPORT_LEN_U16,
    };
    let mut mouse =
        HidWriter::<_, MOUSE_REPORT_LEN>::new(&mut builder, &mut mouse_state, mouse_config);

    let rel_mouse_config = HidConfig {
        report_descriptor: REL_MOUSE_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 4,
        max_packet_size: REL_MOUSE_REPORT_LEN_U16,
    };
    let mut rel_mouse = HidWriter::<_, REL_MOUSE_REPORT_LEN>::new(
        &mut builder,
        &mut rel_mouse_state,
        rel_mouse_config,
    );

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

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let reader_fut = vendor_reader.run(false, &mut vendor_handler);

    let actuator_fut = async {
        let mut kb_keys = KeyboardState::new();
        let mut mouse_pos = MouseState::new();
        // Button latches with per-interface ownership: a press lives on the
        // interface it was pressed on, moves never carry another pointer's
        // buttons, and a release always rides its owner (the OS combines
        // buttons across mice - broadcasting a pressed state onto both
        // interfaces would latch it until both released).
        let mut latches = PointerLatches::new();
        // Which pointer interface new presses ride: the one that moved
        // last. Hosts move before pressing, so the click lands on the same
        // pointer as the move - pressing on the other one would emit an
        // absolute report and yank the cursor to the stale anchor first.
        let mut rel_active = false;
        loop {
            let (caller, cmd) = ACT_CMD.receive().await;
            // Re-check the gates at execution time: a command can sit queued
            // in the channel while the switch flips or `Disarm` lands, and
            // queued actuation must not slip through behind a closed gate.
            // Release-class commands bypass this entirely (see
            // `routing::is_new_actuation`).
            let gate = if routing::is_new_actuation(&cmd) {
                routing::gate_error(ENABLED.load(Ordering::Relaxed), ARMED.load(Ordering::Relaxed))
            } else {
                None
            };
            let reply = match gate {
                Some(code) => Reply::Err(code),
                // Dry-run: the full path runs, the physical step does not.
                // Release-class commands still flow (consistent with the
                // gates' "letting go always works" rule).
                None if DRY_RUN.load(Ordering::Relaxed) && routing::is_new_actuation(&cmd) => {
                    Reply::Echo
                }
                None => match cmd {
                    Command::KeyDown { usage } => {
                        if KeyboardState::is_valid_usage(usage) {
                            if kb_keys.down(usage) {
                                emit_kb(&mut kb, &kb_keys).await;
                                Reply::Ok
                            } else {
                                // Dropped, not latched: report it instead of acking
                                // a tap that would never happen.
                                Reply::Err(ErrorCode::KeyboardFull)
                            }
                        } else {
                            // Usages the flashed descriptor cannot report:
                            // reject them without touching the latched state.
                            Reply::Err(ErrorCode::BadPayload)
                        }
                    }
                    Command::KeyUp { usage } => {
                        if KeyboardState::is_valid_usage(usage) {
                            kb_keys.up(usage);
                            emit_kb(&mut kb, &kb_keys).await;
                            Reply::Ok
                        } else {
                            Reply::Err(ErrorCode::BadPayload)
                        }
                    }
                    Command::KeyTap { usage } => {
                        if KeyboardState::is_valid_usage(usage) {
                            if kb_keys.down(usage) {
                                emit_kb(&mut kb, &kb_keys).await;
                                Timer::after_millis(hold_ms()).await;
                                kb_keys.up(usage);
                                emit_kb(&mut kb, &kb_keys).await;
                                Reply::Ok
                            } else {
                                Reply::Err(ErrorCode::KeyboardFull)
                            }
                        } else {
                            Reply::Err(ErrorCode::BadPayload)
                        }
                    }
                    Command::MouseMoveAbs { x, y } => {
                        mouse_pos.move_abs(x, y);
                        emit_mouse(&mut mouse, &mouse_pos, &latches, 0, 0).await;
                        rel_active = false;
                        Reply::Ok
                    }
                    Command::MouseMoveRel { dx, dy } => {
                        // The relative descriptor carries 8-bit deltas, so a
                        // wire-sized delta is emitted as a paced stream of
                        // <=127-count reports; the ack lands after the last
                        // one. Only this interface's own button bits ride
                        // along (a press owned elsewhere is never carried).
                        let mut stream = chunks::plan(dx, dy).peekable();
                        // A stream can run for seconds - a killswitch closing
                        // mid-flight must stop it at the next report step
                        // instead of letting the whole move land behind a
                        // closed gate. Landing short is safe: hosts verify
                        // placement before pressing. The last move was still
                        // a relative one, so presses keep riding this
                        // interface either way.
                        let mut gate_closed = None;
                        while let Some(chunk) = stream.next() {
                            if let Some(code) = routing::gate_error(
                                ENABLED.load(Ordering::Relaxed),
                                ARMED.load(Ordering::Relaxed),
                            ) {
                                gate_closed = Some(code);
                                break;
                            }
                            emit_rel_mouse(&mut rel_mouse, &latches, 0, chunk.0, chunk.1).await;
                            if stream.peek().is_some() {
                                Timer::after_millis(inter_ms()).await;
                            }
                        }
                        rel_active = true;
                        match gate_closed {
                            Some(code) => Reply::Err(code),
                            None => Reply::Ok,
                        }
                    }
                    Command::MouseDown { button } => {
                        // Emit only on a state change: re-pressing a button
                        // that is already held would re-emit an absolute
                        // report at the stale anchor and yank the OS cursor.
                        let preferred_abs = !rel_active;
                        let before = latches.bits(preferred_abs);
                        let (on_abs, bits) = latches.press(button, preferred_abs);
                        if bits != before {
                            emit_pointer_buttons(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                bits,
                            )
                            .await;
                        }
                        Reply::Ok
                    }
                    Command::MouseUp { button } => {
                        // Releasing a button that was not held changes no
                        // interface's state, so no report goes out - the old
                        // fallback re-emitted the stale absolute anchor and
                        // dragged the cursor there.
                        if let Some((on_abs, bits)) = latches.release(button) {
                            emit_pointer_buttons(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                bits,
                            )
                            .await;
                        }
                        Reply::Ok
                    }
                    Command::ClickAbs { x, y } => {
                        // Button ownership is never carried across
                        // interfaces: while the relative pointer owns the
                        // primary button, the press below would ride the
                        // relative pointer and the cursor would never reach
                        // (x, y). Refuse instead of acking a click at the
                        // wrong place; an absolute move first re-routes new
                        // presses after the rel owner releases.
                        if latches.owner(Button::Left) == Some(false) {
                            Reply::Err(ErrorCode::Undeliverable)
                        } else {
                            mouse_pos.move_abs(x, y);
                            let (on_abs, bits) = latches.press(Button::Left, true);
                            emit_pointer_buttons(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                bits,
                            )
                            .await;
                            rel_active = false;
                            // The click holds for `hold_ms`, exactly like a
                            // key tap; the step gap has no place between a
                            // click's halves.
                            Timer::after_millis(hold_ms()).await;
                            let (_, bits) = latches
                                .release(Button::Left)
                                .unwrap_or((on_abs, latches.bits(on_abs)));
                            emit_pointer_buttons(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                bits,
                            )
                            .await;
                            Reply::Ok
                        }
                    }
                    Command::Wheel { v, h } => {
                        // The descriptors declare the wheel/pan logical range
                        // as -127..=127, but the wire i8 carries -128;
                        // out-of-range values would wrap at the OS, so clamp
                        // to the descriptors' range.
                        let v = v.clamp(-127, 127);
                        let h = h.clamp(-127, 127);
                        let on_abs = !rel_active;
                        // The relative descriptor has no AC Pan axis: refuse
                        // a pan-carrying wheel while it rides the relative
                        // pointer instead of acking Ok with the pan silently
                        // dropped. An absolute move re-anchors the wheel on
                        // the absolute pointer, which carries pan.
                        if h != 0 && !on_abs {
                            Reply::Err(ErrorCode::Undeliverable)
                        } else {
                            emit_wheel(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                latches.bits(on_abs),
                                v,
                                h,
                            )
                            .await;
                            Timer::after_millis(inter_ms()).await;
                            emit_wheel(
                                &mut mouse,
                                &mut rel_mouse,
                                &mouse_pos,
                                on_abs,
                                latches.bits(on_abs),
                                0,
                                0,
                            )
                            .await;
                            Reply::Ok
                        }
                    }
                    Command::ReleaseAll => {
                        kb_keys.clear();
                        emit_kb(&mut kb, &kb_keys).await;
                        latches.clear();
                        // Both pointers: the safety release must unlatch
                        // whatever either interface still reports. After the
                        // clear, both writers report zero buttons.
                        emit_mouse(&mut mouse, &mouse_pos, &latches, 0, 0).await;
                        emit_rel_mouse(&mut rel_mouse, &latches, 0, 0, 0).await;
                        Reply::Ok
                    }
                    // Dispatch never routes config commands here.
                    Command::Ping
                    | Command::Info
                    | Command::Arm
                    | Command::Disarm
                    | Command::SetPacing { .. }
                    | Command::DryRun { .. } => Reply::Ok,
                },
            };
            match caller {
                Caller::Vendor => VENDOR_REPLY.signal(reply),
                Caller::Cdc => CDC_REPLY.signal(reply),
                // The switch's forced release has no waiting caller, and
                // signaling a reply here would desync the request/reply
                // pairing of a real one.
                Caller::Switch => {}
            }
        }
    };

    let logic_fut = async {
        // Duplicate suppression for the host's one-resend policy: the device
        // acks after execution, so a paced command can outlive the host's
        // ack timeout and the resend must replay the cached reply instead of
        // executing twice (double tap, double click). The replay window
        // keeps that promise short, so a fresh host session starting at the
        // cached sequence number executes normally.
        let mut dedup = SeqDedup::new();
        loop {
            let raw = VENDOR_EVENTS.receive().await;
            let received_ms = embassy_time::Instant::now().as_millis();
            let reply = match frame::decode(&raw) {
                Ok((seq, cmd)) => {
                    if let Some(cached) = dedup.replay(seq, received_ms) {
                        cached
                    } else {
                        let reply = dispatch(Caller::Vendor, cmd).await;
                        // Stamp with the COMPLETION time: a chunked command
                        // can run for seconds, and a resend queued behind it
                        // must still land inside the replay window.
                        let done_ms = embassy_time::Instant::now().as_millis();
                        dedup.remember(seq, reply, done_ms);
                        reply
                    }
                }
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

    info!("versa-kbm up");

    #[cfg(feature = "dev-cdc")]
    join(usb_fut, join(reader_fut, join(actuator_fut, join(logic_fut, cdc_fut)))).await;
    #[cfg(not(feature = "dev-cdc"))]
    join(usb_fut, join(reader_fut, join(actuator_fut, logic_fut))).await;
}

/// LED = "can this device actuate right now": solid while the hardware
/// switch is on and the soft gate is armed, slow blink otherwise.
#[embassy_executor::task]
async fn led_task(pin: Peri<'static, PIN_25>) {
    /// Blink half-period in 50 ms poll ticks (250 ms).
    const BLINK_TICKS: u32 = 5;
    let mut led = Output::new(pin, Level::Low);
    let mut tick: u32 = 0;
    let mut blink_on = false;
    loop {
        let live = ENABLED.load(Ordering::Relaxed) && ARMED.load(Ordering::Relaxed);
        if live {
            led.set_high();
        } else {
            tick = tick.wrapping_add(1);
            if tick.is_multiple_of(BLINK_TICKS) {
                blink_on = !blink_on;
            }
            if blink_on {
                led.set_high();
            } else {
                led.set_low();
            }
        }
        Timer::after_millis(50).await;
    }
}

/// Switch sampler cadence (10 ms) and required consecutive equal samples
/// (three - a 30 ms debounce).
const SWITCH_POLL_MS: u64 = 10;
/// Consecutive equal samples needed to accept a new BOOTSEL level.
const SWITCH_DEBOUNCE: u32 = 3;

/// Samples BOOTSEL and toggles [`ENABLED`] on every clean press: the board
/// button is a hardware master switch the host cannot reach. Switching off
/// also queues a release-all so nothing stays latched behind the closed gate.
#[embassy_executor::task]
async fn switch_task() {
    let mut stable = false;
    let mut candidate = false;
    let mut run = 0u32;
    loop {
        let pressed = bootsel_pressed();
        if pressed == candidate {
            run += 1;
        } else {
            candidate = pressed;
            run = 1;
        }
        if run >= SWITCH_DEBOUNCE && candidate != stable {
            stable = candidate;
            if candidate {
                let now_enabled = !ENABLED.load(Ordering::Relaxed);
                ENABLED.store(now_enabled, Ordering::Relaxed);
                if !now_enabled {
                    ACT_CMD.send((Caller::Switch, Command::ReleaseAll)).await;
                }
            }
        }
        Timer::after_millis(SWITCH_POLL_MS).await;
    }
}

/// Reads the BOOTSEL button: `true` while held.
///
/// BOOTSEL hangs off the flash CS line, which the XIP interface normally
/// drives. Sampling follows the pico-sdk recipe (as in tinyusb's
/// `get_bootsel_button`): mask interrupts, float the CS line through the
/// QSPI output-enable override, let the pad settle, read the SIO QSPI input
/// snapshot (the button pulls the line low), then re-attach the line. Every
/// instruction inside that window must execute from RAM - a flash fetch with
/// CS floating would stall the bus - so the sampler lives in `.data` (copied
/// to RAM at boot) and is never inlined.
#[inline(never)]
fn bootsel_pressed() -> bool {
    cortex_m::interrupt::free(|_| sample_cs_line())
}

/// Samples the flash CS line with the QSPI output-enable overridden. Must be
/// called with interrupts masked (see [`bootsel_pressed`]): the CS line is
/// detached for the duration, so nothing may fetch from flash meanwhile.
#[unsafe(link_section = ".data.ram_func")]
#[inline(never)]
fn sample_cs_line() -> bool {
    /// `IO_QSPI` base + `GPIO_QSPI_SS_CTRL` offset (pico-sdk `io_qspi.h`).
    const SS_CTRL: *mut u32 = 0x4001_800C as *mut u32;
    /// `SIO` base + `GPIO_HI_IN` offset (pico-sdk `sio.h`).
    const GPIO_HI_IN: *const u32 = 0xD000_0008 as *const u32;
    /// `OEOVER` field: bits 13:12; `0b10` disables the output (line floats).
    const OEOVER_SHIFT: u32 = 12;
    /// Peripheral-driven output enable (the reset/default state).
    const OEOVER_NORMAL: u32 = 0b00;
    /// Output enable forced off.
    const OEOVER_DISABLE: u32 = 0b10;
    /// QSPI CS is QSPI gpio 1 = bit 1 of the SIO high-bank input snapshot.
    const CS_BIT: u32 = 1 << 1;

    // The fences keep the volatile window from sliding across the caller's
    // interrupt-mask instructions (the asm there declares `nomem`).
    compiler_fence(Ordering::SeqCst);
    // SAFETY: raw register pokes exactly as the pico-sdk performs them; see
    // the function docs for why this runs with interrupts masked from RAM.
    let pressed = unsafe {
        let ctrl = SS_CTRL;
        let saved = ctrl.read_volatile();
        // Float the CS line, let the pad settle, sample, re-attach.
        ctrl.write_volatile((saved & !(0b11 << OEOVER_SHIFT)) | (OEOVER_DISABLE << OEOVER_SHIFT));
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        let pressed = core::ptr::read_volatile(GPIO_HI_IN) & CS_BIT == 0;
        ctrl.write_volatile(
            (ctrl.read_volatile() & !(0b11 << OEOVER_SHIFT)) | (OEOVER_NORMAL << OEOVER_SHIFT),
        );
        pressed
    };
    compiler_fence(Ordering::SeqCst);
    pressed
}

/// Config commands are served inline; actuation is forwarded to the actuator
/// loop and `ReleaseAll` bypasses the armed gate (safety release).
async fn dispatch(caller: Caller, cmd: Command) -> Reply {
    // Commands that may be refused while gated: everything that causes
    // NEW actuation (`routing::is_new_actuation`). Pure releases and the
    // config/queries always work - you must be able to let go of a held
    // key without arming first. The actuator loop re-checks the same gates
    // at execution time, because a command can sit queued while the gate
    // closes.
    if routing::is_new_actuation(&cmd) {
        // The hardware switch outranks the soft arm gate: while it is off,
        // nothing actuates no matter what the host armed.
        if let Some(code) =
            routing::gate_error(ENABLED.load(Ordering::Relaxed), ARMED.load(Ordering::Relaxed))
        {
            return Reply::Err(code);
        }
    }
    // Everything except the inline config commands runs on the actuator loop.
    let routed_to_actuator = !matches!(
        cmd,
        Command::Ping
            | Command::Info
            | Command::Arm
            | Command::Disarm
            | Command::SetPacing { .. }
            | Command::DryRun { .. }
    );
    if routed_to_actuator {
        ACT_CMD.send((caller, cmd)).await;
        return match caller {
            Caller::Vendor => VENDOR_REPLY.wait().await,
            Caller::Cdc => CDC_REPLY.wait().await,
            // Unreachable today: switch-originated commands go straight to
            // the actuator and never wait for a reply.
            Caller::Switch => Reply::Ok,
        };
    }
    match cmd {
        Command::Info => {
            let switch_on = ENABLED.load(Ordering::Relaxed);
            let armed = ARMED.load(Ordering::Relaxed);
            let mut flags = 0u8;
            if switch_on {
                flags |= info_flags::SWITCH_ON;
            }
            if armed {
                flags |= info_flags::ARMED;
            }
            if DRY_RUN.load(Ordering::Relaxed) {
                flags |= info_flags::DRY_RUN;
            }
            Reply::Info { major: VERSION_MAJOR, minor: VERSION_MINOR, flags }
        }
        Command::Arm => {
            ARMED.store(true, Ordering::Relaxed);
            Reply::Ok
        }
        Command::Disarm => {
            ARMED.store(false, Ordering::Relaxed);
            // Same rule as the hardware switch: nothing stays latched behind
            // a closed gate. The release is fire-and-forget (Caller::Switch
            // pairs with no reply) and bypasses both gates in the actuator.
            ACT_CMD.send((Caller::Switch, Command::ReleaseAll)).await;
            Reply::Ok
        }
        Command::SetPacing { inter_ms, hold_ms } => {
            PACING_INTER_MS.store(inter_ms, Ordering::Relaxed);
            PACING_HOLD_MS.store(hold_ms, Ordering::Relaxed);
            Reply::Ok
        }
        Command::DryRun { enabled } => {
            DRY_RUN.store(enabled, Ordering::Relaxed);
            Reply::Ok
        }
        // Exhaustiveness only; routing above already sent every other
        // variant to the actuator, and `Ping` is the only one that really
        // lands here.
        Command::Ping
        | Command::ReleaseAll
        | Command::KeyDown { .. }
        | Command::KeyUp { .. }
        | Command::KeyTap { .. }
        | Command::MouseMoveAbs { .. }
        | Command::MouseMoveRel { .. }
        | Command::MouseDown { .. }
        | Command::MouseUp { .. }
        | Command::ClickAbs { .. }
        | Command::Wheel { .. } => Reply::Ok,
    }
}

/// Pacing gap between sequenced report steps.
fn inter_ms() -> u64 {
    u64::from(PACING_INTER_MS.load(Ordering::Relaxed))
}

/// Button-hold time for clicks and taps.
fn hold_ms() -> u64 {
    u64::from(PACING_HOLD_MS.load(Ordering::Relaxed))
}

/// Latched absolute-pointer anchor: the last absolutely reported position.
/// Relative moves do not touch it - they never emit an absolute report - so
/// the host re-anchors with the real position before any absolute click.
struct MouseState {
    x: u16,
    y: u16,
}

impl MouseState {
    const fn new() -> Self {
        Self { x: ABS_MAX / 2, y: ABS_MAX / 2 }
    }

    fn move_abs(&mut self, x: u16, y: u16) {
        self.x = x.min(ABS_MAX);
        self.y = y.min(ABS_MAX);
    }
}

async fn emit_kb<'a>(
    writer: &mut HidWriter<'a, Driver<'a, USB>, KEYBOARD_REPORT_LEN>,
    kb: &KeyboardState,
) {
    let (modifier, keycodes) = kb.report();
    let report = KeyboardReport { modifier, reserved: 0, leds: 0, keycodes };
    if let Err(e) = writer.write_serialize(&report).await {
        warn!("keyboard report failed: {:?}", e);
    }
}

/// Emits one absolute-mouse report at `anchor`'s position. The absolute
/// pointer's own latched buttons always ride along; a press owned by the
/// relative pointer is never carried, so the button bits cannot be mis-wired
/// at a call site.
async fn emit_mouse<'a>(
    writer: &mut HidWriter<'a, Driver<'a, USB>, MOUSE_REPORT_LEN>,
    anchor: &MouseState,
    latches: &PointerLatches,
    wheel: i8,
    pan: i8,
) {
    emit_abs_report(writer, anchor, latches.bits(true), wheel, pan).await;
}

/// Emits one relative-mouse report: the relative pointer's own latched
/// buttons, a wheel tick and X/Y deltas. Like [`emit_mouse`], the bits come
/// from the latches of the interface doing the emitting.
async fn emit_rel_mouse<'a>(
    writer: &mut HidWriter<'a, Driver<'a, USB>, REL_MOUSE_REPORT_LEN>,
    latches: &PointerLatches,
    wheel: i8,
    dx: i8,
    dy: i8,
) {
    emit_rel_report(writer, latches.bits(false), wheel, dx, dy).await;
}

/// Emits one absolute-mouse report with explicit button bits: `buttons`,
/// wheel and pan ticks at `anchor`'s position.
async fn emit_abs_report<'a>(
    writer: &mut HidWriter<'a, Driver<'a, USB>, MOUSE_REPORT_LEN>,
    anchor: &MouseState,
    buttons: u8,
    wheel: i8,
    pan: i8,
) {
    let mut report = [0u8; MOUSE_REPORT_LEN];
    report[0] = buttons;
    // For an i8 the two's-complement low byte is exactly the report byte
    // the descriptor declares (logical -127..=127).
    #[allow(clippy::cast_sign_loss)]
    {
        report[1] = wheel as u8;
        report[2] = pan as u8;
    }
    report[3..5].copy_from_slice(&anchor.x.to_le_bytes());
    report[5..7].copy_from_slice(&anchor.y.to_le_bytes());
    if let Err(e) = writer.write(&report).await {
        warn!("mouse report failed: {:?}", e);
    }
}

/// Emits one relative-mouse report with explicit button bits: `buttons`, a
/// wheel tick and the X/Y deltas.
async fn emit_rel_report<'a>(
    writer: &mut HidWriter<'a, Driver<'a, USB>, REL_MOUSE_REPORT_LEN>,
    buttons: u8,
    wheel: i8,
    dx: i8,
    dy: i8,
) {
    let mut report = [0u8; REL_MOUSE_REPORT_LEN];
    report[0] = buttons;
    // For an i8 the two's-complement low byte is exactly the report byte
    // the descriptor declares (logical -127..=127).
    #[allow(clippy::cast_sign_loss)]
    {
        report[1] = wheel as u8;
        report[2] = dx as u8;
        report[3] = dy as u8;
    }
    if let Err(e) = writer.write(&report).await {
        warn!("relative mouse report failed: {:?}", e);
    }
}

/// Emits one pointer's button-state report: the absolute pointer reports
/// `buttons` at its anchor, the relative one as a zero-delta report.
async fn emit_pointer_buttons<'a>(
    abs: &mut HidWriter<'a, Driver<'a, USB>, MOUSE_REPORT_LEN>,
    rel: &mut HidWriter<'a, Driver<'a, USB>, REL_MOUSE_REPORT_LEN>,
    anchor: &MouseState,
    on_abs: bool,
    buttons: u8,
) {
    if on_abs {
        emit_abs_report(abs, anchor, buttons, 0, 0).await;
    } else {
        emit_rel_report(rel, buttons, 0, 0, 0).await;
    }
}

/// Emits a wheel pulse on one pointer interface. The relative descriptor
/// has no AC Pan, so horizontal ticks only ever ride the absolute pointer.
async fn emit_wheel<'a>(
    abs: &mut HidWriter<'a, Driver<'a, USB>, MOUSE_REPORT_LEN>,
    rel: &mut HidWriter<'a, Driver<'a, USB>, REL_MOUSE_REPORT_LEN>,
    anchor: &MouseState,
    on_abs: bool,
    buttons: u8,
    v: i8,
    h: i8,
) {
    if on_abs {
        emit_abs_report(abs, anchor, buttons, v, h).await;
    } else {
        emit_rel_report(rel, buttons, v, 0, 0).await;
    }
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
            LineAssembler,
            LineError,
        },
    };

    use super::{
        Caller,
        USB,
        dispatch,
    };

    /// Endpoint-sized reply buffer: embassy rejects writes larger than the
    /// endpoint (64 bytes), and replies are far shorter than that.
    const OUT_LEN: usize = 64;

    /// Serves ASCII command lines until the device is unplugged.
    pub async fn run<'a>(mut class: CdcAcmClass<'a, Driver<'a, USB>>) {
        let mut assembler = LineAssembler::new();
        loop {
            class.wait_connection().await;
            assembler.reset();
            loop {
                let mut pkt = [0u8; 64];
                let n = match class.read_packet(&mut pkt).await {
                    Ok(n) => n,
                    Err(e) => {
                        warn!("cdc read failed: {:?}", e);
                        break;
                    }
                };
                for &byte in pkt.iter().take(n) {
                    match assembler.feed_byte(byte) {
                        // A line landed: decode and serve it.
                        Some(Ok(line)) => match line::decode(line) {
                            Ok(cmd) => {
                                respond(&mut class, &dispatch(Caller::Cdc, cmd).await).await;
                            }
                            Err(LineError::Empty) => {}
                            Err(_) => {
                                respond(&mut class, &Reply::Err(ErrorCode::BadPayload)).await;
                            }
                        },
                        // Overflowed input is dropped, answered like a bad line.
                        Some(Err(_)) => {
                            respond(&mut class, &Reply::Err(ErrorCode::BadPayload)).await;
                        }
                        None => {}
                    }
                }
            }
        }
    }

    async fn respond<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>, reply: &Reply) {
        let mut out = [0u8; OUT_LEN];
        // encode_reply cannot overflow OUT_LEN for any reply; the fallback
        // only exists so the failure path stays honest.
        let mut len = if let Ok(s) = line::encode_reply(reply, &mut out) {
            s.len()
        } else {
            let fallback = b"ERR BAD-PAYLOAD";
            for (d, b) in out.iter_mut().zip(fallback.iter()) {
                *d = *b;
            }
            fallback.len()
        };
        // Clamp before reserving the newline slot so both slices below are
        // in bounds by construction, not by assumption.
        len = len.min(OUT_LEN - 1);
        if let Some(slot) = out.get_mut(len) {
            *slot = b'\n';
        }
        let (head, _) = out.split_at_mut(len + 1);
        if let Err(e) = class.write_packet(head).await {
            warn!("cdc write failed: {:?}", e);
        }
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
            // The wire frame is exactly FRAME_LEN bytes. A short report must
            // not be zero-padded into something decodable, and nothing longer
            // fits the report contract: drop both instead of guessing.
            if let Ok(frame) = <[u8; FRAME_LEN]>::try_from(data) {
                if VENDOR_EVENTS.try_send(frame).is_err() {
                    // The host client resends after its ack timeout, so dropping
                    // is safe; log it for bench visibility.
                    warn!("vendor queue full; frame dropped (host will resend)");
                }
            } else {
                warn!("vendor report dropped: {} bytes (want {})", data.len(), FRAME_LEN);
            }
        }
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}

    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }
}
