//! "Sebas Microphone": a USB Audio Class 1.0 microphone, 48 kHz 16-bit PCM,
//! mono or stereo (USB Audio 1.0 and Audio Data Formats 1.0).
//!
//! Windows binds its in-box `usbaudio.sys` to it by class. UAC 1.0 rather
//! than 2.0: every Windows version Sebas supports has `usbaudio.sys`
//! (`usbaudio2.sys` arrived with Windows 10 1703) and a microphone needs none
//! of UAC 2.0's clock entities. The device presents as High-Speed with its
//! isochronous IN endpoint at bInterval 4 (2^3 microframes, 1 ms): UDE
//! treats every device as High-Speed, and an isochronous endpoint under 4
//! fails before it reaches the driver (usbip-win2 issue 35,
//! `drivers/ude/wsk_receive.cpp`). The endpoint is asynchronous, so a packet
//! may carry one sample frame more or less than 48 to follow the viewer's
//! clock ([`crate::jitter`]); its wMaxPacketSize leaves room for 49.
//!
//! No feature unit: Windows applies a capture endpoint's level in software
//! when the device has no volume control, and the device then never has to
//! answer class requests for one.

use crate::usb::{Descriptors, SPEED_HIGH};

/// The isochronous IN endpoint that carries the samples.
pub const ENDPOINT: u8 = 0x81;
/// The audio streaming interface, alternate setting 1 streams.
pub const STREAMING_INTERFACE: u8 = 1;
pub const SAMPLE_RATE: u32 = 48_000;
/// Sample frames per 1 ms packet at the nominal rate.
pub const FRAMES_PER_PACKET: usize = 48;
/// A packet may carry one frame more to drain the viewer's faster clock.
pub const MAX_FRAMES_PER_PACKET: usize = FRAMES_PER_PACKET + 1;
/// The time one packet covers.
pub const PACKET_MICROS: u64 = 1_000;
/// The USB serial string: one microphone per computer, so the same device
/// instance (and the endpoint settings Windows keeps for it) every time.
/// Viewers recognize it by its name, "Sebas Microphone".
pub const SERIAL: &str = "SEBAS-VMIC-1";

/// The device's USB vendor id: pid.codes' vendor id for open-source hardware
/// (<https://pid.codes/1209/>). Windows binds `usbaudio.sys` by class, so
/// the vendor and product id only name the device; a host also recognizes
/// it by them (Sebas turns Windows' voice processing off on its endpoints).
pub const VENDOR_ID: u16 = 0x1209;
/// The device's USB product id under [`VENDOR_ID`]. 0x0001 is pid.codes'
/// shared test id (<https://pid.codes/1209/0001/>), a placeholder until
/// pid.codes assigns this device its own product id; the test id is not
/// unique and pid.codes allows it only for in-house testing.
pub const PRODUCT_ID: u16 = 0x0001;

const VENDOR: [u8; 2] = VENDOR_ID.to_le_bytes();
const PRODUCT: [u8; 2] = PRODUCT_ID.to_le_bytes();

#[rustfmt::skip]
static DEVICE: [u8; 18] = [
    0x12, 0x01,
    0x00, 0x02, // bcdUSB 2.00
    0x00, 0x00, 0x00, // class per interface
    0x40, // bMaxPacketSize0: 64, as USBHUB3 expects of a High-Speed device
    VENDOR[0], VENDOR[1],
    PRODUCT[0], PRODUCT[1],
    0x00, 0x01, // bcdDevice 1.00
    0x01, 0x02, 0x03, // manufacturer, product, serial strings
    0x01, // one configuration
];

/// Configuration descriptor for `channels` (1 or 2): the audio control
/// interface (microphone input terminal 1 to USB streaming output terminal
/// 2), then the streaming interface with zero-bandwidth alternate setting 0
/// and alternate setting 1 with the isochronous IN endpoint.
macro_rules! configuration {
    ($channels:expr, $channel_config:expr, $packet:expr) => {
        [
            // Configuration: 100 bytes, 2 interfaces, bus-powered, 100 mA.
            0x09,
            0x02,
            0x64,
            0x00,
            0x02,
            0x01,
            0x00,
            0x80,
            0x32,
            // Interface 0, alternate 0: audio control, no endpoints.
            0x09,
            0x04,
            0x00,
            0x00,
            0x00,
            0x01,
            0x01,
            0x00,
            0x00,
            // AC header: ADC 1.00, 30 bytes of class descriptors, streaming
            // interface 1.
            0x09,
            0x24,
            0x01,
            0x00,
            0x01,
            0x1E,
            0x00,
            0x01,
            0x01,
            // Input terminal 1: microphone.
            0x0C,
            0x24,
            0x02,
            0x01,
            0x01,
            0x02,
            0x00,
            $channels,
            $channel_config,
            0x00,
            0x00,
            0x00,
            // Output terminal 2: USB streaming, from terminal 1.
            0x09,
            0x24,
            0x03,
            0x02,
            0x01,
            0x01,
            0x00,
            0x01,
            0x00,
            // Interface 1, alternate 0: audio streaming, no bandwidth.
            0x09,
            0x04,
            0x01,
            0x00,
            0x00,
            0x01,
            0x02,
            0x00,
            0x00,
            // Interface 1, alternate 1: audio streaming, one endpoint.
            0x09,
            0x04,
            0x01,
            0x01,
            0x01,
            0x01,
            0x02,
            0x00,
            0x00,
            // AS general: linked to terminal 2, 1 frame delay, PCM.
            0x07,
            0x24,
            0x01,
            0x02,
            0x01,
            0x01,
            0x00,
            // Format type I: channels, 2-byte subframes, 16 bits, 48 kHz.
            0x0B,
            0x24,
            0x02,
            0x01,
            $channels,
            0x02,
            0x10,
            0x01,
            0x80,
            0xBB,
            0x00,
            // Endpoint 0x81: isochronous, asynchronous, data; 1 ms.
            0x09,
            0x05,
            ENDPOINT,
            0x05,
            $packet,
            0x00,
            0x04,
            0x00,
            0x00,
            // CS endpoint: general, no controls.
            0x07,
            0x25,
            0x01,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    };
}

#[rustfmt::skip]
static MONO_CONFIGURATION: [u8; 100] =
    configuration!(0x01, 0x00, (MAX_FRAMES_PER_PACKET * 2) as u8);
#[rustfmt::skip]
static STEREO_CONFIGURATION: [u8; 100] =
    configuration!(0x02, 0x03, (MAX_FRAMES_PER_PACKET * 4) as u8);

const STRINGS: &[&str] = &["Sebas", "Sebas Microphone"];

pub static MONO: Descriptors = Descriptors {
    speed: SPEED_HIGH,
    device: &DEVICE,
    configuration: &MONO_CONFIGURATION,
    strings: STRINGS,
    hid_report: None,
};

pub static STEREO: Descriptors = Descriptors {
    speed: SPEED_HIGH,
    device: &DEVICE,
    configuration: &STEREO_CONFIGURATION,
    strings: STRINGS,
    hid_report: None,
};

/// The descriptors for 1 or 2 channels; None for any other count.
pub fn for_channels(channels: u8) -> Option<&'static Descriptors> {
    match channels {
        1 => Some(&MONO),
        2 => Some(&STEREO),
        _ => None,
    }
}
