//! Virtual USB devices served over USB/IP: the device framing Sebas's
//! emulated devices share, and "Sebas Microphone", the USB Audio device
//! Sebas plugs into a Windows host so every app there can record a remote
//! viewer's microphone.
//!
//! The shared part is the USB ([`usb`]) and USB/IP ([`usbip`]) framing, the
//! standard requests ([`usb::standard_request`]), the [`Device`] interface a
//! USB/IP server drives and the [`Holder`] of a device's import. Sebas's
//! virtual game controllers are built on it too; their descriptors and
//! reports are not part of this crate.
//!
//! The microphone is a USB Audio Class 1.0 microphone, 48 kHz 16-bit PCM,
//! mono or stereo ([`descriptors`]). It exists only in software: Sebas
//! emulates it and serves it over USB/IP to the usbip-win2 host controller,
//! and Windows binds its in-box `usbaudio.sys` to it. This crate is the
//! device itself: its descriptors, its answers to control requests, the
//! isochronous IN transfers that carry the samples on the host's clock
//! ([`Microphone`]) and the jitter buffer between the viewer's bursts and
//! that clock ([`jitter`]). It has no platform code.
//!
//! An application plugs in at two points: its USB/IP server drives the
//! device through [`Device`] and carries the replies queued on the channel
//! it hands to [`Device::claim`], and a timer of its own calls
//! [`Microphone::tick`] when each URB is due, woken through
//! [`PacketClock`].

pub mod descriptors;
pub mod jitter;
mod microphone;
#[cfg(any(test, feature = "test-support"))]
pub mod test_support;
#[cfg(test)]
mod tests;
pub mod usb;
#[cfg(test)]
mod usb_tests;
pub mod usbip;

use tokio::sync::mpsc::UnboundedSender;

pub use self::microphone::{IDLE_AFTER, Microphone, PacketClock};
use self::usbip::Submit;

/// An emulated device as the USB/IP server sees it: one import at a time
/// holds it and streams its URBs.
pub trait Device: Send + Sync + 'static {
    fn bus_id(&self) -> &str;
    /// OP_REP_IMPORT with the device block.
    fn import_reply(&self) -> Vec<u8>;
    /// Makes `replies` the import that holds the device, unless another one
    /// does or the device is closed; the device starts over unconfigured.
    fn claim(&self, replies: UnboundedSender<Vec<u8>>) -> Option<u64>;
    /// Lets go of import `link`; what it had queued goes with it.
    fn release(&self, link: u64);
    /// No import may hold the device any more; the current one's reply queue
    /// closes, which ends its connection.
    fn close(&self);
    fn is_closed(&self) -> bool;
    /// A CMD_SUBMIT from import `link`, with its OUT data and, for an
    /// isochronous URB, its packet descriptors as the wire carries them.
    /// False when that import no longer holds the device.
    fn submit(&self, link: u64, submit: &Submit, data: &[u8], iso: &[u8]) -> bool;
    /// A CMD_UNLINK from import `link`. False when that import no longer
    /// holds the device.
    fn unlink(&self, link: u64, seqnum: u32, victim: u32) -> bool;
}

/// Which import holds a device and the queue its replies go out on, in the
/// order they are queued; part of each device's locked state.
#[derive(Default)]
pub struct Holder {
    link: Option<(u64, UnboundedSender<Vec<u8>>)>,
    next: u64,
    closed: bool,
}

impl Holder {
    pub fn claim(&mut self, replies: UnboundedSender<Vec<u8>>) -> Option<u64> {
        if self.closed || self.link.is_some() {
            return None;
        }
        self.next += 1;
        self.link = Some((self.next, replies));
        Some(self.next)
    }

    /// True when `link` held the device and no longer does.
    pub fn release(&mut self, link: u64) -> bool {
        if self.holds(link) {
            self.link = None;
            true
        } else {
            false
        }
    }

    pub fn close(&mut self) {
        self.closed = true;
        self.link = None;
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }

    pub fn holds(&self, link: u64) -> bool {
        self.link.as_ref().is_some_and(|(held, _)| *held == link)
    }

    /// Queues a reply to the holding import; dropped when none holds it.
    pub fn send(&self, frame: Vec<u8>) {
        if let Some((_, replies)) = &self.link {
            let _ = replies.send(frame);
        }
    }
}
