//! "Sebas Microphone" as a USB device: its control requests and the
//! isochronous IN transfers that carry the samples.
//!
//! `usbaudio.sys` keeps a few isochronous URBs queued while an app records.
//! A real device fills one packet per millisecond, so each URB is answered
//! when its last packet's millisecond has passed (`due`): the URBs line up
//! back to back on a cursor that starts over after a pause, and the host
//! reads samples at the real rate. Each packet takes its samples from the
//! jitter buffer when the URB is answered. The microphone counts as in use
//! while URBs keep coming; [`IDLE_AFTER`] without one, it is not, and the
//! jitter buffer starts over.
//!
//! Without usbip-win2's filter driver, SET_INTERFACE never reaches the device
//! (UDE handles SELECT_INTERFACE itself), so streaming is inferred from the
//! isochronous traffic alone; a SET_INTERFACE that does arrive is honored.
//! The frame numbers the replies carry count milliseconds since the device
//! was created, not the bus's frames (without the filter driver nothing ties
//! them to QueryBusTime). `usbaudio.sys` records the stream regardless:
//! WASAPI exclusive mode, which bypasses every effect, gets it sample for
//! sample.

use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, MutexGuard},
    time::{Duration, Instant},
};

use tokio::sync::{mpsc::UnboundedSender, watch};

use crate::{
    Device, Holder,
    descriptors::{
        ENDPOINT, FRAMES_PER_PACKET, MAX_FRAMES_PER_PACKET, PACKET_MICROS, SAMPLE_RATE, SERIAL,
        STREAMING_INTERFACE,
    },
    jitter::Jitter,
    usb::{self, ControlReply, Descriptors, Setup},
    usbip::{self, ECONNRESET, EPIPE, IsoInReply, IsoPacket, Submit},
};

/// No isochronous URB for this long: no app records.
pub const IDLE_AFTER: Duration = Duration::from_millis(100);
/// A cursor this far behind the clock means the host paused; the next URB
/// starts over from now rather than catching up.
const RESYNC_AFTER: Duration = Duration::from_millis(20);
/// More queued URBs than `usbaudio.sys` keeps is not a recorder.
const MAX_QUEUED: usize = 64;

/// UAC 1.0 class requests on the endpoint (Audio 1.0 5.2.3.2.3.1).
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const GET_MIN: u8 = 0x82;
const GET_MAX: u8 = 0x83;
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

/// The timer that answers the microphone's URBs on time: it calls
/// [`Microphone::tick`] and waits until the time that returns. Ordinary
/// sleeps round up to the scheduler's tick on some systems, which would hand
/// the host its URBs in bursts, so the application brings a precise one.
pub trait PacketClock: Send + Sync {
    /// Ends the current or next wait early: a URB was queued or the device
    /// closed, so [`Microphone::tick`] has something new to say. A wake sent
    /// while the timer is not waiting must end its next wait.
    fn wake(&self);
}

/// An isochronous IN URB waiting for its time.
struct IsoRead {
    seqnum: u32,
    packets: Vec<IsoPacket>,
    due: Instant,
    /// The bus frame (milliseconds since the device was created) its first
    /// packet belongs to.
    start_frame: u32,
}

struct Inner {
    holder: Holder,
    configuration: u8,
    /// The streaming interface's alternate setting, as far as the device
    /// learns it.
    alternate: u8,
    jitter: Jitter,
    queue: VecDeque<IsoRead>,
    /// When the last queued URB's last packet ends.
    cursor: Option<Instant>,
    /// When the last URB was answered or queued.
    last_activity: Option<Instant>,
    in_use: bool,
}

/// "Sebas Microphone", mono or stereo, served to one USB/IP import at a time
/// ([`Device`]).
pub struct Microphone {
    bus_id: String,
    descriptors: &'static Descriptors,
    channels: u8,
    created: Instant,
    inner: Mutex<Inner>,
    /// Woken when a URB was queued or the device closed, to look again.
    clock: Arc<dyn PacketClock>,
    in_use: watch::Sender<bool>,
}

impl Microphone {
    pub fn new(
        descriptors: &'static Descriptors,
        channels: u8,
        bus_id: String,
        clock: Arc<dyn PacketClock>,
    ) -> Arc<Self> {
        Arc::new(Self {
            bus_id,
            descriptors,
            channels,
            created: Instant::now(),
            inner: Mutex::new(Inner {
                holder: Holder::default(),
                configuration: 0,
                alternate: 0,
                jitter: Jitter::new(channels),
                queue: VecDeque::new(),
                cursor: None,
                last_activity: None,
                in_use: false,
            }),
            clock,
            in_use: watch::channel(false).0,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        // A panic while holding the lock leaves plain data behind; carry on.
        self.inner
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Whether an app records, as it changes.
    pub fn in_use(&self) -> watch::Receiver<bool> {
        self.in_use.subscribe()
    }

    /// The viewer's interleaved samples, `seq` counting pushes.
    pub fn push(&self, seq: u32, pcm: &[i16]) {
        self.lock().jitter.push(seq, pcm);
    }

    /// Answers every URB whose time has come, and settles whether the
    /// microphone is in use. Returns when to look again: the next URB's
    /// time, the moment it goes idle, or None when the device is closed.
    pub fn tick(&self, now: Instant) -> Option<Option<Instant>> {
        let mut inner = self.lock();
        if inner.holder.is_closed() {
            return None;
        }
        while inner.queue.front().is_some_and(|read| read.due <= now) {
            let read = inner.queue.pop_front().expect("checked above");
            let frame = self.answer(&mut inner, &read);
            inner.holder.send(frame);
            inner.last_activity = Some(now);
        }
        let streaming = !inner.queue.is_empty()
            || inner
                .last_activity
                .is_some_and(|last| now.duration_since(last) < IDLE_AFTER);
        if streaming != inner.in_use {
            inner.in_use = streaming;
            let stats = inner.jitter.stats();
            if streaming {
                tracing::info!(channels = self.channels, "virtual_microphone_streaming");
            } else {
                inner.jitter.stop();
                inner.cursor = None;
                tracing::info!(
                    underruns = stats.underruns,
                    dropped = stats.dropped,
                    stale = stats.stale,
                    "virtual_microphone_idle"
                );
            }
            self.in_use.send_replace(streaming);
        }
        let next = match inner.queue.front() {
            Some(read) => Some(read.due),
            None => inner
                .last_activity
                .filter(|_| inner.in_use)
                .map(|last| last + IDLE_AFTER),
        };
        Some(next)
    }

    fn answer(&self, inner: &mut Inner, read: &IsoRead) -> Vec<u8> {
        let frame_bytes = 2 * usize::from(self.channels);
        let mut reply = IsoInReply::new(
            read.seqnum,
            read.start_frame,
            read.packets.len(),
            read.packets.len() * MAX_FRAMES_PER_PACKET * frame_bytes,
        );
        let mut packet_data = Vec::with_capacity(MAX_FRAMES_PER_PACKET * frame_bytes);
        for packet in &read.packets {
            packet_data.clear();
            let room = packet.length as usize / frame_bytes;
            if room >= MAX_FRAMES_PER_PACKET {
                inner.jitter.next_packet(&mut packet_data);
            } else {
                // Smaller than the endpoint's packets: never happens with
                // usbaudio; silence keeps the stream's pace.
                packet_data.resize(room.min(FRAMES_PER_PACKET) * frame_bytes, 0);
            }
            reply.packet(*packet, &packet_data);
        }
        reply.finish()
    }

    /// Queues an isochronous IN URB at the end of the cursor.
    fn queue(&self, inner: &mut Inner, submit: &Submit, iso: &[u8]) {
        let packets: Vec<IsoPacket> = usbip::iso_packets(iso).collect();
        if packets.len() != submit.number_of_packets as usize
            || packets.is_empty()
            || inner.queue.len() >= MAX_QUEUED
        {
            inner
                .holder
                .send(usbip::ret_submit(submit.seqnum, -EPIPE, 0, &[]));
            return;
        }
        let now = Instant::now();
        let start = match inner.cursor {
            Some(cursor) if cursor + RESYNC_AFTER >= now => cursor.max(now - RESYNC_AFTER),
            _ => now,
        };
        let due = start + Duration::from_micros(PACKET_MICROS * packets.len() as u64);
        inner.cursor = Some(due);
        inner.last_activity = Some(now);
        let start_frame =
            (start.duration_since(self.created).as_micros() / u128::from(PACKET_MICROS)) as u32;
        inner.queue.push_back(IsoRead {
            seqnum: submit.seqnum,
            packets,
            due,
            start_frame,
        });
        self.clock.wake();
    }

    /// Forgets queued URBs: the host reset, deconfigured or stopped the
    /// stream, or the import ended.
    fn reset_stream(inner: &mut Inner) {
        inner.queue.clear();
        inner.cursor = None;
    }

    fn control(&self, inner: &mut Inner, submit: &Submit, data: &[u8]) -> ControlReply {
        let setup = Setup::parse(submit.setup);
        if setup.is_port_reset() {
            inner.configuration = 0;
            inner.alternate = 0;
            Self::reset_stream(inner);
            return ControlReply::Accepted;
        }
        match (setup.kind(), setup.recipient()) {
            (0, 1) => self.interface_request(inner, &setup),
            (0, _) => {
                let before = inner.configuration;
                let reply = usb::standard_request(
                    self.descriptors,
                    SERIAL,
                    &mut inner.configuration,
                    &setup,
                );
                if inner.configuration != before {
                    inner.alternate = 0;
                    Self::reset_stream(inner);
                }
                reply
            }
            (1, 2) if setup.index == u16::from(ENDPOINT) => sampling_rate(&setup, data),
            _ => ControlReply::Stall,
        }
    }

    /// GET_INTERFACE and SET_INTERFACE: the control interface has only
    /// setting 0, the streaming interface 0 (idle) and 1 (streaming).
    fn interface_request(&self, inner: &mut Inner, setup: &Setup) -> ControlReply {
        let interface = setup.index as u8;
        match (setup.request, setup.device_to_host(), interface) {
            (0x0A, true, 0) => ControlReply::In(vec![0]),
            (0x0A, true, STREAMING_INTERFACE) => ControlReply::In(vec![inner.alternate]),
            (0x0B, false, 0) if setup.value == 0 => ControlReply::Accepted,
            (0x0B, false, STREAMING_INTERFACE) if setup.value <= 1 => {
                inner.alternate = setup.value as u8;
                if inner.alternate == 0 {
                    Self::reset_stream(inner);
                }
                ControlReply::Accepted
            }
            _ => ControlReply::Stall,
        }
    }
}

/// The endpoint's sampling frequency control: 48 kHz, the only rate.
fn sampling_rate(setup: &Setup, data: &[u8]) -> ControlReply {
    let [_, control] = setup.value.to_le_bytes();
    if control != SAMPLING_FREQ_CONTROL {
        return ControlReply::Stall;
    }
    let rate = SAMPLE_RATE.to_le_bytes();
    match (setup.request, setup.device_to_host()) {
        (SET_CUR, false) if data.len() == 3 && data == &rate[..3] => ControlReply::Accepted,
        (GET_CUR | GET_MIN | GET_MAX, true) => ControlReply::In(rate[..3].to_vec()),
        _ => ControlReply::Stall,
    }
}

impl Device for Microphone {
    fn bus_id(&self) -> &str {
        &self.bus_id
    }

    fn import_reply(&self) -> Vec<u8> {
        // One device on its own server: bus 1, device 1.
        usbip::import_reply(&self.bus_id, (1 << 16) | 1, self.descriptors)
    }

    fn claim(&self, replies: UnboundedSender<Vec<u8>>) -> Option<u64> {
        let mut inner = self.lock();
        let link = inner.holder.claim(replies)?;
        inner.configuration = 0;
        inner.alternate = 0;
        Self::reset_stream(&mut inner);
        Some(link)
    }

    fn release(&self, link: u64) {
        let mut inner = self.lock();
        if inner.holder.release(link) {
            Self::reset_stream(&mut inner);
        }
    }

    fn close(&self) {
        {
            let mut inner = self.lock();
            inner.holder.close();
            Self::reset_stream(&mut inner);
        }
        self.clock.wake();
    }

    fn is_closed(&self) -> bool {
        self.lock().holder.is_closed()
    }

    fn submit(&self, link: u64, submit: &Submit, data: &[u8], iso: &[u8]) -> bool {
        let mut inner = self.lock();
        if !inner.holder.holds(link) {
            return false;
        }
        if submit.endpoint == 0 {
            let frame = match self.control(&mut inner, submit, data) {
                ControlReply::In(mut bytes) => {
                    let setup = Setup::parse(submit.setup);
                    bytes.truncate(
                        usize::from(setup.length).min(submit.transfer_buffer_length as usize),
                    );
                    usbip::ret_submit(submit.seqnum, 0, bytes.len() as u32, &bytes)
                }
                ControlReply::Accepted => {
                    usbip::ret_submit(submit.seqnum, 0, data.len() as u32, &[])
                }
                ControlReply::Stall => usbip::ret_submit(submit.seqnum, -EPIPE, 0, &[]),
            };
            inner.holder.send(frame);
            return true;
        }
        if submit.endpoint_address() == ENDPOINT && submit.is_isochronous() {
            self.queue(&mut inner, submit, iso);
        } else {
            inner
                .holder
                .send(usbip::ret_submit(submit.seqnum, -EPIPE, 0, &[]));
        }
        true
    }

    /// A queued URB is dropped and answered -ECONNRESET; one already
    /// answered gets 0.
    fn unlink(&self, link: u64, seqnum: u32, victim: u32) -> bool {
        let mut inner = self.lock();
        if !inner.holder.holds(link) {
            return false;
        }
        let status = match inner.queue.iter().position(|read| read.seqnum == victim) {
            Some(index) => {
                inner.queue.remove(index);
                if inner.queue.is_empty() {
                    inner.cursor = None;
                }
                -ECONNRESET
            }
            None => 0,
        };
        inner.holder.send(usbip::ret_unlink(seqnum, status));
        true
    }
}
