//! Pure tests: the microphone's descriptors, its jitter buffer and its
//! USB/IP behaviour against in-memory channels.

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use tokio::sync::mpsc::{UnboundedReceiver, unbounded_channel};

use crate::{
    Device, IDLE_AFTER, Microphone, PacketClock,
    descriptors::{self, ENDPOINT, FRAMES_PER_PACKET, MONO, PRODUCT_ID, SERIAL, STEREO, VENDOR_ID},
    jitter::{BASE_MARGIN, CAP_FRAMES, Jitter, MARGIN_STEP, WINDOW_PACKETS},
    test_support::{be32, le16, parse_submit, submit_header},
    usb::{Descriptors, SPEED_HIGH, string_descriptor},
    usbip::{self, HEADER_SIZE},
};

/// One 20 ms burst, as the viewer's Opus frames arrive.
const BURST: usize = 960;
/// What a fresh jitter buffer waits for before it plays: a burst and the
/// base margin.
const START: usize = BURST + BASE_MARGIN;

// --- descriptors ---

/// The configuration's descriptors as (type, bytes).
fn parts(descriptors: &Descriptors) -> Vec<(u8, &'static [u8])> {
    let configuration = descriptors.configuration;
    let mut parts = Vec::new();
    let mut offset = 0;
    while offset < configuration.len() {
        let length = usize::from(configuration[offset]);
        assert!(length >= 2 && offset + length <= configuration.len());
        parts.push((
            configuration[offset + 1],
            &configuration[offset..offset + length],
        ));
        offset += length;
    }
    parts
}

#[test]
fn descriptors_are_a_uac1_high_speed_microphone() {
    for (descriptors, channels) in [(&MONO, 1u8), (&STEREO, 2u8)] {
        assert_eq!(descriptors.speed, SPEED_HIGH);
        let device = descriptors.device;
        assert_eq!((device[0], device[1]), (18, 1));
        assert_eq!(le16(device, 2), 0x0200);
        assert_eq!(&device[4..7], &[0, 0, 0], "class per interface");
        assert_eq!(device[7], 64);
        assert_eq!((le16(device, 8), le16(device, 10)), (VENDOR_ID, PRODUCT_ID));
        assert_eq!(device[17], 1);
        let configuration = descriptors.configuration;
        assert_eq!(usize::from(le16(configuration, 2)), configuration.len());
        assert_eq!(descriptors.num_interfaces(), 2);
        let parts = parts(descriptors);
        // Interfaces: control 0/0, streaming 1/0 (idle) and 1/1.
        let interfaces: Vec<(u8, u8, u8, u8, u8)> = parts
            .iter()
            .filter(|(kind, _)| *kind == 0x04)
            .map(|(_, part)| (part[2], part[3], part[4], part[5], part[6]))
            .collect();
        assert_eq!(
            interfaces,
            [(0, 0, 0, 1, 1), (1, 0, 0, 1, 2), (1, 1, 1, 1, 2)]
        );
        // The AC header's length covers its class descriptors and names
        // the streaming interface.
        let class: Vec<&[u8]> = parts
            .iter()
            .filter(|(kind, _)| *kind == 0x24)
            .map(|(_, part)| *part)
            .collect();
        let header = class[0];
        assert_eq!(header[2], 0x01);
        let ac_length: usize = class[..3].iter().map(|part| part.len()).sum();
        assert_eq!(usize::from(le16(header, 5)), ac_length);
        assert_eq!(&header[7..9], &[1, 1]);
        // Input terminal: a microphone with the channels.
        let input = class[1];
        assert_eq!(
            (input[2], le16(input, 4), input[7]),
            (0x02, 0x0201, channels)
        );
        // Output terminal 2, USB streaming, fed by terminal 1.
        let output = class[2];
        assert_eq!(
            (output[2], output[3], le16(output, 4), output[7]),
            (0x03, 2, 0x0101, 1)
        );
        // AS general links terminal 2, PCM; format type I, 16-bit, 48 kHz.
        let general = class[3];
        assert_eq!(
            (general[2], general[3], le16(general, 5)),
            (0x01, 2, 0x0001)
        );
        let format = class[4];
        assert_eq!(
            (
                format[2], format[3], format[4], format[5], format[6], format[7]
            ),
            (0x02, 1, channels, 2, 16, 1)
        );
        assert_eq!(
            u32::from_le_bytes([format[8], format[9], format[10], 0]),
            48_000
        );
        // One endpoint: 0x81, isochronous asynchronous, 1 ms at High-Speed
        // (bInterval 4), room for 49 frames.
        let endpoints: Vec<_> = descriptors.endpoints().collect();
        assert_eq!(endpoints.len(), 1);
        let (_, endpoint) = parts.iter().find(|(kind, _)| *kind == 0x05).unwrap();
        assert_eq!(endpoint.len(), 9);
        assert_eq!((endpoint[2], endpoint[3]), (ENDPOINT, 0x05));
        assert_eq!(
            usize::from(le16(endpoint, 4)),
            49 * 2 * usize::from(channels)
        );
        assert_eq!(endpoint[6], 4);
        // Strings: the product names the microphone everywhere.
        assert_eq!(
            descriptors.string(device[15]).unwrap(),
            string_descriptor("Sebas Microphone")
        );
        assert!(descriptors.string(device[14]).is_some());
        assert_eq!(device[16], 3, "a serial string");
    }
    assert!(std::ptr::eq(descriptors::for_channels(1).unwrap(), &MONO));
    assert!(std::ptr::eq(descriptors::for_channels(2).unwrap(), &STEREO));
    assert!(descriptors::for_channels(0).is_none());
    assert!(descriptors::for_channels(3).is_none());
}

#[test]
fn import_reply_announces_a_high_speed_device_with_two_interfaces() {
    let reply = usbip::import_reply("sebas-1-2-abcd", 0x0001_0001, &MONO);
    let device = &reply[8..];
    assert_eq!(be32(device, 296), SPEED_HIGH);
    assert_eq!(&device[300..306], &[0x12, 0x09, 0x00, 0x01, 0x01, 0x00]);
    assert_eq!(&device[306..312], &[0, 0, 0, 1, 1, 2]);
}

// --- jitter buffer ---

fn tone(frames: usize, channels: usize, start: i16) -> Vec<i16> {
    (0..frames * channels)
        .map(|index| start.wrapping_add(index as i16))
        .collect()
}

fn samples(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|pair| i16::from_le_bytes([pair[0], pair[1]]))
        .collect()
}

#[test]
fn jitter_waits_for_its_start_level_then_plays_in_order() {
    let mut jitter = Jitter::new(1);
    let mut out = Vec::new();
    assert_eq!(jitter.next_packet(&mut out), FRAMES_PER_PACKET);
    assert_eq!(out, vec![0; FRAMES_PER_PACKET * 2], "silence while priming");
    jitter.push(1, &tone(BURST, 1, 0));
    out.clear();
    jitter.next_packet(&mut out);
    assert!(out.iter().all(|&byte| byte == 0), "20 ms is not enough yet");
    jitter.push(2, &tone(BURST, 1, 960));
    out.clear();
    // 40 ms waited: trimmed to the start level, oldest first.
    assert_eq!(jitter.start_frames(), START);
    assert_eq!(jitter.frames(), START);
    jitter.next_packet(&mut out);
    let first = (2 * BURST - START) as i16;
    assert_eq!(samples(&out), tone(FRAMES_PER_PACKET, 1, first));
    assert_eq!(jitter.stats().dropped, (2 * BURST - START) as u64);
}

#[test]
fn jitter_drops_stale_pushes_and_partial_frames() {
    let mut jitter = Jitter::new(2);
    jitter.push(5, &tone(10, 2, 0));
    jitter.push(5, &tone(10, 2, 0));
    jitter.push(4, &tone(10, 2, 0));
    assert_eq!(jitter.stats().stale, 2);
    jitter.push(6, &[1, 2, 3]);
    assert_eq!(jitter.frames(), 11, "the odd sample is cut off");
}

/// A viewer whose clock runs `ppm` fast (negative: slow): it sends each
/// 20 ms burst once its clock has recorded it, every other one `jitter_ms`
/// late, and the host takes one packet per millisecond. Runs carry on
/// where the last one stopped.
struct Viewer {
    ppm: i64,
    jitter_ms: u64,
    millisecond: u64,
    /// Bursts the viewer has recorded.
    recorded: u32,
    /// Recorded bursts on their way, and when each arrives.
    arriving: std::collections::VecDeque<u64>,
    seq: u32,
}

impl Viewer {
    fn new(ppm: i64, jitter_ms: u64) -> Self {
        Self {
            ppm,
            jitter_ms,
            millisecond: 0,
            recorded: 0,
            arriving: std::collections::VecDeque::new(),
            seq: 0,
        }
    }

    /// Returns the fill after each packet.
    fn run(&mut self, jitter: &mut Jitter, seconds: u32) -> Vec<usize> {
        let mut fills = Vec::new();
        let mut out = Vec::new();
        for _ in 0..seconds * 1000 {
            let produced = (self.millisecond as i64 * 48 * (1_000_000 + self.ppm)) / 1_000_000;
            while produced >= i64::from(self.recorded + 1) * BURST as i64 {
                self.recorded += 1;
                let late = if self.recorded.is_multiple_of(2) {
                    self.jitter_ms.min(19)
                } else {
                    0
                };
                self.arriving.push_back(self.millisecond + late);
            }
            while self
                .arriving
                .front()
                .is_some_and(|&arrives| arrives <= self.millisecond)
            {
                self.arriving.pop_front();
                self.seq += 1;
                jitter.push(self.seq, &tone(BURST, 1, 0));
            }
            out.clear();
            jitter.next_packet(&mut out);
            fills.push(jitter.frames());
            self.millisecond += 1;
        }
        fills
    }
}

/// The highest low a steady stream keeps before steering pulls it down:
/// the margin plus 10 ms.
fn steady_low_limit(jitter: &Jitter) -> usize {
    jitter.margin() + 480
}

#[test]
fn jitter_keeps_latency_low_and_steady() {
    let mut jitter = Jitter::new(1);
    let fills = Viewer::new(0, 8).run(&mut jitter, 20);
    let settled = &fills[2000..];
    assert_eq!(jitter.stats().underruns, 0);
    assert_eq!(jitter.margin(), BASE_MARGIN, "no underrun widens it");
    // On average under 40 ms waiting; at the top of the sawtooth the low
    // plus a burst and the late one's 8 ms, and the low itself stays
    // within 10 ms of the margin.
    let mean = settled.iter().sum::<usize>() / settled.len();
    assert!(mean <= 40 * 48, "mean {mean}");
    assert!(*settled.iter().max().unwrap() <= steady_low_limit(&jitter) + BURST + 8 * 48);
    let low = *settled.iter().min().unwrap();
    assert!(low <= steady_low_limit(&jitter), "low {low}");
}

#[test]
fn jitter_follows_a_fast_or_slow_viewer_clock_without_dropping_samples() {
    for ppm in [-2_000, -300, 300, 2_000] {
        let mut jitter = Jitter::new(1);
        let mut viewer = Viewer::new(ppm, 4);
        viewer.run(&mut jitter, 2);
        // Only the start trimmed what waited before the stream began.
        let dropped_at_start = jitter.stats().dropped;
        let fills = viewer.run(&mut jitter, 58);
        let last_window = &fills[fills.len() - WINDOW_PACKETS as usize..];
        let low = *last_window.iter().min().unwrap();
        assert!(
            low <= steady_low_limit(&jitter) + FRAMES_PER_PACKET,
            "{ppm} ppm drifted up to a low of {low}"
        );
        // Drift is steered away with 47 and 49 frame packets, never dropped.
        assert_eq!(jitter.stats().dropped, dropped_at_start, "{ppm} ppm");
        // A slow clock may run dry once before steering catches up, never
        // again afterwards.
        assert!(
            jitter.stats().underruns <= 1,
            "{ppm} ppm: {:?}",
            jitter.stats()
        );
        let late_underruns_before = jitter.stats().underruns;
        viewer.run(&mut jitter, 10);
        assert_eq!(jitter.stats().underruns, late_underruns_before, "{ppm} ppm");
    }
}

#[test]
fn jitter_drops_a_backlog_at_once() {
    let mut jitter = Jitter::new(1);
    let mut viewer = Viewer::new(0, 0);
    viewer.run(&mut jitter, 2);
    let dropped = jitter.stats().dropped;
    // A stalled network delivers 60 ms more at once.
    viewer.seq += 1;
    jitter.push(viewer.seq, &tone(3 * BURST, 1, 0));
    let fills = viewer.run(&mut jitter, 2);
    assert!(
        jitter.stats().dropped >= dropped + 2 * BURST as u64,
        "{:?}",
        jitter.stats()
    );
    let last_window = &fills[fills.len() - WINDOW_PACKETS as usize / 2..];
    assert!(*last_window.iter().min().unwrap() <= steady_low_limit(&jitter));
    assert_eq!(jitter.stats().underruns, 0);
}

#[test]
fn jitter_plays_silence_when_the_viewer_stalls_and_starts_over() {
    let mut jitter = Jitter::new(1);
    jitter.push(1, &tone(START, 1, 0));
    let mut out = Vec::new();
    for _ in 0..START / FRAMES_PER_PACKET {
        jitter.next_packet(&mut out);
    }
    out.clear();
    jitter.next_packet(&mut out);
    assert!(out.iter().all(|&byte| byte == 0));
    assert_eq!(jitter.stats().underruns, 1);
    // The margin widens, and the stream starts over from priming: a single
    // burst plays nothing yet.
    assert_eq!(jitter.margin(), BASE_MARGIN + MARGIN_STEP);
    assert_eq!(jitter.start_frames(), START + MARGIN_STEP);
    jitter.push(2, &tone(BURST, 1, 1));
    out.clear();
    jitter.next_packet(&mut out);
    assert!(out.iter().all(|&byte| byte == 0));
}

#[test]
fn jitter_narrows_its_margin_after_a_window_without_underruns() {
    let mut jitter = Jitter::new(1);
    jitter.push(1, &tone(START, 1, 0));
    let mut out = Vec::new();
    for _ in 0..=START / FRAMES_PER_PACKET {
        jitter.next_packet(&mut out);
    }
    assert_eq!(jitter.margin(), BASE_MARGIN + MARGIN_STEP);
    // Steady seconds take a millisecond off per window, never going below
    // the base; the window that saw the underrun takes nothing off.
    let mut viewer = Viewer {
        seq: 1,
        ..Viewer::new(0, 0)
    };
    viewer.run(&mut jitter, 3);
    assert_eq!(jitter.stats().underruns, 1);
    let narrowed = jitter.margin();
    assert!(
        (BASE_MARGIN..BASE_MARGIN + MARGIN_STEP).contains(&narrowed),
        "margin {narrowed}"
    );
    viewer.run(&mut jitter, 30);
    assert_eq!(jitter.margin(), BASE_MARGIN);
    assert_eq!(jitter.stats().underruns, 1);
}

#[test]
fn jitter_drops_a_burst_beyond_its_cap() {
    let mut jitter = Jitter::new(1);
    jitter.push(1, &tone(START, 1, 0));
    let mut out = Vec::new();
    jitter.next_packet(&mut out);
    for seq in 2..20 {
        jitter.push(seq, &tone(BURST, 1, 0));
    }
    assert!(jitter.frames() <= CAP_FRAMES);
    assert!(jitter.stats().dropped > 0);
    // Nobody records: what waits is kept fresh and short.
    jitter.stop();
    assert!(jitter.frames() <= START);
}

// --- the device ---

/// The tests call [`Microphone::tick`] themselves.
struct NoClock;

impl PacketClock for NoClock {
    fn wake(&self) {}
}

struct Harness {
    microphone: Arc<Microphone>,
    replies: UnboundedReceiver<Vec<u8>>,
    link: u64,
}

impl Harness {
    fn new() -> Self {
        let microphone = Microphone::new(&MONO, 1, "sebas-1-1-abcd".into(), Arc::new(NoClock));
        let (sender, replies) = unbounded_channel();
        let link = microphone.claim(sender).unwrap();
        Self {
            microphone,
            replies,
            link,
        }
    }

    fn control(&mut self, setup: [u8; 8], data: &[u8]) -> (i32, Vec<u8>) {
        let length = u16::from_le_bytes([setup[6], setup[7]]);
        let direction_in = setup[0] & 0x80 != 0;
        let submit = parse_submit(&submit_header(7, direction_in, 0, u32::from(length), setup));
        assert!(self.microphone.submit(self.link, &submit, data, &[]));
        let reply = self.replies.try_recv().unwrap();
        (be32(&reply, 20) as i32, reply[HEADER_SIZE..].to_vec())
    }

    /// Queues an isochronous IN URB of `packets` 98-byte packets.
    fn iso(&mut self, seqnum: u32, packets: u32) {
        let mut header = submit_header(seqnum, true, 1, packets * 98, [0; 8]);
        header[32..36].copy_from_slice(&(packets as i32).to_be_bytes());
        let submit = parse_submit(&header);
        let mut descriptors = Vec::new();
        for index in 0..packets {
            descriptors.extend_from_slice(&(index * 98).to_be_bytes());
            descriptors.extend_from_slice(&98u32.to_be_bytes());
            descriptors.extend_from_slice(&[0; 8]);
        }
        assert!(
            self.microphone
                .submit(self.link, &submit, &[], &descriptors)
        );
    }
}

fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let mut bytes = [request_type, request, 0, 0, 0, 0, 0, 0];
    bytes[2..4].copy_from_slice(&value.to_le_bytes());
    bytes[4..6].copy_from_slice(&index.to_le_bytes());
    bytes[6..8].copy_from_slice(&length.to_le_bytes());
    bytes
}

#[test]
fn device_answers_enumeration_and_audio_requests() {
    let mut harness = Harness::new();
    let (status, configuration) = harness.control(setup(0x80, 0x06, 0x0200, 0, 255), &[]);
    assert_eq!((status, configuration.as_slice()), (0, MONO.configuration));
    let (_, serial) = harness.control(setup(0x80, 0x06, 0x0303, 0x0409, 255), &[]);
    assert_eq!(serial, string_descriptor(SERIAL));
    assert_eq!(harness.control(setup(0x00, 0x09, 1, 0, 0), &[]).0, 0);
    // Alternate settings of the streaming interface, none of the others.
    assert_eq!(harness.control(setup(0x01, 0x0B, 1, 1, 0), &[]).0, 0);
    assert_eq!(harness.control(setup(0x81, 0x0A, 0, 1, 1), &[]).1, [1]);
    assert_eq!(harness.control(setup(0x01, 0x0B, 2, 1, 0), &[]).0, -32);
    assert_eq!(harness.control(setup(0x01, 0x0B, 1, 0, 0), &[]).0, -32);
    // The endpoint's sampling frequency: 48 kHz only.
    let rate = [0x80, 0xBB, 0x00];
    assert_eq!(
        harness.control(setup(0x22, 0x01, 0x0100, 0x81, 3), &rate).0,
        0
    );
    assert_eq!(
        harness
            .control(setup(0x22, 0x01, 0x0100, 0x81, 3), &[0x44, 0xAC, 0])
            .0,
        -32
    );
    assert_eq!(
        harness.control(setup(0xA2, 0x81, 0x0100, 0x81, 3), &[]).1,
        rate
    );
    // No feature unit: volume requests stall.
    assert_eq!(
        harness.control(setup(0xA1, 0x81, 0x0200, 0x0200, 2), &[]).0,
        -32
    );
}

#[test]
fn isochronous_reads_complete_in_real_time_with_the_viewers_samples() {
    let mut harness = Harness::new();
    let mut in_use = harness.microphone.in_use();
    let start = Instant::now();
    harness.microphone.push(1, &tone(START, 1, 100));
    harness.iso(20, 10);
    harness.iso(21, 10);
    // Nothing is due before its packets' milliseconds have passed.
    let next = harness.microphone.tick(start).unwrap();
    assert!(harness.replies.try_recv().is_err());
    assert!(*in_use.borrow_and_update(), "a recorder streams");
    let due = next.expect("a URB waits");
    assert!(
        due >= start + Duration::from_millis(10)
            && due <= Instant::now() + Duration::from_millis(10)
    );
    harness.microphone.tick(due).unwrap();
    let first = harness.replies.try_recv().unwrap();
    assert!(
        harness.replies.try_recv().is_err(),
        "the second is 10 ms later"
    );
    assert_eq!(be32(&first, 4), 20);
    assert_eq!(be32(&first, 24), 10 * 96, "48 frames per packet");
    assert_eq!(be32(&first, 32), 10);
    assert_eq!(
        samples(&first[HEADER_SIZE..HEADER_SIZE + 96]),
        tone(48, 1, 100)
    );
    let descriptors = &first[HEADER_SIZE + 960..];
    assert_eq!(descriptors.len(), 10 * 16);
    assert_eq!(
        (
            be32(descriptors, 16),
            be32(descriptors, 20),
            be32(descriptors, 24)
        ),
        (98, 98, 96)
    );
    harness
        .microphone
        .tick(due + Duration::from_millis(10))
        .unwrap();
    let second = harness.replies.try_recv().unwrap();
    assert_eq!(be32(&second, 4), 21);
    assert_eq!(
        samples(&second[HEADER_SIZE..HEADER_SIZE + 96]),
        tone(48, 1, 100 + 480)
    );
    // No URB for a while: no app records any more.
    let idle = due + Duration::from_millis(10) + IDLE_AFTER;
    harness.microphone.tick(idle).unwrap();
    assert!(!*in_use.borrow_and_update());
}

#[test]
fn unlinks_cancel_queued_reads_and_resets_forget_them() {
    let mut harness = Harness::new();
    harness.iso(30, 8);
    harness.iso(31, 8);
    assert!(harness.microphone.unlink(harness.link, 40, 30));
    let reply = harness.replies.try_recv().unwrap();
    assert_eq!(
        (be32(&reply, 0), be32(&reply, 4), be32(&reply, 20) as i32),
        (4, 40, -104)
    );
    assert!(harness.microphone.unlink(harness.link, 41, 99));
    assert_eq!(
        be32(&harness.replies.try_recv().unwrap(), 20),
        0,
        "already answered"
    );
    // A port reset forgets the rest: nothing is answered for it.
    assert_eq!(harness.control(setup(0x23, 0x03, 4, 1, 0), &[]).0, 0);
    harness
        .microphone
        .tick(Instant::now() + Duration::from_secs(1))
        .unwrap();
    assert!(harness.replies.try_recv().is_err());
}

#[test]
fn other_endpoints_and_malformed_urbs_stall() {
    let mut harness = Harness::new();
    // An interrupt-style read of the endpoint.
    let submit = parse_submit(&submit_header(50, true, 1, 64, [0; 8]));
    assert!(harness.microphone.submit(harness.link, &submit, &[], &[]));
    assert_eq!(be32(&harness.replies.try_recv().unwrap(), 20) as i32, -32);
    // An isochronous URB whose descriptors don't match its packet count.
    let mut header = submit_header(51, true, 1, 196, [0; 8]);
    header[32..36].copy_from_slice(&2i32.to_be_bytes());
    let submit = parse_submit(&header);
    assert!(
        harness
            .microphone
            .submit(harness.link, &submit, &[], &[0; 16])
    );
    assert_eq!(be32(&harness.replies.try_recv().unwrap(), 20) as i32, -32);
}

#[test]
fn a_released_or_closed_microphone_answers_nothing() {
    let mut harness = Harness::new();
    harness.iso(60, 4);
    harness.microphone.release(harness.link);
    let submit = parse_submit(&submit_header(
        61,
        true,
        0,
        18,
        setup(0x80, 0x06, 0x0100, 0, 18),
    ));
    assert!(!harness.microphone.submit(harness.link, &submit, &[], &[]));
    let (sender, _replies) = unbounded_channel();
    let again = harness.microphone.claim(sender).unwrap();
    harness.microphone.close();
    assert!(harness.microphone.is_closed());
    assert!(!harness.microphone.submit(again, &submit, &[], &[]));
    assert!(
        harness.microphone.tick(Instant::now()).is_none(),
        "the clock stops"
    );
    assert!(harness.replies.try_recv().is_err());
}

/// The device side of a recording that went silent after 150 ms: the
/// stream reaches `usbaudio.sys` whole, 48 frames in every packet, URB
/// after URB on consecutive frame numbers, while the viewer keeps up. (It
/// was Windows' Voice Clarity fading the steady tone out, which Sebas turns
/// off on this device's recording endpoints.)
#[test]
fn a_steady_stream_reaches_the_host_sample_for_sample() {
    let mut harness = Harness::new();
    let mut pushed = 0usize;
    let mut seq = 0;
    let mut push = |microphone: &Microphone, frames: usize| {
        seq += 1;
        microphone.push(seq, &tone(frames, 1, pushed as i16));
        pushed += frames;
    };
    push(&harness.microphone, START);
    // usbaudio.sys keeps about a dozen 10-packet URBs queued.
    for seqnum in 0..12 {
        harness.iso(seqnum, 10);
    }
    let first_due = harness.microphone.tick(Instant::now()).unwrap().unwrap();
    let mut received: Vec<i16> = Vec::new();
    let mut start_frames = Vec::new();
    for urb in 0..100u32 {
        if urb % 2 == 1 {
            push(&harness.microphone, BURST);
        }
        harness
            .microphone
            .tick(first_due + Duration::from_millis(10 * u64::from(urb)))
            .unwrap();
        let reply = harness.replies.try_recv().expect("one URB per 10 ms");
        assert!(harness.replies.try_recv().is_err());
        assert_eq!(be32(&reply, 4), urb);
        assert_eq!(be32(&reply, 20), 0, "status");
        let length = be32(&reply, 24) as usize;
        assert_eq!(length, 10 * 96);
        start_frames.push(be32(&reply, 28));
        let descriptors = &reply[HEADER_SIZE + length..];
        for packet in descriptors.chunks_exact(16) {
            assert_eq!(
                (be32(packet, 8), be32(packet, 12)),
                (96, 0),
                "48 frames, no error"
            );
        }
        received.extend(samples(&reply[HEADER_SIZE..HEADER_SIZE + length]));
        harness.iso(urb + 12, 10);
    }
    assert_eq!(received, tone(100 * 480, 1, 0), "every sample, in order");
    assert!(start_frames.windows(2).all(|pair| pair[1] == pair[0] + 10));
}
