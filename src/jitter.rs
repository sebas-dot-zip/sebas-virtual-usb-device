//! The microphone's jitter buffer: the viewer's samples arrive in bursts
//! (about every 20 ms, with network jitter) and leave at the host's pace,
//! one 1 ms packet at a time.
//!
//! It keeps a safety margin of samples below the bursts' sawtooth: it starts
//! a stream once a burst plus the margin wait ([`Jitter::start_frames`]),
//! and plays silence (and starts over) when it runs dry. Each underrun
//! widens the margin by [`MARGIN_STEP`] up to [`MAX_MARGIN`]; each
//! [`WINDOW_PACKETS`] without one narrows it by a millisecond back towards
//! [`BASE_MARGIN`], so a steady network keeps latency low and a jittery one
//! stops breaking up. It follows the viewer's clock with the asynchronous
//! endpoint's freedom to send 47 or 49 frames instead of 48: once per window
//! it looks at the lowest fill of the window (the bottom of the sawtooth)
//! and, once that left a band around the margin, steers it back by up to
//! `MAX_SPREAD` frames over the next window, so clock drift never costs a
//! sample; only a low more than `DROP_ABOVE` over the margin (a burst
//! after a stall) drops the excess at once. More than [`CAP_FRAMES`]
//! waiting drops the oldest down to the start level, and while nobody
//! records only the start level is kept, so a recording starts with fresh
//! samples. On a steady network a sample waits the margin plus half a burst
//! plus up to 10 ms of band: 20 to 30 ms (30 ms measured with the lab tone).

use std::collections::VecDeque;

use crate::descriptors::FRAMES_PER_PACKET;

const FRAMES_PER_MS: usize = 48;
/// One burst: the viewer's 20 ms frames.
const BURST_FRAMES: usize = 20 * FRAMES_PER_MS;
pub const BASE_MARGIN: usize = 10 * FRAMES_PER_MS;
pub const MARGIN_STEP: usize = 10 * FRAMES_PER_MS;
pub const MAX_MARGIN: usize = 60 * FRAMES_PER_MS;
/// Packets per steering window: one second.
pub const WINDOW_PACKETS: u32 = 1000;
/// The most frames one window steers: every other packet one frame off.
const MAX_SPREAD: usize = WINDOW_PACKETS as usize / 2;
/// A window low this far over the margin is dropped at once rather than
/// steered away over seconds.
const DROP_ABOVE: usize = 2 * BURST_FRAMES;
/// The most frames that may wait.
pub const CAP_FRAMES: usize = 120 * FRAMES_PER_MS;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Stats {
    /// Packets that ran dry.
    pub underruns: u64,
    /// Frames dropped as too much or too old.
    pub dropped: u64,
    /// Pushes older than one already taken.
    pub stale: u64,
}

pub struct Jitter {
    channels: usize,
    samples: VecDeque<i16>,
    /// Silence until [`Jitter::start_frames`] wait.
    priming: bool,
    /// Frames kept below the sawtooth.
    margin: usize,
    window_low: usize,
    window_left: u32,
    window_underruns: u64,
    /// Packets still to carry one frame more (positive) or less (negative).
    adjust: i32,
    last_seq: Option<u32>,
    stats: Stats,
}

impl Jitter {
    pub fn new(channels: u8) -> Self {
        let channels = usize::from(channels.max(1));
        Self {
            channels,
            samples: VecDeque::with_capacity((CAP_FRAMES + BURST_FRAMES) * channels),
            priming: true,
            margin: BASE_MARGIN,
            window_low: usize::MAX,
            window_left: WINDOW_PACKETS,
            window_underruns: 0,
            adjust: 0,
            last_seq: None,
            stats: Stats::default(),
        }
    }

    pub fn frames(&self) -> usize {
        self.samples.len() / self.channels
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    #[cfg(test)]
    pub fn margin(&self) -> usize {
        self.margin
    }

    /// Waiting frames that start (or restart) a stream: a burst plus the
    /// margin.
    pub fn start_frames(&self) -> usize {
        BURST_FRAMES + self.margin
    }

    /// Interleaved samples from the viewer, `seq` counting pushes; a push
    /// not newer than the last one is dropped. A partial frame at the end is
    /// cut off.
    pub fn push(&mut self, seq: u32, pcm: &[i16]) {
        if let Some(last) = self.last_seq
            && (seq.wrapping_sub(last) as i32) <= 0
        {
            self.stats.stale += 1;
            return;
        }
        self.last_seq = Some(seq);
        let whole = pcm.len() - pcm.len() % self.channels;
        self.samples.extend(&pcm[..whole]);
        let start = self.start_frames();
        let limit = if self.priming { start } else { CAP_FRAMES };
        if self.frames() > limit {
            let excess = self.frames() - start;
            self.drop_oldest(excess);
        }
    }

    /// The stream stopped (no app records): start over with the next one,
    /// from fresh samples.
    pub fn stop(&mut self) {
        self.priming = true;
        self.adjust = 0;
        let start = self.start_frames();
        if self.frames() > start {
            let excess = self.frames() - start;
            self.drop_oldest(excess);
        }
    }

    /// Appends the next packet's samples, little-endian, to `out`; returns
    /// how many frames it holds (47 to 49).
    pub fn next_packet(&mut self, out: &mut Vec<u8>) -> usize {
        if self.priming {
            if self.frames() < self.start_frames() {
                silence(out, FRAMES_PER_PACKET * self.channels);
                self.count_window_packet();
                return FRAMES_PER_PACKET;
            }
            self.priming = false;
            self.window_low = usize::MAX;
            self.window_left = WINDOW_PACKETS;
            self.adjust = 0;
        }
        let frames = match self.adjust.signum() {
            1 => {
                self.adjust -= 1;
                FRAMES_PER_PACKET + 1
            }
            -1 => {
                self.adjust += 1;
                FRAMES_PER_PACKET - 1
            }
            _ => FRAMES_PER_PACKET,
        };
        let available = self.frames().min(frames);
        for sample in self.samples.drain(..available * self.channels) {
            out.extend_from_slice(&sample.to_le_bytes());
        }
        if available < frames {
            silence(out, (frames - available) * self.channels);
            self.stats.underruns += 1;
            self.window_underruns += 1;
            self.margin = (self.margin + MARGIN_STEP).min(MAX_MARGIN);
            self.priming = true;
            self.adjust = 0;
            return frames;
        }
        self.window_low = self.window_low.min(self.frames());
        self.count_window_packet();
        frames
    }

    fn count_window_packet(&mut self) {
        self.window_left -= 1;
        if self.window_left == 0 {
            self.end_window();
        }
    }

    /// Narrows the margin after a window without underruns, and steers the
    /// window's low towards it.
    fn end_window(&mut self) {
        let low = self.window_low;
        let underruns = self.window_underruns;
        self.window_low = usize::MAX;
        self.window_left = WINDOW_PACKETS;
        self.window_underruns = 0;
        if underruns == 0 {
            self.margin = self.margin.saturating_sub(FRAMES_PER_MS).max(BASE_MARGIN);
        }
        if self.priming || low == usize::MAX {
            return;
        }
        let band = self.margin / 2..=self.margin + 10 * FRAMES_PER_MS;
        if band.contains(&low) {
            self.adjust = 0;
        } else if low > self.margin {
            let excess = low - self.margin;
            if excess > DROP_ABOVE {
                self.drop_oldest(excess);
                self.adjust = 0;
            } else {
                self.adjust = excess.min(MAX_SPREAD) as i32;
            }
        } else {
            self.adjust = -((self.margin - low).min(MAX_SPREAD) as i32);
        }
    }

    fn drop_oldest(&mut self, frames: usize) {
        let frames = frames.min(self.frames());
        self.samples.drain(..frames * self.channels);
        self.stats.dropped += frames as u64;
    }
}

fn silence(out: &mut Vec<u8>, samples: usize) {
    out.resize(out.len() + samples * 2, 0);
}
