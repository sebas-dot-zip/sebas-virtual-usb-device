//! Helpers for tests of devices built on this crate (the `test-support`
//! feature): wire fields and USB/IP commands built in memory.

use crate::usbip::{self, Command, HEADER_SIZE};

pub fn be32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

pub fn le16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

/// A CMD_SUBMIT header (non-isochronous; set `number_of_packets` at 32..36).
pub fn submit_header(
    seqnum: u32,
    direction_in: bool,
    endpoint: u32,
    length: u32,
    setup: [u8; 8],
) -> [u8; HEADER_SIZE] {
    let mut header = [0u8; HEADER_SIZE];
    header[0..4].copy_from_slice(&1u32.to_be_bytes());
    header[4..8].copy_from_slice(&seqnum.to_be_bytes());
    header[8..12].copy_from_slice(&0x0001_0001u32.to_be_bytes());
    header[12..16].copy_from_slice(&u32::from(direction_in).to_be_bytes());
    header[16..20].copy_from_slice(&endpoint.to_be_bytes());
    header[24..28].copy_from_slice(&length.to_be_bytes());
    header[32..36].copy_from_slice(&(-1i32).to_be_bytes());
    header[40..48].copy_from_slice(&setup);
    header
}

pub fn parse_submit(header: &[u8; HEADER_SIZE]) -> usbip::Submit {
    match usbip::parse_command(header).unwrap() {
        Command::Submit(submit) => submit,
        Command::Unlink { .. } => panic!("expected a submit"),
    }
}
