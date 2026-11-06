//! USB/IP wire format, protocol version 0x0111: the import handshake and the
//! URB commands, all big-endian.
//!
//! Sources: the kernel's protocol description
//! (<https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html>), and
//! usbip-win2 0.9.8.1 `include/usbip/proto.h`, `proto_op.h` and `consts.h`,
//! which the host controller this module serves is built from. Reply rules
//! the driver's receive path enforces (`drivers/ude/wsk_receive.cpp`), as
//! HIDMaestro's MIT `UsbipProtocol.cs` documents them: a non-isochronous
//! RET_SUBMIT carries `number_of_packets = -1`, the reply's devid, direction
//! and endpoint are zero, an OUT reply carries no data, and RET_UNLINK is
//! `-ECONNRESET` when the URB was still queued and 0 when it was answered.
//! An isochronous IN RET_SUBMIT carries the packets' data back to back
//! (`actual_length` is their sum), then one descriptor per packet with the
//! submit's offset and length, the bytes it holds and its status
//! (`fill_isoc_data`, as Linux's `stub_send_ret_submit` sends them).

use crate::usb::Descriptors;

pub const VERSION: u16 = 0x0111;
pub const OP_REQ_IMPORT: u16 = 0x8003;
pub const OP_REQUEST: u16 = 0x8000;

/// `op_status_t` (usbip-win2 `consts.h`).
pub const ST_OK: u32 = 0;
pub const ST_NA: u32 = 1;
pub const ST_DEV_BUSY: u32 = 2;
pub const ST_NODEV: u32 = 4;

const CMD_SUBMIT: u32 = 1;
const CMD_UNLINK: u32 = 2;
const RET_SUBMIT: u32 = 3;
const RET_UNLINK: u32 = 4;

pub const OP_HEADER_SIZE: usize = 8;
pub const HEADER_SIZE: usize = 48;
pub const BUS_ID_SIZE: usize = 32;
const DEV_PATH_MAX: usize = 256;
/// `usbip_usb_device`: path, bus id, busnum, devnum, speed, ids, classes.
pub const USB_DEVICE_SIZE: usize = DEV_PATH_MAX + BUS_ID_SIZE + 4 * 3 + 2 * 3 + 6;
pub const ISO_DESCRIPTOR_SIZE: usize = 16;
/// `proto.h max_iso_packets`.
pub const MAX_ISO_PACKETS: i32 = 1024;
const NUMBER_OF_PACKETS_NON_ISOCH: i32 = -1;

/// Linux errno values the driver maps to USBD status codes
/// (`libdrv/usbd_helper.cpp`), sent negated.
pub const EPIPE: i32 = 32;
pub const ECONNRESET: i32 = 104;

/// `op_common`: the first 8 bytes of every handshake message.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OpHeader {
    pub version: u16,
    pub code: u16,
    pub status: u32,
}

impl OpHeader {
    pub fn parse(bytes: &[u8; OP_HEADER_SIZE]) -> Self {
        Self {
            version: u16::from_be_bytes([bytes[0], bytes[1]]),
            code: u16::from_be_bytes([bytes[2], bytes[3]]),
            status: u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        }
    }

    pub fn encode(self) -> [u8; OP_HEADER_SIZE] {
        let mut bytes = [0; OP_HEADER_SIZE];
        bytes[..2].copy_from_slice(&self.version.to_be_bytes());
        bytes[2..4].copy_from_slice(&self.code.to_be_bytes());
        bytes[4..].copy_from_slice(&self.status.to_be_bytes());
        bytes
    }
}

/// The reply to a request `code` with `status` and no body.
pub fn op_reply(code: u16, status: u32) -> [u8; OP_HEADER_SIZE] {
    OpHeader {
        version: VERSION,
        code: code & !OP_REQUEST,
        status,
    }
    .encode()
}

/// The bus id an OP_REQ_IMPORT names, up to its first NUL.
pub fn bus_id(bytes: &[u8; BUS_ID_SIZE]) -> &[u8] {
    let end = bytes
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(BUS_ID_SIZE);
    &bytes[..end]
}

/// OP_REP_IMPORT with ST_OK and the device block. `devid` is busnum << 16 |
/// devnum, which later commands echo.
pub fn import_reply(bus_id: &str, devid: u32, descriptors: &Descriptors) -> Vec<u8> {
    let mut reply = Vec::with_capacity(OP_HEADER_SIZE + USB_DEVICE_SIZE);
    reply.extend_from_slice(&op_reply(OP_REQ_IMPORT, ST_OK));
    let mut path = [0u8; DEV_PATH_MAX];
    let path_text = format!("/sebas/{bus_id}");
    let path_len = path_text.len().min(DEV_PATH_MAX - 1);
    path[..path_len].copy_from_slice(&path_text.as_bytes()[..path_len]);
    reply.extend_from_slice(&path);
    let mut bus = [0u8; BUS_ID_SIZE];
    let bus_len = bus_id.len().min(BUS_ID_SIZE - 1);
    bus[..bus_len].copy_from_slice(&bus_id.as_bytes()[..bus_len]);
    reply.extend_from_slice(&bus);
    reply.extend_from_slice(&(devid >> 16).to_be_bytes());
    reply.extend_from_slice(&(devid & 0xFFFF).to_be_bytes());
    reply.extend_from_slice(&descriptors.speed.to_be_bytes());
    reply.extend_from_slice(&descriptors.vendor_id().to_be_bytes());
    reply.extend_from_slice(&descriptors.product_id().to_be_bytes());
    reply.extend_from_slice(&descriptors.bcd_device().to_be_bytes());
    let device = descriptors.device;
    reply.extend_from_slice(&[
        device[4],
        device[5],
        device[6],
        descriptors.configuration_value(),
        device[17],
        descriptors.num_interfaces(),
    ]);
    reply
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Submit {
    pub seqnum: u32,
    pub devid: u32,
    pub direction_in: bool,
    /// The endpoint number, without the direction bit.
    pub endpoint: u8,
    pub transfer_flags: u32,
    pub transfer_buffer_length: u32,
    pub number_of_packets: i32,
    pub interval: i32,
    pub setup: [u8; 8],
}

impl Submit {
    pub fn endpoint_address(&self) -> u8 {
        self.endpoint | if self.direction_in { 0x80 } else { 0 }
    }

    pub fn is_isochronous(&self) -> bool {
        self.number_of_packets != NUMBER_OF_PACKETS_NON_ISOCH
    }

    /// Bytes that follow the header on the wire: OUT data, then one
    /// descriptor per isochronous packet.
    pub fn payload_lengths(&self) -> Result<(usize, usize), String> {
        let data = if self.direction_in {
            0
        } else {
            self.transfer_buffer_length as usize
        };
        let iso = if self.is_isochronous() {
            if !(0..=MAX_ISO_PACKETS).contains(&self.number_of_packets) {
                return Err(format!(
                    "USB/IP submit with {} isochronous packets",
                    self.number_of_packets
                ));
            }
            self.number_of_packets as usize * ISO_DESCRIPTOR_SIZE
        } else {
            0
        };
        Ok((data, iso))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
    Submit(Submit),
    Unlink { seqnum: u32, victim: u32 },
}

fn be32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_be_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

pub fn parse_command(header: &[u8; HEADER_SIZE]) -> Result<Command, String> {
    let seqnum = be32(header, 4);
    match be32(header, 0) {
        CMD_SUBMIT => {
            let mut setup = [0; 8];
            setup.copy_from_slice(&header[40..48]);
            Ok(Command::Submit(Submit {
                seqnum,
                devid: be32(header, 8),
                direction_in: be32(header, 12) == 1,
                endpoint: (be32(header, 16) & 0x0F) as u8,
                transfer_flags: be32(header, 20),
                transfer_buffer_length: be32(header, 24),
                number_of_packets: be32(header, 32) as i32,
                interval: be32(header, 36) as i32,
                setup,
            }))
        }
        CMD_UNLINK => Ok(Command::Unlink {
            seqnum,
            victim: be32(header, 20),
        }),
        other => Err(format!("Unexpected USB/IP command {other}")),
    }
}

/// RET_SUBMIT for a non-isochronous URB. `data` is the IN data (empty for
/// OUT); `actual_length` is `data.len()` for IN and the bytes taken for OUT.
pub fn ret_submit(seqnum: u32, status: i32, actual_length: u32, data: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(HEADER_SIZE + data.len());
    ret_submit_header(&mut frame, seqnum, status, actual_length);
    frame.extend_from_slice(data);
    frame
}

/// Starts a RET_SUBMIT for up to `capacity` bytes of IN data the caller
/// appends; [`finish_ret_submit`] writes the length once it is known.
pub fn begin_ret_submit(seqnum: u32, capacity: usize) -> Vec<u8> {
    let mut frame = Vec::with_capacity(HEADER_SIZE + capacity);
    ret_submit_header(&mut frame, seqnum, 0, 0);
    frame
}

fn ret_submit_header(frame: &mut Vec<u8>, seqnum: u32, status: i32, actual_length: u32) {
    frame.extend_from_slice(&RET_SUBMIT.to_be_bytes());
    frame.extend_from_slice(&seqnum.to_be_bytes());
    // devid, direction and ep are zero in every reply.
    frame.extend_from_slice(&[0; 12]);
    frame.extend_from_slice(&status.to_be_bytes());
    frame.extend_from_slice(&actual_length.to_be_bytes());
    frame.extend_from_slice(&0i32.to_be_bytes()); // start_frame
    frame.extend_from_slice(&NUMBER_OF_PACKETS_NON_ISOCH.to_be_bytes());
    frame.extend_from_slice(&0i32.to_be_bytes()); // error_count
    frame.extend_from_slice(&[0; 8]); // setup
}

pub fn finish_ret_submit(frame: &mut [u8]) {
    let length = (frame.len() - HEADER_SIZE) as u32;
    frame[24..28].copy_from_slice(&length.to_be_bytes());
}

/// One packet of an isochronous CMD_SUBMIT, from its descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IsoPacket {
    /// Where the packet starts in the URB's buffer.
    pub offset: u32,
    /// The most bytes the packet may hold.
    pub length: u32,
}

/// The packets of an isochronous CMD_SUBMIT: the descriptors after its
/// header and OUT data, [`ISO_DESCRIPTOR_SIZE`] bytes each.
pub fn iso_packets(descriptors: &[u8]) -> impl Iterator<Item = IsoPacket> + '_ {
    descriptors
        .chunks_exact(ISO_DESCRIPTOR_SIZE)
        .map(|descriptor| IsoPacket {
            offset: be32(descriptor, 0),
            length: be32(descriptor, 4),
        })
}

/// RET_SUBMIT for an isochronous IN URB, built one packet at a time.
pub struct IsoInReply {
    frame: Vec<u8>,
    descriptors: Vec<u8>,
}

impl IsoInReply {
    /// `packets` is how many the submit had; `capacity` the data expected.
    pub fn new(seqnum: u32, start_frame: u32, packets: usize, capacity: usize) -> Self {
        let mut frame = Vec::with_capacity(HEADER_SIZE + capacity + packets * ISO_DESCRIPTOR_SIZE);
        frame.extend_from_slice(&RET_SUBMIT.to_be_bytes());
        frame.extend_from_slice(&seqnum.to_be_bytes());
        frame.extend_from_slice(&[0; 12]);
        frame.extend_from_slice(&0i32.to_be_bytes()); // status
        frame.extend_from_slice(&0u32.to_be_bytes()); // actual_length, at the end
        frame.extend_from_slice(&start_frame.to_be_bytes());
        frame.extend_from_slice(&(packets as i32).to_be_bytes());
        frame.extend_from_slice(&0i32.to_be_bytes()); // error_count
        frame.extend_from_slice(&[0; 8]); // setup
        Self {
            frame,
            descriptors: Vec::with_capacity(packets * ISO_DESCRIPTOR_SIZE),
        }
    }

    /// The next packet's data, cut to the packet's length.
    pub fn packet(&mut self, packet: IsoPacket, data: &[u8]) {
        let data = &data[..data.len().min(packet.length as usize)];
        self.frame.extend_from_slice(data);
        self.descriptors
            .extend_from_slice(&packet.offset.to_be_bytes());
        self.descriptors
            .extend_from_slice(&packet.length.to_be_bytes());
        self.descriptors
            .extend_from_slice(&(data.len() as u32).to_be_bytes());
        self.descriptors.extend_from_slice(&0u32.to_be_bytes()); // status
    }

    pub fn finish(mut self) -> Vec<u8> {
        let length = (self.frame.len() - HEADER_SIZE) as u32;
        self.frame[24..28].copy_from_slice(&length.to_be_bytes());
        self.frame.extend_from_slice(&self.descriptors);
        self.frame
    }
}

pub fn ret_unlink(seqnum: u32, status: i32) -> Vec<u8> {
    let mut frame = Vec::with_capacity(HEADER_SIZE);
    frame.extend_from_slice(&RET_UNLINK.to_be_bytes());
    frame.extend_from_slice(&seqnum.to_be_bytes());
    frame.extend_from_slice(&[0; 12]);
    frame.extend_from_slice(&status.to_be_bytes());
    frame.extend_from_slice(&[0; 24]);
    frame
}
