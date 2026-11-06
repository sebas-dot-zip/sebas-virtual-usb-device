//! Pure tests of the USB and USB/IP layer every emulated device shares:
//! USB/IP framing, the import holder and the standard requests.

use crate::{
    Holder,
    test_support::{be32, parse_submit, submit_header},
    usb::{ControlReply, Descriptors, SPEED_HIGH, Setup, standard_request},
    usbip::{self, Command, HEADER_SIZE, IsoInReply, IsoPacket},
};

// --- USB/IP framing ---

#[test]
fn handshake_headers_round_trip() {
    let request = [0x01, 0x11, 0x80, 0x03, 0, 0, 0, 0];
    let header = usbip::OpHeader::parse(&request);
    assert_eq!(
        (header.version, header.code, header.status),
        (0x0111, 0x8003, 0)
    );
    assert_eq!(header.encode(), request);
    assert_eq!(
        usbip::op_reply(0x8003, usbip::ST_NA),
        [0x01, 0x11, 0x00, 0x03, 0, 0, 0, 1]
    );
    let mut bus_id = [0u8; 32];
    bus_id[..5].copy_from_slice(b"sebas");
    assert_eq!(usbip::bus_id(&bus_id), b"sebas");
}

#[test]
fn commands_parse_from_the_wire() {
    let submit = parse_submit(&submit_header(9, false, 2, 8, [0; 8]));
    assert_eq!(submit.seqnum, 9);
    assert_eq!(submit.endpoint_address(), 0x02);
    assert!(!submit.is_isochronous());
    assert_eq!(submit.payload_lengths().unwrap(), (8, 0));
    let read = parse_submit(&submit_header(10, true, 1, 32, [0; 8]));
    assert_eq!(read.endpoint_address(), 0x81);
    assert_eq!(read.payload_lengths().unwrap(), (0, 0));

    let mut iso = submit_header(11, true, 3, 0, [0; 8]);
    iso[32..36].copy_from_slice(&2i32.to_be_bytes());
    assert_eq!(parse_submit(&iso).payload_lengths().unwrap(), (0, 32));
    iso[32..36].copy_from_slice(&5000i32.to_be_bytes());
    assert!(parse_submit(&iso).payload_lengths().is_err());

    let mut unlink = [0u8; HEADER_SIZE];
    unlink[0..4].copy_from_slice(&2u32.to_be_bytes());
    unlink[4..8].copy_from_slice(&12u32.to_be_bytes());
    unlink[20..24].copy_from_slice(&10u32.to_be_bytes());
    assert_eq!(
        usbip::parse_command(&unlink).unwrap(),
        Command::Unlink {
            seqnum: 12,
            victim: 10
        }
    );
    unlink[0..4].copy_from_slice(&7u32.to_be_bytes());
    assert!(usbip::parse_command(&unlink).is_err());
}

#[test]
fn replies_encode_the_driver_rules() {
    let reply = usbip::ret_submit(5, -usbip::EPIPE, 0, &[]);
    assert_eq!(reply.len(), HEADER_SIZE);
    assert_eq!(be32(&reply, 0), 3);
    assert_eq!(be32(&reply, 4), 5);
    assert_eq!(&reply[8..20], &[0; 12]);
    assert_eq!(be32(&reply, 20) as i32, -32);
    assert_eq!(be32(&reply, 32) as i32, -1); // number_of_packets, non-isochronous
    let data = usbip::ret_submit(6, 0, 2, &[0xAA, 0xBB]);
    assert_eq!(be32(&data, 24), 2);
    assert_eq!(&data[HEADER_SIZE..], &[0xAA, 0xBB]);
    let mut frame = usbip::begin_ret_submit(7, 4);
    frame.extend_from_slice(&[1, 2, 3]);
    usbip::finish_ret_submit(&mut frame);
    assert_eq!(be32(&frame, 24), 3);
    let unlink = usbip::ret_unlink(8, -usbip::ECONNRESET);
    assert_eq!(unlink.len(), HEADER_SIZE);
    assert_eq!(
        (be32(&unlink, 0), be32(&unlink, 4), be32(&unlink, 20) as i32),
        (4, 8, -104)
    );
}

// --- isochronous URBs ---

#[test]
fn iso_packets_parse_from_their_descriptors() {
    let mut descriptors = Vec::new();
    for (offset, length) in [(0u32, 98u32), (98, 98), (196, 98)] {
        descriptors.extend_from_slice(&offset.to_be_bytes());
        descriptors.extend_from_slice(&length.to_be_bytes());
        descriptors.extend_from_slice(&[0; 8]); // actual_length, status
    }
    let packets: Vec<IsoPacket> = usbip::iso_packets(&descriptors).collect();
    assert_eq!(
        packets,
        [
            IsoPacket {
                offset: 0,
                length: 98
            },
            IsoPacket {
                offset: 98,
                length: 98
            },
            IsoPacket {
                offset: 196,
                length: 98
            },
        ]
    );
    // A cut descriptor is not a packet.
    assert_eq!(usbip::iso_packets(&descriptors[..20]).count(), 1);
}

#[test]
fn iso_in_reply_compacts_data_and_keeps_offsets() {
    let packet = |offset| IsoPacket { offset, length: 4 };
    let mut reply = IsoInReply::new(9, 1234, 3, 12);
    reply.packet(packet(0), &[1, 2, 3, 4]);
    reply.packet(packet(4), &[5, 6]);
    // More than the packet holds is cut to its length.
    reply.packet(packet(8), &[7, 8, 9, 10, 11]);
    let frame = reply.finish();
    assert_eq!(be32(&frame, 0), 3); // RET_SUBMIT
    assert_eq!(be32(&frame, 4), 9);
    assert_eq!(&frame[8..20], &[0; 12]);
    assert_eq!(be32(&frame, 20), 0); // status
    assert_eq!(be32(&frame, 24), 10, "actual_length is the packets' sum");
    assert_eq!(be32(&frame, 28), 1234); // start_frame
    assert_eq!(be32(&frame, 32), 3); // number_of_packets
    assert_eq!(be32(&frame, 36), 0); // error_count
    assert_eq!(
        &frame[HEADER_SIZE..HEADER_SIZE + 10],
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    );
    let descriptors = &frame[HEADER_SIZE + 10..];
    assert_eq!(descriptors.len(), 3 * usbip::ISO_DESCRIPTOR_SIZE);
    let fields: Vec<[u32; 4]> = descriptors
        .chunks_exact(16)
        .map(|d| [be32(d, 0), be32(d, 4), be32(d, 8), be32(d, 12)])
        .collect();
    assert_eq!(fields, [[0, 4, 4, 0], [4, 4, 2, 0], [8, 4, 4, 0]]);
}

#[test]
fn isochronous_submits_carry_their_descriptors() {
    let mut header = submit_header(4, true, 1, 980, [0; 8]);
    header[32..36].copy_from_slice(&10i32.to_be_bytes());
    let submit = parse_submit(&header);
    assert!(submit.is_isochronous());
    assert_eq!(submit.payload_lengths().unwrap(), (0, 160));
}

// --- import holder ---

#[test]
fn one_import_holds_a_device_until_released_or_closed() {
    let mut holder = Holder::default();
    let (first, mut first_queue) = tokio::sync::mpsc::unbounded_channel();
    let (second, _second_queue) = tokio::sync::mpsc::unbounded_channel();
    let link = holder.claim(first).unwrap();
    assert!(holder.holds(link));
    assert!(holder.claim(second.clone()).is_none(), "already held");
    holder.send(vec![1]);
    assert_eq!(first_queue.try_recv().unwrap(), [1]);
    assert!(!holder.release(link + 1), "another import's release");
    assert!(holder.release(link));
    assert!(!holder.holds(link));
    holder.send(vec![2]);
    assert!(first_queue.try_recv().is_err(), "nobody holds it");
    let again = holder.claim(second.clone()).unwrap();
    assert_ne!(again, link, "a new import gets a new link");
    holder.close();
    assert!(holder.is_closed() && !holder.holds(again));
    assert!(holder.claim(second).is_none(), "closed for good");
}

// --- standard requests ---

#[rustfmt::skip]
static TEST_DEVICE: [u8; 18] = [
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
];

/// One interface with an isochronous IN endpoint at bInterval 4.
#[rustfmt::skip]
static TEST_CONFIGURATION: [u8; 25] = [
    0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x05, 0x62, 0x00, 0x04,
];

static TEST_DESCRIPTORS: Descriptors = Descriptors {
    speed: SPEED_HIGH,
    device: &TEST_DEVICE,
    configuration: &TEST_CONFIGURATION,
    strings: &["Sebas", "Test"],
    hid_report: None,
};

fn request(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
    Setup {
        request_type,
        request,
        value,
        index,
        length,
    }
}

fn answer(reply: ControlReply) -> Option<Vec<u8>> {
    match reply {
        ControlReply::In(bytes) => Some(bytes),
        ControlReply::Accepted => Some(Vec::new()),
        ControlReply::Stall => None,
    }
}

#[test]
fn standard_requests_answer_the_fixed_descriptors() {
    let mut configuration = 0;
    let mut ask = |setup: Setup| {
        answer(standard_request(
            &TEST_DESCRIPTORS,
            "SERIAL-1",
            &mut configuration,
            &setup,
        ))
    };
    assert_eq!(
        ask(request(0x80, 0x06, 0x0100, 0, 18)).unwrap(),
        TEST_DEVICE
    );
    assert_eq!(
        ask(request(0x80, 0x06, 0x0200, 0, 255)).unwrap(),
        TEST_CONFIGURATION
    );
    assert_eq!(
        ask(request(0x80, 0x06, 0x0300, 0, 255)).unwrap(),
        [4, 3, 0x09, 0x04]
    );
    // iSerialNumber (3) is the device's own serial.
    let serial = ask(request(0x80, 0x06, 0x0303, 0x0409, 255)).unwrap();
    assert_eq!(serial, crate::usb::string_descriptor("SERIAL-1"));
    assert!(ask(request(0x80, 0x06, 0x0309, 0x0409, 255)).is_none());
    assert_eq!(ask(request(0x80, 0x00, 0, 0, 2)).unwrap(), [0, 0]);
    // SET_CONFIGURATION takes only the one configuration.
    assert!(ask(request(0x00, 0x09, 2, 0, 0)).is_none());
    assert!(ask(request(0x00, 0x09, 1, 0, 0)).is_some());
    assert_eq!(ask(request(0x80, 0x08, 0, 0, 1)).unwrap(), [1]);
    // A filter marker is a no-op; interface requests are the device's own.
    assert!(ask(request(0x80, 0x1A, 0, 0xFFFF, 0)).unwrap().is_empty());
    assert!(ask(request(0x01, 0x0B, 1, 1, 0)).is_none());
    assert!(request(0x23, 0x03, 4, 1, 0).is_port_reset());
    assert!(!request(0x00, 0x03, 4, 1, 0).is_port_reset());
}

#[test]
fn other_speed_configuration_converts_isochronous_intervals_to_frames() {
    let other = TEST_DESCRIPTORS.standard(0x07, 0).unwrap();
    assert_eq!(other[1], 0x07);
    // bInterval 4: 2^3 microframes at high speed is 2^0 frames at full speed.
    assert_eq!(other[24], 1);
}
