//! USB descriptors, setup packets and the standard requests shared by the
//! emulated devices (USB 2.0 chapter 9, HID 1.11 section 7).

/// `usb_device_speed` as the USB/IP import reply carries it
/// (usbip-win2 `include/usbip/ch9.h`): 2 full speed, 3 high speed.
pub const SPEED_FULL: u32 = 2;
pub const SPEED_HIGH: u32 = 3;

const DESCRIPTOR_DEVICE: u8 = 0x01;
const DESCRIPTOR_CONFIGURATION: u8 = 0x02;
pub const DESCRIPTOR_STRING: u8 = 0x03;
const DESCRIPTOR_INTERFACE: u8 = 0x04;
const DESCRIPTOR_ENDPOINT: u8 = 0x05;
const DESCRIPTOR_DEVICE_QUALIFIER: u8 = 0x06;
const DESCRIPTOR_OTHER_SPEED_CONFIGURATION: u8 = 0x07;
pub const DESCRIPTOR_HID: u8 = 0x21;
pub const DESCRIPTOR_HID_REPORT: u8 = 0x22;
/// US English, the only language the devices report.
const LANGUAGE_EN_US: u16 = 0x0409;
/// GET_FIRMWARE_STATUS (USB 3.2 9.4): usbip-win2's filter driver tags its
/// select-configuration, select-interface and pipe-reset requests with it
/// (`drivers/ude_filter/request.h`, wIndex 0xFFFF where a real request has
/// zero). Without the filter none arrive, but an older filter's marker can
/// reach the device as a plain request; it is answered as a no-op.
const REQUEST_GET_FIRMWARE_STATUS: u8 = 0x1A;
const FILTER_MARKER_INDEX: u16 = 0xFFFF;
/// SET_FEATURE(PORT_RESET) to the hub port, which usbip-win2 forwards when
/// Windows resets the port (`device_ioctl.cpp make_reset_port`).
const PORT_RESET_REQUEST_TYPE: u8 = 0x23;
const REQUEST_SET_FEATURE: u8 = 0x03;

/// How a device answers a control transfer.
pub enum ControlReply {
    /// Data to the host, cut to the request's length.
    In(Vec<u8>),
    /// An OUT request taken, with its data stage.
    Accepted,
    Stall,
}

/// One emulated device's descriptors, all fixed at build time.
pub struct Descriptors {
    pub speed: u32,
    pub device: &'static [u8; 18],
    /// The only configuration, with its interface, class and endpoint
    /// descriptors.
    pub configuration: &'static [u8],
    /// String descriptors 1 and up; index 0 is the language list.
    pub strings: &'static [&'static str],
    /// The HID report descriptor of interface 0, for HID devices.
    pub hid_report: Option<&'static [u8]>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Endpoint {
    /// bEndpointAddress, direction bit included.
    pub address: u8,
    /// bmAttributes bits 1..0: 0 control, 1 isochronous, 2 bulk, 3 interrupt.
    pub transfer_type: u8,
}

impl Descriptors {
    pub fn vendor_id(&self) -> u16 {
        u16::from_le_bytes([self.device[8], self.device[9]])
    }

    pub fn product_id(&self) -> u16 {
        u16::from_le_bytes([self.device[10], self.device[11]])
    }

    pub fn bcd_device(&self) -> u16 {
        u16::from_le_bytes([self.device[12], self.device[13]])
    }

    pub fn configuration_value(&self) -> u8 {
        self.configuration[5]
    }

    pub fn num_interfaces(&self) -> u8 {
        self.configuration[4]
    }

    /// bmAttributes bit 6: the device reports itself self-powered in GET_STATUS.
    pub fn self_powered(&self) -> bool {
        self.configuration[7] & 0x40 != 0
    }

    /// The descriptors inside the configuration, each as its own slice.
    fn configuration_parts(&self) -> impl Iterator<Item = &'static [u8]> {
        let configuration = self.configuration;
        let mut offset = 0;
        std::iter::from_fn(move || {
            let length = usize::from(*configuration.get(offset)?);
            if length < 2 || offset + length > configuration.len() {
                return None;
            }
            let part = &configuration[offset..offset + length];
            offset += length;
            Some(part)
        })
    }

    pub fn endpoints(&self) -> impl Iterator<Item = Endpoint> {
        self.configuration_parts()
            .filter(|part| part[1] == DESCRIPTOR_ENDPOINT && part.len() >= 7)
            .map(|part| Endpoint {
                address: part[2],
                transfer_type: part[3] & 0x03,
            })
    }

    /// The class descriptor of a HID interface (type 0x21), from the
    /// configuration.
    pub fn hid_descriptor(&self, interface: u8) -> Option<&'static [u8]> {
        let mut current = None;
        self.configuration_parts().find(|part| match part[1] {
            DESCRIPTOR_INTERFACE if part.len() >= 9 => {
                current = Some(part[2]);
                false
            }
            DESCRIPTOR_HID => current == Some(interface) && part.len() >= 9,
            _ => false,
        })
    }

    /// A string descriptor: UTF-16LE after the two header bytes.
    pub fn string(&self, index: u8) -> Option<Vec<u8>> {
        if index == 0 {
            let [low, high] = LANGUAGE_EN_US.to_le_bytes();
            return Some(vec![4, DESCRIPTOR_STRING, low, high]);
        }
        let text = self.strings.get(usize::from(index) - 1)?;
        Some(string_descriptor(text))
    }

    /// GET_DESCRIPTOR for the device recipient, or None to stall.
    pub fn standard(&self, descriptor_type: u8, index: u8) -> Option<Vec<u8>> {
        match descriptor_type {
            DESCRIPTOR_DEVICE => Some(self.device.to_vec()),
            DESCRIPTOR_CONFIGURATION if index == 0 => Some(self.configuration.to_vec()),
            DESCRIPTOR_STRING => self.string(index),
            // A full-speed-only device stalls both (USB 2.0 9.6.2, 9.6.4).
            DESCRIPTOR_DEVICE_QUALIFIER if self.speed == SPEED_HIGH => {
                let device = self.device;
                Some(vec![
                    10,
                    DESCRIPTOR_DEVICE_QUALIFIER,
                    device[2],
                    device[3],
                    device[4],
                    device[5],
                    device[6],
                    device[7],
                    device[17],
                    0,
                ])
            }
            DESCRIPTOR_OTHER_SPEED_CONFIGURATION if self.speed == SPEED_HIGH && index == 0 => {
                // The same functions at full speed: interrupt intervals in
                // frames rather than in 2^(n-1) microframes, isochronous ones
                // in 2^(n-1) frames rather than microframes.
                let mut other = self.configuration.to_vec();
                other[1] = DESCRIPTOR_OTHER_SPEED_CONFIGURATION;
                let mut offset = 0;
                while offset + 7 <= other.len() && other[offset] >= 2 {
                    let length = usize::from(other[offset]);
                    if other[offset + 1] == DESCRIPTOR_ENDPOINT {
                        let interval = other[offset + 6].clamp(1, 16);
                        match other[offset + 3] & 0x03 {
                            3 => {
                                other[offset + 6] =
                                    (1u32 << (interval - 1)).div_ceil(8).min(255) as u8;
                            }
                            1 => other[offset + 6] = interval.saturating_sub(3).max(1),
                            _ => {}
                        }
                    }
                    offset += length;
                }
                Some(other)
            }
            _ => None,
        }
    }
}

pub fn string_descriptor(text: &str) -> Vec<u8> {
    let mut descriptor = vec![0, DESCRIPTOR_STRING];
    for unit in text.encode_utf16().take(126) {
        descriptor.extend_from_slice(&unit.to_le_bytes());
    }
    descriptor[0] = descriptor.len() as u8;
    descriptor
}

/// A control transfer's setup packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Setup {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl Setup {
    pub fn parse(bytes: [u8; 8]) -> Self {
        Self {
            request_type: bytes[0],
            request: bytes[1],
            value: u16::from_le_bytes([bytes[2], bytes[3]]),
            index: u16::from_le_bytes([bytes[4], bytes[5]]),
            length: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    pub fn device_to_host(&self) -> bool {
        self.request_type & 0x80 != 0
    }

    /// 0 standard, 1 class, 2 vendor.
    pub fn kind(&self) -> u8 {
        (self.request_type >> 5) & 0x03
    }

    /// 0 device, 1 interface, 2 endpoint, 3 other.
    pub fn recipient(&self) -> u8 {
        self.request_type & 0x1F
    }

    /// SET_FEATURE(PORT_RESET): the host reset the port; the device starts
    /// over unconfigured.
    pub fn is_port_reset(&self) -> bool {
        self.request_type == PORT_RESET_REQUEST_TYPE && self.request == REQUEST_SET_FEATURE
    }
}

/// The standard requests every emulated device answers alike (USB 2.0 9.4):
/// status, features, device descriptors and strings (`serial` for the
/// device's iSerialNumber), and its one configuration. Interface requests
/// are each device's own; here they stall.
pub fn standard_request(
    descriptors: &Descriptors,
    serial: &str,
    configuration: &mut u8,
    setup: &Setup,
) -> ControlReply {
    let [descriptor_index, descriptor_type] = setup.value.to_le_bytes();
    let reply = match (setup.request, setup.device_to_host(), setup.recipient()) {
        // GET_STATUS
        (0x00, true, 0) => Some(vec![u8::from(descriptors.self_powered()), 0]),
        (0x00, true, _) => Some(vec![0, 0]),
        // CLEAR_FEATURE, SET_FEATURE (remote wakeup, endpoint halt), and
        // SET_ADDRESS, which UDE handles before it would get here.
        (0x01 | 0x03 | 0x05, false, _) => return ControlReply::Accepted,
        // GET_DESCRIPTOR
        (0x06, true, 0) if descriptor_type == DESCRIPTOR_STRING => {
            if descriptor_index != 0 && descriptor_index == descriptors.device[16] {
                Some(string_descriptor(serial))
            } else {
                descriptors.string(descriptor_index)
            }
        }
        (0x06, true, 0) => descriptors.standard(descriptor_type, descriptor_index),
        // GET_CONFIGURATION
        (0x08, true, 0) => Some(vec![*configuration]),
        // SET_CONFIGURATION: the one configuration, or none.
        (0x09, false, 0) => {
            let value = descriptor_index;
            if value != 0 && value != descriptors.configuration_value() {
                return ControlReply::Stall;
            }
            *configuration = value;
            return ControlReply::Accepted;
        }
        (REQUEST_GET_FIRMWARE_STATUS, _, _) if setup.index == FILTER_MARKER_INDEX => {
            tracing::debug!("Ignoring a usbip-win2 filter marker request");
            Some(Vec::new())
        }
        _ => None,
    };
    reply.map_or(ControlReply::Stall, ControlReply::In)
}
