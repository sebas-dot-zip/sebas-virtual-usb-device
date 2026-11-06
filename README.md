# sebas-virtual-usb-device

Virtual USB devices for [Sebas](https://sebas.zip), a remote-support and
remote-desktop platform. Sebas emulates USB devices in software on a Windows
computer during a remote session and serves them over
[USB/IP](https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html) to
the [usbip-win2](https://github.com/vadimgrn/usbip-win2) host controller
driver, so Windows sees them as plugged-in USB devices. This crate, in plain
Rust with no platform code, contains two things:

- **The shared USB/IP device framing**: USB descriptors, setup packets and
  the standard requests (`usb`), the USB/IP wire format (`usbip`), and the
  `Device` interface a USB/IP server drives, with the `Holder` that tracks
  which import holds a device. Sebas's virtual game controllers are built on
  this framing as well, but their descriptors and reports are not part of
  this crate.
- **Sebas Microphone**, the USB Audio Class device that lets every app on the
  Windows computer record a remote viewer's microphone. Windows binds its
  in-box USB Audio Class driver (`usbaudio.sys`) to it, and every Sebas
  installation that hosts a session emulates the same device. The crate holds
  its descriptors (`descriptors`), its answers to standard and audio class
  control requests and the isochronous IN transfers that carry the samples on
  the host's clock (`Microphone`), and the jitter buffer between the viewer's
  20 ms bursts and the host's 1 ms packets (`jitter`).

The Windows parts of Sebas (the USB/IP server, attaching devices to the host
controller, the high-resolution timer, the service and its policy checks)
are not part of this crate.

## Sebas Microphone

|            |                                                                                                |
| ---------- | ---------------------------------------------------------------------------------------------- |
| Class      | USB Audio Class 1.0 (class per interface), USB 2.0 High-Speed                                  |
| Vendor ID  | `0x1209` ([pid.codes](https://pid.codes/1209/))                                                |
| Product ID | `0x0001`, pid.codes' shared test ID, a placeholder until pid.codes assigns this device its own |
| Strings    | manufacturer "Sebas", product "Sebas Microphone", serial "SEBAS-VMIC-1"                        |
| Format     | PCM, type I, 16-bit, 48 kHz, mono or stereo                                                    |

The single configuration (100 bytes, bus-powered, 100 mA) has two interfaces:

- **Interface 0, audio control** (ADC 1.00): input terminal 1 (microphone,
  `0x0201`) feeding output terminal 2 (USB streaming, `0x0101`). There is no
  feature unit, so Windows applies the capture level in software and the
  device never answers volume or mute requests.
- **Interface 1, audio streaming**: alternate setting 0 has no endpoint;
  alternate setting 1 has the isochronous IN endpoint `0x81`, asynchronous,
  `bInterval` 4 (1 ms at High-Speed), with room for 49 sample frames per
  packet (98 bytes mono, 196 stereo), so the device can send 47 to 49 frames
  per millisecond to follow the sender's clock. The endpoint's sampling
  frequency control accepts and reports 48 kHz only.

UAC 1.0 rather than 2.0 because every Windows version Sebas supports has
`usbaudio.sys` (`usbaudio2.sys` arrived with Windows 10 1703) and a
microphone needs none of UAC 2.0's clock entities. Windows' UDE treats every
emulated device as High-Speed, hence the High-Speed descriptors.

## Using it

An application plugs a device in at two points (the microphone shown; any
device built on the `Device` trait plugs into the server the same way):

- Its USB/IP server drives the device through the `Device` trait: the import
  reply, claiming the device for one import with a reply channel, and each
  `CMD_SUBMIT` and `CMD_UNLINK`. The device queues its replies, already
  framed, on that channel.
- Its timer calls `Microphone::tick` and sleeps until the time that returns;
  the device wakes it through the `PacketClock` trait when a URB is queued
  or the device closes. Sebas uses a high-resolution waitable timer on
  Windows, since ordinary sleeps there round up to the 15.6 ms scheduler
  tick.

The application feeds the viewer's interleaved 48 kHz samples to
`Microphone::push` and can watch `Microphone::in_use` to learn whether an app
is recording.

## Build and test

Requires Rust 1.89 or newer (edition 2024).

```text
cargo build
cargo test
```

The tests are pure: they exercise the USB/IP framing, the standard requests,
the microphone's descriptors, its jitter buffer and its USB/IP replies
against in-memory channels. The only dependencies are
`tokio` (its `sync` feature, for the reply channel and the in-use state) and
`tracing`. The `test-support` feature exposes the in-memory USB/IP helpers
for tests of devices built on this crate.

## Source

This repository mirrors `apps/desktop/virtual-usb-device` in the
Sebas source tree, where it is developed alongside the rest of Sebas;
updates are published here from there.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
