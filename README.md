# `usb-audio-rpi`

Experimenting with the [usbd_audio](https://docs.rs/usbd-audio/latest/usbd_audio/)
on RP2040.

## Examples

### Cricket

Flash RP2040 board with the firmware and connect USB to a PC.
It should be recognized as a sound card with one input channel.
A cricket chirp will be audible when playing from this card.

```sh
arecord -c1 -f S16_LE  -r 48000 -t wav -V mono -v | aplay
```
