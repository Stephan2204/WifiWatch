# Firmware images

This directory is populated automatically by `pio run`.

For each supported hardware platform the build creates **two** images:

- `*-OTA.bin` — application image for ESP-WiFiWatch's Web OTA updater.
- `*-Factory.bin` — merged first-install image containing bootloader, partition table, OTA initialization data (when supplied by the framework) and application.

Expected files after a full build:

```text
ESP-WiFiWatch-v0.4.8-WT32-ETH01-OTA.bin
ESP-WiFiWatch-v0.4.8-WT32-ETH01-Factory.bin
ESP-WiFiWatch-v0.4.8-Waveshare-ESP32-S3-ETH-OTA.bin
ESP-WiFiWatch-v0.4.8-Waveshare-ESP32-S3-ETH-Factory.bin
```

## Which image should I use?

### OTA

Use `*-OTA.bin` in the ESP-WiFiWatch web interface when the device is already running ESP-WiFiWatch.

### Factory

Use `*-Factory.bin` for a blank device / complete first installation with an ESP flashing tool. The merged image is written starting at flash address `0x0`.

For development, the simplest first installation remains PlatformIO itself:

```bash
pio run -e wt32-eth01 -t upload
pio run -e waveshare-esp32-s3-eth -t upload
```

Never flash an image for the other hardware platform.
