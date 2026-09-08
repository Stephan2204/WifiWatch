# ESP-WiFiWatch wiring quick reference

## WT32-ETH01 + ST7789

See:

```text
docs/images/WiFiWatch-Wiring-WT32-ETH01.png
```

Always verify the actual ST7789 module pin order and supply voltage.

## Waveshare ESP32-S3-ETH + ST7789

Tested wiring:

```text
ST7789        Waveshare ESP32-S3-ETH
------------------------------------
GND       ->  GND
VCC       ->  VSYS (5 V)
SCL/SCLK  ->  GPIO39
SDA/MOSI  ->  GPIO40
RST/RES   ->  GPIO38
DC        ->  GPIO42
CS        ->  GPIO41
BL/BLK    ->  3V3
```

The onboard W5500 uses GPIO9-GPIO14 internally and must not be reused for the
display.
