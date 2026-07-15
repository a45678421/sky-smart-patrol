# Device Setup Notes

This project uses three runtime targets: the computer, Nicla Vision, and RP2040.

## Computer

The computer runs:

```text
computer/road_detection_server.py
```

It provides:

- Dashboard UI.
- MJPEG camera stream.
- Remote control API.
- AI road hazard detection.
- LINE notification.

Run from the `computer` folder:

```powershell
python road_detection_server.py
```

## Nicla Vision

Upload:

```text
nicla/main.py
```

Save it on the Nicla Vision as:

```text
main.py
```

Before upload, edit:

```python
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
SERVER_BASE = "http://YOUR_COMPUTER_IP:5000"
DEVICE_API_KEY = "sky_robot_2026_test"
```

The Nicla Vision:

- Captures JPEG frames.
- Uploads frames to `/frame`.
- Uploads telemetry to `/telemetry`.
- Polls `/device/command`.
- Forwards commands to the RP2040 over UART.

## RP2040

Copy these files to the RP2040/CircuitPython filesystem:

```text
rp2040/code.py
rp2040/mcl_localization.py
rp2040/pid_controller.py
rp2040/pio_encoder.py
rp2040/robot.py
```

Optional:

```text
rp2040/code_route_control.py
```

`code_route_control.py` is an alternate route-control copy. Copy it as `code.py` only when you intentionally want to use that version.

## Network Checklist

- Computer and Nicla Vision are on the same Wi-Fi network.
- Windows firewall allows inbound access to Python or port `5000`.
- `SERVER_BASE` in Nicla matches the computer IPv4 address.
- `DEVICE_API_KEY` matches on both sides.

## UART Checklist

- Nicla TX connects to RP2040 RX.
- Nicla RX connects to RP2040 TX.
- GND is shared.
- Baud rate matches in both programs.
