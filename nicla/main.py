"""Nicla Vision Wi-Fi camera + UART bridge.

Save this file to the Nicla Vision internal filesystem as main.py.
It uploads JPEG frames and RP2040 telemetry to the Flask server, polls web
commands, and forwards those commands to the RP2040 over UART.
"""

import gc
import json
import network
import requests
import sensor
import time
from machine import UART


WIFI_SSID = "MCUTee304"
WIFI_PASSWORD = "mcutee304"

SERVER_BASE = "http://192.168.0.110:5000"
DEVICE_API_KEY = "sky_robot_2026_test"

FRAME_INTERVAL_MS = 800
COMMAND_INTERVAL_MS = 150
TELEMETRY_INTERVAL_MS = 250
JPEG_QUALITY = 45

# OpenMV firmware exposes UART4 on the Nicla SDA/SCL header pins.
# Nicla SDA / UART4 TX -> RP2040 GP13 RX
# Nicla SCL / UART4 RX <- RP2040 GP12 TX
UART_BAUDRATE = 115200
uart = UART(4, baudrate=UART_BAUDRATE)


def close_response(response):
    if response is not None and hasattr(response, "close"):
        response.close()


def decode_response_json(response):
    try:
        return response.json()
    except Exception:
        pass

    try:
        content = response.content
        if isinstance(content, bytes):
            content = content.decode("utf-8")
        return json.loads(content)
    except Exception:
        return {}


def post_json(path, payload):
    response = None

    try:
        body = json.dumps(payload)

        response = requests.post(
            SERVER_BASE + path,
            data=body,
            headers={
                "Content-Type": "application/json",
                "X-API-Key": DEVICE_API_KEY,
            },
        )

        status = response.status_code

        if status != 200:
            print(
                "POST",
                path,
                "HTTP:",
                status,
                "payload bytes:",
                len(body),
            )

            try:
                print("Server response:", decode_response_json(response))
            except Exception as error:
                print("Cannot decode response:", error)

        return status

    finally:
        close_response(response)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1500)
print("Camera initialized")

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Connecting Wi-Fi...")
wlan.connect(WIFI_SSID, WIFI_PASSWORD)
start_time = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), start_time) > 20000:
        raise RuntimeError("Wi-Fi connection timeout")
    print(".", end="")
    time.sleep_ms(300)
print("\nWi-Fi connected")
print("Nicla IP:", wlan.ifconfig()[0])

last_frame_ms = time.ticks_ms() - FRAME_INTERVAL_MS
last_command_ms = time.ticks_ms() - COMMAND_INTERVAL_MS
last_telemetry_ms = time.ticks_ms()
last_command_sequence = -1
rx_buffer = b""
latest_telemetry = None
telemetry_dirty = False
frame_number = 0

while True:
    now = time.ticks_ms()

    # Read newline-delimited telemetry from RP2040.
    # Read newline-delimited telemetry from RP2040.
    try:
        while uart.any():
            chunk = uart.read()
            if chunk:
                rx_buffer += chunk

        # 防止收到不完整或損壞的 UART 資料後，
        # rx_buffer 因為一直等不到換行而持續增加。
        if len(rx_buffer) > 4096:
            print(
                "UART buffer overflow, clearing:",
                len(rx_buffer),
            )
            rx_buffer = b""

        while b"\n" in rx_buffer:
            line, rx_buffer = rx_buffer.split(b"\n", 1)
            line = line.strip()

            if not line:
                continue

            try:
                message = json.loads(line.decode("utf-8"))

                if isinstance(message, dict):
                    latest_telemetry = message
                    telemetry_dirty = True

            except Exception as error:
                print("UART parse error:", error)
                print("Bad UART line:", repr(line[:300]))

    except Exception as error:
        print("UART read error:", error)

    # Send telemetry to Flask separately from frames.
    if telemetry_dirty and time.ticks_diff(now, last_telemetry_ms) >= TELEMETRY_INTERVAL_MS:
        try:
            status = post_json("/telemetry", latest_telemetry)
            if status == 200:
                telemetry_dirty = False
            else:
                print("Telemetry HTTP:", status)
        except Exception as error:
            print("Telemetry upload error:", error)
        last_telemetry_ms = now

    # Poll the newest browser command and relay it to RP2040.
    if time.ticks_diff(now, last_command_ms) >= COMMAND_INTERVAL_MS:
        response = None
        try:
            response = requests.get(
                SERVER_BASE + "/device/command",
                headers={"X-API-Key": DEVICE_API_KEY},
            )
            if response.status_code == 200:
                data = decode_response_json(response)
                command = data.get("command")
                sequence = data.get("seq", -1)
                if command and sequence != last_command_sequence:
                    uart.write(("CMD:" + str(command) + "\n").encode("utf-8"))
                    last_command_sequence = sequence
                    print("Command -> RP2040:", command)
            else:
                print("Command HTTP:", response.status_code)
        except Exception as error:
            print("Command poll error:", error)
        finally:
            close_response(response)
        last_command_ms = now

    # Upload current camera frame.
    if time.ticks_diff(now, last_frame_ms) >= FRAME_INTERVAL_MS:
        image = None
        jpeg = None
        image_bytes = None
        response = None
        try:
            image = sensor.snapshot()
            jpeg = image.compress(quality=JPEG_QUALITY)
            image_bytes = bytes(jpeg)
            response = requests.post(
                SERVER_BASE + "/frame",
                data=image_bytes,
                headers={
                    "Content-Type": "image/jpeg",
                    "X-API-Key": DEVICE_API_KEY,
                },
            )
            frame_number += 1
            print("Frame:", frame_number, "HTTP:", response.status_code)
        except Exception as error:
            print("Frame upload error:", error)
        finally:
            close_response(response)
            image = jpeg = image_bytes = response = None
            gc.collect()
        last_frame_ms = now

    time.sleep_ms(10)
