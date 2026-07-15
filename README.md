# SKY Smart Patrol

SKY Smart Patrol is a robot patrol system that combines a Flask dashboard, a Nicla Vision camera bridge, and an RP2040 robot controller. The computer dashboard receives live images and telemetry, supports manual and automatic patrol control, runs AI-based road hazard detection, and can send alerts to LINE.

## Project Layout

```text
sky-smart-patrol/
├─ computer/   # Flask dashboard, AI detection, LINE notification, web UI
├─ nicla/      # Nicla Vision Wi-Fi camera and UART bridge
├─ rp2040/     # Robot control, MCL localization, PID, patrol route logic
└─ docs/       # Setup notes and configuration reference
```

## What You Need

- Windows computer with Python 3.11 or newer.
- Git.
- Nicla Vision with OpenMV-style MicroPython support.
- RP2040/CircuitPython robot controller.
- Same Wi-Fi network for the computer and Nicla Vision.
- Optional: OpenAI/Azure OpenAI or Gemini API key for image detection.
- Optional: LINE Messaging API and Cloudinary for alert notifications with images.

## Start From Zero On One Computer

### 1. Install Python and Git

Install Python from https://www.python.org/downloads/ and Git from https://git-scm.com/downloads.

Open PowerShell and check:

```powershell
python --version
git --version
```

### 2. Clone This Repository

```powershell
cd D:\
git clone https://github.com/a45678421/sky-smart-patrol.git
cd sky-smart-patrol\computer
```

If you already downloaded the project manually, just open PowerShell in the `computer` folder.

### 3. Create Python Virtual Environment

```powershell
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Create Local Config

```powershell
copy config.example.yaml config.yaml
```

Edit `config.yaml`.

Minimum Azure OpenAI setup:

```yaml
detection:
  provider: "openai"

openai:
  api_key: "YOUR_OPENAI_OR_AZURE_OPENAI_API_KEY"
  base_url: "https://YOUR-RESOURCE.services.ai.azure.com/openai/v1"
  model: "YOUR_DEPLOYMENT_NAME"
```

For Azure OpenAI, `model` must be the deployment name.

If you want to use Gemini instead:

```yaml
detection:
  provider: "gemini"

gemini:
  api_key: "YOUR_GEMINI_API_KEY"
  model: "gemini-3.1-flash-lite"
```

Do not upload `config.yaml` to GitHub.

### 5. Run The Dashboard

```powershell
python road_detection_server.py
```

Open:

```text
http://127.0.0.1:5000
```

If another device needs to connect, find your computer IP:

```powershell
ipconfig
```

Use the IPv4 address, for example:

```text
http://192.168.0.110:5000
```

### 6. Test AI Image Detection

From the `computer` folder:

```powershell
python .\test_img\ai_image_upload_test.py .\test_img\road_nail.jpg
```

Send to LINE only when AI judges the image dangerous:

```powershell
python .\test_img\ai_image_upload_test.py .\test_img\road_nail.jpg --send-line
```

Force LINE sending for channel testing:

```powershell
python .\test_img\ai_image_upload_test.py .\test_img\road_nail.jpg --force-line
```

## Connect Nicla Vision

Edit `nicla/main.py` before uploading it to the Nicla Vision:

```python
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
SERVER_BASE = "http://YOUR_COMPUTER_IP:5000"
DEVICE_API_KEY = "sky_robot_2026_test"
```

`DEVICE_API_KEY` must match `DEVICE_API_KEY` in `computer/road_detection_server.py`.

Upload `nicla/main.py` to the Nicla Vision internal filesystem as `main.py`, then reboot the board. The Nicla will send JPEG frames to `/frame`, send telemetry to `/telemetry`, and poll `/device/command`.

## Connect RP2040

Copy these files in `rp2040/` to the RP2040/CircuitPython board:

```text
code.py
mcl_localization.py
pid_controller.py
pio_encoder.py
robot.py
```

`code_route_control.py` is an alternate route-control copy that can be kept for reference or copied as `code.py` when needed.

The RP2040 talks to Nicla Vision through UART. Make sure the UART TX/RX wiring and baud rate match the values in `robot.py` and `nicla/main.py`.

## Web Control Password

Protected dashboard actions require the web control token. The default in `computer/road_detection_server.py` is:

```text
sky_control_2026
```

Type this token into the dashboard before using protected controls. Change it in code before real deployment.

## Road Hazard AI Rules

The AI detector asks the model to classify:

- `normal`
- `pothole`
- `crack`
- `standing_water`
- `nails`
- `branches`
- `fallen_tree`
- `rockfall`
- `debris`
- `other`
- `unclear`

Road cracks and surface cracking are normalized as `severity: low`. More serious alerts are reserved for potholes, collapse, sharp metal objects, fallen trees, rockfall, large obstacles, or blocked traffic paths.

## Security Notes

- Never commit `config.yaml`.
- Rotate API keys if they were pasted into chat, screenshots, or public issue trackers.
- Do not expose the Flask server directly to the public Internet without authentication and a reverse proxy.
- LINE image messages require a public HTTPS image URL. Cloudinary is the recommended option for this project.

## Useful Git Commands

```powershell
git status
git add .
git commit -m "Update project documentation"
git push
```
