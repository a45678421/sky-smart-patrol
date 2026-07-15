"""SKY Smart Patrol web dashboard, robot bridge, image recognition and LINE alerts.

Install:
    pip install -r requirements.txt

Run:
    python road_detection_server.py

The browser, Windows computer, Nicla Vision and RP2040 bridge must use the
same LAN. Gemini recognition is only called when the user presses the
recognition button. LINE notification can be sent automatically for an alert
result or manually from the dashboard.
"""

from __future__ import annotations

import base64
from collections import deque
from datetime import datetime
from io import BytesIO
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from typing import Any

from flask import Flask, Response, jsonify, render_template, request
import requests


HOST = "0.0.0.0"
PORT = 5000

# Must match Nicla main.py.
DEVICE_API_KEY = "sky_robot_2026_test"

# Type this value into the dashboard before using protected functions.
WEB_CONTROL_TOKEN = "sky_control_2026"

ARENA_WIDTH_MM = 1500
ARENA_HEIGHT_MM = 1500
CUTOUT_WIDTH_MM = 500
CUTOUT_HEIGHT_MM = 500
EDGE_MARGIN_MM = 150
LANE_SPACING_MM = 250

BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = Path(
    os.getenv("SKY_CONFIG_PATH", str(BASE_DIR / "config.yaml"))
)


def _read_config() -> dict[str, Any]:
    if not CONFIG_PATH.exists():
        return {}

    try:
        import yaml
    except ImportError:
        print("WARNING: config.yaml exists, but PyYAML is not installed.")
        return {}

    try:
        data = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception as error:
        print("WARNING: cannot read config.yaml:", repr(error))
        return {}

    return data if isinstance(data, dict) else {}


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _first_value(*values: Any, default: str = "") -> str:
    for value in values:
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return default


CONFIG = _read_config()
GEMINI_CONFIG = _mapping(CONFIG.get("gemini"))
OPENAI_CONFIG = _mapping(CONFIG.get("openai"))
DETECTION_CONFIG = _mapping(CONFIG.get("detection"))
LINE_CONFIG = _mapping(CONFIG.get("line"))
CLOUDINARY_CONFIG = _mapping(CONFIG.get("cloudinary"))

GEMINI_API_KEY = _first_value(
    os.getenv("GEMINI_API_KEY"),
    GEMINI_CONFIG.get("api_key"),
    CONFIG.get("gemini_api_key"),
    CONFIG.get("GEMINI_API_KEY"),
)

# Current stable multimodal model. It can be changed in config.yaml without
# modifying the program.
GEMINI_MODEL = _first_value(
    os.getenv("GEMINI_MODEL"),
    GEMINI_CONFIG.get("model"),
    CONFIG.get("gemini_model"),
    default="gemini-3.5-flash",
)
OPENAI_API_KEY = _first_value(
    os.getenv("OPENAI_API_KEY"),
    OPENAI_CONFIG.get("api_key"),
    CONFIG.get("openai_api_key"),
    CONFIG.get("OPENAI_API_KEY"),
)

OPENAI_BASE_URL = _first_value(
    os.getenv("OPENAI_BASE_URL"),
    OPENAI_CONFIG.get("base_url"),
    CONFIG.get("openai_base_url"),
)

OPENAI_MODEL = _first_value(
    os.getenv("OPENAI_MODEL"),
    OPENAI_CONFIG.get("model"),
    CONFIG.get("openai_model"),
    default="gpt-5-mini",
)

DETECTION_PROVIDER = _first_value(
    os.getenv("SKY_DETECTION_PROVIDER"),
    DETECTION_CONFIG.get("provider"),
    CONFIG.get("detection_provider"),
    default="gemini",
).lower()
if DETECTION_PROVIDER not in {"gemini", "openai"}:
    DETECTION_PROVIDER = "gemini"

LINE_CHANNEL_ACCESS_TOKEN = _first_value(
    os.getenv("LINE_CHANNEL_ACCESS_TOKEN"),
    LINE_CONFIG.get("channel_access_token"),
    LINE_CONFIG.get("channel_token"),
    CONFIG.get("line_channel_access_token"),
    CONFIG.get("channel_access_token"),
    CONFIG.get("LINE_CHANNEL_ACCESS_TOKEN"),
)

LINE_TARGET_ID = _first_value(
    os.getenv("LINE_TARGET_ID"),
    os.getenv("LINE_USER_ID"),
    LINE_CONFIG.get("target_id"),
    LINE_CONFIG.get("user_id"),
    LINE_CONFIG.get("to"),
    CONFIG.get("line_target_id"),
    CONFIG.get("line_user_id"),
    CONFIG.get("user_id"),
    CONFIG.get("LINE_USER_ID"),
)


CLOUDINARY_URL = _first_value(
    os.getenv("CLOUDINARY_URL"),
    CLOUDINARY_CONFIG.get("url"),
    CONFIG.get("cloudinary_url"),
)

CLOUDINARY_CLOUD_NAME = _first_value(
    os.getenv("CLOUDINARY_CLOUD_NAME"),
    CLOUDINARY_CONFIG.get("cloud_name"),
)

CLOUDINARY_API_KEY = _first_value(
    os.getenv("CLOUDINARY_API_KEY"),
    CLOUDINARY_CONFIG.get("api_key"),
)

CLOUDINARY_API_SECRET = _first_value(
    os.getenv("CLOUDINARY_API_SECRET"),
    CLOUDINARY_CONFIG.get("api_secret"),
)

CLOUDINARY_FOLDER = _first_value(
    CLOUDINARY_CONFIG.get("folder"),
    default="sky_smart_patrol",
)


def cloudinary_is_configured() -> bool:
    return bool(
        CLOUDINARY_URL
        or (
            CLOUDINARY_CLOUD_NAME
            and CLOUDINARY_API_KEY
            and CLOUDINARY_API_SECRET
        )
    )

# Optional HTTPS URL exposed by Cloudflare Tunnel, ngrok or a reverse proxy.
# LINE can fetch an image only from a public HTTPS URL.
PUBLIC_BASE_URL = _first_value(
    os.getenv("PUBLIC_BASE_URL"),
    LINE_CONFIG.get("public_base_url"),
    CONFIG.get("public_base_url"),
).rstrip("/")

LINE_IMAGE_TOKEN = secrets.token_urlsafe(24)

app = Flask(__name__)
lock = threading.RLock()
detection_lock = threading.Lock()

latest_frame: bytes | None = None
latest_frame_time = 0.0
camera_frame_fps = 0.0

latest_telemetry = {
    "x": EDGE_MARGIN_MM,
    "y": EDGE_MARGIN_MM,
    "height": 0.0,
    "heading": 0.0,
    "mode": "OFFLINE",
    "status": "No telemetry",
    "distance": None,
    "plan_x": EDGE_MARGIN_MM,
    "plan_y": EDGE_MARGIN_MM,
    "speed": 40.0,
    "line_error": 0.0,
    "recovery": "IDLE",
    "raw_x": EDGE_MARGIN_MM,
    "raw_y": EDGE_MARGIN_MM,
    "raw_heading": 0.0,
    "mcl_position_std": 0.0,
    "mcl_heading_std": 0.0,
    "mcl_neff": 0.0,
    "mcl_measurement_used": False,
    "mcl_reason": "NO_TELEMETRY",
    "particles": [],
}
latest_telemetry_time = 0.0

pending_command: str | None = None
command_sequence = 0
last_device_poll_time = 0.0

path_history = deque(maxlen=3000)
path_sequence = 0
path_epoch = 1

latest_detection_frame: bytes | None = None
latest_line_frame: bytes | None = None
detection_state: dict[str, Any] = {
    "busy": False,
    "status": "IDLE",
    "result": None,
    "error": None,
    "started_at": None,
    "finished_at": None,
    "line_sent": False,
    "line_error": None,
    "model": GEMINI_MODEL,
}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def x_limit_for_y(y_mm: float) -> float:
    if y_mm < CUTOUT_HEIGHT_MM:
        return ARENA_WIDTH_MM - CUTOUT_WIDTH_MM - EDGE_MARGIN_MM
    return ARENA_WIDTH_MM - EDGE_MARGIN_MM


def build_patrol_waypoints() -> list[list[float]]:
    y_values = []
    y = EDGE_MARGIN_MM
    while y <= ARENA_HEIGHT_MM - EDGE_MARGIN_MM:
        y_values.append(y)
        y += LANE_SPACING_MM

    points = []
    for lane_index, lane_y in enumerate(y_values):
        x_min = EDGE_MARGIN_MM
        x_max = x_limit_for_y(lane_y)
        if lane_index % 2 == 0:
            lane_start = [x_min, lane_y]
            lane_end = [x_max, lane_y]
        else:
            lane_start = [x_max, lane_y]
            lane_end = [x_min, lane_y]

        if not points:
            points.append(lane_start)
        else:
            current_x, current_y = points[-1]
            if current_y != lane_y:
                points.append([current_x, lane_y])
            if current_x != lane_start[0]:
                points.append(lane_start)
        points.append(lane_end)
    return points


PATROL_ROUTE = build_patrol_waypoints()


def require_device_key() -> bool:
    return request.headers.get("X-API-Key", "") == DEVICE_API_KEY


def require_control_token() -> bool:
    return request.headers.get("X-Control-Token", "") == WEB_CONTROL_TOKEN


def safe_float(value: Any, default: float = 0.0) -> float:
    """Convert a value to a finite float, otherwise return the default."""
    if value is None:
        return float(default)

    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(number):
        return float(default)

    return number


def safe_optional_float(value: Any) -> float | None:
    """Convert a value to a finite float, or None when unavailable."""
    if value is None:
        return None

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(number):
        return None

    return number


def normalise_boolean(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {
            "true",
            "1",
            "yes",
            "on",
        }

    return bool(value)


def normalise_particles(value: Any) -> list[list[float]]:
    """Validate and compact an MCL particle list for the web dashboard."""
    particles: list[list[float]] = []

    if not isinstance(value, list):
        return particles

    for item in value[:48]:
        if not isinstance(item, (list, tuple)) or len(item) < 3:
            continue

        x = safe_optional_float(item[0])
        y = safe_optional_float(item[1])
        heading = safe_optional_float(item[2])

        if x is None or y is None or heading is None:
            continue

        if not (0.0 <= x <= ARENA_WIDTH_MM):
            continue
        if not (0.0 <= y <= ARENA_HEIGHT_MM):
            continue

        particles.append(
            [
                round(x, 1),
                round(y, 1),
                round(heading % 360.0, 1),
            ]
        )

    return particles


def normalise_telemetry(data: dict[str, Any]) -> dict[str, Any]:
    """Convert compact RP2040 telemetry into dashboard field names."""
    x = safe_float(data.get("x"), EDGE_MARGIN_MM)
    y = safe_float(data.get("y"), EDGE_MARGIN_MM)
    heading = safe_float(
        data.get("h", data.get("heading")),
        0.0,
    ) % 360.0

    distance = safe_optional_float(
        data.get("d", data.get("distance"))
    )

    particles = normalise_particles(
        data.get("particles", data.get("pt", []))
    )

    return {
        # MCL-corrected pose.
        "x": x,
        "y": y,
        "height": safe_float(
            data.get("z", data.get("height")),
            0.0,
        ),
        "heading": heading,

        # General state.
        "mode": str(data.get("m", data.get("mode", "UNKNOWN"))),
        "status": str(data.get("s", data.get("status", ""))),
        "distance": distance,

        # Planner pose.
        "plan_x": safe_optional_float(
            data.get("px", data.get("plan_x"))
        ),
        "plan_y": safe_optional_float(
            data.get("py", data.get("plan_y"))
        ),

        # Motion and route recovery.
        "speed": safe_float(
            data.get("v", data.get("speed")),
            40.0,
        ),
        "line_error": safe_float(
            data.get("e", data.get("line_error")),
            0.0,
        ),
        "recovery": str(
            data.get("r", data.get("recovery", "IDLE"))
        ),

        # Raw encoder-only odometry.
        "raw_x": safe_float(
            data.get("ox", data.get("raw_x")),
            x,
        ),
        "raw_y": safe_float(
            data.get("oy", data.get("raw_y")),
            y,
        ),
        "raw_heading": safe_float(
            data.get("oh", data.get("raw_heading")),
            heading,
        ) % 360.0,

        # MCL diagnostics.
        "mcl_position_std": safe_float(
            data.get("mp", data.get("mcl_position_std")),
            0.0,
        ),
        "mcl_heading_std": safe_float(
            data.get("mh", data.get("mcl_heading_std")),
            0.0,
        ),
        "mcl_neff": safe_float(
            data.get("mn", data.get("mcl_neff")),
            0.0,
        ),
        "mcl_measurement_used": normalise_boolean(
            data.get(
                "mu",
                data.get("mcl_measurement_used", False),
            )
        ),
        "mcl_reason": str(
            data.get("mr", data.get("mcl_reason", ""))
        ),

        # Particle cloud for Canvas visualization.
        "particles": particles,
    }


def line_image_mode() -> str:
    if cloudinary_is_configured():
        return "cloudinary"
    if PUBLIC_BASE_URL.lower().startswith("https://"):
        return "public_https"
    return "disabled"


def integration_status() -> dict[str, Any]:
    image_mode = line_image_mode()
    return {
        "gemini_configured": bool(GEMINI_API_KEY),
        "gemini_model": GEMINI_MODEL,
        "openai_configured": bool(OPENAI_API_KEY),
        "openai_model": OPENAI_MODEL,
        "detection_provider": DETECTION_PROVIDER,
        "detection_model": detection_model_label(),
        "line_configured": bool(
            LINE_CHANNEL_ACCESS_TOKEN and LINE_TARGET_ID
        ),
        "line_image_enabled": image_mode != "disabled",
        "line_image_mode": image_mode,
        "cloudinary_configured": cloudinary_is_configured(),
    }


def _extract_json_object(text: str) -> dict[str, Any]:
    cleaned = text.strip()

    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        if lines:
            lines = lines[1:]
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        cleaned = "\n".join(lines).strip()

    first = cleaned.find("{")
    last = cleaned.rfind("}")
    if first < 0 or last <= first:
        raise ValueError("模型回覆未包含 JSON object")

    parsed = json.loads(cleaned[first:last + 1])
    if not isinstance(parsed, dict):
        raise ValueError("模型 JSON 結果必須是物件")
    return parsed


def _normalise_detection_result(data: dict[str, Any]) -> dict[str, Any]:
    allowed_scene = {"road", "not_road", "unclear"}
    allowed_anomaly = {
        "normal",
        "pothole",
        "crack",
        "standing_water",
        "debris",
        "nails",
        "branches",
        "fallen_tree",
        "rockfall",
        "other",
        "unclear",
    }
    allowed_severity = {"none", "low", "medium", "high"}

    scene = str(data.get("scene", "unclear")).lower().strip()
    anomaly = str(data.get("anomaly", "unclear")).lower().strip()
    severity = str(data.get("severity", "none")).lower().strip()

    if scene not in allowed_scene:
        scene = "unclear"
    if anomaly not in allowed_anomaly:
        anomaly = "other"
    if severity not in allowed_severity:
        severity = "none"
    if anomaly == "crack":
        severity = "low"

    try:
        confidence = float(data.get("confidence", 0))
    except (TypeError, ValueError):
        confidence = 0.0
    confidence = max(0.0, min(100.0, confidence))

    alert_value = data.get("alert", False)
    if isinstance(alert_value, str):
        alert = alert_value.strip().lower() in {"true", "1", "yes", "是"}
    else:
        alert = bool(alert_value)

    evidence = data.get("evidence_zh", [])
    if isinstance(evidence, str):
        evidence = [evidence]
    if not isinstance(evidence, list):
        evidence = []
    evidence = [str(item).strip() for item in evidence if str(item).strip()][:5]

    return {
        "scene": scene,
        "anomaly": anomaly,
        "alert": alert,
        "severity": severity,
        "confidence": round(confidence, 1),
        "summary_zh": str(data.get("summary_zh", "")).strip()
        or "模型未提供摘要。",
        "evidence_zh": evidence,
    }



def road_hazard_detection_prompt() -> str:
    return """
你是道路巡檢影像判讀助手。請檢查這張由低解析度車載相機拍攝的目前畫面，
判斷是否為道路或地面，以及是否出現會危害道路使用者的情況。

請特別注意以下道路危害：
- pothole：坑洞、破洞、路面塌陷
- crack：道路裂縫、龜裂；一般裂縫與龜裂一律 severity=low
- standing_water：積水
- nails：散落鐵釘、螺絲、尖銳金屬物
- branches：枯枝、斷枝、樹枝落物
- fallen_tree：颱風或強風後倒樹、大片樹幹阻路
- rockfall：落石、碎石堆、土石掉落
- debris：其他妨礙通行的異物

嚴重度規則：crack 一律輸出 severity="low"；不要因一般裂縫、龜裂判成 medium 或 high。只有坑洞、路面塌陷、散落尖銳物、倒樹、落石、大片障礙物或明顯阻礙通行時，才可使用 medium 或 high。
畫面不清楚時必須標示 unclear，不可臆測。若不是道路或地面，scene 設為 not_road。
只輸出一個 JSON 物件，不要 Markdown，不要額外說明：
{
  "scene": "road|not_road|unclear",
  "anomaly": "normal|pothole|crack|standing_water|nails|branches|fallen_tree|rockfall|debris|other|unclear",
  "alert": true,
  "severity": "none|low|medium|high",
  "confidence": 0,
  "summary_zh": "繁體中文簡短判讀",
  "evidence_zh": ["繁體中文影像依據"]
}

alert 只有在疑似坑洞、具風險裂縫、積水、鐵釘尖銳物、枯枝、倒樹、落石、異物，
或其他可能影響行人、騎士、車輛通行安全的情況下才設為 true。
confidence 為 0 到 100，代表影像判讀把握度，不是統計保證。
""".strip()


def image_data_url(frame_bytes: bytes) -> str:
    encoded = base64.b64encode(frame_bytes).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}"


def detection_model_label() -> str:
    if DETECTION_PROVIDER == "openai":
        return OPENAI_MODEL
    return GEMINI_MODEL


def detection_is_configured() -> bool:
    if DETECTION_PROVIDER == "openai":
        return bool(OPENAI_API_KEY)
    return bool(GEMINI_API_KEY)


def run_openai_detection(frame_bytes: bytes) -> dict[str, Any]:
    if not OPENAI_API_KEY:
        raise RuntimeError(
            "尚未設定 OpenAI API Key。請在 config.yaml 填入 openai.api_key 或設定 OPENAI_API_KEY。"
        )

    try:
        from openai import OpenAI
    except ImportError as error:
        raise RuntimeError(
            "尚未安裝 openai，請執行 pip install -r requirements.txt。"
        ) from error

    client_kwargs = {"api_key": OPENAI_API_KEY}
    if OPENAI_BASE_URL:
        client_kwargs["base_url"] = OPENAI_BASE_URL

    client = OpenAI(**client_kwargs)
    response = client.responses.create(
        model=OPENAI_MODEL,
        input=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": road_hazard_detection_prompt(),
                    },
                    {
                        "type": "input_image",
                        "image_url": image_data_url(frame_bytes),
                        "detail": "low",
                    },
                ],
            }
        ],
    )

    response_text = getattr(response, "output_text", "")
    if not response_text:
        raise RuntimeError("OpenAI 沒有回傳文字結果。")

    parsed = _extract_json_object(response_text)
    return _normalise_detection_result(parsed)


def run_detection(frame_bytes: bytes) -> dict[str, Any]:
    if DETECTION_PROVIDER == "openai":
        return run_openai_detection(frame_bytes)
    return run_gemini_detection(frame_bytes)
def run_gemini_detection(frame_bytes: bytes) -> dict[str, Any]:
    if not GEMINI_API_KEY:
        raise RuntimeError(
            "尚未設定 Gemini API Key。請在 config.yaml 填入 gemini.api_key。"
        )

    try:
        from google import genai
    except ImportError as error:
        raise RuntimeError(
            "尚未安裝 google-genai，請執行 pip install -r requirements.txt。"
        ) from error

    prompt = road_hazard_detection_prompt()

    client = genai.Client(api_key=GEMINI_API_KEY)
    encoded = base64.b64encode(frame_bytes).decode("ascii")

    # Prefer the current Interactions API. Fall back to generateContent for
    # installed SDK versions that do not yet expose client.interactions.
    try:
        interaction = client.interactions.create(
            model=GEMINI_MODEL,
            input=[
                {"type": "text", "text": prompt},
                {
                    "type": "image",
                    "data": encoded,
                    "mime_type": "image/jpeg",
                },
            ],
        )
        response_text = interaction.output_text
    except AttributeError:
        from google.genai import types

        response = client.models.generate_content(
            model=GEMINI_MODEL,
            contents=[
                prompt,
                types.Part.from_bytes(
                    data=frame_bytes,
                    mime_type="image/jpeg",
                ),
            ],
        )
        response_text = response.text

    if not response_text:
        raise RuntimeError("Gemini 沒有回傳文字結果。")

    parsed = _extract_json_object(response_text)
    return _normalise_detection_result(parsed)


def _line_push(messages: list[dict[str, Any]]) -> None:
    if not LINE_CHANNEL_ACCESS_TOKEN or not LINE_TARGET_ID:
        raise RuntimeError(
            "尚未設定 LINE channel access token 或 target ID。"
        )

    response = requests.post(
        "https://api.line.me/v2/bot/message/push",
        headers={
            "Authorization": f"Bearer {LINE_CHANNEL_ACCESS_TOKEN}",
            "Content-Type": "application/json",
        },
        json={
            "to": LINE_TARGET_ID,
            "messages": messages,
        },
        timeout=20,
    )

    if response.status_code != 200:
        detail = response.text[:500]
        raise RuntimeError(
            f"LINE Messaging API HTTP {response.status_code}: {detail}"
        )


def _line_detection_text(
    result: dict[str, Any],
    telemetry: dict[str, Any],
) -> str:
    status = "⚠️ 發現疑似道路異常" if result["alert"] else "✅ 未發現明顯道路異常"
    evidence = "、".join(result.get("evidence_zh") or []) or "無"

    distance = telemetry.get("distance")
    distance_text = "--" if distance is None else f"{float(distance):.0f} mm"

    return (
        "SKY Smart Patrol\n"
        f"{status}\n"
        f"類型：{result['anomaly']}\n"
        f"嚴重度：{result['severity']}\n"
        f"把握度：{result['confidence']:.0f}%\n"
        f"判讀：{result['summary_zh']}\n"
        f"依據：{evidence}\n"
        f"車輛座標：({float(telemetry.get('x', 0)):.0f}, "
        f"{float(telemetry.get('y', 0)):.0f}) mm\n"
        f"前方距離：{distance_text}\n"
        f"時間：{now_iso()}"
    )[:5000]


def _upload_line_image_to_cloudinary(
    frame_bytes: bytes,
) -> str:
    if not cloudinary_is_configured():
        raise RuntimeError("Cloudinary 尚未設定。")

    if CLOUDINARY_URL:
        # The SDK reads CLOUDINARY_URL during import/configuration.
        os.environ["CLOUDINARY_URL"] = CLOUDINARY_URL

    try:
        import cloudinary
        import cloudinary.uploader
    except ImportError as error:
        raise RuntimeError(
            "尚未安裝 cloudinary，請重新執行 "
            "pip install -r requirements.txt。"
        ) from error

    if CLOUDINARY_URL:
        cloudinary.config(secure=True)
    else:
        cloudinary.config(
            cloud_name=CLOUDINARY_CLOUD_NAME,
            api_key=CLOUDINARY_API_KEY,
            api_secret=CLOUDINARY_API_SECRET,
            secure=True,
        )

    # Cloudinary needs an in-memory stream to expose a filename.
    # A bare BytesIO object can fail because it has no file name or extension.
    image_stream = BytesIO(frame_bytes)
    image_stream.name = "nicla_frame.jpg"

    try:
        upload_result = cloudinary.uploader.upload(
            image_stream,
            resource_type="image",
            folder=CLOUDINARY_FOLDER,
            public_id="line_{:d}".format(int(time.time() * 1000)),
            format="jpg",
            overwrite=False,
            unique_filename=True,
        )
    except Exception as error:
        raise RuntimeError(
            "Cloudinary 圖片上傳失敗：{}".format(error)
        ) from error

    secure_url = str(upload_result.get("secure_url", "")).strip()
    if not secure_url.startswith("https://"):
        raise RuntimeError("Cloudinary 沒有回傳有效的 HTTPS 圖片網址。")

    return secure_url


def _prepare_line_image_url(
    frame_bytes: bytes,
) -> tuple[str | None, str]:
    """Return an HTTPS image URL and delivery mode.

    Cloudinary is preferred because it works without exposing the local Flask
    server. A public HTTPS tunnel is used as the fallback.
    """
    global latest_line_frame

    if cloudinary_is_configured():
        return _upload_line_image_to_cloudinary(frame_bytes), "cloudinary"

    if PUBLIC_BASE_URL.lower().startswith("https://"):
        with lock:
            latest_line_frame = bytes(frame_bytes)

        return (
            f"{PUBLIC_BASE_URL}/line/image/{LINE_IMAGE_TOKEN}.jpg",
            "public_https",
        )

    return None, "disabled"


def _line_image_message(image_url: str) -> dict[str, Any]:
    return {
        "type": "image",
        "originalContentUrl": image_url,
        "previewImageUrl": image_url,
    }


def send_detection_to_line(
    result: dict[str, Any],
    telemetry: dict[str, Any],
    frame_bytes: bytes | None = None,
) -> tuple[bool, str]:
    messages: list[dict[str, Any]] = [
        {
            "type": "text",
            "text": _line_detection_text(result, telemetry),
        }
    ]

    image_sent = False
    image_mode = "disabled"
    selected_frame = frame_bytes or latest_detection_frame

    image_error = None

    if selected_frame:
        try:
            image_url, image_mode = _prepare_line_image_url(selected_frame)
            if image_url:
                messages.append(_line_image_message(image_url))
                image_sent = True
        except Exception as error:
            # Do not lose the recognition text merely because image hosting failed.
            image_error = str(error)
            messages[0]["text"] += (
                "\n\n圖片上傳失敗，已改為只傳送文字："
                + image_error[:500]
            )

    _line_push(messages)

    if image_error:
        image_mode = "cloudinary_error"

    return image_sent, image_mode


def send_current_frame_to_line(
    frame_bytes: bytes,
    telemetry: dict[str, Any],
) -> str:
    distance = telemetry.get("distance")
    distance_text = (
        "--"
        if distance is None
        else f"{float(distance):.0f} mm"
    )

    caption = (
        "SKY Smart Patrol 即時影像\n"
        f"車輛座標：({float(telemetry.get('x', 0)):.0f}, "
        f"{float(telemetry.get('y', 0)):.0f}) mm\n"
        f"方向：{float(telemetry.get('heading', 0)):.0f}°\n"
        f"前方距離：{distance_text}\n"
        f"時間：{now_iso()}"
    )

    try:
        image_url, image_mode = _prepare_line_image_url(frame_bytes)

        if not image_url:
            raise RuntimeError(
                "尚未設定 Cloudinary 或公開 HTTPS 圖片網址。"
            )
    except Exception as error:
        # LINE itself is working, so still deliver a diagnostic text message.
        _line_push(
            [
                {
                    "type": "text",
                    "text": (
                        caption
                        + "\n\n圖片上傳失敗："
                        + str(error)[:700]
                    ),
                }
            ]
        )
        raise RuntimeError(
            "文字已送到 LINE，但圖片上傳失敗：{}".format(error)
        ) from error

    _line_push(
        [
            {"type": "text", "text": caption},
            _line_image_message(image_url),
        ]
    )

    return image_mode


def send_line_test_message() -> None:
    _line_push(
        [
            {
                "type": "text",
                "text": (
                    "SKY Smart Patrol LINE 測試成功\n"
                    f"時間：{now_iso()}"
                ),
            }
        ]
    )


def detection_worker(
    frame_bytes: bytes,
    telemetry_snapshot: dict[str, Any],
    notify_line: bool,
) -> None:
    global latest_detection_frame

    line_sent = False
    line_error = None

    try:
        result = run_detection(frame_bytes)

        with lock:
            latest_detection_frame = frame_bytes

        if notify_line and result["alert"]:
            try:
                image_sent, image_mode = send_detection_to_line(
                    result,
                    telemetry_snapshot,
                    frame_bytes,
                )
                line_sent = True
                if not image_sent:
                    line_error = (
                        "LINE 已送出文字，但圖片傳送尚未設定。"
                    )
            except Exception as error:
                line_error = str(error)

        with lock:
            detection_state.update(
                {
                    "busy": False,
                    "status": "DONE",
                    "result": result,
                    "error": None,
                    "finished_at": now_iso(),
                    "line_sent": line_sent,
                    "line_error": line_error,
                    "model": detection_model_label(),
                }
            )
    except Exception as error:
        with lock:
            detection_state.update(
                {
                    "busy": False,
                    "status": "ERROR",
                    "result": None,
                    "error": str(error),
                    "finished_at": now_iso(),
                    "line_sent": False,
                    "line_error": None,
                    "model": detection_model_label(),
                }
            )
    finally:
        if detection_lock.locked():
            detection_lock.release()


@app.get("/")
def dashboard():
    return render_template("index.html")


@app.post("/frame")
def receive_frame():
    global latest_frame, latest_frame_time, camera_frame_fps

    if not require_device_key():
        return jsonify(ok=False, error="invalid device key"), 403

    body = request.get_data(cache=False)
    if not body or len(body) > 2_000_000:
        return jsonify(ok=False, error="invalid JPEG payload"), 400

    now = time.monotonic()

    with lock:
        latest_frame = bytes(body)
        if latest_frame_time > 0:
            interval_s = max(now - latest_frame_time, 0.001)
            instant_fps = 1.0 / interval_s
            camera_frame_fps = (
                instant_fps
                if camera_frame_fps <= 0
                else camera_frame_fps * 0.75 + instant_fps * 0.25
            )
        latest_frame_time = now

    return jsonify(ok=True, size=len(body))


@app.get("/camera.jpg")
def camera_jpeg():
    with lock:
        frame = latest_frame

    if frame is None:
        return Response(status=503)

    return Response(
        frame,
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )



@app.get("/camera.mjpeg")
def camera_mjpeg():
    def generate():
        last_frame_time = 0.0

        while True:
            with lock:
                frame = latest_frame
                frame_time = latest_frame_time

            if frame is None or frame_time == last_frame_time:
                time.sleep(0.03)
                continue

            last_frame_time = frame_time
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-store\r\n\r\n"
                + frame
                + b"\r\n"
            )

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )

@app.get("/line/image/<token>.jpg")
def line_detection_image(token: str):
    if not secrets.compare_digest(token, LINE_IMAGE_TOKEN):
        return Response(status=404)

    with lock:
        frame = latest_line_frame

    if frame is None:
        return Response(status=404)

    return Response(
        frame,
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


@app.post("/telemetry")
def receive_telemetry():
    global latest_telemetry
    global latest_telemetry_time
    global path_sequence

    if not require_device_key():
        return jsonify(ok=False, error="invalid device key"), 403

    raw_body = request.get_data(cache=True, as_text=True)
    data = request.get_json(silent=True)

    if not isinstance(data, dict):
        print("TELEMETRY JSON ERROR")
        print("Content-Type:", request.content_type)
        print("Body length:", len(raw_body))
        print("Raw body:", repr(raw_body[:1000]))

        return jsonify(
            ok=False,
            error="JSON object required",
            body_length=len(raw_body),
        ), 400

    try:
        telemetry = normalise_telemetry(data)
    except Exception as error:
        print("TELEMETRY NORMALISE ERROR:", repr(error))
        print("Telemetry data:", repr(data))

        return jsonify(
            ok=False,
            error="invalid telemetry",
            detail=str(error),
        ), 400

    now = time.monotonic()

    with lock:
        latest_telemetry = telemetry
        latest_telemetry_time = now

        x = telemetry["x"]
        y = telemetry["y"]

        append_point = True
        if path_history:
            _, last_x, last_y = path_history[-1]
            append_point = math.hypot(
                x - last_x,
                y - last_y,
            ) >= 8.0

        if append_point:
            path_sequence += 1
            path_history.append((path_sequence, x, y))

    return jsonify(
        ok=True,
        particle_count=len(telemetry.get("particles", [])),
    )


@app.get("/device/command")
def device_command():
    global pending_command, last_device_poll_time

    if not require_device_key():
        return jsonify(ok=False, error="invalid device key"), 403

    with lock:
        last_device_poll_time = time.monotonic()
        command = pending_command
        pending_command = None
        sequence = command_sequence

    return jsonify(ok=True, command=command, seq=sequence)


@app.post("/api/command")
def browser_command():
    global pending_command, command_sequence

    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    data = request.get_json(silent=True) or {}
    command = str(data.get("command", "")).upper().strip()
    allowed = {
        "FORWARD",
        "BACKWARD",
        "LEFT",
        "RIGHT",
        "STOP",
        "AUTO",
        "RESET_POSE",
        "MCL_RESET",
    }

    if command not in allowed:
        return jsonify(ok=False, error="unsupported command"), 400

    with lock:
        command_sequence += 1
        pending_command = command
        sequence = command_sequence

    return jsonify(ok=True, command=command, seq=sequence)


@app.post("/api/speed")
def set_speed():
    global pending_command, command_sequence

    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    data = request.get_json(silent=True) or {}

    try:
        percent = float(data.get("percent"))
    except (TypeError, ValueError):
        return jsonify(ok=False, error="numeric percent required"), 400

    percent = max(20.0, min(100.0, percent))
    pwm_ratio = percent / 100.0

    with lock:
        command_sequence += 1
        pending_command = f"SPEED:{pwm_ratio:.2f}"
        sequence = command_sequence

    return jsonify(
        ok=True,
        percent=percent,
        pwm_ratio=pwm_ratio,
        seq=sequence,
    )


@app.post("/api/detect")
def start_detection():
    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    if not detection_is_configured():
        provider_name = "OpenAI" if DETECTION_PROVIDER == "openai" else "Gemini"
        return jsonify(
            ok=False,
            error=f"{provider_name} 尚未設定，請檢查 config.yaml。",
        ), 503

    if not detection_lock.acquire(blocking=False):
        return jsonify(ok=False, error="辨識正在執行中"), 409

    data = request.get_json(silent=True) or {}
    notify_line = bool(data.get("notify_line", True))

    with lock:
        frame = latest_frame
        telemetry_snapshot = dict(latest_telemetry)

    if frame is None:
        detection_lock.release()
        return jsonify(ok=False, error="尚未收到相機影像"), 409

    with lock:
        detection_state.update(
            {
                "busy": True,
                "status": "RUNNING",
                "result": None,
                "error": None,
                "started_at": now_iso(),
                "finished_at": None,
                "line_sent": False,
                "line_error": None,
                "model": detection_model_label(),
            }
        )

    worker = threading.Thread(
        target=detection_worker,
        args=(bytes(frame), telemetry_snapshot, notify_line),
        daemon=True,
    )
    worker.start()

    return jsonify(ok=True, status="RUNNING"), 202


@app.post("/api/line/test")
def line_test():
    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    try:
        send_line_test_message()
    except Exception as error:
        return jsonify(ok=False, error=str(error)), 502

    return jsonify(ok=True)


@app.post("/api/line/send_last")
def line_send_last():
    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    with lock:
        result = detection_state.get("result")
        telemetry_snapshot = dict(latest_telemetry)
        frame = latest_detection_frame

    if not isinstance(result, dict):
        return jsonify(ok=False, error="尚無可傳送的辨識結果"), 409

    try:
        image_sent, image_mode = send_detection_to_line(
            result,
            telemetry_snapshot,
            frame,
        )
    except Exception as error:
        return jsonify(ok=False, error=str(error)), 502

    with lock:
        detection_state["line_sent"] = True
        detection_state["line_error"] = (
            None
            if image_sent
            else "文字已送出，但圖片傳送尚未設定。"
        )

    return jsonify(
        ok=True,
        image_sent=image_sent,
        image_mode=image_mode,
    )


@app.post("/api/line/send_current_image")
def line_send_current_image():
    """Send the newest Nicla camera frame to LINE without Gemini."""
    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    with lock:
        frame = latest_frame
        telemetry_snapshot = dict(latest_telemetry)

    if frame is None:
        return jsonify(ok=False, error="尚未收到相機影像"), 409

    try:
        image_mode = send_current_frame_to_line(
            bytes(frame),
            telemetry_snapshot,
        )
    except Exception as error:
        return jsonify(ok=False, error=str(error)), 502

    return jsonify(
        ok=True,
        image_sent=True,
        image_mode=image_mode,
    )


@app.post("/notify")
def legacy_notify():
    """Compatibility endpoint for existing device-side LINE notifications."""
    if not require_device_key():
        return jsonify(ok=False, error="invalid device key"), 403

    data = request.get_json(silent=True) or {}
    message = str(data.get("message", "")).strip()

    if not message:
        return jsonify(ok=False, error="message required"), 400

    try:
        _line_push([{"type": "text", "text": message[:5000]}])
    except Exception as error:
        return jsonify(ok=False, error=str(error)), 502

    return jsonify(ok=True)


@app.post("/api/clear_path")
def clear_path():
    global path_sequence, path_epoch

    if not require_control_token():
        return jsonify(ok=False, error="invalid control token"), 403

    with lock:
        path_history.clear()
        path_sequence = 0
        path_epoch += 1

    return jsonify(ok=True, epoch=path_epoch)


@app.get("/api/state")
def api_state():
    try:
        after = int(request.args.get("after", "0"))
    except ValueError:
        after = 0

    now = time.monotonic()
    with lock:
        telemetry = dict(latest_telemetry)
        points = [
            list(point)
            for point in path_history
            if point[0] > after
        ]
        detection = json.loads(json.dumps(detection_state))

        state = {
            "telemetry": telemetry,
            "telemetry_age_s": (
                None
                if latest_telemetry_time == 0
                else now - latest_telemetry_time
            ),
            "camera_age_s": (
                None
                if latest_frame_time == 0
                else now - latest_frame_time
            ),
            "camera_fps": round(camera_frame_fps, 2),
            "device_poll_age_s": (
                None
                if last_device_poll_time == 0
                else now - last_device_poll_time
            ),
            "path": points[:200],
            "path_epoch": path_epoch,
            "route": PATROL_ROUTE,
            "arena": {
                "width": ARENA_WIDTH_MM,
                "height": ARENA_HEIGHT_MM,
                "cutout_width": CUTOUT_WIDTH_MM,
                "cutout_height": CUTOUT_HEIGHT_MM,
                "edge_margin": EDGE_MARGIN_MM,
            },
            "detection": detection,
            "integrations": integration_status(),
        }

    return jsonify(state)


if __name__ == "__main__":
    status = integration_status()

    print("SKY Smart Patrol Server")
    print(f"Dashboard: http://127.0.0.1:{PORT}")
    print("Detection provider:", status["detection_provider"], f"({status['detection_model']})")
    print(
        "OpenAI:",
        "configured" if status["openai_configured"] else "NOT configured",
        f"({OPENAI_MODEL})",
    )
    print(
        "Gemini:",
        "configured" if status["gemini_configured"] else "NOT configured",
        f"({GEMINI_MODEL})",
    )
    print(
        "LINE:",
        "configured" if status["line_configured"] else "NOT configured",
    )
    if status["line_configured"]:
        print("LINE image mode:", status["line_image_mode"])
        if status["line_image_mode"] == "cloudinary":
            print("Cloudinary cloud name:", CLOUDINARY_CLOUD_NAME or "(from CLOUDINARY_URL)")
            print("Cloudinary folder:", CLOUDINARY_FOLDER)
            print(
                "Cloudinary API key:",
                (
                    CLOUDINARY_API_KEY[:4] + "..." + CLOUDINARY_API_KEY[-2:]
                    if len(CLOUDINARY_API_KEY) >= 7
                    else "(configured)"
                ),
            )
        if not status["line_image_enabled"]:
            print(
                "LINE image: disabled; configure Cloudinary or an "
                "HTTPS public_base_url. Text notifications still work."
            )

    app.run(
        host=HOST,
        port=PORT,
        debug=False,
        threaded=True,
    )




