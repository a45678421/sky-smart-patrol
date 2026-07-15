from __future__ import annotations

import argparse
import base64
import json
import os
import time
from io import BytesIO
from pathlib import Path

import requests
import yaml
from openai import OpenAI
from PIL import Image


BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "config.yaml"


def load_config() -> dict:
    if not CONFIG_PATH.exists():
        raise FileNotFoundError(f"找不到設定檔：{CONFIG_PATH}")

    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError("config.yaml 格式錯誤")

    return config


def image_to_jpeg_bytes(image_path: Path) -> bytes:
    if not image_path.exists():
        raise FileNotFoundError(f"找不到圖片：{image_path}")

    image = Image.open(image_path).convert("RGB")
    image.thumbnail((960, 720))

    output = BytesIO()
    image.save(output, format="JPEG", quality=80)
    return output.getvalue()


def image_to_data_url(image_bytes: bytes) -> str:
    encoded = base64.b64encode(image_bytes).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}"


def road_hazard_prompt() -> str:
    return """
你是道路巡檢影像判讀助手。請檢查這張道路或地面影像，
判斷是否出現會危害道路使用者的情況。

請特別注意：
- pothole：坑洞、破洞、路面塌陷
- crack：道路裂縫、龜裂；一般裂縫與龜裂一律 severity=low
- standing_water：積水
- nails：散落鐵釘、螺絲、尖銳金屬物
- branches：枯枝、斷枝、樹枝落物
- fallen_tree：颱風或強風後倒樹、大片樹幹阻路
- rockfall：落石、碎石堆、土石掉落
- debris：其他妨礙通行的異物

嚴重度規則：crack 一律輸出 severity="low"；不要因一般裂縫、龜裂判成 medium 或 high。只有坑洞、路面塌陷、散落尖銳物、倒樹、落石、大片障礙物或明顯阻礙通行時，才可使用 medium 或 high。
畫面不清楚時必須標示 unclear，不可臆測。
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
""".strip()


def extract_json_object(text: str) -> dict:
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

    return json.loads(cleaned[first:last + 1])


def build_client(config: dict) -> tuple[OpenAI, str]:
    openai_config = config.get("openai") or {}
    if not isinstance(openai_config, dict):
        raise ValueError("config.yaml 的 openai 設定格式錯誤")

    api_key = str(openai_config.get("api_key") or "").strip()
    base_url = str(openai_config.get("base_url") or "").strip()
    model = str(openai_config.get("model") or "gpt-5-mini").strip()

    if not api_key:
        raise ValueError("config.yaml 缺少 openai.api_key")

    client_kwargs = {"api_key": api_key}
    if base_url:
        client_kwargs["base_url"] = base_url

    return OpenAI(**client_kwargs), model


def get_line_config(config: dict) -> tuple[str, str]:
    line_config = config.get("line") or {}
    if not isinstance(line_config, dict):
        raise ValueError("config.yaml 的 line 設定格式錯誤")

    token = str(
        os.getenv("LINE_CHANNEL_ACCESS_TOKEN")
        or line_config.get("channel_access_token")
        or line_config.get("channel_token")
        or config.get("line_channel_access_token")
        or config.get("channel_access_token")
        or ""
    ).strip()
    target_id = str(
        os.getenv("LINE_TARGET_ID")
        or os.getenv("LINE_USER_ID")
        or line_config.get("target_id")
        or line_config.get("user_id")
        or line_config.get("to")
        or config.get("line_target_id")
        or config.get("line_user_id")
        or config.get("user_id")
        or ""
    ).strip()

    if not token or not target_id:
        raise ValueError("config.yaml 缺少 LINE channel_access_token 或 user_id/target_id")

    return token, target_id


def get_cloudinary_config(config: dict) -> dict[str, str]:
    cloudinary_config = config.get("cloudinary") or {}
    if not isinstance(cloudinary_config, dict):
        cloudinary_config = {}

    return {
        "url": str(
            os.getenv("CLOUDINARY_URL")
            or cloudinary_config.get("url")
            or config.get("cloudinary_url")
            or ""
        ).strip(),
        "cloud_name": str(
            os.getenv("CLOUDINARY_CLOUD_NAME")
            or cloudinary_config.get("cloud_name")
            or ""
        ).strip(),
        "api_key": str(
            os.getenv("CLOUDINARY_API_KEY")
            or cloudinary_config.get("api_key")
            or ""
        ).strip(),
        "api_secret": str(
            os.getenv("CLOUDINARY_API_SECRET")
            or cloudinary_config.get("api_secret")
            or ""
        ).strip(),
        "folder": str(cloudinary_config.get("folder") or "sky_smart_patrol").strip(),
    }


def cloudinary_is_configured(config: dict) -> bool:
    cfg = get_cloudinary_config(config)
    return bool(cfg["url"] or (cfg["cloud_name"] and cfg["api_key"] and cfg["api_secret"]))


def upload_image_to_cloudinary(config: dict, image_bytes: bytes) -> str:
    cfg = get_cloudinary_config(config)
    if not cfg["url"] and not (cfg["cloud_name"] and cfg["api_key"] and cfg["api_secret"]):
        raise ValueError("Cloudinary 尚未設定，LINE 圖片訊息需要公開 HTTPS 圖片網址。")

    if cfg["url"]:
        os.environ["CLOUDINARY_URL"] = cfg["url"]

    try:
        import cloudinary
        import cloudinary.uploader
    except ImportError as error:
        raise RuntimeError("尚未安裝 cloudinary，請執行 pip install -r requirements.txt") from error

    if cfg["url"]:
        cloudinary.config(secure=True)
    else:
        cloudinary.config(
            cloud_name=cfg["cloud_name"],
            api_key=cfg["api_key"],
            api_secret=cfg["api_secret"],
            secure=True,
        )

    image_stream = BytesIO(image_bytes)
    image_stream.name = "ai_line_test.jpg"

    upload_result = cloudinary.uploader.upload(
        image_stream,
        resource_type="image",
        folder=cfg["folder"],
        public_id="line_test_{:d}".format(int(time.time() * 1000)),
        format="jpg",
        overwrite=False,
        unique_filename=True,
    )

    image_url = str(upload_result.get("secure_url") or "").strip()
    if not image_url.startswith("https://"):
        raise RuntimeError("Cloudinary 沒有回傳有效 HTTPS 圖片網址。")

    return image_url


def confidence_percent(value: object) -> float:
    try:
        confidence = float(value)
    except (TypeError, ValueError):
        return 0.0
    if 0 <= confidence <= 1:
        confidence *= 100
    return max(0.0, min(100.0, confidence))


def normalise_result(result: dict) -> dict:
    normalised = dict(result)
    anomaly = str(normalised.get("anomaly") or "").strip().lower()
    if anomaly == "crack":
        normalised["severity"] = "low"
    return normalised


def is_dangerous_result(result: dict) -> bool:
    anomaly = str(result.get("anomaly") or "").strip().lower()
    severity = str(result.get("severity") or "").strip().lower()
    alert = bool(result.get("alert"))

    if anomaly in {"", "normal", "unclear"}:
        return False
    if severity in {"", "none"}:
        return False
    return alert


def build_line_text(result: dict, image_sent: bool, image_error: str | None) -> str:
    alert = bool(result.get("alert"))
    status = "警示：發現疑似道路危害" if alert else "未發現明顯道路危害"
    evidence = result.get("evidence_zh") or []
    if isinstance(evidence, str):
        evidence = [evidence]
    evidence_text = "、".join(str(item) for item in evidence if str(item).strip()) or "無"

    text = (
        "SKY Smart Patrol AI 圖片測試\n"
        f"{status}\n"
        f"類型：{result.get('anomaly', '--')}\n"
        f"嚴重度：{result.get('severity', '--')}\n"
        f"把握度：{confidence_percent(result.get('confidence')):.0f}%\n"
        f"判讀：{result.get('summary_zh', '--')}\n"
        f"依據：{evidence_text}\n"
        f"圖片：{'已上傳' if image_sent else '未上傳'}"
    )
    if image_error:
        text += "\n圖片上傳失敗，已改送文字：" + image_error[:500]
    return text[:5000]


def push_line_message(token: str, target_id: str, messages: list[dict]) -> None:
    response = requests.post(
        "https://api.line.me/v2/bot/message/push",
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        },
        json={"to": target_id, "messages": messages},
        timeout=20,
    )
    if response.status_code != 200:
        raise RuntimeError(
            f"LINE Messaging API HTTP {response.status_code}: {response.text[:500]}"
        )


def send_result_to_line(config: dict, result: dict, image_bytes: bytes) -> str:
    token, target_id = get_line_config(config)
    image_url = None
    image_error = None

    try:
        if cloudinary_is_configured(config):
            image_url = upload_image_to_cloudinary(config, image_bytes)
        else:
            image_error = "Cloudinary 未設定，無法傳送 LINE 圖片訊息。"
    except Exception as error:
        image_error = str(error)

    messages = [
        {
            "type": "text",
            "text": build_line_text(result, bool(image_url), image_error),
        }
    ]
    if image_url:
        messages.append(
            {
                "type": "image",
                "originalContentUrl": image_url,
                "previewImageUrl": image_url,
            }
        )

    push_line_message(token, target_id, messages)
    return "image" if image_url else "text_only"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="上傳本機圖片到 OpenAI/Azure OpenAI 測試道路危害判斷，可選擇推送 LINE。"
    )
    parser.add_argument(
        "image",
        nargs="?",
        default="test_road.jpg",
        help="圖片路徑，預設為 test_road.jpg",
    )
    parser.add_argument(
        "--detail",
        choices=("low", "high", "auto"),
        default="high",
        help="影像判讀細節等級，預設 high",
    )
    parser.add_argument(
        "--send-line",
        action="store_true",
        help="AI 判斷為危險情況時，自動推送到 LINE；若 Cloudinary 已設定會附上圖片。",
    )
    parser.add_argument(
        "--force-line",
        action="store_true",
        help="不論 AI 是否判斷危險，都強制推送 LINE，適合測試 LINE 通道。",
    )
    args = parser.parse_args()

    config = load_config()
    client, model = build_client(config)
    image_path = Path(args.image)
    if not image_path.is_absolute():
        image_path = BASE_DIR / image_path

    print(f"Image : {image_path}")
    print(f"Model : {model}")
    print("Sending image to AI...")
    image_bytes = image_to_jpeg_bytes(image_path)

    response = client.responses.create(
        model=model,
        input=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": road_hazard_prompt(),
                    },
                    {
                        "type": "input_image",
                        "image_url": image_to_data_url(image_bytes),
                        "detail": args.detail,
                    },
                ],
            }
        ],
    )

    raw_text = getattr(response, "output_text", "")
    print("\nRaw result:")
    print(raw_text)

    try:
        result = normalise_result(extract_json_object(raw_text))
    except Exception as error:
        print("\n模型回覆不是有效 JSON：", error)
        return

    print("\nParsed result:")
    print("Scene      :", result.get("scene"))
    print("Anomaly    :", result.get("anomaly"))
    print("Alert      :", result.get("alert"))
    print("Severity   :", result.get("severity"))
    print("Confidence :", result.get("confidence"))
    print("Summary    :", result.get("summary_zh"))
    print("Evidence   :")
    evidence = result.get("evidence_zh") or []
    if isinstance(evidence, str):
        evidence = [evidence]
    for item in evidence:
        print("-", item)

    dangerous = is_dangerous_result(result)
    print("\nDangerous :", dangerous)

    should_send_line = args.force_line or (args.send_line and dangerous)
    if should_send_line:
        print("\nSending result to LINE...")
        mode = send_result_to_line(config, result, image_bytes)
        print(f"LINE sent: {mode}")
    elif args.send_line:
        print("\nLINE skipped: AI 判斷未達危險條件。")


if __name__ == "__main__":
    main()



