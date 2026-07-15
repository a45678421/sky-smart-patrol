# Configuration Reference

Create a local config file from the example:

```powershell
cd computer
copy config.example.yaml config.yaml
```

`config.yaml` contains secrets and must not be committed to GitHub.

## Detection Provider

Use OpenAI/Azure OpenAI:

```yaml
detection:
  provider: "openai"

openai:
  api_key: "YOUR_OPENAI_OR_AZURE_OPENAI_API_KEY"
  base_url: "https://YOUR-RESOURCE.services.ai.azure.com/openai/v1"
  model: "YOUR_DEPLOYMENT_NAME"
```

For Azure OpenAI, `model` must be the deployment name.

Use Gemini:

```yaml
detection:
  provider: "gemini"

gemini:
  api_key: "YOUR_GEMINI_API_KEY"
  model: "gemini-3.1-flash-lite"
```

## LINE Notification

```yaml
line:
  channel_access_token: "YOUR_LINE_CHANNEL_ACCESS_TOKEN"
  user_id: "YOUR_LINE_USER_ID_OR_GROUP_ID"
```

The dashboard sends LINE messages through the Messaging API push endpoint.

## LINE Images

LINE image messages require an HTTPS image URL. The recommended setup is Cloudinary:

```yaml
cloudinary:
  cloud_name: "YOUR_CLOUDINARY_CLOUD_NAME"
  api_key: "YOUR_CLOUDINARY_API_KEY"
  api_secret: "YOUR_CLOUDINARY_API_SECRET"
  folder: "sky-smart-patrol"
```

If Cloudinary is not configured, the system can still send text notifications.

## Device API Key And Web Token

These are currently defined in code:

```text
computer/road_detection_server.py
nicla/main.py
```

Before real deployment, change these values in both places:

```python
DEVICE_API_KEY = "sky_robot_2026_test"
WEB_CONTROL_TOKEN = "sky_control_2026"
```

`DEVICE_API_KEY` must match between the computer server and Nicla Vision.

## Environment Variable Override

The server also checks environment variables such as:

- `OPENAI_API_KEY`
- `OPENAI_BASE_URL`
- `OPENAI_MODEL`
- `SKY_DETECTION_PROVIDER`
- `GEMINI_API_KEY`
- `LINE_CHANNEL_ACCESS_TOKEN`
- `LINE_TARGET_ID`
- `CLOUDINARY_URL`

Environment variables take priority over `config.yaml` for supported keys.
