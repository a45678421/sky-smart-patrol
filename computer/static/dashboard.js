const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

let state = null;
let trail = [];
let lastPathSeq = 0;
let pathEpoch = null;
let holdTimer = null;
let heldCommand = null;
let displayPose = null;

const POSE_SMOOTHING = 0.25;
const POSE_SNAP_DISTANCE_MM = 500;

function token() {
  return document.getElementById('token').value.trim();
}


let tokenAlertVisible = false;

function showTokenAlert(message) {
  if (tokenAlertVisible) {
    return;
  }

  tokenAlertVisible = true;
  window.alert(message);
  tokenAlertVisible = false;
  document.getElementById('token').focus();
}

function controlTokenError(message) {
  const error = new Error(message);
  error.controlTokenError = true;
  return error;
}

function ensureControlToken() {
  if (token().trim()) {
    return true;
  }

  showTokenAlert('請先輸入控制密碼。');
  return false;
}

async function apiPost(path, body = {}) {
  if (!ensureControlToken()) {
    throw controlTokenError('missing control token');
  }

  const response = await fetch(path, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'X-Control-Token': token(),
    },
    body: JSON.stringify(body),
  });

  let data = {};
  try {
    data = await response.json();
  } catch (_) {
    data = {};
  }

  if (!response.ok) {
    if (response.status === 403 && data.error === 'invalid control token') {
      showTokenAlert('控制密碼錯誤，請重新輸入。');
      throw controlTokenError(data.error);
    }

    throw new Error(data.error || `HTTP ${response.status}`);
  }

  return data;
}

const speedSlider = document.getElementById('speed');
const speedValue = document.getElementById('speedValue');
let speedEditing = false;
let pendingSpeed = null;
let pendingSpeedStartedAt = 0;

speedSlider.addEventListener('input', () => {
  speedEditing = true;
  speedValue.textContent = speedSlider.value + '%';
});

document.getElementById('applySpeed').onclick = async () => {
  if (!ensureControlToken()) {
    return;
  }

  try {
    const percent = Number(speedSlider.value);
    pendingSpeed = percent;
    pendingSpeedStartedAt = Date.now();
    speedEditing = true;
    speedValue.textContent = percent.toFixed(0) + '%（等待車子確認）';

    const data = await apiPost('/api/speed', { percent });
    document.getElementById('status').textContent =
      '速度命令已送出：' + data.percent.toFixed(0) + '%，等待 RP2040 回報';
  } catch (error) {
    pendingSpeed = null;
    speedEditing = false;
    document.getElementById('status').textContent =
      '速度設定失敗：' + error.message;
  }
};

async function sendCommand(command) {
  try {
    await apiPost('/api/command', { command });
    return true;
  } catch (error) {
    if (error.controlTokenError) {
      stopHold(false);
    }

    document.getElementById('status').textContent =
      '控制失敗：' + error.message;
    return false;
  }
}

function startHold(command, element) {
  if (!ensureControlToken()) {
    return;
  }

  stopHold(false);
  heldCommand = command;
  element.classList.add('active');
  sendCommand(command);
  holdTimer = setInterval(() => sendCommand(command), 250);
}

function stopHold(sendStop = true) {
  const shouldSendStop = sendStop && heldCommand !== null;

  if (holdTimer) {
    clearInterval(holdTimer);
  }

  holdTimer = null;
  heldCommand = null;

  document
    .querySelectorAll('[data-hold]')
    .forEach((button) => button.classList.remove('active'));

  if (shouldSendStop) {
    sendCommand('STOP');
  }
}

function preventControlTextSelection(event) {
  event.preventDefault();
}

function bindControlSelectionGuards(element) {
  element.addEventListener('selectstart', preventControlTextSelection);
  element.addEventListener('dragstart', preventControlTextSelection);
  element.addEventListener('contextmenu', preventControlTextSelection);
}
document.querySelectorAll('[data-hold]').forEach((button) => {
  bindControlSelectionGuards(button);
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    startHold(button.dataset.hold, button);
  });

  button.addEventListener('pointerup', (event) => {
    event.preventDefault();
    stopHold();
  });

  button.addEventListener('pointercancel', () => stopHold());

  button.addEventListener('lostpointercapture', () => {
    if (heldCommand) {
      stopHold();
    }
  });
});

document.querySelectorAll('button').forEach(bindControlSelectionGuards);

document.getElementById('stop').onclick = () => {
  stopHold(false);
  sendCommand('STOP');
};

document.getElementById('auto').onclick = () => {
  stopHold(false);
  sendCommand('AUTO');
};

document.getElementById('reset').onclick = () => {
  stopHold(false);
  sendCommand('RESET_POSE');
};

document.getElementById('clear').onclick = async () => {
  try {
    await apiPost('/api/clear_path');
    trail = [];
    lastPathSeq = 0;
    pathEpoch = null;
  } catch (error) {
    document.getElementById('status').textContent =
      '清除軌跡失敗：' + error.message;
  }
};

document.getElementById('detect').onclick = async () => {
  const button = document.getElementById('detect');
  button.disabled = true;

  try {
    await apiPost('/api/detect', {
      notify_line: document.getElementById('notifyLine').checked,
    });
    renderDetection({
      busy: true,
      status: 'RUNNING',
      result: null,
      error: null,
    });
  } catch (error) {
    button.disabled = false;
    renderDetection({
      busy: false,
      status: 'ERROR',
      result: null,
      error: error.message,
    });
  }
};

document.getElementById('sendCurrentImage').onclick = async () => {
  const button = document.getElementById('sendCurrentImage');
  button.disabled = true;

  try {
    const data = await apiPost('/api/line/send_current_image');
    const modeLabel =
      data.image_mode === 'cloudinary'
        ? 'Cloudinary'
        : '公開 HTTPS';
    document.getElementById('detectionMeta').textContent =
      '目前影像已傳送到 LINE｜' + modeLabel;
  } catch (error) {
    document.getElementById('detectionMeta').textContent =
      'LINE 影像傳送失敗：' + error.message;
  } finally {
    button.disabled = false;
  }
};

document.getElementById('sendLine').onclick = async () => {
  try {
    const data = await apiPost('/api/line/send_last');
    document.getElementById('detectionMeta').textContent =
      data.image_sent
        ? 'LINE 文字與影像已送出'
        : 'LINE 文字已送出；未設定公開 HTTPS，因此沒有附圖';
  } catch (error) {
    document.getElementById('detectionMeta').textContent =
      'LINE 傳送失敗：' + error.message;
  }
};

document.getElementById('lineTest').onclick = async () => {
  try {
    await apiPost('/api/line/test');
    document.getElementById('detectionMeta').textContent =
      'LINE 測試訊息已送出';
  } catch (error) {
    document.getElementById('detectionMeta').textContent =
      'LINE 測試失敗：' + error.message;
  }
};

const keyMap = {
  ArrowUp: 'FORWARD',
  w: 'FORWARD',
  W: 'FORWARD',
  ArrowDown: 'BACKWARD',
  s: 'BACKWARD',
  S: 'BACKWARD',
  ArrowLeft: 'LEFT',
  a: 'LEFT',
  A: 'LEFT',
  ArrowRight: 'RIGHT',
  d: 'RIGHT',
  D: 'RIGHT',
};

let keyActive = false;

window.addEventListener('keydown', (event) => {
  if (event.code === 'Space') {
    event.preventDefault();
    stopHold(false);
    sendCommand('STOP');
    return;
  }

  const command = keyMap[event.key];

  if (command && !keyActive) {
    event.preventDefault();
    keyActive = true;
    startHold(
      command,
      document.querySelector(`[data-hold="${command}"]`),
    );
  }
});

window.addEventListener('keyup', (event) => {
  if (keyMap[event.key]) {
    event.preventDefault();
    keyActive = false;
    stopHold();
  }
});

window.addEventListener('blur', () => {
  keyActive = false;
  if (heldCommand) {
    stopHold();
  }
});

function mapPoint(x, y, arena) {
  return [
    (x / arena.width) * canvas.width,
    canvas.height - (y / arena.height) * canvas.height,
  ];
}

function shortestAngleDelta(from, to) {
  return ((((to - from) % 360) + 540) % 360) - 180;
}

function smoothPose(telemetry) {
  const x = Number(telemetry.x);
  const y = Number(telemetry.y);
  const heading = Number(telemetry.heading);

  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(heading)) {
    return displayPose;
  }

  if (!displayPose) {
    displayPose = { x, y, heading };
    return displayPose;
  }

  const jumpDistance = Math.hypot(x - displayPose.x, y - displayPose.y);
  if (jumpDistance > POSE_SNAP_DISTANCE_MM) {
    displayPose = { x, y, heading };
    return displayPose;
  }

  displayPose.x += (x - displayPose.x) * POSE_SMOOTHING;
  displayPose.y += (y - displayPose.y) * POSE_SMOOTHING;
  displayPose.heading =
    (displayPose.heading +
      shortestAngleDelta(displayPose.heading, heading) * POSE_SMOOTHING +
      360) %
    360;

  return displayPose;
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#080b10';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (!state) {
    return;
  }

  const arena = state.arena;
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 1;

  for (let x = 0; x <= arena.width; x += 250) {
    const p1 = mapPoint(x, 0, arena);
    const p2 = mapPoint(x, arena.height, arena);
    ctx.beginPath();
    ctx.moveTo(...p1);
    ctx.lineTo(...p2);
    ctx.stroke();
  }

  for (let y = 0; y <= arena.height; y += 250) {
    const p1 = mapPoint(0, y, arena);
    const p2 = mapPoint(arena.width, y, arena);
    ctx.beginPath();
    ctx.moveTo(...p1);
    ctx.lineTo(...p2);
    ctx.stroke();
  }

  const polygon = [
    [0, 0],
    [0, arena.height],
    [arena.width, arena.height],
    [arena.width, arena.cutout_height],
    [arena.width - arena.cutout_width, arena.cutout_height],
    [arena.width - arena.cutout_width, 0],
  ];

  ctx.beginPath();
  polygon.forEach((point, index) => {
    const mapped = mapPoint(point[0], point[1], arena);
    if (index) {
      ctx.lineTo(...mapped);
    } else {
      ctx.moveTo(...mapped);
    }
  });
  ctx.closePath();
  ctx.strokeStyle = '#e6edf3';
  ctx.lineWidth = 4;
  ctx.stroke();

  if (state.route) {
    ctx.beginPath();
    state.route.forEach((point, index) => {
      const mapped = mapPoint(point[0], point[1], arena);
      if (index) {
        ctx.lineTo(...mapped);
      } else {
        ctx.moveTo(...mapped);
      }
    });
    ctx.setLineDash([10, 8]);
    ctx.strokeStyle = '#d29922';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (trail.length > 1) {
    ctx.beginPath();
    trail.forEach((point, index) => {
      const mapped = mapPoint(point[0], point[1], arena);
      if (index) {
        ctx.lineTo(...mapped);
      } else {
        ctx.moveTo(...mapped);
      }
    });
    ctx.strokeStyle = '#2f81f7';
    ctx.lineWidth = 4;
    ctx.stroke();
  }

  const telemetry = state.telemetry;
  const robotPose = smoothPose(telemetry) || telemetry;
  const robotPoint = mapPoint(robotPose.x, robotPose.y, arena);
  const radians = (robotPose.heading * Math.PI) / 180;

  ctx.save();
  ctx.translate(robotPoint[0], robotPoint[1]);
  ctx.rotate(-radians);
  ctx.fillStyle = '#3fb950';
  ctx.beginPath();
  ctx.arc(0, 0, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(24, 0);
  ctx.stroke();
  ctx.restore();
}

const anomalyLabels = {
  normal: '正常',
  pothole: '疑似破洞',
  crack: '疑似裂縫',
  standing_water: '疑似積水',
  debris: '疑似異物',
  other: '其他異常',
  unclear: '無法確認',
};

const severityLabels = {
  none: '無',
  low: '低',
  medium: '中',
  high: '高',
};

function renderDetection(detection) {
  const panel = document.getElementById('detectionPanel');
  const headline = document.getElementById('detectionHeadline');
  const confidence = document.getElementById('detectionConfidence');
  const summary = document.getElementById('detectionSummary');
  const meta = document.getElementById('detectionMeta');
  const evidence = document.getElementById('detectionEvidence');
  const detectButton = document.getElementById('detect');

  evidence.replaceChildren();

  if (!detection || detection.status === 'IDLE') {
    panel.className = 'detection-panel idle';
    headline.textContent = '尚未辨識';
    confidence.textContent = '--';
    summary.textContent =
      '按下「辨識目前畫面」後，才會呼叫 AI API。';
    meta.textContent = '';
    detectButton.disabled = false;
    return;
  }

  if (detection.busy || detection.status === 'RUNNING') {
    panel.className = 'detection-panel running';
    headline.textContent = '辨識中';
    confidence.textContent = '請稍候';
    summary.textContent = '正在將目前畫面送至 AI 模型判讀。';
    meta.textContent = detection.model || '';
    detectButton.disabled = true;
    return;
  }

  detectButton.disabled = false;

  if (detection.status === 'ERROR') {
    panel.className = 'detection-panel error';
    headline.textContent = '辨識失敗';
    confidence.textContent = '--';
    summary.textContent = detection.error || '未知錯誤';
    meta.textContent = detection.model || '';
    return;
  }

  const result = detection.result;
  if (!result) {
    return;
  }

  panel.className =
    'detection-panel ' + (result.alert ? 'alert' : 'safe');
  headline.textContent =
    (result.alert ? '警示：' : '') +
    (anomalyLabels[result.anomaly] || result.anomaly);
  confidence.textContent =
    Number(result.confidence).toFixed(0) + '%';
  summary.textContent = result.summary_zh;
  meta.textContent =
    '嚴重度：' +
    (severityLabels[result.severity] || result.severity) +
    '｜模型：' +
    detection.model +
    (detection.line_sent ? '｜LINE 已傳送' : '') +
    (detection.line_error
      ? '｜LINE 失敗：' + detection.line_error
      : '');

  for (const item of result.evidence_zh || []) {
    const listItem = document.createElement('li');
    listItem.textContent = item;
    evidence.appendChild(listItem);
  }
}

function renderIntegrations(integrations) {
  const gemini = document.getElementById('geminiStatus');
  const line = document.getElementById('lineStatus');

  if (integrations.gemini_configured) {
    gemini.textContent = '已設定｜' + integrations.gemini_model;
    gemini.className = 'online';
  } else {
    gemini.textContent = '尚未設定 API Key';
    gemini.className = 'offline';
  }

  if (integrations.line_configured) {
    const imageModeLabels = {
      cloudinary: 'Cloudinary 圖片',
      public_https: '公開 HTTPS 圖片',
      disabled: '僅文字',
    };

    line.textContent =
      '已設定｜' +
      (imageModeLabels[integrations.line_image_mode] || '僅文字');
    line.className =
      integrations.line_image_enabled ? 'online' : 'warn';
  } else {
    line.textContent = '尚未設定';
    line.className = 'offline';
  }
}

async function refreshState() {
  try {
    const response = await fetch(
      '/api/state?after=' + lastPathSeq,
      { cache: 'no-store' },
    );
    const data = await response.json();
    state = data;

    if (
      pathEpoch !== null &&
      pathEpoch !== data.path_epoch
    ) {
      trail = [];
      lastPathSeq = 0;
    }

    pathEpoch = data.path_epoch;

    for (const point of data.path) {
      trail.push([point[1], point[2]]);
      lastPathSeq = Math.max(lastPathSeq, point[0]);
    }

    if (trail.length > 3000) {
      trail = trail.slice(-3000);
    }

    const telemetry = data.telemetry;
    document.getElementById('x').textContent =
      telemetry.x.toFixed(0) + ' mm';
    document.getElementById('y').textContent =
      telemetry.y.toFixed(0) + ' mm';
    const height = Number(telemetry.height);
    document.getElementById('height').textContent =
      Number.isFinite(height) ? height.toFixed(0) + ' mm' : '--';
    document.getElementById('heading').textContent =
      telemetry.heading.toFixed(0) + '°';
    document.getElementById('distance').textContent =
      telemetry.distance == null
        ? '--'
        : Number(telemetry.distance).toFixed(0) + ' mm';
    document.getElementById('mode').textContent =
      telemetry.mode;
    document.getElementById('status').textContent =
      telemetry.status;

    const reportedSpeed = Number(telemetry.speed);
    document.getElementById('speedMetric').textContent =
      reportedSpeed.toFixed(0) + '%';
    document.getElementById('lineError').textContent =
      Number(telemetry.line_error).toFixed(0) +
      ' mm / ' +
      telemetry.recovery;

    if (pendingSpeed !== null) {
      if (Math.abs(reportedSpeed - pendingSpeed) <= 0.5) {
        speedSlider.value = Math.round(reportedSpeed);
        speedValue.textContent =
          Math.round(reportedSpeed) + '%';
        pendingSpeed = null;
        speedEditing = false;
      } else if (Date.now() - pendingSpeedStartedAt > 5000) {
        speedValue.textContent =
          Math.round(Number(speedSlider.value)) +
          '%（未確認）';
        pendingSpeed = null;
        speedEditing = true;
      }
    } else if (!speedEditing) {
      speedSlider.value = Math.round(reportedSpeed);
      speedValue.textContent =
        Math.round(reportedSpeed) + '%';
    }

    const online =
      data.telemetry_age_s != null &&
      data.telemetry_age_s < 2.0 &&
      data.device_poll_age_s != null &&
      data.device_poll_age_s < 2.0;

    const connection = document.getElementById('connection');
    connection.textContent =
      online ? '裝置在線' : '裝置離線';
    connection.className =
      online ? 'online' : 'offline';

    document.getElementById('cameraStatus').textContent =
      data.camera_age_s == null
        ? '尚未收到影像'
        : '影像更新於 ' +
          data.camera_age_s.toFixed(1) +
          ' 秒前';
    document.getElementById('cameraFps').textContent =
      '影像 FPS：' +
      (Number(data.camera_fps) > 0
        ? Number(data.camera_fps).toFixed(2)
        : '--');

    renderIntegrations(data.integrations);
    renderDetection(data.detection);
    draw();
  } catch (error) {
    const connection = document.getElementById('connection');
    connection.textContent = '伺服器連線失敗';
    connection.className = 'offline';
  }
}

setInterval(refreshState, 250);
refreshState();

function startCameraStream() {
  const camera = document.getElementById('camera');
  camera.src = '/camera.mjpeg?t=' + Date.now();
}

startCameraStream();
