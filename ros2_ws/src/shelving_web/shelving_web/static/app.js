"use strict";


const SYSTEM_READY = 3;
const SOCKET_RECONNECT_DELAY_MS = 1500;

const dashboard = {
  socket: null,
  reconnectTimer: null,
  snapshot: null,
  mapMetadata: null,
  mapImageLoaded: false,
  commandPending: false,
  notificationTimer: null,
};

const elements = {
  connectionIndicator:
    document.getElementById("connectionIndicator"),
  connectionText:
    document.getElementById("connectionText"),
  lastUpdate:
    document.getElementById("lastUpdate"),

  mapStage:
    document.getElementById("mapStage"),
  mapImage:
    document.getElementById("mapImage"),
  mapCanvas:
    document.getElementById("mapCanvas"),
  mapLoading:
    document.getElementById("mapLoading"),
  mapFrame:
    document.getElementById("mapFrame"),
  mapPoseX:
    document.getElementById("mapPoseX"),
  mapPoseY:
    document.getElementById("mapPoseY"),
  mapPoseYaw:
    document.getElementById("mapPoseYaw"),

  stateBadge:
    document.getElementById("stateBadge"),
  statusMessage:
    document.getElementById("statusMessage"),
  runId:
    document.getElementById("runId"),
  errorCode:
    document.getElementById("errorCode"),

  startButton:
    document.getElementById("startButton"),
  buttonLabel:
    document.querySelector(
      "#startButton .button-label"
    ),
  buttonState:
    document.querySelector(
      "#startButton .button-state"
    ),
  commandResult:
    document.getElementById("commandResult"),

  readySimulation:
    document.getElementById("readySimulation"),
  readyPcA:
    document.getElementById("readyPcA"),
  readyPcB:
    document.getElementById("readyPcB"),
  readyNavigation:
    document.getElementById("readyNavigation"),
  readyManipulation:
    document.getElementById("readyManipulation"),
  readyPerception:
    document.getElementById("readyPerception"),

  progressTrack:
    document.querySelector(".progress-track"),
  progressBar:
    document.getElementById("progressBar"),
  progressText:
    document.getElementById("progressText"),
  phaseValue:
    document.getElementById("phaseValue"),
  jobId:
    document.getElementById("jobId"),

  poseX:
    document.getElementById("poseX"),
  poseY:
    document.getElementById("poseY"),
  poseYaw:
    document.getElementById("poseYaw"),

  notification:
    document.getElementById("notification"),
};


function formatNumber(value, digits = 2) {
  const number = Number(value);

  if (!Number.isFinite(number)) {
    return "--";
  }

  return number.toFixed(digits);
}


function radiansToDegrees(radians) {
  return Number(radians) * 180 / Math.PI;
}


function formatAge(ageSeconds) {
  if (
    ageSeconds === null
    || ageSeconds === undefined
    || !Number.isFinite(Number(ageSeconds))
  ) {
    return "수신 데이터 없음";
  }

  const age = Number(ageSeconds);

  if (age < 1) {
    return "방금 갱신";
  }

  if (age < 60) {
    return `${Math.floor(age)}초 전 갱신`;
  }

  return `${Math.floor(age / 60)}분 전 갱신`;
}


function stateClassName(stateName) {
  return (
    "state-"
    + String(stateName || "UNKNOWN")
      .toLowerCase()
      .replaceAll("_", "-")
  );
}


function setReadiness(element, ready) {
  element.textContent = ready ? "READY" : "WAIT";
  element.classList.toggle("is-ready", ready);
  element.classList.toggle("is-waiting", !ready);
}


function showNotification(message, type = "success") {
  window.clearTimeout(dashboard.notificationTimer);

  elements.notification.textContent = message;
  elements.notification.className = (
    `notification is-visible is-${type}`
  );

  dashboard.notificationTimer = window.setTimeout(
    () => {
      elements.notification.className = "notification";
    },
    3500,
  );
}


function renderConnection(snapshot) {
  const ros = snapshot?.ros;
  const connected = Boolean(
    ros?.status_connected
  );

  elements.connectionIndicator.classList.toggle(
    "is-online",
    connected,
  );
  elements.connectionIndicator.classList.toggle(
    "is-offline",
    !connected,
  );

  if (connected) {
    elements.connectionText.textContent = "ROS 연결됨";
  } else if (
    dashboard.socket
    && dashboard.socket.readyState === WebSocket.OPEN
  ) {
    elements.connectionText.textContent = "ROS 상태 대기";
  } else {
    elements.connectionText.textContent = "웹 게이트웨이 연결 대기";
  }

  elements.lastUpdate.textContent = formatAge(
    ros?.status_age_sec,
  );
}


function renderStatus(snapshot) {
  const status = snapshot?.status;
  const ros = snapshot?.ros;

  if (!status) {
    elements.stateBadge.textContent = "UNKNOWN";
    elements.stateBadge.className = (
      "state-badge state-unknown"
    );
    elements.statusMessage.textContent = (
      "시스템 상태를 기다리고 있습니다."
    );
    elements.runId.textContent = "--";
    elements.errorCode.textContent = "--";

    setReadiness(elements.readySimulation, false);
    setReadiness(elements.readyPcA, false);
    setReadiness(elements.readyPcB, false);
    setReadiness(elements.readyNavigation, false);
    setReadiness(elements.readyManipulation, false);
    setReadiness(elements.readyPerception, false);

    renderJob(null);
    updateStartButton(snapshot);
    return;
  }

  const stateName = status.state_name || "UNKNOWN";

  elements.stateBadge.textContent = stateName;
  elements.stateBadge.className = (
    `state-badge ${stateClassName(stateName)}`
  );

  elements.statusMessage.textContent = (
    status.message || "상태 메시지가 없습니다."
  );
  elements.runId.textContent = status.run_id || "--";
  elements.errorCode.textContent = String(
    status.error_code ?? 0
  );

  setReadiness(
    elements.readySimulation,
    Boolean(status.simulation_ready),
  );
  setReadiness(
    elements.readyPcA,
    Boolean(status.pc_a_ready),
  );
  setReadiness(
    elements.readyPcB,
    Boolean(status.pc_b_ready),
  );
  setReadiness(
    elements.readyNavigation,
    Boolean(status.navigation_ready),
  );
  setReadiness(
    elements.readyManipulation,
    Boolean(status.manipulation_ready),
  );
  setReadiness(
    elements.readyPerception,
    Boolean(status.perception_ready),
  );

  renderJob(status);
  updateStartButton({
    status,
    ros,
  });
}


function renderJob(status) {
  const rawProgress = Number(status?.progress ?? 0);
  const boundedProgress = Math.max(
    0,
    Math.min(1, rawProgress),
  );
  const percentage = Math.round(
    boundedProgress * 100
  );

  elements.progressBar.style.width = `${percentage}%`;
  elements.progressText.textContent = `${percentage}%`;
  elements.progressTrack.setAttribute(
    "aria-valuenow",
    String(percentage),
  );

  elements.phaseValue.textContent = (
    status?.phase || "--"
  );
  elements.jobId.textContent = (
    status?.active_job_id || "--"
  );
}


function renderPose(snapshot) {
  const pose = snapshot?.pose;

  if (!pose) {
    elements.poseX.textContent = "--";
    elements.poseY.textContent = "--";
    elements.poseYaw.textContent = "--";

    elements.mapPoseX.textContent = "--";
    elements.mapPoseY.textContent = "--";
    elements.mapPoseYaw.textContent = "--";
    elements.mapFrame.textContent = "frame: --";

    updateMapLoading();
    drawRobotOnMap();
    return;
  }

  const x = formatNumber(pose.x);
  const y = formatNumber(pose.y);
  const yawDegrees = formatNumber(
    radiansToDegrees(pose.yaw),
    1,
  );

  elements.poseX.textContent = x;
  elements.poseY.textContent = y;
  elements.poseYaw.textContent = yawDegrees;

  elements.mapPoseX.textContent = x;
  elements.mapPoseY.textContent = y;
  elements.mapPoseYaw.textContent = `${yawDegrees}°`;
  elements.mapFrame.textContent = (
    `frame: ${pose.frame_id || "--"}`
  );

  updateMapLoading();
  drawRobotOnMap();
}


function updateStartButton(snapshot) {
  const status = snapshot?.status;
  const ros = snapshot?.ros;

  const canStart = Boolean(
    !dashboard.commandPending
    && ros?.status_connected
    && ros?.start_service_ready
    && status?.state === SYSTEM_READY
  );

  elements.startButton.disabled = !canStart;

  if (dashboard.commandPending) {
    elements.buttonLabel.textContent = "명령 전송 중";
    elements.buttonState.textContent = "WAIT";
    return;
  }

  elements.buttonLabel.textContent = "사이클 시작";

  if (!ros?.status_connected) {
    elements.buttonState.textContent = "ROS 연결 필요";
  } else if (!ros?.start_service_ready) {
    elements.buttonState.textContent = "서비스 대기";
  } else if (status?.state !== SYSTEM_READY) {
    elements.buttonState.textContent = (
      status?.state_name || "READY 필요"
    );
  } else {
    elements.buttonState.textContent = "실행 가능";
  }
}


function renderSnapshot(snapshot) {
  dashboard.snapshot = snapshot;

  renderConnection(snapshot);
  renderStatus(snapshot);
  renderPose(snapshot);
}


async function loadMapMetadata() {
  try {
    const response = await fetch("/api/map", {
      cache: "no-store",
    });

    if (!response.ok) {
      throw new Error(
        `Map metadata request failed: ${response.status}`
      );
    }

    dashboard.mapMetadata = await response.json();
    updateMapLoading();
    drawRobotOnMap();
  } catch (error) {
    console.error(error);
    elements.mapLoading.textContent = (
      "Nav2 지도 정보를 불러오지 못했습니다."
    );
    elements.mapLoading.classList.remove("is-hidden");
  }
}


function updateMapLoading() {
  if (
    !dashboard.mapMetadata
    || !dashboard.mapImageLoaded
  ) {
    elements.mapLoading.textContent = (
      "Nav2 지도를 불러오는 중입니다."
    );
    elements.mapLoading.classList.remove("is-hidden");
    return;
  }

  if (!dashboard.snapshot?.pose) {
    elements.mapLoading.textContent = (
      "AMCL 로봇 위치를 기다리고 있습니다."
    );
    elements.mapLoading.classList.remove("is-hidden");
    return;
  }

  elements.mapLoading.classList.add("is-hidden");
}


function resizeMapCanvas() {
  if (!dashboard.mapImageLoaded) {
    return null;
  }

  const imageRect =
    elements.mapImage.getBoundingClientRect();
  const stageRect =
    elements.mapStage.getBoundingClientRect();

  if (
    imageRect.width <= 0
    || imageRect.height <= 0
  ) {
    return null;
  }

  const deviceScale = window.devicePixelRatio || 1;

  elements.mapCanvas.style.left = (
    `${imageRect.left - stageRect.left}px`
  );
  elements.mapCanvas.style.top = (
    `${imageRect.top - stageRect.top}px`
  );
  elements.mapCanvas.style.width = `${imageRect.width}px`;
  elements.mapCanvas.style.height = `${imageRect.height}px`;

  elements.mapCanvas.width = Math.round(
    imageRect.width * deviceScale
  );
  elements.mapCanvas.height = Math.round(
    imageRect.height * deviceScale
  );

  const context = elements.mapCanvas.getContext("2d");

  context.setTransform(
    deviceScale,
    0,
    0,
    deviceScale,
    0,
    0,
  );

  return {
    context,
    width: imageRect.width,
    height: imageRect.height,
  };
}


function worldToDisplayedPixel(
  x,
  y,
  displayWidth,
  displayHeight,
) {
  const metadata = dashboard.mapMetadata;
  const image = elements.mapImage;

  if (
    !metadata
    || !image.naturalWidth
    || !image.naturalHeight
  ) {
    return null;
  }

  const originX = Number(metadata.origin[0]);
  const originY = Number(metadata.origin[1]);
  const originYaw = Number(metadata.origin[2]);
  const resolution = Number(metadata.resolution);

  if (
    !Number.isFinite(resolution)
    || resolution <= 0
  ) {
    return null;
  }

  const deltaX = Number(x) - originX;
  const deltaY = Number(y) - originY;

  const cosine = Math.cos(originYaw);
  const sine = Math.sin(originYaw);

  const mapX = (
    cosine * deltaX + sine * deltaY
  ) / resolution;

  const mapY = (
    -sine * deltaX + cosine * deltaY
  ) / resolution;

  const imageX = mapX;
  const imageY = (
    image.naturalHeight - 1 - mapY
  );

  return {
    x: (
      imageX
      * displayWidth
      / image.naturalWidth
    ),
    y: (
      imageY
      * displayHeight
      / image.naturalHeight
    ),
    originYaw,
  };
}


function drawRobotOnMap() {
  const canvasState = resizeMapCanvas();

  if (!canvasState) {
    return;
  }

  const {
    context,
    width,
    height,
  } = canvasState;

  context.clearRect(0, 0, width, height);

  const pose = dashboard.snapshot?.pose;

  if (!pose || !dashboard.mapMetadata) {
    return;
  }

  const pixel = worldToDisplayedPixel(
    pose.x,
    pose.y,
    width,
    height,
  );

  if (!pixel) {
    return;
  }

  if (
    pixel.x < 0
    || pixel.y < 0
    || pixel.x > width
    || pixel.y > height
  ) {
    return;
  }

  const relativeYaw = (
    Number(pose.yaw) - pixel.originYaw
  );

  const headingLength = 30;
  const headingX = (
    pixel.x
    + Math.cos(relativeYaw) * headingLength
  );
  const headingY = (
    pixel.y
    - Math.sin(relativeYaw) * headingLength
  );

  context.save();

  context.lineCap = "round";
  context.lineJoin = "round";

  context.strokeStyle = "#0a0e11";
  context.lineWidth = 7;
  context.beginPath();
  context.moveTo(pixel.x, pixel.y);
  context.lineTo(headingX, headingY);
  context.stroke();

  context.strokeStyle = "#55b8ff";
  context.lineWidth = 4;
  context.beginPath();
  context.moveTo(pixel.x, pixel.y);
  context.lineTo(headingX, headingY);
  context.stroke();

  const arrowAngle = Math.atan2(
    headingY - pixel.y,
    headingX - pixel.x,
  );
  const arrowSize = 9;

  context.fillStyle = "#55b8ff";
  context.beginPath();
  context.moveTo(headingX, headingY);
  context.lineTo(
    headingX - arrowSize * Math.cos(
      arrowAngle - Math.PI / 6
    ),
    headingY - arrowSize * Math.sin(
      arrowAngle - Math.PI / 6
    ),
  );
  context.lineTo(
    headingX - arrowSize * Math.cos(
      arrowAngle + Math.PI / 6
    ),
    headingY - arrowSize * Math.sin(
      arrowAngle + Math.PI / 6
    ),
  );
  context.closePath();
  context.fill();

  context.shadowColor = "rgba(0, 0, 0, 0.45)";
  context.shadowBlur = 10;
  context.fillStyle = "#ff6577";
  context.beginPath();
  context.arc(pixel.x, pixel.y, 9, 0, Math.PI * 2);
  context.fill();

  context.shadowBlur = 0;
  context.strokeStyle = "#ffffff";
  context.lineWidth = 3;
  context.stroke();

  context.restore();
}


function connectWebSocket() {
  window.clearTimeout(dashboard.reconnectTimer);

  const protocol = (
    window.location.protocol === "https:"
      ? "wss:"
      : "ws:"
  );

  const socketUrl = (
    `${protocol}//${window.location.host}/ws`
  );

  const socket = new WebSocket(socketUrl);
  dashboard.socket = socket;

  socket.addEventListener("open", () => {
    renderConnection(dashboard.snapshot);
  });

  socket.addEventListener("message", (event) => {
    try {
      const snapshot = JSON.parse(event.data);
      renderSnapshot(snapshot);
    } catch (error) {
      console.error(
        "Invalid dashboard WebSocket payload:",
        error,
      );
    }
  });

  socket.addEventListener("close", () => {
    renderConnection(null);

    dashboard.reconnectTimer = window.setTimeout(
      connectWebSocket,
      SOCKET_RECONNECT_DELAY_MS,
    );
  });

  socket.addEventListener("error", () => {
    socket.close();
  });
}


async function requestStartCycle() {
  if (dashboard.commandPending) {
    return;
  }

  dashboard.commandPending = true;
  elements.commandResult.textContent = "";
  elements.commandResult.className = "command-result";
  updateStartButton(dashboard.snapshot);

  try {
    const response = await fetch("/api/start", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
    });

    const result = await response.json();

    if (!response.ok || !result.ok) {
      throw new Error(
        result.message || "작업 시작 요청이 거부됐습니다."
      );
    }

    elements.commandResult.textContent = result.message;
    elements.commandResult.className = (
      "command-result is-success"
    );

    showNotification(
      "새 도서 배치 사이클을 시작했습니다.",
      "success",
    );
  } catch (error) {
    const message = (
      error instanceof Error
        ? error.message
        : "작업 시작 요청에 실패했습니다."
    );

    elements.commandResult.textContent = message;
    elements.commandResult.className = (
      "command-result is-error"
    );

    showNotification(message, "error");
  } finally {
    dashboard.commandPending = false;
    updateStartButton(dashboard.snapshot);
  }
}


function initializeDashboard() {
  elements.startButton.addEventListener(
    "click",
    requestStartCycle,
  );

  elements.mapImage.addEventListener("load", () => {
    dashboard.mapImageLoaded = true;
    updateMapLoading();
    drawRobotOnMap();
  });

  if (elements.mapImage.complete) {
    dashboard.mapImageLoaded = true;
  }

  if ("ResizeObserver" in window) {
    const resizeObserver = new ResizeObserver(
      drawRobotOnMap
    );
    resizeObserver.observe(elements.mapStage);
  } else {
    window.addEventListener(
      "resize",
      drawRobotOnMap,
    );
  }

  window.addEventListener("beforeunload", () => {
    window.clearTimeout(dashboard.reconnectTimer);

    if (dashboard.socket) {
      dashboard.socket.close();
    }
  });

  loadMapMetadata();
  connectWebSocket();
  updateMapLoading();
  updateStartButton(null);
}


initializeDashboard();