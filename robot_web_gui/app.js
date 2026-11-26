const statusLight = document.getElementById("status-light");
const statusText = document.getElementById("status-text");
const msgTypeSelect = document.getElementById("msg-type");
const navEngineSelect = document.getElementById("nav-engine");

const powerSwitch = document.getElementById("robot-power-switch");
const powerStateLabel = document.getElementById("power-state-label");

const scanModeBtn = document.getElementById("scan-mode-btn");
const autopilotBtn = document.getElementById("autopilot-btn");
const autopilotStatusBox = document.getElementById("autopilot-status");

const alertBanner = document.getElementById("alert-banner");
const alertMessage = document.getElementById("alert-message");
const alertActionBtn = document.getElementById("alert-action-btn");
const alertClose = document.getElementById("alert-close");

const mapSourcePill = document.getElementById("map-source-pill");
const saveMapBtn = document.getElementById("save-map-btn");
const uploadMapBtn = document.getElementById("upload-map-btn");
const setPoseBtn = document.getElementById("set-pose-btn");
const resumeSlamBtn = document.getElementById("resume-slam-btn");
const uploadMapInput = document.getElementById("upload-map-input");

const togglePinBtn = document.getElementById("toggle-pin-btn");
const undoWpBtn = document.getElementById("undo-wp-btn");
const clearWpBtn = document.getElementById("clear-wp-btn");
const wpListContainer = document.getElementById("wp-list");

const saveMapModal = document.getElementById("save-map-modal");
const mapNameInput = document.getElementById("map-name-input");
const filePreview = document.getElementById("file-preview");
const cancelSaveBtn = document.getElementById("cancel-save-btn");
const confirmSaveBtn = document.getElementById("confirm-save-btn");

const dpadContainer = document.getElementById("dpad-container");
const teleopNote = document.getElementById("teleop-note");

const batteryFill = document.getElementById("battery-fill");
const batteryPctText = document.getElementById("battery-pct");
const batteryTimeText = document.getElementById("battery-time");

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");

const linSlider = document.getElementById("lin-speed");
const angSlider = document.getElementById("ang-speed");
linSlider.oninput = () => (document.getElementById("lin-val").textContent = Number(linSlider.value).toFixed(2));
angSlider.oninput = () => (document.getElementById("ang-val").textContent = Number(angSlider.value).toFixed(2));

const wsUrl = `ws://${window.location.hostname}:9090`;
let ws = null;
let lastHeartbeatTime = 0;
let robotActive = true;
let autoExploreActive = false;
let autopilotActive = false;
let alertTimeout = null;
let saveCounter = 1;

// Map state buffers
let liveSlamMapData = null;
let uploadedMapData = null;
let mapData = null;
let usingUploadedMap = false;
let pendingYamlMeta = null;

// Waypoints & Target End Point state
let pinModeEnabled = true;
let setPoseMode = false;
let poseDragStart = null;
let poseDragCurrent = null;
let waypoints = [];         // Array of { id, x, y }
let targetIndex = -1;       // Index in waypoints[] designated as Targeted End Point
let nextWaypointId = 1;
let hoverWorldCoord = null;

// Autonomous Navigation State (Uploaded Map Autopilot)
let plannedPath = [];
let nav2GlobalPath = [];
let nav2ActiveLegIndex = 0;
let lastReplanTime = 0;
let avoidanceHoldUntil = 0;
let avoidanceTurnDir = 1;

// Autonomous Auto-Explore State (Live SLAM Scanning)
let explorePath = [];
let lastExplorePlanTime = 0;
let explorePivotUntil = 0;
let explorePivotDir = 1;
let exploreStepCount = 0;
let unreachableFrontiers = []; // [{ x, y, until }]

let scanData = null;
let rawOdomPose = { x: 0, y: 0, yaw: 0 };
let mapToOdomOffset = { x: 0, y: 0, yaw: 0 };
let hasExternalMapPose = false;
let robotPose = { x: 0, y: 0, yaw: 0 };

// 3S LiPo battery constants (TurtleBot3 Burger)
const V_MIN = 11.0;
const V_MAX = 12.6;
const DISCHARGE_RATE = 0.59;

function normalizeAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function updateRobotPoseFromOdom() {
  if (hasExternalMapPose) return;
  const cos = Math.cos(mapToOdomOffset.yaw);
  const sin = Math.sin(mapToOdomOffset.yaw);
  robotPose.x = mapToOdomOffset.x + rawOdomPose.x * cos - rawOdomPose.y * sin;
  robotPose.y = mapToOdomOffset.y + rawOdomPose.x * sin + rawOdomPose.y * cos;
  robotPose.yaw = normalizeAngle(mapToOdomOffset.yaw + rawOdomPose.yaw);
}

function calibrateRobotPoseOnMap(desiredX, desiredY, desiredYaw) {
  const yawOffset = normalizeAngle(desiredYaw - rawOdomPose.yaw);
  const cos = Math.cos(yawOffset);
  const sin = Math.sin(yawOffset);
  mapToOdomOffset = {
    x: desiredX - (rawOdomPose.x * cos - rawOdomPose.y * sin),
    y: desiredY - (rawOdomPose.x * sin + rawOdomPose.y * cos),
    yaw: yawOffset
  };
  hasExternalMapPose = false;
  updateRobotPoseFromOdom();

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      op: "publish",
      topic: "/initialpose",
      msg: {
        header: { frame_id: "map" },
        pose: {
          pose: {
            position: { x: desiredX, y: desiredY, z: 0.0 },
            orientation: {
              x: 0.0,
              y: 0.0,
              z: Math.sin(desiredYaw / 2),
              w: Math.cos(desiredYaw / 2)
            }
          }
        }
      }
    }));
  }
}

function showBanner(message, type = "error", actionLabel = null, onAction = null) {
  alertBanner.classList.remove("success");
  if (type === "success") {
    alertBanner.classList.add("success");
    alertMessage.textContent = `✓ ${message}`;
  } else {
    alertMessage.textContent = `⚠ ${message}`;
  }

  if (actionLabel && typeof onAction === "function") {
    alertActionBtn.textContent = actionLabel;
    alertActionBtn.style.display = "inline-flex";
    alertActionBtn.onclick = onAction;
  } else {
    alertActionBtn.style.display = "none";
    alertActionBtn.onclick = null;
  }

  alertBanner.classList.add("visible");
  if (alertTimeout) clearTimeout(alertTimeout);
  if (!actionLabel) {
    alertTimeout = setTimeout(() => {
      alertBanner.classList.remove("visible");
    }, 6500);
  }
}

function showError(message, actionLabel = null, onAction = null) {
  showBanner(message, "error", actionLabel, onAction);
}

function showSuccess(message, actionLabel = null, onAction = null) {
  showBanner(message, "success", actionLabel, onAction);
}

function hideError() {
  alertBanner.classList.remove("visible");
  alertActionBtn.style.display = "none";
  if (alertTimeout) clearTimeout(alertTimeout);
}

alertClose.onclick = hideError;

function setConnectedState(isConnected, label) {
  statusLight.classList.toggle("connected", isConnected);
  statusText.textContent = label;
}

function hasValidMap() {
  if (!mapData || !Array.isArray(mapData.data) || mapData.data.length === 0) {
    return false;
  }
  return mapData.data.some((cell) => cell !== -1);
}

function syncModeButtonsUI() {
  // Scan Mode Button
  scanModeBtn.classList.toggle("auto", autoExploreActive);
  scanModeBtn.textContent = autoExploreActive
    ? "🔍 Scan Mode: AUTO-EXPLORE"
    : "🧭 Scan Mode: MANUAL";

  // Autopilot Button (Only enabled/ready when an uploaded map is loaded)
  const hasUploaded = usingUploadedMap && uploadedMapData !== null;
  autopilotBtn.classList.toggle("active", autopilotActive);
  autopilotBtn.classList.toggle("ready", !autopilotActive && hasUploaded);
  autopilotBtn.textContent = autopilotActive
    ? "🚀 Autopilot: ON"
    : "🚀 Autopilot: OFF";
}

function setMapSourceMode(useUploaded, labelName = "") {
  usingUploadedMap = useUploaded;
  plannedPath = [];
  if (useUploaded && uploadedMapData) {
    if (autoExploreActive) {
      setAutoExploreState(false);
    }
    mapData = uploadedMapData;
    mapSourcePill.textContent = `Source: Uploaded (${labelName})`;
    mapSourcePill.classList.add("uploaded");
    resumeSlamBtn.style.display = "inline-flex";
  } else {
    usingUploadedMap = false;
    if (autopilotActive) {
      setAutopilotState(false);
    }
    mapData = liveSlamMapData;
    mapSourcePill.textContent = "Source: Live SLAM (/map)";
    mapSourcePill.classList.remove("uploaded");
    resumeSlamBtn.style.display = "none";
  }
  syncModeButtonsUI();
}

resumeSlamBtn.onclick = () => {
  setMapSourceMode(false);
  showSuccess("Switched back to Live SLAM map (/map).");
};

// --- Shared Coordinate Transform (World <-> Canvas <-> Grid) ---
function getMapTransform() {
  if (!mapData) {
    const resolution = 0.05;
    const width = 160;
    const height = 160;
    const scale = Math.min(canvas.width / width, canvas.height / height);
    const originX = robotPose.x - (width * resolution) / 2;
    const originY = robotPose.y - (height * resolution) / 2;
    return {
      width,
      height,
      resolution,
      origin: { position: { x: originX, y: originY, z: 0 } },
      scale,
      offsetX: 0,
      offsetY: 0,
      toCanvasX: (wx) => ((wx - originX) / resolution) * scale,
      toCanvasY: (wy) => (height - (wy - originY) / resolution) * scale,
      worldToGrid: (wx, wy) => ({
        gx: Math.floor((wx - originX) / resolution),
        gy: Math.floor((wy - originY) / resolution)
      }),
      gridToWorld: (gx, gy) => ({
        x: originX + (gx + 0.5) * resolution,
        y: originY + (gy + 0.5) * resolution
      }),
      toWorld: (cx, cy) => ({
        x: originX + (cx / scale) * resolution,
        y: originY + (height - cy / scale) * resolution,
        inBounds: false,
        occ: null
      })
    };
  }

  const { width, height, resolution, origin } = mapData.info;
  const scale = Math.min(canvas.width / width, canvas.height / height);
  const offsetX = (canvas.width - width * scale) / 2;
  const offsetY = (canvas.height - height * scale) / 2;

  return {
    width,
    height,
    resolution,
    origin,
    scale,
    offsetX,
    offsetY,
    toCanvasX: (wx) => offsetX + ((wx - origin.position.x) / resolution) * scale,
    toCanvasY: (wy) => offsetY + (height - (wy - origin.position.y) / resolution) * scale,
    worldToGrid: (wx, wy) => ({
      gx: Math.floor((wx - origin.position.x) / resolution),
      gy: Math.floor((wy - origin.position.y) / resolution)
    }),
    gridToWorld: (gx, gy) => ({
      x: origin.position.x + (gx + 0.5) * resolution,
      y: origin.position.y + (gy + 0.5) * resolution
    }),
    toWorld: (cx, cy) => {
      const gridX = (cx - offsetX) / scale;
      const gridY = height - (cy - offsetY) / scale;
      const gx = Math.floor(gridX);
      const gy = Math.floor(gridY);
      const inBounds = gx >= 0 && gx < width && gy >= 0 && gy < height;
      const occ = inBounds ? mapData.data[gy * width + gx] : null;
      return {
        x: origin.position.x + gridX * resolution,
        y: origin.position.y + gridY * resolution,
        inBounds,
        occ
      };
    }
  };
}

// --- A* Global Path Planner with Wall Inflation ---
class MinHeap {
  constructor() {
    this.items = [];
  }
  push(node) {
    this.items.push(node);
    let idx = this.items.length - 1;
    while (idx > 0) {
      const parent = (idx - 1) >> 1;
      if (this.items[parent].f <= node.f) break;
      this.items[idx] = this.items[parent];
      idx = parent;
    }
    this.items[idx] = node;
  }
  pop() {
    if (this.items.length === 0) return null;
    const top = this.items[0];
    const end = this.items.pop();
    if (this.items.length > 0) {
      let idx = 0;
      const len = this.items.length;
      while (true) {
        let smallest = idx;
        const left = (idx << 1) + 1;
        const right = left + 1;
        if (left < len && this.items[left].f < this.items[smallest].f) smallest = left;
        if (right < len && this.items[right].f < this.items[smallest].f) smallest = right;
        if (smallest === idx) break;
        this.items[idx] = this.items[smallest];
        idx = smallest;
      }
      this.items[idx] = end;
    }
    return top;
  }
  get size() {
    return this.items.length;
  }
}

function buildInflatedCostGrid(tf) {
  const { width, height, resolution } = tf;
  const total = width * height;
  const hardRadiusCells = Math.max(1, Math.ceil(0.14 / resolution));
  const softRadiusCells = Math.max(hardRadiusCells + 1, Math.ceil(0.28 / resolution));

  const distSq = new Int32Array(total);
  distSq.fill(999999);

  const obstacles = [];
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const occ = mapData.data[y * width + x];
      if (occ > 50) {
        distSq[y * width + x] = 0;
        obstacles.push(x, y);
      }
    }
  }

  const rMax = softRadiusCells;
  for (let i = 0; i < obstacles.length; i += 2) {
    const ox = obstacles[i];
    const oy = obstacles[i + 1];
    const minX = Math.max(0, ox - rMax);
    const maxX = Math.min(width - 1, ox + rMax);
    const minY = Math.max(0, oy - rMax);
    const maxY = Math.min(height - 1, oy + rMax);

    for (let ny = minY; ny <= maxY; ny++) {
      const dy = ny - oy;
      for (let nx = minX; nx <= maxX; nx++) {
        const dx = nx - ox;
        const d2 = dx * dx + dy * dy;
        const idx = ny * width + nx;
        if (d2 < distSq[idx]) distSq[idx] = d2;
      }
    }
  }

  const hardSq = hardRadiusCells * hardRadiusCells;
  const softSq = softRadiusCells * softRadiusCells;
  const costGrid = new Float32Array(total);

  for (let i = 0; i < total; i++) {
    const occ = mapData.data[i];
    if (distSq[i] <= hardSq || occ > 50) {
      costGrid[i] = Infinity;
    } else if (occ === -1) {
      costGrid[i] = 4.0;
    } else if (distSq[i] < softSq) {
      const ratio = 1 - Math.sqrt(distSq[i]) / softRadiusCells;
      costGrid[i] = 1.0 + ratio * 6.0;
    } else {
      costGrid[i] = 1.0;
    }
  }

  return costGrid;
}

function findNearestFreeCell(gx, gy, width, height, costGrid, maxSearch = 12) {
  const clampX = Math.min(width - 1, Math.max(0, gx));
  const clampY = Math.min(height - 1, Math.max(0, gy));
  if (isFinite(costGrid[clampY * width + clampX])) {
    return { gx: clampX, gy: clampY };
  }
  for (let r = 1; r <= maxSearch; r++) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        const nx = clampX + dx;
        const ny = clampY + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
          if (isFinite(costGrid[ny * width + nx])) {
            return { gx: nx, gy: ny };
          }
        }
      }
    }
  }
  return null;
}

function runAStarSegment(startWorld, goalWorld, tf, costGrid) {
  const { width, height } = tf;
  const sRaw = tf.worldToGrid(startWorld.x, startWorld.y);
  const gRaw = tf.worldToGrid(goalWorld.x, goalWorld.y);

  const start = findNearestFreeCell(sRaw.gx, sRaw.gy, width, height, costGrid);
  const goal = findNearestFreeCell(gRaw.gx, gRaw.gy, width, height, costGrid);
  if (!start || !goal) return null;

  const total = width * height;
  const gScore = new Float32Array(total);
  gScore.fill(Infinity);
  const cameFrom = new Int32Array(total);
  cameFrom.fill(-1);
  const closed = new Uint8Array(total);

  const startIdx = start.gy * width + start.gx;
  const goalIdx = goal.gy * width + goal.gx;
  gScore[startIdx] = 0;

  const open = new MinHeap();
  open.push({ idx: startIdx, f: Math.hypot(goal.gx - start.gx, goal.gy - start.gy) });

  const dirs = [
    [1, 0, 1.0], [-1, 0, 1.0], [0, 1, 1.0], [0, -1, 1.0],
    [1, 1, 1.414], [1, -1, 1.414], [-1, 1, 1.414], [-1, -1, 1.414]
  ];

  while (open.size > 0) {
    const curr = open.pop();
    const cIdx = curr.idx;
    if (cIdx === goalIdx) {
      const pts = [];
      let trace = goalIdx;
      while (trace !== -1) {
        const tx = trace % width;
        const ty = (trace - tx) / width;
        pts.push(tf.gridToWorld(tx, ty));
        trace = cameFrom[trace];
      }
      pts.reverse();
      return pts;
    }

    if (closed[cIdx]) continue;
    closed[cIdx] = 1;

    const cx = cIdx % width;
    const cy = (cIdx - cx) / width;

    for (const [dx, dy, stepDist] of dirs) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
      const nIdx = ny * width + nx;
      if (closed[nIdx]) continue;

      const cellPenalty = costGrid[nIdx];
      if (!isFinite(cellPenalty)) continue;

      if (dx !== 0 && dy !== 0) {
        if (!isFinite(costGrid[cy * width + nx]) || !isFinite(costGrid[ny * width + cx])) continue;
      }

      const tentativeG = gScore[cIdx] + stepDist * cellPenalty;
      if (tentativeG < gScore[nIdx]) {
        gScore[nIdx] = tentativeG;
        cameFrom[nIdx] = cIdx;
        const h = Math.hypot(goal.gx - nx, goal.gy - ny);
        open.push({ idx: nIdx, f: tentativeG + h });
      }
    }
  }

  return null;
}

function smoothPath(rawPts) {
  if (!rawPts || rawPts.length <= 2) return rawPts || [];
  const sampled = [rawPts[0]];
  for (let i = 2; i < rawPts.length - 1; i += 2) {
    sampled.push(rawPts[i]);
  }
  sampled.push(rawPts[rawPts.length - 1]);

  if (sampled.length <= 2) return sampled;
  const smoothed = [sampled[0]];
  for (let i = 1; i < sampled.length - 1; i++) {
    smoothed.push({
      x: 0.25 * sampled[i - 1].x + 0.5 * sampled[i].x + 0.25 * sampled[i + 1].x,
      y: 0.25 * sampled[i - 1].y + 0.5 * sampled[i].y + 0.25 * sampled[i + 1].y
    });
  }
  smoothed.push(sampled[sampled.length - 1]);
  return smoothed;
}

function computeFullPlannedPath(plan) {
  if (!hasValidMap() || !plan || plan.route.length === 0) return null;
  const tf = getMapTransform();
  const costGrid = buildInflatedCostGrid(tf);
  const fullPoints = [];
  let cursor = { x: robotPose.x, y: robotPose.y };

  for (const wp of plan.route) {
    const seg = runAStarSegment(cursor, wp, tf, costGrid);
    if (!seg) return null;
    const cleanSeg = smoothPath(seg);
    if (fullPoints.length > 0) cleanSeg.shift();
    fullPoints.push(...cleanSeg);
    cursor = wp;
  }

  if (fullPoints.length > 0) {
    fullPoints[fullPoints.length - 1] = { x: plan.target.x, y: plan.target.y };
  }
  return fullPoints;
}

// --- Waypoint & Targeted End Point Selection ---
function getActiveRouteAndTarget() {
  if (waypoints.length === 0 || targetIndex < 0 || targetIndex >= waypoints.length) {
    return null;
  }
  const route = waypoints.slice(0, targetIndex + 1);
  const target = waypoints[targetIndex];
  return { route, target };
}

function previewPlanIfPossible() {
  const plan = getActiveRouteAndTarget();
  if (!plan || !hasValidMap() || navEngineSelect.value !== "builtin") {
    plannedPath = [];
    return;
  }
  const path = computeFullPlannedPath(plan);
  plannedPath = path || [];
}

function renderWaypointList() {
  undoWpBtn.disabled = waypoints.length === 0;
  clearWpBtn.disabled = waypoints.length === 0;

  if (waypoints.length === 0) {
    wpListContainer.innerHTML = `<div class="wp-empty">No waypoints placed yet. Click on the map to add points.</div>`;
    return;
  }

  wpListContainer.innerHTML = waypoints
    .map((wp, idx) => {
      const isTarget = idx === targetIndex;
      return `
        <div class="wp-item ${isTarget ? "is-target" : ""}">
          <div class="wp-info">
            <span class="wp-num">${idx + 1}</span>
            <span>(${wp.x.toFixed(2)}m, ${wp.y.toFixed(2)}m)</span>
          </div>
          <div class="wp-item-actions">
            <button
              type="button"
              class="target-pill ${isTarget ? "active" : ""}"
              onclick="selectTargetWaypoint(${idx})"
              title="Set this point as the Autopilot end target"
            >
              ${isTarget ? "🎯 Target" : "Set Target"}
            </button>
            <button
              type="button"
              class="wp-del"
              onclick="removeWaypoint(${idx})"
              title="Remove point"
            >✕</button>
          </div>
        </div>
      `;
    })
    .join("");
}

window.selectTargetWaypoint = (idx) => {
  if (idx >= 0 && idx < waypoints.length) {
    targetIndex = idx;
    renderWaypointList();
    previewPlanIfPossible();
    updateTeleopLockState();
  }
};

window.removeWaypoint = (idx) => {
  if (idx < 0 || idx >= waypoints.length) return;
  waypoints.splice(idx, 1);
  if (waypoints.length === 0) {
    targetIndex = -1;
    plannedPath = [];
    if (autopilotActive) setAutopilotState(false);
  } else if (targetIndex >= waypoints.length) {
    targetIndex = waypoints.length - 1;
  } else if (idx < targetIndex) {
    targetIndex -= 1;
  }
  renderWaypointList();
  previewPlanIfPossible();
  updateTeleopLockState();
};

togglePinBtn.onclick = () => {
  if (setPoseMode) setPoseCalibrationMode(false);
  pinModeEnabled = !pinModeEnabled;
  togglePinBtn.textContent = pinModeEnabled ? "📍 Pin Mode: ON" : "📍 Pin Mode: OFF";
  togglePinBtn.classList.toggle("off", !pinModeEnabled);
  canvas.classList.toggle("pin-cursor", pinModeEnabled);
};

function setPoseCalibrationMode(enable) {
  setPoseMode = enable;
  poseDragStart = null;
  poseDragCurrent = null;
  setPoseBtn.classList.toggle("active", enable);
  setPoseBtn.textContent = enable ? "🤖 Click & Drag Heading..." : "🤖 Set Robot Pose";
  canvas.classList.toggle("pin-cursor", enable || pinModeEnabled);
  if (enable) {
    showSuccess("Click on the map where the robot is located, then drag in the direction it is facing.");
  }
}

setPoseBtn.onclick = () => {
  if (!hasValidMap()) {
    showError("Load or scan a map first before setting the robot pose.");
    return;
  }
  setPoseCalibrationMode(!setPoseMode);
};

undoWpBtn.onclick = () => {
  if (waypoints.length === 0) return;
  window.removeWaypoint(waypoints.length - 1);
};

clearWpBtn.onclick = () => {
  waypoints = [];
  targetIndex = -1;
  plannedPath = [];
  nav2GlobalPath = [];
  if (autopilotActive) setAutopilotState(false);
  renderWaypointList();
  updateTeleopLockState();
};

navEngineSelect.onchange = () => {
  if (autopilotActive) {
    setAutopilotState(false);
  }
  previewPlanIfPossible();
};

function getCanvasPointerCoords(e) {
  const rect = canvas.getBoundingClientRect();
  return {
    cx: (e.clientX - rect.left) * (canvas.width / rect.width),
    cy: (e.clientY - rect.top) * (canvas.height / rect.height)
  };
}

canvas.addEventListener("mousedown", (e) => {
  if (!setPoseMode || !hasValidMap()) return;
  const tf = getMapTransform();
  const { cx, cy } = getCanvasPointerCoords(e);
  const pt = tf.toWorld(cx, cy);
  if (pt.inBounds) {
    poseDragStart = pt;
    poseDragCurrent = pt;
  }
});

canvas.addEventListener("mousemove", (e) => {
  if (!hasValidMap()) {
    hoverWorldCoord = null;
    return;
  }
  const tf = getMapTransform();
  const { cx, cy } = getCanvasPointerCoords(e);
  const pt = tf.toWorld(cx, cy);
  hoverWorldCoord = pt.inBounds ? pt : null;
  if (setPoseMode && poseDragStart) {
    poseDragCurrent = pt;
  }
});

canvas.addEventListener("mouseup", (e) => {
  if (!setPoseMode || !poseDragStart) return;
  const tf = getMapTransform();
  const { cx, cy } = getCanvasPointerCoords(e);
  const endPt = tf.toWorld(cx, cy);
  const dx = endPt.x - poseDragStart.x;
  const dy = endPt.y - poseDragStart.y;
  const yaw = Math.hypot(dx, dy) > 0.05 ? Math.atan2(dy, dx) : robotPose.yaw;

  calibrateRobotPoseOnMap(poseDragStart.x, poseDragStart.y, yaw);
  setPoseCalibrationMode(false);
  previewPlanIfPossible();
  showSuccess(`Robot pose calibrated to (${poseDragStart.x.toFixed(2)}m, ${poseDragStart.y.toFixed(2)}m, ${(yaw * 180 / Math.PI).toFixed(0)}°).`);
});

canvas.addEventListener("mouseleave", () => {
  hoverWorldCoord = null;
});

canvas.addEventListener("click", (e) => {
  if (setPoseMode || !pinModeEnabled) return;
  if (!hasValidMap()) {
    showError("No map loaded yet! Scan the maze or upload a saved map before placing waypoints.");
    return;
  }

  const tf = getMapTransform();
  const { cx, cy } = getCanvasPointerCoords(e);
  const pt = tf.toWorld(cx, cy);

  if (!pt.inBounds) {
    showError("Clicked outside the map boundaries. Click inside the mapped area.");
    return;
  }

  if (pt.occ !== null && pt.occ > 50) {
    showError("Cannot place a waypoint inside an obstacle wall! Click on open floor space.");
    return;
  }

  hideError();
  waypoints.push({ id: nextWaypointId++, x: pt.x, y: pt.y });
  targetIndex = waypoints.length - 1;
  renderWaypointList();
  previewPlanIfPossible();
  updateTeleopLockState();
});

// --- Upload Map (.yaml + .pgm) Parser & Loader ---
function parseMapYaml(yamlText) {
  const meta = {
    yamlFileName: "",
    image: "",
    mode: "trinary",
    resolution: 0.05,
    origin: [0, 0, 0],
    negate: 0,
    occupied_thresh: 0.65,
    free_thresh: 0.25
  };

  const lines = yamlText.split(/\r?\n/);
  for (const rawLine of lines) {
    const line = rawLine.split("#")[0].trim();
    if (!line || !line.includes(":")) continue;

    const colonIdx = line.indexOf(":");
    const key = line.slice(0, colonIdx).trim();
    const val = line.slice(colonIdx + 1).trim();

    if (key === "image") {
      const unquoted = val.replace(/^["']|["']$/g, "").trim();
      meta.image = unquoted.split(/[\\/]/).pop();
    } else if (key === "mode") {
      meta.mode = val.replace(/^["']|["']$/g, "").trim().toLowerCase();
    } else if (key === "resolution") {
      const num = parseFloat(val);
      if (isFinite(num) && num > 0) meta.resolution = num;
    } else if (key === "origin") {
      const match = val.match(/\[([^\]]+)\]/);
      if (match) {
        const nums = match[1].split(",").map((s) => parseFloat(s.trim()));
        if (nums.length >= 2 && nums.every((n) => isFinite(n))) {
          meta.origin = [nums[0], nums[1], isFinite(nums[2]) ? nums[2] : 0];
        }
      }
    } else if (key === "negate") {
      meta.negate = parseInt(val, 10) === 1 ? 1 : 0;
    } else if (key === "occupied_thresh") {
      const num = parseFloat(val);
      if (isFinite(num)) meta.occupied_thresh = num;
    } else if (key === "free_thresh") {
      const num = parseFloat(val);
      if (isFinite(num)) meta.free_thresh = num;
    }
  }

  return meta;
}

function parsePGM(arrayBuffer) {
  const bytes = new Uint8Array(arrayBuffer);
  const decoder = new TextDecoder("ascii");
  const isWs = (b) => b === 32 || b === 9 || b === 10 || b === 13;

  function skipWsAndComments(pos) {
    while (pos < bytes.length) {
      if (bytes[pos] === 35) {
        while (pos < bytes.length && bytes[pos] !== 10) pos++;
      } else if (isWs(bytes[pos])) {
        pos++;
      } else {
        break;
      }
    }
    return pos;
  }

  function nextToken(pos) {
    pos = skipWsAndComments(pos);
    const start = pos;
    while (pos < bytes.length && !isWs(bytes[pos]) && bytes[pos] !== 35) {
      pos++;
    }
    return { token: decoder.decode(bytes.subarray(start, pos)), nextPos: pos };
  }

  const t1 = nextToken(0);
  const t2 = nextToken(t1.nextPos);
  const t3 = nextToken(t2.nextPos);
  const t4 = nextToken(t3.nextPos);

  const magic = t1.token;
  const width = parseInt(t2.token, 10);
  const height = parseInt(t3.token, 10);
  const maxVal = parseInt(t4.token, 10);

  if ((magic !== "P5" && magic !== "P2") || !width || !height || !maxVal) {
    throw new Error("Invalid PGM header. Expected P5 or P2 PGM file.");
  }

  const totalPixels = width * height;
  const pixels = new Uint16Array(totalPixels);

  if (magic === "P5") {
    const dataStart = t4.nextPos + 1;
    const bytesPerSample = maxVal > 255 ? 2 : 1;
    if (dataStart + totalPixels * bytesPerSample > bytes.length) {
      throw new Error("PGM file is truncated or corrupted.");
    }
    if (bytesPerSample === 1) {
      for (let i = 0; i < totalPixels; i++) {
        pixels[i] = bytes[dataStart + i];
      }
    } else {
      for (let i = 0; i < totalPixels; i++) {
        const offset = dataStart + i * 2;
        pixels[i] = (bytes[offset] << 8) | bytes[offset + 1];
      }
    }
  } else {
    let pos = t4.nextPos;
    for (let i = 0; i < totalPixels; i++) {
      const tok = nextToken(pos);
      if (!tok.token) throw new Error("Unexpected end of ASCII PGM pixel data.");
      pixels[i] = parseInt(tok.token, 10);
      pos = tok.nextPos;
    }
  }

  return { width, height, maxVal, pixels };
}

function buildOccupancyGrid(meta, pgm) {
  const { width, height, maxVal, pixels } = pgm;
  const grid = new Int8Array(width * height);

  for (let imgY = 0; imgY < height; imgY++) {
    const gridY = height - 1 - imgY;
    for (let x = 0; x < width; x++) {
      const p = pixels[imgY * width + x];

      if (meta.mode === "raw") {
        grid[gridY * width + x] = p <= 100 ? p : -1;
        continue;
      }

      if (meta.mode === "trinary" && meta.negate === 0 && maxVal === 255 && p === 205) {
        grid[gridY * width + x] = -1;
        continue;
      }

      const occ = meta.negate === 1 ? p / maxVal : (maxVal - p) / maxVal;
      if (occ >= meta.occupied_thresh) {
        grid[gridY * width + x] = 100;
      } else if (occ <= meta.free_thresh) {
        grid[gridY * width + x] = 0;
      } else {
        grid[gridY * width + x] = -1;
      }
    }
  }

  const yaw = meta.origin[2] || 0;
  return {
    header: { frame_id: "map" },
    info: {
      resolution: meta.resolution,
      width,
      height,
      origin: {
        position: { x: meta.origin[0], y: meta.origin[1], z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
      }
    },
    data: Array.from(grid)
  };
}

async function handleUploadedFiles(fileList) {
  const files = Array.from(fileList || []);
  if (files.length === 0) return;

  const yamlFile = files.find((f) => /\.ya?ml$/i.test(f.name));
  const pgmFile = files.find((f) => /\.pgm$/i.test(f.name));

  try {
    if (yamlFile) {
      const yamlText = await yamlFile.text();
      const parsedMeta = parseMapYaml(yamlText);
      parsedMeta.yamlFileName = yamlFile.name;
      pendingYamlMeta = parsedMeta;
    }

    if (pendingYamlMeta && pgmFile) {
      const pgmBuffer = await pgmFile.arrayBuffer();
      const pgmData = parsePGM(pgmBuffer);
      uploadedMapData = buildOccupancyGrid(pendingYamlMeta, pgmData);

      const displayName = pendingYamlMeta.yamlFileName || pgmFile.name;
      setMapSourceMode(true, displayName);
      pendingYamlMeta = null;
      previewPlanIfPossible();

      showSuccess(
        `Uploaded "${displayName}" (${pgmData.width}×${pgmData.height} px). Place waypoints, set 🎯 Target, and click "Autopilot: ON".`
      );
      return;
    }

    if (yamlFile && !pgmFile) {
      const expectedPgm = pendingYamlMeta.image || yamlFile.name.replace(/\.ya?ml$/i, ".pgm");
      showError(
        `Loaded "${yamlFile.name}". Now select its matching image file "${expectedPgm}" (or select both .yaml and .pgm together).`,
        `Select ${expectedPgm}`,
        () => uploadMapInput.click()
      );
      return;
    }

    if (pgmFile && !pendingYamlMeta) {
      const pgmBuffer = await pgmFile.arrayBuffer();
      const pgmData = parsePGM(pgmBuffer);
      const fallbackMeta = {
        yamlFileName: pgmFile.name,
        image: pgmFile.name,
        mode: "trinary",
        resolution: 0.05,
        origin: [-(pgmData.width * 0.05) / 2, -(pgmData.height * 0.05) / 2, 0],
        negate: 0,
        occupied_thresh: 0.65,
        free_thresh: 0.25
      };
      uploadedMapData = buildOccupancyGrid(fallbackMeta, pgmData);
      setMapSourceMode(true, pgmFile.name);
      previewPlanIfPossible();
      showSuccess(`Loaded "${pgmFile.name}". Place waypoints and click "Autopilot: ON".`);
    }
  } catch (err) {
    showError(`Failed to load map: ${err.message}`);
  } finally {
    uploadMapInput.value = "";
  }
}

uploadMapBtn.onclick = () => uploadMapInput.click();
uploadMapInput.onchange = (e) => handleUploadedFiles(e.target.files);

// --- Save Scan Map Pop-up & ROS 2 YAML/PGM Export ---
function sanitizeMapName(raw) {
  const cleaned = raw.trim().replace(/\.ya?ml$/i, "").replace(/[^a-zA-Z0-9_-]+/g, "_").replace(/^_+|_+$/g, "");
  return cleaned || `maze_map_${saveCounter}`;
}

function updateFilePreview() {
  const name = sanitizeMapName(mapNameInput.value);
  filePreview.textContent = `Files: ${name}.yaml + ${name}.pgm`;
}

function openSaveMapModal() {
  if (!hasValidMap()) {
    showError("No scannable map available to save! Scan the maze first.");
    return;
  }
  hideError();
  mapNameInput.value = `maze_map_${saveCounter}`;
  updateFilePreview();
  saveMapModal.classList.add("open");
  setTimeout(() => {
    mapNameInput.focus();
    mapNameInput.select();
  }, 20);
}

function closeSaveMapModal() {
  saveMapModal.classList.remove("open");
}

function triggerDownload(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 15000);
}

function exportMapToPC(baseName) {
  if (!hasValidMap()) {
    showError("Map data is no longer available.");
    return;
  }

  const { width, height, resolution, origin } = mapData.info;
  const q = origin.orientation || { x: 0, y: 0, z: 0, w: 1 };
  const originYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

  const pgmFilename = `${baseName}.pgm`;
  const yamlFilename = `${baseName}.yaml`;

  const yamlContent = [
    `image: ${pgmFilename}`,
    `mode: trinary`,
    `resolution: ${Number(resolution).toFixed(6)}`,
    `origin: [${Number(origin.position.x).toFixed(6)}, ${Number(origin.position.y).toFixed(6)}, ${originYaw.toFixed(6)}]`,
    `negate: 0`,
    `occupied_thresh: 0.65`,
    `free_thresh: 0.25`,
    ``
  ].join("\n");

  const headerStr = `P5\n# CREATOR: TurtleBot3 Web GUI\n${width} ${height}\n255\n`;
  const headerBytes = new TextEncoder().encode(headerStr);
  const pixelBytes = new Uint8Array(width * height);

  for (let imgY = 0; imgY < height; imgY++) {
    const gridY = height - 1 - imgY;
    for (let x = 0; x < width; x++) {
      const val = mapData.data[gridY * width + x];
      let pixel = 205;
      if (val >= 0) {
        if (val >= 65) {
          pixel = 0;
        } else if (val <= 25) {
          pixel = 254;
        } else {
          pixel = Math.round(254 * (1 - val / 100));
        }
      }
      pixelBytes[imgY * width + x] = pixel;
    }
  }

  const yamlBlob = new Blob([yamlContent], { type: "text/yaml;charset=utf-8" });
  const pgmBlob = new Blob([headerBytes, pixelBytes], { type: "image/x-portable-graymap" });

  triggerDownload(yamlBlob, yamlFilename);
  triggerDownload(pgmBlob, pgmFilename);

  saveCounter += 1;
  closeSaveMapModal();
  showSuccess(
    `Saved "${yamlFilename}" + "${pgmFilename}" to Downloads.`,
    `⬇ Download ${pgmFilename}`,
    () => triggerDownload(pgmBlob, pgmFilename)
  );
}

saveMapBtn.onclick = openSaveMapModal;
cancelSaveBtn.onclick = closeSaveMapModal;
mapNameInput.oninput = updateFilePreview;

confirmSaveBtn.onclick = () => {
  const baseName = sanitizeMapName(mapNameInput.value);
  exportMapToPC(baseName);
};

mapNameInput.addEventListener("keydown", (e) => {
  if (e.key === "Enter") {
    e.preventDefault();
    confirmSaveBtn.click();
  } else if (e.key === "Escape") {
    e.preventDefault();
    closeSaveMapModal();
  }
});

saveMapModal.addEventListener("click", (e) => {
  if (e.target === saveMapModal) closeSaveMapModal();
});

// --- Built-In Web GUI Auto-Explore Controller (Autonomous SLAM Scanning) ---
function evaluateExploreLidar() {
  if (!scanData || !Array.isArray(scanData.ranges)) {
    return {
      hasScan: false,
      minFront: Infinity,
      minWideFront: Infinity,
      avgFrontLeft: 1.0,
      avgFrontRight: 1.0,
      avgLeft: 1.0,
      avgRight: 1.0,
      bestOpenAngle: 0
    };
  }

  let minFront = Infinity;
  let minWideFront = Infinity;
  let frontLeftSum = 0, frontLeftCnt = 0;
  let frontRightSum = 0, frontRightCnt = 0;
  let leftSum = 0, leftCnt = 0;
  let rightSum = 0, rightCnt = 0;

  // Score candidate headings from -110 deg to +110 deg in 10-deg bins
  const binCount = 23;
  const binScores = new Float32Array(binCount);
  const binCounts = new Uint16Array(binCount);

  for (let i = 0; i < scanData.ranges.length; i++) {
    const r = scanData.ranges[i];
    if (!isFinite(r) || r < scanData.range_min || r > scanData.range_max) continue;

    const angle = normalizeAngle(scanData.angle_min + i * scanData.angle_increment);
    const absAngle = Math.abs(angle);

    if (absAngle <= 0.48) { // +/- 27.5 deg
      if (r < minFront) minFront = r;
    }
    if (absAngle <= 0.78) { // +/- 45 deg
      if (r < minWideFront) minWideFront = r;
    }

    if (angle > 0.35 && angle <= 1.15) {
      frontLeftSum += Math.min(r, 1.8);
      frontLeftCnt++;
    } else if (angle < -0.35 && angle >= -1.15) {
      frontRightSum += Math.min(r, 1.8);
      frontRightCnt++;
    } else if (angle > 1.15 && angle <= 1.92) {
      leftSum += Math.min(r, 1.8);
      leftCnt++;
    } else if (angle < -1.15 && angle >= -1.92) {
      rightSum += Math.min(r, 1.8);
      rightCnt++;
    }

    if (absAngle <= 1.92) {
      const bin = Math.min(binCount - 1, Math.max(0, Math.round((angle + 1.92) / (3.84 / (binCount - 1)))));
      binScores[bin] += Math.min(r, 2.2);
      binCounts[bin]++;
    }
  }

  let bestScore = -1;
  let bestOpenAngle = 0;
  for (let b = 0; b < binCount; b++) {
    if (binCounts[b] === 0) continue;
    const binAngle = -1.92 + b * (3.84 / (binCount - 1));
    // Slight preference for forward/right-wall sweep to systematically explore maze loops
    const forwardBonus = 0.18 * Math.cos(binAngle) - 0.06 * Math.sin(binAngle);
    const score = binScores[b] / binCounts[b] + forwardBonus;
    if (score > bestScore) {
      bestScore = score;
      bestOpenAngle = binAngle;
    }
  }

  return {
    hasScan: true,
    minFront,
    minWideFront,
    avgFrontLeft: frontLeftCnt > 0 ? frontLeftSum / frontLeftCnt : 1.0,
    avgFrontRight: frontRightCnt > 0 ? frontRightSum / frontRightCnt : 1.0,
    avgLeft: leftCnt > 0 ? leftSum / leftCnt : 1.0,
    avgRight: rightCnt > 0 ? rightSum / rightCnt : 1.0,
    bestOpenAngle
  };
}

function findNextFrontierPath() {
  if (!hasValidMap()) return { status: "no_map", path: [] };
  const tf = getMapTransform();
  const { width, height, resolution } = tf;
  const costGrid = buildInflatedCostGrid(tf);
  const now = Date.now();
  unreachableFrontiers = unreachableFrontiers.filter((f) => f.until > now);

  const candidates = [];
  const stride = resolution < 0.03 ? 3 : 2;

  for (let y = 1; y < height - 1; y += stride) {
    for (let x = 1; x < width - 1; x += stride) {
      const idx = y * width + x;
      if (mapData.data[idx] !== 0) continue;
      if (!isFinite(costGrid[idx])) continue;

      // Check if adjacent to unexplored (-1) space
      const hasUnknownNeighbor =
        mapData.data[idx - 1] === -1 ||
        mapData.data[idx + 1] === -1 ||
        mapData.data[idx - width] === -1 ||
        mapData.data[idx + width] === -1;

      if (!hasUnknownNeighbor) continue;

      const worldPt = tf.gridToWorld(x, y);
      const dist = Math.hypot(worldPt.x - robotPose.x, worldPt.y - robotPose.y);
      if (dist < 0.28) continue;

      const recentlyFailed = unreachableFrontiers.some(
        (f) => Math.hypot(f.x - worldPt.x, f.y - worldPt.y) < 0.25
      );
      if (recentlyFailed) continue;

      candidates.push({ x: worldPt.x, y: worldPt.y, dist });
    }
  }

  if (candidates.length === 0) {
    return { status: "complete", path: [] };
  }

  // Nearest-frontier first so the robot clears local corridors systematically
  candidates.sort((a, b) => a.dist - b.dist);

  const maxTests = Math.min(8, candidates.length);
  for (let i = 0; i < maxTests; i++) {
    const goal = candidates[i];
    const seg = runAStarSegment(robotPose, goal, tf, costGrid);
    if (seg && seg.length > 1) {
      return { status: "ok", path: smoothPath(seg) };
    } else {
      unreachableFrontiers.push({ x: goal.x, y: goal.y, until: now + 15000 });
    }
  }

  return { status: "local_fallback", path: [] };
}

function stepAutoExploreControl() {
  if (!robotActive || !autoExploreActive) return;

  const lidar = evaluateExploreLidar();
  if (!lidar.hasScan) {
    autopilotStatusBox.textContent = "Auto-Explore: Waiting for /scan LiDAR data...";
    return;
  }

  exploreStepCount += 1;
  const now = Date.now();
  const maxLin = Math.min(parseFloat(linSlider.value), 0.12); // Cap speed to prevent SLAM slip
  const maxAng = Math.min(parseFloat(angSlider.value), 0.55);

  // 1. Immediate LiDAR Collision Shield (pivots cleanly away from close walls)
  if (lidar.minFront < 0.22 || (now < explorePivotUntil && lidar.minFront < 0.34)) {
    if (now >= explorePivotUntil) {
      const leftClear = lidar.avgFrontLeft + lidar.avgLeft;
      const rightClear = lidar.avgFrontRight + lidar.avgRight;
      explorePivotDir = leftClear >= rightClear ? 1 : -1;
      explorePivotUntil = now + 500;
      explorePath = [];
    }
    publishVelocity(0.0, maxAng * explorePivotDir);
    autopilotStatusBox.textContent = `Auto-Explore: Pivoting clear (${lidar.minFront.toFixed(2)}m ahead)`;
    return;
  }

  // 2. Frontier-Based Exploration when /map is available
  if (hasValidMap()) {
    const needReplan =
      explorePath.length === 0 ||
      now - lastExplorePlanTime > 2400 ||
      Math.hypot(
        explorePath[explorePath.length - 1].x - robotPose.x,
        explorePath[explorePath.length - 1].y - robotPose.y
      ) < 0.16;

    if (needReplan) {
      const res = findNextFrontierPath();
      lastExplorePlanTime = now;
      if (res.status === "complete" && exploreStepCount > 45) {
        publishVelocity(0, 0);
        autopilotStatusBox.textContent = "Auto-Explore: Maze scan complete!";
        setAutoExploreState(false);
        showSuccess("Maze scan complete! Click 'Save Scan Map' to export your .yaml and .pgm files.");
        return;
      }
      explorePath = res.path;
    }

    if (explorePath.length > 0) {
      while (explorePath.length > 1) {
        const d0 = Math.hypot(explorePath[0].x - robotPose.x, explorePath[0].y - robotPose.y);
        if (d0 < 0.13) explorePath.shift();
        else break;
      }

      let lookahead = explorePath[explorePath.length - 1];
      for (let i = 0; i < explorePath.length; i++) {
        if (Math.hypot(explorePath[i].x - robotPose.x, explorePath[i].y - robotPose.y) >= 0.24) {
          lookahead = explorePath[i];
          break;
        }
      }

      const desiredHeading = Math.atan2(lookahead.y - robotPose.y, lookahead.x - robotPose.x);
      const headingError = normalizeAngle(desiredHeading - robotPose.yaw);

      if (Math.abs(headingError) > 0.60) {
        const wz = Math.sign(headingError) * Math.min(maxAng, Math.max(0.28, Math.abs(1.6 * headingError)));
        publishVelocity(0.0, wz);
      } else {
        const turnFactor = Math.max(0.30, 1.0 - Math.abs(headingError) / 0.65);
        const cautionFactor = lidar.minWideFront < 0.30 ? 0.55 : 1.0;
        let wz = Math.max(-maxAng, Math.min(maxAng, 1.6 * headingError));
        if (lidar.minWideFront < 0.30) {
          wz += 0.22 * (lidar.avgFrontLeft >= lidar.avgFrontRight ? 1 : -1);
          wz = Math.max(-maxAng, Math.min(maxAng, wz));
        }
        publishVelocity(maxLin * turnFactor * cautionFactor, wz);
      }

      const goalPt = explorePath[explorePath.length - 1];
      autopilotStatusBox.textContent = `Auto-Explore → Frontier (${goalPt.x.toFixed(2)}m, ${goalPt.y.toFixed(2)}m)`;
      return;
    }
  }

  // 3. Fallback LiDAR Corridor Sweep (while waiting for initial /map updates)
  const steerAngle = lidar.bestOpenAngle;
  if (Math.abs(steerAngle) > 0.65) {
    publishVelocity(0.0, Math.sign(steerAngle) * Math.min(maxAng, 0.45));
  } else {
    const vx = Math.min(maxLin, 0.11) * Math.max(0.35, 1.0 - Math.abs(steerAngle) / 0.70);
    const wz = Math.max(-maxAng, Math.min(maxAng, 1.35 * steerAngle));
    publishVelocity(vx, wz);
  }
  autopilotStatusBox.textContent = `Auto-Explore: LiDAR corridor scan (front: ${lidar.minFront.toFixed(2)}m)`;
}

// --- Autonomous Navigation Controller (Uploaded Map Autopilot) ---
function publishNav2Goal(wp) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const headingToGoal = Math.atan2(wp.y - robotPose.y, wp.x - robotPose.x);
  ws.send(JSON.stringify({
    op: "publish",
    topic: "/goal_pose",
    msg: {
      header: { frame_id: "map" },
      pose: {
        position: { x: wp.x, y: wp.y, z: 0.0 },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: Math.sin(headingToGoal / 2),
          w: Math.cos(headingToGoal / 2)
        }
      }
    }
  }));
}

function evaluateLidarSafety() {
  if (!scanData || !Array.isArray(scanData.ranges)) {
    return { blocked: false, caution: false, minFront: Infinity, steerBias: 0 };
  }

  let minFront = Infinity;
  let leftClearance = 0;
  let rightClearance = 0;
  let leftCount = 0;
  let rightCount = 0;

  for (let i = 0; i < scanData.ranges.length; i++) {
    const r = scanData.ranges[i];
    if (!isFinite(r) || r < scanData.range_min || r > scanData.range_max) continue;

    const angle = normalizeAngle(scanData.angle_min + i * scanData.angle_increment);
    const absAngle = Math.abs(angle);

    if (absAngle <= 0.56) {
      if (r < minFront) minFront = r;
    }
    if (angle > 0.26 && angle < 1.31) {
      leftClearance += Math.min(r, 1.5);
      leftCount++;
    } else if (angle < -0.26 && angle > -1.31) {
      rightClearance += Math.min(r, 1.5);
      rightCount++;
    }
  }

  const avgLeft = leftCount > 0 ? leftClearance / leftCount : 1.0;
  const avgRight = rightCount > 0 ? rightClearance / rightCount : 1.0;
  const steerBias = avgLeft >= avgRight ? 1 : -1;

  return {
    blocked: minFront < 0.17,
    caution: minFront < 0.28,
    minFront,
    steerBias
  };
}

function stepAutopilotNavigation() {
  if (!robotActive || !autopilotActive) return;

  if (!usingUploadedMap || !uploadedMapData) {
    setAutopilotState(false);
    return;
  }

  const plan = getActiveRouteAndTarget();
  if (!plan) {
    setAutopilotState(false);
    return;
  }

  const distToFinalTarget = Math.hypot(plan.target.x - robotPose.x, plan.target.y - robotPose.y);
  const GOAL_TOLERANCE = 0.10;

  if (distToFinalTarget <= GOAL_TOLERANCE) {
    publishVelocity(0, 0);
    autopilotStatusBox.textContent = `Status: Arrived at Target (${plan.target.x.toFixed(2)}m, ${plan.target.y.toFixed(2)}m)!`;
    setAutopilotState(false);
    showSuccess(`Target end point reached (${plan.target.x.toFixed(2)}m, ${plan.target.y.toFixed(2)}m)!`);
    return;
  }

  if (navEngineSelect.value === "nav2") {
    const activeGoal = plan.route[nav2ActiveLegIndex] || plan.target;
    const legDist = Math.hypot(activeGoal.x - robotPose.x, activeGoal.y - robotPose.y);
    if (legDist <= 0.15 && nav2ActiveLegIndex < plan.route.length - 1) {
      nav2ActiveLegIndex += 1;
      publishNav2Goal(plan.route[nav2ActiveLegIndex]);
    }
    autopilotStatusBox.textContent = `Nav2 Goal ${nav2ActiveLegIndex + 1}/${plan.route.length} | Dist to Target: ${distToFinalTarget.toFixed(2)}m`;
    return;
  }

  const now = Date.now();
  if (plannedPath.length === 0 || now - lastReplanTime > 2500) {
    const updated = computeFullPlannedPath(plan);
    if (updated && updated.length > 0) {
      plannedPath = updated;
      lastReplanTime = now;
    } else if (plannedPath.length === 0) {
      publishVelocity(0, 0);
      autopilotStatusBox.textContent = "Status: No valid path to target (blocked by walls)!";
      showError("A* planner could not find an unobstructed corridor path to the target point.");
      setAutopilotState(false);
      return;
    }
  }

  while (plannedPath.length > 1) {
    const d0 = Math.hypot(plannedPath[0].x - robotPose.x, plannedPath[0].y - robotPose.y);
    if (d0 < 0.12) {
      plannedPath.shift();
    } else {
      break;
    }
  }

  const LOOKAHEAD_DIST = 0.24;
  let lookahead = plannedPath[plannedPath.length - 1];
  for (let i = 0; i < plannedPath.length; i++) {
    const d = Math.hypot(plannedPath[i].x - robotPose.x, plannedPath[i].y - robotPose.y);
    if (d >= LOOKAHEAD_DIST) {
      lookahead = plannedPath[i];
      break;
    }
  }

  const safety = evaluateLidarSafety();
  const maxLin = parseFloat(linSlider.value);
  const maxAng = parseFloat(angSlider.value);

  if (safety.blocked) {
    if (now > avoidanceHoldUntil) {
      avoidanceTurnDir = safety.steerBias;
      avoidanceHoldUntil = now + 450;
    }
    const turnSpeed = Math.min(maxAng, 0.55) * avoidanceTurnDir;
    publishVelocity(0.0, turnSpeed);
    autopilotStatusBox.textContent = `Obstacle ${safety.minFront.toFixed(2)}m ahead — pivoting clear | Target: ${distToFinalTarget.toFixed(2)}m`;
    return;
  }

  const desiredHeading = Math.atan2(lookahead.y - robotPose.y, lookahead.x - robotPose.x);
  const headingError = normalizeAngle(desiredHeading - robotPose.yaw);

  let vx = 0;
  let wz = Math.max(-maxAng, Math.min(maxAng, 1.8 * headingError));

  if (Math.abs(headingError) > 0.62) {
    vx = 0.0;
    wz = Math.sign(headingError) * Math.min(maxAng, Math.max(0.28, Math.abs(wz)));
  } else {
    const turnFactor = Math.max(0.25, 1.0 - Math.abs(headingError) / 0.65);
    const arrivalFactor = distToFinalTarget < 0.35 ? Math.max(0.35, distToFinalTarget / 0.35) : 1.0;
    const lidarFactor = safety.caution ? 0.5 : 1.0;
    vx = maxLin * turnFactor * arrivalFactor * lidarFactor;

    if (safety.caution) {
      wz += 0.25 * safety.steerBias;
      wz = Math.max(-maxAng, Math.min(maxAng, wz));
    }
  }

  publishVelocity(vx, wz);
  autopilotStatusBox.textContent = `Autopilot → Target (${plan.target.x.toFixed(2)}m, ${plan.target.y.toFixed(2)}m) | Rem: ${distToFinalTarget.toFixed(2)}m`;
}

// 10 Hz Control Loop (runs either Auto-Explore Scan OR Uploaded-Map Autopilot)
setInterval(() => {
  if (!robotActive) return;
  if (autoExploreActive) {
    stepAutoExploreControl();
  } else if (autopilotActive) {
    stepAutopilotNavigation();
  }
}, 100);

// --- Teleop Lock, Scan Mode & Autopilot Button States ---
function updateTeleopLockState() {
  const locked = !robotActive || autoExploreActive || autopilotActive;
  dpadContainer.classList.toggle("disabled", locked);

  if (!robotActive) {
    teleopNote.textContent = "Standby active — motors immobilized";
  } else if (autoExploreActive) {
    teleopNote.textContent = "Auto-Explore active — scanning maze autonomously";
  } else if (autopilotActive) {
    const plan = getActiveRouteAndTarget();
    teleopNote.textContent = plan
      ? `Autopilot → Target (${plan.target.x.toFixed(2)}m, ${plan.target.y.toFixed(2)}m)`
      : "Autopilot active — manual keyboard locked";
  } else {
    teleopNote.textContent = "";
  }
}

function clearPressedKeys() {
  activeKeys.clear();
  Object.values(keyMap).forEach((id) => document.getElementById(id).classList.remove("active"));
  publishVelocity(0, 0);
}

function setAutoExploreState(enable) {
  if (enable) {
    if (!robotActive) {
      showError("Cannot start Auto-Explore while Standby is OFF. Switch Standby to ACTIVE first.");
      return;
    }
    if (autopilotActive) {
      setAutopilotState(false);
    }
    if (usingUploadedMap) {
      setMapSourceMode(false);
    }

    hideError();
    autoExploreActive = true;
    explorePath = [];
    unreachableFrontiers = [];
    exploreStepCount = 0;
    lastExplorePlanTime = 0;
    clearPressedKeys();
    syncModeButtonsUI();
    updateTeleopLockState();
    autopilotStatusBox.textContent = "Auto-Explore: Starting autonomous maze scan...";
  } else {
    autoExploreActive = false;
    explorePath = [];
    publishVelocity(0, 0);
    syncModeButtonsUI();
    updateTeleopLockState();
    if (!autopilotActive && !autopilotStatusBox.textContent.includes("complete")) {
      autopilotStatusBox.textContent = "Status: Idle (Manual Scan Mode)";
    }
  }
}

scanModeBtn.onclick = () => setAutoExploreState(!autoExploreActive);

function setAutopilotState(enable) {
  if (enable) {
    if (!robotActive) {
      showError("Cannot engage Autopilot while Standby is OFF. Switch Standby to ACTIVE first.");
      return;
    }
    if (!usingUploadedMap || !uploadedMapData) {
      showError(
        "Autopilot is only for autonomous navigation on an uploaded map! Save your scanned map and click 'Upload Map' first.",
        "📂 Upload Map",
        () => uploadMapInput.click()
      );
      return;
    }
    const plan = getActiveRouteAndTarget();
    if (!plan) {
      showError("No targeted end point selected! Click on the uploaded map to place waypoints and select 🎯 Target first.");
      return;
    }

    if (autoExploreActive) {
      setAutoExploreState(false);
    }

    if (navEngineSelect.value === "builtin") {
      const path = computeFullPlannedPath(plan);
      if (!path || path.length === 0) {
        showError("Cannot plan a valid corridor path to the target! Check that the robot pose and waypoints are on open floor.");
        return;
      }
      plannedPath = path;
      lastReplanTime = Date.now();
    } else {
      nav2ActiveLegIndex = 0;
      publishNav2Goal(plan.route[0]);
    }

    hideError();
    autopilotActive = true;
    clearPressedKeys();
    syncModeButtonsUI();
    updateTeleopLockState();
  } else {
    autopilotActive = false;
    publishVelocity(0, 0);
    syncModeButtonsUI();
    updateTeleopLockState();
    if (!autoExploreActive && !autopilotStatusBox.textContent.startsWith("Status: Arrived")) {
      autopilotStatusBox.textContent = "Status: Idle (Autopilot OFF)";
    }
  }
}

autopilotBtn.onclick = () => setAutopilotState(!autopilotActive);

function updateBatteryDisplay(msg) {
  const voltage = Number(msg.voltage);
  const rawPct = Number(msg.percentage);
  let pct = null;

  if (isFinite(voltage) && voltage > 1.0) {
    pct = Math.min(100, Math.max(0, Math.round(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100)));
    batteryPctText.textContent = `${pct}% (${voltage.toFixed(2)}V)`;
    if (voltage <= V_MIN) {
      batteryTimeText.textContent = "Est. time: 0m (Cutoff!)";
    } else {
      const hoursLeft = (voltage - V_MIN) / DISCHARGE_RATE;
      const hrs = Math.floor(hoursLeft);
      const mins = Math.round((hoursLeft - hrs) * 60);
      batteryTimeText.textContent = hrs > 0 ? `Est. time: ~${hrs}h ${mins}m` : `Est. time: ~${mins}m`;
    }
  } else if (isFinite(rawPct) && rawPct > 0) {
    pct = rawPct <= 1.0 ? Math.round(rawPct * 100) : Math.round(rawPct);
    batteryPctText.textContent = `${pct}%`;
    batteryTimeText.textContent = "Est. time: N/A (No voltage)";
  }

  if (pct !== null) {
    batteryFill.style.width = `${pct}%`;
    batteryFill.style.background = pct > 40 ? "#43a047" : pct > 20 ? "#fb8c00" : "#e53935";
  }
}

function subscribeSensorTopics() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({
    op: "subscribe",
    topic: "/scan",
    type: "sensor_msgs/msg/LaserScan",
    throttle_rate: 100
  }));
  ws.send(JSON.stringify({
    op: "subscribe",
    topic: "/odom",
    type: "nav_msgs/msg/Odometry",
    throttle_rate: 80
  }));
  ws.send(JSON.stringify({
    op: "subscribe",
    topic: "/amcl_pose",
    type: "geometry_msgs/msg/PoseWithCovarianceStamped",
    throttle_rate: 100
  }));
  ws.send(JSON.stringify({
    op: "subscribe",
    topic: "/plan",
    type: "nav_msgs/msg/Path",
    throttle_rate: 250
  }));
}

function unsubscribeSensorTopics() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ op: "unsubscribe", topic: "/scan" }));
  ws.send(JSON.stringify({ op: "unsubscribe", topic: "/odom" }));
  ws.send(JSON.stringify({ op: "unsubscribe", topic: "/amcl_pose" }));
  ws.send(JSON.stringify({ op: "unsubscribe", topic: "/plan" }));
  scanData = null;
}

function callSetBoolService(serviceName, state) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({
    op: "call_service",
    service: serviceName,
    type: "std_srvs/srv/SetBool",
    args: { data: state }
  }));
}

function setRobotActiveState(isActive) {
  robotActive = isActive;
  powerSwitch.checked = isActive;
  powerStateLabel.textContent = isActive ? "ACTIVE" : "OFF (STANDBY)";
  powerStateLabel.style.color = isActive ? "#66bb6a" : "#ef5350";

  if (!isActive) {
    if (autoExploreActive) setAutoExploreState(false);
    if (autopilotActive) setAutopilotState(false);
    clearPressedKeys();
    setTimeout(() => publishVelocity(0, 0), 100);
    callSetBoolService("/motor_power", false);
    callSetBoolService("/lidar_power", false);
    unsubscribeSensorTopics();
  } else {
    callSetBoolService("/motor_power", true);
    callSetBoolService("/lidar_power", true);
    subscribeSensorTopics();
  }

  updateTeleopLockState();
}

powerSwitch.onchange = () => setRobotActiveState(powerSwitch.checked);

function connect() {
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    setConnectedState(true, `Connected (${window.location.hostname})`);
    advertiseCmdVel();

    ws.send(JSON.stringify({
      op: "subscribe",
      topic: "/map",
      type: "nav_msgs/msg/OccupancyGrid",
      throttle_rate: 1000,
      qos: { reliability: "reliable", durability: "transient_local" }
    }));

    ws.send(JSON.stringify({
      op: "subscribe",
      topic: "/battery_state",
      type: "sensor_msgs/msg/BatteryState",
      throttle_rate: 2000,
      qos: { reliability: "best_effort", durability: "volatile" }
    }));

    if (robotActive) {
      subscribeSensorTopics();
    }
  };

  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.topic === "/battery_state") {
      updateBatteryDisplay(data.msg);
      lastHeartbeatTime = Date.now();
    } else if (data.topic === "/map") {
      liveSlamMapData = data.msg;
      if (!usingUploadedMap) {
        mapData = liveSlamMapData;
        if (!autopilotActive) previewPlanIfPossible();
      }
      lastHeartbeatTime = Date.now();
    } else if (data.topic === "/scan" && robotActive) {
      scanData = data.msg;
      lastHeartbeatTime = Date.now();
    } else if (data.topic === "/amcl_pose" && robotActive) {
      const p = data.msg.pose.pose.position;
      const q = data.msg.pose.pose.orientation;
      robotPose.x = p.x;
      robotPose.y = p.y;
      robotPose.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      hasExternalMapPose = true;
      lastHeartbeatTime = Date.now();
    } else if (data.topic === "/odom" && robotActive) {
      const p = data.msg.pose.pose.position;
      const q = data.msg.pose.pose.orientation;
      rawOdomPose.x = p.x;
      rawOdomPose.y = p.y;
      rawOdomPose.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      updateRobotPoseFromOdom();
      lastHeartbeatTime = Date.now();
    } else if (data.topic === "/plan" && Array.isArray(data.msg.poses)) {
      nav2GlobalPath = data.msg.poses.map((ps) => ({
        x: ps.pose.position.x,
        y: ps.pose.position.y
      }));
    }
  };

  ws.onclose = () => {
    setConnectedState(false, "Disconnected (Retrying...)");
    setTimeout(connect, 2000);
  };

  ws.onerror = () => ws.close();
}

function advertiseCmdVel() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ op: "unadvertise", topic: "/cmd_vel" }));
  const fullType = msgTypeSelect.value === "TwistStamped"
    ? "geometry_msgs/msg/TwistStamped"
    : "geometry_msgs/msg/Twist";
  ws.send(JSON.stringify({ op: "advertise", topic: "/cmd_vel", type: fullType }));
}
msgTypeSelect.onchange = advertiseCmdVel;

// Watchdog
setInterval(() => {
  if (ws && ws.readyState === WebSocket.OPEN && lastHeartbeatTime > 0) {
    const stale = Date.now() - lastHeartbeatTime > 3000;
    const statusMsg = stale
      ? "Bridge Up — No Robot Data"
      : !robotActive
        ? `Standby (${window.location.hostname})`
        : autoExploreActive
          ? `Auto-Explore (${window.location.hostname})`
          : autopilotActive
            ? `Autopilot (${window.location.hostname})`
            : `Connected (${window.location.hostname})`;
    setConnectedState(!stale, statusMsg);
  }
}, 500);

// --- Keyboard Teleop & E-Stop ---
const activeKeys = new Set();
const keyMap = {
  ArrowUp: "key-up", KeyW: "key-up",
  ArrowDown: "key-down", KeyS: "key-down",
  ArrowLeft: "key-left", KeyA: "key-left",
  ArrowRight: "key-right", KeyD: "key-right"
};

window.addEventListener("keydown", (e) => {
  if (saveMapModal.classList.contains("open")) return;

  // Spacebar Emergency Stop for both Auto-Explore and Autopilot
  if (e.code === "Space" && (autoExploreActive || autopilotActive)) {
    e.preventDefault();
    if (autoExploreActive) setAutoExploreState(false);
    if (autopilotActive) setAutopilotState(false);
    showError("Emergency Stop (Spacebar) — Autonomous motion halted.");
    return;
  }

  if (!robotActive || autoExploreActive || autopilotActive) return;
  if (keyMap[e.code]) {
    e.preventDefault();
    activeKeys.add(keyMap[e.code]);
    document.getElementById(keyMap[e.code]).classList.add("active");
  }
});

window.addEventListener("keyup", (e) => {
  if (keyMap[e.code]) {
    activeKeys.delete(keyMap[e.code]);
    document.getElementById(keyMap[e.code]).classList.remove("active");
    if (activeKeys.size === 0 && !autoExploreActive && !autopilotActive) publishVelocity(0, 0);
  }
});

function publishVelocity(linearX, angularZ) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const twist = {
    linear: { x: linearX, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angularZ }
  };
  const msg = msgTypeSelect.value === "TwistStamped"
    ? { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "base_link" }, twist }
    : twist;

  ws.send(JSON.stringify({ op: "publish", topic: "/cmd_vel", msg }));
}

setInterval(() => {
  if (!robotActive || autoExploreActive || autopilotActive || activeKeys.size === 0) return;
  const lin = parseFloat(linSlider.value);
  const ang = parseFloat(angSlider.value);
  let vx = 0, wz = 0;
  if (activeKeys.has("key-up")) vx += lin;
  if (activeKeys.has("key-down")) vx -= lin;
  if (activeKeys.has("key-left")) wz += ang;
  if (activeKeys.has("key-right")) wz -= ang;
  publishVelocity(vx, wz);
}, 100);

// --- Real-Time Map, LiDAR, Exploration Path & Waypoint Renderer ---
function renderLoop() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const tf = getMapTransform();
  if (tf) {
    const { width, height, scale, offsetX, offsetY, toCanvasX, toCanvasY } = tf;

    if (mapData && Array.isArray(mapData.data)) {
      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const val = mapData.data[y * width + x];
          if (val === -1) continue;
          ctx.fillStyle = val > 50 ? "#000000" : "#e8e8e8";
          ctx.fillRect(offsetX + x * scale, offsetY + (height - 1 - y) * scale, Math.ceil(scale), Math.ceil(scale));
        }
      }
    }

    if (scanData && robotActive) {
      ctx.fillStyle = "#ff1744";
      for (let i = 0; i < scanData.ranges.length; i++) {
        const r = scanData.ranges[i];
        if (r < scanData.range_min || r > scanData.range_max || !isFinite(r)) continue;
        const theta = scanData.angle_min + i * scanData.angle_increment;
        const wx = robotPose.x + r * Math.cos(robotPose.yaw + theta);
        const wy = robotPose.y + r * Math.sin(robotPose.yaw + theta);
        ctx.fillRect(toCanvasX(wx) - 1.5, toCanvasY(wy) - 1.5, 3, 3);
      }
    }

    // Draw Auto-Explore frontier path (cyan) when scanning autonomously
    if (autoExploreActive && explorePath.length > 1) {
      ctx.save();
      ctx.strokeStyle = "#00e5ff";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(toCanvasX(explorePath[0].x), toCanvasY(explorePath[0].y));
      for (let i = 1; i < explorePath.length; i++) {
        ctx.lineTo(toCanvasX(explorePath[i].x), toCanvasY(explorePath[i].y));
      }
      ctx.stroke();
      ctx.restore();

      const fGoal = explorePath[explorePath.length - 1];
      ctx.fillStyle = "#00e5ff";
      ctx.beginPath();
      ctx.arc(toCanvasX(fGoal.x), toCanvasY(fGoal.y), 5, 0, Math.PI * 2);
      ctx.fill();
    }

    // Draw straight waypoint guide segments (dashed light blue)
    if (waypoints.length > 0) {
      ctx.save();
      ctx.strokeStyle = "rgba(41, 182, 246, 0.55)";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(toCanvasX(robotPose.x), toCanvasY(robotPose.y));

      const activeEnd = targetIndex >= 0 ? targetIndex : waypoints.length - 1;
      for (let i = 0; i <= activeEnd; i++) {
        ctx.lineTo(toCanvasX(waypoints[i].x), toCanvasY(waypoints[i].y));
      }
      ctx.stroke();
      ctx.restore();
    }

    // Draw computed obstacle-free A* / Nav2 corridor path (solid bright green)
    const activePath = navEngineSelect.value === "nav2" ? nav2GlobalPath : plannedPath;
    if (activePath.length > 1) {
      ctx.save();
      ctx.strokeStyle = "#00e676";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(toCanvasX(activePath[0].x), toCanvasY(activePath[0].y));
      for (let i = 1; i < activePath.length; i++) {
        ctx.lineTo(toCanvasX(activePath[i].x), toCanvasY(activePath[i].y));
      }
      ctx.stroke();
      ctx.restore();
    }

    // Draw waypoint markers
    if (waypoints.length > 0) {
      waypoints.forEach((wp, idx) => {
        const wx = toCanvasX(wp.x);
        const wy = toCanvasY(wp.y);
        const isTarget = idx === targetIndex;

        if (isTarget) {
          ctx.strokeStyle = "#ff4081";
          ctx.lineWidth = 2.5;
          ctx.beginPath();
          ctx.arc(wx, wy, 12, 0, Math.PI * 2);
          ctx.stroke();
        }

        ctx.fillStyle = isTarget ? "#ff4081" : "#ffb300";
        ctx.beginPath();
        ctx.arc(wx, wy, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = isTarget ? "#ffffff" : "#000000";
        ctx.font = "bold 10px system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(idx + 1), wx, wy);
      });
    }

    // Draw pose calibration drag arrow if active
    if (setPoseMode && poseDragStart && poseDragCurrent) {
      const sx = toCanvasX(poseDragStart.x);
      const sy = toCanvasY(poseDragStart.y);
      const ex = toCanvasX(poseDragCurrent.x);
      const ey = toCanvasY(poseDragCurrent.y);
      ctx.strokeStyle = "#ce93d8";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
    }

    // Draw robot pose dot + heading indicator
    const rx = toCanvasX(robotPose.x);
    const ry = toCanvasY(robotPose.y);
    ctx.fillStyle = !robotActive
      ? "#ffa726"
      : autoExploreActive
        ? "#00e5ff"
        : autopilotActive
          ? "#29b6f6"
          : "#00e676";
    ctx.beginPath();
    ctx.arc(rx, ry, 5.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + Math.cos(robotPose.yaw) * 12, ry - Math.sin(robotPose.yaw) * 12);
    ctx.stroke();
  }

  // Hover coordinate badge
  if (hoverWorldCoord && (pinModeEnabled || setPoseMode)) {
    const prefix = setPoseMode ? "Set Pose" : "Cursor";
    const label = `${prefix}: (${hoverWorldCoord.x.toFixed(2)}m, ${hoverWorldCoord.y.toFixed(2)}m)`;
    ctx.fillStyle = "rgba(18, 18, 18, 0.82)";
    ctx.fillRect(10, canvas.height - 32, 220, 22);
    ctx.fillStyle = "#b0bec5";
    ctx.font = "11px monospace";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(label, 18, canvas.height - 21);
  }

  if (autoExploreActive && robotActive) {
    ctx.fillStyle = "rgba(0, 105, 92, 0.90)";
    ctx.fillRect(10, 10, 185, 28);
    ctx.fillStyle = "#e0f2f1";
    ctx.font = "bold 12px system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText("SCAN: AUTO-EXPLORE", 20, 24);
  } else if (autopilotActive && robotActive) {
    ctx.fillStyle = "rgba(2, 136, 209, 0.88)";
    ctx.fillRect(10, 10, 165, 28);
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 12px system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText("MODE: AUTOPILOT", 20, 24);
  }

  if (!robotActive) {
    ctx.fillStyle = "rgba(0, 0, 0, 0.55)";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#ffb74d";
    ctx.font = "bold 18px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("STANDBY — SENSORS & MOTORS OFF", canvas.width / 2, canvas.height / 2);
  }

  requestAnimationFrame(renderLoop);
}

syncModeButtonsUI();
connect();
requestAnimationFrame(renderLoop);