const state = {
  datasetPath: "/out/autofused_1970_2090_v3_replay",
  dataset: null,
  index: 0,
  debug: null,
  mode: "vcs",
  views: {
    vcs: { forward: 50, lateral: 0, scale: 6 },
    frenet: { forward: 50, lateral: 0, scale: 6 },
    smooth: { forward: 0, lateral: 0, scale: 2 },
  },
  dragging: false,
  lastX: 0,
  lastY: 0,
  dynamicLayerState: {},
};

const CHECKBOX_STORAGE_KEY = "topologyV3Viewer.checkboxes";

const el = {
  path: document.querySelector("#datasetPath"),
  load: document.querySelector("#load"),
  slider: document.querySelector("#slider"),
  previous: document.querySelector("#previous"),
  next: document.querySelector("#next"),
  jump: document.querySelector("#jump"),
  jumpButton: document.querySelector("#jumpButton"),
  fit: document.querySelector("#fit"),
  modeVcs: document.querySelector("#modeVcs"),
  modeFrenet: document.querySelector("#modeFrenet"),
  modeSmooth: document.querySelector("#modeSmooth"),
  visualLanes: document.querySelector("#showVisualLanes"),
  visualReference: document.querySelector("#showVisualReference"),
  navigationReference: document.querySelector("#showNavigationReference"),
  dynamicLayers: document.querySelector("#dynamicLayers"),
  labels: document.querySelector("#showLabels"),
  points: document.querySelector("#showPoints"),
  grid: document.querySelector("#showGrid"),
  autoFit: document.querySelector("#showAutoFit"),
  datasetMeta: document.querySelector("#datasetMeta"),
  frameMeta: document.querySelector("#frameMeta"),
  info: document.querySelector("#info"),
  canvas: document.querySelector("#canvas"),
};

const ctx = el.canvas.getContext("2d");
const staticCheckboxes = [
  el.visualLanes,
  el.visualReference,
  el.navigationReference,
  el.labels,
  el.points,
  el.grid,
  el.autoFit,
];

function restoreViewerState() {
  try {
    const saved = JSON.parse(localStorage.getItem(CHECKBOX_STORAGE_KEY) || "null");
    if (!saved || typeof saved !== "object") return;
    staticCheckboxes.forEach((checkbox) => {
      if (Object.hasOwn(saved, checkbox.id)) checkbox.checked = Boolean(saved[checkbox.id]);
    });
    state.dynamicLayerState = saved.dynamicLayerState || {};
    if (saved.mode === "vcs" || saved.mode === "frenet" || saved.mode === "smooth") {
      state.mode = saved.mode;
    } else if (saved.mode === "gcs") {
      state.mode = "smooth";
    }
  } catch (_) {
    // Defaults remain usable when local storage is unavailable.
  }
}

function saveCheckboxState() {
  const values = { dynamicLayerState: state.dynamicLayerState, mode: state.mode };
  staticCheckboxes.forEach((checkbox) => { values[checkbox.id] = checkbox.checked; });
  try { localStorage.setItem(CHECKBOX_STORAGE_KEY, JSON.stringify(values)); } catch (_) {}
}

function normalizePath(path) {
  const trimmed = path.trim().replace(/\/+$/, "");
  return trimmed.startsWith("/") ? trimmed : `/${trimmed}`;
}

async function json(url, dataset = false) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}: ${url}`);
  const text = await response.text();
  if (!dataset) return JSON.parse(text);
  return JSON.parse(text.trim().replace("window.REPLAY_DATASET = ", "").replace(/;$/, ""));
}

async function loadDataset(path) {
  state.datasetPath = normalizePath(path);
  el.path.value = state.datasetPath;
  state.dataset = await json(`${state.datasetPath}/dataset.js`, true);
  const count = state.dataset.frames?.length || 0;
  if (!count) throw new Error("V3 dataset has no frames");
  el.slider.max = String(count - 1);
  el.datasetMeta.textContent = `Topology V3 · ${count} frames`;
  await selectFrame(0);
}

async function selectFrame(index) {
  if (!state.dataset) return;
  state.index = Math.max(0, Math.min(index, state.dataset.frames.length - 1));
  const item = state.dataset.frames[state.index];
  state.debug = await json(`${state.datasetPath}/${item.files.topology_v3_debug}`);
  el.slider.value = String(state.index);
  el.frameMeta.textContent = `${state.index + 1} / ${state.dataset.frames.length} · ${item.main_frame_id}`;
  updateDynamicLayers();
  updateInfo(item);
  if (el.autoFit.checked) fit(); else render();
}

function updateInfo(item) {
  const visual = state.debug?.visual_reference || {};
  const nav = state.debug?.navigation_reference || {};
  const input = state.debug?.input_summary || {};
  el.info.textContent = [
    `frame: ${state.debug?.frame_id ?? item.main_frame_id}`,
    `visual input lines: ${input.visual_boundary_line_count ?? 0}`,
    `smooth pose: ${input.has_smooth_pose ? "ok" : "missing"}`,
    `gnss: ${input.has_gnss ? "ok" : "missing"}`,
    `route: ${input.has_navigation_route ? "ok" : "missing"}`,
    `visual reference: ${visual.ok ? visual.method || "ok" : visual.error || "ng"}`,
    `navigation reference: ${nav.ok ? `lat ${Number(nav.lateral_error_m || 0).toFixed(2)}m heading ${Number(nav.heading_error_rad || 0).toFixed(3)}rad` : nav.error || "ng"}`,
    `mode: ${modeTitle()}`,
    `debug layers: ${(state.debug?.debug_layers || []).map((layer) => layer.name).join(", ")}`,
  ].join("\n");
}

function modeTitle() {
  if (state.mode === "frenet") return "Frenet (s forward, l left)";
  if (state.mode === "smooth") return "Smooth (x, y)";
  return "VCS (x forward, y left)";
}

function view() {
  return state.views[state.mode];
}

function layerEnabled(layer) {
  if (!layerVisibleInCurrentMode(layer)) return false;
  if (layer.id === "visual_reference") return el.visualReference.checked;
  if (layer.id === "navigation_reference") return el.navigationReference.checked;
  return state.dynamicLayerState[layer.id] !== false;
}

function controlVisibleInCurrentMode(control) {
  if (!control?.dataset?.modes) return true;
  return control.dataset.modes.split(/\s+/).includes(state.mode);
}

function layerVisibleInCurrentMode(layer) {
  if (layer.id === "visual_reference") return controlVisibleInCurrentMode(el.visualReference.closest("[data-modes]"));
  if (layer.id === "navigation_reference") return controlVisibleInCurrentMode(el.navigationReference.closest("[data-modes]"));
  if (layer.id === "fused_reference") return state.mode === "vcs";
  if (layer.id === "raw_boundary_evidence") return state.mode === "frenet";
  if (layer.id === "raw_ribbon_graph") return state.mode === "frenet";
  if (!layer.modes && !layer.data_modes) return true;
  const modes = Array.isArray(layer.modes) ? layer.modes : String(layer.data_modes || "").split(/\s+/);
  return modes.includes(state.mode);
}

function updateLayerVisibility() {
  document.querySelectorAll("[data-modes]").forEach((control) => {
    control.hidden = !controlVisibleInCurrentMode(control);
  });
}

function updateDynamicLayers() {
  const layers = (state.debug?.viz_layers || [])
    .filter((layer) => layer.id !== "visual_reference" && layer.id !== "navigation_reference");
  el.dynamicLayers.innerHTML = "";
  if (!layers.length) return;
  const title = document.createElement("div");
  title.className = "layer-title";
  title.textContent = "Additional Layers";
  el.dynamicLayers.appendChild(title);
  layers.forEach((layer) => {
    if (!Object.hasOwn(state.dynamicLayerState, layer.id)) {
      state.dynamicLayerState[layer.id] = layer.visible !== false;
    }
    const label = document.createElement("label");
    label.className = "toggle";
    if (layer.id === "fused_reference") {
      label.dataset.modes = "vcs";
    } else if (layer.id === "raw_boundary_evidence") {
      label.dataset.modes = "frenet";
    } else if (layer.id === "raw_ribbon_graph") {
      label.dataset.modes = "frenet";
    } else if (Array.isArray(layer.modes)) {
      label.dataset.modes = layer.modes.join(" ");
    } else if (layer.data_modes) {
      label.dataset.modes = String(layer.data_modes);
    }
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = state.dynamicLayerState[layer.id] !== false;
    checkbox.onchange = () => {
      state.dynamicLayerState[layer.id] = checkbox.checked;
      saveCheckboxState();
      if (el.autoFit.checked) fit(); else render();
    };
    label.appendChild(checkbox);
    label.appendChild(document.createTextNode(` ${layer.name || layer.id}`));
    el.dynamicLayers.appendChild(label);
  });
  updateLayerVisibility();
}

function resize() {
  const rect = el.canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * ratio);
  const height = Math.round(rect.height * ratio);
  if (el.canvas.width !== width || el.canvas.height !== height) {
    el.canvas.width = width;
    el.canvas.height = height;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function screen(forward, lateral) {
  const rect = el.canvas.getBoundingClientRect();
  const current = view();
  return {
    x: rect.width / 2 - (lateral - current.lateral) * current.scale,
    y: rect.height / 2 - (forward - current.forward) * current.scale,
  };
}

function screenToWorld(x, y) {
  const rect = el.canvas.getBoundingClientRect();
  const current = view();
  return {
    forward: current.forward + (rect.height / 2 - y) / current.scale,
    lateral: current.lateral + (rect.width / 2 - x) / current.scale,
  };
}

function chooseGridStep() {
  const raw = 80 / Math.max(0.001, view().scale);
  const unit = 10 ** Math.floor(Math.log10(raw));
  const ratio = raw / unit;
  return ratio < 2 ? unit : ratio < 5 ? 2 * unit : 5 * unit;
}

function formatTick(value, step) {
  const digits = step >= 1 ? 0 : Math.min(2, Math.ceil(-Math.log10(step)));
  return Number(Math.abs(value) < 1e-9 ? 0 : value).toFixed(digits);
}

function drawGrid() {
  if (!el.grid.checked) return;
  const rect = el.canvas.getBoundingClientRect();
  const rangeA = screenToWorld(0, rect.height);
  const rangeB = screenToWorld(rect.width, 0);
  const minForward = Math.min(rangeA.forward, rangeB.forward);
  const maxForward = Math.max(rangeA.forward, rangeB.forward);
  const minLateral = Math.min(rangeA.lateral, rangeB.lateral);
  const maxLateral = Math.max(rangeA.lateral, rangeB.lateral);
  const step = chooseGridStep();

  ctx.strokeStyle = "#252b31";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let forward = Math.floor(minForward / step) * step; forward <= maxForward; forward += step) {
    const a = screen(forward, minLateral), b = screen(forward, maxLateral);
    ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
  }
  for (let lateral = Math.floor(minLateral / step) * step; lateral <= maxLateral; lateral += step) {
    const a = screen(minForward, lateral), b = screen(maxForward, lateral);
    ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
  }
  ctx.stroke();

  ctx.strokeStyle = "#5c6873";
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  const forwardAxisA = screen(0, minLateral), forwardAxisB = screen(0, maxLateral);
  const lateralAxisA = screen(minForward, 0), lateralAxisB = screen(maxForward, 0);
  ctx.moveTo(forwardAxisA.x, forwardAxisA.y); ctx.lineTo(forwardAxisB.x, forwardAxisB.y);
  ctx.moveTo(lateralAxisA.x, lateralAxisA.y); ctx.lineTo(lateralAxisB.x, lateralAxisB.y);
  ctx.stroke();

  const origin = screen(0, 0);
  ctx.fillStyle = "#aab5bf";
  ctx.font = "11px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  const tickY = Math.max(4, Math.min(rect.height - 16, origin.y + 8));
  for (let forward = Math.floor(minForward / step) * step; forward <= maxForward; forward += step) {
    if (Math.abs(forward) < 1e-9) continue;
    const p = screen(forward, 0);
    if (p.y >= 10 && p.y <= rect.height - 10) {
      ctx.fillText(formatTick(forward, step), Math.max(20, Math.min(rect.width - 20, origin.x + 20)), p.y - 6);
    }
  }
  for (let lateral = Math.floor(minLateral / step) * step; lateral <= maxLateral; lateral += step) {
    if (Math.abs(lateral) < 1e-9) continue;
    const p = screen(0, lateral);
    if (p.x >= 14 && p.x <= rect.width - 14) ctx.fillText(formatTick(lateral, step), p.x, tickY);
  }

  ctx.fillStyle = "#f1f3f5";
  ctx.font = "12px system-ui, sans-serif";
  ctx.textAlign = "left";
  const axes = state.mode === "frenet" ? ["s forward", "l left"] :
    state.mode === "smooth" ? ["x smooth", "y smooth"] : ["x forward", "y left"];
  ctx.fillText(`${axes[0]} ↑`, 10, 18);
  ctx.textAlign = "right";
  ctx.fillText(`← ${axes[1]}`, rect.width - 10, rect.height - 10);
  ctx.fillText("O", origin.x + 5, origin.y - 5);
}

function pointForMode(point) {
  if (Array.isArray(point)) {
    if (state.mode !== "vcs") return null;
    return { forward: Number(point[0]), lateral: Number(point[1]) };
  }
  if (state.mode === "frenet") {
    return {
      forward: Number(point.s_m ?? point.s),
      lateral: Number(point.l_m ?? point.l),
    };
  }
  if (state.mode === "smooth") {
    return {
      forward: Number(point.x_smooth_m),
      lateral: Number(point.y_smooth_m),
    };
  }
  return {
    forward: Number(point.x_vcs_m ?? point.x_m ?? point.x),
    lateral: Number(point.y_vcs_m ?? point.y_m ?? point.y),
  };
}

function itemPoints(item) {
  if (item.type === "ribbon") {
    return [...(item.right_points || []), ...(item.left_points || [])]
      .map(pointForMode)
      .filter((point) => point && Number.isFinite(point.forward) && Number.isFinite(point.lateral));
  }
  return (item.points || [])
    .map(pointForMode)
    .filter((point) => point && Number.isFinite(point.forward) && Number.isFinite(point.lateral));
}

function visualLaneStyle(line) {
  if (line.source_type === "curb") {
    return { color: "#eb5757", widthM: 0.12, dash: [] };
  }
  if (line.source_type === "road_edge") {
    return { color: "#f2994a", widthM: 0.1, dash: [10, 5] };
  }
  if (line.source_type === "stopline") {
    return { color: "#56ccf2", widthM: 0.09, dash: [3, 3] };
  }
  return { color: "#f4f4f2", widthM: 0.06, dash: [] };
}

function drawPolyline(points, color, width, dash = []) {
  if (points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  points.forEach((point, index) => {
    const p = screen(point.forward, point.lateral);
    if (index) ctx.lineTo(p.x, p.y); else ctx.moveTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawRibbon(rightPoints, leftPoints, style = {}) {
  if (state.mode !== "frenet" || rightPoints.length < 2 || leftPoints.length < 2) return;
  const color = style.color || "#27ae60";
  const fill = style.fill || color;
  const alpha = Math.max(0.02, Math.min(0.45, Number(style.alpha ?? 0.18)));
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = fill;
  ctx.beginPath();
  rightPoints.forEach((point, index) => {
    const p = screen(point.forward, point.lateral);
    if (index) ctx.lineTo(p.x, p.y); else ctx.moveTo(p.x, p.y);
  });
  for (let index = leftPoints.length - 1; index >= 0; --index) {
    const p = screen(leftPoints[index].forward, leftPoints[index].lateral);
    ctx.lineTo(p.x, p.y);
  }
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  const width = Math.max(1.0, Number(style.width || 0.04) * view().scale);
  drawPolyline(rightPoints, color, width);
  drawPolyline(leftPoints, color, width);
}

function drawSamples(points, color) {
  if (!el.points.checked) return;
  ctx.fillStyle = color;
  points.forEach((point) => {
    const p = screen(point.forward, point.lateral);
    ctx.beginPath(); ctx.arc(p.x, p.y, 2, 0, Math.PI * 2); ctx.fill();
  });
}

function drawLabel(points, text, color) {
  if (!el.labels.checked || !points.length) return;
  const anchor = points[Math.floor(points.length / 2)];
  const p = screen(anchor.forward, anchor.lateral);
  ctx.fillStyle = color;
  ctx.font = "bold 12px system-ui, sans-serif";
  ctx.textAlign = "left";
  ctx.textBaseline = "bottom";
  ctx.fillText(text, p.x + 6, p.y - 5);
}

function layerLabel(layer, item) {
  if (layer.id === "visual_reference") {
    const method = item.properties?.method || state.debug?.visual_reference?.method || "";
    return `Visual ${method}`;
  }
  if (layer.id === "navigation_reference") {
    const nav = state.debug?.navigation_reference || {};
    return `Navigation ${Number(nav.lateral_error_m || 0).toFixed(2)}m`;
  }
  return item.name || item.id || layer.name || layer.id;
}

function drawVizItem(layer, item) {
  if (item.type === "ribbon") {
    const rightPoints = (item.right_points || [])
      .map(pointForMode)
      .filter((point) => point && Number.isFinite(point.forward) && Number.isFinite(point.lateral));
    const leftPoints = (item.left_points || [])
      .map(pointForMode)
      .filter((point) => point && Number.isFinite(point.forward) && Number.isFinite(point.lateral));
    const style = item.style || {};
    const color = style.color || "#27ae60";
    drawRibbon(rightPoints, leftPoints, style);
    drawSamples([...rightPoints, ...leftPoints], color);
    drawLabel([...rightPoints, ...leftPoints], layerLabel(layer, item), color);
    return;
  }
  if (item.type !== "polyline") return;
  const points = itemPoints(item);
  const style = item.style || {};
  const color = style.color || (layer.id === "navigation_reference" ? "#2d9cdb" : "#00b894");
  const width = Math.max(1.2, Number(style.width || 0.14) * view().scale);
  drawPolyline(points, color, width, style.dash ? [8, 5] : []);
  drawSamples(points, color);
  drawLabel(points, layerLabel(layer, item), color);
}

function visibleLayers() {
  return (state.debug?.viz_layers || []).filter(layerEnabled);
}

function render() {
  resize();
  const rect = el.canvas.getBoundingClientRect();
  ctx.fillStyle = "#111316";
  ctx.fillRect(0, 0, rect.width, rect.height);
  drawGrid();
  if (controlVisibleInCurrentMode(el.visualLanes.closest("[data-modes]")) && el.visualLanes.checked) {
    (state.debug?.visual_lane_lines || []).forEach((line) => {
      const points = itemPoints(line);
      const style = visualLaneStyle(line);
      drawPolyline(points, style.color, Math.max(1.2, style.widthM * view().scale), style.dash);
      drawSamples(points, style.color);
      drawLabel(points, `V${line.lane_id} ${line.source_type || "lane"} ${line.lane_position || ""}`, style.color);
    });
  }
  visibleLayers().forEach((layer) => {
    (layer.items || []).forEach((item) => drawVizItem(layer, item));
  });
  const ego = screen(0, 0);
  ctx.fillStyle = "#ffffff";
  ctx.beginPath(); ctx.arc(ego.x, ego.y, 4, 0, Math.PI * 2); ctx.fill();
}

function fit() {
  const points = [];
  if (controlVisibleInCurrentMode(el.visualLanes.closest("[data-modes]")) && el.visualLanes.checked) {
    (state.debug?.visual_lane_lines || []).forEach((line) => points.push(...itemPoints(line)));
  }
  visibleLayers().forEach((layer) => {
    (layer.items || []).forEach((item) => points.push(...itemPoints(item)));
  });
  if (!points.length) return render();
  const forward = points.map((point) => point.forward);
  const lateral = points.map((point) => point.lateral);
  const minForward = Math.min(...forward), maxForward = Math.max(...forward);
  const minLateral = Math.min(...lateral), maxLateral = Math.max(...lateral);
  const rect = el.canvas.getBoundingClientRect();
  const current = view();
  current.forward = (minForward + maxForward) / 2;
  current.lateral = (minLateral + maxLateral) / 2;
  current.scale = Math.max(0.2, Math.min(
    (rect.height * 0.82) / Math.max(6, maxForward - minForward + 8),
    (rect.width * 0.82) / Math.max(6, maxLateral - minLateral + 8)));
  render();
}

function setMode(mode) {
  state.mode = mode;
  el.modeVcs.classList.toggle("active", mode === "vcs");
  el.modeFrenet.classList.toggle("active", mode === "frenet");
  el.modeSmooth.classList.toggle("active", mode === "smooth");
  updateLayerVisibility();
  saveCheckboxState();
  if (state.debug) updateInfo(state.dataset.frames[state.index]);
  if (el.autoFit.checked) fit(); else render();
}

el.load.onclick = () => loadDataset(el.path.value).catch((error) => alert(error.message));
el.slider.oninput = () => selectFrame(Number(el.slider.value)).catch((error) => alert(error.message));
el.previous.onclick = () => selectFrame(state.index - 1).catch((error) => alert(error.message));
el.next.onclick = () => selectFrame(state.index + 1).catch((error) => alert(error.message));
el.jumpButton.onclick = () => {
  const frame = Number(el.jump.value);
  const index = state.dataset?.frames.findIndex((item) => Number(item.main_frame_id) === frame);
  if (index >= 0) selectFrame(index).catch((error) => alert(error.message));
};
el.fit.onclick = fit;
el.modeVcs.onclick = () => setMode("vcs");
el.modeFrenet.onclick = () => setMode("frenet");
el.modeSmooth.onclick = () => setMode("smooth");
staticCheckboxes.forEach((checkbox) => {
  checkbox.onchange = () => {
    saveCheckboxState();
    if (el.autoFit.checked) fit(); else render();
  };
});
el.canvas.onwheel = (event) => {
  event.preventDefault();
  const rect = el.canvas.getBoundingClientRect();
  const before = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
  view().scale = Math.max(0.1, Math.min(100, view().scale * (event.deltaY < 0 ? 1.1 : 0.9)));
  const after = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
  view().forward += before.forward - after.forward;
  view().lateral += before.lateral - after.lateral;
  render();
};
el.canvas.onpointerdown = (event) => {
  state.dragging = true;
  state.lastX = event.clientX;
  state.lastY = event.clientY;
  el.canvas.classList.add("dragging");
  el.canvas.setPointerCapture(event.pointerId);
};
el.canvas.onpointermove = (event) => {
  if (!state.dragging) return;
  view().forward += (event.clientY - state.lastY) / view().scale;
  view().lateral += (event.clientX - state.lastX) / view().scale;
  state.lastX = event.clientX;
  state.lastY = event.clientY;
  render();
};
el.canvas.onpointerup = () => {
  state.dragging = false;
  el.canvas.classList.remove("dragging");
};
window.onresize = render;

restoreViewerState();
setMode(state.mode);
loadDataset(state.datasetPath).catch((error) => { el.info.textContent = error.message; render(); });
