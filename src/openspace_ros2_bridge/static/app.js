// ---- Auth guard ----
const token = sessionStorage.getItem('token');
if (!token) { location.href = '/index.html'; }

function authHeaders() {
  return { 'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json' };
}

async function apiGet(path) {
  const res = await fetch(path, { headers: authHeaders() });
  if (res.status === 401) { location.href = '/index.html'; return {}; }
  if (!res.ok) { const t = await res.text(); throw new Error(t); }
  return res.json();
}

async function apiPost(path, body) {
  const res = await fetch(path, { method: 'POST', headers: authHeaders(), body: JSON.stringify(body) });
  if (res.status === 401) { location.href = '/index.html'; return {}; }
  if (!res.ok) { const t = await res.text(); throw new Error(t); }
  return res.json();
}

async function apiDelete(path) {
  const res = await fetch(path, { method: 'DELETE', headers: authHeaders() });
  if (res.status === 401) { location.href = '/index.html'; return {}; }
  if (!res.ok) { const t = await res.text(); throw new Error(t); }
  return res.json();
}

// ---- DOM ----
const $ = (id) => document.getElementById(id);
function setStatus(text, cls) {
  const bar = $('statusBar');
  bar.textContent = text;
  bar.className = 'status-bar ' + (cls || '');
}

// ---- State ----
let map = null;
let gridLayer = null;
let axesLayer = null;
let polygonLayer = null;       // completed polygons (L.LayerGroup)
let drawingLayer = null;       // in-progress drawing preview
// Start/goal: draggable arrow markers (base + shaft + head + rotate-handle)
let sBase = null, sShaft = null, sHead = null, sHandle = null;
let gBase = null, gShaft = null, gHead = null, gHandle = null;
const ARROW_LEN = 2.5;
const HEAD_SZ = 0.8;
let pathLayer = null;
let boxesLayer = null;
let boxesVisible = true;        // toggle vehicle box visibility
let pathPolyline = null;
let vehiclePolylines = [];
let vehicleParams = {};        // cached from /api/vehicle-config, used by renderBoxes

let polygons = [];             // [{vertices: [[x,y],...], is_occupied: bool}, ...]
let selectedPolyIndices = [];   // indices of selected polygons (can be multiple)
let polygonRefs = [];           // L.Polygon references for style updates on selection
let drawingPoints = [];        // [[x,y], ...]
let drawingVertexMarkers = [];  // L.CircleMarker[]
let drawingPreviewLine = null; // L.Polyline (mouse follow)
let currentMode = 'obstacle';   // 'obstacle' | 'selectObstacle' | 'start' | 'goal'
let sceneList = [];

// ---- Auth bar ----
try {
  const payload = JSON.parse(atob(token.split('.')[1]));
  $('authUser').textContent = payload.sub || 'user';
} catch (e) { $('authUser').textContent = 'user'; }

function doLogout() {
  sessionStorage.removeItem('token');
  location.href = '/index.html';
}

// ===================================================================
// MAP
// ===================================================================
function initMap() {
  map = L.map('map', {
    crs: L.CRS.Simple,
    minZoom: -1,
    maxZoom: 5,
    zoomSnap: 0.25,
    zoomDelta: 0.5,
    attributionControl: false,
  });

  // Fit the grid area (-5..55) with padding so it fills the view nicely
  map.fitBounds([[ -5, -5 ], [ 55, 55 ]], { padding: [ 15, 15 ] });

  // Layer groups
  gridLayer = L.layerGroup().addTo(map);
  axesLayer = L.layerGroup().addTo(map);
  polygonLayer = L.layerGroup().addTo(map);
  drawingLayer = L.layerGroup().addTo(map);
  pathLayer = L.layerGroup().addTo(map);
  boxesLayer = L.layerGroup().addTo(map);

  drawGrid();
  drawAxes();

  // Map click handler
  map.on('click', onMapClick);
  map.on('mousemove', onMapMouseMove);
}

function drawGrid() {
  const lines = [];
  for (let i = -5; i <= 55; i += 5) {
    lines.push(L.polyline([[i, -5], [i, 55]], { color: '#e2e8f0', weight: 1, interactive: false }));
    lines.push(L.polyline([[-5, i], [55, i]], { color: '#e2e8f0', weight: 1, interactive: false }));
  }
  lines.forEach(l => gridLayer.addLayer(l));
}

function drawAxes() {
  // X axis (red) — Y=0
  axesLayer.addLayer(L.polyline([[0, -5], [0, 55]], { color: '#e53e3e', weight: 1.5, dashArray: '8,4', interactive: false }));
  // Y axis (green) — X=0
  axesLayer.addLayer(L.polyline([[-5, 0], [55, 0]], { color: '#38a169', weight: 1.5, dashArray: '8,4', interactive: false }));

  // Axis labels
  const labelIcon = (text, color) => L.divIcon({
    className: '',
    html: `<div style="font-size:11px;font-weight:bold;color:${color};white-space:nowrap;text-shadow:0 0 3px #fff">${text}</div>`,
    iconSize: [30, 14],
    iconAnchor: [0, 7],
  });
  axesLayer.addLayer(L.marker([0.3, 53], { icon: labelIcon('X+', '#e53e3e'), interactive: false }));
  axesLayer.addLayer(L.marker([53, 0.3], { icon: labelIcon('Y+', '#38a169'), interactive: false }));
  // Origin
  axesLayer.addLayer(L.circleMarker([0, 0], { radius: 3, color: '#1a1a2e', fillColor: '#1a1a2e', fillOpacity: 1, interactive: false }));
}

// ---- Mode ----
function setMode(mode) {
  currentMode = mode;
  document.querySelectorAll('.mode-btn').forEach(b => {
    b.classList.remove('active');
    if (b.dataset.mode === mode) b.classList.add('active');
  });

  // Clear any in-progress drawing and selection
  cancelDrawing();
  if (mode !== 'selectObstacle') {
    selectedPolyIndices = [];
    renderAllPolygons();
    updatePolyInfo();
  }

  if (mode === 'obstacle') {
    map.getContainer().style.cursor = 'crosshair';
  } else if (mode === 'selectObstacle') {
    map.getContainer().style.cursor = 'pointer';
  } else if (mode === 'start' || mode === 'goal') {
    map.getContainer().style.cursor = 'pointer';
  }
}

// ---- Map click ----
function onMapClick(e) {
  // Suppress click if it came from a just-finished handle drag
  if (Date.now() < _blockMapClickUntil) return;

  const pt = [e.latlng.lng, e.latlng.lat]; // CRS.Simple: lng=x, lat=y

  if (currentMode === 'obstacle') {
    addDrawingVertex(pt);
  } else if (currentMode === 'selectObstacle') {
    // Find all polygons that contain the clicked point
    const hits = [];
    for (let i = 0; i < polygons.length; i++) {
      if (_pointInPolygon(pt, polygons[i].vertices)) {
        hits.push(i);
      }
    }
    selectedPolyIndices = hits;
    renderAllPolygons();
    updatePolyInfo();
    if (hits.length === 0) {
      setStatus('No obstacle at this location', '');
    } else {
      setStatus(`Selected ${hits.length} obstacle(s): #${hits.map(i => i + 1).join(', ')}`, 'ok');
    }
  } else if (currentMode === 'start') {
    setStart(pt);
  } else if (currentMode === 'goal') {
    setGoal(pt);
  }
}

// Ray-casting point-in-polygon test
function _pointInPolygon(pt, vertices) {
  const x = pt[0], y = pt[1];
  let inside = false;
  for (let i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
    const xi = vertices[i][0], yi = vertices[i][1];
    const xj = vertices[j][0], yj = vertices[j][1];
    if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

function onMapMouseMove(e) {
  if (drawingPoints.length === 0) return;
  if (currentMode !== 'obstacle') return;
  const lastPt = drawingPoints[drawingPoints.length - 1];
  // Leaflet: [lat, lng] = [y, x]
  const mousePt = [e.latlng.lat, e.latlng.lng];
  if (drawingPreviewLine) drawingLayer.removeLayer(drawingPreviewLine);
  drawingPreviewLine = L.polyline([[lastPt[1], lastPt[0]], mousePt], {
    color: '#4361ee', weight: 2, dashArray: '6,4', interactive: false
  }).addTo(drawingLayer);
}

// ===================================================================
// POLYGON DRAWING
// ===================================================================
function _drawingToLatLngs(pts) {
  return pts.map(p => [p[1], p[0]]); // [x,y] → [y,x] for Leaflet CRS.Simple
}

function addDrawingVertex(pt) {
  drawingPoints.push(pt);
  // Leaflet: [lat, lng] = [y, x]
  const marker = L.circleMarker([pt[1], pt[0]], {
    radius: 4, color: '#1a1a2e', fillColor: '#e53e3e',
    fillOpacity: 0.8, weight: 1.5
  }).addTo(drawingLayer);
  drawingVertexMarkers.push(marker);

  // Update preview line between placed vertices
  if (drawingPoints.length >= 2) {
    if (drawingPreviewLine) drawingLayer.removeLayer(drawingPreviewLine);
    drawingPreviewLine = L.polyline(_drawingToLatLngs(drawingPoints), {
      color: '#4361ee', weight: 2, dashArray: '6,4', interactive: false
    }).addTo(drawingLayer);
  }
  updatePolyInfo();
}

function closePolygon() {
  if (drawingPoints.length < 3) {
    setStatus('Need at least 3 vertices', 'err');
    return;
  }
  const poly = {
    vertices: drawingPoints.map(p => [p[0], p[1]]),
    is_occupied: currentMode === 'obstacle'
  };
  polygons.push(poly);
  renderAllPolygons();
  clearDrawing();
  updatePolyInfo();
  setStatus(`Polygon #${polygons.length} added`, 'ok');
}

function undoLastVertex() {
  if (drawingPoints.length === 0) return;
  drawingPoints.pop();
  const m = drawingVertexMarkers.pop();
  if (m) drawingLayer.removeLayer(m);
  if (drawingPreviewLine) drawingLayer.removeLayer(drawingPreviewLine);
  if (drawingPoints.length >= 2) {
    drawingPreviewLine = L.polyline(_drawingToLatLngs(drawingPoints), {
      color: '#4361ee', weight: 2, dashArray: '6,4', interactive: false
    }).addTo(drawingLayer);
  }
  updatePolyInfo();
}

function cancelDrawing() {
  clearDrawing();
  updatePolyInfo();
}

function clearDrawing() {
  drawingPoints = [];
  drawingVertexMarkers.forEach(m => drawingLayer.removeLayer(m));
  drawingVertexMarkers = [];
  if (drawingPreviewLine) drawingLayer.removeLayer(drawingPreviewLine);
  drawingPreviewLine = null;
}

function clearAllPolygons() {
  polygons = [];
  polygonRefs = [];
  selectedPolyIndices = [];
  clearDrawing();
  renderAllPolygons();
  updatePolyInfo();
  setStatus('All polygons cleared', 'ok');
}

function updatePolyInfo() {
  const total = polygons.length;
  const drawing = drawingPoints.length;
  let parts = [`${total} obstacle(s) on map`];
  if (selectedPolyIndices.length > 0) {
    parts.push(`Selected: #${selectedPolyIndices.map(i => i + 1).join(', ')}`);
  }
  if (drawing > 0) {
    parts.push(`Drawing: ${drawing} verts`);
  }
  $('polyInfo').textContent = parts.join(' | ');
}

function deleteSelectedPolygon() {
  if (selectedPolyIndices.length === 0) {
    setStatus('No obstacle selected — use SelectObstacle mode first', 'err');
    return;
  }
  const count = selectedPolyIndices.length;
  // Delete from highest index to lowest to preserve indices
  const sorted = [...selectedPolyIndices].sort((a, b) => b - a);
  for (const idx of sorted) {
    polygons.splice(idx, 1);
    polygonRefs.splice(idx, 1);
  }
  selectedPolyIndices = [];
  renderAllPolygons();
  updatePolyInfo();
  setStatus(`Deleted ${count} obstacle(s)`, 'ok');
}

// ---- Render completed polygons ----
function renderAllPolygons() {
  polygonLayer.clearLayers();
  polygonRefs = [];
  const selectedSet = new Set(selectedPolyIndices);
  polygons.forEach((poly, i) => {
    const verts = poly.vertices.map(v => [v[1], v[0]]); // Leaflet: [lat, lng] = [y, x]
    const fillColor = poly.is_occupied ? '#e53e3e' : '#38a169';
    const strokeColor = poly.is_occupied ? '#c53030' : '#2f855a';
    const isSelected = selectedSet.has(i);

    const pg = L.polygon(verts, {
      color: strokeColor,
      weight: isSelected ? 4 : 2,
      fillColor: fillColor,
      fillOpacity: isSelected ? 0.55 : 0.35,
    }).addTo(polygonLayer);
    polygonRefs.push(pg);

    // Number label at centroid
    const cx = poly.vertices.reduce((s, v) => s + v[0], 0) / poly.vertices.length;
    const cy = poly.vertices.reduce((s, v) => s + v[1], 0) / poly.vertices.length;
    const icon = L.divIcon({
      className: '',
      html: `<div style="font-size:12px;font-weight:bold;color:#fff;background:${fillColor};border-radius:10px;width:20px;height:20px;line-height:20px;text-align:center">${i + 1}</div>`,
      iconSize: [20, 20],
      iconAnchor: [10, 10],
    });
    L.marker([cy, cx], { icon, interactive: false }).addTo(polygonLayer);
  });
}

// ---- Bulk coordinate import ----
function importCoords() {
  const raw = $('coordInput').value.trim();
  if (!raw) return;
  const verts = raw.split('\n')
    .map(line => line.split(',').map(s => parseFloat(s.trim())))
    .filter(pair => pair.length === 2 && !isNaN(pair[0]) && !isNaN(pair[1]));
  if (verts.length < 3) {
    setStatus('Need at least 3 coordinate pairs', 'err');
    return;
  }
  const poly = {
    vertices: verts.map(v => [v[0], v[1]]),
    is_occupied: true
  };
  polygons.push(poly);
  renderAllPolygons();
  updatePolyInfo();
  $('coordInput').value = '';
  setStatus(`Polygon #${polygons.length} imported (${verts.length} verts)`, 'ok');
}

// ===================================================================
// START / GOAL — arrow markers with independent position + angle control
//
// Each arrow has 4 Leaflet layers:
//   base   — solid circle, L.marker (draggable: true) → moves (x,y)
//   shaft  — L.polyline from base to tip
//   head   — L.polygon (arrowhead triangle at tip)
//   handle — hollow circle, L.marker (draggable: false, custom DOM drag)
//
// Interaction rules:
//   - Dragging BASE updates (x,y) and moves the whole arrow
//   - Dragging HANDLE updates yaw only — (x,y) is LOCKED at mousedown
//   - Clicking on map in start/goal mode places a new arrow at default 0°
//   - After handle drag ends, map clicks are suppressed for 250ms to
//     prevent the mouseup from leaking into onMapClick
// ===================================================================

let _blockMapClickUntil = 0;   // timestamp — ignore onMapClick until this time

function _circleIcon(sizePx, color, fillColor, fillOpacity, weight) {
  const sz = sizePx;
  const border = weight || 2;
  return L.divIcon({
    className: 'arrow-circle-icon',
    html: `<div style="width:${sz}px;height:${sz}px;border-radius:50%;background:${fillColor};opacity:${fillOpacity};border:${border}px solid ${color};box-sizing:border-box"></div>`,
    iconSize: [sz, sz],
    iconAnchor: [sz / 2, sz / 2],
  });
}

function _arrowGeo(x, y, yawDeg) {
  const rad = yawDeg * Math.PI / 180;
  const tipX = x + Math.cos(rad) * ARROW_LEN;
  const tipY = y + Math.sin(rad) * ARROW_LEN;
  const bx = tipX - Math.cos(rad) * HEAD_SZ;
  const by = tipY - Math.sin(rad) * HEAD_SZ;
  const px = Math.sin(rad) * HEAD_SZ * 0.4;
  const py = Math.cos(rad) * HEAD_SZ * 0.4;
  return {
    base: [y, x],
    tip:  [tipY, tipX],
    head: [[tipY, tipX], [by - py, bx + px], [by + py, bx - px]],
  };
}

function _clearArrow(kind) {
  if (kind === 's') {
    if (sHandle && sHandle._cleanup) sHandle._cleanup();
    [sBase, sShaft, sHead, sHandle].forEach(m => { if (m) map.removeLayer(m); });
    sBase = sShaft = sHead = sHandle = null;
  } else {
    if (gHandle && gHandle._cleanup) gHandle._cleanup();
    [gBase, gShaft, gHead, gHandle].forEach(m => { if (m) map.removeLayer(m); });
    gBase = gShaft = gHead = gHandle = null;
  }
}

// Move EVERYTHING (base + shaft + head + handle) — used when base is dragged
function _redrawArrow(kind, x, y, yawDeg) {
  const g = _arrowGeo(x, y, yawDeg);
  if (kind === 's') {
    sBase.setLatLng(g.base); sShaft.setLatLngs([g.base, g.tip]);
    sHead.setLatLngs(g.head); sHandle.setLatLng(g.tip);
  } else {
    gBase.setLatLng(g.base); gShaft.setLatLngs([g.base, g.tip]);
    gHead.setLatLngs(g.head); gHandle.setLatLng(g.tip);
  }
}

// Move ONLY shaft + head + handle — base (x,y) is immutable.  Used during handle drag.
function _redrawArrowDirection(kind, x, y, yawDeg) {
  const g = _arrowGeo(x, y, yawDeg);
  if (kind === 's') {
    sShaft.setLatLngs([g.base, g.tip]);
    sHead.setLatLngs(g.head);
    sHandle.setLatLng(g.tip);
  } else {
    gShaft.setLatLngs([g.base, g.tip]);
    gHead.setLatLngs(g.head);
    gHandle.setLatLng(g.tip);
  }
}

// Base (solid circle) drag — moves the whole arrow
function _onBaseDrag(kind) {
  return function(e) {
    const ll = e.target.getLatLng();
    const x = ll.lng, y = ll.lat;
    const yaw = parseFloat($(kind === 's' ? 'startYaw' : 'goalYaw').value) || 0;
    _redrawArrow(kind, x, y, yaw);
    $(kind === 's' ? 'startX' : 'goalX').value = x.toFixed(2);
    $(kind === 's' ? 'startY' : 'goalY').value = y.toFixed(2);
    $(kind === 's' ? 'startLabel' : 'goalLabel').textContent =
      `(${x.toFixed(1)}, ${y.toFixed(1)}, ${yaw.toFixed(1)}°)`;
  };
}

// Handle (hollow circle) drag — uses raw DOM capture-phase listeners to
// completely bypass Leaflet's event system.  Only yaw is updated; (x,y) is
// locked at mousedown.  Map clicks are suppressed for 250ms after drag ends.
function _setupHandleDrag(kind, handle, baseRef) {
  handle.on('add', function() {
    const el = handle.getElement();
    if (!el) return;

    let active = false;
    let lockedBase = null;
    let moved = false;  // track if actual drag happened (vs. static click)

    // Capture phase: fire BEFORE Leaflet sees the event
    function onDown(e) {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();
      active = true;
      moved = false;
      const baseLL = baseRef.getLatLng();
      lockedBase = { lng: baseLL.lng, lat: baseLL.lat };
      // Prevent map pan and base-marker drag while adjusting angle
      map.dragging.disable();
      if (baseRef.dragging) baseRef.dragging.disable();
    }

    function onMove(e) {
      if (!active || !lockedBase) return;
      const rect = map.getContainer().getBoundingClientRect();
      const cx = e.clientX - rect.left;
      const cy = e.clientY - rect.top;
      const ll = map.containerPointToLatLng([cx, cy]);
      const dx = ll.lng - lockedBase.lng;
      const dy = ll.lat - lockedBase.lat;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < 0.3) return;
      moved = true;
      const yaw = Math.atan2(dy, dx) * 180 / Math.PI;

      _redrawArrowDirection(kind, lockedBase.lng, lockedBase.lat, yaw);

      $(kind === 's' ? 'startYaw' : 'goalYaw').value = yaw.toFixed(1);
      $(kind === 's' ? 'startLabel' : 'goalLabel').textContent =
        `(${lockedBase.lng.toFixed(1)}, ${lockedBase.lat.toFixed(1)}, ${yaw.toFixed(1)}°)`;
    }

    function onUp(e) {
      if (!active) return;
      e.preventDefault();
      e.stopPropagation();
      active = false;
      lockedBase = null;
      map.dragging.enable();
      if (baseRef.dragging) baseRef.dragging.enable();
      // Block map click for 250ms after handle drag ends, whether or not
      // the mouse actually moved.  This prevents the mouseup from leaking
      // through as a map click that would call setStart/setGoal again.
      _blockMapClickUntil = Date.now() + 250;
    }

    // Capture phase (true) so we intercept before Leaflet
    el.addEventListener('mousedown', onDown, true);
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp, true);

    handle._cleanup = function() {
      el.removeEventListener('mousedown', onDown, true);
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp, true);
    };
  });
}

function _makeArrow(kind, pt, yawDeg, baseColor) {
  const g = _arrowGeo(pt[0], pt[1], yawDeg);

  const baseIcon = _circleIcon(16, baseColor, baseColor, 1, 2);
  const base = L.marker(g.base, { icon: baseIcon, draggable: true, zIndexOffset: 100 });

  const shaft = L.polyline([g.base, g.tip], { color: baseColor, weight: 3, opacity: 0.9 });

  const head = L.polygon(g.head, { color: baseColor, fillColor: baseColor, fillOpacity: 0.9, weight: 1 });

  const handleIcon = _circleIcon(16, baseColor, '#fff', 1, 2.5);
  const handle = L.marker(g.tip, { icon: handleIcon, draggable: false, zIndexOffset: 200 });

  base.on('drag', _onBaseDrag(kind));
  _setupHandleDrag(kind, handle, base);

  base.addTo(map);
  shaft.addTo(map);
  head.addTo(map);
  handle.addTo(map);

  return { base, shaft, head, handle };
}

function setStart(pt, yawDeg) {
  if (typeof yawDeg === 'undefined') yawDeg = parseFloat($('startYaw').value) || 0;
  _clearArrow('s');
  const parts = _makeArrow('s', pt, yawDeg, '#3182ce');
  sBase = parts.base; sShaft = parts.shaft; sHead = parts.head; sHandle = parts.handle;
  $('startX').value = pt[0].toFixed(2);
  $('startY').value = pt[1].toFixed(2);
  $('startYaw').value = yawDeg.toFixed(1);
  $('startLabel').textContent = `(${pt[0].toFixed(1)}, ${pt[1].toFixed(1)}, ${yawDeg.toFixed(1)}°)`;
}

function setGoal(pt, yawDeg) {
  if (typeof yawDeg === 'undefined') yawDeg = parseFloat($('goalYaw').value) || 0;
  _clearArrow('g');
  const parts = _makeArrow('g', pt, yawDeg, '#dd6b20');
  gBase = parts.base; gShaft = parts.shaft; gHead = parts.head; gHandle = parts.handle;
  $('goalX').value = pt[0].toFixed(2);
  $('goalY').value = pt[1].toFixed(2);
  $('goalYaw').value = yawDeg.toFixed(1);
  $('goalLabel').textContent = `(${pt[0].toFixed(1)}, ${pt[1].toFixed(1)}, ${yawDeg.toFixed(1)}°)`;
}

// ===================================================================
// SCENES
// ===================================================================
async function loadSceneList() {
  try {
    const data = await apiGet('/api/scenes');
    sceneList = data.scenes || [];
    const sel = $('sceneSelect');
    sel.innerHTML = '';
    sceneList.forEach(s => {
      const opt = document.createElement('option');
      opt.value = s.name;
      opt.textContent = `${s.name} (${s.polygon_count} polys)`;
      sel.appendChild(opt);
    });
  } catch (e) {
    setStatus('Failed to load scene list: ' + e.message, 'err');
  }
}

function onSceneSelect() {
  const name = $('sceneSelect').value;
  $('saveName').value = name;
}

async function loadSelectedScene() {
  const name = $('sceneSelect').value;
  if (!name) return;
  try {
    const data = await apiGet(`/api/scenes/${name}`);
    polygons = data.polygons || [];
    selectedPolyIndices = [];
    clearDrawing();
    renderAllPolygons();
    updatePolyInfo();
    $('saveName').value = name;

    // Fit map to polygon bounds
    if (polygons.length > 0) {
      const allVerts = polygons.flatMap(p => p.vertices);
      const xs = allVerts.map(v => v[0]);
      const ys = allVerts.map(v => v[1]);
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minY = Math.min(...ys), maxY = Math.max(...ys);
      const pad = Math.max((maxX - minX) * 0.1, (maxY - minY) * 0.1, 5);
      map.fitBounds([[minY - pad, minX - pad], [maxY + pad, maxX + pad]]);
    }

    setStatus(`Loaded "${name}" (${polygons.length} polygons)`, 'ok');
  } catch (e) {
    setStatus('Failed to load scene: ' + e.message, 'err');
  }
}

async function saveScene() {
  const name = $('saveName').value.trim();
  if (!name) { setStatus('Enter a filename', 'err'); return; }
  if (!name.endsWith('.json')) { setStatus('Filename must end with .json', 'err'); return; }
  try {
    await apiPost('/api/scenes/save', { name, polygons });
    setStatus(`Saved "${name}"`, 'ok');
    await loadSceneList();
    $('sceneSelect').value = name;
  } catch (e) {
    setStatus('Failed to save: ' + e.message, 'err');
  }
}

async function deleteSelectedScene() {
  const name = $('sceneSelect').value;
  if (!name) return;
  if (!confirm(`Delete "${name}"?`)) return;
  try {
    await apiDelete(`/api/scenes/${name}`);
    setStatus(`Deleted "${name}"`, 'ok');
    await loadSceneList();
  } catch (e) {
    setStatus('Failed to delete: ' + e.message, 'err');
  }
}

// ===================================================================
// VEHICLE CONFIG
// ===================================================================
const VEH_PARAM_LABELS = {
  'vehicle.length': 'Length (m)',
  'vehicle.width': 'Width (m)',
  'vehicle.wheel_base': 'Wheel Base (m)',
  'vehicle.front_overhang': 'Front Overhang (m)',
  'vehicle.rear_overhang': 'Rear Overhang (m)',
  'vehicle.rear_edge_to_ego': 'Rear Edge to Ego (m)',
  'vehicle.width_without_rearview_mirror': 'Width w/o Mirror (m)',
};

let vehiclePresets = [];
let activePreset = '9m6';

async function loadVehicleConfig() {
  try {
    const data = await apiGet('/api/vehicle-config');
    vehiclePresets = data.presets || [];
    activePreset = data.preset || '9m6';
    vehicleParams = data.params || {};
    buildPresetSelect();
    renderVehicleParams(data.params || {});
  } catch (e) {
    setStatus('Failed to load vehicle config: ' + e.message, 'err');
  }
}

function buildPresetSelect() {
  const sel = $('vehPreset');
  sel.innerHTML = '';
  vehiclePresets.forEach(p => {
    const opt = document.createElement('option');
    opt.value = p.name;
    opt.textContent = p.label || p.name;
    if (p.name === activePreset) opt.selected = true;
    sel.appendChild(opt);
  });
}

function renderVehicleParams(params) {
  const container = $('vehParams');
  container.innerHTML = '';
  const isCustom = activePreset === 'custom';
  for (const [key, label] of Object.entries(VEH_PARAM_LABELS)) {
    const val = params[key] !== undefined ? params[key] : '';
    container.innerHTML += `
      <label>${label}</label>
      <input type="number" step="0.01" value="${val}" data-key="${key}" ${isCustom ? '' : 'readonly'}>
    `;
  }
}

async function onPresetChange() {
  const presetName = $('vehPreset').value;
  try {
    const data = await apiPost('/api/vehicle-config', { preset: presetName });
    activePreset = data.preset || presetName;
    renderVehicleParams(data.params || {});
    if (data.note) {
      $('vehNote').textContent = data.note;
    } else {
      $('vehNote').textContent = '';
    }
  } catch (e) {
    setStatus('Failed to set vehicle config: ' + e.message, 'err');
  }
}

async function saveCustomParams() {
  const inputs = document.querySelectorAll('#vehParams input');
  const params = {};
  inputs.forEach(inp => {
    const val = parseFloat(inp.value);
    if (!isNaN(val)) params[inp.dataset.key] = val;
  });
  try {
    const data = await apiPost('/api/vehicle-config', { params });
    activePreset = data.preset || 'custom';
    renderVehicleParams(data.params || {});
    $('vehNote').textContent = data.note || '';
  } catch (e) {
    setStatus('Failed to save custom params: ' + e.message, 'err');
  }
}

// ===================================================================
// PLANNING
// ===================================================================
let planning = false;

async function triggerPlan() {
  if (planning) return;

  const startX = parseFloat($('startX').value);
  const startY = parseFloat($('startY').value);
  const startYaw = parseFloat($('startYaw').value);
  const goalX = parseFloat($('goalX').value);
  const goalY = parseFloat($('goalY').value);
  const goalYaw = parseFloat($('goalYaw').value);

  if (isNaN(startX) || isNaN(startY) || isNaN(goalX) || isNaN(goalY)) {
    setStatus('Set start and goal positions first', 'err');
    return;
  }

  planning = true;
  const btn = $('planBtn');
  btn.classList.add('computing');
  btn.innerHTML = '<span class="spinner"></span> Computing...';
  $('resultBox').className = 'result-box';
  $('resultBox').style.display = 'none';
  pathLayer.clearLayers();
  boxesLayer.clearLayers();
  setStatus('Plan submitted, waiting for result...', '');

  try {
    await apiPost('/api/plan', {
      start: { x: startX, y: startY, yaw_deg: startYaw },
      goal: { x: goalX, y: goalY, yaw_deg: goalYaw },
      polygons: polygons
    });
    // Result will come via WebSocket
  } catch (e) {
    setStatus('Plan request failed: ' + e.message, 'err');
    resetPlanButton();
  }
}

function resetPlanButton() {
  planning = false;
  const btn = $('planBtn');
  btn.classList.remove('computing');
  btn.innerHTML = '<span class="spinner"></span> Start Planning';
}

// ===================================================================
// WEBSOCKET
// ===================================================================
let ws = null;
let wsReconnectTimer = null;

function connectWS() {
  if (ws && ws.readyState === WebSocket.OPEN) return;

  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url = `${proto}//${location.host}/ws?token=${token}`;

  try {
    ws = new WebSocket(url);
  } catch (e) {
    setStatus('WebSocket connection failed', 'err');
    return;
  }

  ws.onopen = () => {
    setStatus('Connected', 'ok');
    if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; }
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleWSMessage(msg);
    } catch (e) { /* ignore */ }
  };

  ws.onclose = () => {
    setStatus('Disconnected - reconnecting in 3s...', 'err');
    wsReconnectTimer = setTimeout(connectWS, 3000);
  };

  ws.onerror = () => {
    // onclose will fire next
  };
}

function handleWSMessage(msg) {
  switch (msg.type) {
    case 'plan_status':
      setStatus(`Plan: ${msg.status}`, '');
      break;

    case 'plan_result':
      resetPlanButton();
      if (msg.success) {
        setStatus(`Plan succeeded: ${msg.point_count || '?'} pts, ${(msg.path_length || 0).toFixed(2)}m`, 'ok');
        $('resultBox').className = 'result-box success';
        $('resultBox').textContent = `Path: ${msg.point_count || 0} points, ${(msg.path_length || 0).toFixed(2)}m total`;
        $('resultBox').style.display = 'block';
        // Render path and boxes from plan_result (authoritative source)
        if (msg.path) renderPath(msg.path);
        if (msg.boxes) renderBoxes(msg.boxes);
      } else {
        setStatus(`Plan failed: ${msg.message || 'unknown'}`, 'err');
        $('resultBox').className = 'result-box fail';
        $('resultBox').textContent = `Failed: ${msg.message || 'unknown'}`;
        $('resultBox').style.display = 'block';
      }
      break;

    case 'path_update':
      renderPath(msg.data || []);
      break;

    case 'boxes_update':
      renderBoxes(msg.data || []);
      break;

    case 'planning_result':
      // Raw planning result text from ROS2
      break;

    case 'polygons_updated':
      polygons = msg.polygons || [];
      renderAllPolygons();
      updatePolyInfo();
      break;

    case 'vehicle_config_updated':
      activePreset = msg.preset || activePreset;
      if (msg.params) vehicleParams = msg.params;
      renderVehicleParams(msg.params || {});
      break;
  }
}

// ---- Render path and boxes on map ----
function toggleBoxes() {
  boxesVisible = $('showBoxes').checked;
  if (boxesVisible) {
    // Re-add the layer; existing content stays intact
    map.addLayer(boxesLayer);
  } else {
    map.removeLayer(boxesLayer);
  }
}

function renderPath(pathData) {
  pathLayer.clearLayers();
  if (!pathData || pathData.length < 2) return;

  const latlngs = pathData.map(p => [p.y, p.x]);
  pathPolyline = L.polyline(latlngs, {
    color: '#38a169',
    weight: 3.5,
    opacity: 0.85,
  }).addTo(pathLayer);

  // Direction arrow at each point
  pathData.forEach((p, i) => {
    if (i % Math.max(1, Math.floor(pathData.length / 20)) !== 0) return;
    const yawRad = (p.yaw_deg || 0) * Math.PI / 180;
    const arrowLen = 0.6;
    const dx = Math.cos(yawRad) * arrowLen;
    const dy = Math.sin(yawRad) * arrowLen;
    const tip = [p.y + dy, p.x + dx];
    L.polyline([[p.y, p.x], tip], {
      color: '#2f855a',
      weight: 2,
      opacity: 0.7,
    }).addTo(pathLayer);
  });
}

function renderBoxes(boxesData) {
  boxesLayer.clearLayers();
  if (!boxesData || boxesData.length === 0) return;

  const TARGET = 23;
  const n = boxesData.length;
  // Dynamic step: ceil so we never exceed TARGET boxes
  const step = Math.max(1, Math.ceil(n / TARGET));

  // Collect indices: evenly spaced, always include first and last
  const indices = new Set();
  for (let i = 0; i < n; i += step) {
    indices.add(i);
  }
  indices.add(n - 1);  // ensure goal-pose box is always shown

  // Render in index order
  [...indices].sort((a, b) => a - b).forEach(i => {
    const box = boxesData[i];

    if (box.corners && box.corners.length >= 3) {
      const latlngs = box.corners.map(c => [c.y, c.x]);
      L.polygon(latlngs, {
        color: '#38a169',
        weight: 1.5,
        fillColor: '#68d391',
        fillOpacity: 0.3,
      }).addTo(boxesLayer);
    } else {
      const BOX_W = vehicleParams['vehicle.length'] || 12;
      const BOX_H = vehicleParams['vehicle.width'] || 2.6;
      const hw = BOX_W / 2, hh = BOX_H / 2;
      const corners = [
        [box.y - hh, box.x - hw],
        [box.y + hh, box.x - hw],
        [box.y + hh, box.x + hw],
        [box.y - hh, box.x + hw],
      ];
      L.polygon(corners, {
        color: '#38a169',
        weight: 1.5,
        fillColor: '#68d391',
        fillOpacity: 0.3,
      }).addTo(boxesLayer);
    }
  });
}

// ===================================================================
// INIT
// ===================================================================
document.addEventListener('DOMContentLoaded', async () => {
  initMap();
  setMode('obstacle');
  connectWS();
  await Promise.all([
    loadSceneList(),
    loadVehicleConfig(),
  ]);

  // Ping keepalive
  setInterval(() => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: 'ping' }));
    }
  }, 30000);

  // Auto-load customer_map.json if available
  const sel = $('sceneSelect');
  if (sel.options.length > 0) {
    for (let i = 0; i < sel.options.length; i++) {
      if (sel.options[i].value === 'customer_map.json') {
        sel.selectedIndex = i;
        loadSelectedScene();
        break;
      }
    }
  }

  // Listen for start/goal input changes
  ['startX', 'startY', 'startYaw'].forEach(id => {
    $(id).addEventListener('change', updateStartMarkerFromInputs);
  });
  ['goalX', 'goalY', 'goalYaw'].forEach(id => {
    $(id).addEventListener('change', updateGoalMarkerFromInputs);
  });
});

function updateStartMarkerFromInputs() {
  const x = parseFloat($('startX').value);
  const y = parseFloat($('startY').value);
  const yaw = parseFloat($('startYaw').value);
  if (isNaN(x) || isNaN(y)) return;
  setStart([x, y], isNaN(yaw) ? 0 : yaw);
}

function updateGoalMarkerFromInputs() {
  const x = parseFloat($('goalX').value);
  const y = parseFloat($('goalY').value);
  const yaw = parseFloat($('goalYaw').value);
  if (isNaN(x) || isNaN(y)) return;
  setGoal([x, y], isNaN(yaw) ? 0 : yaw);
}
