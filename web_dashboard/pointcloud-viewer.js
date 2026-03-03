// ROS2 PointCloud2 Viewer for FluentVision
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.158.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.158.0/examples/jsm/controls/OrbitControls.js';

const STYLE_ID = 'fv-pointcloud-viewer-style';

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function ensureViewerStyle() {
  if (document.getElementById(STYLE_ID)) return;

  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = `
  .fv-pc-root {
    position: relative;
    width: 100%;
    height: 100%;
    overflow: hidden;
    background:
      radial-gradient(circle at 15% 12%, rgba(0, 220, 255, 0.08), transparent 36%),
      radial-gradient(circle at 82% 88%, rgba(0, 180, 255, 0.08), transparent 35%),
      linear-gradient(160deg, #0a1118 0%, #04090f 100%);
    color: #d6ecff;
    font-family: "IBM Plex Sans", "Fira Sans", "Segoe UI", sans-serif;
  }

  .fv-pc-root canvas {
    display: block;
    width: 100% !important;
    height: 100% !important;
  }

  .fv-pc-hud {
    position: absolute;
    top: 10px;
    right: 10px;
    display: grid;
    gap: 5px;
    min-width: 185px;
    padding: 10px 12px;
    border-radius: 10px;
    border: 1px solid rgba(0, 207, 255, 0.25);
    background: rgba(2, 12, 18, 0.72);
    backdrop-filter: blur(7px);
    box-shadow: 0 10px 25px rgba(0, 0, 0, 0.35);
    pointer-events: none;
    z-index: 3;
  }

  .fv-pc-hud-title {
    font-size: 11px;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: #79deff;
    font-weight: 700;
  }

  .fv-pc-hud-line {
    display: flex;
    justify-content: space-between;
    gap: 10px;
    font-size: 11px;
    color: rgba(214, 236, 255, 0.92);
  }

  .fv-pc-hud-key {
    color: rgba(132, 181, 209, 0.9);
  }

  .fv-pc-panel {
    position: absolute;
    left: 10px;
    top: 10px;
    width: 242px;
    max-height: calc(100% - 20px);
    overflow: auto;
    border-radius: 12px;
    border: 1px solid rgba(0, 207, 255, 0.26);
    background: rgba(2, 11, 17, 0.80);
    backdrop-filter: blur(8px);
    box-shadow: 0 12px 28px rgba(0, 0, 0, 0.42);
    z-index: 4;
    transition: transform 180ms ease, opacity 180ms ease;
  }

  .fv-pc-panel.collapsed {
    transform: translateX(-206px);
    opacity: 0.94;
  }

  .fv-pc-panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    padding: 10px;
    border-bottom: 1px solid rgba(0, 207, 255, 0.16);
  }

  .fv-pc-panel-title {
    font-size: 11px;
    letter-spacing: 0.09em;
    text-transform: uppercase;
    font-weight: 700;
    color: #80ddff;
  }

  .fv-pc-btn {
    appearance: none;
    border: 1px solid rgba(0, 207, 255, 0.35);
    border-radius: 8px;
    background: linear-gradient(180deg, rgba(8, 32, 47, 0.95), rgba(3, 14, 22, 0.95));
    color: #d7efff;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.03em;
    padding: 6px 9px;
    cursor: pointer;
    transition: transform 120ms ease, border-color 120ms ease, box-shadow 120ms ease;
  }

  .fv-pc-btn:hover {
    transform: translateY(-1px);
    border-color: rgba(88, 227, 255, 0.8);
    box-shadow: 0 0 0 1px rgba(88, 227, 255, 0.35) inset;
  }

  .fv-pc-panel-body {
    padding: 10px;
    display: grid;
    gap: 9px;
  }

  .fv-pc-row {
    display: grid;
    gap: 5px;
  }

  .fv-pc-row-label {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 11px;
    color: rgba(160, 208, 232, 0.95);
  }

  .fv-pc-value {
    font-family: "JetBrains Mono", "IBM Plex Mono", monospace;
    color: #d3f3ff;
  }

  .fv-pc-range {
    width: 100%;
    accent-color: #26d7ff;
    cursor: pointer;
  }

  .fv-pc-select {
    width: 100%;
    border-radius: 8px;
    border: 1px solid rgba(0, 207, 255, 0.33);
    background: rgba(7, 26, 39, 0.9);
    color: #dcf1ff;
    font-size: 12px;
    padding: 6px 8px;
    outline: none;
  }

  .fv-pc-checkbox-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-size: 12px;
    color: rgba(194, 225, 243, 0.95);
  }

  .fv-pc-checkbox-row input {
    accent-color: #28d8ff;
    cursor: pointer;
  }

  .fv-pc-view-buttons {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 6px;
  }

  .fv-pc-status-ok { color: #79ffb0; }
  .fv-pc-status-warn { color: #ffd86f; }
  .fv-pc-status-err { color: #ff8f8f; }

  @media (max-width: 880px) {
    .fv-pc-panel {
      width: 224px;
    }
    .fv-pc-panel.collapsed {
      transform: translateX(-188px);
    }
  }
  `;

  document.head.appendChild(style);
}

function fieldOffset(fields, name) {
  const f = (fields || []).find((it) => it.name === name);
  return f ? f.offset : -1;
}

function decodeBase64ToBytes(base64) {
  const raw = atob(base64);
  const out = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) out[i] = raw.charCodeAt(i);
  return out;
}

function packedRgbToFloat3(value) {
  const r = ((value >> 16) & 0xff) / 255.0;
  const g = ((value >> 8) & 0xff) / 255.0;
  const b = (value & 0xff) / 255.0;
  return [r, g, b];
}

class PointCloudViewer {
  constructor(container, options = {}) {
    this.container = container;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.pointCloud = null;
    this.gridHelper = null;
    this.axesHelper = null;

    this.ros = null;
    this.topic = null;
    this.topicName = '';

    this.lastCloudMessage = null;
    this.lastMsgWallMs = 0;
    this.firstFitDone = false;

    this.settings = {
      pointSize: 0.011,
      density: 68,
      maxPoints: 90000,
      worldScale: 1.0,
      colorMode: 'rgb',
      minRange: 0.0,
      maxRange: 4.5,
      autoRotate: false,
      showGrid: true,
      showAxes: true,
      ...options,
    };

    this.stats = {
      renderFps: 0,
      topicHz: 0,
      pointsDrawn: 0,
      pointsRaw: 0,
      latencyMs: 0,
      status: 'idle',
      statusText: 'Idle',
    };

    this.frameCounter = 0;
    this.lastFpsTick = performance.now();
    this.lastAnimationHandle = 0;
    this.refreshTimer = 0;

    this.boundResize = this.onResize.bind(this);
    this.boundAnimate = this.animate.bind(this);

    this.ui = {
      hudStatus: null,
      hudFps: null,
      hudRx: null,
      hudPoints: null,
      hudLatency: null,
      hudMode: null,
      panel: null,
      values: {},
    };

    this.init();
  }

  init() {
    ensureViewerStyle();

    this.container.classList.add('fv-pc-root');

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x02070d);

    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);

    this.camera = new THREE.PerspectiveCamera(68, width / height, 0.03, 200.0);
    this.camera.position.set(1.9, 1.4, 2.7);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.setSize(width, height);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.maxDistance = 30;
    this.controls.minDistance = 0.2;
    this.controls.target.set(0, 0, 0);

    this.addHelpers();
    this.setupUi();
    this.updateSettingLabels();
    this.setStatus('idle', 'Viewer ready');

    window.addEventListener('resize', this.boundResize);
    this.lastAnimationHandle = requestAnimationFrame(this.boundAnimate);
  }

  addHelpers() {
    this.gridHelper = new THREE.GridHelper(3.2, 20, 0x2c91ae, 0x17313a);
    this.gridHelper.material.opacity = 0.45;
    this.gridHelper.material.transparent = true;
    this.gridHelper.visible = this.settings.showGrid;
    this.scene.add(this.gridHelper);

    this.axesHelper = new THREE.AxesHelper(0.55);
    this.axesHelper.visible = this.settings.showAxes;
    this.scene.add(this.axesHelper);

    const ambient = new THREE.AmbientLight(0xffffff, 0.56);
    this.scene.add(ambient);

    const key = new THREE.DirectionalLight(0x88dfff, 0.65);
    key.position.set(2.4, 2.4, 1.8);
    this.scene.add(key);

    const rim = new THREE.DirectionalLight(0x2fd2ff, 0.22);
    rim.position.set(-2.0, 1.5, -2.1);
    this.scene.add(rim);
  }

  setupUi() {
    const hud = document.createElement('div');
    hud.className = 'fv-pc-hud';
    hud.innerHTML = `
      <div class="fv-pc-hud-title">PointCloud Telemetry</div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Status</span><span data-k="status">--</span></div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Render FPS</span><span data-k="fps">--</span></div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Topic Hz</span><span data-k="rx">--</span></div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Points</span><span data-k="points">--</span></div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Latency</span><span data-k="lat">--</span></div>
      <div class="fv-pc-hud-line"><span class="fv-pc-hud-key">Color</span><span data-k="mode">--</span></div>
    `;
    this.container.appendChild(hud);

    this.ui.hudStatus = hud.querySelector('[data-k="status"]');
    this.ui.hudFps = hud.querySelector('[data-k="fps"]');
    this.ui.hudRx = hud.querySelector('[data-k="rx"]');
    this.ui.hudPoints = hud.querySelector('[data-k="points"]');
    this.ui.hudLatency = hud.querySelector('[data-k="lat"]');
    this.ui.hudMode = hud.querySelector('[data-k="mode"]');

    const panel = document.createElement('div');
    panel.className = 'fv-pc-panel';

    const header = document.createElement('div');
    header.className = 'fv-pc-panel-header';

    const title = document.createElement('div');
    title.className = 'fv-pc-panel-title';
    title.textContent = 'PointCloud Control';

    const collapseBtn = document.createElement('button');
    collapseBtn.className = 'fv-pc-btn';
    collapseBtn.textContent = 'Hide';
    collapseBtn.onclick = () => {
      panel.classList.toggle('collapsed');
      collapseBtn.textContent = panel.classList.contains('collapsed') ? 'Show' : 'Hide';
    };

    header.appendChild(title);
    header.appendChild(collapseBtn);

    const body = document.createElement('div');
    body.className = 'fv-pc-panel-body';

    body.appendChild(this.createSliderRow({
      key: 'pointSize',
      label: 'Point Size',
      min: 0.002,
      max: 0.08,
      step: 0.001,
      format: (v) => v.toFixed(3),
      onInput: (v) => {
        this.settings.pointSize = v;
        if (this.pointCloud && this.pointCloud.material) {
          this.pointCloud.material.size = v;
          this.pointCloud.material.needsUpdate = true;
        }
      },
    }));

    body.appendChild(this.createSliderRow({
      key: 'density',
      label: 'Density',
      min: 10,
      max: 100,
      step: 1,
      format: (v) => `${Math.round(v)}%`,
      onInput: (v) => {
        this.settings.density = Math.round(v);
        this.scheduleRefresh();
      },
    }));

    body.appendChild(this.createSliderRow({
      key: 'maxPoints',
      label: 'Max Points',
      min: 10000,
      max: 220000,
      step: 5000,
      format: (v) => Math.round(v).toLocaleString(),
      onInput: (v) => {
        this.settings.maxPoints = Math.round(v);
        this.scheduleRefresh();
      },
    }));

    body.appendChild(this.createSliderRow({
      key: 'worldScale',
      label: 'Scale',
      min: 0.25,
      max: 2.5,
      step: 0.01,
      format: (v) => `${v.toFixed(2)}x`,
      onInput: (v) => {
        this.settings.worldScale = v;
        this.scheduleRefresh();
      },
    }));

    body.appendChild(this.createSliderRow({
      key: 'maxRange',
      label: 'Max Range',
      min: 0.5,
      max: 10.0,
      step: 0.1,
      format: (v) => `${v.toFixed(1)}m`,
      onInput: (v) => {
        this.settings.maxRange = v;
        if (this.settings.minRange > v) this.settings.minRange = v;
        this.scheduleRefresh();
      },
    }));

    body.appendChild(this.createSelectRow({
      key: 'colorMode',
      label: 'Color Mode',
      options: [
        { value: 'depth', label: 'Depth' },
        { value: 'height', label: 'Height' },
        { value: 'rgb', label: 'RGB' },
      ],
      onChange: (v) => {
        this.settings.colorMode = v;
        this.scheduleRefresh();
      },
    }));

    body.appendChild(this.createCheckboxRow('autoRotate', 'Auto Rotate', (checked) => {
      this.settings.autoRotate = checked;
      this.controls.autoRotate = checked;
      this.controls.autoRotateSpeed = 0.7;
    }));

    body.appendChild(this.createCheckboxRow('showGrid', 'Show Grid', (checked) => {
      this.settings.showGrid = checked;
      if (this.gridHelper) this.gridHelper.visible = checked;
    }));

    body.appendChild(this.createCheckboxRow('showAxes', 'Show Axes', (checked) => {
      this.settings.showAxes = checked;
      if (this.axesHelper) this.axesHelper.visible = checked;
    }));

    const btnWrap = document.createElement('div');
    btnWrap.className = 'fv-pc-view-buttons';

    const views = [
      { label: 'Reset', fn: () => this.resetView() },
      { label: 'Top', fn: () => this.setTopView() },
      { label: 'Front', fn: () => this.setFrontView() },
      { label: 'Side', fn: () => this.setSideView() },
    ];

    for (const v of views) {
      const b = document.createElement('button');
      b.className = 'fv-pc-btn';
      b.textContent = v.label;
      b.onclick = v.fn;
      btnWrap.appendChild(b);
    }

    body.appendChild(btnWrap);

    panel.appendChild(header);
    panel.appendChild(body);

    this.container.appendChild(panel);
    this.ui.panel = panel;
  }

  createSliderRow({ key, label, min, max, step, format, onInput }) {
    const row = document.createElement('div');
    row.className = 'fv-pc-row';

    const header = document.createElement('div');
    header.className = 'fv-pc-row-label';

    const labelEl = document.createElement('span');
    labelEl.textContent = label;

    const valueEl = document.createElement('span');
    valueEl.className = 'fv-pc-value';
    this.ui.values[key] = { el: valueEl, format };

    header.appendChild(labelEl);
    header.appendChild(valueEl);

    const input = document.createElement('input');
    input.className = 'fv-pc-range';
    input.type = 'range';
    input.min = String(min);
    input.max = String(max);
    input.step = String(step);
    input.value = String(this.settings[key]);

    input.addEventListener('input', () => {
      const value = Number(input.value);
      onInput(value);
      this.updateSettingLabel(key);
    });

    row.appendChild(header);
    row.appendChild(input);
    return row;
  }

  createSelectRow({ key, label, options, onChange }) {
    const row = document.createElement('div');
    row.className = 'fv-pc-row';

    const labelRow = document.createElement('div');
    labelRow.className = 'fv-pc-row-label';

    const labelEl = document.createElement('span');
    labelEl.textContent = label;

    labelRow.appendChild(labelEl);

    const sel = document.createElement('select');
    sel.className = 'fv-pc-select';

    for (const opt of options) {
      const el = document.createElement('option');
      el.value = opt.value;
      el.textContent = opt.label;
      if (opt.value === this.settings[key]) el.selected = true;
      sel.appendChild(el);
    }

    sel.onchange = () => onChange(sel.value);

    row.appendChild(labelRow);
    row.appendChild(sel);
    return row;
  }

  createCheckboxRow(key, label, onChange) {
    const row = document.createElement('label');
    row.className = 'fv-pc-checkbox-row';

    const text = document.createElement('span');
    text.textContent = label;

    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = !!this.settings[key];
    input.onchange = () => onChange(input.checked);

    row.appendChild(text);
    row.appendChild(input);
    return row;
  }

  connectToROS(rosbridgeUrl = 'ws://localhost:9090') {
    if (typeof ROSLIB === 'undefined') {
      this.loadROSLIB().then(() => this.connectToROS(rosbridgeUrl)).catch((err) => {
        console.error('Failed to load ROSLIB:', err);
        this.setStatus('error', 'ROSLIB load failed');
      });
      return;
    }

    this.ros = new ROSLIB.Ros({ url: rosbridgeUrl });

    this.ros.on('connection', () => {
      this.setStatus('ok', 'ROS connected');
    });

    this.ros.on('error', (error) => {
      console.error('ROS connection error:', error);
      this.setStatus('error', 'ROS connection error');
    });

    this.ros.on('close', () => {
      this.setStatus('warn', 'ROS disconnected');
    });
  }

  subscribeToPointCloud(topicName = '/camera/depth/points') {
    if (!this.ros) {
      console.error('ROS is not connected');
      this.setStatus('warn', 'ROS not connected');
      return;
    }

    if (this.topic) {
      this.topic.unsubscribe();
      this.topic = null;
    }

    this.topicName = topicName;

    this.topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'sensor_msgs/PointCloud2',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 0,
    });

    this.topic.subscribe((message) => {
      const now = performance.now();
      if (this.lastMsgWallMs > 0) {
        const dt = now - this.lastMsgWallMs;
        if (dt > 0) {
          const hz = 1000.0 / dt;
          this.stats.topicHz = this.stats.topicHz > 0 ? this.stats.topicHz * 0.84 + hz * 0.16 : hz;
        }
      }
      this.lastMsgWallMs = now;

      this.lastCloudMessage = message;
      this.processPointCloud(message);
    });

    this.setStatus('ok', `Subscribed: ${topicName}`);
  }

  processPointCloud(msg) {
    const parsed = this.parsePointCloud2(msg);
    if (!parsed || parsed.count === 0) {
      this.stats.pointsDrawn = 0;
      this.updateHud();
      return;
    }

    this.updatePointCloud(parsed.positions, parsed.colors, parsed.count);

    this.stats.pointsDrawn = parsed.count;
    this.stats.pointsRaw = parsed.total;

    const stampMs = this.extractStampMs(msg);
    this.stats.latencyMs = stampMs > 0 ? clamp(Date.now() - stampMs, 0, 99999) : 0;

    this.updateHud();
  }

  parsePointCloud2(msg) {
    const fields = msg.fields || [];
    const xOffset = fieldOffset(fields, 'x');
    const yOffset = fieldOffset(fields, 'y');
    const zOffset = fieldOffset(fields, 'z');

    if (xOffset < 0 || yOffset < 0 || zOffset < 0) {
      console.error('PointCloud2 missing x/y/z fields');
      return null;
    }

    const rgbOffset = fieldOffset(fields, 'rgb');
    const rgbaOffset = fieldOffset(fields, 'rgba');
    const hasPackedColor = rgbOffset >= 0 || rgbaOffset >= 0;
    const packedOffset = rgbOffset >= 0 ? rgbOffset : rgbaOffset;

    let bytes;
    if (typeof msg.data === 'string') {
      bytes = decodeBase64ToBytes(msg.data);
    } else if (msg.data instanceof ArrayBuffer) {
      bytes = new Uint8Array(msg.data);
    } else if (ArrayBuffer.isView(msg.data)) {
      bytes = new Uint8Array(msg.data.buffer, msg.data.byteOffset, msg.data.byteLength);
    } else if (Array.isArray(msg.data)) {
      bytes = Uint8Array.from(msg.data);
    } else {
      console.error('Unsupported PointCloud2 data format');
      return null;
    }

    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const pointStep = Math.max(1, msg.point_step || 0);
    const total = Math.floor(view.byteLength / pointStep);
    if (total <= 0) return null;

    const density = clamp(this.settings.density, 10, 100);
    const sampleStep = Math.max(1, Math.round((110 - density) / 10));
    const maxPoints = Math.max(1000, this.settings.maxPoints | 0);
    const scale = this.settings.worldScale;
    const mode = this.settings.colorMode;
    const minRange = this.settings.minRange;
    const maxRange = Math.max(minRange + 0.1, this.settings.maxRange);

    const pos = [];
    const col = [];
    const little = msg.is_bigendian ? false : true;

    for (let i = 0; i < total; i += sampleStep) {
      const base = i * pointStep;

      const x = view.getFloat32(base + xOffset, little);
      const y = view.getFloat32(base + yOffset, little);
      const z = view.getFloat32(base + zOffset, little);

      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;

      const dist = Math.sqrt(x * x + y * y + z * z);
      if (!Number.isFinite(dist) || dist < minRange || dist > maxRange) continue;

      const sx = x * scale;
      const sy = y * scale;
      const sz = z * scale;

      pos.push(sx, sy, sz);

      let r = 0.30;
      let g = 0.75;
      let b = 1.0;

      if (mode === 'rgb' && hasPackedColor) {
        const packed = view.getUint32(base + packedOffset, little);
        [r, g, b] = packedRgbToFloat3(packed);
      } else if (mode === 'height') {
        const t = clamp((sy + 1.2) / 2.4, 0.0, 1.0);
        r = clamp(t * 1.2, 0.0, 1.0);
        g = clamp(1.0 - Math.abs(t - 0.5) * 1.8, 0.0, 1.0);
        b = clamp(1.0 - t * 0.8, 0.0, 1.0);
      } else {
        const t = clamp(dist / maxRange, 0.0, 1.0);
        r = clamp(1.5 * t, 0.0, 1.0);
        g = clamp(1.2 * (1.0 - Math.abs(t - 0.5) * 2.0), 0.0, 1.0);
        b = clamp(1.0 - t * 0.9, 0.0, 1.0);
      }

      col.push(r, g, b);

      if ((pos.length / 3) >= maxPoints) break;
    }

    return {
      positions: new Float32Array(pos),
      colors: new Float32Array(col),
      count: pos.length / 3,
      total,
    };
  }

  updatePointCloud(positions, colors, count) {
    if (!this.pointCloud) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

      const material = new THREE.PointsMaterial({
        size: this.settings.pointSize,
        vertexColors: true,
        sizeAttenuation: true,
        transparent: true,
        opacity: 0.98,
      });

      this.pointCloud = new THREE.Points(geometry, material);
      this.scene.add(this.pointCloud);
      this.fitCameraToCloud();
      return;
    }

    const geometry = this.pointCloud.geometry;
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.computeBoundingSphere();

    this.pointCloud.material.size = this.settings.pointSize;
    this.pointCloud.material.needsUpdate = true;

    if (!this.firstFitDone && count > 0) {
      this.fitCameraToCloud();
    }
  }

  fitCameraToCloud() {
    if (!this.pointCloud || !this.pointCloud.geometry) return;

    this.pointCloud.geometry.computeBoundingSphere();
    const bs = this.pointCloud.geometry.boundingSphere;
    if (!bs || !Number.isFinite(bs.radius) || bs.radius <= 1e-6) return;

    const center = bs.center;
    const radius = Math.max(0.3, bs.radius);

    this.controls.target.set(center.x, center.y, center.z);
    this.camera.position.set(center.x + radius * 1.8, center.y + radius * 1.2, center.z + radius * 1.8);
    this.camera.near = Math.max(0.01, radius * 0.02);
    this.camera.far = Math.max(30, radius * 60.0);
    this.camera.updateProjectionMatrix();
    this.controls.update();

    this.firstFitDone = true;
  }

  extractStampMs(msg) {
    const stamp = msg?.header?.stamp;
    if (!stamp) return 0;

    const sec = (typeof stamp.sec === 'number') ? stamp.sec :
      (typeof stamp.secs === 'number') ? stamp.secs : 0;
    const nsec = (typeof stamp.nanosec === 'number') ? stamp.nanosec :
      (typeof stamp.nsecs === 'number') ? stamp.nsecs : 0;

    if (!sec && !nsec) return 0;
    return sec * 1000 + nsec / 1e6;
  }

  scheduleRefresh() {
    if (this.refreshTimer) clearTimeout(this.refreshTimer);
    this.refreshTimer = setTimeout(() => {
      this.refreshTimer = 0;
      if (this.lastCloudMessage) this.processPointCloud(this.lastCloudMessage);
      this.updateSettingLabels();
    }, 65);
  }

  resetView() {
    this.firstFitDone = false;
    this.fitCameraToCloud();
  }

  setTopView() {
    const t = this.controls.target;
    this.camera.position.set(t.x, t.y + 3.0, t.z + 0.001);
    this.camera.up.set(0, 0, -1);
    this.controls.update();
  }

  setFrontView() {
    const t = this.controls.target;
    this.camera.position.set(t.x, t.y + 0.2, t.z + 3.0);
    this.camera.up.set(0, 1, 0);
    this.controls.update();
  }

  setSideView() {
    const t = this.controls.target;
    this.camera.position.set(t.x + 3.0, t.y + 0.2, t.z);
    this.camera.up.set(0, 1, 0);
    this.controls.update();
  }

  setStatus(kind, text) {
    this.stats.status = kind;
    this.stats.statusText = text;
    this.updateHud();
  }

  updateSettingLabel(key) {
    const info = this.ui.values[key];
    if (!info || !info.el) return;
    info.el.textContent = info.format ? info.format(this.settings[key]) : String(this.settings[key]);
  }

  updateSettingLabels() {
    Object.keys(this.ui.values).forEach((key) => this.updateSettingLabel(key));
  }

  updateHud() {
    if (!this.ui.hudStatus) return;

    const statusClass = this.stats.status === 'ok'
      ? 'fv-pc-status-ok'
      : this.stats.status === 'error'
        ? 'fv-pc-status-err'
        : 'fv-pc-status-warn';

    this.ui.hudStatus.className = statusClass;
    this.ui.hudStatus.textContent = this.stats.statusText;
    this.ui.hudFps.textContent = this.stats.renderFps > 0 ? this.stats.renderFps.toFixed(1) : '--';
    this.ui.hudRx.textContent = this.stats.topicHz > 0 ? this.stats.topicHz.toFixed(1) : '--';
    this.ui.hudPoints.textContent = `${this.stats.pointsDrawn.toLocaleString()} / ${this.stats.pointsRaw.toLocaleString()}`;
    this.ui.hudLatency.textContent = this.stats.latencyMs > 0 ? `${Math.round(this.stats.latencyMs)} ms` : '--';
    this.ui.hudMode.textContent = this.settings.colorMode;
  }

  animate() {
    this.lastAnimationHandle = requestAnimationFrame(this.boundAnimate);

    this.controls.autoRotate = this.settings.autoRotate;
    this.controls.autoRotateSpeed = 0.7;
    this.controls.update();

    this.renderer.render(this.scene, this.camera);

    this.frameCounter += 1;
    const now = performance.now();
    const elapsed = now - this.lastFpsTick;
    if (elapsed >= 500) {
      this.stats.renderFps = (this.frameCounter * 1000.0) / elapsed;
      this.frameCounter = 0;
      this.lastFpsTick = now;
      this.updateHud();
    }
  }

  onResize() {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);

    if (!this.camera || !this.renderer) return;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  showStatus(message, type = 'info') {
    console.log(`[PointCloud ${type}] ${message}`);
  }

  async loadROSLIB() {
    return new Promise((resolve, reject) => {
      const script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js';
      script.onload = resolve;
      script.onerror = reject;
      document.head.appendChild(script);
    });
  }

  dispose() {
    if (this.topic) {
      this.topic.unsubscribe();
      this.topic = null;
    }

    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }

    if (this.refreshTimer) {
      clearTimeout(this.refreshTimer);
      this.refreshTimer = 0;
    }

    if (this.lastAnimationHandle) {
      cancelAnimationFrame(this.lastAnimationHandle);
      this.lastAnimationHandle = 0;
    }

    window.removeEventListener('resize', this.boundResize);

    if (this.pointCloud) {
      this.scene.remove(this.pointCloud);
      this.pointCloud.geometry?.dispose();
      this.pointCloud.material?.dispose();
      this.pointCloud = null;
    }

    if (this.renderer) {
      this.renderer.dispose();
      if (this.renderer.domElement && this.renderer.domElement.parentNode) {
        this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
      }
      this.renderer = null;
    }

    if (this.controls) {
      this.controls.dispose();
      this.controls = null;
    }
  }
}

// StreamManagerに統合するための拡張
export function addPointCloudSupport(streamManager) {
  const originalCreateStreamElement = streamManager.createStreamElement;
  const originalRemoveStream = streamManager.removeStream;

  streamManager.createStreamElement = function (container, config) {
    if (config.type !== 'pointcloud') {
      originalCreateStreamElement.call(this, container, config);
      return;
    }

    const viewer = new PointCloudViewer(container);

    if (config.rosbridge) {
      viewer.connectToROS(config.rosbridge);
      viewer.subscribeToPointCloud(config.rosTopic || '/camera/depth/points');
    } else {
      viewer.generateDemoPointCloud?.();
    }

    container.pointCloudViewer = viewer;
  };

  streamManager.removeStream = function (streamId) {
    const tile = document.getElementById(streamId);
    if (tile) {
      const container = tile.querySelector('.stream-container');
      if (container && container.pointCloudViewer) {
        container.pointCloudViewer.dispose();
        container.pointCloudViewer = null;
      }
    }

    originalRemoveStream.call(this, streamId);
  };
}

// グローバルに公開
window.PointCloudViewer = PointCloudViewer;
window.addPointCloudSupport = addPointCloudSupport;
