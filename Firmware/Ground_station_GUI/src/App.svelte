<script>
  import { onMount, onDestroy } from "svelte";
  import * as THREE from "three";
  import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

  const states = ["IDLE", "READY", "ASCENT", "DESCENT", "LANDED", "SAFE"];

  const stateStatusMap = {
    IDLE: "Systems safe. Awaiting command.",
    READY: "Vehicle armed. Pre-launch checks complete.",
    ASCENT: "Launch detected. Stabilization active.",
    DESCENT: "Descent phase active.",
    LANDED: "Touchdown confirmed. Recovery mode.",
    SAFE: "System fault detected. Safe mode active."
  };

  let missionState = "IDLE";
  let systemStatus = stateStatusMap.IDLE;
  let transitionFlash = false;

  let telemetry = {
    pitch: "--",
    yaw: "--",
    roll: "--",
    altitude: "--",
    speed: "--",
    acceleration: "--"
  };

  let batteryVoltage = "--";
  let satelliteCount = "--";

  let errors = [];
  let elapsedMs = 0;
  let RSSI = 0;

  let gps = {
    lat: null,
    lon: null
  };

  let gpsHome = null;
  let gpsPath = [];

  const MAX_GPS_POINTS = 300;
  const MAP_RANGE_METERS = 150;

  const MAX_GRAPH_POINTS = 60;
  let pitchHistory = [];
  let yawHistory = [];
  let rollHistory = [];

  let threeContainer;
  let scene;
  let camera;
  let renderer;
  let controls;
  let rocketGroup;
  let animationFrame;
  let resizeObserver;

  function isRocketFlying() {
    return missionState === "ASCENT" || missionState === "DESCENT";
  }

  function flashTransition() {
    transitionFlash = true;
    setTimeout(() => (transitionFlash = false), 300);
  }

  function clearFlightTracking() {
    pitchHistory = [];
    yawHistory = [];
    rollHistory = [];
    gpsHome = null;
    gpsPath = [];
  }

  function setState(state, statusText = null) {
    const normalizedState = (state || "").trim().toUpperCase();
    const wasFlying = isRocketFlying();

    if (!states.includes(normalizedState)) {
      missionState = "SAFE";
      systemStatus = `Unknown mission state: ${state}`;
      clearFlightTracking();
      flashTransition();
      return;
    }

    const changed = missionState !== normalizedState;

    missionState = normalizedState;
    systemStatus = statusText?.trim() || stateStatusMap[normalizedState] || "Unknown system state.";

    const nowFlying = isRocketFlying();

    if (changed && nowFlying && !wasFlying) {
      clearFlightTracking();
    }

    if (changed) flashTransition();
  }

  function formatTimer(ms) {
    const totalMs = Number(ms) || 0;
    const totalSeconds = Math.floor(totalMs / 1000);
    const hours = String(Math.floor(totalSeconds / 3600)).padStart(2, "0");
    const minutes = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, "0");
    const seconds = String(totalSeconds % 60).padStart(2, "0");
    return `T+ ${hours}:${minutes}:${seconds}`;
  }

  function resetDisplay() {
    missionState = "IDLE";
    systemStatus = stateStatusMap.IDLE;

    telemetry = {
      pitch: "--",
      yaw: "--",
      roll: "--",
      altitude: "--",
      speed: "--",
      acceleration: "--"
    };

    batteryVoltage = "--";
    satelliteCount = "--";

    errors = [];
    elapsedMs = 0;
    RSSI = 0;

    gps = {
      lat: null,
      lon: null
    };

    clearFlightTracking();
  }

  function pushHistory(history, value) {
    if (!Number.isFinite(value)) return history;

    const updated = [...history, value];

    if (updated.length > MAX_GRAPH_POINTS) {
      updated.shift();
    }

    return updated;
  }

function buildGraphPoints(values, width = 100, height = 36, defaultRangeDeg = 30) {
  if (!values.length) return "";

  const maxAbsValue = Math.max(...values.map((value) => Math.abs(value)));
  const range = Math.max(defaultRangeDeg, maxAbsValue);
  const min = -range;
  const max = range;

  return values
    .map((value, index) => {
      const x = values.length === 1 ? 0 : (index / (values.length - 1)) * width;
      const y = height - ((value - min) / (max - min)) * height;
      return `${x},${y}`;
    })
    .join(" ");
}

  function isValidGps(lat, lon) {
    return (
      Number.isFinite(lat) &&
      Number.isFinite(lon) &&
      lat >= -90 &&
      lat <= 90 &&
      lon >= -180 &&
      lon <= 180 &&
      !(lat === 0 && lon === 0)
    );
  }

  function gpsToMapPoint(lat, lon, width = 100, height = 100) {
    if (!gpsHome) return null;

    const metersPerDegreeLat = 111320;
    const metersPerDegreeLon = 111320 * Math.cos((gpsHome.lat * Math.PI) / 180);

    const dx = (lon - gpsHome.lon) * metersPerDegreeLon;
    const dy = (lat - gpsHome.lat) * metersPerDegreeLat;

    const x = 50 + (dx / MAP_RANGE_METERS) * 45;
    const y = 50 - (dy / MAP_RANGE_METERS) * 45;

    return {
      x: Math.max(2, Math.min(98, x)),
      y: Math.max(2, Math.min(98, y)),
      dx,
      dy
    };
  }

  function buildGpsPath(points) {
    return points.map((point) => `${point.x},${point.y}`).join(" ");
  }

  function addGpsPoint(lat, lon) {
    if (!isValidGps(lat, lon)) return;

    gps = { lat, lon };

    if (!isRocketFlying()) return;

    if (!gpsHome) {
      gpsHome = { lat, lon };
    }

    const point = gpsToMapPoint(lat, lon);

    if (!point) return;

    gpsPath = [...gpsPath, point];

    if (gpsPath.length > MAX_GPS_POINTS) {
      gpsPath.shift();
    }
  }

  function addFlightGraphPoint(pitchDeg, yawDeg, rollDeg) {
    if (!isRocketFlying()) return;

    pitchHistory = pushHistory(pitchHistory, pitchDeg);
    yawHistory = pushHistory(yawHistory, yawDeg);
    rollHistory = pushHistory(rollHistory, rollDeg);
  }

  let port;
  let reader;
  let serialData = "";

  async function connectSerial() {
    try {
      port = await navigator.serial.requestPort();
      await port.open({ baudRate: 9600 });

      const decoder = new TextDecoderStream();
      port.readable.pipeTo(decoder.writable);
      reader = decoder.readable.getReader();

      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        if (!value) continue;

        if (value.includes("\r\n")) {
          const lines = value.split("\r\n");

          serialData += lines[0];
          parseTelemetryFromSerial(serialData);

          serialData = lines[1] || "";
        } else {
          serialData += value;
        }
      }
    } catch (err) {
      console.error("Serial error:", err);
      setState("SAFE", `Serial error: ${err.message || err}`);
    }
  }

  function parseTelemetryFromSerial(data) {
    const cleanHex = data.replace(/\s+/g, "");
    const raw_bytes = hexToBytes(cleanHex);

    if (raw_bytes.length < 54) {
      console.warn("Received incomplete telemetry data. Expected 54 bytes, got:", raw_bytes.length);
      return;
    }

    const view = new DataView(raw_bytes.buffer, raw_bytes.byteOffset, raw_bytes.byteLength);

    const q = {
      w: view.getFloat32(0, true),
      x: view.getFloat32(4, true),
      y: view.getFloat32(8, true),
      z: view.getFloat32(12, true)
    };

    const euler = toEulerAngles(q);
    const pitchDeg = euler.roll  * (180 / Math.PI);
    const rollDeg  = euler.pitch * (180 / Math.PI);
    const yawDeg   = euler.yaw   * (180 / Math.PI);

    telemetry.roll  = rollDeg.toFixed(1);
    telemetry.pitch = pitchDeg.toFixed(1);
    telemetry.yaw   = yawDeg.toFixed(1);

    const rx = view.getFloat32(16, true);
    const ry = view.getFloat32(20, true);
    const rz = view.getFloat32(24, true);

    const accelerationMg   = Math.sqrt(rx * rx + ry * ry + rz * rz);
    const accelerationMps2 = (accelerationMg * 9.80665) / 1000.0;

    telemetry.acceleration = accelerationMps2.toFixed(2);

    const stateCode = view.getUint8(28);

    switch (stateCode) {
      case 0: setState("READY");   break;
      case 1: setState("ASCENT");  break;
      case 2: setState("DESCENT"); break;
      case 3: setState("LANDED");  break;
      case 4: setState("IDLE");    break;
      default: setState("SAFE", `Unknown state: ${stateCode}`);
    }

    addFlightGraphPoint(pitchDeg, yawDeg, rollDeg);

    telemetry.altitude = view.getFloat32(29, true).toFixed(2);

    elapsedMs = Number((view.getUint32(33, true) / 1000).toFixed(0));

    const lat = view.getFloat32(37, true);
    const lon = view.getFloat32(41, true);
    addGpsPoint(lat, lon);

    telemetry.speed = view.getFloat32(45, true).toFixed(2);

    const batV = view.getFloat32(49, true);
    batteryVoltage = Number.isFinite(batV) ? batV.toFixed(2) : "--";

    const sat = view.getUint8(53);
    satelliteCount = sat;

    if (raw_bytes.length >= 55) {
      RSSI = view.getInt8(54);
    }
  }

  function hexToBytes(hexString) {
    if (!hexString || hexString.length % 2 !== 0) {
      return new Uint8Array(0);
    }

    const bytes = new Uint8Array(hexString.length / 2);

    for (let i = 0; i < hexString.length; i += 2) {
      const byte = parseInt(hexString.substring(i, i + 2), 16);

      if (Number.isNaN(byte)) {
        return new Uint8Array(0);
      }

      bytes[i / 2] = byte;
    }

    return bytes;
  }

  function toEulerAngles(q) {
    const angles = { roll: 0, pitch: 0, yaw: 0 };

    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = Math.atan2(sinr_cosp, cosr_cosp);

    const sinp = 2 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1) {
      angles.pitch = (Math.PI / 2) * Math.sign(sinp);
    } else {
      angles.pitch = Math.asin(sinp);
    }

    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = Math.atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  function angleValue(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num : 0;
  }

  function batClass(v) {
    const n = parseFloat(v);
    if (!Number.isFinite(n)) return "";
    if (n < 3.5)  return "crit";
    if (n < 3.7)  return "warn";
    return "ok";
  }

  function satClass(s) {
    const n = parseInt(s);
    if (!Number.isFinite(n)) return "";
    if (n < 4)  return "crit";
    if (n < 6)  return "warn";
    return "ok";
  }

  function createFin(material) {
    const finShape = new THREE.Shape();

    finShape.moveTo(0.0, -0.32);
    finShape.lineTo(0.0, 0.24);
    finShape.lineTo(0.26, 0.08);
    finShape.lineTo(0.34, -0.32);
    finShape.lineTo(0.0, -0.32);

    const finGeometry = new THREE.ExtrudeGeometry(finShape, {
      depth: 0.035,
      bevelEnabled: false
    });

    finGeometry.translate(0, 0, -0.0175);
    finGeometry.computeVertexNormals();

    const fin = new THREE.Mesh(finGeometry, material);
    return fin;
  }

  function createRocketModel() {
    const group = new THREE.Group();

    const whiteMaterial = new THREE.MeshStandardMaterial({
      color: 0xf8fafc, roughness: 0.34, metalness: 0.05, side: THREE.DoubleSide
    });
    const redMaterial = new THREE.MeshStandardMaterial({
      color: 0xdc2626, roughness: 0.32, metalness: 0.05
    });
    const blackMaterial = new THREE.MeshStandardMaterial({
      color: 0x111827, roughness: 0.42, metalness: 0.12, side: THREE.DoubleSide
    });

    const bodyRadius = 0.22;
    const bodyLength = 3.1;
    const bodyTop    = bodyLength / 2;
    const bodyBottom = -bodyLength / 2;

    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(bodyRadius, bodyRadius, bodyLength, 96),
      whiteMaterial
    );
    body.position.y = 0;
    group.add(body);

    const noseHeight = 0.85;
    const nosePoints = [];
    for (let i = 0; i <= 18; i++) {
      const t = i / 18;
      const y = t * noseHeight;
      const radius = bodyRadius * Math.sin((1 - t) * Math.PI / 2);
      nosePoints.push(new THREE.Vector2(radius, y));
    }
    const nose = new THREE.Mesh(new THREE.LatheGeometry(nosePoints, 96), redMaterial);
    nose.position.y = bodyTop;
    group.add(nose);

    const upperRing = new THREE.Mesh(
      new THREE.CylinderGeometry(bodyRadius + 0.008, bodyRadius + 0.008, 0.06, 96),
      blackMaterial
    );
    upperRing.position.y = 0.72;
    group.add(upperRing);

    const lowerRing = new THREE.Mesh(
      new THREE.CylinderGeometry(bodyRadius + 0.008, bodyRadius + 0.008, 0.06, 96),
      blackMaterial
    );
    lowerRing.position.y = -0.82;
    group.add(lowerRing);

    const finY = bodyBottom + 0.3;
    const finAngles = [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2];
    finAngles.forEach((angle) => {
      const fin = createFin(blackMaterial);
      fin.position.set(
        Math.cos(angle) * (bodyRadius + 0.002),
        finY,
        Math.sin(angle) * (bodyRadius + 0.002)
      );
      fin.rotation.y = -angle;
      group.add(fin);
    });

    return group;
  }

  function resizeThreeView() {
    if (!threeContainer || !camera || !renderer) return;
    const width  = threeContainer.clientWidth  || 260;
    const height = threeContainer.clientHeight || 200;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }

  function initThreeView() {
    if (!threeContainer) return;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);

    const width  = threeContainer.clientWidth  || 260;
    const height = threeContainer.clientHeight || 200;

    camera = new THREE.PerspectiveCamera(38, width / height, 0.1, 100);
    camera.position.set(2.45, 1.45, 3.35);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(width, height);
    threeContainer.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping  = true;
    controls.dampingFactor  = 0.08;
    controls.enablePan      = false;
    controls.minDistance    = 2.2;
    controls.maxDistance    = 7;
    controls.target.set(0, 0.15, 0);

    scene.add(new THREE.AmbientLight(0xffffff, 0.75));

    const mainLight = new THREE.DirectionalLight(0xffffff, 1.55);
    mainLight.position.set(4, 5, 3);
    scene.add(mainLight);

    const sideLight = new THREE.DirectionalLight(0x93c5fd, 0.4);
    sideLight.position.set(-3, 2, -4);
    scene.add(sideLight);

    rocketGroup = createRocketModel();
    scene.add(rocketGroup);

    const grid = new THREE.GridHelper(4, 8, 0x475569, 0x263241);
    grid.position.y = -1.65;
    grid.material.transparent = true;
    grid.material.opacity     = 0.4;
    grid.material.depthWrite  = false;
    scene.add(grid);

    resizeObserver = new ResizeObserver(resizeThreeView);
    resizeObserver.observe(threeContainer);

    animateThreeView();
  }

  function animateThreeView() {
    animationFrame = requestAnimationFrame(animateThreeView);

    if (rocketGroup) {
      const pitch = THREE.MathUtils.degToRad(angleValue(telemetry.pitch));
      const yaw   = THREE.MathUtils.degToRad(angleValue(telemetry.yaw));
      const roll  = THREE.MathUtils.degToRad(angleValue(telemetry.roll));

      rocketGroup.rotation.order = "ZXY";
      rocketGroup.rotation.x = pitch;
      rocketGroup.rotation.z = yaw;
      rocketGroup.rotation.y = roll;
    }

    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
  }

  onMount(() => { initThreeView(); });

  onDestroy(() => {
    if (animationFrame) cancelAnimationFrame(animationFrame);
    if (resizeObserver) resizeObserver.disconnect();
    if (controls) controls.dispose();
    if (renderer) {
      renderer.dispose();
      if (renderer.domElement?.parentNode) {
        renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    }
  });

  $: flying = missionState === "ASCENT" || missionState === "DESCENT";

  $: pitchGraphPoints = buildGraphPoints(pitchHistory);
  $: yawGraphPoints   = buildGraphPoints(yawHistory);
  $: rollGraphPoints  = buildGraphPoints(rollHistory);

  $: gpsPathPoints    = buildGpsPath(gpsPath);
  $: currentGpsPoint  = gpsPath.length > 0 ? gpsPath[gpsPath.length - 1] : null;
</script>

<div class="app">
  <header>
    <div class="header-left">
      <div class="title">GROUND STATION</div>
      <button on:click={connectSerial}>Connect to GS</button>
    </div>

    <div class="states">
      {#each states as state}
        <button
          class:selected={missionState === state}
          class:safe-state={state === "SAFE"}
          type="button"
        >
          {state}
        </button>
      {/each}
    </div>

    <div class="timer">{formatTimer(elapsedMs)}</div>
  </header>

  <div class="status-strip {transitionFlash ? 'flash' : ''}">
    {systemStatus}
  </div>

  <main>
    <!-- ================================================================
         LEFT COLUMN – 3D view + map
    ================================================================ -->
    <div class="column">
      <div class="panel view">
        <div class="panel-title">3D VEHICLE VIEW</div>
        <div class="three-view" bind:this={threeContainer}></div>
      </div>

      <div class="panel map">
        <div class="panel-title">MISSION MAP</div>

        <div class="map-stage">
          <svg class="mission-map-svg" viewBox="0 0 100 100" preserveAspectRatio="none">
            <line x1="0" y1="50" x2="100" y2="50" class="map-axis" />
            <line x1="50" y1="0" x2="50" y2="100" class="map-axis" />

            <circle cx="50" cy="50" r="15" class="map-range" />
            <circle cx="50" cy="50" r="30" class="map-range" />
            <circle cx="50" cy="50" r="45" class="map-range" />

            {#if flying}
              <text x="51.5" y="48" class="map-home-label">HOME</text>
              <circle cx="50" cy="50" r="1.4" class="map-home" />

              {#if gpsPath.length > 1}
                <polyline points={gpsPathPoints} class="map-path" />
              {/if}

              {#if currentGpsPoint}
                <circle cx={currentGpsPoint.x} cy={currentGpsPoint.y} r="0.5" class="map-rocket" />
              {/if}
            {/if}
          </svg>

          {#if !flying}
            <div class="map-idle">GPS map tracking starts during ASCENT or DESCENT.</div>
          {/if}
        </div>

        <div class="map-readout">
          {#if gps.lat !== null && gps.lon !== null}
            LAT {gps.lat.toFixed(6)} · LON {gps.lon.toFixed(6)}
          {:else}
            Waiting for GPS fix
          {/if}
        </div>
      </div>
    </div>

    <!-- ================================================================
         CENTRE COLUMN – telemetry metrics (2×3 grid)
         IMU orientation: PITCH, YAW, ROLL (from quaternion)
         Barometric altitude, vertical speed, vertical acceleration
    ================================================================ -->
    <div class="column telemetry">
      <div class="panel metric">
        <div class="label">PITCH (°)</div>
        <div class="value">{telemetry.pitch}</div>
        <div class="graph-stage">
          <svg class="mini-graph" viewBox="0 0 100 36" preserveAspectRatio="none">
            <line x1="0" y1="9"  x2="100" y2="9"  class="graph-grid" />
            <line x1="0" y1="18" x2="100" y2="18" class="graph-axis" />
            <line x1="0" y1="27" x2="100" y2="27" class="graph-grid" />
            <line x1="25" y1="0" x2="25" y2="36" class="graph-grid" />
            <line x1="50" y1="0" x2="50" y2="36" class="graph-grid" />
            <line x1="75" y1="0" x2="75" y2="36" class="graph-grid" />
            {#if flying}
              <polyline points={pitchGraphPoints} fill="none" class="graph-line" />
            {/if}
          </svg>
        </div>
      </div>

      <div class="panel metric">
        <div class="label">YAW (°)</div>
        <div class="value">{telemetry.yaw}</div>
        <div class="graph-stage">
          <svg class="mini-graph" viewBox="0 0 100 36" preserveAspectRatio="none">
            <line x1="0" y1="9"  x2="100" y2="9"  class="graph-grid" />
            <line x1="0" y1="18" x2="100" y2="18" class="graph-axis" />
            <line x1="0" y1="27" x2="100" y2="27" class="graph-grid" />
            <line x1="25" y1="0" x2="25" y2="36" class="graph-grid" />
            <line x1="50" y1="0" x2="50" y2="36" class="graph-grid" />
            <line x1="75" y1="0" x2="75" y2="36" class="graph-grid" />
            {#if flying}
              <polyline points={yawGraphPoints} fill="none" class="graph-line" />
            {/if}
          </svg>
        </div>
      </div>

      <div class="panel metric">
        <div class="label">ROLL (°)</div>
        <div class="value">{telemetry.roll}</div>
        <div class="graph-stage">
          <svg class="mini-graph" viewBox="0 0 100 36" preserveAspectRatio="none">
            <line x1="0" y1="9"  x2="100" y2="9"  class="graph-grid" />
            <line x1="0" y1="18" x2="100" y2="18" class="graph-axis" />
            <line x1="0" y1="27" x2="100" y2="27" class="graph-grid" />
            <line x1="25" y1="0" x2="25" y2="36" class="graph-grid" />
            <line x1="50" y1="0" x2="50" y2="36" class="graph-grid" />
            <line x1="75" y1="0" x2="75" y2="36" class="graph-grid" />
            {#if flying}
              <polyline points={rollGraphPoints} fill="none" class="graph-line" />
            {/if}
          </svg>
        </div>
      </div>

      <div class="panel metric no-graph">
        <div class="label">BARO ALTITUDE (m)</div>
        <div class="value">{telemetry.altitude}</div>
      </div>

      <div class="panel metric no-graph">
        <div class="label">VERTICAL SPEED (m/s)</div>
        <div class="value">{telemetry.speed}</div>
      </div>

      <div class="panel metric no-graph">
        <div class="label">VERT. ACCELERATION (m/s²)</div>
        <div class="value">{telemetry.acceleration}</div>
      </div>
    </div>

    <!-- ================================================================
         RIGHT COLUMN – battery + satellites (small, top) + control + errors
    ================================================================ -->
    <div class="column right-col">
      <!-- Two small status blocks side by side at the top -->
      <div class="small-panels-row">
        <div class="panel small-metric">
          <div class="label">BATTERY (V)</div>
          <div class="value small-value {batClass(batteryVoltage)}">{batteryVoltage}</div>
        </div>
        <div class="panel small-metric">
          <div class="label">SATELLITES</div>
          <div class="value small-value {satClass(satelliteCount)}">{satelliteCount}</div>
        </div>
      </div>

      <div class="panel errors">
        <div class="panel-title">SYSTEM ERRORS</div>

        {#if errors.length === 0}
          <div class="no-errors">No active faults</div>
        {:else}
          <ul>
            {#each errors as error}
              <li>{error}</li>
            {/each}
          </ul>
        {/if}
      </div>
    </div>
  </main>
</div>

<style>
  :global(body) {
    margin: 0;
    font-family: "Segoe UI", Arial, sans-serif;
    background: #0e131a;
    color: #cbd5e1;

    overflow: hidden;
  }

  .app {
    display: flex;
    flex-direction: column;
    height: 100vh;
    overflow: hidden;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 22px;
    background: #111821;
    border-bottom: 1px solid #1f2937;
    gap: 14px;
    flex-shrink: 0;
  }

  .header-left {
    display: flex;
    align-items: center;
    gap: 12px;
    min-width: 0;
  }

  .title {
    font-size: 13px;
    letter-spacing: 1px;
    white-space: nowrap;
  }

  button {
    background: #151c26;
    border: 1px solid #2a3442;
    color: #cbd5e1;
    padding: 5px 10px;
    font-size: 11px;
    cursor: pointer;
  }

  button:hover {
    background: #1d2632;
  }

  .states {
    display: flex;
    gap: 5px;
  }

  .states button {
    color: #94a3b8;
    padding: 5px 12px;
    transition: 0.15s;
    cursor: default;
  }

  .states button.selected {
    background: #2c394a;
    color: white;
    border-color: #3b4758;
  }

  .states button.safe-state.selected {
    background: #4b1f1f;
    border-color: #7f1d1d;
    color: #fecaca;
  }

  .timer {
    font-family: monospace;
    font-size: 15px;
    color: #94a3b8;
    white-space: nowrap;
  }

  .status-strip {
    padding: 5px 22px;
    background: #0f1620;
    border-bottom: 1px solid #1f2937;
    font-size: 12px;
    flex-shrink: 0;
  }

  .status-strip.flash {
    background: #1c2633;
  }

  main {
    flex: 1;
    display: grid;
    grid-template-columns: minmax(280px, 1fr) minmax(440px, 1.22fr) minmax(240px, 0.82fr);
    gap: 12px;
    padding: 12px;
    min-height: 0;
    overflow: hidden;
  }

  .column {
    min-height: 0;
    overflow: hidden;
  }

  main > .column:first-child {
    display: grid;
    grid-template-rows: minmax(210px, 0.82fr) minmax(260px, 1.18fr);
    gap: 12px;
  }

  .panel {
    background: #131a23;
    border: 1px solid #1f2937;
    padding: 10px;
  }

  .panel-title {
    font-size: 10px;
    letter-spacing: 1px;
    color: #8b98a8;
    margin-bottom: 8px;
  }

  .view {
    min-height: 0;
    overflow: hidden;
    display: flex;
    flex-direction: column;
  }

  .three-view {
    position: relative;
    width: 100%;
    height: calc(100% - 20px);
    background: #111827;
    overflow: hidden;
  }

  .three-view canvas {
    display: block;
    width: 100%;
    height: 100%;
  }

  .map {
    flex: 1;
    min-height: 0;
    position: relative;
    overflow: hidden;
    display: flex;
    flex-direction: column;
  }

  .map-stage {
    position: relative;
    flex: 1;
    min-height: 0;
  }

  .mission-map-svg {
    width: 100%;
    height: 100%;
    display: block;
    background: #111827;
    border: 1px solid #1f2937;
  }

  .map-idle {
    position: absolute;
    inset: 1px;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #64748b;
    font-family: monospace;
    font-size: 11px;
    text-align: center;
    padding: 10px;
    pointer-events: none;
  }

  .map-axis { stroke: #334155; stroke-width: 0.35; opacity: 0.7; }
  .map-range { fill: none; stroke: #263241; stroke-width: 0.45; opacity: 0.75; }
  .map-path { fill: none; stroke: #ffffff; stroke-width: 0.65; stroke-linecap: round; stroke-linejoin: round; opacity: 0.9; }
  .map-rocket { fill: rgb(92,92,92); stroke: rgb(92,92,92); stroke-width: 0.8; }
  .map-home { fill: #64748b; opacity: 0.9; }
  .map-home-label { fill: #64748b; font-size: 3px; font-family: monospace; }

  .map-readout {
    flex-shrink: 0;
    margin-top: 6px;
    font-size: 10px;
    color: #64748b;
    font-family: monospace;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .telemetry {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    grid-template-rows: repeat(3, minmax(0, 1fr));
    gap: 12px;
    min-height: 0;
  }

  .metric {
    display: flex;
    flex-direction: column;
    min-width: 0;
    min-height: 0;
    overflow: hidden;
  }

  .metric .label {
    font-size: 10px;
    color: #8b98a8;
    display: flex;
    align-items: center;
    gap: 5px;
  }

  .metric .value {
    font-size: 22px;
    font-family: monospace;
    margin-top: 4px;
    margin-bottom: 8px;
  }

  .metric.no-graph .value {
    margin-top: 10px;
    font-size: 30px;
  }

  .graph-stage {
    width: 100%;
    flex: 1;
    min-height: 50px;
  }

  .mini-graph {
    width: 100%;
    height: 100%;
    display: block;
    background: transparent;
    border: none;
    border-top: 1px solid #1f2937;
    padding-top: 6px;
    margin-top: 2px;
  }

  .graph-grid { stroke: #263241; stroke-width: 0.25; opacity: 0.45; }
  .graph-axis { stroke: #334155; stroke-width: 0.35; opacity: 0.65; }
  .graph-line { stroke: #ffffff; stroke-width: 0.25; stroke-linecap: round; stroke-linejoin: round; opacity: 0.95; }

  .right-col {
    display: grid;
    grid-template-rows: 112px minmax(0, 1fr);
    gap: 12px;
    min-height: 0;
  }

  .small-panels-row {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
    min-height: 0;
  }

  .small-metric {
    display: flex;
    flex-direction: column;
    justify-content: center;
    min-width: 0;
    min-height: 0;
    padding: 14px;
  }

  .small-metric .label {
    font-size: 11px;
    color: #8b98a8;
    margin-bottom: 8px;
  }

  .small-value {
    font-size: 29px;
    line-height: 1;
    font-family: monospace;
    color: #cbd5e1;
  }

  .small-value.ok   { color: #4ade80; }
  .small-value.warn { color: #fbbf24; }
  .small-value.crit { color: #f87171; }

  .errors {
    flex: 1;
    overflow: auto;
    min-height: 0;
  }

  .no-errors {
    font-size: 11px;
    color: #64748b;
  }

  ul { padding-left: 14px; margin: 0; }

  li {
    font-size: 11px;
    color: #ef4444;
    margin-bottom: 5px;
  }
  @media (max-width: 1180px) {
    main {
      grid-template-columns: minmax(250px, 0.95fr) minmax(400px, 1.2fr) minmax(215px, 0.78fr);
      gap: 10px;
      padding: 10px;
    }

    .column,
    main > .column:first-child,
    .telemetry,
    .right-col {
      gap: 10px;
    }

    .panel {
      padding: 9px;
    }

    .small-value {
      font-size: 26px;
    }
  }

  @media (max-width: 980px) {
    :global(body) {
      overflow: auto;
    }

    .app {
      height: auto;
      min-height: 100vh;
      overflow: visible;
    }

    main {
      grid-template-columns: 1fr;
      overflow: visible;
    }

    main > .column:first-child {
      grid-template-rows: 300px 360px;
    }

    .telemetry {
      grid-template-rows: repeat(3, 150px);
    }

    .right-col {
      grid-template-rows: 112px 220px;
    }
  }

</style>
