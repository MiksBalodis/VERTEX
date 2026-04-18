<script>
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

  let errors = [];
  let csvRows = [];
  let currentIndex = -1;
  let playbackTimer = null;
  let isPlaying = false;
  let loadedFileName = "";
  let elapsedMs = 0;
  let RSSI = 0;

  function flashTransition() {
    transitionFlash = true;
    setTimeout(() => (transitionFlash = false), 300);
  }

  function setState(state, statusText = null) {
    const normalizedState = (state || "").trim().toUpperCase();

    if (!states.includes(normalizedState)) {
      missionState = "SAFE";
      systemStatus = `Unknown mission state: ${state}`;
      flashTransition();
      return;
    }

    const changed = missionState !== normalizedState;
    missionState = normalizedState;
    systemStatus = statusText?.trim() || stateStatusMap[normalizedState] || "Unknown system state.";

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

  function parseNumeric(value) {
    if (value == null) return "--";
    const trimmed = String(value).trim();

    if (trimmed === "") return "--";
    if (trimmed.toLowerCase() === "nan") return "NaN";

    const num = Number(trimmed);
    if (!Number.isFinite(num)) return trimmed;

    return num.toFixed(1);
  }

  function parseErrorFlags(errorFlags) {
    const raw = (errorFlags || "").trim();
    if (!raw || raw === "NONE") return [];

    return raw
      .split("|")
      .map((flag) => flag.trim())
      .filter(Boolean);
  }

  function applyRow(row) {
    if (!row) return;

    elapsedMs = Number(row.time_ms) || 0;

    setState(row.state, row.status_text);

    telemetry = {
      pitch: parseNumeric(row.pitch),
      yaw: parseNumeric(row.yaw),
      roll: parseNumeric(row.roll),
      altitude: parseNumeric(row.altitude),
      speed: parseNumeric(row.speed),
      acceleration: parseNumeric(row.acceleration)
    };

    errors = parseErrorFlags(row.error_flags);
  }

  function stopPlayback() {
    isPlaying = false;
    if (playbackTimer) {
      clearTimeout(playbackTimer);
      playbackTimer = null;
    }
  }

  function resetDisplay() {
    stopPlayback();
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
    errors = [];
    elapsedMs = 0;
    currentIndex = -1;
  }

  function playCsv() {
    if (!csvRows.length || isPlaying) return;

    if (currentIndex < 0) {
      currentIndex = 0;
      applyRow(csvRows[0]);
    }

    isPlaying = true;
    scheduleNextStep();
  }

  function scheduleNextStep() {
    if (!isPlaying) return;

    if (currentIndex >= csvRows.length - 1) {
      stopPlayback();
      return;
    }

    const currentRow = csvRows[currentIndex];
    const nextRow = csvRows[currentIndex + 1];

    const currentTime = Number(currentRow?.time_ms) || 0;
    const nextTime = Number(nextRow?.time_ms) || 0;
    const deltaMs = Math.max(1, nextTime - currentTime);

    playbackTimer = setTimeout(() => {
      currentIndex += 1;
      applyRow(csvRows[currentIndex]);
      scheduleNextStep();
    }, deltaMs);
  }

  function splitCsvLine(line) {
    const result = [];
    let current = "";
    let insideQuotes = false;

    for (let i = 0; i < line.length; i++) {
      const char = line[i];
      const nextChar = line[i + 1];

      if (char === '"' && insideQuotes && nextChar === '"') {
        current += '"';
        i++;
      } else if (char === '"') {
        insideQuotes = !insideQuotes;
      } else if (char === "," && !insideQuotes) {
        result.push(current);
        current = "";
      } else {
        current += char;
      }
    }

    result.push(current);
    return result;
  }

  function parseCsv(text) {
    const lines = text
      .split(/\r?\n/)
      .map((line) => line.trim())
      .filter(Boolean);

    if (lines.length < 2) {
      throw new Error("CSV file is empty or missing data rows.");
    }

    const headers = splitCsvLine(lines[0]).map((h) => h.trim());

    const requiredHeaders = [
      "time_ms",
      "state",
      "pitch",
      "yaw",
      "roll",
      "altitude",
      "speed",
      "acceleration",
      "error_flags",
      "status_text"
    ];

    const missing = requiredHeaders.filter((header) => !headers.includes(header));
    if (missing.length > 0) {
      throw new Error(`Missing columns: ${missing.join(", ")}`);
    }

    const rows = [];

    for (let i = 1; i < lines.length; i++) {
      const values = splitCsvLine(lines[i]);
      if (values.length !== headers.length) continue;

      const row = {};
      headers.forEach((header, idx) => {
        row[header] = values[idx];
      });

      rows.push(row);
    }

    rows.sort((a, b) => (Number(a.time_ms) || 0) - (Number(b.time_ms) || 0));
    return rows;
  }

  async function handleFileChange(event) {
    const file = event.target.files?.[0];
    if (!file) return;

    try {
      const text = await file.text();
      csvRows = parseCsv(text);
      loadedFileName = file.name;
      resetDisplay();

      if (csvRows.length > 0) {
        currentIndex = 0;
        applyRow(csvRows[0]);
      }
    } catch (error) {
      csvRows = [];
      loadedFileName = file.name;
      stopPlayback();
      missionState = "SAFE";
      systemStatus = `CSV load error: ${error.message}`;
      errors = ["CSV_PARSE_ERROR"];
      telemetry = {
        pitch: "--",
        yaw: "--",
        roll: "--",
        altitude: "--",
        speed: "--",
        acceleration: "--"
      };
      elapsedMs = 0;
      currentIndex = -1;
      flashTransition();
    }
  }

  let port;
  let reader;
  let serialData = "";

  async function connectSerial() {
    try {
      // 1. Request port from user
      port = await navigator.serial.requestPort();
      await port.open({ baudRate: 9600 });

      // 2. Set up the stream reader
      const decoder = new TextDecoderStream();
      port.readable.pipeTo(decoder.writable);
      reader = decoder.readable.getReader();

      // 3. Read loop
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;

        if(value.includes('\r\n')){
          const lines = value.split('\r\n');
          serialData += lines[0];
          parseTelemetryFromSerial(serialData);
          serialData = lines[1];
        }else{
          serialData += value;
        }

        console.log("Received serial data:", value);
        

      }
    } catch (err) {
      console.error("Serial error:", err);
    }
  }

  function parseTelemetryFromSerial(data) {
    const cleanHex = data.replace(/\s+/g, '');
    const raw_bytes = hexToBytes(cleanHex);
    if (raw_bytes.length < 20) {
      console.warn("Received incomplete telemetry data");
      console.warn("Raw data:", data);
      console.warn("Parsed bytes:", raw_bytes);
      return;
    }

    const view = new DataView(raw_bytes.buffer, raw_bytes.byteOffset, raw_bytes.byteLength);

    telemetry.altitude =  view.getFloat32(0, true).toFixed(2);

    // int16_t accel (2 bytes) - Offset 4
    telemetry.acceleration = (view.getInt16(4, true) / 100).toFixed(2); // Convert cm/s^2 to m/s^2

    // int16_t speed (2 bytes) - Offset 6
    telemetry.speed = (view.getInt16(6, true) / 100).toFixed(2); // Convert cm/s to m/s

    // int16_t pitch, roll, yaw (2 bytes each) - Offsets 8, 10, 12
    telemetry.pitch = (view.getInt16(8, true) / 10).toFixed(1);
    telemetry.roll = (view.getInt16(10, true) / 10).toFixed(1);
    telemetry.yaw = (view.getInt16(12, true) / 10).toFixed(1);


    switch (view.getUint8(14)) {
      case 0: setState("READY"); break;
      case 1: setState("ASCENT"); break;
      case 2: setState("DESCENT"); break;
      case 3: setState("LANDED"); break;
      default: setState("SAFE", `Unknown flight state code: ${telemetry.flightState}`);
    }

    // uint32_t timestamp (4 bytes) - Offset 15
    elapsedMs = (view.getUint32(15, true) / 1000).toFixed(0); // Convert us to ms

    // int8_t RSSI (1 byte) - Offset 19
    RSSI = view.getInt8(19)
  }
  function hexToBytes(hexString) {
  const bytes = new Uint8Array(hexString.length / 2);
  
  for (let i = 0; i < hexString.length; i += 2) {
    bytes[i / 2] = parseInt(hexString.substring(i, i + 2), 16);
  }
  
  return bytes;
}
</script>

<div class="app">
  <header>
    <div class="header-left">
      <div class="title">GROUND STATION</div>

      <button on:click={connectSerial}>Connect to GS</button>

      <label class="file-picker">
        <span>Choose CSV</span>
        <input type="file" accept=".csv,text/csv" on:change={handleFileChange} />
      </label>

      {#if loadedFileName}
        <div class="file-name">{loadedFileName}</div>
      {/if}
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
    <div class="column">
      <div class="panel view">
        <div class="panel-title">3D VEHICLE VIEW</div>
      </div>

      <div class="panel map">
        <div class="panel-title">MISSION MAP</div>
      </div>
    </div>

    <div class="column telemetry">
      <div class="panel metric">
        <div class="label">PITCH (°)</div>
        <div class="value" class:invalid={telemetry.pitch === "NaN"}>{telemetry.pitch}</div>
      </div>

      <div class="panel metric">
        <div class="label">YAW (°)</div>
        <div class="value" class:invalid={telemetry.yaw === "NaN"}>{telemetry.yaw}</div>
      </div>

      <div class="panel metric">
        <div class="label">ROLL (°)</div>
        <div class="value" class:invalid={telemetry.roll === "NaN"}>{telemetry.roll}</div>
      </div>

      <div class="panel metric">
        <div class="label">BAROMETRIC ALTITUDE (m)</div>
        <div class="value">{telemetry.altitude}</div>
      </div>

      <div class="panel metric">
        <div class="label">SPEED (m/s)</div>
        <div class="value">{telemetry.speed}</div>
      </div>

      <div class="panel metric">
        <div class="label">ACCELERATION (m/s²)</div>
        <div class="value">{telemetry.acceleration}</div>
      </div>
    </div>

    <div class="column">
      <div class="panel control">
        <div class="panel-title">FLIGHT CONTROL</div>
        <button class="control-btn" type="button" on:click={playCsv}>PLAY</button>
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
  }

  .app {
    display: flex;
    flex-direction: column;
    height: 100vh;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 28px;
    background: #111821;
    border-bottom: 1px solid #1f2937;
    gap: 14px;
  }

  .header-left {
    display: flex;
    align-items: center;
    gap: 12px;
    min-width: 0;
  }

  .title {
    font-size: 14px;
    letter-spacing: 1px;
    font-weight: 600;
    white-space: nowrap;
  }

  .file-picker {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 12px;
    color: #94a3b8;
    white-space: nowrap;
  }

  .file-picker input {
    font-size: 12px;
    color: #94a3b8;
    max-width: 180px;
  }

  .file-name {
    font-size: 11px;
    color: #64748b;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 160px;
  }

  .states {
    display: flex;
    gap: 6px;
  }

  .states button {
    background: #151c26;
    border: 1px solid #2a3442;
    color: #94a3b8;
    padding: 6px 14px;
    font-size: 12px;
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
    font-size: 13px;
    color: #94a3b8;
    white-space: nowrap;
  }

  .status-strip {
    padding: 8px 28px;
    background: #0f1620;
    border-bottom: 1px solid #1f2937;
    font-size: 13px;
  }

  .status-strip.flash {
    background: #1c2633;
  }

  main {
    flex: 1;
    display: grid;
    grid-template-columns: 1fr 1.2fr 0.9fr;
    gap: 18px;
    padding: 18px;
  }

  .column {
    display: flex;
    flex-direction: column;
    gap: 18px;
  }

  .panel {
    background: #131a23;
    border: 1px solid #1f2937;
    padding: 14px;
  }

  .panel-title {
    font-size: 11px;
    letter-spacing: 1px;
    color: #8b98a8;
    margin-bottom: 10px;
  }

  .view {
    height: 220px;
  }

  .map {
    height: 280px;
  }

  .telemetry {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 18px;
  }

  .metric .label {
    font-size: 11px;
    color: #8b98a8;
  }

  .metric .value {
    font-size: 22px;
    font-family: monospace;
    margin-top: 6px;
  }

  .metric .value.invalid {
    color: #f59e0b;
  }

  .control-btn {
    width: 100%;
    margin-top: 8px;
    padding: 8px;
    background: #151c26;
    border: 1px solid #2a3442;
    color: #cbd5e1;
    font-size: 12px;
    cursor: pointer;
  }

  .control-btn:hover {
    background: #1d2632;
  }

  .errors {
    flex: 1;
    overflow: auto;
  }

  .no-errors {
    font-size: 12px;
    color: #64748b;
  }

  ul {
    padding-left: 16px;
    margin: 0;
  }

  li {
    font-size: 12px;
    color: #ef4444;
    margin-bottom: 6px;
  }
</style>