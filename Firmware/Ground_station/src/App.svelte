<script>
  const states = ["IDLE", "READY", "ASCENT", "DESCENT", "LANDED"];
  let missionState = "IDLE";
  let systemStatus = "Systems safe. Awaiting command.";
  let transitionFlash = false;

  function setState(state) {
    if (missionState === state) return;

    missionState = state;
    transitionFlash = true;
    setTimeout(() => (transitionFlash = false), 300);

    switch (state) {
      case "IDLE":
        systemStatus = "Systems safe. Awaiting command.";
        break;
      case "READY":
        systemStatus = "Vehicle armed. Pre-launch checks complete.";
        break;
      case "ASCENT":
        systemStatus = "Launch detected. Stabilization active.";
        break;
      case "DESCENT":
        systemStatus = "Descent phase active.";
        break;
      case "LANDED":
        systemStatus = "Touchdown confirmed. Recovery mode.";
        break;
    }
  }

  let telemetry = {
    pitch: "--",
    yaw: "--",
    roll: "--",
    altitude: "--",
    speed: "--",
    acceleration: "--"
  };

  let errors = [];
</script>

<div class="app">

  <!-- HEADER -->
  <header>
    <div class="title">GROUND STATION</div>

    <div class="states">
      {#each states as state}
        <button
          class:selected={missionState === state}
          on:click={() => setState(state)}
        >
          {state}
        </button>
      {/each}
    </div>

    <div class="timer">T+ 00:00:00</div>
  </header>

  <!-- STATUS STRIP -->
  <div class="status-strip {transitionFlash ? 'flash' : ''}">
    {systemStatus}
  </div>

  <!-- MAIN -->
  <main>

    <!-- LEFT COLUMN -->
    <div class="column">
      <div class="panel view">
        <div class="panel-title">3D VEHICLE VIEW</div>
      </div>

      <div class="panel map">
        <div class="panel-title">MISSION MAP</div>
      </div>
    </div>

    <!-- CENTER COLUMN -->
    <div class="column telemetry">

      <div class="panel metric">
        <div class="label">PITCH (°)</div>
        <div class="value">{telemetry.pitch}</div>
      </div>

      <div class="panel metric">
        <div class="label">YAW (°)</div>
        <div class="value">{telemetry.yaw}</div>
      </div>

      <div class="panel metric">
        <div class="label">ROLL (°)</div>
        <div class="value">{telemetry.roll}</div>
      </div>

      <div class="panel metric">
        <div class="label">ALTITUDE (m)</div>
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

    <!-- RIGHT COLUMN -->
    <div class="column">

      <div class="panel control">
        <div class="panel-title">FLIGHT CONTROL</div>
        <button class="control-btn">ARM SYSTEM</button>
        <button class="control-btn">DISARM SYSTEM</button>
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
  }

  .title {
    font-size: 14px;
    letter-spacing: 1px;
    font-weight: 600;
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
    cursor: pointer;
    transition: 0.15s;
  }

  .states button:hover {
    background: #1d2632;
  }

  .states button.selected {
    background: #2c394a;
    color: white;
    border-color: #3b4758;
  }

  .timer {
    font-family: monospace;
    font-size: 13px;
    color: #94a3b8;
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
  }
</style>
