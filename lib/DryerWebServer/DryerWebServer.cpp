#include "DryerWebServer.h"

#include <WebServer.h>

#include <cstdlib>

#include "DeviceSettings.h"
#include "Discovery.h"
#include "DryerRegisters.h"
#include "FirmwareVersion.h"
#include "ModbusRegisterMap.h"
#include "ModelFactory.h"
#include "SpiCcp.h"
#include "rtsnow_node.h"

DryerWebServer DryerWeb;

namespace
{

    WebServer server(80);

    // Matches DryerRegisters.cpp's onSetBaudRate() validation - the sheet's
    // documented set of 5 rates for the SPI-CCP link.
    constexpr uint32_t kValidSpiBauds[] = {1200, 2400, 4800, 9600, 19200};

    bool isValidSpiBaud(uint32_t baud)
    {
        for (uint32_t b : kValidSpiBauds)
        {
            if (b == baud)
                return true;
        }
        return false;
    }

    const char *equipmentTypeString(DeviceSettings::EquipmentType type)
    {
        switch (type)
        {
        case DeviceSettings::EquipmentType::Dryer:
            return "Dryer";
        case DeviceSettings::EquipmentType::Crystallizer:
            return "Crystallizer";
        }
        return "Dryer";
    }

    // The dashboard header shows this node's RTS-NOW friendly name (the
    // same one set from RTS-ESPNOW-Gateway's desktop app, so both
    // surfaces agree) rather than DeviceSettings::deviceName() ("Dryer-1"
    // by default) - that field predates RTS-NOW and is otherwise unused
    // by the dashboard. Falls back to deviceName() if the friendly name
    // is empty, which is only the case on env:gateway (rtsnowNodeBegin(),
    // and therefore loadFriendlyName(), only ever runs for ROLE_NODE -
    // see main.cpp).
    String friendlyName()
    {
        const char *name = rtsnowNodeFriendlyName();
        return (name != nullptr && name[0] != '\0') ? String(name) : Settings.deviceName();
    }


    String hexDumpBytes(const uint8_t *data, size_t len)
    {
        String s;
        s.reserve(len * 3);
        for (size_t i = 0; i < len; i++)
        {
            if (i)
                s += ' ';
            char b[3];
            snprintf(b, sizeof(b), "%02X", data[i]);
            s += b;
        }
        return s;
    }

    // A discovered CMD1/CMD2 pair is "new" if it doesn't match anything
    // the active model already knows about - its own query table (see
    // EquipmentModel::queryInfo()) or the protocol-mandated ECHO/REVISION
    // pair (see EquipmentModel::echoCmd1()). Shared between the live
    // status endpoint and the printable report so the two can't disagree.
    bool isNewCommand(uint8_t cmd1, uint8_t cmd2)
    {
        if (cmd1 == ActiveModel->echoCmd1() && (cmd2 == 0x20 || cmd2 == 0x22))
            return false;
        for (size_t i = 0; i < ActiveModel->queryCount(); i++)
        {
            SpiCcpQueryInfo q = ActiveModel->queryInfo(i);
            if (q.cmd1 == cmd1 && q.cmd2 == cmd2)
                return false;
        }
        return true;
    }

    // Collects every register the active model actually knows about - "the
    // model can name it" (see EquipmentModel::registerName()), not "we
    // currently have a working poll that populates it". That distinction
    // matters: e.g. the crystallizer's Process Temp/Status, Machine
    // Status, and Return Temp (40012-40015) are real per the "#PXB-SPI-
    // MOD-485" Modbus map even though no confirmed SPI-CCP command
    // populates them yet on that device - they should still show up in
    // the table (as "no data yet") rather than disappear entirely just
    // because nothing polls them. Iterating in register-number order
    // means the result is already sorted, no separate sort step needed.
    constexpr size_t kMaxSupportedRegisters = 64;

    size_t collectSupportedRegisters(uint16_t *out, size_t outCap)
    {
        size_t count = 0;
        for (uint16_t reg = ModbusReg::kProcessSetpoint; reg <= ModbusReg::kLastRegister && count < outCap; reg++)
        {
            if (ActiveModel->registerName(reg) != nullptr)
                out[count++] = reg;
        }
        return count;
    }

    const char *outcomeText(SpiCcp::PollOutcome outcome)
    {
        switch (outcome)
        {
        case SpiCcp::PollOutcome::kNone:
            return "No poll yet";
        case SpiCcp::PollOutcome::kEot:
            return "EOT (device has nothing to send)";
        case SpiCcp::PollOutcome::kNoReply:
            return "No reply (timeout)";
        case SpiCcp::PollOutcome::kShortReply:
            return "Short reply";
        case SpiCcp::PollOutcome::kFramingMismatch:
            return "Framing mismatch";
        case SpiCcp::PollOutcome::kMissingTrailer:
            return "Missing trailer";
        case SpiCcp::PollOutcome::kCrcMismatch:
            return "CRC mismatch";
        case SpiCcp::PollOutcome::kSuccess:
            return "Success";
        }
        return "Unknown";
    }

    // Single-page dashboard: static poll/query reference plus a live table
    // that refetches /api/data every 2s (matching updateDryerReadings()'s own
    // poll interval in main.cpp) via fetch()/JS, no page reload needed.
    // Equipment/model selection re-renders the queries/registers tables from
    // scratch on change (they're keyed by model name, see JS renderXxx()
    // functions), since a different model has an entirely different query
    // set and register meanings.
    const char kIndexHtml[] = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RTS-Now Dashboard</title>
<style>
  body { font-family: -apple-system, Arial, sans-serif; margin: 0; padding: 16px; background: #111; color: #eee; }
  h1 { font-size: 1.3em; margin: 0 0 4px; }
  h2 { font-size: 1.05em; margin: 20px 0 6px; color: #9cf; }
  #meta { color: #aaa; font-size: 0.9em; margin-bottom: 12px; }
  table { border-collapse: collapse; width: 100%; margin-bottom: 12px; }
  th, td { text-align: left; padding: 4px 10px; border-bottom: 1px solid #333; font-size: 0.9em; }
  th { color: #9cf; font-weight: 600; }
  tr.stale td { color: #777; }
  .ok { color: #6f6; }
  .bad { color: #f66; }
  #outcome { font-weight: 600; }
  #frames { font-family: "SF Mono", Menlo, monospace; font-size: 0.85em; color: #ccc; white-space: pre-wrap; word-break: break-all; }
  #frames div { margin-bottom: 4px; }
  .baudBtn { background: #222; color: #eee; border: 1px solid #555; border-radius: 4px; padding: 4px 10px; margin-right: 6px; margin-bottom: 6px; font-size: 0.9em; }
  .baudBtn.active { background: #06c; border-color: #06c; }
  .scanActive { background: #1a6b1a !important; border-color: #2a8a2a !important; color: #fff !important; }
  .scanIdleStop { background: #6b1a1a !important; border-color: #8a2a2a !important; color: #fff !important; }
  .tabBtn { background: #1a1a1a; color: #ccc; border: 1px solid #444; border-bottom: none; border-radius: 6px 6px 0 0; padding: 8px 16px; font-size: 0.95em; cursor: pointer; }
  .tabBtn.active { background: #111; color: #fff; border-color: #06c; font-weight: 600; }
  #tabBar { border-bottom: 1px solid #444; margin-bottom: 16px; }
  .tabPanel { display: none; }
  .tabPanel.active { display: block; }
</style>
</head>
<body>
  <h1>RTS-Now Dashboard</h1>
  <div id="meta">loading...</div>

  <div id="tabBar">
    <button class="tabBtn" id="tabBtnNormal">Normal SPI Functions</button>
    <button class="tabBtn" id="tabBtnDiagnostics">Diagnostics</button>
    <button class="tabBtn" id="tabBtnDiscovery">Discovery</button>
  </div>

  <div id="tabNormal" class="tabPanel">
  <h2>Equipment</h2>
  <div id="equipButtons"></div>
  <h2>Model</h2>
  <div id="modelButtons"></div>

  <h2>Device Settings (40001-40009)</h2>
  <table>
    <thead><tr><th>Register</th><th>Name</th><th>Value</th></tr></thead>
    <tbody id="deviceRegisters"></tbody>
  </table>
  <div style="color:#aaa;font-size:0.85em;margin-top:-6px;margin-bottom:12px;">Double-click SPI Station ID, SPI Baud Rate, or Model Type to change; SPI CRC Error has a Reset button instead.</div>

  <h2>SPI-CCP Poll Queries</h2>
  <table>
    <thead><tr><th>#</th><th>Query</th><th>CMD1</th><th>CMD2</th><th>Registers</th></tr></thead>
    <tbody id="queries"></tbody>
  </table>

  <h2>Last RS485 Frame</h2>
  <div id="frames">
    <div>TX: <span id="txHex">-</span></div>
    <div>RX: <span id="rxHex">-</span></div>
  </div>

  <h2>SPI Baud Rate</h2>
  <div id="baudButtons"></div>

  <h2>Live Register Data (from RS485)</h2>
  <div>Last poll outcome: <span id="outcome">-</span></div>
  <table>
    <thead><tr><th>Register</th><th>Name</th><th>Value</th><th>Status</th></tr></thead>
    <tbody id="registers"></tbody>
  </table>
  </div>

  <div id="tabDiagnostics" class="tabPanel">
  <h2>Manual Test</h2>
  <div id="manualTestButtons">
    <button class="baudBtn" id="pollNowBtn">Poll Now (next in round-robin)</button>
    <button class="baudBtn" id="echoBtn">Send ECHO</button>
    <button class="baudBtn" id="versionBtn">Send Version</button>
  </div>
  <div id="testHint" style="color:#aaa;font-size:0.85em;margin-top:6px;">Fires immediately (doesn't wait for the 2s auto-poll loop). Probe the TTL-side UART signal (before the RS485 transceiver, single wire vs GND) for a clean logic-analyzer decode - a single-ended probe on the A/B differential pair won't decode cleanly.</div>
  <div id="testStatus" style="color:#9cf;font-size:0.9em;margin-top:6px;font-weight:600;">No manual test sent yet.</div>

  <h2>Raw Command Test</h2>
  <div>
    CMD1 (hex): <input type="text" id="rawCmd1" value="C2" size="4" maxlength="2">
    CMD2 (hex): <input type="text" id="rawCmd2" value="20" size="4" maxlength="2">
    <button class="baudBtn" id="rawPollBtn">Poll</button>
  </div>
  <div id="rawSent" style="font-family:'SF Mono',Menlo,monospace;font-size:0.85em;color:#ccc;margin-top:6px;">Sent: -</div>
  <div id="rawResult" style="font-family:'SF Mono',Menlo,monospace;font-size:0.85em;color:#ccc;margin-top:4px;">Result: -</div>

  <h2>Raw SELECT Test (write)</h2>
  <div style="color:#f66;font-size:0.85em;margin-bottom:6px;">Writes data to the device - only use this when you're sure it's safe to do so on the connected equipment.</div>
  <div>
    CMD1 (hex): <input type="text" id="selCmd1" value="C2" size="4" maxlength="2">
    CMD2 (hex): <input type="text" id="selCmd2" value="48" size="4" maxlength="2">
    Data bytes (hex, space-separated): <input type="text" id="selData" value="00 01" size="12">
    <button class="baudBtn" id="selectBtn">Send SELECT</button>
  </div>
  <div style="margin-top:6px;">
    Or enter a value: <input type="text" id="selValue" size="8" placeholder="e.g. 250">
    as <select id="selValueType">
      <option value="float32">float32 (4 bytes)</option>
      <option value="uint16">uint16 (2 bytes)</option>
      <option value="uint8">uint8 (1 byte)</option>
    </select>
    <button class="baudBtn" id="selFillBtn">Fill data bytes</button>
  </div>
  <div id="selSent" style="font-family:'SF Mono',Menlo,monospace;font-size:0.85em;color:#ccc;margin-top:6px;">Sent: -</div>
  <div id="selResult" style="font-family:'SF Mono',Menlo,monospace;font-size:0.85em;color:#ccc;margin-top:4px;">Result: -</div>
  </div>

  <div id="tabDiscovery" class="tabPanel">
  <p style="color:#aaa;font-size:0.85em;">Normal round-robin polling pauses automatically while a scan below is running.</p>
  <h2>Discovery</h2>
  <div>
    <button class="baudBtn" id="scanDevIdBtn">Scan DEVIDs (0x20-0xFF)</button>
    <button class="baudBtn" id="stopScanBtn">Stop Scan</button>
  </div>
  <div id="discoverProgress" style="color:#9cf;font-size:0.9em;margin-top:6px;">No scan running.</div>
  <div id="discoverDevIds" style="font-size:0.9em;margin-top:10px;">No DEVIDs found yet.</div>

  <table id="discoverCommandsTable" style="display:none;">
    <thead><tr><th>CMD1</th><th>CMD2</th><th>Data</th><th>New?</th></tr></thead>
    <tbody id="discoverCommands"></tbody>
  </table>
  <div style="margin-top:8px;font-size:0.9em;"><a href="/report" target="_blank" style="color:#9cf;">Open printable Discovery report (use your browser's Print &rarr; Save as PDF)</a></div>
  </div>

<script>
const kTabs = ['normal', 'diagnostics', 'discovery'];
const tabBtns = { normal: document.getElementById('tabBtnNormal'), diagnostics: document.getElementById('tabBtnDiagnostics'), discovery: document.getElementById('tabBtnDiscovery') };
const tabPanels = { normal: document.getElementById('tabNormal'), diagnostics: document.getElementById('tabDiagnostics'), discovery: document.getElementById('tabDiscovery') };

function showTab(name) {
  kTabs.forEach(t => {
    const active = t === name;
    tabBtns[t].classList.toggle('active', active);
    tabPanels[t].classList.toggle('active', active);
  });
  localStorage.setItem('spiimTab', name);
}
kTabs.forEach(t => tabBtns[t].addEventListener('click', () => showTab(t)));
showTab(kTabs.includes(localStorage.getItem('spiimTab')) ? localStorage.getItem('spiimTab') : 'normal');

const queriesBody = document.getElementById('queries');
const registersBody = document.getElementById('registers');
const baudButtons = document.getElementById('baudButtons');
const equipButtons = document.getElementById('equipButtons');
const modelButtons = document.getElementById('modelButtons');
const kBauds = [1200, 2400, 4800, 9600, 19200];
// ETHERNET is a model choice, not its own equipment-type button - see
// modelListByEquip below, which always appends it to whichever model
// list is shown regardless of which of these two is active.
const kEquipTypes = ['Dryer', 'Crystallizer'];
let lastModelKey = null; // "<equipmentType>/<model>" - re-render queries/registers only when this changes
let baudButtonsRendered = false;

function renderBaudButtons(activeBaud) {
  if (!baudButtonsRendered) {
    baudButtonsRendered = true;
    baudButtons.innerHTML = kBauds.map(b => `<button class="baudBtn" data-baud="${b}">${b}</button>`).join('');
    baudButtons.querySelectorAll('.baudBtn').forEach(btn => {
      btn.addEventListener('click', () => setBaud(btn.dataset.baud));
    });
  }
  baudButtons.querySelectorAll('.baudBtn').forEach(btn => {
    btn.classList.toggle('active', Number(btn.dataset.baud) === activeBaud);
  });
}

async function setBaud(baud) {
  await fetch('/api/baud', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'value=' + baud });
  refresh();
}

function renderEquipButtons(activeEquip) {
  equipButtons.innerHTML = kEquipTypes.map(e => `<button class="baudBtn" data-equip="${e}">${e}</button>`).join('');
  equipButtons.querySelectorAll('.baudBtn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.equip === activeEquip);
    btn.addEventListener('click', () => setEquipment(btn.dataset.equip));
  });
}

async function setEquipment(equip) {
  await fetch('/api/equipment', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'value=' + equip });
  refresh();
}

function renderModelButtons(models, activeModel) {
  modelButtons.innerHTML = models.map(m => `<button class="baudBtn" data-model="${m}">${m}</button>`).join('');
  modelButtons.querySelectorAll('.baudBtn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.model === activeModel);
    btn.addEventListener('click', () => setModel(btn.dataset.model));
  });
}

async function setModel(model) {
  await fetch('/api/model', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'value=' + model });
  refresh();
}

let manualTestCount = 0;
const testStatus = document.getElementById('testStatus');

// Gives obvious feedback even when the outcome is another timeout, so a
// click is never mistaken for "nothing happened" - disables every button
// in the Manual Test group during the ~1s blocking request, then reports a
// running count + the exact TX bytes just sent, independent of whatever
// refresh() renders elsewhere.
const manualTestButtons = document.querySelectorAll('#manualTestButtons button');

async function sendManualTest(path, label) {
  manualTestButtons.forEach(b => b.disabled = true);
  testStatus.textContent = `Sending ${label}...`;
  try {
    const res = await fetch(path, { method: 'POST' });
    const data = await res.json();
    manualTestCount++;
    const time = new Date().toLocaleTimeString();
    testStatus.textContent =
      `#${manualTestCount} ${label} sent at ${time} - TX: ${data.lastTxHex} - outcome: ${data.pollOutcome}`;
    renderQueries(data.queries);
    renderRegisters(data.registers);
  } catch (e) {
    testStatus.textContent = `${label} request failed: ${e}`;
  } finally {
    manualTestButtons.forEach(b => b.disabled = false);
  }
}

document.getElementById('pollNowBtn').addEventListener('click', () => sendManualTest('/api/poll', 'Poll'));
document.getElementById('echoBtn').addEventListener('click', () => sendManualTest('/api/echo', 'ECHO'));
document.getElementById('versionBtn').addEventListener('click', () => sendManualTest('/api/version', 'Version'));

// Raw Command Test - lets you try an arbitrary CMD1/CMD2 pair against
// whichever model/DevID is currently active, for exploring commands with
// no known meaning yet. Sent/Result stay on screen until the next raw
// poll is sent - they're set only from this button's own response, never
// touched by the 2s auto-refresh(), so they don't get overwritten by the
// round-robin's own polling in between.
const rawPollBtn = document.getElementById('rawPollBtn');
const rawSent = document.getElementById('rawSent');
const rawResult = document.getElementById('rawResult');

rawPollBtn.addEventListener('click', async () => {
  const cmd1 = document.getElementById('rawCmd1').value.trim() || '00';
  const cmd2 = document.getElementById('rawCmd2').value.trim() || '00';
  rawPollBtn.disabled = true;
  rawResult.textContent = 'Result: sending...';
  try {
    const res = await fetch('/api/rawpoll', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: `cmd1=${cmd1}&cmd2=${cmd2}` });
    const data = await res.json();
    rawSent.textContent = `Sent: CMD1=0x${cmd1.toUpperCase()} CMD2=0x${cmd2.toUpperCase()} - TX: ${data.lastTxHex}`;
    rawResult.textContent = `Result: RX: ${data.lastRxHex || '(nothing)'} - outcome: ${data.pollOutcome}`;
  } catch (e) {
    rawResult.textContent = `Result: request failed: ${e}`;
  } finally {
    rawPollBtn.disabled = false;
  }
});

// Raw SELECT Test - write-side counterpart to Raw Command Test above, for
// exploring a command's write format/effect. Deliberately requires an
// explicit click every time (no auto-repeat, no keyboard-Enter submit) -
// this sends real writes.
const selectBtn = document.getElementById('selectBtn');
const selSent = document.getElementById('selSent');
const selResult = document.getElementById('selResult');
const selFillBtn = document.getElementById('selFillBtn');

// Converts the decimal "Value" field into the same big-endian byte layout
// EquipmentModel::writeRegister() sends on the C++ side (float32 = IEEE-754
// big-endian, uint16/uint8 = plain big-endian), so raw hex math isn't a
// prerequisite for testing a command here - matches the convenience the
// Live Register Data table's double-click prompt already has. Returns null
// (and leaves selData untouched) if the Value field is empty, so entering
// raw hex bytes directly still works exactly as before.
function fillDataBytesFromValue() {
  const raw = document.getElementById('selValue').value.trim();
  if (raw === '') return null;
  const type = document.getElementById('selValueType').value;
  const num = Number(raw);
  if (Number.isNaN(num)) return 'error';
  const bytes = [];
  if (type === 'float32') {
    const buf = new ArrayBuffer(4);
    new DataView(buf).setFloat32(0, num, false);
    new Uint8Array(buf).forEach(b => bytes.push(b));
  } else if (type === 'uint16') {
    const v = Math.trunc(num) & 0xFFFF;
    bytes.push((v >> 8) & 0xFF, v & 0xFF);
  } else {
    bytes.push(Math.trunc(num) & 0xFF);
  }
  const hex = bytes.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
  document.getElementById('selData').value = hex;
  return hex;
}

selFillBtn.addEventListener('click', fillDataBytesFromValue);

selectBtn.addEventListener('click', async () => {
  const fillResult = fillDataBytesFromValue();
  if (fillResult === 'error') {
    selResult.textContent = 'Result: value is not a number';
    return;
  }
  const cmd1 = document.getElementById('selCmd1').value.trim() || '00';
  const cmd2 = document.getElementById('selCmd2').value.trim() || '00';
  const databytes = document.getElementById('selData').value.trim();
  selectBtn.disabled = true;
  selResult.textContent = 'Result: sending...';
  try {
    const res = await fetch('/api/rawselect', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `cmd1=${cmd1}&cmd2=${cmd2}&databytes=${encodeURIComponent(databytes)}`
    });
    const data = await res.json();
    selSent.textContent = `Sent: CMD1=0x${cmd1.toUpperCase()} CMD2=0x${cmd2.toUpperCase()} data=[${databytes}] - TX: ${data.lastTxHex}`;
    selResult.textContent = `Result: RX: ${data.lastRxHex || '(nothing)'} - outcome: ${data.pollOutcome}`;
  } catch (e) {
    selResult.textContent = `Result: request failed: ${e}`;
  } finally {
    selectBtn.disabled = false;
  }
});

// Discovery - brute-force DEVID and CMD1xCMD2 scanning (see lib/Discovery).
// Both scans run in the background on the device itself; this just polls
// /api/discover/status once a second while one is active to show live
// progress, and stops polling once it reports not running.
let discoverPolling = null;
const scanDevIdBtn = document.getElementById('scanDevIdBtn');
const stopScanBtn = document.getElementById('stopScanBtn');

async function startCommandScan(devid) {
  await fetch('/api/discover/command/start', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'devid=' + devid });
  startDiscoverPolling();
}

function renderDevIds(devIds, running, mode, commandScanDevId) {
  const div = document.getElementById('discoverDevIds');
  if (!devIds.length) {
    div.textContent = 'No DEVIDs found yet.';
    return;
  }
  div.innerHTML = `Responding DEVIDs (${devIds.length}): ` + devIds.map(d => {
    const isThisOneScanning = running && mode === 'command' && commandScanDevId === d;
    return `<span style="display:inline-block;margin:0 12px 0 0;">0x${d} ` +
           `<button class="baudBtn scanCmdBtn${isThisOneScanning ? ' scanActive' : ''}" data-devid="${d}" style="padding:2px 8px;font-size:0.85em;">Scan Commands</button></span>`;
  }).join('');
  div.querySelectorAll('.scanCmdBtn').forEach(btn => {
    btn.addEventListener('click', () => startCommandScan(btn.dataset.devid));
  });
}

function renderDiscoverCommands(commands, running, mode) {
  // Hidden until a command scan has actually started (or has results) -
  // no point showing an empty CMD1/CMD2/Data/New? table before then.
  document.getElementById('discoverCommandsTable').style.display =
    (running && mode === 'command') || commands.length ? '' : 'none';
  document.getElementById('discoverCommands').innerHTML = commands.map(c =>
    `<tr><td>0x${c.cmd1}</td><td>0x${c.cmd2}</td><td style="font-family:'SF Mono',Menlo,monospace;">${c.data}</td>` +
    `<td>${c.isNew ? '<span class="ok">NEW</span>' : ''}</td></tr>`
  ).join('');
}

async function pollDiscoverStatus() {
  const res = await fetch('/api/discover/status');
  const data = await res.json();
  const progDiv = document.getElementById('discoverProgress');

  scanDevIdBtn.classList.toggle('scanActive', data.running && data.mode === 'devid');
  stopScanBtn.classList.toggle('scanIdleStop', !data.running);

  if (data.running) {
    const pct = data.progressTotal ? Math.round(100 * data.progressCurrent / data.progressTotal) : 0;
    progDiv.textContent = `Scanning (${data.mode})... ${data.progressCurrent}/${data.progressTotal} (${pct}%)`;
  } else {
    progDiv.textContent = 'No scan running.';
    if (discoverPolling) { clearInterval(discoverPolling); discoverPolling = null; }
  }
  renderDevIds(data.devIds, data.running, data.mode, data.commandScanDevId);
  renderDiscoverCommands(data.commands, data.running, data.mode);
}

function startDiscoverPolling() {
  if (discoverPolling) return;
  discoverPolling = setInterval(pollDiscoverStatus, 1000);
  pollDiscoverStatus();
}

scanDevIdBtn.addEventListener('click', async () => {
  await fetch('/api/discover/devid/start', { method: 'POST' });
  startDiscoverPolling();
});
stopScanBtn.addEventListener('click', async () => {
  await fetch('/api/discover/stop', { method: 'POST' });
  pollDiscoverStatus();
});

pollDiscoverStatus(); // pick up a scan already running from before a page reload

function renderQueries(queries) {
  queriesBody.innerHTML = queries.map((q, i) =>
    `<tr><td>${i + 1}</td><td>${q.name}</td><td>0x${q.cmd1}</td><td>0x${q.cmd2}</td><td>${q.registers}</td></tr>`
  ).join('');
}

function renderRegisters(regs) {
  registersBody.innerHTML = regs.map(r =>
    `<tr class="${r.present ? '' : 'stale'}"><td>${r.reg}</td><td>${r.name}</td>` +
    `<td class="regValue" data-reg="${r.reg}" title="Double-click to write a new value">${r.present ? r.value : '-'}</td>` +
    `<td>${r.present ? '<span class="ok">live</span>' : '<span class="bad">no data yet</span>'}</td></tr>`
  ).join('');
  registersBody.querySelectorAll('.regValue').forEach(td => {
    td.style.cursor = 'pointer';
    td.addEventListener('dblclick', () => writeRegisterPrompt(td.dataset.reg, td.textContent));
  });
}

// Double-click-to-edit on the Live Register Data table - currently only
// Process Setpoint (40010) and Process Limit Delta (40011) actually write
// anything (see EquipmentModel::writeRegister()); anything else gets a
// clear "not supported" from the server rather than silently doing
// nothing.
async function writeRegisterPrompt(reg, currentValue) {
  const input = prompt(`Enter new value for register ${reg}:`, currentValue === '-' ? '' : currentValue);
  if (input === null || input.trim() === '') return;
  try {
    const res = await fetch('/api/writeregister', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `reg=${reg}&value=${encodeURIComponent(input.trim())}`
    });
    if (!res.ok) {
      alert(`Write failed: ${await res.text()}`);
      return;
    }
    const data = await res.json();
    renderRegisters(data.registers);
  } catch (e) {
    alert(`Write request failed: ${e}`);
  }
}

const kModelTypeDescriptions = [
  '0 = FC (Dryer)', '1 = FD (Dryer)', '2 = FN (Dryer)', '3 = ADV (Dryer)',
  '4 = CD (Dryer)', '5 = FC-XTLR (Crystallizer)', '6 = FN-XTLR (Crystallizer)',
  '7 = ETHERNET (no equipment)'
];

// Device Settings (40001-40009) - separate from renderRegisters() above
// since these are device config, not per-model SPI-CCP data. Only
// Station ID (40002), Baud Rate (40003), and Model Type (40004) are
// editable; SPI CRC Error (40009) gets a Reset button instead of a
// value prompt, and everything else here is read-only.
function renderDeviceRegisters(regs) {
  document.getElementById('deviceRegisters').innerHTML = regs.map(r => {
    let valueCell = String(r.value);
    if (r.reg === 40002 || r.reg === 40003 || r.reg === 40004) {
      valueCell = `<span class="regValue" data-reg="${r.reg}" style="cursor:pointer;" title="Double-click to change">${r.value}</span>`;
    } else if (r.reg === 40009) {
      valueCell = `${r.value} <button class="baudBtn" id="resetCrcBtn" style="padding:1px 8px;font-size:0.8em;">Reset</button>`;
    }
    return `<tr><td>${r.reg}</td><td>${r.name}</td><td>${valueCell}</td></tr>`;
  }).join('');

  document.querySelectorAll('#deviceRegisters .regValue').forEach(span => {
    span.addEventListener('dblclick', () => writeDeviceRegisterPrompt(Number(span.dataset.reg), span.textContent));
  });
  const resetBtn = document.getElementById('resetCrcBtn');
  if (resetBtn) {
    resetBtn.addEventListener('click', async () => {
      await fetch('/api/resetcrc', { method: 'POST' });
      refresh();
    });
  }
}

async function writeDeviceRegisterPrompt(reg, currentValue) {
  let url, body, promptText = `Enter new value for register ${reg}:`;
  if (reg === 40002) {
    url = '/api/writestationid';
  } else if (reg === 40003) {
    url = '/api/baud';
    promptText = 'Enter new SPI Baud Rate (1200, 2400, 4800, 9600, or 19200):';
  } else if (reg === 40004) {
    url = '/api/writemodeltype';
    promptText = 'Enter Model Type code:\n' + kModelTypeDescriptions.join('\n');
  } else {
    return;
  }
  const input = prompt(promptText, currentValue);
  if (input === null || input.trim() === '') return;
  body = 'value=' + encodeURIComponent(input.trim());
  try {
    const res = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body });
    if (!res.ok) {
      alert(`Write failed: ${await res.text()}`);
      return;
    }
    refresh();
  } catch (e) {
    alert(`Write request failed: ${e}`);
  }
}

async function refresh() {
  try {
    const res = await fetch('/api/data');
    const data = await res.json();
    document.getElementById('meta').textContent =
      `${data.friendlyName} - uptime ${data.uptimeSec}s`;
    document.getElementById('outcome').textContent = data.pollOutcome;
    document.getElementById('txHex').textContent = data.lastTxHex || '-';
    document.getElementById('rxHex').textContent = data.lastRxHex || '(nothing)';

    const modelKey = data.equipmentType + '/' + data.model;
    if (modelKey !== lastModelKey) {
      lastModelKey = modelKey;
      renderQueries(data.queries);
    }
    renderRegisters(data.registers);
    renderDeviceRegisters(data.deviceRegisters);
    renderBaudButtons(data.spiBaudRate);
    renderEquipButtons(data.equipmentType);
    // ETHERNET is always offered as a model choice, regardless of which
    // equipment type is currently active - not its own equipButtons entry
    // (see kEquipTypes above).
    const modelListByEquip = { Dryer: data.dryerModels, Crystallizer: data.crystallizerModels };
    const models = (modelListByEquip[data.equipmentType] || []).concat(data.ethernetModels);
    renderModelButtons(models, data.model);
  } catch (e) {
    document.getElementById('meta').textContent = 'Failed to reach device: ' + e;
  }
}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)HTML";

    String jsonEscape(const String &in)
    {
        String out;
        out.reserve(in.length());
        for (size_t i = 0; i < in.length(); i++)
        {
            char c = in[i];
            if (c == '"' || c == '\\')
                out += '\\';
            out += c;
        }
        return out;
    }

    void appendStringArray(String &json, const char *const *values, int count)
    {
        json += "[";
        for (int i = 0; i < count; i++)
        {
            if (i)
                json += ",";
            json += "\"" + String(values[i]) + "\"";
        }
        json += "]";
    }

    void handleRoot() { server.send(200, "text/html", kIndexHtml); }

    void handleData()
    {
        String json;
        json.reserve(3072);
        json += "{";
        json += "\"deviceName\":\"" + jsonEscape(Settings.deviceName()) + "\",";
        json += "\"friendlyName\":\"" + jsonEscape(friendlyName()) + "\",";
        json += "\"equipmentType\":\"" + String(equipmentTypeString(Settings.equipmentType())) + "\",";
        json += "\"model\":\"" + jsonEscape(Settings.model()) + "\",";
        json += "\"spiAddress\":" + String(Settings.spiAddress()) + ",";
        json += "\"spiBaudRate\":" + String(SpiIm.baud()) + ",";
        json += "\"uptimeSec\":" + String(millis() / 1000) + ",";
        json += "\"pollOutcome\":\"" + String(outcomeText(SpiIm.lastOutcome())) + "\",";
        json += "\"lastTxHex\":\"" + jsonEscape(SpiIm.lastTxHex()) + "\",";
        json += "\"lastRxHex\":\"" + jsonEscape(SpiIm.lastRxHex()) + "\",";

        json += "\"dryerModels\":";
        appendStringArray(json, DeviceSettings::kDryerModels, DeviceSettings::kDryerModelCount);
        json += ",\"crystallizerModels\":";
        appendStringArray(json, DeviceSettings::kCrystallizerModels, DeviceSettings::kCrystallizerModelCount);
        json += ",\"ethernetModels\":";
        appendStringArray(json, DeviceSettings::kEthernetModels, DeviceSettings::kEthernetModelCount);
        json += ",";

        json += "\"queries\":[";
        for (size_t i = 0; i < ActiveModel->queryCount(); i++)
        {
            if (i)
                json += ",";
            SpiCcpQueryInfo q = ActiveModel->queryInfo(i);
            char cmd1[3], cmd2[3];
            snprintf(cmd1, sizeof(cmd1), "%02X", q.cmd1);
            snprintf(cmd2, sizeof(cmd2), "%02X", q.cmd2);
            json += "{\"name\":\"" + String(q.name) + "\",";
            json += "\"cmd1\":\"" + String(cmd1) + "\",";
            json += "\"cmd2\":\"" + String(cmd2) + "\",";
            json += "\"registers\":\"" + String(q.registers) + "\"}";
        }
        json += "],";

        // Only the registers the active model's own queries actually
        // target (see collectSupportedRegisters()) - not the whole
        // 40010-40041 sheet range, most of which a given model doesn't
        // use at all.
        uint16_t supportedRegs[kMaxSupportedRegisters];
        size_t supportedCount = collectSupportedRegisters(supportedRegs, kMaxSupportedRegisters);
        json += "\"registers\":[";
        for (size_t i = 0; i < supportedCount; i++)
        {
            if (i)
                json += ",";
            uint16_t reg = supportedRegs[i];
            bool present = ActiveModel->hasRegister(reg);
            const char *name = ActiveModel->registerName(reg);
            json += "{\"reg\":" + String(reg) + ",";
            json += "\"name\":\"" + (name ? String(name) : String(reg)) + "\",";
            json += "\"value\":" + String(ActiveModel->getRegister(reg)) + ",";
            json += "\"present\":" + String(present ? "true" : "false") + "}";
        }
        json += "],";

        // XBEE SETUP block (40001-40009) - device config, not per-model
        // SPI-CCP data, so read straight from the live Modbus table
        // (Registers) rather than ActiveModel, guaranteeing this always
        // matches exactly what an external Modbus client sees.
        static constexpr const char *kDeviceRegNames[] = {
            "Software Version", "SPI Station ID", "SPI Baud Rate", "Model Type",
            "RSSI",             "Write Enable",   "Board Temp",    "Xbee Radio Voltage",
            "SPI CRC Error",
        };
        json += "\"deviceRegisters\":[";
        for (uint16_t reg = ModbusReg::kFirstRegister; reg <= ModbusReg::kSpiCrcError; reg++)
        {
            if (reg != ModbusReg::kFirstRegister)
                json += ",";
            json += "{\"reg\":" + String(reg) + ",";
            json += "\"name\":\"" + String(kDeviceRegNames[reg - ModbusReg::kFirstRegister]) + "\",";
            json += "\"value\":" + String(Registers.get(reg - ModbusReg::kFirstRegister)) + "}";
        }
        json += "]}";

        server.send(200, "application/json", json);
    }

    // Fires the next query in the active model's round-robin immediately,
    // instead of waiting for updateDryerReadings()'s 2s loop - lets whoever's
    // holding a scope/logic analyzer on the RS485 line trigger a known-good
    // moment to capture instead of guessing when the next auto-poll lands.
    // Blocks for up to ~1s (SpiIm's poll timeout) before responding.
    void handlePollNow()
    {
        ActiveModel->pollNext();
        handleData();
    }

    // Sends the protocol-mandated ECHO/REVISION commands directly via
    // SpiIm, bypassing the active model's own query table entirely - every
    // SPI-CCP device must support these regardless of model (see
    // DataSheets/SPI Protocol.pdf), so they're the simplest possible frames
    // to go looking for on a scope: if even these get total silence, the
    // fault is upstream of anything model-specific. Uses the active
    // model's own devId/cmd1 (see EquipmentModel::echoCmd1()) so these
    // buttons stay correct no matter which equipment/model is selected.
    void handleEcho()
    {
        uint8_t data[4];
        size_t len;
        uint8_t err;
        SpiIm.poll(ActiveModel->devId(), Settings.spiAddress(), ActiveModel->echoCmd1(), 0x20, data, sizeof(data),
                   len, err);
        handleData();
    }

    void handleVersion()
    {
        uint8_t data[4];
        size_t len;
        uint8_t err;
        SpiIm.poll(ActiveModel->devId(), Settings.spiAddress(), ActiveModel->echoCmd1(), 0x22, data, sizeof(data),
                   len, err);
        handleData();
    }

    // Raw Command Test - sends an arbitrary CMD1/CMD2 pair against the
    // active model's DevID/address, for exploring commands with no known
    // meaning yet (e.g. the crystallizer's still-unidentified 0x3C/0x3E/
    // 0x40/0x42/0x46/0x48). Uses SpiIm::pollRaw() rather than poll() since
    // the reply length isn't known ahead of time for an arbitrary command.
    void handleRawPoll()
    {
        if (!server.hasArg("cmd1") || !server.hasArg("cmd2"))
        {
            server.send(400, "text/plain", "missing cmd1/cmd2");
            return;
        }
        uint8_t cmd1 = static_cast<uint8_t>(strtoul(server.arg("cmd1").c_str(), nullptr, 16));
        uint8_t cmd2 = static_cast<uint8_t>(strtoul(server.arg("cmd2").c_str(), nullptr, 16));
        uint8_t data[128];
        size_t len;
        uint8_t err;
        SpiIm.pollRaw(ActiveModel->devId(), Settings.spiAddress(), cmd1, cmd2, data, sizeof(data), len, err);
        handleData();
    }

    // Raw SELECT (write) test - the write-side counterpart to
    // handleRawPoll(), for exploring a command's write format when it
    // isn't known yet (e.g. the crystallizer's cmd2=0x48 - its read reply
    // is a 5-word block, but the expected write payload shape is
    // unconfirmed). "databytes" is a space-separated hex string (e.g.
    // "00 01") sent verbatim as the SELECT's data - deliberately no
    // length/format assumptions here, unlike EquipmentModel::
    // writeRegister()'s fixed-format writes for registers already
    // confirmed against real hardware.
    void handleRawSelect()
    {
        if (!server.hasArg("cmd1") || !server.hasArg("cmd2"))
        {
            server.send(400, "text/plain", "missing cmd1/cmd2");
            return;
        }
        uint8_t cmd1 = static_cast<uint8_t>(strtoul(server.arg("cmd1").c_str(), nullptr, 16));
        uint8_t cmd2 = static_cast<uint8_t>(strtoul(server.arg("cmd2").c_str(), nullptr, 16));
        uint8_t data[64];
        size_t dataLen = 0;
        if (server.hasArg("databytes"))
        {
            String bytesStr = server.arg("databytes");
            const char *p = bytesStr.c_str();
            while (*p && dataLen < sizeof(data))
            {
                while (*p == ' ')
                    p++;
                if (!*p)
                    break;
                char *end;
                long v = strtol(p, &end, 16);
                if (end == p)
                    break;
                data[dataLen++] = static_cast<uint8_t>(v);
                p = end;
            }
        }
        SpiIm.select(ActiveModel->devId(), Settings.spiAddress(), cmd1, cmd2, data, dataLen);
        handleData();
    }

    // Live Register Data's double-click-to-edit feature (see the
    // dashboard's writeRegisterPrompt()). Delegates to the active model's
    // own EquipmentModel::writeRegister() - currently only Process
    // Setpoint (40010) is wired up there; anything else reports back a
    // clear "not supported" rather than pretending to succeed.
    void handleWriteRegister()
    {
        if (!server.hasArg("reg") || !server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing reg/value");
            return;
        }
        uint16_t reg = static_cast<uint16_t>(strtoul(server.arg("reg").c_str(), nullptr, 10));
        float value = server.arg("value").toFloat();
        if (!ActiveModel->writeRegister(reg, value))
        {
            server.send(400, "text/plain", "write not supported for this register (or the device didn't ack it)");
            return;
        }
        handleData();
    }

    // Device Settings writes (40002/40004) - separate from
    // handleWriteRegister() above since these are device config
    // (DeviceSettings/ModelFactory), not SPI-CCP data the active model
    // owns. 40003 (SPI Baud Rate) reuses the existing handleSetBaud()
    // below instead of a third near-identical handler.
    void handleWriteStationId()
    {
        if (!server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing value");
            return;
        }
        uint8_t addr = static_cast<uint8_t>(server.arg("value").toInt());
        Settings.setSpiAddress(addr);
        ActiveModel->begin(ActiveModel->devId(), addr);
        Registers.set(ModbusReg::kSpiStationId - ModbusReg::kFirstRegister, addr);
        handleData();
    }

    void handleWriteModelType()
    {
        if (!server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing value");
            return;
        }
        int code = server.arg("value").toInt();
        if (code < 0 || code > 7)
        {
            server.send(400, "text/plain", "invalid model type code (0-7)");
            return;
        }
        Settings.setModelTypeCode(static_cast<uint8_t>(code));
        SelectEquipmentModel();
        Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, static_cast<uint16_t>(code));
        handleData();
    }

    void handleResetCrc()
    {
        SpiIm.resetCrcErrorCount();
        Registers.set(ModbusReg::kSpiCrcError - ModbusReg::kFirstRegister, 0);
        handleData();
    }

    // Discovery (see lib/Discovery) - both scans run incrementally from
    // Discovery.task(), called every loop(), so these handlers only ever
    // start/stop/report status; they never block waiting for a scan to
    // finish.
    void handleDiscoverDevIdStart()
    {
        Discovery.startDevIdScan(Settings.spiAddress());
        server.send(200, "text/plain", "OK");
    }

    void handleDiscoverCommandStart()
    {
        if (!server.hasArg("devid"))
        {
            server.send(400, "text/plain", "missing devid");
            return;
        }
        uint8_t devId = static_cast<uint8_t>(strtoul(server.arg("devid").c_str(), nullptr, 16));
        Discovery.startCommandScan(devId, Settings.spiAddress());
        server.send(200, "text/plain", "OK");
    }

    void handleDiscoverStop()
    {
        Discovery.stop();
        server.send(200, "text/plain", "OK");
    }

    void handleDiscoverStatus()
    {
        String json;
        json.reserve(4096);
        json += "{";
        json += "\"running\":" + String(Discovery.isRunning() ? "true" : "false") + ",";
        const char *modeStr = Discovery.mode() == DiscoveryScanner::Mode::kDevIdScan     ? "devid"
                              : Discovery.mode() == DiscoveryScanner::Mode::kCommandScan ? "command"
                                                                                          : "none";
        json += "\"mode\":\"" + String(modeStr) + "\",";
        json += "\"progressCurrent\":" + String(Discovery.progressCurrent()) + ",";
        json += "\"progressTotal\":" + String(Discovery.progressTotal()) + ",";
        char scanDevIdHex[3];
        snprintf(scanDevIdHex, sizeof(scanDevIdHex), "%02X", Discovery.commandScanDevId());
        json += "\"commandScanDevId\":\"" + String(scanDevIdHex) + "\",";

        json += "\"devIds\":[";
        for (uint8_t i = 0; i < Discovery.devIdCount(); i++)
        {
            if (i)
                json += ",";
            char hex[3];
            snprintf(hex, sizeof(hex), "%02X", Discovery.devIdAt(i));
            json += "\"" + String(hex) + "\"";
        }
        json += "],";

        json += "\"commands\":[";
        for (uint8_t i = 0; i < Discovery.commandCount(); i++)
        {
            if (i)
                json += ",";
            const DiscoveredCommand &c = Discovery.commandAt(i);
            char cmd1Hex[3], cmd2Hex[3];
            snprintf(cmd1Hex, sizeof(cmd1Hex), "%02X", c.cmd1);
            snprintf(cmd2Hex, sizeof(cmd2Hex), "%02X", c.cmd2);
            json += "{\"cmd1\":\"" + String(cmd1Hex) + "\",\"cmd2\":\"" + String(cmd2Hex) + "\",";
            json += "\"data\":\"" + jsonEscape(hexDumpBytes(c.data, c.dataLen)) + "\",";
            json += "\"isNew\":" + String(isNewCommand(c.cmd1, c.cmd2) ? "true" : "false") + "}";
        }
        json += "]}";

        server.send(200, "application/json", json);
    }

    // Static server-rendered snapshot (no JS/auto-refresh) meant to be
    // saved via the browser's own Print -> Save as PDF - generating an
    // actual PDF file on-device isn't practical here (no flash/RAM budget
    // for a PDF library, fonts, etc.), so this leans on the browser to do
    // that conversion instead.
    void handleReport()
    {
        String html;
        html.reserve(4096);
        html += "<!DOCTYPE html><html><head><meta charset=\"utf-8\">";
        html += "<title>SPI-IM Discovery Report</title><style>";
        html += "body{font-family:Arial,Helvetica,sans-serif;margin:24px;color:#111;}";
        html += "h1{font-size:1.4em;} h2{font-size:1.1em;margin-top:28px;}";
        html += "table{border-collapse:collapse;width:100%;margin-top:8px;}";
        html += "th,td{border:1px solid #999;padding:4px 8px;text-align:left;font-size:0.9em;}";
        html += "th{background:#eee;} .new{font-weight:bold;}";
        html += "@media print { .noprint {display:none;} }";
        html += "</style></head><body>";
        html += "<p class=\"noprint\"><a href=\"#\" onclick=\"window.print();return false;\">Print / Save as PDF</a></p>";
        html += "<h1>SPI-IM Discovery Report</h1>";
        html += "<p>Device: " + jsonEscape(Settings.deviceName()) + " &middot; Equipment: " +
                String(equipmentTypeString(Settings.equipmentType())) +
                " &middot; Model: " + jsonEscape(Settings.model()) + " &middot; SPI addr 0x" +
                String(Settings.spiAddress(), HEX) + "</p>";
        html += "<p>Generated at device uptime " + String(millis() / 1000) + "s</p>";

        html += "<h2>Discovered DEVIDs (0x20-0xFF scan)</h2>";
        if (Discovery.devIdCount() == 0)
        {
            html += "<p>No DEVID scan results yet.</p>";
        }
        else
        {
            html += "<table><thead><tr><th>DEVID</th></tr></thead><tbody>";
            for (uint8_t i = 0; i < Discovery.devIdCount(); i++)
            {
                char hex[3];
                snprintf(hex, sizeof(hex), "%02X", Discovery.devIdAt(i));
                html += "<tr><td>0x" + String(hex) + "</td></tr>";
            }
            html += "</tbody></table>";
        }

        char scanDevIdHex[3];
        snprintf(scanDevIdHex, sizeof(scanDevIdHex), "%02X", Discovery.commandScanDevId());
        html += "<h2>Discovered Commands (DEVID 0x" + String(scanDevIdHex) + " scan)</h2>";
        if (Discovery.commandCount() == 0)
        {
            html += "<p>No command scan results yet.</p>";
        }
        else
        {
            html += "<table><thead><tr><th>CMD1</th><th>CMD2</th><th>Data</th><th>New?</th></tr></thead><tbody>";
            for (uint8_t i = 0; i < Discovery.commandCount(); i++)
            {
                const DiscoveredCommand &c = Discovery.commandAt(i);
                char cmd1Hex[3], cmd2Hex[3];
                snprintf(cmd1Hex, sizeof(cmd1Hex), "%02X", c.cmd1);
                snprintf(cmd2Hex, sizeof(cmd2Hex), "%02X", c.cmd2);
                bool isNew = isNewCommand(c.cmd1, c.cmd2);
                html += "<tr><td>0x" + String(cmd1Hex) + "</td><td>0x" + String(cmd2Hex) + "</td><td>" +
                        jsonEscape(hexDumpBytes(c.data, c.dataLen)) + "</td><td" +
                        (isNew ? " class=\"new\">NEW" : ">") + "</td></tr>";
            }
            html += "</tbody></table>";
        }

        html += "</body></html>";
        server.send(200, "text/html", html);
    }

    // Applies immediately (no restart), same as writing Modbus register
    // 40003 - see DryerRegisters.cpp's onSetBaudRate() for the equivalent
    // Modbus TCP path. Keeps that register mirrored so both surfaces agree.
    void handleSetBaud()
    {
        if (!server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing value");
            return;
        }
        uint32_t baud = server.arg("value").toInt();
        if (!isValidSpiBaud(baud))
        {
            server.send(400, "text/plain", "invalid baud");
            return;
        }
        Settings.setSpiBaudRate(baud);
        SpiIm.begin(baud);
        Registers.set(ModbusReg::kSpiBaudRate - ModbusReg::kFirstRegister, baud);
        server.send(200, "text/plain", "OK");
    }

    // Applies immediately - re-selects and begin()s the matching
    // EquipmentModel (see ModelFactory). Resets Model to the new equipment
    // type's first option, same as the Tough's Gateway Settings page does,
    // since whatever model string was stored for the old type generally
    // isn't valid for the new one.
    void handleSetEquipment()
    {
        if (!server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing value");
            return;
        }
        String value = server.arg("value");
        DeviceSettings::EquipmentType type;
        const char *defaultModel;
        if (value == "Dryer")
        {
            type = DeviceSettings::EquipmentType::Dryer;
            defaultModel = DeviceSettings::kDryerModels[0];
        }
        else if (value == "Crystallizer")
        {
            type = DeviceSettings::EquipmentType::Crystallizer;
            defaultModel = DeviceSettings::kCrystallizerModels[0];
        }
        else
        {
            // No "Ethernet" branch here - it's a model choice, not its own
            // equipment-type button (see DeviceSettings.h's kEthernetModels
            // comment), so it's only ever reachable via /api/model below.
            server.send(400, "text/plain", "invalid equipment type");
            return;
        }
        Settings.setEquipmentType(type);
        Settings.setModel(defaultModel);
        SelectEquipmentModel();
        Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
        server.send(200, "text/plain", "OK");
    }

    // Applies immediately. Validated against the current equipment type's
    // own model list (DeviceSettings::kDryerModels/kCrystallizerModels) -
    // rejects anything else rather than silently accepting a model string
    // that doesn't match any real EquipmentModel subclass. ETHERNET is
    // checked first, regardless of current equipment type, since it's a
    // model choice reachable from any equipment type (see
    // DeviceSettings.h's kEthernetModels comment) - deliberately does NOT
    // touch equipmentType(), leaving whichever of Dryer/Crystallizer was
    // already set untouched, exactly like picking FC vs. FD doesn't change
    // equipment type either.
    void handleSetModel()
    {
        if (!server.hasArg("value"))
        {
            server.send(400, "text/plain", "missing value");
            return;
        }
        String value = server.arg("value");
        if (value == DeviceSettings::kEthernetModels[0])
        {
            Settings.setModel(value);
            SelectEquipmentModel();
            Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
            server.send(200, "text/plain", "OK");
            return;
        }
        const char *const *options;
        int count;
        switch (Settings.equipmentType())
        {
        case DeviceSettings::EquipmentType::Dryer:
            options = DeviceSettings::kDryerModels;
            count = DeviceSettings::kDryerModelCount;
            break;
        case DeviceSettings::EquipmentType::Crystallizer:
        default:
            options = DeviceSettings::kCrystallizerModels;
            count = DeviceSettings::kCrystallizerModelCount;
            break;
        }
        bool valid = false;
        for (int i = 0; i < count; i++)
        {
            if (value == options[i])
            {
                valid = true;
                break;
            }
        }
        if (!valid)
        {
            server.send(400, "text/plain", "invalid model for current equipment type");
            return;
        }
        Settings.setModel(value);
        SelectEquipmentModel();
        Registers.set(ModbusReg::kModelType - ModbusReg::kFirstRegister, Settings.modelTypeCode());
        server.send(200, "text/plain", "OK");
    }

} // namespace

void DryerWebServer::begin()
{
    server.on("/", handleRoot);
    server.on("/api/data", handleData);
    server.on("/api/baud", HTTP_POST, handleSetBaud);
    server.on("/api/equipment", HTTP_POST, handleSetEquipment);
    server.on("/api/model", HTTP_POST, handleSetModel);
    server.on("/api/poll", HTTP_POST, handlePollNow);
    server.on("/api/echo", HTTP_POST, handleEcho);
    server.on("/api/version", HTTP_POST, handleVersion);
    server.on("/api/rawpoll", HTTP_POST, handleRawPoll);
    server.on("/api/rawselect", HTTP_POST, handleRawSelect);
    server.on("/api/writeregister", HTTP_POST, handleWriteRegister);
    server.on("/api/writestationid", HTTP_POST, handleWriteStationId);
    server.on("/api/writemodeltype", HTTP_POST, handleWriteModelType);
    server.on("/api/resetcrc", HTTP_POST, handleResetCrc);
    server.on("/api/discover/devid/start", HTTP_POST, handleDiscoverDevIdStart);
    server.on("/api/discover/command/start", HTTP_POST, handleDiscoverCommandStart);
    server.on("/api/discover/stop", HTTP_POST, handleDiscoverStop);
    server.on("/api/discover/status", handleDiscoverStatus);
    server.on("/report", handleReport);
    server.begin();
}

void DryerWebServer::handleClient() { server.handleClient(); }
