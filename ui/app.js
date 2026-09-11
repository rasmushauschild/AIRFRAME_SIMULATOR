import { createScene } from '/static/scene.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
const api = async (path, body, method) => {
  const r = await fetch(path, { method: method || (body === undefined ? 'GET' : 'POST'),
    headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
  let j = null; try { j = await r.json(); } catch { }
  if (!r.ok) throw new Error((j && j.error) || r.statusText);
  return j;
};
const fmt = (v, d = 3) => (typeof v === 'number' ? v.toFixed(d) : String(v));
const deg = (r) => r * 180 / Math.PI;

let airframe = null;
let selected = -1;
let status = {};
let params = {};
let meta = {};
let pushTimer = null;
let joyWs = null;          // websocket handle used by the USB remote (declared early: connectWs() runs before the joystick code)

// ============================================================ scene
let scene;
try {
  scene = createScene($('#c'), {
    onSelect: (i) => { selected = i; renderRotorTable(); renderReadout(); },
    onRotorChanged: (i, r, commit) => { renderRotorRow(i); renderReadout(); if (commit) pushAirframe(); },
  });
} catch (e) {
  // no WebGL (hidden window, remote desktop, old GPU): keep the rest of the app working without the 3D view
  console.warn('3D view unavailable:', e.message);
  const noop = () => { };
  scene = { setAirframe: noop, updateState: noop, select: noop, updateRotorNode: noop, setTheme: noop, setFollow: noop, setMode: noop, selected: -1, focusOrigin: noop };
  $('#viewport').insertAdjacentHTML('afterbegin', '<div class="hint" style="padding:18px">3D view unavailable in this window (no WebGL). Everything else works.</div>');
}

// ============================================================ tabs
$$('.tabs button').forEach(b => b.addEventListener('click', () => {
  $$('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  $$('.tab').forEach(t => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  if (b.dataset.tab === 'airframe') { ensureParams(); loadExport(); }
  if (b.dataset.tab === 'connect') refreshConnection();
}));
function openTab(name) { $$('.tabs button').find(b => b.dataset.tab === name)?.click(); }

// ============================================================ airframe editing
function pushAirframe(immediate = false) {
  clearTimeout(pushTimer);
  const doPush = async () => {
    try {
      const res = await api('/api/airframe', { airframe, keep_state: true });
      if (res.airframe && res.airframe.leg_points) { airframe.leg_points = res.airframe.leg_points; scene.setAirframe(airframe); }
      showProblems(res.problems);
      showHover(res.hover);
      markDirty();
      loadExport();
    } catch (e) { logLine('[ui] airframe rejected: ' + e.message); }
  };
  if (immediate) doPush(); else pushTimer = setTimeout(doPush, 120);
}
function showProblems(p) { $('#af-problems').textContent = (p && p.length) ? '⚠ ' + p.join('\n⚠ ') : ''; }
function showHover(h) {
  if (!h || !h.shares) return;
  $$('#rotor-table tr[data-i]').forEach((tr, i) => {
    const cell = tr.querySelector('td.hover'); if (!cell) return;
    const u = h.hover_utilisation ? h.hover_utilisation[i] : null;
    const bad = h.negative && h.negative.includes(i + 1);
    cell.innerHTML = bad ? '<span class="err" title="allocator needs negative thrust here">neg</span>'
      : (u == null ? '' : `<span class="bar ${u > 0.85 ? 'warn-bar' : ''}" style="width:${Math.round(Math.min(u, 1) * 40)}px" title="hover: ${(u * 100).toFixed(0)}% of max thrust"></span>`);
  });
}

function setAirframe(af) {
  airframe = af;
  scene.setAirframe(af);
  $('#af-name').value = af.name;
  $('#title-name').textContent = af.name;
  $('#af-mass').value = af.mass;
  ['ixx', 'iyy', 'izz'].forEach((k, i) => $('#af-' + k).value = +af.inertia[i].toFixed(5));
  $('#af-hover').value = af.hover_pitch_deg || 0;
  ['dragx', 'dragy', 'dragz'].forEach((k, i) => $('#af-' + k).value = af.drag_quadratic[i]);
  ['bx', 'by', 'bz'].forEach((k, i) => $('#af-' + k).value = af.body_size[i]);
  $('#af-landed').value = af.landed_pitch_deg || 0;
  $('#af-legz').value = af.leg_height ?? 0.2;
  $('#af-legxy').value = af.leg_spread ?? 0.2;
  renderRotorTable();
  renderMotorSliders();
  fillMotorCard();
  airframe.px4_overrides = airframe.px4_overrides || {};
  loadExport();          // fills the edited/affected-parameters section (needs the server's export view)
}
const KIND_DEFAULTS = { prop: { km: 0.05, tau: 0.04, prop_diameter: 0.25, thrust_exponent: 2, ram_drag: false },
                        ducted: { km: 0.01, tau: 0.12, prop_diameter: 0.12, thrust_exponent: 2, ram_drag: true } };
function fillMotorCard() {
  const r = airframe.rotors[selected >= 0 ? selected : 0]; if (!r) return;
  $('#m-kind').value = r.kind || 'prop'; $('#m-tmax').value = r.max_thrust; $('#m-tau').value = r.tau;
  $('#m-km').value = Math.abs(r.km); $('#m-dia').value = r.prop_diameter; $('#m-exp').value = r.thrust_exponent; $('#m-ram').checked = !!r.ram_drag;
}
function applyMotorCard(kindChanged) {
  const kind = $('#m-kind').value;
  if (kindChanged) { const d = KIND_DEFAULTS[kind]; $('#m-tau').value = d.tau; $('#m-km').value = d.km; $('#m-dia').value = d.prop_diameter; $('#m-exp').value = d.thrust_exponent; $('#m-ram').checked = d.ram_drag; }
  const tmax = +$('#m-tmax').value, tau = +$('#m-tau').value, km = Math.abs(+$('#m-km').value), dia = +$('#m-dia').value, ex = +$('#m-exp').value, ram = $('#m-ram').checked;
  airframe.rotors.forEach(r => { r.kind = kind; r.max_thrust = tmax; r.tau = tau; r.km = (r.km >= 0 ? 1 : -1) * km; r.prop_diameter = dia; r.thrust_exponent = ex; r.ram_drag = ram; });
  setAirframe(airframe); pushAirframe(true);
}
$('#m-kind').addEventListener('change', () => applyMotorCard(true));
['m-tmax', 'm-tau', 'm-km', 'm-dia', 'm-exp', 'm-ram'].forEach(id => $('#' + id).addEventListener('change', () => applyMotorCard(false)));

function bindNumber(id, fn) {
  $('#' + id).addEventListener('change', (e) => { fn(parseFloat(e.target.value)); scene.setAirframe(airframe); pushAirframe(true); });
}
$('#af-name').addEventListener('change', e => { airframe.name = e.target.value; $('#title-name').textContent = airframe.name; pushAirframe(true); });
bindNumber('af-mass', v => airframe.mass = v);
bindNumber('af-ixx', v => airframe.inertia[0] = v);
bindNumber('af-iyy', v => airframe.inertia[1] = v);
bindNumber('af-izz', v => airframe.inertia[2] = v);
bindNumber('af-hover', v => airframe.hover_pitch_deg = v);
bindNumber('af-dragx', v => airframe.drag_quadratic[0] = v);
bindNumber('af-dragy', v => airframe.drag_quadratic[1] = v);
bindNumber('af-dragz', v => airframe.drag_quadratic[2] = v);
bindNumber('af-bx', v => airframe.body_size[0] = v);
bindNumber('af-by', v => airframe.body_size[1] = v);
bindNumber('af-bz', v => airframe.body_size[2] = v);
const setLegs = () => {
  airframe.landed_pitch_deg = parseFloat($('#af-landed').value) || 0;
  airframe.leg_height = parseFloat($('#af-legz').value) || 0.2;
  airframe.leg_spread = parseFloat($('#af-legxy').value) || 0.2;
};
bindNumber('af-landed', setLegs);
bindNumber('af-legz', setLegs);
bindNumber('af-legxy', setLegs);
$('#af-estimate').addEventListener('click', async () => {
  await api('/api/airframe', { airframe, keep_state: true });
  const r = await api('/api/airframe/estimate_inertia', {});
  airframe.inertia = r.inertia; setAirframe(airframe);
});
$('#af-save').addEventListener('click', async () => {
  const name = airframe.name.trim().toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '') || 'airframe';
  await api('/api/airframe', { airframe, keep_state: true });
  const r = await api('/api/airframe/save', { name });
  logLine('[ui] saved ' + r.path);
  const b = $('#af-save'); b.textContent = 'Saved'; setTimeout(() => b.textContent = 'Save', 1500);
  loadPresetList();
});
async function loadPresetList() {
  const r = await api('/api/airframes');
  const sel = $('#af-preset');
  sel.innerHTML = '<option value="">Choose…</option>' +
    r.files.map(f => `<option value="${f}">${f.replace(/\.json$/, '')}</option>`).join('') +
    r.presets.map(p => `<option value="${p}">built-in ${p}</option>`).join('');
}
$('#af-preset').addEventListener('change', async (e) => {
  if (!e.target.value) return;
  const r = await api('/api/airframe/load', { name: e.target.value });
  selected = -1; setAirframe(r.airframe); showProblems(r.problems); showHover(r.hover);
  e.target.value = '';
});

// --- rotor helpers: axis <-> tilt/direction
const axisToTilt = (a) => {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  const tilt = deg(Math.acos(Math.max(-1, Math.min(1, -a[2] / n))));
  const dir = (Math.hypot(a[0], a[1]) < 1e-6) ? 0 : deg(Math.atan2(a[1], a[0]));
  return [tilt, dir];
};
const tiltToAxis = (tilt, dir) => {
  const t = tilt * Math.PI / 180, d = dir * Math.PI / 180;
  return [+(Math.sin(t) * Math.cos(d)).toFixed(4), +(Math.sin(t) * Math.sin(d)).toFixed(4), +(-Math.cos(t)).toFixed(4)];
};

function renderRotorTable() {
  const el = $('#rotor-table');
  const rows = airframe.rotors.map((r, i) => rotorRowHtml(i, r)).join('');
  el.innerHTML = `<table class="grid"><thead><tr><th>#</th><th title="position, m">X</th><th>Y</th><th>Z</th><th title="tilt from vertical, degrees">Tilt°</th><th title="direction of tilt: 0 = forward, 90 = right, 180 = back, -90 = left">Dir°</th><th title="resulting unit thrust vector = CA_ROTORn_AX / AY / AZ">Axis AX AY AZ</th><th>Spin</th><th title="max thrust N">Tmax</th><th title="share of max thrust this rotor needs to hover, as PX4's allocator would solve it">Hover</th><th></th></tr></thead><tbody>${rows}</tbody></table>`;
  api('/api/airframe/hover_check').then(showHover).catch(() => { });
  el.querySelectorAll('tr[data-i]').forEach(tr => {
    const i = +tr.dataset.i;
    tr.addEventListener('click', (e) => { if (e.target.tagName !== 'INPUT' && !e.target.classList.contains('spin') && !e.target.classList.contains('del')) { selected = i; scene.select(i); renderRotorTable(); renderReadout(); } });
    tr.querySelectorAll('input').forEach(inp => inp.addEventListener('change', () => applyRow(i, tr)));
    tr.querySelector('.spin').addEventListener('click', () => { airframe.rotors[i].km = -airframe.rotors[i].km; scene.updateRotorNode(i, airframe.rotors[i]); renderRotorTable(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { airframe.rotors.splice(i, 1); selected = -1; setAirframe(airframe); pushAirframe(true); });
  });
}
const axisText = (a) => a.map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(' ');
function rotorRowHtml(i, r) {
  const ccw = r.km >= 0;
  const [tilt, dir] = axisToTilt(r.axis);
  return `<tr data-i="${i}" class="${i === selected ? 'selected' : ''}"><td class="idx">${i + 1}</td>
  <td><input type="number" step="0.005" data-k="x" value="${r.pos[0]}"></td>
  <td><input type="number" step="0.005" data-k="y" value="${r.pos[1]}"></td>
  <td><input type="number" step="0.005" data-k="z" value="${r.pos[2]}"></td>
  <td><input type="number" step="1" data-k="tilt" value="${+tilt.toFixed(1)}"></td>
  <td><input type="number" step="5" data-k="dir" value="${+dir.toFixed(1)}"></td>
  <td class="axis" title="CA_ROTOR${i}_AX / AY / AZ">${axisText(r.axis)}</td>
  <td><span class="spin ${ccw ? 'ccw' : 'cw'}" title="click to flip (KM=${r.km})">${ccw ? 'CCW' : 'CW'}</span></td>
  <td><input type="number" step="0.5" data-k="tmax" value="${r.max_thrust}"></td>
  <td class="hover"></td>
  <td><button class="del" title="remove rotor">✕</button></td></tr>`;
}
function renderRotorRow(i) {
  const tr = document.querySelector(`#rotor-table tr[data-i="${i}"]`);
  if (!tr) return;
  const r = airframe.rotors[i];
  const set = (k, v) => { const inp = tr.querySelector(`input[data-k="${k}"]`); if (inp && document.activeElement !== inp) inp.value = v; };
  const [tilt, dir] = axisToTilt(r.axis);
  set('x', r.pos[0]); set('y', r.pos[1]); set('z', r.pos[2]); set('tilt', +tilt.toFixed(1)); set('dir', +dir.toFixed(1));
  const ax = tr.querySelector('td.axis'); if (ax) ax.textContent = axisText(r.axis);
}
function applyRow(i, tr) {
  const r = airframe.rotors[i];
  const g = (k) => parseFloat(tr.querySelector(`input[data-k="${k}"]`).value);
  r.pos = [g('x'), g('y'), g('z')];
  r.axis = tiltToAxis(g('tilt'), g('dir'));
  r.max_thrust = g('tmax');
  scene.updateRotorNode(i, r);
  renderRotorRow(i);
  renderReadout();
  pushAirframe(true);
}
function renderReadout() {
  const el = $('#rotor-readout');
  if (selected < 0 || !airframe.rotors[selected]) { el.classList.remove('show'); return; }
  const r = airframe.rotors[selected];
  const [tilt, dir] = axisToTilt(r.axis);
  el.classList.add('show');
  el.innerHTML = `<b>Motor ${selected + 1}</b> pos [${r.pos.map(v => fmt(v)).join(', ')}] · axis [${r.axis.map(v => fmt(v, 3)).join(', ')}] (${tilt.toFixed(1)}° from vertical) · ${r.km >= 0 ? 'CCW' : 'CW'} · CA_ROTOR${selected}_*`;
}
$('#rotor-add').addEventListener('click', () => {
  const base = airframe.rotors[selected] || airframe.rotors[airframe.rotors.length - 1] || { pos: [0.2, 0, 0], axis: [0, 0, -1], km: 0.05, max_thrust: 8, tau: 0.04, prop_diameter: 0.25, thrust_exponent: 2, kind: 'prop', ram_drag: false };
  const n = JSON.parse(JSON.stringify(base));
  n.pos = [n.pos[0] + 0.05, n.pos[1] + 0.05, n.pos[2]];
  airframe.rotors.push(n); selected = airframe.rotors.length - 1;
  setAirframe(airframe); scene.select(selected); pushAirframe(true);
});
function mirror(axisIdx) {
  if (selected < 0) return;
  const n = JSON.parse(JSON.stringify(airframe.rotors[selected]));
  n.pos[axisIdx] = -n.pos[axisIdx]; n.axis[axisIdx] = -n.axis[axisIdx]; n.km = -n.km;
  airframe.rotors.push(n); selected = airframe.rotors.length - 1;
  setAirframe(airframe); scene.select(selected); pushAirframe(true);
}
$('#rotor-mirror-y').addEventListener('click', () => mirror(1));
$('#rotor-mirror-x').addEventListener('click', () => mirror(0));
$('#rotor-apply-all').addEventListener('click', () => {
  if (selected < 0) return;
  const s = airframe.rotors[selected];
  airframe.rotors.forEach(r => { r.max_thrust = s.max_thrust; r.tau = s.tau; r.prop_diameter = s.prop_diameter; r.thrust_exponent = s.thrust_exponent; r.kind = s.kind; r.ram_drag = s.ram_drag; r.km = (r.km >= 0 ? 1 : -1) * Math.abs(s.km); });
  setAirframe(airframe); pushAirframe(true);
});

// ============================================================ PX4 export
async function loadExport() {
  let r;
  try { r = await api('/api/px4/export'); } catch (e) { return; }
  lastExport = r;
  renderOverrides(r);
  $('#px4-export-status').innerHTML = r.problems.length ? `<div class="problems">⚠ ${r.problems.join('<br>⚠ ')}</div>` : '';
}
let lastExport = null;
$('#px4-refresh').addEventListener('click', loadExport);
async function pushToPX4(statusEl, short = false) {
  statusEl.innerHTML = '<span class="muted">Pushing…</span>';
  try {
    await api('/api/airframe', { airframe, keep_state: true });
    const r = await api('/api/px4/push', { save: true });
    const failed = r.results.filter(x => !x.ok);
    statusEl.innerHTML = r.ok
      ? `<span class="ok">✓ PX4 updated (${r.results.length} parameters)</span>`
      : `<span class="err">${failed.length} failed: ${failed.map(f => f.name + ' (' + f.error + ')').join(', ')}</span>`;
    if (!short && r.missing && r.missing.length) statusEl.innerHTML += `<div class="muted">not present in this firmware: ${r.missing.join(', ')}</div>`;
    geometryDirty = false;
    await loadExport();
  } catch (e) { statusEl.innerHTML = `<span class="err">${e.message}</span>`; }
}
$('#btn-update').addEventListener('click', () => {
  if (status.armed) { $('#update-status').innerHTML = '<span class="err">Disarm before updating PX4</span>'; return; }
  pushToPX4($('#update-status'), true);
});
let geometryDirty = false;
function markDirty() { geometryDirty = true; $('#update-status').innerHTML = '<span class="warn">Changed since last update</span>'; }
function updateFooter() {
  const armBtn = $('#btn-arm');
  armBtn.textContent = status.armed ? 'Kill' : 'Arm';
  armBtn.title = status.armed ? 'Force disarm immediately (motors stop, even in the air)' : 'Arm the vehicle';
  armBtn.classList.toggle('armed', !!status.armed);
  armBtn.disabled = !status.ctl_connected || (!status.armed && !status.arm_ready);
  if (!status.armed && status.ctl_connected && !status.arm_ready) armBtn.title = 'Not armable yet: ' + (status.arm_block_reason || 'estimator not ready');
  const upd = $('#btn-update');
  upd.disabled = !status.ctl_connected || !!status.armed;
  upd.title = status.armed ? 'Disarm first: PX4 rebuilds its allocation when these parameters change' :
    (status.ctl_connected ? 'Write the rotor geometry and output mapping to the flight controller and save it' : 'PX4 not connected');
  $$('#mode-pills .pill').forEach(b => b.classList.toggle('active', status.connected && (status.mode_name || '').toLowerCase() === b.textContent.toLowerCase()));
}
$('#btn-arm').addEventListener('click', async () => {
  try { await api('/api/px4/command', status.armed ? { command: 'kill', force: true } : { command: 'arm' }); } catch (e) { logLine('[ui] ' + e.message); }
});

// ============================================================ edited parameters (overrides saved with the airframe)
function renderOverrides(r) {
  r = r || lastExport;
  const ov = (r && r.overrides) || (airframe && airframe.px4_overrides) || {};
  if (airframe) airframe.px4_overrides = ov;
  const el = $('#overrides');
  const f = (x) => (typeof x === 'number' && !Number.isInteger(x)) ? +x.toFixed(4) : x;
  const cur = (r && r.current) || {};
  const same = (k, v) => cur[k] != null && Math.abs(+cur[k] - +v) < 1e-4;
  const vehicleCell = (k, v) => `<span class="ov-cur ${cur[k] == null ? 'muted' : same(k, v) ? 'ok' : 'warn'}" title="value on the vehicle">${cur[k] == null ? '—' : f(cur[k])}</span>`;
  let html = '';
  // hand-edited, editable
  Object.keys(ov).sort().forEach(n => {
    const m = meta[n] || {};
    html += `<div class="ov-row"><span class="pname">${n}</span><input type="number" step="any" value="${ov[n]}" data-n="${n}">${vehicleCell(n, ov[n])}<span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span><button class="del" data-n="${n}" title="forget this edit">✕</button></div>`;
  });
  // derived from the geometry, read-only, grouped per rotor
  if (r && r.params) {
    const P = r.params, keys = r.geometry_keys || [];
    const rotors = {}; const rest = [];
    keys.forEach(k => { const m = k.match(/^CA_ROTOR(\d+)_(PX|PY|PZ|AX|AY|AZ|KM)$/); if (m) (rotors[m[1]] = rotors[m[1]] || {})[m[2]] = P[k]; else if (!/^(HIL_ACT_FUNC|PWM_MAIN_FUNC)\d+$/.test(k)) rest.push(k); });
    const diff = (prefix) => keys.filter(k => k.startsWith(prefix)).some(k => !same(k, P[k]));
    if (html) html += '<div class="ov-sep"></div>';
    html += '<div class="hint" style="margin:2px 0 4px">From the geometry above · read-only</div>';
    rest.forEach(k => { html += `<div class="ov-row ro"><span class="pname">${k}</span><span class="ov-val">${f(P[k])}</span>${vehicleCell(k, P[k])}<span class="pdesc">${(meta[k] || {}).short || ''}</span><span></span></div>`; });
    Object.keys(rotors).sort((a, b) => a - b).forEach(i => {
      const R = rotors[i]; const d = diff(`CA_ROTOR${i}_`);
      html += `<div class="ov-row ro rotor"><span class="pname">CA_ROTOR${i}_*</span><span class="ov-val rot">P ${f(R.PX)} ${f(R.PY)} ${f(R.PZ)} · A ${f(R.AX)} ${f(R.AY)} ${f(R.AZ)} · KM ${f(R.KM)}</span><span class="ov-cur ${d ? 'warn' : 'ok'}">${d ? '≠ vehicle' : '✓'}</span><span class="pdesc">motor ${+i + 1}</span><span></span></div>`;
    });
    const funcs = keys.filter(k => /^(HIL_ACT_FUNC|PWM_MAIN_FUNC)\d+$/.test(k)).sort((a, b) => parseInt(a.match(/\d+$/)[0]) - parseInt(b.match(/\d+$/)[0]));
    if (funcs.length) {
      const on = funcs.filter(k => P[k] > 0), d = funcs.some(k => !same(k, P[k]));
      html += `<div class="ov-row ro"><span class="pname">${funcs[0].replace(/\d+$/, '')}1‑${funcs.length}</span><span class="ov-val rot">${on.map(k => P[k]).join(', ')}${on.length < funcs.length ? ', rest 0' : ''}</span><span class="ov-cur ${d ? 'warn' : 'ok'}">${d ? '≠ vehicle' : '✓'}</span><span class="pdesc">output → motor mapping</span><span></span></div>`;
    }
  }
  el.innerHTML = html || '<div class="hint">Nothing yet. Edit any parameter below and it appears here.</div>';
  $$('#overrides input').forEach(inp => inp.addEventListener('change', () => setParamValue(inp.dataset.n, parseFloat(inp.value))));
  $$('#overrides .del').forEach(b => b.addEventListener('click', async () => { await api('/api/airframe/override_remove', { name: b.dataset.n }); delete airframe.px4_overrides[b.dataset.n]; loadExport(); markDirty(); }));
}
async function setParamValue(n, value) {
  if (status.ctl_connected) {
    const r = await api('/api/params/set', { name: n, value });
    if (!r.ok) { logLine('[ui] ' + n + ': ' + r.error); return r; }
    params[n] = params[n] || { type: 9 }; params[n].value = r.value;
    airframe.px4_overrides[n] = r.value;
  } else {
    await api('/api/airframe/override', { name: n, value });
    airframe.px4_overrides[n] = value;
  }
  loadExport(); renderParams(); markDirty();
  return { ok: true, value };
}

// ============================================================ parameters
let paramsLoaded = false;
async function ensureParams() {
  if (paramsLoaded) return;
  await loadMeta();
  await loadParams();
}
async function loadMeta() {
  const r = await api('/api/params/meta');
  meta = r.meta || {};
  const groups = [...new Set(Object.values(meta).map(m => m.group))].sort();
  $('#param-group').innerHTML = '<option value="">all groups</option>' + groups.map(g => `<option>${g}</option>`).join('');
}
async function loadParams() {
  const r = await api('/api/params');
  params = r.params || {};
  paramsLoaded = Object.keys(params).length > 0;
  if (params.COM_RC_IN_MODE) $('#joy-priority').value = String(params.COM_RC_IN_MODE.value);
  loadExport();          // vehicle values are now known: refresh the ✓/≠ marks
  $('#param-count').textContent = `${Object.keys(params).length}/${r.count} loaded · ${Object.keys(meta).length} described`;
  renderParams();
}
function renderParams() {
  const q = $('#param-search').value.trim().toLowerCase();
  const grp = $('#param-group').value;
  const names = Object.keys(params).sort();
  let shown = 0;
  const html = [];
  if (!q && !grp) { $('#param-list').innerHTML = `<div class="hint">${names.length} parameters loaded · type to search, or pick a group.</div>`; return; }
  for (const n of names) {
    const m = meta[n] || {};
    if (grp && m.group !== grp) continue;
    if (q && !(n.toLowerCase().includes(q) || (m.short || '').toLowerCase().includes(q))) continue;
    if (++shown > 40) break;
    const v = params[n].value;
    const edited = airframe && airframe.px4_overrides && (n in airframe.px4_overrides);
    html.push(`<div class="prow ${n === selectedParam ? 'selected' : ''} ${edited ? 'edited' : ''}" data-n="${n}"><span class="pname">${edited ? '● ' : ''}${n}</span><span class="pval">${typeof v === 'number' && !Number.isInteger(v) ? +v.toPrecision(6) : v}${m.unit ? ' <span class="muted">' + m.unit + '</span>' : ''}</span><span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span></div>`);
  }
  $('#param-list').innerHTML = html.join('') + (shown > 40 ? '<div class="hint">… more matches, refine the search</div>' : '');
  $$('#param-list .prow').forEach(el => el.addEventListener('click', () => openParam(el.dataset.n)));
}
let selectedParam = null;
function openParam(n) {
  selectedParam = n;
  const p = params[n], m = meta[n] || {};
  const ed = $('#param-editor');
  ed.classList.remove('hidden');
  let input;
  if (m.values && m.values.length) {
    input = `<select id="pe-value">${m.values.map(o => `<option value="${o.value}" ${+o.value === +p.value ? 'selected' : ''}>${o.value}: ${o.description}</option>`).join('')}</select>`;
  } else if (m.bitmask && m.bitmask.length) {
    input = `<div id="pe-bits">${m.bitmask.map(b => `<label class="row"><input type="checkbox" data-bit="${b.index}" ${(p.value >> b.index) & 1 ? 'checked' : ''}> ${b.index}: ${b.description}</label>`).join('')}</div>`;
  } else {
    input = `<input id="pe-value" type="number" step="${m.increment || (p.type === 6 ? 1 : 'any')}" value="${p.value}" ${m.min != null ? 'min="' + m.min + '"' : ''} ${m.max != null ? 'max="' + m.max + '"' : ''}>`;
  }
  ed.innerHTML = `<div><span class="pn">${n}</span> <span class="muted">${m.group || ''}</span></div>
    <div>${m.short || ''}</div>
    <div class="long">${m.long || ''}</div>
    <div class="meta">${p.type === 6 ? 'INT32' : 'FLOAT'}${m.unit ? ' · ' + m.unit : ''}${m.min != null ? ' · min ' + m.min : ''}${m.max != null ? ' · max ' + m.max : ''}${m.default != null ? ' · default ' + m.default : ''}${m.reboot ? ' · <span class="warn">reboot required</span>' : ''}</div>
    <div class="row">${input}<button id="pe-set" class="primary small">set</button><button id="pe-close" class="small">close</button><span id="pe-result"></span></div>`;
  $('#pe-close').addEventListener('click', () => { ed.classList.add('hidden'); selectedParam = null; renderParams(); });
  $('#pe-set').addEventListener('click', async () => {
    let value;
    if ($('#pe-bits')) value = $$('#pe-bits input').reduce((acc, cb) => acc | (cb.checked ? (1 << +cb.dataset.bit) : 0), 0);
    else value = parseFloat($('#pe-value').value);
    $('#pe-result').textContent = '…';
    try {
      const r = await setParamValue(n, value);
      $('#pe-result').innerHTML = r.ok ? `<span class="ok">✓ ${r.value} · saved with the airframe</span>` : `<span class="err">${r.error}</span>`;
    } catch (e) { $('#pe-result').innerHTML = `<span class="err">${e.message}</span>`; }
  });
  renderParams();
}
$('#param-search').addEventListener('input', renderParams);
$('#param-group').addEventListener('change', renderParams);
$('#param-refresh').addEventListener('click', async () => { $('#param-count').textContent = 'loading…'; await api('/api/params/refresh', {}); await loadParams(); });
$('#param-save').addEventListener('click', () => api('/api/params/save', {}));
$('#param-reboot').addEventListener('click', () => { if (confirm('Reboot the flight controller?')) api('/api/px4/command', { command: 'reboot' }); });
$('#param-meta-fetch').addEventListener('click', async () => {
  $('#param-count').textContent = 'downloading descriptions via MAVLink FTP…';
  try { const r = await api('/api/params/meta/fetch', {}); await loadMeta(); logLine(`[ui] ${r.count} parameter descriptions from ${r.source}`); }
  catch (e) { logLine('[ui] ' + e.message); }
  await loadParams();
});

// ============================================================ flight / sim
$$('#tab-sim button[data-cmd]').forEach(b => b.addEventListener('click', () =>
  api('/api/px4/command', { command: b.dataset.cmd, mode: b.dataset.mode, force: b.dataset.cmd === 'kill' }).catch(e => logLine('[ui] ' + e.message))));
async function recover(btn, full) {
  const label = btn.textContent; btn.textContent = 'Resetting…'; btn.disabled = true;
  try { const r = await api('/api/px4/reset_all', {}); logLine('[ui] reset: ' + (r.steps || []).join(', ')); }
  catch (e) { logLine('[ui] reset failed: ' + e.message); }
  btn.textContent = label; btn.disabled = false;
}
$('#btn-reset').addEventListener('click', (e) => recover(e.target, true));     // everything: vehicle + PX4 reboot
$('#btn-pause').addEventListener('click', async () => { const r = await api('/api/sim/pause', {}); $('#btn-pause').textContent = r.paused ? 'Resume' : 'Pause'; });
$('#btn-follow').addEventListener('click', (e) => { e.target.classList.toggle('on'); scene.setFollow(e.target.classList.contains('on')); });
$('#sim-speed').addEventListener('change', e => api('/api/sim/speed', { speed: parseFloat(e.target.value) }));
$('#sim-noise').addEventListener('change', e => api('/api/sim/noise', { enabled: e.target.checked }));
$('#wind-apply').addEventListener('click', () => api('/api/sim/wind', { north: +$('#wind-n').value, east: +$('#wind-e').value, down: +$('#wind-d').value }));
$('#home-apply').addEventListener('click', () => api('/api/sim/home', { lat: +$('#home-lat').value, lon: +$('#home-lon').value, alt: +$('#home-alt').value }));

function renderMotorSliders() {
  const el = $('#motor-sliders');
  el.innerHTML = airframe.rotors.map((r, i) => `<label>M${i + 1} <input type="range" min="0" max="1" step="0.01" value="0" data-i="${i}"><span class="mv">0.00</span></label>`).join('');
  el.querySelectorAll('input').forEach(inp => inp.addEventListener('input', sendOverride));
}
function sendOverride() {
  const vals = $$('#motor-sliders input').map(i => parseFloat(i.value));
  $$('#motor-sliders .mv').forEach((s, i) => s.textContent = vals[i].toFixed(2));
  if ($('#motor-enable').checked) api('/api/sim/motor_override', { values: vals });
}
$('#motor-enable').addEventListener('change', (e) => {
  if (e.target.checked) sendOverride(); else api('/api/sim/motor_override', { values: null });
});

// ============================================================ websocket + status
let homeFilled = false;
function applyStatus(s) {
  status = s;
  const none = !s.conn_mode;
  $('#st-mode').textContent = none ? 'No link' : (s.mode === 'hitl' ? 'Pixhawk' : 'SITL');
  $('#st-conn').textContent = none ? (s.conn_error ? 'failed — open Connect' : 'open Connect')
    : s.connected ? (s.mode === 'hitl' ? s.address.replace('/dev/', '') + (s.hil_enabled ? '' : ' · HITL off') : 'PX4 connected')
    : (s.mode === 'sitl' ? (s.px4_running ? 'PX4 starting…' : 'waiting for PX4') : 'no data from ' + s.address.replace('/dev/', ''));
  $('#st-link .dot').classList.toggle('on', s.connected);
  const joyCard = $('#joy-card');
  if (joyCard) { const sitl = s.conn_mode === 'sitl'; joyCard.classList.toggle('hidden', !sitl); if (!sitl) $('#joy-enable').checked = false; }
  const det = $('#st-detected');
  const showDet = !s.flashing && s.mode !== 'hitl' && s.px4_ports && s.px4_ports.length > 0;
  if (s.flashing) { $('#st-mode').textContent = 'Pixhawk'; $('#st-conn').textContent = 'flashing firmware…'; }
  det.classList.toggle('hidden', !showDet); det.classList.toggle('blink', showDet);
  if (showDet) det.textContent = `Pixhawk on ${s.px4_ports[0].replace('/dev/', '')} · connect`;
  $('#st-armed').textContent = s.resetting > 0 ? `Resetting… ${Math.ceil(s.resetting)} s` : (s.armed ? 'Armed' : 'Disarmed');
  $('#st-armed').classList.toggle('armed', s.armed);
  $('#st-flightmode').textContent = s.connected ? s.mode_name + (s.mode === 'hitl' && !s.hil_enabled ? ' · HIL OFF (set SYS_HITL=1)' : '') : '—';
  if (!homeFilled) { $('#home-lat').value = s.home.lat; $('#home-lon').value = s.home.lon; $('#home-alt').value = s.home.alt; homeFilled = true; }
  if (s.params_loaded && !paramsLoaded && $('#tab-airframe').classList.contains('active')) ensureParams();
  updateFooter();
}
function applyState(st) {
  scene.updateState(st);
  $('#st-time').textContent = `${st.t.toFixed(1)} s`;
  $('#st-rtf').textContent = `RTF ${st.rtf ? st.rtf.toFixed(2) : '—'}${st.lockstep_timeouts ? ' · ' + st.lockstep_timeouts + ' waits' : ''}`;
  const [r, p, y] = st.euler.map(deg);
  $('#st-pose').textContent = ` · N ${st.pos[0].toFixed(1)} E ${st.pos[1].toFixed(1)} alt ${(-st.pos[2]).toFixed(2)} m · R ${r.toFixed(0)}° P ${p.toFixed(0)}° Y ${y.toFixed(0)}°${st.on_ground ? ' · on ground' : ''}`;
  if ($('#tab-sim').classList.contains('active')) {
    $('#rotor-live').innerHTML = st.rotors.map((x, i) => `<div><b>M${i + 1}</b> cmd ${x.cmd.toFixed(2)} · ω ${x.omega.toFixed(2)} <span class="bar" style="width:${Math.round(x.thrust / (airframe.rotors[i]?.max_thrust || 1) * 120)}px"></span> ${x.thrust.toFixed(2)} N</div>`).join('');
  }
}
const logPre = $('#log-pre');
function logLine(s) { logPre.textContent += s + '\n'; if (logPre.textContent.length > 40000) logPre.textContent = logPre.textContent.slice(-30000); const b = $('#log-body'); b.scrollTop = b.scrollHeight; }
$('#log-toggle').addEventListener('click', (e) => { const c = $('#log').classList.toggle('collapsed'); e.target.textContent = c ? 'Show' : 'Hide'; });
$('#btn-theme').addEventListener('click', () => {
  const dark = document.documentElement.dataset.theme !== 'dark';
  document.documentElement.dataset.theme = dark ? 'dark' : 'light';
  try { localStorage.setItem('airframe-theme', dark ? 'dark' : 'light'); } catch { }
  scene.setTheme(dark ? 'dark' : 'light');
});

function connectWs() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  joyWs = ws;
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'airframe') { setAirframe(m.airframe); }
    if (m.type === 'state') { applyState(m.state); if (m.status) applyStatus(m.status); if (m.log) m.log.forEach(l => logLine(l[1])); }
  };
  ws.onclose = () => { logLine('[ui] connection to simulator lost, retrying…'); setTimeout(connectWs, 1500); };
}
connectWs();
loadPresetList();
api('/api/log?since=0').then(lines => lines.forEach(l => logLine(l[1]))).catch(() => { });


// ============================================================ resizable panels
(function () {
  const side = $('#side'), log = $('#log');
  let layout = {};
  try { layout = JSON.parse(localStorage.getItem('airframe-layout') || '{}'); } catch { }
  if (layout.sideW) side.style.width = layout.sideW + 'px';
  if (layout.logH) log.style.height = layout.logH + 'px';
  const save = () => { try { localStorage.setItem('airframe-layout', JSON.stringify(layout)); } catch { } };

  function drag(handle, axis, onMove) {
    handle.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      handle.setPointerCapture(e.pointerId);
      handle.classList.add('dragging');
      document.body.classList.add('resizing', axis === 'x' ? 'resizing-x' : 'resizing-y');
      const start = axis === 'x' ? e.clientX : e.clientY;
      const move = (ev) => onMove((axis === 'x' ? ev.clientX : ev.clientY) - start);
      const up = () => {
        handle.classList.remove('dragging');
        document.body.classList.remove('resizing', 'resizing-x', 'resizing-y');
        handle.removeEventListener('pointermove', move); handle.removeEventListener('pointerup', up);
        save();
      };
      handle.addEventListener('pointermove', move); handle.addEventListener('pointerup', up);
      onMove.begin && onMove.begin();
    });
  }

  const sideMove = (dx) => {
    const w = Math.min(window.innerWidth * 0.7, Math.max(380, sideMove.startW - dx));
    side.style.width = w + 'px'; layout.sideW = Math.round(w);
  };
  sideMove.begin = () => { sideMove.startW = side.getBoundingClientRect().width; };
  drag($('#resize-side'), 'x', sideMove);

  const logMove = (dy) => {
    if (log.classList.contains('collapsed')) { log.classList.remove('collapsed'); $('#log-toggle').textContent = 'Hide'; }
    const h = Math.min(window.innerHeight * 0.7, Math.max(48, logMove.startH - dy));
    log.style.height = h + 'px'; layout.logH = Math.round(h);
  };
  logMove.begin = () => { logMove.startH = log.getBoundingClientRect().height; };
  drag($('#resize-log'), 'y', logMove);
})();


// ============================================================ connection / HITL
let connTimer = null;
$('#st-link').addEventListener('click', () => openTab('connect'));
$('#st-detected').addEventListener('click', async () => { openTab('connect'); await connectHitl(); });
$('#conn-sitl').addEventListener('click', async () => { await connCall('/api/connection/connect', { mode: 'sitl' }); });
$('#conn-rescan').addEventListener('click', refreshConnection);
$('#conn-disconnect').addEventListener('click', async () => { await connCall('/api/connection/disconnect', {}); });
$('#conn-reboot').addEventListener('click', async () => { if (confirm('Reboot the flight controller? The link reconnects by itself.')) { await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); } });
// ---- PX4 messages (decoded events) in the Flight tab
let eventsTimer = null;
async function refreshEvents() {
  clearTimeout(eventsTimer);
  if (!$('#tab-sim').classList.contains('active')) return;
  try {
    const r = await api('/api/events');
    const ev = r.events || [];
    let summary = null;
    for (let i = ev.length - 1; i >= 0; i--) if (ev[i].name === 'commander_arming_check_summary') { summary = ev[i]; break; }
    const el = $('#px4-messages');
    let html = '';
    if (summary) {
      const d = {}; (summary.arg_names || []).forEach((k, i) => d[k] = summary.args[i]);
      const canArm = String(d.can_arm || '').split('|').filter(x => x && !/^\d+$/.test(x));
      const errs = String(d.error || '').split('|').filter(x => x && x !== '0' && x.toLowerCase() !== 'system');   // "system" = offboard/mission checks
      html += `<div class="conn-row"><div><b>${canArm.length ? 'Can arm in' : 'Cannot arm'}</b> <span class="hint">${canArm.length ? canArm.join(', ') : 'see messages below'}</span>` +
        `${errs.length ? `<div class="err">blocking: ${errs.join(', ').replace(/_/g, ' ')}</div>` : ''}</div></div>`;
    }
    // only checks that matter for the current mode; offboard/mission checks always fail elsewhere
    const modeNow = (status.mode_name || '').toLowerCase();
    const seen = new Set();
    const shown = ev.filter(x => x.level <= 6 && x.group !== 'protocol' && !x.name.includes('summary'))
      .filter(x => !(/offboard/i.test(x.text) && modeNow !== 'offboard') && !(/mission/i.test(x.text) && modeNow !== 'mission'))
      .reverse().filter(x => { if (seen.has(x.text)) return false; seen.add(x.text); return true; }).slice(0, 8);
    html += shown.map(x => `<div class="msg ${x.level <= 3 ? 'err' : x.level === 4 ? 'warn' : ''}"><span class="lvl">${x.level_name}</span> ${x.text}</div>`).join('') || '<div class="hint">no messages from PX4 yet</div>';
    el.innerHTML = html;
  } catch (e) { }
  eventsTimer = setTimeout(refreshEvents, 2000);
}
let rcTimer = null;
async function refreshRc() {
  clearTimeout(rcTimer);
  if (!$('#tab-sim').classList.contains('active')) return;
  try {
    const rc = await api('/api/rc');
    const el = $('#rc-card');
    if (!rc.channels || !rc.channels.length) {
      el.innerHTML = `<div class="hint">no RC data from the flight controller${rc.rc_in_mode != null ? ' · COM_RC_IN_MODE = ' + rc.rc_in_mode : ''}. Plug a receiver into the board, or use a joystick through QGroundControl.</div>`;
    } else {
      const rows = rc.channels.map((v, i) => {
        const pct = Math.max(0, Math.min(1, (v - 1000) / 1000));
        const names = (rc.mapping[String(i + 1)] || []).join(', ');
        return `<div class="rc-row"><span class="rc-n">${i + 1}</span><span class="rc-name">${names}</span><span class="rc-bar"><i style="width:${(pct * 100).toFixed(0)}%"></i></span><span class="rc-val num">${v}</span></div>`;
      }).join('');
      el.innerHTML = `<div class="rc-head"><b>${rc.count} channels</b><span class="hint">RSSI ${rc.rssi === 255 ? '—' : rc.rssi}${rc.rc_in_mode != null ? ' · COM_RC_IN_MODE ' + rc.rc_in_mode : ''}</span></div>${rows}`;
    }
  } catch (e) { }
  rcTimer = setTimeout(refreshRc, 200);
}
$$('.tabs button').forEach(b => b.addEventListener('click', () => { if (b.dataset.tab === 'sim') refreshRc(); }));
$$('.tabs button').forEach(b => b.addEventListener('click', () => { if (b.dataset.tab === 'sim') refreshEvents(); }));
async function connectHitl(serial) {
  await connCall('/api/connection/connect', { mode: 'hitl', serial, baud: +$('#conn-baud').value || 921600 });
}
async function connCall(path, body) {
  $('#conn-error').textContent = '';
  $('#conn-current').innerHTML = '<span class="muted">Working…</span>';
  try { await api(path, body); paramsLoaded = false; } catch (e) { $('#conn-error').textContent = e.message; }
  await refreshConnection();
}
async function refreshConnection() {
  clearTimeout(connTimer);
  if (!$('#tab-connect').classList.contains('active')) return;
  let c;
  try { c = await api('/api/connection'); } catch (e) { $('#conn-error').textContent = e.message; return; }
  const L = c.link;
  const cur = $('#conn-current');
  if (!c.mode) cur.innerHTML = '<div class="conn-row"><div><b>Not connected</b><div class="hint">Pick PX4 SITL or a Pixhawk below.</div></div></div>';
  else if (c.mode === 'sitl') cur.innerHTML = `<div class="conn-row"><div><b>PX4 SITL</b> <span class="${L.connected ? 'ok' : 'muted'}">${L.connected ? '● connected' : (c.px4_running ? '○ starting…' : '○ waiting for PX4')}</span><div class="hint">instance ${c.px4_instance ?? 0} · simulator tcp ${L.address} · control ${L.ctl_address}</div></div></div>`;
  else cur.innerHTML = `<div class="conn-row"><div><b>Pixhawk</b> <span class="${L.connected ? 'ok' : 'muted'}">${L.connected ? '● link up' : '○ no data yet'}</span><div class="dev">${c.serial} @ ${c.baud}</div><div class="hint">QGroundControl: connect over UDP ${c.qgc}</div></div></div>`;
  $('#conn-error').textContent = c.error || '';
  $('#conn-sitl-card').classList.toggle('current', c.mode === 'sitl');
  const ports = c.ports || [];
  $('#conn-ports').innerHTML = ports.length ? ports.map(p => `
    <div class="card ${c.mode === 'hitl' && c.serial === p.device ? 'current' : ''}"><div class="conn-row">
      <div><b>${p.likely_px4 ? 'Pixhawk' : 'Serial device'}</b> <span class="hint">${p.description || ''}</span><div class="dev">${p.device}</div></div>
      <button class="pill small ${p.likely_px4 ? 'primary' : ''}" data-dev="${p.device}">${c.mode === 'hitl' && c.serial === p.device ? 'Reconnect' : 'Connect'}</button>
    </div></div>`).join('')
    : '<div class="card"><div class="conn-row"><div><b>No USB flight controller found</b><div class="hint">Plug the Pixhawk in over USB and click Rescan. If QGroundControl is open, close it or disable its serial auto-connect.</div></div></div></div>';
  $$('#conn-ports button[data-dev]').forEach(b => b.addEventListener('click', () => connectHitl(b.dataset.dev)));
  const steps = c.checklist || [];
  $('#conn-checklist').innerHTML = steps.map(s => `<div class="check ${s.ok ? 'ok' : ''}"><div class="mark">${s.ok ? '✓' : ''}</div>
    <div class="body"><div class="label">${s.label}</div>${s.detail ? `<div class="detail">${s.detail}</div>` : ''}</div>
    ${s.busy ? '<span class="pill small">Working…</span>' : ''}
    ${s.action === 'enable_hitl' ? '<button class="pill small primary" data-act="enable_hitl">Enable HITL</button>' : ''}
    ${s.action === 'build_firmware' ? '<button class="pill small primary" data-act="build_firmware">Build firmware</button>' : ''}
    ${s.action === 'upload_firmware' ? '<button class="pill small primary" data-act="upload_firmware">Flash firmware</button>' : ''}
    ${s.action === 'push' ? '<button class="pill small primary" data-act="push">Push geometry</button>' : ''}
    ${s.action === 'reboot' ? '<button class="pill small" data-act="reboot">Reboot board</button>' : ''}
    ${s.action === 'ekf' ? '<button class="pill small" data-act="ekf">Restart estimator</button>' : ''}</div>`).join('');
  $$('#conn-checklist button[data-act]').forEach(b => b.addEventListener('click', async () => {
    if (b.dataset.act === 'enable_hitl') { b.textContent = 'Rebooting…'; await connCall('/api/connection/enable_hitl', {}); }
    if (b.dataset.act === 'push') { if (status.armed) { alert('Disarm before updating PX4'); return; } await pushToPX4($('#update-status'), true); await refreshConnection(); }
    if (b.dataset.act === 'build_firmware') { b.textContent = 'Building…'; await connCall('/api/firmware/build', {}); }
    if (b.dataset.act === 'ekf') { b.textContent = 'Restarting…'; await api('/api/connection/restart_estimator', {}); setTimeout(refreshConnection, 3000); }
    if (b.dataset.act === 'reboot') { b.textContent = 'Rebooting…'; await api('/api/px4/command', { command: 'reboot' }); setTimeout(refreshConnection, 3000); }
    if (b.dataset.act === 'upload_firmware') {
      if (!confirm('Flash the HITL-capable firmware to the board now? It reboots and reconnects when done. Parameters are kept.')) return;
      b.textContent = 'Flashing…'; await connCall('/api/firmware/upload', {});
    }
  }));
  connTimer = setTimeout(refreshConnection, 2000);
}


// ============================================================ USB remote (WebHID picker, Gamepad API fallback) -> MANUAL_CONTROL
const JOY_FUNCS = [
  { key: 'roll', label: 'Roll', axis: 0, invert: false },
  { key: 'pitch', label: 'Pitch', axis: 1, invert: true },      // stick forward is negative on most devices
  { key: 'throttle', label: 'Throttle', axis: 2, invert: false },
  { key: 'yaw', label: 'Yaw', axis: 3, invert: false },
];
let joyMap = JOY_FUNCS.map(f => ({ ...f }));
try { const saved = JSON.parse(localStorage.getItem('airframe-joystick') || 'null'); if (saved && saved.length === 4) joyMap = saved; } catch { }
let joyLearn = null, joyLearnBase = null;
// current input source: { name, axes: [-1..1], buttons: [bool], kind: 'hid' | 'gamepad' }
let joySrc = null;
let hidDevice = null;
function joySave() { try { localStorage.setItem('airframe-joystick', JSON.stringify(joyMap)); } catch { } }

// --- WebHID: parse the device's own report descriptor so any joystick layout works (8-bit, 11-bit, 16-bit axes…)
const HID_AXIS_USAGES = { 0x30: 'X', 0x31: 'Y', 0x32: 'Z', 0x33: 'Rx', 0x34: 'Ry', 0x35: 'Rz', 0x36: 'Slider', 0x37: 'Dial', 0x38: 'Wheel' };
function hidBuildLayout(device) {
  const reports = {};   // reportId -> { axes: [{bit,size,min,max,name}], buttons: [{bit}] }
  for (const col of device.collections) {
    for (const rep of col.inputReports) {
      const lay = reports[rep.reportId] || (reports[rep.reportId] = { axes: [], buttons: [] });
      let bit = 0;
      for (const it of rep.items) {
        for (let k = 0; k < it.reportCount; k++) {
          const usage = it.isRange ? (it.usageMinimum + k) : (it.usages[k] ?? it.usages[0] ?? 0);
          const page = usage >>> 16, id = usage & 0xffff;
          if (page === 0x01 && HID_AXIS_USAGES[id] && it.reportSize > 1) {
            lay.axes.push({ bit, size: it.reportSize, min: it.logicalMinimum, max: it.logicalMaximum, name: HID_AXIS_USAGES[id] });
          } else if (page === 0x09 && it.reportSize === 1) {
            lay.buttons.push({ bit });
          }
          bit += it.reportSize;
        }
      }
    }
  }
  return reports;
}
function hidBits(dv, bit, size, signed) {
  let v = 0;
  for (let i = 0; i < size; i++) { const b = bit + i; if ((dv.getUint8(b >> 3) >> (b & 7)) & 1) v |= (1 << i); }
  if (signed && (v & (1 << (size - 1)))) v -= (1 << size);
  return v;
}
async function hidUse(device) {
  try {
    if (!device.opened) await device.open();
  } catch (e) { logLine('[ui] could not open the radio: ' + e.message); return; }
  const layout = hidBuildLayout(device);
  hidDevice = device;
  joySrc = { name: device.productName || 'HID joystick', axes: [], buttons: [], kind: 'hid' };
  device.addEventListener('inputreport', (e) => {
    const lay = layout[e.reportId] || layout[0]; if (!lay) return;
    const dv = e.data;
    try {
      joySrc.axes = lay.axes.map(a => { const signed = a.min < 0; const v = hidBits(dv, a.bit, a.size, signed); return Math.max(-1, Math.min(1, ((v - a.min) / (a.max - a.min)) * 2 - 1)); });
      joySrc.buttons = lay.buttons.map(b => !!hidBits(dv, b.bit, 1, false));
    } catch { }
  });
  const n = Object.values(layout).reduce((a, l) => a + l.axes.length, 0);
  logLine(`[ui] radio connected: ${joySrc.name} (${n} axes)`);
  joyRenderMap();
}
async function joyConnect() {
  if (!navigator.hid) {
    logLine('[ui] this browser has no device picker (WebHID). Open the app in Chrome or Edge, or move a stick so the gamepad fallback finds the radio.');
    $('#joy-hint').textContent = 'This browser cannot show a device picker (Safari has no WebHID). Open the page in Chrome/Edge, or move a stick: the gamepad fallback may still find it.';
    return;
  }
  try {
    const devices = await navigator.hid.requestDevice({ filters: [] });
    if (devices.length) await hidUse(devices[0]);
  } catch (e) { logLine('[ui] radio picker: ' + e.message); }
}
async function hidReconnect() {   // devices the user already granted come back without the picker
  if (!navigator.hid) return;
  try {
    const devs = await navigator.hid.getDevices();
    const joy = devs.find(d => d.collections.some(c => c.usagePage === 1 && (c.usage === 4 || c.usage === 5))) || devs[0];
    if (joy) await hidUse(joy);
  } catch { }
}
function joyGamepad() {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  const p = pads[0];
  if (!p) return null;
  return { name: p.id.replace(/\s*\(.*$/, '').slice(0, 48), axes: Array.from(p.axes), buttons: p.buttons.map(b => b.pressed), kind: 'gamepad' };
}
function joyCurrent() {
  if (joySrc && joySrc.kind === 'hid' && hidDevice && hidDevice.opened) return joySrc;
  return joyGamepad();
}
function joyRenderMap() {
  const src = joyCurrent();
  const n = src && src.axes.length ? src.axes.length : 8;
  $('#joy-map').innerHTML = joyMap.map((f, i) => `<div class="joy-row"><span class="joy-label">${f.label}</span>
    <select data-i="${i}" class="joy-axis">${Array.from({ length: n }, (_, a) => `<option value="${a}" ${a === f.axis ? 'selected' : ''}>axis ${a + 1}</option>`).join('')}</select>
    <label class="row" style="margin:0"><input type="checkbox" data-i="${i}" class="joy-inv" ${f.invert ? 'checked' : ''}> invert</label>
    <button class="pill small joy-learn" data-i="${i}">${joyLearn === i ? 'move it…' : 'Learn'}</button>
    <span class="rc-bar joy-bar"><i data-i="${i}" style="width:50%"></i></span><span class="rc-val num joy-val" data-i="${i}">—</span></div>`).join('');
  $$('.joy-axis').forEach(sel => sel.addEventListener('change', () => { joyMap[+sel.dataset.i].axis = +sel.value; joySave(); }));
  $$('.joy-inv').forEach(cb => cb.addEventListener('change', () => { joyMap[+cb.dataset.i].invert = cb.checked; joySave(); }));
  $$('.joy-learn').forEach(b => b.addEventListener('click', () => { joyLearn = +b.dataset.i; const s = joyCurrent(); joyLearnBase = s ? s.axes.slice() : null; joyRenderMap(); }));
}
function joyValue(src, f) {
  if (!src) return 0;
  let v = src.axes[f.axis] ?? 0;
  if (Math.abs(v) < 0.02) v = 0;                  // deadband
  return f.invert ? -v : v;
}
let joyLastSend = 0;
function joyTick() {
  const src = joyCurrent();
  const nameEl = $('#joy-name');
  if (!src) {
    nameEl.textContent = 'No radio connected';
    if (status.radio_vcp_ports && status.radio_vcp_ports.length) {
      $('#joy-hint').innerHTML = `<span class="warn">The radio is in config (VCP) mode</span> — it shows up as a serial port (${status.radio_vcp_ports[0].replace('/dev/', '')}), not as a joystick. Power it off, then power it on with the Power button only (no M button), reconnect USB, and click Connect radio.`;
    }
  }
  else {
    nameEl.textContent = src.name + (src.kind === 'hid' ? '' : ' (gamepad)');
    $('#joy-hint').textContent = `${src.axes.length} axes, ${src.buttons.length} buttons`;
    if (joyLearn != null && joyLearnBase) {
      let best = -1, bestD = 0.3;
      src.axes.forEach((v, a) => { const d = Math.abs(v - (joyLearnBase[a] ?? 0)); if (d > bestD) { bestD = d; best = a; } });
      if (best >= 0) { joyMap[joyLearn].axis = best; joyLearn = null; joyLearnBase = null; joySave(); joyRenderMap(); }
    }
    joyMap.forEach((f, i) => {
      const v = joyValue(src, f); const bar = document.querySelector(`.joy-bar i[data-i="${i}"]`), val = document.querySelector(`.joy-val[data-i="${i}"]`);
      if (bar) bar.style.width = ((v + 1) / 2 * 100).toFixed(0) + '%';
      if (val) val.textContent = v.toFixed(2);
    });
    const now = performance.now();
    if ($('#joy-enable').checked && status.conn_mode === 'sitl' && joyWs && joyWs.readyState === 1 && now - joyLastSend > 20) {   // 50 Hz, SITL only
      joyLastSend = now;
      const g = (k) => joyValue(src, joyMap.find(f => f.key === k));
      const aux = src.axes.slice(4, 10).map(v => +v.toFixed(3));
      let buttons = 0; src.buttons.forEach((b, i) => { if (b && i < 16) buttons |= (1 << i); });
      joyWs.send(JSON.stringify({ type: 'manual', roll: g('roll'), pitch: g('pitch'), throttle: (g('throttle') + 1) / 2, yaw: g('yaw'), buttons, aux }));
    }
  }
  if ($('#tab-sim').classList.contains('active')) requestAnimationFrame(joyTick); else setTimeout(joyTick, 500);
}
window.addEventListener('gamepadconnected', () => { joyRenderMap(); const s = joyGamepad(); logLine('[ui] gamepad found: ' + (s ? s.name : '')); });
if (navigator.hid) navigator.hid.addEventListener('disconnect', (e) => { if (e.device === hidDevice) { hidDevice = null; joySrc = null; logLine('[ui] radio disconnected'); joyRenderMap(); } });
$('#joy-connect').addEventListener('click', joyConnect);
$('#joy-enable').addEventListener('change', (e) => logLine('[ui] radio ' + (e.target.checked ? 'sending to PX4 (MANUAL_CONTROL at 50 Hz)' : 'stopped')));
$('#joy-priority').addEventListener('change', (e) => setParamValue('COM_RC_IN_MODE', +e.target.value));
joyRenderMap(); joyTick(); hidReconnect();
