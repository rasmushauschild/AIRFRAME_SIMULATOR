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
  scene = { setAirframe: noop, updateState: noop, select: noop, updateRotorNode: noop, setTheme: noop, setFollow: noop, setCameraMode: noop, setMode: noop, selected: -1, focusOrigin: noop };
  $('#viewport').insertAdjacentHTML('afterbegin', '<div class="hint" style="padding:18px">3D view unavailable in this window (no WebGL). Everything else works.</div>');
}

// ============================================================ tabs
$$('.tabs button').forEach(b => b.addEventListener('click', () => {
  $$('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  $$('.tab').forEach(t => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  if (b.dataset.tab === 'airframe') { ensureParams(); loadExport(); }
  if (b.dataset.tab === 'connect') refreshConnection();
  if (b.dataset.tab === 'design') refreshDesign();
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
      if ($('#tab-design').classList.contains('active')) refreshDesign();
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
  airframe.wings = airframe.wings || [];
  airframe.design = airframe.design || {};
  designGroups = null;
  fillWingCard();
  $('#d-speed').value = airframe.design.cruise_speed_kmh ?? 50;
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
  $('#m-turnloss').value = Math.round((r.turn_loss ?? 0.1) * 100);
}
function applyMotorCard(kindChanged) {
  const kind = $('#m-kind').value;
  if (kindChanged) { const d = KIND_DEFAULTS[kind]; $('#m-tau').value = d.tau; $('#m-km').value = d.km; $('#m-dia').value = d.prop_diameter; $('#m-exp').value = d.thrust_exponent; $('#m-ram').checked = d.ram_drag; }
  const tmax = +$('#m-tmax').value, tau = +$('#m-tau').value, km = Math.abs(+$('#m-km').value), dia = +$('#m-dia').value, ex = +$('#m-exp').value, ram = $('#m-ram').checked;
  const turnLoss = Math.max(0, +$('#m-turnloss').value) / 100;
  airframe.rotors.forEach(r => { r.kind = kind; r.max_thrust = tmax; r.tau = tau; r.km = (r.km >= 0 ? 1 : -1) * km; r.prop_diameter = dia; r.thrust_exponent = ex; r.ram_drag = ram; r.turn_loss = turnLoss; if (kind !== 'ducted') r.duct_axis = null; });
  setAirframe(airframe); pushAirframe(true);
}
$('#m-kind').addEventListener('change', () => applyMotorCard(true));
['m-tmax', 'm-tau', 'm-km', 'm-dia', 'm-exp', 'm-ram', 'm-turnloss'].forEach(id => $('#' + id).addEventListener('change', () => applyMotorCard(false)));

// ---- wing
const WING_DEFAULT = { pos: [-0.2, 0, 0.06], area: 0.5, span: 1.074, incidence_deg: 10, cd0: 0.02, stall_deg: 30, vortex_lift: true, enabled: true };
function fillWingCard() {
  const w = (airframe.wings && airframe.wings[0]) || { ...WING_DEFAULT, enabled: false };
  $('#w-on').checked = !!w.enabled; $('#w-area').value = w.area; $('#w-span').value = w.span; $('#w-inc').value = w.incidence_deg;
  $('#w-cd0').value = w.cd0; $('#w-stall').value = w.stall_deg; $('#w-vortex').checked = !!w.vortex_lift;
  $('#w-x').value = w.pos[0]; $('#w-y').value = w.pos[1]; $('#w-z').value = w.pos[2];
}
function applyWingCard() {
  if (!airframe.wings.length) airframe.wings.push({ ...WING_DEFAULT });
  const w = airframe.wings[0];
  w.enabled = $('#w-on').checked; w.area = +$('#w-area').value; w.span = +$('#w-span').value; w.incidence_deg = +$('#w-inc').value;
  w.cd0 = +$('#w-cd0').value; w.stall_deg = +$('#w-stall').value; w.vortex_lift = $('#w-vortex').checked;
  w.pos = [+$('#w-x').value, +$('#w-y').value, +$('#w-z').value];
  scene.setAirframe(airframe); pushAirframe(true);
}
['w-on', 'w-area', 'w-span', 'w-inc', 'w-cd0', 'w-stall', 'w-vortex', 'w-x', 'w-y', 'w-z'].forEach(id => $('#' + id).addEventListener('change', applyWingCard));

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
// Tilt: forward lean of the thrust axis from vertical (negative = backward). Cant: sideways lean, positive = outward
// (away from the centreline; for a rotor on the centreline positive = right). Same convention as the Optimize tab.
const outward = (y) => (y < 0 ? -1 : 1);
const axisToTilt = (a, y = 1) => {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  const tilt = deg(Math.asin(Math.max(-1, Math.min(1, a[0] / n))));
  const cantRight = (Math.hypot(a[1], a[2]) < 1e-6) ? 0 : deg(Math.atan2(a[1], -a[2]));
  return [tilt, cantRight * outward(y)];
};
const tiltToAxis = (tilt, cant, y = 1) => {
  const t = tilt * Math.PI / 180, c = cant * outward(y) * Math.PI / 180;
  return [+Math.sin(t).toFixed(4), +(Math.sin(c) * Math.cos(t)).toFixed(4), +(-Math.cos(c) * Math.cos(t)).toFixed(4)];
};

function renderRotorTable() {
  const el = $('#rotor-table');
  const rows = airframe.rotors.map((r, i) => rotorRowHtml(i, r)).join('');
  el.innerHTML = `<table class="grid"><thead><tr><th>#</th><th title="position, m">X</th><th>Y</th><th>Z</th><th title="forward lean of the thrust axis from vertical, degrees; negative = backward">Tilt°</th><th title="sideways lean, degrees; positive = outward from the centreline (right for a rotor on the centreline)">Cant°</th><th title="resulting unit thrust vector = CA_ROTORn_AX / AY / AZ">Axis AX AY AZ</th><th title="ducted fans: the fan along the thrust (jet), or a horizontal fan whose jetfoil bends the jet to the thrust axis (foil)">Duct</th><th>Spin</th><th title="max thrust N">Tmax</th><th title="share of max thrust this rotor needs to hover, as PX4's allocator would solve it">Hover</th><th></th></tr></thead><tbody>${rows}</tbody></table>`;
  api('/api/airframe/hover_check').then(showHover).catch(() => { });
  el.querySelectorAll('tr[data-i]').forEach(tr => {
    const i = +tr.dataset.i;
    tr.addEventListener('click', (e) => { if (!['INPUT', 'SELECT', 'OPTION'].includes(e.target.tagName) && !e.target.classList.contains('spin') && !e.target.classList.contains('del')) { selected = i; scene.select(i); renderRotorTable(); renderReadout(); } });
    tr.querySelectorAll('input, select').forEach(inp => inp.addEventListener('change', () => applyRow(i, tr)));
    tr.querySelector('.spin').addEventListener('click', () => { airframe.rotors[i].km = -airframe.rotors[i].km; scene.updateRotorNode(i, airframe.rotors[i]); renderRotorTable(); pushAirframe(true); });
    tr.querySelector('.del').addEventListener('click', () => { airframe.rotors.splice(i, 1); selected = -1; setAirframe(airframe); pushAirframe(true); });
  });
}
const axisText = (a) => a.map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(' ');
function rotorRowHtml(i, r) {
  const ccw = r.km >= 0;
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  return `<tr data-i="${i}" class="${i === selected ? 'selected' : ''}"><td class="idx">${i + 1}</td>
  <td><input type="number" step="0.005" data-k="x" value="${r.pos[0]}"></td>
  <td><input type="number" step="0.005" data-k="y" value="${r.pos[1]}"></td>
  <td><input type="number" step="0.005" data-k="z" value="${r.pos[2]}"></td>
  <td><input type="number" step="1" data-k="tilt" value="${+tilt.toFixed(1)}"></td>
  <td><input type="number" step="1" data-k="cant" value="${+cant.toFixed(1)}"></td>
  <td class="axis" title="CA_ROTOR${i}_AX / AY / AZ">${axisText(r.axis)}</td>
  <td>${r.kind === 'ducted' ? `<select data-k="duct"><option value="jet" ${r.duct_axis ? '' : 'selected'}>jet</option><option value="foil" ${r.duct_axis ? 'selected' : ''}>foil</option></select>` : ''}</td>
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
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  set('x', r.pos[0]); set('y', r.pos[1]); set('z', r.pos[2]); set('tilt', +tilt.toFixed(1)); set('cant', +cant.toFixed(1));
  const ax = tr.querySelector('td.axis'); if (ax) ax.textContent = axisText(r.axis);
}
function applyRow(i, tr) {
  const r = airframe.rotors[i];
  const g = (k) => parseFloat(tr.querySelector(`input[data-k="${k}"]`).value);
  r.pos = [g('x'), g('y'), g('z')];
  r.axis = tiltToAxis(g('tilt'), g('cant'), r.pos[1]);
  r.max_thrust = g('tmax');
  const duct = tr.querySelector('select[data-k="duct"]');
  if (duct) r.duct_axis = duct.value === 'foil' ? [1, 0, 0] : null;
  scene.updateRotorNode(i, r);
  renderRotorRow(i);
  renderReadout();
  pushAirframe(true);
}
function renderReadout() {
  const el = $('#rotor-readout');
  if (selected < 0 || !airframe.rotors[selected]) { el.classList.remove('show'); return; }
  const r = airframe.rotors[selected];
  const [tilt, cant] = axisToTilt(r.axis, r.pos[1]);
  el.classList.add('show');
  let foil = '';
  if (r.duct_axis) {
    const a = r.axis, d = r.duct_axis, na = Math.hypot(...a) || 1, nd = Math.hypot(...d) || 1;
    const bend = deg(Math.acos(Math.max(-1, Math.min(1, (a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / (na * nd)))));
    foil = ` · jetfoil bends ${bend.toFixed(0)}° (${((1 - (r.turn_loss ?? 0.1) * bend / 90) * 100).toFixed(0)}% thrust)`;
  }
  el.innerHTML = `<b>Motor ${selected + 1}</b> pos [${r.pos.map(v => fmt(v)).join(', ')}] · axis [${r.axis.map(v => fmt(v, 3)).join(', ')}] (tilt ${tilt.toFixed(1)}°, cant ${cant.toFixed(1)}°)${foil} · ${r.km >= 0 ? 'CCW' : 'CW'} · CA_ROTOR${selected}_*`;
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

// ============================================================ design / optimize
let designTimer = null, designGroups = null, optPoll = null, lastAnalysis = null;
const pct = (v) => (v == null || !isFinite(v)) ? '—' : (v * 100).toFixed(0) + '%';
const num = (v, d = 1, unit = '') => (v == null || !isFinite(v)) ? '—' : v.toFixed(d) + unit;
function designSpec() {
  const groups = {};
  for (const [name, g] of Object.entries(designGroups || {})) {
    const row = $(`#d-groups tr[data-g="${name}"]`);
    const on = (k) => row && row.querySelector(`input[data-k="${k}-on"]`).checked;
    const rng = (k) => [+row.querySelector(`input[data-k="${k}-lo"]`).value, +row.querySelector(`input[data-k="${k}-hi"]`).value];
    groups[name] = { rotors: g.rotors, tilt: on('tilt') ? rng('tilt') : null, cant: on('cant') ? rng('cant') : null };
  }
  return { groups, hover_pitch: $('#d-hp-on').checked ? [+$('#d-hp-lo').value, +$('#d-hp-hi').value] : null,
           weight: 1 - (+$('#d-weight').value) / 100, speed_kmh: +$('#d-speed').value, samples: +$('#d-samples').value, refine: 6 };
}
function groupsFromRotors() {   // saved assignment (airframe.design.groups) if it still fits the rotor count, else the server's default
  const saved = airframe.design && airframe.design.groups;
  if (saved && Object.values(saved).flatMap(g => g.rotors).length === airframe.rotors.length) return JSON.parse(JSON.stringify(saved));
  return lastAnalysis ? JSON.parse(JSON.stringify(lastAnalysis.groups)) : null;
}
function renderGroups() {
  if (!designGroups) return;
  const rows = Object.entries(designGroups).map(([name, g]) => {
    const [tilt, cant] = g.rotors[0] < airframe.rotors.length ? axisToTilt(airframe.rotors[g.rotors[0]].axis, airframe.rotors[g.rotors[0]].pos[1]) : [0, 0];
    const prev = g.ui || {};
    return `<tr data-g="${name}"><td class="idx">${name}</td><td class="motors">${g.rotors.map(i => 'M' + (i + 1)).join(' ')}</td>
      <td><input type="checkbox" data-k="tilt-on" ${prev.tilt === false ? '' : 'checked'}> <span class="num">${tilt.toFixed(0)}°</span></td>
      <td><input type="number" data-k="tilt-lo" value="${prev.tiltLo ?? 0}"> – <input type="number" data-k="tilt-hi" value="${prev.tiltHi ?? 90}"></td>
      <td><input type="checkbox" data-k="cant-on" ${prev.cant ? 'checked' : ''}> <span class="num">${cant.toFixed(0)}°</span></td>
      <td><input type="number" data-k="cant-lo" value="${prev.cantLo ?? 0}"> – <input type="number" data-k="cant-hi" value="${prev.cantHi ?? 30}"></td></tr>`;
  }).join('');
  const motorRow = airframe.rotors.map((r, i) => {
    const g = Object.entries(designGroups).find(([, g]) => g.rotors.includes(i));
    return `<label class="row tight" style="gap:2px"><span class="hint">M${i + 1}</span><input type="text" data-m="${i}" value="${g ? g[0] : ''}" maxlength="1"></label>`;
  }).join('');
  $('#d-groups').innerHTML = `<table class="grid"><thead><tr><th>Group</th><th>Motors</th><th title="tilt from vertical, forward/back">Tilt</th><th>Range°</th><th title="lateral cant, symmetric left/right">Cant</th><th>Range°</th></tr></thead><tbody>${rows}</tbody></table>
    <div class="row tight" style="margin-top:6px">${motorRow}</div>`;
  $$('#d-groups input[data-m]').forEach(inp => inp.addEventListener('change', () => {
    const letter = inp.value.trim().toUpperCase() || 'A'; const i = +inp.dataset.m;
    for (const g of Object.values(designGroups)) g.rotors = g.rotors.filter(k => k !== i);
    (designGroups[letter] = designGroups[letter] || { rotors: [] }).rotors.push(i);
    for (const k of Object.keys(designGroups)) if (!designGroups[k].rotors.length) delete designGroups[k];
    designGroups = Object.fromEntries(Object.keys(designGroups).sort().map(k => [k, designGroups[k]]));
    saveGroups(); renderGroups();
  }));
  $$('#d-groups tr[data-g] input').forEach(inp => inp.addEventListener('change', saveGroups));
}
function saveGroups() {
  for (const [name, g] of Object.entries(designGroups)) {
    const row = $(`#d-groups tr[data-g="${name}"]`); if (!row) continue;
    const v = (k) => row.querySelector(`input[data-k="${k}"]`);
    g.ui = { tilt: v('tilt-on').checked, tiltLo: +v('tilt-lo').value, tiltHi: +v('tilt-hi').value, cant: v('cant-on').checked, cantLo: +v('cant-lo').value, cantHi: +v('cant-hi').value };
  }
  airframe.design.groups = designGroups;
}
async function refreshDesign() {
  clearTimeout(designTimer);
  designTimer = setTimeout(async () => {
    if (!airframe) return;
    let r;
    try { r = await api('/api/design/analysis', { airframe, speed_kmh: +$('#d-speed').value }); } catch (e) { $('#d-now').textContent = e.message; return; }
    lastAnalysis = r;
    if (!designGroups) designGroups = groupsFromRotors(); else saveGroups();
    if (!document.activeElement || !document.activeElement.closest('#d-groups')) renderGroups();   // refresh current angles
    renderNow(r);
  }, 80);
}
function renderNow(r) {
  const h = r.hover, c = r.cruise;
  const auth = h.authority || {};
  const probs = [...(h.problems || []), ...(c.problems || []), ...(r.notes || [])];
  $('#d-notes').textContent = '';
  $('#d-now').innerHTML = `<h4>Hover · ${airframe.hover_pitch_deg || 0}° nose-up</h4><div class="kv">
      <div><span>Busiest motor</span><span class="${h.max_util > 0.85 ? 'err' : ''}">${pct(h.max_util)}</span></div>
      <div><span>Wasted thrust</span><span>${pct(h.waste)}</span></div>
      <div><span>Power</span><span>${num(h.power, 0, ' W')}</span></div>
      <div><span>Roll / pitch / yaw</span><span>${num(auth.roll, 1)} / ${num(auth.pitch, 1)} / ${num(auth.yaw, 1)} Nm</span></div></div>
    <h4>Cruise · ${(r.airspeed * 3.6).toFixed(0)} km/h</h4><div class="kv">
      <div><span>Body pitch</span><span>${num(c.pitch_deg, 1, '°')}</span></div>
      <div><span>PX4 pitch</span><span class="${Math.abs(c.px4_pitch_deg) > (r.tilt_limit_deg || 45) ? 'err' : ''}">${num(c.px4_pitch_deg, 1, '°')}</span></div>
      <div><span>Busiest motor</span><span class="${c.max_util > 0.85 ? 'err' : ''}">${pct(c.max_util)}</span></div>
      <div><span>Power vs hover</span><span>${pct(c.power_ratio)}</span></div>
      <div><span>Total thrust</span><span>${num(c.total_thrust, 0, ' N')}</span></div>
      <div><span>Wing lift</span><span>${pct(c.lift_share)} of weight</span></div>
      <div><span>Wing AoA</span><span>${num(c.alpha_deg, 1, '°')}</span></div>
      <div><span>Ram / wing / body drag</span><span>${num(c.ram_drag, 0)} / ${num(c.wing_drag, 0)} / ${num(c.body_drag, 0)} N</span></div></div>
    ${probs.length ? `<div class="problems">⚠ ${probs.join('<br>⚠ ')}</div>` : ''}`;
}
$('#d-speed').addEventListener('change', () => { airframe.design.cruise_speed_kmh = +$('#d-speed').value; pushAirframe(true); });
$('#d-run').addEventListener('click', async () => {
  saveGroups();
  const spec = designSpec();
  $('#d-run').disabled = true; $('#d-progress').textContent = 'starting…'; $('#d-results').innerHTML = '';
  try { await api('/api/design/optimize', { airframe, spec }); } catch (e) { $('#d-progress').textContent = e.message; $('#d-run').disabled = false; return; }
  clearInterval(optPoll);
  optPoll = setInterval(async () => {
    let j; try { j = await api('/api/design/optimize'); } catch { return; }
    if (j.running) { $('#d-progress').textContent = `${j.message} · ${Math.round(j.progress * 100)}%`; return; }
    clearInterval(optPoll); $('#d-run').disabled = false;
    if (j.error) { $('#d-progress').textContent = j.error; return; }
    if (j.result && j.result.ok === false) { $('#d-progress').textContent = j.result.error; return; }
    if (j.result) renderResults(j.result);
  }, 400);
});
function renderResults(res) {
  $('#d-progress').textContent = `${res.evaluated} designs evaluated, ${res.feasible} feasible`;
  const vars = res.variables;
  const head = vars.map(v => `<th>${v.kind === 'hover_pitch' ? 'Hover°' : v.group + ' ' + v.kind + '°'}</th>`).join('');
  const row = (m, i, cls, label) => `<tr class="${cls}"><td class="idx">${label}</td>${m.x.map(x => `<td class="num">${x.toFixed(1)}</td>`).join('')}
    <td class="num ${m.hover.ok ? '' : 'err'}">${pct(m.hover.max_util)}</td><td class="num">${num(m.hover.authority.yaw, 1)}</td>
    <td class="num ${m.cruise.converged ? '' : 'err'}">${pct(m.cruise.power_ratio)}</td><td class="num">${num(m.cruise.px4_pitch_deg, 0, '°')}</td><td class="num">${pct(m.cruise.lift_share)}</td>
    <td class="num">${pct(m.cruise.max_util)}</td><td>${i >= 0 ? `<button class="pill small" data-apply="${i}">Apply</button>` : ''}</td></tr>`;
  $('#d-results').innerHTML = `<table class="grid"><thead><tr><th></th>${head}<th title="busiest motor in hover">Hover</th><th title="yaw torque available in hover, Nm">Yaw</th><th title="cruise power relative to hover power">Cruise</th><th>PX4 pitch</th><th>Wing</th><th title="busiest motor in cruise">Motor</th><th></th></tr></thead>
    <tbody>${row(res.current, -1, 'current', 'now')}${res.results.map((m, i) => row(m, i, m.pareto ? 'pareto' : '', String(i + 1))).join('')}</tbody></table>`;
  $$('#d-results button[data-apply]').forEach(b => b.addEventListener('click', () => {
    const m = res.results[+b.dataset.apply];
    m.axes.forEach((a, i) => { if (airframe.rotors[i]) airframe.rotors[i].axis = a; });
    airframe.hover_pitch_deg = m.hover_pitch_deg;
    setAirframe(airframe); pushAirframe(true);
    b.textContent = 'Applied'; setTimeout(() => b.textContent = 'Apply', 1500);
  }));
}

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
  const tko = $('#btn-takeoff');
  tko.disabled = !status.ctl_connected || (!status.armed && !status.arm_ready);
  tko.classList.toggle('hidden', !!status.armed && !status.on_ground_hint && false);
  const upd = $('#btn-update');
  upd.disabled = !status.ctl_connected || !!status.armed;
  upd.title = status.armed ? 'Disarm first: PX4 rebuilds its allocation when these parameters change' :
    (status.ctl_connected ? 'Write the rotor geometry and output mapping to the flight controller and save it' : 'PX4 not connected');
  $$('#mode-pills .pill').forEach(b => b.classList.toggle('active', status.connected && (status.mode_name || '').toLowerCase() === b.textContent.toLowerCase()));
}
$('#btn-takeoff').addEventListener('click', async () => {
  try {
    if (!status.armed) { await api('/api/px4/command', { command: 'arm' }); await new Promise(r => setTimeout(r, 1500)); }
    await api('/api/px4/command', { command: 'takeoff' });
  } catch (e) { logLine('[ui] ' + e.message); }
});
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
const CAM_MODES = ['static', 'track', 'follow'], CAM_LABELS = { static: 'Static', track: 'Track', follow: 'Follow' };
let camMode = 'static';
$('#btn-follow').addEventListener('click', (e) => {
  camMode = CAM_MODES[(CAM_MODES.indexOf(camMode) + 1) % CAM_MODES.length];
  e.target.textContent = CAM_LABELS[camMode];
  e.target.classList.toggle('on', camMode !== 'static');
  scene.setCameraMode(camMode);
});
$('#sim-speed').addEventListener('change', e => api('/api/sim/speed', { speed: parseFloat(e.target.value) }));
$('#sim-noise').addEventListener('change', e => api('/api/sim/noise', { enabled: e.target.checked }));
$('#wind-apply').addEventListener('click', () => api('/api/sim/wind', { north: +$('#wind-n').value, east: +$('#wind-e').value, down: +$('#wind-d').value }));
$('#home-apply').addEventListener('click', () => api('/api/sim/home', { lat: +$('#home-lat').value, lon: +$('#home-lon').value, alt: +$('#home-alt').value }));

// one set of sliders: shows PX4's live motor commands, or drives the physics directly with Manual override on
function renderMotorSliders() {
  const el = $('#motor-sliders');
  el.innerHTML = airframe.rotors.map((r, i) => `<label><b>M${i + 1}</b> <input type="range" min="0" max="1" step="0.01" value="0" data-i="${i}" ${$('#motor-enable').checked ? '' : 'disabled'}><span class="mv num">0%</span></label>`).join('');
  el.querySelectorAll('input').forEach(inp => inp.addEventListener('input', sendOverride));
}
function sendOverride() {
  const vals = $$('#motor-sliders input').map(i => parseFloat(i.value));
  if ($('#motor-enable').checked) api('/api/sim/motor_override', { values: vals });
}
function updateMotorSliders(st) {
  const manual = $('#motor-enable').checked;
  st.rotors.forEach((x, i) => {
    const inp = document.querySelector(`#motor-sliders input[data-i="${i}"]`), lab = document.querySelector(`#motor-sliders label:nth-child(${i + 1}) .mv`);
    if (inp && !manual) inp.value = x.cmd.toFixed(2);
    const pct = airframe.rotors[i] && airframe.rotors[i].max_thrust > 0 ? x.thrust / airframe.rotors[i].max_thrust * 100 : 0;
    if (lab) lab.textContent = pct.toFixed(0) + '%';
  });
}
$('#motor-enable').addEventListener('change', (e) => {
  $$('#motor-sliders input').forEach(inp => inp.disabled = !e.target.checked);
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
  if (joyCard) joyCard.classList.toggle('hidden', s.conn_mode !== 'sitl');
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
  $('#st-pose').textContent = `N ${st.pos[0].toFixed(1)} E ${st.pos[1].toFixed(1)} alt ${(-st.pos[2]).toFixed(2)} m · R ${r.toFixed(0)}° P ${p.toFixed(0)}° Y ${y.toFixed(0)}°${st.on_ground ? ' · on ground' : ''}`;
  if ($('#tab-sim').classList.contains('active') && airframe) updateMotorSliders(st);
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
    // PX4 sends its arming/health checks as a batch right before each summary. Show only the batch that belongs
    // to the latest summary (everything older is history, not the current state), plus anything newer than it.
    const modeNow = (status.mode_name || '').toLowerCase();
    const lastSummary = [...ev].reverse().find(x => x.name === 'commander_arming_check_summary');
    const tCut = lastSummary ? lastSummary.t - 0.6 : (Date.now() / 1000 - 10);
    const seen = new Set();
    const shown = ev.filter(x => x.t >= tCut && x.level <= 6 && x.group !== 'protocol' && !x.name.includes('summary'))
      .filter(x => !(/offboard/i.test(x.text) && modeNow !== 'offboard') && !(/mission/i.test(x.text) && modeNow !== 'mission'))
      .reverse().filter(x => { if (seen.has(x.text)) return false; seen.add(x.text); return true; }).slice(0, 8);
    html += shown.map(x => `<div class="msg ${x.level <= 3 ? 'err' : x.level === 4 ? 'warn' : ''}"><span class="lvl">${x.level_name}</span> ${x.text}</div>`).join('') || '<div class="hint">no messages from PX4 yet</div>';
    el.innerHTML = html;
  } catch (e) { }
  eventsTimer = setTimeout(refreshEvents, 2000);
}
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
// --- Web Serial: RadioMaster T8L in config (VCP) mode. Protocol taken from RadioMaster's own web configurator:
// 460800 baud, poll with A5 55 1B 0D 0A every 20 ms, the radio answers frames  EE len <6 bytes header> ch1..ch10 (u16 LE) crc8
// where crc8 (CRSF polynomial 0xD5) covers bytes [2 .. len] and channels are 988..2012 with 1500 centred.
let serialPort = null, serialReader = null, serialWriter = null, serialTimer = null;
const T8L_VID = 0x19F5;
function crc8(bytes) { let c = 0; for (const b of bytes) { c ^= b; for (let i = 0; i < 8; i++) c = (c & 0x80) ? ((c << 1) ^ 0xD5) & 0xFF : (c << 1) & 0xFF; } return c; }
async function serialConnect() {
  if (!navigator.serial) { $('#joy-hint').textContent = 'This browser has no Web Serial (Safari). Open the page in Chrome or Edge.'; return; }
  let port;
  try {
    port = await navigator.serial.requestPort({ filters: [{ usbVendorId: T8L_VID }] });
  } catch (e) {
    if (e.name === 'NotFoundError') { try { port = await navigator.serial.requestPort(); } catch (e2) { return; } } else return;
  }
  await serialUse(port);
}
async function serialUse(port) {
  try { await port.open({ baudRate: 460800 }); } catch (e) { logLine('[ui] could not open the radio port: ' + e.message + ' (is the RadioMaster config page still connected to it?)'); return; }
  await serialDisconnect(false);
  serialPort = port; serialWriter = port.writable.getWriter(); serialReader = port.readable.getReader();
  joySrc = { name: 'RadioMaster T8L (serial)', axes: new Array(10).fill(0), buttons: [], kind: 'serial', frames: 0, t: 0 };
  hidDevice = null;
  const poll = new Uint8Array([0xA5, 0x55, 0x1B, 0x0D, 0x0A]);
  serialTimer = setInterval(() => { if (serialWriter) serialWriter.write(poll).catch(() => { }); }, 20);
  logLine('[ui] radio serial link open at 460800, polling channels');
  joyRenderMap();
  (async () => {
    let buf = [];
    try {
      while (serialReader) {
        const { value, done } = await serialReader.read();
        if (done) break;
        for (const b of value) buf.push(b);
        while (buf.length >= 3) {
          if (buf[0] !== 0xEE) { buf.shift(); continue; }
          const len = buf[1];
          if (len < 3 || len > 80) { buf.shift(); continue; }
          if (buf.length < len + 2) break;
          const frame = buf.slice(0, len + 2);
          if (crc8(frame.slice(2, 2 + len - 1)) === frame[len + 1] && len >= 27) {
            const ch = []; for (let i = 0; i < 10; i++) ch.push(frame[8 + 2 * i] | (frame[9 + 2 * i] << 8));
            joySrc.axes = ch.map(v => Math.max(-1, Math.min(1, (v - 1500) / 512)));
            joySrc.frames++; joySrc.t = performance.now();
          }
          buf.splice(0, len + 2);
        }
        if (buf.length > 4096) buf = [];
      }
    } catch (e) { logLine('[ui] radio serial read ended: ' + e.message); }
    await serialDisconnect(true);
  })();
}
async function serialDisconnect(announce) {
  if (serialTimer) { clearInterval(serialTimer); serialTimer = null; }
  const r = serialReader, w = serialWriter, p = serialPort;
  serialReader = null; serialWriter = null; serialPort = null;
  try { if (r) { await r.cancel(); r.releaseLock(); } } catch { }
  try { if (w) { w.releaseLock(); } } catch { }
  try { if (p) await p.close(); } catch { }
  if (joySrc && joySrc.kind === 'serial') joySrc = null;
  if (announce && p) { logLine('[ui] radio serial link closed'); joyRenderMap(); }
}
async function serialReconnect() {   // a port granted earlier comes back without the picker
  if (!navigator.serial) return;
  try {
    const ports = await navigator.serial.getPorts();
    const p = ports.find(x => (x.getInfo().usbVendorId === T8L_VID));
    if (p) await serialUse(p);
  } catch { }
}
if (navigator.serial) navigator.serial.addEventListener('disconnect', (e) => { if (e.target === serialPort) serialDisconnect(true); });

function joyCurrent() {
  if (joySrc && joySrc.kind === 'serial' && serialPort) return (performance.now() - joySrc.t < 1000 || joySrc.frames === 0) ? joySrc : { ...joySrc, name: joySrc.name + ' · no data' };
  return null;
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
    $('#joy-hint').textContent = (status.radio_vcp_ports && status.radio_vcp_ports.length) ? status.radio_vcp_ports[0].replace('/dev/', '') : '';
  }
  else {
    nameEl.textContent = src.name + (src.kind === 'gamepad' ? ' (gamepad)' : '');
    $('#joy-hint').textContent = src.kind === 'serial' ? `${src.axes.length} channels` : `${src.axes.length} axes`;
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
    if (status.conn_mode === 'sitl' && joyWs && joyWs.readyState === 1 && now - joyLastSend > 20) {   // 50 Hz, SITL only
      joyLastSend = now;
      const g = (k) => joyValue(src, joyMap.find(f => f.key === k));
      const aux = src.axes.slice(4, 10).map(v => +v.toFixed(3));
      let buttons = 0; src.buttons.forEach((b, i) => { if (b && i < 16) buttons |= (1 << i); });
      joyWs.send(JSON.stringify({ type: 'manual', roll: g('roll'), pitch: g('pitch'), throttle: (g('throttle') + 1) / 2, yaw: g('yaw'), buttons, aux }));
    }
  }
  if (src || $('#tab-connect').classList.contains('active')) requestAnimationFrame(joyTick); else setTimeout(joyTick, 500);
}
if (navigator.hid) navigator.hid.addEventListener('disconnect', (e) => { if (e.device === hidDevice) { hidDevice = null; joySrc = null; logLine('[ui] radio disconnected'); joyRenderMap(); } });
$('#joy-connect').addEventListener('click', serialConnect);
joyRenderMap(); joyTick(); serialReconnect();
