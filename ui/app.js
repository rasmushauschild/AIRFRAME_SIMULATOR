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

// ============================================================ scene
const scene = createScene($('#c'), {
  onSelect: (i) => { selected = i; renderRotorTable(); renderReadout(); },
  onRotorChanged: (i, r, commit) => { renderRotorRow(i); renderReadout(); if (commit) pushAirframe(); },
});

// ============================================================ tabs
$$('.tabs button').forEach(b => b.addEventListener('click', () => {
  $$('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  $$('.tab').forEach(t => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  if (b.dataset.tab === 'px4') loadExport();
  if (b.dataset.tab === 'params') ensureParams();
  if (b.dataset.tab === 'connect') refreshConnection();
}));
function openTab(name) { $$('.tabs button').find(b => b.dataset.tab === name)?.click(); }

// ============================================================ airframe editing
function pushAirframe(immediate = false) {
  clearTimeout(pushTimer);
  const doPush = async () => {
    try {
      const res = await api('/api/airframe', { airframe, keep_state: true });
      showProblems(res.problems);
      showHover(res.hover);
      markDirty();
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
  if (af.leg_points && af.leg_points.length) {
    $('#af-legz').value = +af.leg_points[0][2].toFixed(3);
    $('#af-legxy').value = +Math.abs(af.leg_points[0][0]).toFixed(3);
  }
  renderRotorTable();
  renderMotorSliders();
  fillMotorCard();
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
  const z = parseFloat($('#af-legz').value), s = parseFloat($('#af-legxy').value);
  airframe.leg_points = [[s, s, z], [s, -s, z], [-s, s, z], [-s, -s, z]];
};
bindNumber('af-legz', setLegs);
bindNumber('af-legxy', setLegs);
// legs that hold the airframe at its hover pitch on the ground: a flat rectangle under the vehicle in the hover
// frame, rotated back into the structural frame
$('#af-legs-hover').addEventListener('click', () => {
  const phi = (airframe.hover_pitch_deg || 0) * Math.PI / 180, c = Math.cos(phi), sn = Math.sin(phi);
  const h = parseFloat($('#af-legz').value) || 0.2, sp = parseFloat($('#af-legxy').value) || 0.2;
  const toStructural = ([x, y, z]) => [c * x - sn * z, y, sn * x + c * z];   // inverse of the hover rotation
  airframe.leg_points = [[sp, sp, h], [sp, -sp, h], [-sp, sp, h], [-sp, -sp, h]].map(toStructural).map(v => v.map(x => +x.toFixed(3)));
  scene.setAirframe(airframe); pushAirframe(true); api('/api/sim/reset', {});
});
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
  const r = await api('/api/px4/export');
  const tb = $('#px4-export-table tbody');
  tb.innerHTML = Object.entries(r.params).map(([k, v]) => {
    const cur = r.current[k];
    const same = cur !== null && cur !== undefined && Math.abs(+cur - +v) < 1e-4;
    const f = (x) => (typeof x === 'number' && !Number.isInteger(x)) ? +x.toFixed(4) : x;
    return `<tr><td class="mono">${k}</td><td class="num">${f(v)}</td><td class="${cur == null ? 'muted' : same ? 'ok' : 'diff'}">${cur == null ? '—' : f(cur)}</td><td>${cur == null ? '' : same ? '✓' : '≠'}</td></tr>`;
  }).join('');
  $('#px4-export-status').innerHTML = r.problems.length ? `<div class="problems">⚠ ${r.problems.join('<br>⚠ ')}</div>` : '';
}
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
    if ($('#tab-px4').classList.contains('active')) await loadExport();
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
  armBtn.textContent = status.armed ? 'Disarm' : 'Arm';
  armBtn.classList.toggle('armed', !!status.armed);
  armBtn.disabled = !status.ctl_connected;
  const upd = $('#btn-update');
  upd.disabled = !status.ctl_connected || !!status.armed;
  upd.title = status.armed ? 'Disarm first: PX4 rebuilds its allocation when these parameters change' :
    (status.ctl_connected ? 'Write the rotor geometry and output mapping to the flight controller and save it' : 'PX4 not connected');
  $$('#mode-pills .pill').forEach(b => b.classList.toggle('active', status.connected && (status.mode_name || '').toLowerCase() === b.textContent.toLowerCase()));
}
$('#btn-arm').addEventListener('click', async () => {
  try { await api('/api/px4/command', { command: status.armed ? 'disarm' : 'arm' }); } catch (e) { logLine('[ui] ' + e.message); }
});

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
  $('#param-count').textContent = `${Object.keys(params).length}/${r.count} loaded · ${Object.keys(meta).length} described`;
  renderParams();
}
function renderParams() {
  const q = $('#param-search').value.trim().toLowerCase();
  const grp = $('#param-group').value;
  const names = Object.keys(params).sort();
  let shown = 0;
  const html = [];
  for (const n of names) {
    const m = meta[n] || {};
    if (grp && m.group !== grp) continue;
    if (q && !(n.toLowerCase().includes(q) || (m.short || '').toLowerCase().includes(q))) continue;
    if (++shown > 400) break;
    const v = params[n].value;
    html.push(`<div class="prow ${n === selectedParam ? 'selected' : ''}" data-n="${n}"><span class="pname">${n}</span><span class="pval">${typeof v === 'number' && !Number.isInteger(v) ? +v.toPrecision(6) : v}${m.unit ? ' <span class="muted">' + m.unit + '</span>' : ''}</span><span class="pdesc" title="${(m.short || '').replace(/"/g, '&quot;')}">${m.short || ''}</span></div>`);
  }
  $('#param-list').innerHTML = html.join('') + (shown > 400 ? '<div class="muted">… refine the search to see more</div>' : '');
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
      const r = await api('/api/params/set', { name: n, value });
      $('#pe-result').innerHTML = r.ok ? `<span class="ok">✓ ${r.value}</span>` : `<span class="err">${r.error}</span>`;
      if (r.ok) { params[n].value = r.value; renderParams(); }
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
$('#btn-reset').addEventListener('click', () => api('/api/sim/reset', {}));
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
  const det = $('#st-detected');
  const showDet = !s.flashing && s.mode !== 'hitl' && s.px4_ports && s.px4_ports.length > 0;
  if (s.flashing) { $('#st-mode').textContent = 'Pixhawk'; $('#st-conn').textContent = 'flashing firmware…'; }
  det.classList.toggle('hidden', !showDet); det.classList.toggle('blink', showDet);
  if (showDet) det.textContent = `Pixhawk on ${s.px4_ports[0].replace('/dev/', '')} · connect`;
  $('#st-armed').textContent = s.armed ? 'Armed' : 'Disarmed';
  $('#st-armed').classList.toggle('armed', s.armed);
  $('#st-flightmode').textContent = s.connected ? s.mode_name + (s.mode === 'hitl' && !s.hil_enabled ? ' · HIL OFF (set SYS_HITL=1)' : '') : '—';
  if (!homeFilled) { $('#home-lat').value = s.home.lat; $('#home-lon').value = s.home.lon; $('#home-alt').value = s.home.alt; homeFilled = true; }
  if (s.params_loaded && !paramsLoaded && $('#tab-params').classList.contains('active')) ensureParams();
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
      html += `<div class="conn-row"><div><b>${canArm.length ? 'Can arm in' : 'Cannot arm'}</b> <span class="hint">${canArm.length ? canArm.join(', ') : 'see messages below'}</span>` +
        `${d.error && d.error !== 0 ? `<div class="err">errors: ${String(d.error).replace(/\|/g, ', ')}</div>` : ''}${d.warning && d.warning !== 0 ? `<div class="warn">warnings: ${String(d.warning).replace(/\|/g, ', ')}</div>` : ''}</div></div>`;
    }
    const shown = ev.filter(x => x.level <= 6 && x.group !== 'protocol' && !x.name.includes('summary')).slice(-10).reverse();
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
