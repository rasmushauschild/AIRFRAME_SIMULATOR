// 3D scene: vehicle body, rotors with drag gizmos, live thrust vectors.
// Frames: PX4 FRD/NED (x fwd/north, y right/east, z down)  ->  three.js (x, y=up, z)  via  (x, -z, y).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

export const frdToThree = (v) => new THREE.Vector3(v[0], -v[2], v[1]);
export const threeToFrd = (v) => [v.x, v.z, -v.y];
const UP = new THREE.Vector3(0, 1, 0);

export function createScene(canvas, handlers) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const THEMES = {
    light: { bg: 0xf6f6f8, grid1: 0xd6d7dd, grid2: 0xe6e7ec, ground: 0xf3f3f6, body: 0x3a3a3f, arm: 0x9a9aa3, motor: 0x2a2a2f },
    dark: { bg: 0x131419, grid1: 0x2a2c34, grid2: 0x1d1f26, ground: 0x15161b, body: 0x4a4d57, arm: 0x8a8f9a, motor: 0x22242b },
  };
  let theme = THEMES.light;

  const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 500);
  camera.position.set(1.4, 1.0, 1.6);

  const orbit = new OrbitControls(camera, canvas);
  orbit.enableDamping = true;
  orbit.dampingFactor = 0.12;
  orbit.target.set(0, 0.1, 0);

  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1e26, 1.0));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(5, 10, 4);
  scene.add(sun);

  // ground
  let grid = new THREE.GridHelper(200, 200, theme.grid1, theme.grid2);
  scene.add(grid);
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ color: theme.ground, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.002;
  scene.add(ground);
  // world axes: N (red x), E (blue z), up (green)
  scene.add(new THREE.AxesHelper(0.5));
  addLabel(scene, 'N', [0.55, 0.02, 0], 0xff3b30);
  addLabel(scene, 'E', [0, 0.02, 0.55], 0x0a84ff);

  // vehicle
  const vehicle = new THREE.Group();
  scene.add(vehicle);
  const frame = new THREE.Group();     // geometry only (edited)
  vehicle.add(frame);
  const bodyMat = new THREE.MeshStandardMaterial({ color: theme.body, roughness: 0.6, metalness: 0.2 });
  const armMat = new THREE.MeshStandardMaterial({ color: theme.arm, roughness: 0.7 });
  const matCCW = new THREE.MeshStandardMaterial({ color: 0x34c759, transparent: true, opacity: 0.35, side: THREE.DoubleSide });
  const matCW = new THREE.MeshStandardMaterial({ color: 0xff9500, transparent: true, opacity: 0.35, side: THREE.DoubleSide });
  const matSel = new THREE.MeshStandardMaterial({ color: 0x0a84ff, roughness: 0.4 });
  const motorMat = new THREE.MeshStandardMaterial({ color: theme.motor, roughness: 0.5, metalness: 0.5 });
  const bodyAxes = new THREE.AxesHelper(0.25);
  frame.add(bodyAxes);

  let body = null;
  let rotorNodes = [];   // { group, disc, motor, arrow, label, ring, rotorIndex }
  let legNodes = [];
  let airframe = null;
  let selected = -1;
  let follow = false;

  // gizmo
  const gizmo = new TransformControls(camera, canvas);
  gizmo.setSize(0.55);
  gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; if (!e.value) commitGizmo(); });
  gizmo.addEventListener('objectChange', () => onGizmoChange());
  const gizmoHelper = gizmo.getHelper ? gizmo.getHelper() : gizmo;
  scene.add(gizmoHelper);

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let downPos = null;
  canvas.addEventListener('pointerdown', (e) => { downPos = [e.clientX, e.clientY]; });
  canvas.addEventListener('pointerup', (e) => {
    if (!downPos) return;
    const moved = Math.hypot(e.clientX - downPos[0], e.clientY - downPos[1]);
    downPos = null;
    if (moved > 4 || gizmo.dragging) return;
    const r = canvas.getBoundingClientRect();
    pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObjects(rotorNodes.flatMap(n => [n.disc, n.motor]), false);
    if (hits.length) {
      const idx = hits[0].object.userData.rotorIndex;
      select(idx);
      handlers.onSelect && handlers.onSelect(idx);
    } else if (!gizmo.axis) {
      select(-1);
      handlers.onSelect && handlers.onSelect(-1);
    }
  });
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
    if (e.key === 'w' || e.key === 'W') gizmo.setMode('translate');
    if (e.key === 'e' || e.key === 'E') gizmo.setMode('rotate');
    if (e.key === 'Escape') { select(-1); handlers.onSelect && handlers.onSelect(-1); }
  });

  function onGizmoChange() {
    if (selected < 0 || !airframe) return;
    const n = rotorNodes[selected];
    const r = airframe.rotors[selected];
    r.pos = threeToFrd(n.group.position).map(v => +v.toFixed(4));
    const ax = UP.clone().applyQuaternion(n.group.quaternion);
    r.axis = threeToFrd(ax).map(v => +v.toFixed(4));
    updateArm(n, r);
    handlers.onRotorChanged && handlers.onRotorChanged(selected, r, false);
  }
  function commitGizmo() {
    if (selected < 0 || !airframe) return;
    handlers.onRotorChanged && handlers.onRotorChanged(selected, airframe.rotors[selected], true);
  }

  // ------------------------------------------------------------ build
  function setAirframe(af) {
    airframe = af;
    const keepSel = selected;
    gizmo.detach();
    for (const n of rotorNodes) frame.remove(n.group), frame.remove(n.arm);
    for (const l of legNodes) frame.remove(l);
    if (body) frame.remove(body);
    rotorNodes = []; legNodes = [];

    const [bx, by, bz] = af.body_size;
    body = new THREE.Mesh(new THREE.BoxGeometry(bx, bz, by), bodyMat);
    frame.add(body);

    af.rotors.forEach((r, i) => {
      const group = new THREE.Group();
      group.position.copy(frdToThree(r.pos));
      group.quaternion.setFromUnitVectors(UP, frdToThree(r.axis).normalize());
      const rad = Math.max(0.03, r.prop_diameter / 2);
      const disc = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, 0.004, 40), r.km >= 0 ? matCCW : matCW);
      disc.position.y = 0.02;
      disc.userData.rotorIndex = i;
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.016, 0.03, 18), motorMat);
      motor.userData.rotorIndex = i;
      const ring = new THREE.Mesh(new THREE.TorusGeometry(rad, 0.003, 6, 48), r.km >= 0 ? matCCW : matCW);
      ring.rotation.x = Math.PI / 2; ring.position.y = 0.02;
      const arrow = new THREE.ArrowHelper(UP, new THREE.Vector3(0, 0.02, 0), 0.001, 0x0a84ff, 0.04, 0.02);
      const spinArrow = makeSpinArrow(rad * 0.75, r.km >= 0);
      spinArrow.position.y = 0.024;
      const label = makeSprite(String(i + 1), document.documentElement.dataset.theme === 'dark' ? '#fafafa' : '#171717');
      label.position.set(0, 0.09, 0);
      group.add(disc, motor, ring, arrow, spinArrow, label);
      const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 1, 8), armMat);
      const node = { group, disc, motor, arrow, ring, label, arm, spinArrow, rotorIndex: i };
      updateArm(node, r);
      frame.add(group, arm);
      rotorNodes.push(node);
    });

    for (const p of af.leg_points || []) {
      const v = frdToThree(p);
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, v.length(), 6), armMat);
      leg.position.copy(v.clone().multiplyScalar(0.5));
      leg.quaternion.setFromUnitVectors(UP, v.clone().normalize());
      const foot = new THREE.Mesh(new THREE.SphereGeometry(0.01, 8, 8), armMat);
      foot.position.copy(v);
      frame.add(leg, foot);
      legNodes.push(leg, foot);
    }
    if (keepSel >= 0 && keepSel < rotorNodes.length) select(keepSel);
  }

  function updateArm(node, r) {
    const p = node.group.position;
    const len = p.length();
    node.arm.visible = len > 0.02;
    node.arm.scale.y = Math.max(len, 0.001);
    node.arm.position.copy(p.clone().multiplyScalar(0.5));
    node.arm.quaternion.setFromUnitVectors(UP, p.clone().normalize());
  }

  function updateRotorNode(i, r) {   // called when the table edits a rotor
    const n = rotorNodes[i];
    if (!n) return;
    n.group.position.copy(frdToThree(r.pos));
    n.group.quaternion.setFromUnitVectors(UP, frdToThree(r.axis).normalize());
    const m = r.km >= 0 ? matCCW : matCW;
    n.disc.material = m; n.ring.material = m;
    updateArm(n, r);
  }

  function select(i) {
    selected = i;
    rotorNodes.forEach((n, k) => { n.motor.material = k === i ? matSel : motorMat; });
    if (i >= 0 && rotorNodes[i]) gizmo.attach(rotorNodes[i].group);
    else gizmo.detach();
  }

  // ------------------------------------------------------------ live
  const tmpQ = new THREE.Quaternion();
  function updateState(st) {
    if (!st) return;
    vehicle.position.set(st.pos[0], -st.pos[2], st.pos[1]);
    const [w, x, y, z] = st.q;
    tmpQ.set(x, -z, y, w);
    vehicle.quaternion.copy(tmpQ);
    st.rotors.forEach((rs, i) => {
      const n = rotorNodes[i];
      if (!n) return;
      const r = airframe.rotors[i];
      const frac = r.max_thrust > 0 ? rs.thrust / r.max_thrust : 0;
      n.arrow.setLength(Math.max(0.001, frac * 0.35), 0.04, 0.02);
      n.disc.rotation.y += (r.km >= 0 ? 1 : -1) * rs.omega * 0.6;
      n.disc.material.opacity = 0.25 + 0.5 * rs.omega;
    });
    if (follow) {
      orbit.target.lerp(vehicle.position, 0.15);
    }
  }

  // ------------------------------------------------------------ loop
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w || canvas.height !== h) {
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
  }
  function animate() {
    resize();
    orbit.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }
  animate();

  function setTheme(name) {
    theme = THEMES[name] || THEMES.light;
    scene.background = new THREE.Color(theme.bg);
    scene.fog = new THREE.Fog(theme.bg, 40, 140);
    scene.remove(grid);
    grid = new THREE.GridHelper(200, 200, theme.grid1, theme.grid2);
    scene.add(grid);
    ground.material.color.set(theme.ground);
    bodyMat.color.set(theme.body); armMat.color.set(theme.arm); motorMat.color.set(theme.motor);
    rotorNodes.forEach(n => { n.label.material.map = makeSprite(String(n.rotorIndex + 1), name === 'dark' ? '#fafafa' : '#171717', name === 'dark').material.map; });
  }
  setTheme(document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light');

  return {
    setAirframe, updateState, select, updateRotorNode, setTheme,
    setFollow: (b) => { follow = b; if (!b) orbit.target.set(0, 0.1, 0); },
    setMode: (m) => gizmo.setMode(m),
    get selected() { return selected; },
    focusOrigin: () => { orbit.target.set(0, 0.1, 0); camera.position.set(1.4, 1.0, 1.6); },
  };
}

function makeSprite(text, color = '#171717', dark = document.documentElement.dataset.theme === 'dark') {
  const c = document.createElement('canvas'); c.width = 64; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = dark ? 'rgba(18,19,23,0.92)' : 'rgba(255,255,255,0.92)'; g.beginPath(); g.arc(32, 32, 28, 0, Math.PI * 2); g.fill();
  g.strokeStyle = dark ? 'rgba(255,255,255,0.15)' : 'rgba(0,0,0,0.12)'; g.lineWidth = 2; g.stroke();
  g.fillStyle = color; g.font = '700 32px -apple-system, BlinkMacSystemFont, "SF Pro Text", sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  s.scale.set(0.06, 0.06, 1);
  return s;
}
function addLabel(scene, text, pos, color) {
  const s = makeSprite(text, '#' + color.toString(16).padStart(6, '0'));
  s.position.set(...pos); s.scale.set(0.12, 0.12, 1);
  scene.add(s);
}
function makeSpinArrow(radius, ccw) {
  // arc with an arrowhead showing spin direction when viewed from above (three: looking down -y)
  const pts = [];
  const a0 = 0, a1 = Math.PI * 1.4;
  for (let k = 0; k <= 24; k++) {
    const a = a0 + (a1 - a0) * k / 24;
    // CCW from above: in three coords (x right? no) -> viewed from +y looking down, x to the right and z toward viewer.
    // A rotation that is CCW as seen from above corresponds to positive rotation about +y (right hand rule): x -> -z.
    const s = ccw ? 1 : -1;
    pts.push(new THREE.Vector3(radius * Math.cos(a), 0, -s * radius * Math.sin(a)));
  }
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: ccw ? 0x34c759 : 0xff9500 }));
  const tip = pts[pts.length - 1], prev = pts[pts.length - 2];
  const dir = tip.clone().sub(prev).normalize();
  const head = new THREE.ArrowHelper(dir, prev, 0.02, ccw ? 0x34c759 : 0xff9500, 0.02, 0.012);
  const g = new THREE.Group(); g.add(line, head);
  return g;
}
