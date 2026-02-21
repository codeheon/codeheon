const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const telemetry = document.getElementById('telemetry');
const rateInput = document.getElementById('rate');
const camTiltInput = document.getElementById('camTilt');
const windInput = document.getElementById('wind');

const keys = new Set();
const world = {
  gravity: 9.81,
  drag: 0.11,
  angularDrag: 3.5,
  maxThrustAccel: 20.5,
  gateVisible: true,
  windPhase: Math.random() * Math.PI * 2,
  gates: [
    { x: 0, y: 2.5, z: 30, r: 3.3 },
    { x: 9, y: 4, z: 63, r: 2.8 },
    { x: -8, y: 3.5, z: 92, r: 3.2 },
    { x: 0, y: 4.5, z: 125, r: 3.4 },
  ],
};

const drone = {
  pos: { x: 0, y: 2.2, z: 0 },
  vel: { x: 0, y: 0, z: 0 },
  angles: { roll: 0, pitch: 0, yaw: 0 },
  angVel: { roll: 0, pitch: 0, yaw: 0 },
  throttle: 0.52,
  battery: 100,
  gateIndex: 0,
  lastGateTime: 0,
  crash: false,
  lapCount: 0,
};

let last = performance.now();

function resetDrone() {
  Object.assign(drone.pos, { x: 0, y: 2.2, z: 0 });
  Object.assign(drone.vel, { x: 0, y: 0, z: 0 });
  Object.assign(drone.angles, { roll: 0, pitch: 0, yaw: 0 });
  Object.assign(drone.angVel, { roll: 0, pitch: 0, yaw: 0 });
  drone.throttle = 0.52;
  drone.battery = 100;
  drone.gateIndex = 0;
  drone.lapCount = 0;
  drone.crash = false;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function rotateVectorByEuler(v, a) {
  const cx = Math.cos(a.roll);
  const sx = Math.sin(a.roll);
  const cy = Math.cos(a.pitch);
  const sy = Math.sin(a.pitch);
  const cz = Math.cos(a.yaw);
  const sz = Math.sin(a.yaw);

  const r00 = cz * cy;
  const r01 = cz * sy * sx - sz * cx;
  const r02 = cz * sy * cx + sz * sx;
  const r10 = sz * cy;
  const r11 = sz * sy * sx + cz * cx;
  const r12 = sz * sy * cx - cz * sx;
  const r20 = -sy;
  const r21 = cy * sx;
  const r22 = cy * cx;

  return {
    x: r00 * v.x + r01 * v.y + r02 * v.z,
    y: r10 * v.x + r11 * v.y + r12 * v.z,
    z: r20 * v.x + r21 * v.y + r22 * v.z,
  };
}

function getInput(dt) {
  const stick = {
    roll: (keys.has('d') ? 1 : 0) + (keys.has('a') ? -1 : 0),
    pitch: (keys.has('w') ? 1 : 0) + (keys.has('s') ? -1 : 0),
    yaw: (keys.has('e') ? 1 : 0) + (keys.has('q') ? -1 : 0),
  };

  const throttleRate = 0.42;
  if (keys.has('shift')) drone.throttle += throttleRate * dt;
  if (keys.has('control')) drone.throttle -= throttleRate * dt;
  drone.throttle = clamp(drone.throttle, 0.1, 1);

  const rateDeg = Number(rateInput.value);
  const maxRate = (rateDeg * Math.PI) / 180;
  drone.angVel.roll += (stick.roll * maxRate - drone.angVel.roll) * 10 * dt;
  drone.angVel.pitch += (stick.pitch * maxRate - drone.angVel.pitch) * 10 * dt;
  drone.angVel.yaw += (stick.yaw * maxRate * 0.72 - drone.angVel.yaw) * 8.2 * dt;
}

function updatePhysics(dt, time) {
  if (drone.crash) {
    drone.vel.x *= 0.98;
    drone.vel.z *= 0.98;
    return;
  }

  getInput(dt);

  drone.angles.roll += drone.angVel.roll * dt;
  drone.angles.pitch += drone.angVel.pitch * dt;
  drone.angles.yaw += drone.angVel.yaw * dt;

  const thrustAxis = rotateVectorByEuler({ x: 0, y: 1, z: 0 }, drone.angles);
  const batteryFactor = 0.75 + 0.25 * (drone.battery / 100);
  const thrustAccel = world.maxThrustAccel * drone.throttle * batteryFactor;

  const windStrength = Number(windInput.value);
  const wind = {
    x: Math.sin(time * 0.37 + world.windPhase) * windStrength,
    y: Math.sin(time * 0.21 + 1.2) * 0.5,
    z: Math.cos(time * 0.24 + world.windPhase) * windStrength * 0.5,
  };

  const accel = {
    x: thrustAxis.x * thrustAccel - world.drag * drone.vel.x + wind.x * 0.35,
    y: thrustAxis.y * thrustAccel - world.gravity - world.drag * drone.vel.y + wind.y * 0.2,
    z: thrustAxis.z * thrustAccel - world.drag * drone.vel.z + wind.z * 0.35,
  };

  drone.vel.x += accel.x * dt;
  drone.vel.y += accel.y * dt;
  drone.vel.z += accel.z * dt;

  drone.pos.x += drone.vel.x * dt;
  drone.pos.y += drone.vel.y * dt;
  drone.pos.z += drone.vel.z * dt;

  drone.angVel.roll -= drone.angVel.roll * world.angularDrag * dt * 0.1;
  drone.angVel.pitch -= drone.angVel.pitch * world.angularDrag * dt * 0.1;
  drone.angVel.yaw -= drone.angVel.yaw * world.angularDrag * dt * 0.1;

  const speed = Math.hypot(drone.vel.x, drone.vel.y, drone.vel.z);
  drone.battery = Math.max(0, drone.battery - (0.16 + drone.throttle * 0.25 + speed * 0.004) * dt);

  if (drone.pos.y < 0.18 && Math.abs(drone.vel.y) > 1.7) {
    drone.crash = true;
  }
  if (drone.pos.y < 0.05) {
    drone.pos.y = 0.05;
    drone.vel.y = 0;
  }

  handleGateProgress(time);
}

function handleGateProgress(time) {
  const gate = world.gates[drone.gateIndex];
  const dx = drone.pos.x - gate.x;
  const dy = drone.pos.y - gate.y;
  const dz = drone.pos.z - gate.z;
  const dist = Math.hypot(dx, dy, dz);

  if (dist < gate.r && time - drone.lastGateTime > 0.35) {
    drone.lastGateTime = time;
    drone.gateIndex += 1;
    if (drone.gateIndex >= world.gates.length) {
      drone.gateIndex = 0;
      drone.lapCount += 1;
    }
  }
}

function worldToCamera(point) {
  const camTilt = (Number(camTiltInput.value) * Math.PI) / 180;
  const camAngles = {
    roll: drone.angles.roll,
    pitch: drone.angles.pitch + camTilt,
    yaw: drone.angles.yaw,
  };

  const relative = {
    x: point.x - drone.pos.x,
    y: point.y - drone.pos.y,
    z: point.z - drone.pos.z,
  };

  const inv = {
    roll: -camAngles.roll,
    pitch: -camAngles.pitch,
    yaw: -camAngles.yaw,
  };

  return rotateVectorByEuler(relative, inv);
}

function project(camPoint) {
  const f = 750;
  if (camPoint.z < 0.2) return null;
  return {
    x: canvas.width / 2 + (camPoint.x / camPoint.z) * f,
    y: canvas.height / 2 - (camPoint.y / camPoint.z) * f,
    depth: camPoint.z,
  };
}

function drawHorizon() {
  const sky = ctx.createLinearGradient(0, 0, 0, canvas.height);
  sky.addColorStop(0, '#6ca4ff');
  sky.addColorStop(0.5, '#9bc1ff');
  sky.addColorStop(0.501, '#4e8f47');
  sky.addColorStop(1, '#2f5932');

  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawGate(gate, active) {
  const segments = 36;
  const pts = [];
  for (let i = 0; i <= segments; i += 1) {
    const a = (i / segments) * Math.PI * 2;
    const p = {
      x: gate.x + Math.cos(a) * gate.r,
      y: gate.y + Math.sin(a) * gate.r,
      z: gate.z,
    };
    const prj = project(worldToCamera(p));
    if (!prj) return;
    pts.push(prj);
  }

  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
  ctx.strokeStyle = active ? '#ffcc5f' : '#e8f1ff';
  ctx.lineWidth = active ? 4 : 2;
  ctx.shadowBlur = active ? 12 : 4;
  ctx.shadowColor = active ? '#ffcf6d' : '#bfd0ff';
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function drawGroundMarkers() {
  for (let z = 10; z <= 220; z += 5) {
    const left = project(worldToCamera({ x: -22, y: 0, z }));
    const right = project(worldToCamera({ x: 22, y: 0, z }));
    if (!left || !right) continue;

    const alpha = clamp(1 - z / 220, 0.1, 0.5);
    ctx.strokeStyle = `rgba(15,30,40,${alpha.toFixed(3)})`;
    ctx.beginPath();
    ctx.moveTo(left.x, left.y);
    ctx.lineTo(right.x, right.y);
    ctx.stroke();
  }
}

function drawCrosshair() {
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  ctx.strokeStyle = 'rgba(255,255,255,0.75)';
  ctx.lineWidth = 1.5;

  ctx.beginPath();
  ctx.moveTo(cx - 18, cy);
  ctx.lineTo(cx + 18, cy);
  ctx.moveTo(cx, cy - 18);
  ctx.lineTo(cx, cy + 18);
  ctx.stroke();
}

function drawScene() {
  drawHorizon();
  drawGroundMarkers();

  if (world.gateVisible) {
    world.gates.forEach((gate, idx) => drawGate(gate, idx === drone.gateIndex));
  }

  drawCrosshair();

  if (drone.crash) {
    ctx.fillStyle = 'rgba(0,0,0,0.45)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#ff8e88';
    ctx.font = '700 48px sans-serif';
    ctx.fillText('CRASHED', canvas.width / 2 - 120, canvas.height / 2 - 10);
    ctx.fillStyle = '#fff3dc';
    ctx.font = '500 24px sans-serif';
    ctx.fillText('R 키로 리셋', canvas.width / 2 - 68, canvas.height / 2 + 30);
  }
}

function updateTelemetry() {
  const speed = Math.hypot(drone.vel.x, drone.vel.y, drone.vel.z);
  const gate = world.gates[drone.gateIndex];
  const dist = Math.hypot(drone.pos.x - gate.x, drone.pos.y - gate.y, drone.pos.z - gate.z);

  const roll = (drone.angles.roll * 180) / Math.PI;
  const pitch = (drone.angles.pitch * 180) / Math.PI;
  const yaw = (drone.angles.yaw * 180) / Math.PI;

  telemetry.textContent = [
    `속도       : ${speed.toFixed(1)} m/s`,
    `고도       : ${drone.pos.y.toFixed(1)} m`,
    `스로틀     : ${(drone.throttle * 100).toFixed(0)} %`,
    `배터리     : ${drone.battery.toFixed(1)} %`,
    `각도 R/P/Y : ${roll.toFixed(0)}° / ${pitch.toFixed(0)}° / ${yaw.toFixed(0)}°`,
    `다음 게이트: #${drone.gateIndex + 1} (${dist.toFixed(1)} m)`,
    `랩 수      : ${drone.lapCount}`,
    drone.crash ? '상태       : CRASH' : '상태       : FLYING',
  ].join('\n');
}

function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.033);
  const time = now / 1000;
  last = now;

  updatePhysics(dt, time);
  drawScene();
  updateTelemetry();

  requestAnimationFrame(frame);
}

window.addEventListener('keydown', (e) => {
  const key = e.key.toLowerCase();
  keys.add(key);

  if (key === 'r') resetDrone();
  if (key === 'g') world.gateVisible = !world.gateVisible;
});

window.addEventListener('keyup', (e) => {
  keys.delete(e.key.toLowerCase());
});

window.addEventListener('blur', () => keys.clear());

resetDrone();
requestAnimationFrame(frame);
