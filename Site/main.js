// ─── CURSOR ───────────────────────────────────────────────────────────────────
const cursor = document.getElementById('cursor');
const ring   = document.getElementById('cursor-ring');
let mx = 0, my = 0, rx = 0, ry = 0;

document.addEventListener('mousemove', e => {
    mx = e.clientX; my = e.clientY;
    cursor.style.left = mx - 6 + 'px';
    cursor.style.top  = my - 6 + 'px';
});

(function lerpRing(){
    rx += (mx - rx) * 0.12;
    ry += (my - ry) * 0.12;
    ring.style.left = rx - 18 + 'px';
    ring.style.top  = ry - 18 + 'px';
    requestAnimationFrame(lerpRing);
})();

document.querySelectorAll('a, button').forEach(el => {
    el.addEventListener('mouseenter', () => { ring.style.width = '60px'; ring.style.height = '60px'; });
    el.addEventListener('mouseleave', () => { ring.style.width = '36px'; ring.style.height = '36px'; });
});

// ─── HUD UPTIME ───────────────────────────────────────────────────────────────
let t = 0;
setInterval(() => {
    t++;
    const h = String(Math.floor(t / 3600)).padStart(2, '0');
    const m = String(Math.floor((t % 3600) / 60)).padStart(2, '0');
    const s = String(t % 60).padStart(2, '0');
    document.getElementById('hud-uptime').textContent = `${h}:${m}:${s}`;
}, 1000);

// ─── THREE.JS HERO ────────────────────────────────────────────────────────────
const canvas = document.getElementById('three-canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(canvas.parentElement.clientWidth, canvas.parentElement.clientHeight);
renderer.setClearColor(0x000000, 0);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
camera.position.set(0, 2, 14);

// Lights
const ambient = new THREE.AmbientLight(0x0a1a2a, 1);
scene.add(ambient);
const blueLight = new THREE.PointLight(0x00c8ff, 3, 30);
blueLight.position.set(5, 5, 5);
scene.add(blueLight);
const orangeLight = new THREE.PointLight(0xff6a00, 2, 20);
orangeLight.position.set(-5, -2, 3);
scene.add(orangeLight);
const greenLight = new THREE.PointLight(0x00ff9d, 1, 15);
greenLight.position.set(0, 8, -5);
scene.add(greenLight);

// Materials
const bodyMat   = new THREE.MeshStandardMaterial({ color: 0x0d2236, metalness: 0.7, roughness: 0.3 });
const wireMat   = new THREE.MeshStandardMaterial({ color: 0x00c8ff, metalness: 0.9, roughness: 0.1, emissive: 0x004466 });
const accentMat = new THREE.MeshStandardMaterial({ color: 0xff6a00, emissive: 0x441a00, metalness: 0.5, roughness: 0.4 });
const glassMat  = new THREE.MeshStandardMaterial({ color: 0x00aaff, transparent: true, opacity: 0.35, metalness: 1, roughness: 0 });
const darkMat   = new THREE.MeshStandardMaterial({ color: 0x050e18, metalness: 0.6, roughness: 0.4 });

// ROVER GROUP
const rover = new THREE.Group();
scene.add(rover);

// ── CHASSIS ──
const chassisGeo = new THREE.BoxGeometry(3.2, 0.7, 2.0);
const chassis = new THREE.Mesh(chassisGeo, bodyMat);
chassis.position.y = 0.5;
rover.add(chassis);

// Chassis edge glow strips
const edgeMat = new THREE.MeshStandardMaterial({ color: 0x00c8ff, emissive: 0x00c8ff, emissiveIntensity: 0.6 });
const edgeGeo = new THREE.BoxGeometry(3.25, 0.04, 0.04);
for (let z of [-1.02, 1.02]) {
    const edge = new THREE.Mesh(edgeGeo, edgeMat);
    edge.position.set(0, 0.87, z);
    rover.add(edge);
}
const edgeGeo2 = new THREE.BoxGeometry(0.04, 0.04, 2.05);
for (let x of [-1.62, 1.62]) {
    const edge = new THREE.Mesh(edgeGeo2, edgeMat);
    edge.position.set(x, 0.87, 0);
    rover.add(edge);
}

// ── TOP MODULE ──
const topGeo = new THREE.BoxGeometry(2.2, 0.5, 1.5);
const topMod = new THREE.Mesh(topGeo, darkMat);
topMod.position.set(0, 1.1, 0);
rover.add(topMod);

// ── SENSOR TOWER ──
const towerGeo = new THREE.CylinderGeometry(0.18, 0.22, 0.6, 8);
const tower = new THREE.Mesh(towerGeo, bodyMat);
tower.position.set(0, 1.65, 0);
rover.add(tower);

// LIDAR disk
const lidarGeo = new THREE.CylinderGeometry(0.28, 0.28, 0.12, 16);
const lidar = new THREE.Mesh(lidarGeo, wireMat);
lidar.position.set(0, 2.0, 0);
rover.add(lidar);

// LIDAR scan plane (rotating)
const scanPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(2, 0.02),
    new THREE.MeshBasicMaterial({ color: 0x00ff9d, transparent: true, opacity: 0.7, side: THREE.DoubleSide })
);
scanPlane.position.y = 2.07;
rover.add(scanPlane);

// ── CAMERAS (front) ──
const camGeo = new THREE.BoxGeometry(0.22, 0.16, 0.14);
for (let x of [-0.28, 0.28]) {
    const cam = new THREE.Mesh(camGeo, darkMat);
    cam.position.set(x, 1.15, -0.76);
    rover.add(cam);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.06, 12), glassMat);
    lens.rotation.x = Math.PI / 2;
    lens.position.set(x, 1.15, -0.83);
    rover.add(lens);
}

// ── ANTENNA ──
const antGeo = new THREE.CylinderGeometry(0.015, 0.015, 1.0, 6);
const ant = new THREE.Mesh(antGeo, wireMat);
ant.position.set(0.7, 2.05, 0.3);
rover.add(ant);
const antTip = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 8, 8),
    new THREE.MeshStandardMaterial({ color: 0xff6a00, emissive: 0xff6a00, emissiveIntensity: 1 })
);
antTip.position.set(0.7, 2.57, 0.3);
rover.add(antTip);

// ── WHEELS (4) ──
const wheelGeo = new THREE.CylinderGeometry(0.42, 0.42, 0.28, 20);
const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111820, metalness: 0.4, roughness: 0.8 });
const rimMat   = new THREE.MeshStandardMaterial({ color: 0x1a3a50, metalness: 0.8, roughness: 0.2 });
const wheelPositions = [
    [-1.75, 0.3, -1.02], [1.75, 0.3, -1.02],
    [-1.75, 0.3,  1.02], [1.75, 0.3,  1.02]
];
const wheels = [];
wheelPositions.forEach(pos => {
    const wg = new THREE.Group();
    const w = new THREE.Mesh(wheelGeo, wheelMat);
    w.rotation.z = Math.PI / 2;
    wg.add(w);
    const rim = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.3, 6), rimMat);
    rim.rotation.z = Math.PI / 2;
    wg.add(rim);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.31, 8), wireMat);
    hub.rotation.z = Math.PI / 2;
    wg.add(hub);
    const axleGeo = new THREE.BoxGeometry(0.3, 0.12, 0.12);
    const axle = new THREE.Mesh(axleGeo, darkMat);
    axle.position.x = pos[0] > 0 ? -0.15 : 0.15;
    wg.add(axle);
    wg.position.set(...pos);
    rover.add(wg);
    wheels.push(wg);
});

// ── SUSPENSION ARMS ──
const armGeo = new THREE.BoxGeometry(0.6, 0.1, 0.1);
[[-1.45, 0.5, -1.02], [1.45, 0.5, -1.02],
 [-1.45, 0.5,  1.02], [1.45, 0.5,  1.02]].forEach(pos => {
    const arm = new THREE.Mesh(armGeo, darkMat);
    arm.position.set(...pos);
    rover.add(arm);
});

// ── ORANGE ACCENT PANELS ──
const panelGeo = new THREE.BoxGeometry(2.8, 0.06, 0.06);
for (let z of [-0.98, 0.98]) {
    const p = new THREE.Mesh(panelGeo, accentMat);
    p.position.set(0, 0.16, z);
    rover.add(p);
}

// ── UNDERBODY ──
const underbody = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.12, 1.6), darkMat);
underbody.position.y = 0.13;
rover.add(underbody);

rover.position.y = -0.5;
rover.rotation.y = 0.4;

// ── PARTICLE FIELD ──
const particleCount = 600;
const pGeo = new THREE.BufferGeometry();
const positions = new Float32Array(particleCount * 3);
const particleSpeeds = [];
for (let i = 0; i < particleCount; i++) {
    positions[i * 3]     = (Math.random() - 0.5) * 60;
    positions[i * 3 + 1] = (Math.random() - 0.5) * 40;
    positions[i * 3 + 2] = (Math.random() - 0.5) * 40 - 10;
    particleSpeeds.push(Math.random() * 0.015 + 0.003);
}
pGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
const pMat = new THREE.PointsMaterial({ color: 0x00c8ff, size: 0.06, transparent: true, opacity: 0.5 });
const particles = new THREE.Points(pGeo, pMat);
scene.add(particles);

// ── GRID FLOOR ──
const gridHelper = new THREE.GridHelper(60, 60, 0x00c8ff, 0x001a2a);
gridHelper.position.y = -2.5;
gridHelper.material.opacity = 0.15;
gridHelper.material.transparent = true;
scene.add(gridHelper);

// ── ORBIT RING ──
const ringGeo = new THREE.TorusGeometry(5, 0.02, 8, 100);
const ringMat = new THREE.MeshBasicMaterial({ color: 0x00c8ff, transparent: true, opacity: 0.2 });
const orbitRing = new THREE.Mesh(ringGeo, ringMat);
orbitRing.rotation.x = Math.PI / 2.5;
scene.add(orbitRing);

const orbitDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.12, 8, 8),
    new THREE.MeshBasicMaterial({ color: 0x00c8ff })
);
scene.add(orbitDot);

// ── MOUSE PARALLAX ──
let targetRotY = 0, targetRotX = 0, currentRotY = 0.4, currentRotX = 0;
document.addEventListener('mousemove', e => {
    targetRotY = 0.4 + (e.clientX / window.innerWidth - 0.5) * 1.2;
    targetRotX = (e.clientY / window.innerHeight - 0.5) * 0.4;
});

// ── ANIMATE ──
let frame = 0;
function animate() {
    requestAnimationFrame(animate);
    frame++;

    currentRotY += (targetRotY - currentRotY) * 0.04;
    currentRotX += (targetRotX - currentRotX) * 0.04;
    rover.rotation.y = currentRotY;
    rover.rotation.x = currentRotX * 0.3;

    wheels.forEach(w => { w.rotation.x += 0.025; });

    lidar.rotation.y += 0.06;
    scanPlane.rotation.y += 0.06;

    if (frame % 40 < 20) antTip.material.emissiveIntensity = 1;
    else antTip.material.emissiveIntensity = 0.2;

    const orbitAngle = frame * 0.018;
    orbitDot.position.x = Math.cos(orbitAngle) * 5;
    orbitDot.position.z = Math.sin(orbitAngle) * 5 * Math.cos(Math.PI / 2.5);
    orbitDot.position.y = Math.sin(orbitAngle) * 5 * Math.sin(Math.PI / 2.5);

    rover.position.y = -0.5 + Math.sin(frame * 0.02) * 0.12;

    const pos = pGeo.attributes.position.array;
    for (let i = 0; i < particleCount; i++) {
        pos[i * 3 + 1] += particleSpeeds[i];
        if (pos[i * 3 + 1] > 20) pos[i * 3 + 1] = -20;
    }
    pGeo.attributes.position.needsUpdate = true;

    blueLight.intensity = 3 + Math.sin(frame * 0.03) * 0.8;

    renderer.render(scene, camera);
}
animate();

// ── RESIZE ──
window.addEventListener('resize', () => {
    const w = canvas.parentElement.clientWidth;
    const h = canvas.parentElement.clientHeight;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
});

// ─── SCROLL REVEAL ────────────────────────────────────────────────────────────
const reveals = document.querySelectorAll('.reveal');
const observer = new IntersectionObserver((entries) => {
    entries.forEach(e => { if (e.isIntersecting) e.target.classList.add('visible'); });
}, { threshold: 0.1 });
reveals.forEach(el => observer.observe(el));

// ─── BAR ANIMATION ────────────────────────────────────────────────────────────
const barObserver = new IntersectionObserver((entries) => {
    entries.forEach(e => {
        if (e.isIntersecting) {
            e.target.querySelectorAll('.bar-fill').forEach(bar => {
                const w = bar.getAttribute('data-width');
                bar.style.width = w + '%';
            });
        }
    });
}, { threshold: 0.3 });
document.querySelectorAll('.bar-chart').forEach(el => barObserver.observe(el));
