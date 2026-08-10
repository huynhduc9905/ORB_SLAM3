import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { FrameState } from './protocol';

export class ThreeRenderer {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  controls: OrbitControls;
  cameraFrustum: THREE.LineSegments;
  pointCloud: THREE.Points;
  pointGeometry: THREE.BufferGeometry;
  trailLine: THREE.Line;
  trailPositions: number[] = [];

  followCamera: boolean = true;

  constructor(canvas: HTMLCanvasElement) {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f172a);

    this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 1000);
    this.camera.position.set(0, 3, 5);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;

    // Grid removed as requested

    // Camera Frustum
    const frustumGeo = new THREE.BufferGeometry();
    const frustumPts = new Float32Array([
      0,0,0, -0.2,-0.15,-0.3,  0,0,0, 0.2,-0.15,-0.3,
      0,0,0, 0.2,0.15,-0.3,   0,0,0, -0.2,0.15,-0.3,
      -0.2,-0.15,-0.3, 0.2,-0.15,-0.3,  0.2,-0.15,-0.3, 0.2,0.15,-0.3,
      0.2,0.15,-0.3, -0.2,0.15,-0.3,   -0.2,0.15,-0.3, -0.2,-0.15,-0.3
    ]);
    frustumGeo.setAttribute('position', new THREE.BufferAttribute(frustumPts, 3));
    this.cameraFrustum = new THREE.LineSegments(frustumGeo, new THREE.LineBasicMaterial({ color: 0x38bdf8 }));
    this.scene.add(this.cameraFrustum);

    // Point Cloud
    this.pointGeometry = new THREE.BufferGeometry();
    const pointMat = new THREE.PointsMaterial({ color: 0x00ff88, size: 0.04, sizeAttenuation: true });
    this.pointCloud = new THREE.Points(this.pointGeometry, pointMat);
    this.scene.add(this.pointCloud);

    // Trail Line
    const trailGeo = new THREE.BufferGeometry();
    this.trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xf59e0b }));
    this.scene.add(this.trailLine);

    window.addEventListener('resize', () => this.onWindowResize());
    this.animate();
  }

  updateFrame(frame: FrameState) {
    if (!frame.poseValid) return;

    // Coordinate conversion: (x, y, z)_three = (x, -y, -z)_orb
    const tx = frame.tx;
    const ty = -frame.ty;
    const tz = -frame.tz;

    this.cameraFrustum.position.set(tx, ty, tz);
    this.cameraFrustum.quaternion.set(frame.qx, -frame.qy, -frame.qz, frame.qw);

    if (this.followCamera) {
      this.camera.position.set(tx, ty + 2, tz + 4);
      this.controls.target.set(tx, ty, tz);
      this.controls.update();
    }

    // Add to trail
    this.trailPositions.push(tx, ty, tz);
    if (this.trailPositions.length > 6000) this.trailPositions.splice(0, 3);
    this.trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(this.trailPositions, 3));
  }

  updatePoints(points: Float32Array) {
    this.pointGeometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    this.pointGeometry.attributes.position.needsUpdate = true;
    this.pointGeometry.computeBoundingSphere();
  }

  private onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  private animate() {
    requestAnimationFrame(() => this.animate());
    if (!this.followCamera) {
      this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
  }
}
