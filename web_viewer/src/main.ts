import { ThreeRenderer } from './renderer';
import { CameraOverlay } from './overlay';
import { parseWireHeader, parseFrameState, parsePointChunk } from './protocol';

const canvas = document.getElementById('webgl-canvas') as HTMLCanvasElement;
const renderer = new ThreeRenderer(canvas);
const overlay = new CameraOverlay('camera-img', 'overlay-canvas');

const statStatus = document.getElementById('stat-status')!;
const statPose = document.getElementById('stat-pose')!;
const statKp = document.getElementById('stat-kp')!;
const statPoints = document.getElementById('stat-points')!;

// Connect Realtime WebSocket
const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const host = window.location.host || '127.0.0.1:8080';

const wsRealtime = new WebSocket(`${protocol}//${host}/ws/realtime`);
wsRealtime.binaryType = 'arraybuffer';

wsRealtime.onopen = () => {
  statStatus.textContent = 'Connected (Realtime)';
  statStatus.style.color = '#4ade80';
};

wsRealtime.onmessage = (event) => {
  const buf = event.data as ArrayBuffer;
  const header = parseWireHeader(buf);
  if (!header) return;

  if (header.msgType === 2) { // MSG_FRAME_STATE
    const frame = parseFrameState(buf);
    if (frame) {
      renderer.updateFrame(frame);
      statPose.textContent = frame.poseValid ? `(${frame.tx.toFixed(2)}, ${frame.ty.toFixed(2)}, ${frame.tz.toFixed(2)})` : 'Invalid';
      statKp.textContent = frame.trackedKp.toString();
    }
  }
};

wsRealtime.onclose = () => {
  statStatus.textContent = 'Disconnected';
  statStatus.style.color = '#f87171';
};

// Connect Bulk WebSocket
const wsBulk = new WebSocket(`${protocol}//${host}/ws/bulk`);
wsBulk.binaryType = 'arraybuffer';

wsBulk.onmessage = (event) => {
  const buf = event.data as ArrayBuffer;
  const header = parseWireHeader(buf);
  if (!header) return;

  if (header.msgType === 31) { // MSG_POINTS_FULL_CHUNK
    const chunk = parsePointChunk(buf);
    if (chunk) {
      renderer.updatePoints(chunk.points);
      statPoints.textContent = chunk.count.toString();
    }
  } else if (header.msgType === 40) { // MSG_IMAGE_JPEG
    const jpegData = buf.slice(48);
    const blob = new Blob([jpegData], { type: 'image/jpeg' });
    overlay.updateImage(blob);
  } else if (header.msgType === 41) { // MSG_FEATURE_OVERLAY
    overlay.updateFeatures(buf);
  }
};

// Buttons
document.getElementById('btn-follow')?.addEventListener('click', (e) => {
  const btn = e.target as HTMLButtonElement;
  renderer.followCamera = !renderer.followCamera;
  btn.classList.toggle('active', renderer.followCamera);
});

document.getElementById('btn-top')?.addEventListener('click', () => {
  renderer.followCamera = false;
  document.getElementById('btn-follow')?.classList.remove('active');
  renderer.camera.position.set(0, 10, 0);
  renderer.camera.lookAt(0, 0, 0);
});

document.getElementById('btn-reset')?.addEventListener('click', () => {
  renderer.followCamera = true;
  document.getElementById('btn-follow')?.classList.add('active');
});

document.getElementById('camera-toggle')?.addEventListener('click', () => {
  const panel = document.getElementById('camera-panel');
  panel?.classList.toggle('minimized');
});
