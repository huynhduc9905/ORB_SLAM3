export interface WireHeader {
  magic: number;
  version: number;
  msgType: number;
  flags: number;
  payloadSize: number;
  epoch: bigint;
  revision: bigint;
  sequence: bigint;
  timestampNs: bigint;
}

export interface FrameState {
  mapId: bigint;
  refKfId: bigint;
  trackingState: number;
  poseValid: boolean;
  tx: number; ty: number; tz: number;
  qx: number; qy: number; qz: number; qw: number;
  trackedKp: number;
  trackedMp: number;
}

export interface PointChunk {
  ox: number; oy: number; oz: number;
  scale: number;
  count: number;
  points: Float32Array; // x, y, z triplets
}

export function parseWireHeader(buffer: ArrayBuffer): WireHeader | null {
  if (buffer.byteLength < 48) return null;
  const view = new DataView(buffer);
  const magic = view.getUint32(0, true);
  if (magic !== 0x4f524257) return null;

  return {
    magic,
    version: view.getUint16(4, true),
    msgType: view.getUint16(6, true),
    flags: view.getUint32(8, true),
    payloadSize: view.getUint32(12, true),
    epoch: view.getBigUint64(16, true),
    revision: view.getBigUint64(24, true),
    sequence: view.getBigUint64(32, true),
    timestampNs: view.getBigInt64(40, true),
  };
}

export function parseFrameState(buffer: ArrayBuffer): FrameState | null {
  if (buffer.byteLength < 48 + 57) return null;
  const view = new DataView(buffer, 48);
  return {
    mapId: view.getBigUint64(0, true),
    refKfId: view.getBigUint64(8, true),
    trackingState: view.getInt32(16, true),
    poseValid: view.getUint8(20) === 1,
    tx: view.getFloat32(21, true),
    ty: view.getFloat32(25, true),
    tz: view.getFloat32(29, true),
    qx: view.getFloat32(33, true),
    qy: view.getFloat32(37, true),
    qz: view.getFloat32(41, true),
    qw: view.getFloat32(45, true),
    trackedKp: view.getUint32(49, true),
    trackedMp: view.getUint32(53, true),
  };
}

export function parsePointChunk(buffer: ArrayBuffer): PointChunk | null {
  if (buffer.byteLength < 48 + 20) return null;
  const view = new DataView(buffer, 48);
  const ox = view.getFloat32(0, true);
  const oy = view.getFloat32(4, true);
  const oz = view.getFloat32(8, true);
  const scale = view.getFloat32(12, true);
  const count = view.getUint32(16, true);

  const points = new Float32Array(count * 3);
  let offset = 20;
  for (let i = 0; i < count; i++) {
    const id = view.getUint32(offset, true);
    const dx = view.getInt16(offset + 4, true);
    const dy = view.getInt16(offset + 6, true);
    const dz = view.getInt16(offset + 8, true);

    // Coordinate conversion: (x, y, z)_three = (x, -y, -z)_orb
    const wx = ox + dx * scale;
    const wy = oy + dy * scale;
    const wz = oz + dz * scale;

    points[i * 3 + 0] = wx;
    points[i * 3 + 1] = -wy;
    points[i * 3 + 2] = -wz;

    offset += 12;
  }

  return { ox, oy, oz, scale, count, points };
}
