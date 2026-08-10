export class CameraOverlay {
  imgElem: HTMLImageElement;
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;

  constructor(imgId: string, canvasId: string) {
    this.imgElem = document.getElementById(imgId) as HTMLImageElement;
    this.canvas = document.getElementById(canvasId) as HTMLCanvasElement;
    this.ctx = this.canvas.getContext('2d')!;

    this.imgElem.onload = () => {
      this.syncCanvasSize();
    };
  }

  updateImage(jpegBlob: Blob) {
    const url = URL.createObjectURL(jpegBlob);
    this.imgElem.src = url;
  }

  private syncCanvasSize() {
    if (this.imgElem.clientWidth > 0 && this.imgElem.clientHeight > 0) {
      this.canvas.width = this.imgElem.clientWidth;
      this.canvas.height = this.imgElem.clientHeight;
    }
  }

  updateFeatures(featuresData: ArrayBuffer) {
    const view = new DataView(featuresData, 48);
    const count = view.getUint32(0, true);

    this.syncCanvasSize();
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    const origW = this.imgElem.naturalWidth || 848;
    const origH = this.imgElem.naturalHeight || 480;

    const scaleX = this.canvas.width / origW;
    const scaleY = this.canvas.height / origH;

    let offset = 4;
    for (let i = 0; i < count; i++) {
      const x = view.getFloat32(offset, true) * scaleX;
      const y = view.getFloat32(offset + 4, true) * scaleY;
      const st = view.getUint8(offset + 8);
      offset += 9;

      this.ctx.beginPath();
      this.ctx.arc(x, y, 2.5, 0, 2 * Math.PI);
      this.ctx.fillStyle = st === 3 ? '#ef4444' : st === 2 ? '#00ff88' : '#f59e0b';
      this.ctx.fill();
    }
  }
}
