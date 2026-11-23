#!/usr/bin/env python3
"""Jetson Wi-Fi monitoring example: send processed frames + publish health page.

Standalone camera test:
    python3 jetson_monitor.py --dest 10.42.0.123 --camera 0

Existing YOLO pipeline:
    from jetson_monitor import VideoSender, MonitorState, start_status_server
    state = MonitorState()
    start_status_server(state)
    sender = VideoSender('10.42.0.123')
    # in existing loop: sender.send(annotated_bgr_frame); state.update_detection(...)
"""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import re
import shutil
import subprocess
import threading
import time
from urllib.parse import urlparse

# OpenCV is imported only for standalone --camera mode or when resizing frames.
import cv2

WIDTH, HEIGHT, FPS = 640, 480, 15

DASHBOARD = """<!doctype html><html lang=\"ko\"><head><meta charset=\"utf-8\">
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
<title>Jetson Monitor</title><style>
body{background:#101827;color:#e5edf6;font-family:system-ui,sans-serif;padding:20px;max-width:740px;margin:auto}
h1{font-size:24px}.cards{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.card{background:#192638;border:1px solid #344357;border-radius:12px;padding:14px}
.label{color:#a6b6c9;font-size:13px}.value{font-size:21px;margin-top:4px;font-weight:700}
.ok{color:#79e3b4}.bad{color:#ffa4a4}small{color:#a6b6c9}
</style></head><body><h1>Jetson 상태 모니터</h1><p><small>2초마다 갱신 · 노트북에서 VLC로 영상 별도 수신</small></p>
<div class=\"cards\" id=\"cards\"></div><pre id=\"extra\"></pre>
<script>
const specs=[['영상 프레임 갱신','last_frame_age_s','초 전'],['송출 요청 FPS','input_fps',' FPS'],['YOLO 처리 FPS','yolo_fps',' FPS'],
['객체 감지 수','detection_count','개'],['추론 시간','inference_ms',' ms'],
['GPU 사용률 (GR3D)','gpu_percent','%'],['CPU 평균 사용률','cpu_percent','%'],
['RAM 사용량','ram_mb',' MB'],['CPU 온도','cpu_temp_c','°C']];
async function refresh(){try{let r=await fetch('/status',{cache:'no-store'});let d=await r.json();
document.getElementById('cards').innerHTML=specs.map(([n,k,s])=>
'<div class="card"><div class="label">'+n+'</div><div class="value">'+(d[k]??'—')+s+'</div></div>').join('');
document.getElementById('extra').textContent='처리 파이프라인: '+d.pipeline_state+' | 감지 라벨: '+(d.detection_label||'없음')+' | 서버 가동: '+d.uptime_s+'초';
}catch(e){document.getElementById('extra').textContent='연결 끊김: '+e;}}
refresh();setInterval(refresh,2000);
</script></body></html>"""


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.frames = deque(maxlen=40)
        self.last_frame = None
        self.detection_count = 0
        self.detection_label = ""
        self.inference_ms = None
        self.stats = {}

    def frame_sent(self):
        with self.lock:
            t = time.monotonic()
            self.last_frame = t
            self.frames.append(t)

    def update_detection(self, count=0, label="", inference_ms=None):
        """Call after each YOLO inference. Counts should reflect actual model output."""
        with self.lock:
            self.detection_count = int(count)
            self.detection_label = str(label)
            self.inference_ms = round(float(inference_ms), 1) if inference_ms is not None else None

    def update_stats(self, fields):
        with self.lock:
            self.stats.update(fields)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            age = None if self.last_frame is None else round(now - self.last_frame, 2)
            recent = [t for t in self.frames if now - t <= 3]
            fps = round((len(recent) - 1) / (recent[-1] - recent[0]), 1) if len(recent) >= 2 and recent[-1] > recent[0] else 0
            data = {
                "uptime_s": int(now - self.started), "last_frame_age_s": age,
                "input_fps": fps, "pipeline_state": "RUNNING" if age is not None and age <= 3 else "NO RECENT FRAME",
                "detection_count": self.detection_count,
                "detection_label": self.detection_label,
                "inference_ms": self.inference_ms,
            }
            data.update(self.stats)
            return data


def start_status_server(state, port=8765):
    """Read-only HTTP dashboard. Keep on a WPA2-protected local network."""
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/status":
                payload = json.dumps(state.snapshot(), ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif path == "/":
                payload = DASHBOARD.encode("utf-8")
                content_type = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format, *args):
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _extract_tegrastats(line):
    out = {}
    ram = re.search(r"\bRAM\s+(\d+)/(\d+)MB", line)
    gpu = re.search(r"\bGR3D_FREQ\s+(\d+)%", line)
    cpu_temp = re.search(r"\bcpu@([\d.]+)C", line)
    cpus = re.search(r"\bCPU\s+\[([^]]+)\]", line)
    if ram:
        out["ram_mb"] = int(ram.group(1))
        out["ram_total_mb"] = int(ram.group(2))
    if gpu:
        out["gpu_percent"] = int(gpu.group(1))
    if cpu_temp:
        out["cpu_temp_c"] = float(cpu_temp.group(1))
    if cpus:
        vals = [int(v) for v in re.findall(r"(\d+)%", cpus.group(1))]
        if vals:
            out["cpu_percent"] = round(sum(vals) / len(vals), 1)
    return out


def start_tegrastats(state):
    """Optional Jetson-only background stat reader; returns process or None."""
    if not shutil.which("tegrastats"):
        return None
    proc = subprocess.Popen(["tegrastats", "--interval", "1000"], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)
    def read_lines():
        for line in proc.stdout:
            state.update_stats(_extract_tegrastats(line))
    threading.Thread(target=read_lines, daemon=True).start()
    return proc


class VideoSender:
    """Keep only newest frame so low-quality wireless links do not stall inference.

    Caller provides 3-channel uint8 BGR NumPy frames (e.g. result.plot()).
    """
    def __init__(self, dest_ip, state=None, port=5600, width=WIDTH, height=HEIGHT,
                 fps=FPS, bitrate_k=1200):
        self.width, self.height = width, height
        self.state = state
        self.q = queue.Queue(maxsize=1)
        self.error = None
        dest = f"udp://{dest_ip}:{port}?pkt_size=1316"
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin", "-y",
            "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", f"{width}x{height}",
            "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p",
            "-b:v", f"{bitrate_k}k", "-maxrate", f"{bitrate_k}k", "-bufsize", f"{bitrate_k}k",
            "-g", str(fps), "-bf", "0", "-f", "mpegts", dest,
        ]
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE)
        self.thread = threading.Thread(target=self._writer, daemon=True)
        self.thread.start()

    def send(self, annotated_bgr):
        if self.error:
            raise RuntimeError(f"Video pipeline failed: {self.error}")
        if self.proc.poll() is not None:
            raise RuntimeError(f"ffmpeg exited: {self.proc.returncode}; check libx264 support")
        if annotated_bgr is None:
            return
        if len(annotated_bgr.shape) != 3 or annotated_bgr.shape[2] != 3:
            raise ValueError("Expected 3-channel BGR image (H,W,3)")
        frame = cv2.resize(annotated_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if not frame.flags["C_CONTIGUOUS"]:
            frame = frame.copy()
        if self.q.full():
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
        self.q.put_nowait(frame.tobytes())
        if self.state is not None:
            self.state.frame_sent()

    def _writer(self):
        while True:
            frame_bytes = self.q.get()
            if frame_bytes is None:
                return
            try:
                self.proc.stdin.write(frame_bytes)
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                self.error = str(exc)
                return

    def close(self):
        try:
            self.q.put_nowait(None)
        except queue.Full:
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
            self.q.put_nowait(None)
        self.thread.join(timeout=2)
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            self.proc.wait(timeout=2)


def run_demo(args):
    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError(f"Camera index {args.camera} could not be opened; try --camera 1 or check /dev/video*")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    state = MonitorState()
    server = start_status_server(state)
    stats = start_tegrastats(state)
    sender = VideoSender(args.dest, state, width=args.width, height=args.height,
                         fps=args.fps, bitrate_k=args.bitrate)
    print(f"Sending camera video to udp://{args.dest}:5600 | status http://<JETSON_IP>:8765")
    try:
        next_ts = time.monotonic()
        while True:
            ret, frame = camera.read()
            if not ret:
                state.update_detection(count=0, label="CAMERA READ FAILED")
                time.sleep(.1)
                continue
            sender.send(frame)  # Plain camera, no YOLO in demo mode
            next_ts += 1 / args.fps
            time.sleep(max(0, next_ts - time.monotonic()))
            if time.monotonic() - next_ts > 1:
                next_ts = time.monotonic()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        sender.close()
        server.shutdown()
        camera.release()
        if stats:
            stats.terminate()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Jetson camera/YOLO monitor over a local Wi-Fi link")
    p.add_argument("--dest", required=True, help="Laptop IPv4 on Jetson hotspot, e.g. 10.42.0.123")
    p.add_argument("--camera", type=int, default=0, help="OpenCV camera index (demo only)")
    p.add_argument("--width", type=int, default=WIDTH)
    p.add_argument("--height", type=int, default=HEIGHT)
    p.add_argument("--fps", type=int, default=FPS)
    p.add_argument("--bitrate", type=int, default=1200, help="H.264 target kbps")
    run_demo(p.parse_args())
