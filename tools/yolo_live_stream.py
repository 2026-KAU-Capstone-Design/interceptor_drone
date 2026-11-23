"""Optional localhost preview; no ROS publishers or camera access."""
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
_q = queue.Queue(maxsize=1)
_jpeg = None
_lock = threading.Lock()
_last = 0.0
_started = False
_disabled = False

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass
    def do_GET(self):
        if self.path == '/':
            body = b'''<!doctype html><meta charset="utf-8"><title>YOLO live preview</title><style>body{background:#111;color:white;font:18px sans-serif}img{max-width:95vw}#s{padding:12px}</style><h2>YOLO live preview (up to 3 FPS)</h2><div id="s">Waiting for detector frames...</div><img id="v"><script>async function tick(){try{let r=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store'});if(!r.ok)throw Error('Waiting for frames');let age=Number(r.headers.get('X-Frame-Age'));if(age>2)throw Error('STALE: '+age.toFixed(1)+' seconds');let b=await r.blob();let im=document.getElementById('v'),old=im.src;im.src=URL.createObjectURL(b);if(old.startsWith('blob:'))URL.revokeObjectURL(old);im.style.display='block';document.getElementById('s').textContent='Frame age at Jetson: '+age.toFixed(2)+' s (network delay excluded)';}catch(e){document.getElementById('v').style.display='none';document.getElementById('s').textContent=e.message;}setTimeout(tick,350);}tick();</script>'''
            self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8')
        elif self.path.startswith('/frame.jpg'):
            with _lock:
                item = _jpeg
            if item is None:
                self.send_error(503, 'No detector frames yet'); return
            stamp, body = item
            self.send_response(200); self.send_header('Content-Type','image/jpeg')
            self.send_header('X-Frame-Age', str(time.monotonic()-stamp))
        else:
            self.send_error(404); return
        self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(body))); self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

def encoder():
    global _jpeg
    while True:
        stamp, frame = _q.get()
        try:
            ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY,70])
            if ok:
                with _lock:
                    _jpeg = (stamp, data.tobytes())
        except Exception:
            pass

def push_frame(frame):
    global _started, _disabled, _last
    try:
        if _disabled: return
        if not _started:
            try:
                server = ThreadingHTTPServer(('127.0.0.1',8765),Handler)
            except OSError as e:
                print('LIVE_PREVIEW_DISABLED:',e,flush=True); _disabled=True; return
            threading.Thread(target=server.serve_forever,daemon=True).start()
            threading.Thread(target=encoder,daemon=True).start()
            _started=True
            print('LIVE_PREVIEW_READY: http://127.0.0.1:8765',flush=True)
        now=time.monotonic()
        if now-_last < 1/3: return
        _last=now
        try: _q.put_nowait((now,frame.copy()))
        except queue.Full: pass
    except Exception:
        pass
