#!/usr/bin/env python3
"""Wrap the existing ROS YOLO detector with bounded, optional review recording.

No flight-command publishers are created. Box annotations use the result from
the same detector callback and frame, rather than an asynchronously paired box.
"""
import csv
import array
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import shutil
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool
from px4_msgs.msg import (
    VehicleStatus, VehicleLocalPosition, VehicleOdometry,
    ManualControlSetpoint, TrajectorySetpoint, OffboardControlMode,
    VehicleCommand, VehicleCommandAck,
)


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray, array.array)):
        return [clean(v) for v in value]
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def frame_stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def annotate(raw, box, label, age_ms):
    frame = raw.copy()
    h, w = frame.shape[:2]
    detected = box is not None and bool(box.get('detected', 0))
    if detected:
        x1 = max(0, min(w - 1, int(box['x1'])))
        y1 = max(0, min(h - 1, int(box['y1'])))
        x2 = max(0, min(w - 1, int(box['x2'])))
        y2 = max(0, min(h - 1, int(box['y2'])))
        color = (0, 255, 0) if box.get('lock_on') else (0, 220, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        details = (
            f"class={int(box['class_id'])} conf={box['conf']:.2f} "
            f"lock={bool(box['lock_on'])}"
        )
        cv2.putText(frame, details, (8, h - 34), cv2.FONT_HERSHEY_SIMPLEX, .46, color, 1)
        cv2.putText(frame, f"estimated distance={box['distance_m']:.2f}m", (8, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, .46, color, 1)
    else:
        cv2.putText(frame, 'NO DETECTION', (8, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 100, 255), 2)
    cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    cv2.putText(frame, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .44, (255, 255, 255), 1)
    cv2.putText(frame, f"ROS header age at callback={age_ms:.0f}ms", (8, 40),
                cv2.FONT_HERSHEY_SIMPLEX, .44, (255, 255, 255), 1)
    # YOLO_BROWSER_PREVIEW_V1
    try:
        from yolo_live_stream import push_frame
        push_frame(frame)
    except Exception:
        pass
    return frame


FRAME_FIELDS = (
    'segment', 'frame_index', 'wall_ns', 'elapsed_s', 'source_stamp_ns',
    'header_age_ms', 'callback_ms', 'video_fps', 'width', 'height',
    'detected', 'conf', 'class_id', 'x1', 'y1', 'x2', 'y2',
    'cx', 'cy', 'dx', 'dy', 'distance_m', 'near_2m', 'lock_on',
)


class Recorder:
    def __init__(self):
        root = Path(os.environ.get('FLIGHT_REVIEW_ROOT', str(Path.home() / 'flight_reviews')))
        root.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}'
        self.path = root / name
        self.path.mkdir()
        self.fps = min(10., max(1., float(os.environ.get('FLIGHT_REVIEW_FPS', '5'))))
        self.min_free = 2 * 1024**3
        self.max_duration = 2 * 60 * 60
        self.started = time.monotonic()
        self.next_frame = 0.
        self.frame_drops = 0
        self.event_drops = 0
        self.failure = None
        self.disabled = False
        self.frames = queue.Queue(maxsize=8)
        self.events = queue.Queue(maxsize=256)
        self.stopping = threading.Event()
        self.manifest = {
            'session': name, 'started_wall_ns': time.time_ns(),
            'video_fps': self.fps, 'segment_seconds': 30,
            'minimum_free_bytes': self.min_free,
            'maximum_session_seconds': self.max_duration,
            'detector_script': os.environ.get('FLIGHT_REVIEW_DETECTOR'),
            'detector_arguments': __import__('sys').argv[1:],
            'status': 'recording',
            'timing_note': (
                'AVI is constant-rate, sampled at most video_fps. Use frames.csv '
                'for actual time; do not infer elapsed time from video alone. '
                'Image ROS header is set by the camera publisher after capture; '
                'it is not a hardware exposure timestamp. Telemetry wall/ROS '
                'receive timestamps are distinct from PX4 embedded timestamps.'
            ),
            'limits': (
                'No automatic deletion. Recorder stops when disk is low, recording '
                'fails, or 2 hours pass; detector continues. Abrupt power loss may '
                'lose the current 30-second AVI segment and recent buffered data.'
            ),
        }
        self._manifest()
        self.worker = threading.Thread(target=self._run, name='flight-review-writer', daemon=True)
        self.worker.start()
        print(f'REVIEW_RECORDING: {self.path} (sampled {self.fps:g} FPS)', flush=True)

    def _manifest(self):
        temp = self.path / 'manifest.tmp'
        temp.write_text(json.dumps(clean(self.manifest), indent=2, ensure_ascii=False))
        temp.replace(self.path / 'manifest.json')

    def event(self, topic, data, ros_ns=None, source_ns=None):
        if self.disabled or self.stopping.is_set():
            return
        item = {
            'wall_ns': time.time_ns(), 'elapsed_s': time.monotonic() - self.started,
            'ros_receive_ns': ros_ns, 'image_source_stamp_ns': source_ns,
            'topic': topic, 'data': clean(data),
        }
        try:
            self.events.put_nowait(item)
        except queue.Full:
            self.event_drops += 1

    def frame(self, raw, box, source_ns, ros_ns, wall_ns, callback_ms):
        if self.disabled or self.stopping.is_set():
            return
        now = time.monotonic()
        if now < self.next_frame:
            return
        self.next_frame = now + 1. / self.fps
        item = (raw, clean(box), source_ns, ros_ns, wall_ns,
                now - self.started, callback_ms)
        try:
            self.frames.put_nowait(item)
        except queue.Full:
            self.frame_drops += 1

    def _run(self):
        raw_writer = overlay_writer = None
        event_file = frame_file = None
        frame_count = segment_count = 0
        segment_start = None
        shape = None
        last_flush = last_disk = 0.
        segment = ''
        index = 0
        error = None
        try:
            event_file = (self.path / 'telemetry.jsonl').open('w', buffering=1)
            frame_file = (self.path / 'frames.csv').open('w', newline='', buffering=1)
            rows = csv.DictWriter(frame_file, fieldnames=FRAME_FIELDS)
            rows.writeheader()
            while not self.stopping.is_set() or not self.frames.empty() or not self.events.empty():
                now = time.monotonic()
                if now - last_disk > 1.:
                    if shutil.disk_usage(self.path).free < self.min_free:
                        raise RuntimeError('less than 2 GiB disk space remains')
                    if now - self.started > self.max_duration:
                        raise RuntimeError('2-hour recording limit reached')
                    last_disk = now
                # Limit each drain so a telemetry burst cannot starve videos.
                for _ in range(64):
                    try:
                        event = self.events.get_nowait()
                    except queue.Empty:
                        break
                    event_file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
                try:
                    item = self.frames.get(timeout=.05)
                except queue.Empty:
                    item = None
                if item is not None:
                    raw, box, source_ns, ros_ns, wall_ns, elapsed, callback_ms = item
                    h, w = raw.shape[:2]
                    if shape != (w, h) or segment_start is None or elapsed - segment_start >= 30.:
                        if raw_writer is not None:
                            raw_writer.release()
                            overlay_writer.release()
                        segment_count += 1
                        segment = f'{segment_count:04d}'
                        index = 0
                        shape = (w, h)
                        segment_start = elapsed
                        codec = cv2.VideoWriter_fourcc(*'MJPG')
                        raw_writer = cv2.VideoWriter(str(self.path / f'raw_{segment}.avi'), codec, self.fps, shape)
                        overlay_writer = cv2.VideoWriter(str(self.path / f'yolo_{segment}.avi'), codec, self.fps, shape)
                        if not raw_writer.isOpened() or not overlay_writer.isOpened():
                            raise RuntimeError('OpenCV MJPG AVI writer could not open')
                    age_ms = (ros_ns - source_ns) / 1e6
                    label = f'{datetime.fromtimestamp(wall_ns / 1e9):%Y-%m-%d %H:%M:%S} t={elapsed:.2f}s'
                    overlay = annotate(raw, box, label, age_ms)
                    raw_writer.write(raw)
                    overlay_writer.write(overlay)
                    row = {key: box.get(key) for key in FRAME_FIELDS} if box else {}
                    row.update(segment=segment, frame_index=index, wall_ns=wall_ns,
                               elapsed_s=elapsed, source_stamp_ns=source_ns,
                               header_age_ms=age_ms, callback_ms=callback_ms,
                               video_fps=self.fps, width=w, height=h)
                    rows.writerow(row)
                    index += 1
                    frame_count += 1
                if now - last_flush >= 1.:
                    for stream in (event_file, frame_file):
                        stream.flush()
                        os.fsync(stream.fileno())
                    self.manifest.update(frames_written=frame_count, segments=segment_count,
                                         frame_queue_drops=self.frame_drops,
                                         event_queue_drops=self.event_drops)
                    self._manifest()
                    last_flush = now
        except Exception as exc:
            error = str(exc)
            self.failure = error
            self.disabled = True
            print(f'REVIEW_RECORDING_STOPPED: {error}; detector continues', flush=True)
        finally:
            for writer in (raw_writer, overlay_writer):
                if writer is not None:
                    writer.release()
            for stream in (event_file, frame_file):
                if stream is not None:
                    stream.flush()
                    os.fsync(stream.fileno())
                    stream.close()
            self.manifest.update(
                status='error' if error else 'closed', error=error,
                ended_wall_ns=time.time_ns(), frames_written=frame_count,
                segments=segment_count, frame_queue_drops=self.frame_drops,
                event_queue_drops=self.event_drops,
            )
            try:
                self._manifest()
            except Exception as exc:
                print(f'REVIEW_MANIFEST_ERROR: {exc}', flush=True)

    def close(self):
        self.stopping.set()
        self.worker.join(timeout=15.)
        if self.worker.is_alive():
            print('REVIEW_CLOSE_TIMEOUT: current files may be incomplete', flush=True)
        else:
            print(f'REVIEW_SAVED: {self.path}', flush=True)


def load_detector(path):
    spec = importlib.util.spec_from_file_location('review_original_detector', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    detector = os.environ.get('FLIGHT_REVIEW_DETECTOR',
                              str(Path.home() / 'drone_project/interceptor_drone/tools/yolo_live_detect.py'))
    module = load_detector(detector)
    original = module.YoloLiveDetector
    recorders = []

    class RecordingDetector(original):
        def __init__(self, args):
            super().__init__(args)
            self._review = None
            self._review_frame = None
            self._review_box = None
            self._review_ros_ns = None
            self._review_source_ns = None
            self._review_subs = []
            self._review_last = {}
            try:
                self._review = Recorder()
                recorders.append(self._review)
                self._attach_telemetry()
            except Exception as exc:
                print(f'REVIEW_INIT_ERROR: {exc}; detector continues', flush=True)

        def _attach_telemetry(self):
            px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=10,
            )
            topics = (
                ('/fmu/out/vehicle_status_v1', VehicleStatus, .2),
                ('/fmu/out/vehicle_local_position', VehicleLocalPosition, .1),
                ('/fmu/out/vehicle_odometry', VehicleOdometry, .1),
                ('/fmu/out/manual_control_setpoint', ManualControlSetpoint, .1),
                ('/fmu/out/vehicle_command_ack', VehicleCommandAck, 0.),
                ('/fmu/in/trajectory_setpoint', TrajectorySetpoint, 0.),
                ('/fmu/in/offboard_control_mode', OffboardControlMode, .1),
                ('/fmu/in/vehicle_command', VehicleCommand, 0.),
            )
            for topic, kind, interval in topics:
                self._review_subs.append(self.create_subscription(
                    kind, topic,
                    lambda msg, t=topic, dt=interval: self._telemetry(t, msg, dt),
                    px4_qos,
                ))
            for topic in ('/mission/start', '/mission/stop'):
                self._review_subs.append(self.create_subscription(
                    Bool, topic, lambda msg, t=topic: self._telemetry(t, msg, 0.), 10,
                ))
            self._review_subs.append(self.create_subscription(Log, '/rosout', self._mission_log, 50))

        def _telemetry(self, topic, msg, interval):
            if self._review is None:
                return
            now = time.monotonic()
            if now - self._review_last.get(topic, -math.inf) < interval:
                return
            self._review_last[topic] = now
            try:
                self._review.event(topic, message_to_ordereddict(msg),
                                   self.get_clock().now().nanoseconds)
            except Exception as exc:
                print(f'REVIEW_TELEMETRY_ERROR: {topic}: {exc}', flush=True)

        def _mission_log(self, msg):
            if msg.name == 'intercept':
                self._telemetry('/rosout/intercept', msg, 0.)

        def ros_image_to_bgr(self, msg):
            frame = super().ros_image_to_bgr(msg)
            if self._review is not None and not self._review.disabled:
                self._review_frame = frame.copy()
            return frame

        def publish_bbox(self, *args, **kwargs):
            result = super().publish_bbox(*args, **kwargs)
            # Match the original signature, preserving the exact accepted result.
            import inspect
            values = inspect.signature(original.publish_bbox).bind(self, *args, **kwargs).arguments
            self._review_box = {k: v for k, v in values.items() if k != 'self'}
            self._review_detection()
            return result

        def publish_no_detection(self, image_w, image_h):
            result = super().publish_no_detection(image_w, image_h)
            self._review_box = dict(detected=0., conf=0., class_id=-1,
                                    image_w=image_w, image_h=image_h,
                                    distance_m=-1., near_2m=0., lock_on=0.)
            self._review_detection()
            return result

        def _review_detection(self):
            if self._review is not None:
                self._review.event('/target/balloon_bbox', self._review_box,
                                   self._review_ros_ns, self._review_source_ns)

        def _image_callback_impl(self, msg):
            self._review_frame = self._review_box = None
            self._review_ros_ns = self.get_clock().now().nanoseconds
            self._review_source_ns = frame_stamp_ns(msg)
            wall_ns = time.time_ns()
            start = time.monotonic()
            try:
                return super()._image_callback_impl(msg)
            finally:
                if (self._review is not None and self._review_frame is not None
                        and self._review_box is not None):
                    try:
                        self._review.frame(self._review_frame, self._review_box,
                                           self._review_source_ns, self._review_ros_ns,
                                           wall_ns, (time.monotonic() - start) * 1000.)
                    except Exception as exc:
                        print(f'REVIEW_FRAME_ERROR: {exc}', flush=True)
                self._review_frame = None

    module.YoloLiveDetector = RecordingDetector
    try:
        module.main()
    finally:
        for recorder in recorders:
            recorder.close()


if __name__ == '__main__':
    main()
