import sqlite3
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir = Path("missions/mission2_nav/rosbags/run_no_wind")
db_path = bag_dir / "run_no_wind_0.db3"

topic_name = "/fmu/out/vehicle_local_position"
msg_type = get_message("px4_msgs/msg/VehicleLocalPosition")

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
topic_id = cursor.fetchone()[0]

cursor.execute(
    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
    (topic_id,)
)

times, xs, ys, zs = [], [], [], []

for timestamp, data in cursor.fetchall():
    msg = deserialize_message(data, msg_type)
    times.append(timestamp * 1e-9)
    xs.append(msg.x)
    ys.append(msg.y)
    zs.append(msg.z)

conn.close()

times = np.array(times)
xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)
times = times - times[0]
altitudes = -zs

# A, B 목표점
x_a = xs[0]
y_a = ys[0]
x_b = x_a + 20.0
y_b = y_a

# B 도착 오차: B에 가장 가까웠던 지점 기준
dist_to_b = np.sqrt((xs - x_b) ** 2 + (ys - y_b) ** 2)
b_error = np.min(dist_to_b)
b_index = np.argmin(dist_to_b)

# 경로 편차: A→B 구간에서 y=0 직선으로부터 얼마나 벗어났는지
path_mask = (xs >= x_a) & (xs <= x_b) & (altitudes > 4.0)

if np.any(path_mask):
    path_deviation = np.abs(ys[path_mask] - y_a)
    max_path_deviation = np.max(path_deviation)
else:
    max_path_deviation = float("nan")

b_pass = b_error < 0.5
path_pass = max_path_deviation < 0.3

result = f"""Mission 2 A-to- Navigation Analysis

Bag:
{bag_dir}

Total samples:
{len(xs)}

Target A:
x = {x_a:.3f}, y = {y_a:.3f}

Target B:
x = {x_b:.3f}, y = {y_b:.3f}

Closest B position:
x = {xs[b_index]:.3f}, y = {ys[b_index]:.3f}

B arrival error:
{b_error:.3f} m
Criteria: < 0.5 m
Result: {"PASS" if b_pass else "FAIL"}

Path deviation:
max_y_deviation = {max_path_deviation:.3f} m
Criteria: < 0.3 m
Result: {"PASS" if path_pass else "FAIL"}
"""

print(result)

Path("missions/mission2_nav/analysis/result.txt").write_text(result, encoding="utf-8")

# 그래프 대신 CSV 저장
csv_path = Path("missions/mission2_nav/analysis/path_data.csv")
with csv_path.open("w", encoding="utf-8") as f:
    f.write("time,x,y,z,altitude\n")
    for t, x, y, z, alt in zip(times, xs, ys, zs, altitudes):
        f.write(f"{t:.3f},{x:.6f},{y:.6f},{z:.6f},{alt:.6f}\n")

print("Saved result.txt")
print("Saved path_data.csv")
