import sqlite3
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir = Path("missions/mission1_hover/rosbags/run_no_wind")
db_path = bag_dir / "run_no_wind_0.db3"

topic_name = "/fmu/out/vehicle_local_position"
msg_type = get_message("px4_msgs/msg/VehicleLocalPosition")

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
row = cursor.fetchone()

if row is None:
    raise RuntimeError(f"Topic not found: {topic_name}")

topic_id = row[0]

cursor.execute(
    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
    (topic_id,)
)

times = []
xs = []
ys = []
zs = []

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

# PX4 NED 좌표계: 위로 올라가면 z가 음수
altitudes = -zs

# 5m 근처를 호버 구간으로 판단
hover_mask = (altitudes >= 4.5) & (altitudes <= 5.5)

hover_x = xs[hover_mask]
hover_y = ys[hover_mask]

x_std = np.std(hover_x)
y_std = np.std(hover_y)
hover_drift_std = math.sqrt(x_std**2 + y_std**2)

# 시작 위치: 처음 1초 평균
start_mask = times <= 1.0
x_start = np.mean(xs[start_mask])
y_start = np.mean(ys[start_mask])

# 착륙 위치: 마지막 1초 평균
end_mask = times >= (times[-1] - 1.0)
x_land = np.mean(xs[end_mask])
y_land = np.mean(ys[end_mask])

landing_error = math.sqrt((x_land - x_start)**2 + (y_land - y_start)**2)

landing_pass = landing_error < 0.3
hover_pass = hover_drift_std < 0.2

result = f"""Mission 1 Hover/Landing Analysis

Bag:
{bag_dir}

Total samples:
{len(xs)}

Max altitude:
{np.max(altitudes):.3f} m

Hover samples near 5m:
{len(hover_x)}

Landing error:
{landing_error:.3f} m
Criteria: < 0.3 m
Result: {"PASS" if landing_pass else "FAIL"}

Hover drift std:
x_std = {x_std:.3f} m
y_std = {y_std:.3f} m
combined_std = {hover_drift_std:.3f} m
Criteria: < 0.2 m
Result: {"PASS" if hover_pass else "FAIL"}

Start position:
x = {x_start:.3f}, y = {y_start:.3f}

Landing position:
x = {x_land:.3f}, y = {y_land:.3f}
"""

print(result)

Path("missions/mission1_hover/analysis/result.txt").write_text(result, encoding="utf-8")
