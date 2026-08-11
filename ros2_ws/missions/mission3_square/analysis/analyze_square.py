import sqlite3
from pathlib import Path
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_NAME = "run_windy"  # 유풍 분석할 때는 "run_windy"로 바꾸면 됨

bag_dir = Path(f"missions/mission3_square/rosbags/{BAG_NAME}")
db_path = bag_dir / f"{BAG_NAME}_0.db3"

analysis_dir = Path("missions/mission3_square/analysis")
analysis_dir.mkdir(parents=True, exist_ok=True)

topic_name = "/fmu/out/vehicle_local_position"
msg_type = get_message("px4_msgs/msg/VehicleLocalPosition")


def point_line_distance(px, py, ax, ay, bx, by):
    line_vec = np.array([bx - ax, by - ay])
    point_vec = np.column_stack([px - ax, py - ay])
    line_len = np.linalg.norm(line_vec)

    if line_len == 0:
        return np.zeros_like(px)

    return np.abs(
        line_vec[0] * point_vec[:, 1] - line_vec[1] * point_vec[:, 0]
    ) / line_len


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

flight_mask = altitudes > 4.0
flight_indices = np.where(flight_mask)[0]

if len(flight_indices) == 0:
    raise RuntimeError("No flight data above 4m altitude.")

start_idx = flight_indices[0]
end_idx = flight_indices[-1]

x_a = xs[start_idx]
y_a = ys[start_idx]

targets = {
    "B": np.array([x_a + 10.0, y_a]),
    "C": np.array([x_a + 10.0, y_a + 10.0]),
    "D": np.array([x_a, y_a + 10.0]),
    "A_return": np.array([x_a, y_a]),
}

positions = np.column_stack([xs, ys])

arrival_indices = {}
search_start = start_idx

for name in ["B", "C", "D", "A_return"]:
    target = targets[name]
    candidate_indices = np.arange(search_start, end_idx + 1)
    candidate_pos = positions[candidate_indices]

    distances = np.linalg.norm(candidate_pos - target, axis=1)
    local_min_idx = np.argmin(distances)

    arrival_idx = candidate_indices[local_min_idx]
    arrival_indices[name] = arrival_idx
    search_start = arrival_idx + 1

corner_errors = {
    name: float(np.linalg.norm(positions[idx] - targets[name]))
    for name, idx in arrival_indices.items()
}

avg_corner_error = float(np.mean(list(corner_errors.values())))

segments = {
    "A->B": (start_idx, arrival_indices["B"], np.array([x_a, y_a]), targets["B"]),
    "B->C": (arrival_indices["B"], arrival_indices["C"], targets["B"], targets["C"]),
    "C->D": (arrival_indices["C"], arrival_indices["D"], targets["C"], targets["D"]),
    "D->A": (arrival_indices["D"], arrival_indices["A_return"], targets["D"], targets["A_return"]),
}

edge_deviations = {}
cumulative_path_error = 0.0

for name, (i0, i1, p0, p1) in segments.items():
    if i1 <= i0:
        edge_deviations[name] = float("nan")
        continue

    seg_x = xs[i0:i1 + 1]
    seg_y = ys[i0:i1 + 1]
    seg_t = times[i0:i1 + 1]

    dists = point_line_distance(
        seg_x,
        seg_y,
        p0[0],
        p0[1],
        p1[0],
        p1[1],
    )

    edge_deviations[name] = float(np.max(dists))

    # 누적 경로 오차: 직선으로부터의 수직거리 적분
    cumulative_path_error += float(np.trapz(dists, seg_t))

max_edge_deviation = float(np.nanmax(list(edge_deviations.values())))
flight_time = float(times[end_idx] - times[start_idx])

corner_pass = avg_corner_error < 0.5
edge_pass = max_edge_deviation < 0.5

result = f"""Mission 3 Square Flight Analysis

Bag:
{bag_dir}

Total samples:
{len(xs)}

Flight time above 4m:
{flight_time:.3f} s

Target points:
A = ({x_a:.3f}, {y_a:.3f})
B = ({targets["B"][0]:.3f}, {targets["B"][1]:.3f})
C = ({targets["C"][0]:.3f}, {targets["C"][1]:.3f})
D = ({targets["D"][0]:.3f}, {targets["D"][1]:.3f})

Corner errors:
B        = {corner_errors["B"]:.3f} m
C        = {corner_errors["C"]:.3f} m
D        = {corner_errors["D"]:.3f} m
A_return = {corner_errors["A_return"]:.3f} m

Average corner error:
{avg_corner_error:.3f} m
Criteria: < 0.5 m
Result: {"PASS" if corner_pass else "FAIL"}

Edge straightness max deviation:
A->B = {edge_deviations["A->B"]:.3f} m
B->C = {edge_deviations["B->C"]:.3f} m
C->D = {edge_deviations["C->D"]:.3f} m
D->A = {edge_deviations["D->A"]:.3f} m

Max edge deviation:
{max_edge_deviation:.3f} m
Criteria: < 0.5 m
Result: {"PASS" if edge_pass else "FAIL"}

Cumulative path error:
{cumulative_path_error:.3f} m*s
"""

print(result)

(analysis_dir / f"result_{BAG_NAME}.txt").write_text(result, encoding="utf-8")
(analysis_dir / "result.txt").write_text(result, encoding="utf-8")

with (analysis_dir / f"corner_errors_{BAG_NAME}.csv").open("w", encoding="utf-8") as f:
    f.write("point,error_m\n")
    for name, err in corner_errors.items():
        f.write(f"{name},{err:.6f}\n")
    f.write(f"average,{avg_corner_error:.6f}\n")

with (analysis_dir / f"path_data_{BAG_NAME}.csv").open("w", encoding="utf-8") as f:
    f.write("time,x,y,z,altitude\n")
    for t, x, y, z, alt in zip(times, xs, ys, zs, altitudes):
        f.write(f"{t:.3f},{x:.6f},{y:.6f},{z:.6f},{alt:.6f}\n")


# matplotlib 없이 SVG 직접 생성
svg_path = analysis_dir / f"square_path_{BAG_NAME}.svg"

min_x = min(np.min(xs), x_a) - 2
max_x = max(np.max(xs), x_a + 10.0) + 2
min_y = min(np.min(ys), y_a) - 2
max_y = max(np.max(ys), y_a + 10.0) + 2

width = 800
height = 800
padding = 60

def map_x(x):
    return padding + (x - min_x) / (max_x - min_x) * (width - 2 * padding)

def map_y(y):
    return height - padding - (y - min_y) / (max_y - min_y) * (height - 2 * padding)

actual_points = " ".join(
    f"{map_x(x):.2f},{map_y(y):.2f}"
    for x, y in zip(xs[flight_mask], ys[flight_mask])
)

ideal = [
    (x_a, y_a),
    (x_a + 10.0, y_a),
    (x_a + 10.0, y_a + 10.0),
    (x_a, y_a + 10.0),
    (x_a, y_a),
]

ideal_points = " ".join(
    f"{map_x(x):.2f},{map_y(y):.2f}"
    for x, y in ideal
)

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="40" y="30" font-size="22" font-family="Arial">Mission 3 Square Path ({BAG_NAME})</text>

  <polyline points="{ideal_points}" fill="none" stroke="black" stroke-width="3" stroke-dasharray="8 6"/>
  <polyline points="{actual_points}" fill="none" stroke="blue" stroke-width="2"/>

  <circle cx="{map_x(x_a):.2f}" cy="{map_y(y_a):.2f}" r="5" fill="red"/>
  <text x="{map_x(x_a)+8:.2f}" y="{map_y(y_a)-8:.2f}" font-size="16" font-family="Arial">A</text>

  <circle cx="{map_x(x_a+10):.2f}" cy="{map_y(y_a):.2f}" r="5" fill="red"/>
  <text x="{map_x(x_a+10)+8:.2f}" y="{map_y(y_a)-8:.2f}" font-size="16" font-family="Arial">B</text>

  <circle cx="{map_x(x_a+10):.2f}" cy="{map_y(y_a+10):.2f}" r="5" fill="red"/>
  <text x="{map_x(x_a+10)+8:.2f}" y="{map_y(y_a+10)-8:.2f}" font-size="16" font-family="Arial">C</text>

  <circle cx="{map_x(x_a):.2f}" cy="{map_y(y_a+10):.2f}" r="5" fill="red"/>
  <text x="{map_x(x_a)+8:.2f}" y="{map_y(y_a+10)-8:.2f}" font-size="16" font-family="Arial">D</text>

  <line x1="40" y1="{height-35}" x2="100" y2="{height-35}" stroke="black" stroke-width="3" stroke-dasharray="8 6"/>
  <text x="110" y="{height-30}" font-size="16" font-family="Arial">Ideal path</text>

  <line x1="250" y1="{height-35}" x2="310" y2="{height-35}" stroke="blue" stroke-width="2"/>
  <text x="320" y="{height-30}" font-size="16" font-family="Arial">Actual path</text>
</svg>
'''

svg_path.write_text(svg, encoding="utf-8")

print(f"Saved {analysis_dir / f'result_{BAG_NAME}.txt'}")
print(f"Saved {analysis_dir / f'corner_errors_{BAG_NAME}.csv'}")
print(f"Saved {analysis_dir / f'path_data_{BAG_NAME}.csv'}")
print(f"Saved {svg_path}")
