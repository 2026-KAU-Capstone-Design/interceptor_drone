import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

# interceptor_control 패키지 경로
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interceptor_control.intercept import InterceptNode


class TestAltitudeLimit(unittest.TestCase):

    def setUp(self):
        # ROS 2 노드 생성 없이 필요한 값만 설정
        self.node = object.__new__(InterceptNode)
        self.node.min_tracking_alt = 2.0
        self.node.max_tracking_alt = 5.0
        self.node.takeoff_z = 0.0
        self.node.local_pos = SimpleNamespace(z=-3.0)

        # 실제 PX4 상태 대신 위치 유효성만 모의 처리
        self.node._is_local_position_ready = lambda: True

    def check_velocity(self, altitude, requested_vz, expected_vz):
        # NED: 고도 3m -> z=-3m
        self.node.local_pos.z = -altitude

        result = self.node._limit_vertical_velocity(requested_vz)

        self.assertTrue(math.isfinite(result))
        self.assertAlmostEqual(result, expected_vz, places=5)

    def test_normal_ascent(self):
        self.check_velocity(3.0, -0.30, -0.30)

    def test_upper_boundary_deceleration(self):
        self.check_velocity(4.8, -0.30, -0.12)

    def test_upper_boundary_block(self):
        self.check_velocity(5.0, -0.30, 0.0)

    def test_lower_boundary_deceleration(self):
        self.check_velocity(2.2, 0.30, 0.12)

    def test_lower_boundary_block(self):
        self.check_velocity(2.0, 0.30, 0.0)

    def test_allowed_descent_at_upper_boundary(self):
        self.check_velocity(5.0, 0.30, 0.30)

    def test_invalid_position(self):
        self.node._is_local_position_ready = lambda: False
        result = self.node._limit_vertical_velocity(-0.30)
        self.assertEqual(result, 0.0)

    def test_missing_takeoff_reference(self):
        self.node.takeoff_z = None
        result = self.node._limit_vertical_velocity(-0.30)
        self.assertEqual(result, 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
