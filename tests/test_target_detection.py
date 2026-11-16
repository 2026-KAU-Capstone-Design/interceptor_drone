
#!/usr/bin/env python3
"""YOLO 인식 데이터 수신 로직 단위 테스트."""

import sys
import unittest
from pathlib import Path

import rclpy
from std_msgs.msg import Float32MultiArray

# 로컬 ROS 2 패키지 소스 가져오기
PACKAGE_DIR = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/interceptor_control"
)
sys.path.insert(0, str(PACKAGE_DIR))

from interceptor_control.intercept import InterceptNode, FlightState


def make_bbox(class_id=0, detected=True, lock_on=True):
    """17개 필드를 가진 테스트 메시지 생성."""
    msg = Float32MultiArray()
    msg.data = [
        float(detected),  # 0: detected
        0.9,              # 1: confidence
        500.0, 350.0,     # 2-3: x1, y1
        700.0, 550.0,     # 4-5: x2, y2
        600.0, 450.0,     # 6-7: cx, cy
        -40.0, -30.0,     # 8-9: dx, dy
        1280.0, 960.0,    # 10-11: image size
        0.4,              # 12: red ratio
        3.0,              # 13: distance
        0.0,              # 14: near_2m
        float(class_id),  # 15: class_id
        float(lock_on),   # 16: lock_on
    ]
    return msg


class TestTargetDetection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = InterceptNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_balloon_valid(self):
        self.node._cb_bbox(make_bbox(class_id=0))
        self.assertTrue(self.node._has_valid_detection())
        self.assertEqual(self.node.target_class_id, 0)

    def test_fixedwing_rejected_in_balloon_mode(self):
        self.node._cb_bbox(make_bbox(class_id=1))
        self.assertFalse(self.node._has_valid_detection())

    def test_no_detection_resets_state(self):
        self.node._cb_bbox(make_bbox(class_id=0))
        self.node._cb_bbox(
            make_bbox(class_id=-1, detected=False, lock_on=False)
        )
        self.assertFalse(self.node.detected)
        self.assertFalse(self.node.lock_on)
        self.assertEqual(self.node.bbox_dx, 0.0)
        self.assertEqual(self.node.bbox_dy, 0.0)
        self.assertEqual(self.node.distance_m, -1.0)

    def test_short_message_rejected(self):
        self.node._cb_bbox(make_bbox(class_id=0))
        msg = Float32MultiArray()
        msg.data = [1.0, 0.9]
        self.node._cb_bbox(msg)
        self.assertFalse(self.node._has_valid_detection())
        self.assertIsNone(self.node.last_bbox_time)

    def test_stale_detection_rejected(self):
        self.node._cb_bbox(make_bbox(class_id=0))
        self.node.detection_timeout_sec = -1.0
        self.assertFalse(self.node._has_valid_detection())

    def test_detection_timeout_boundary(self):
        """실제 경과 시간을 가정한 검출 유효기간 테스트."""
        from rclpy.duration import Duration

        self.node._cb_bbox(make_bbox(class_id=0))

        # 마지막 검출이 0.2초 전이었다고 가정
        self.node.last_bbox_time = (
            self.node.get_clock().now()
            - Duration(seconds=0.2)
        )
        self.assertTrue(self.node._has_valid_detection())

        # 0.5초 경계값 대신 약간 여유가 있는 0.4초 검사
        self.node.last_bbox_time = (
            self.node.get_clock().now()
            - Duration(seconds=0.4)
        )
        self.assertTrue(self.node._has_valid_detection())

        # 마지막 검출이 0.6초 전이었다고 가정
        self.node.last_bbox_time = (
            self.node.get_clock().now()
            - Duration(seconds=0.6)
        )
        self.assertFalse(self.node._has_valid_detection())


    def test_track_to_intercept_publishes_zero_velocity(self):
        """TRACK에서 INTERCEPT로 전환할 때 제로 속도 명령을 발행하는지 검사."""

        # 현재 상태를 TRACK으로 설정
        self.node.state = FlightState.TRACK

        # PX4 상태 메시지를 정상적으로 수신한 상황 구성
        from px4_msgs.msg import VehicleStatus

        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 유효한 풍선 검출 메시지 입력
        msg = make_bbox(class_id=0, detected=True, lock_on=True)
        msg.data[13] = 1.5  # 추정 거리 1.5m
        msg.data[14] = 1.0  # near_2m=True
        self.node._cb_bbox(msg)

        # 실제 ROS 2 발행 대신 호출 내용을 기록
        velocity_calls = []
        offboard_calls = []

        self.node._publish_velocity_sp = (
            lambda *args, **kwargs:
            velocity_calls.append((args, kwargs))
        )
        self.node._publish_offboard_mode = (
            lambda *args, **kwargs:
            offboard_calls.append((args, kwargs))
        )

        # 이 테스트에서는 풍속 검사 통과로 가정
        self.node._check_wind_safety = lambda: True

        # 제어 루프를 딱 한 번 실행
        self.node._control_loop()

        # INTERCEPT 상태로 전환되었는지 확인
        self.assertEqual(self.node.state, FlightState.INTERCEPT)

        # 속도 명령이 정확히 한 번 발행되었는지 확인
        self.assertEqual(len(velocity_calls), 1)

        # 인자 없는 호출 = 기본값 vx=vy=vz=yawspeed=0.0
        self.assertEqual(velocity_calls[0], ((), {}))

        # Offboard 모드 발행 함수도 호출되었는지 확인
        self.assertEqual(len(offboard_calls), 1)


    def test_intercept_detection_loss_publishes_zero_velocity(self):
        """INTERCEPT 중 검출 소실 시 제로 속도 명령을 발행하는지 검사."""

        # INTERCEPT 상태 설정
        self.node.state = FlightState.INTERCEPT

        # PX4 상태 메시지가 정상적으로 수신된 상황 설정
        from px4_msgs.msg import VehicleStatus

        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 먼저 정상 풍선 검출
        self.node._cb_bbox(
            make_bbox(class_id=0, detected=True, lock_on=True)
        )

        # 이어서 검출 실패 메시지 입력
        self.node._cb_bbox(
            make_bbox(class_id=-1, detected=False, lock_on=False)
        )

        # 실제 PX4 발행 대신 함수 호출만 기록
        velocity_calls = []
        self.node._publish_velocity_sp = (
            lambda *args, **kwargs:
            velocity_calls.append((args, kwargs))
        )
        self.node._publish_offboard_mode = (
            lambda *args, **kwargs: None
        )

        # 제어 루프 한 번 실행
        self.node._control_loop()

        # 제로 속도 명령이 한 번 호출되었는지 확인
        self.assertEqual(velocity_calls, [((), {})])

        # 첫 번째 미검출에서는 INTERCEPT 상태 유지
        self.assertEqual(self.node.state, FlightState.INTERCEPT)
        self.assertEqual(self.node.miss_count, 1)


    def test_intercept_repeated_detection_loss_returns_search(self):
        """INTERCEPT에서 연속 미검출 시 SEARCH로 전환하는지 검사."""

        self.node.state = FlightState.INTERCEPT

        # PX4 상태 메시지가 정상적으로 수신된 상황 설정
        from px4_msgs.msg import VehicleStatus

        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 정상 검출 후 표적 소실
        self.node._cb_bbox(
            make_bbox(class_id=0, detected=True, lock_on=True)
        )
        self.node._cb_bbox(
            make_bbox(class_id=-1, detected=False, lock_on=False)
        )

        # 실제 ROS 2 발행 대신 호출 기록
        velocity_calls = []

        self.node._publish_velocity_sp = (
            lambda *args, **kwargs:
            velocity_calls.append((args, kwargs))
        )
        self.node._publish_offboard_mode = (
            lambda *args, **kwargs: None
        )

        # 미검출 허용 횟수만큼 제어 루프 실행
        for _ in range(self.node.lock_miss_limit):
            self.node._control_loop()

        # SEARCH 상태로 전환되었는지 확인
        self.assertEqual(self.node.state, FlightState.SEARCH)

        # 미검출 카운터가 초기화되었는지 확인
        self.assertEqual(self.node.miss_count, 0)

        # 매 제어 주기에 제로 속도 명령이 발행되었는지 확인
        self.assertEqual(
            len(velocity_calls),
            self.node.lock_miss_limit
        )

        self.assertTrue(
            all(call == ((), {}) for call in velocity_calls)
        )


    def test_intercept_timeout_publishes_zero_velocity(self):
        """YOLO 메시지 수신이 중단되면 전진 대신 제로 속도를 요청한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.INTERCEPT

        # PX4 상태 메시지는 정상적으로 수신된 상황
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 마지막으로 수신한 메시지는 정상적인 풍선 검출
        msg = make_bbox(class_id=0, detected=True, lock_on=True)
        msg.data[13] = 1.5
        msg.data[14] = 1.0
        self.node._cb_bbox(msg)

        # 새로운 메시지가 0.6초 동안 오지 않았다고 가정
        self.node.last_bbox_time = (
            self.node.get_clock().now()
            - Duration(seconds=0.6)
        )

        # 실제 PX4 명령 발행 대신 호출 내역만 기록
        velocity_calls = []

        self.node._publish_velocity_sp = (
            lambda *args, **kwargs:
            velocity_calls.append((args, kwargs))
        )
        self.node._publish_offboard_mode = (
            lambda *args, **kwargs: None
        )

        # 제어 루프 한 번 실행
        self.node._control_loop()

        # 이전 검출값이 True여도 타임아웃이면 무효
        self.assertTrue(self.node.detected)
        self.assertFalse(self.node._has_valid_detection())

        # 전진 명령이 아니라 제로 속도 명령을 호출해야 함
        self.assertEqual(velocity_calls, [((), {})])

        # 첫 번째 타임아웃 주기에서는 상태 유지
        self.assertEqual(self.node.state, FlightState.INTERCEPT)
        self.assertEqual(self.node.miss_count, 1)


    def test_nan_bbox_rejected(self):
        """중심 오차에 NaN이 포함된 검출 메시지는 거부해야 한다."""

        # 정상 메시지 생성
        msg = make_bbox(class_id=0, detected=True, lock_on=True)

        # 좌우 중심 오차를 비정상적인 숫자로 변경
        msg.data[8] = float("nan")

        # 실제 수신 콜백에 입력
        self.node._cb_bbox(msg)

        # 안전 요구사항: 비정상적인 값이 있으면 무효 검출 처리
        self.assertFalse(self.node._has_valid_detection())
        self.assertFalse(self.node.lock_on)
        self.assertFalse(self.node.near_2m)

    def test_zero_image_width_rejected(self):
        """영상 너비가 0인 검출 메시지는 거부해야 한다."""

        # 정상 풍선 검출 메시지 생성
        msg = make_bbox(class_id=0, detected=True, lock_on=True)

        # 영상 너비를 비정상적인 값으로 변경
        msg.data[10] = 0.0

        # 수신 콜백 실행
        self.node._cb_bbox(msg)

        # 검출 데이터가 무효화되었는지 확인
        self.assertFalse(self.node._has_valid_detection())
        self.assertFalse(self.node.detected)
        self.assertFalse(self.node.lock_on)
        self.assertFalse(self.node.near_2m)

        # 잘못된 입력으로 이전 중심 오차가 남지 않아야 함
        self.assertEqual(self.node.bbox_dx, 0.0)
        self.assertEqual(self.node.bbox_dy, 0.0)
        self.assertIsNone(self.node.last_bbox_time)

    def test_inconsistent_near_distance_rejected(self):
        """거리 4.5m인데 near_2m=True인 메시지는 근접으로 인정하지 않는다."""

        msg = make_bbox(class_id=0, detected=True, lock_on=True)

        # 서로 모순되는 거리 정보
        msg.data[13] = 4.5
        msg.data[14] = 1.0

        self.node._cb_bbox(msg)

        # 잘못된 거리 조합은 근접 판정에 사용할 수 없어야 함
        self.assertFalse(self.node.near_2m)

        # 근접 판정에 오류가 있더라도 유효하지 않은 검출로 처리
        self.assertFalse(self.node._has_valid_detection())


    def test_fractional_class_id_rejected(self):
        """정수가 아닌 클래스 ID 0.5는 풍선으로 인정하지 않는다."""

        # 정상적인 풍선 검출 메시지 생성
        msg = make_bbox(class_id=0, detected=True, lock_on=True)

        # 클래스 ID만 잘못된 값으로 변경
        msg.data[15] = 0.5

        # 실제 메시지 수신 함수 실행
        self.node._cb_bbox(msg)

        # 잘못된 클래스 ID는 무효한 검출로 처리되어야 함
        self.assertFalse(self.node._has_valid_detection())
        self.assertFalse(self.node.detected)
        self.assertEqual(self.node.target_class_id, -1)
        self.assertFalse(self.node.lock_on)
        self.assertFalse(self.node.near_2m)
        self.assertIsNone(self.node.last_bbox_time)

    def test_done_state_publishes_no_commands(self):
        """DONE 상태에서는 Offboard 및 Setpoint를 발행하지 않는다."""
        from unittest.mock import patch

        self.node.state = FlightState.DONE

        with patch.object(self.node, "_publish_offboard_mode") as mode_mock, \
             patch.object(self.node, "_publish_position_sp") as position_mock, \
             patch.object(self.node, "_publish_velocity_sp") as velocity_mock, \
             patch.object(self.node, "_send_command") as command_mock:

            self.node._control_loop()

            mode_mock.assert_not_called()
            position_mock.assert_not_called()
            velocity_mock.assert_not_called()
            command_mock.assert_not_called()

    def test_command_ack_is_stored(self):
        """PX4의 RTL 명령 응답을 정상적으로 저장하는지 확인한다."""
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        ack = VehicleCommandAck()
        ack.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED

        # ACK 수신 전
        self.assertIsNone(self.node.last_command_ack)

        # PX4에서 ACK를 수신했다고 가정
        self.node._cb_command_ack(ack)

        # 수신한 ACK가 저장되었는지 확인
        self.assertIsNotNone(self.node.last_command_ack)
        self.assertEqual(self.node.last_command_ack.command, 20)
        self.assertEqual(self.node.last_command_ack.result, 0)


    def test_command_ack_receive_time_is_stored(self):
        """ACK 수신 시 메시지와 수신 시각을 함께 저장한다."""
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        ack = VehicleCommandAck()
        ack.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED

        # ACK 수신 이전
        self.assertIsNone(self.node.last_command_ack)
        self.assertIsNone(self.node.last_command_ack_time)

        # 수신 시각의 전후를 기록
        before = self.node.get_clock().now()
        self.node._cb_command_ack(ack)
        after = self.node.get_clock().now()

        # ACK와 시각이 저장되었는지 확인
        self.assertIs(self.node.last_command_ack, ack)
        self.assertIsNotNone(self.node.last_command_ack_time)

        # 저장된 시각이 실제 콜백 호출 시점에 해당하는지 확인
        received = self.node.last_command_ack_time
        self.assertGreaterEqual(received.nanoseconds, before.nanoseconds)
        self.assertLessEqual(received.nanoseconds, after.nanoseconds)


    def test_new_command_clears_previous_ack(self):
        """새 명령 전송 시 기존 ACK와 수신 시각을 초기화한다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        # 이전 RTL 명령의 ACK를 수신한 상황
        old_ack = VehicleCommandAck()
        old_ack.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        old_ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED

        self.node._cb_command_ack(old_ack)

        self.assertIsNotNone(self.node.last_command_ack)
        self.assertIsNotNone(self.node.last_command_ack_time)

        # 새로운 LAND 명령 전송을 가정
        # 실제 ROS 2 메시지 발행은 차단
        with patch.object(self.node.command_pub, "publish") as publish_mock:
            self.node._send_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

            publish_mock.assert_called_once()

            # 실제 발행하려던 명령이 LAND인지 확인
            sent_msg = publish_mock.call_args.args[0]
            self.assertEqual(
                sent_msg.command,
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

        # 새 명령 전송 후 이전 ACK가 남아 있으면 안 됨
        self.assertIsNone(self.node.last_command_ack)
        self.assertIsNone(self.node.last_command_ack_time)


    def test_pending_command_ack_validation(self):
        """현재 명령과 일치하고 수락된 ACK만 인정한다."""
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        land = VehicleCommand.VEHICLE_CMD_NAV_LAND

        # 현재 RTL 명령의 응답을 기다리는 상황
        self.node.pending_command = rtl
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        # 1. ACK가 아직 없다면 인정하지 않음
        self.assertFalse(
            self.node._is_pending_command_accepted()
        )

        # 2. 다른 명령(LAND)의 ACK라면 인정하지 않음
        ack = VehicleCommandAck()
        ack.command = land
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.node._cb_command_ack(ack)

        self.assertFalse(
            self.node._is_pending_command_accepted()
        )

        # 3. RTL 명령이지만 DENIED라면 인정하지 않음
        ack.command = rtl
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        self.node._cb_command_ack(ack)

        self.assertFalse(
            self.node._is_pending_command_accepted()
        )

        # 4. RTL 명령이고 ACCEPTED라면 인정
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.node._cb_command_ack(ack)

        self.assertTrue(
            self.node._is_pending_command_accepted()
        )

        # 5. 요청 이전 시각의 ACK는 인정하지 않음
        from rclpy.duration import Duration

        self.node.last_command_ack_time = (
            self.node.pending_command_sent_time
            - Duration(seconds=1)
        )

        self.assertFalse(
            self.node._is_pending_command_accepted()
        )


    def test_rtl_command_registers_pending_state(self):
        """RTL 명령 전송 시 ACK 대기 상태가 등록되는지 확인한다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleCommand

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

        # 명령 전송 전에는 대기 상태가 없음
        self.assertIsNone(self.node.pending_command)
        self.assertIsNone(self.node.pending_command_sent_time)

        before = self.node.get_clock().now()

        # 실제 PX4 메시지 발행은 차단
        with patch.object(self.node.command_pub, "publish") as publish_mock:
            self.node._send_command(rtl)
            publish_mock.assert_called_once()

        after = self.node.get_clock().now()

        # 대기 중인 명령이 RTL인지 확인
        self.assertEqual(self.node.pending_command, rtl)

        # 명령 요청 시각이 기록되었는지 확인
        sent_time = self.node.pending_command_sent_time
        self.assertIsNotNone(sent_time)
        self.assertGreaterEqual(
            sent_time.nanoseconds, before.nanoseconds
        )
        self.assertLessEqual(
            sent_time.nanoseconds, after.nanoseconds
        )

        # 새로운 명령을 기다리므로 기존 ACK는 없어야 함
        self.assertIsNone(self.node.last_command_ack)
        self.assertIsNone(self.node.last_command_ack_time)


    def test_pending_flight_mode_validation(self):
        """요청한 종료 명령과 실제 PX4 비행 모드가 일치하는지 검사."""
        from px4_msgs.msg import VehicleCommand, VehicleStatus

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        land = VehicleCommand.VEHICLE_CMD_NAV_LAND

        # 1. 대기 중인 명령이 없으면 False
        self.assertFalse(
            self.node._is_pending_mode_active()
        )

        # 테스트용 PX4 상태 수신 시각 설정
        self.node.pending_command_sent_time = self.node.get_clock().now()
        self.node.last_vehicle_status_time = self.node.get_clock().now()

        # 2. RTL 요청 중이지만 PX4는 아직 Offboard
        self.node.pending_command = rtl
        self.node.vehicle_status.nav_state = (
            VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

        self.assertFalse(
            self.node._is_pending_mode_active()
        )

        # 3. RTL 요청 후 실제 AUTO_RTL로 변경
        self.node.vehicle_status.nav_state = (
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        )

        self.assertTrue(
            self.node._is_pending_mode_active()
        )

        # 4. LAND 요청인데 PX4가 RTL 모드라면 False
        self.node.pending_command = land

        self.assertFalse(
            self.node._is_pending_mode_active()
        )

        # 5. LAND 요청 후 실제 AUTO_LAND로 변경
        self.node.vehicle_status.nav_state = (
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        )

        self.assertTrue(
            self.node._is_pending_mode_active()
        )


    def test_vehicle_status_receive_time_is_stored(self):
        """PX4 상태 메시지 수신 시 시각을 함께 저장한다."""
        from px4_msgs.msg import VehicleStatus

        msg = VehicleStatus()
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL

        # 메시지 수신 전
        self.assertIsNone(self.node.last_vehicle_status_time)

        # 콜백 실행 전후 시각 기록
        before = self.node.get_clock().now()
        self.node._cb_status(msg)
        after = self.node.get_clock().now()

        # 상태 메시지가 저장되었는지 확인
        self.assertIs(self.node.vehicle_status, msg)

        # 수신 시각이 기록되었는지 확인
        received = self.node.last_vehicle_status_time
        self.assertIsNotNone(received)

        # 기록된 시각이 콜백 실행 시점에 해당하는지 확인
        self.assertGreaterEqual(
            received.nanoseconds,
            before.nanoseconds
        )
        self.assertLessEqual(
            received.nanoseconds,
            after.nanoseconds
        )

    def test_vehicle_status_timeout_independence(self):
        """통신 감시와 제어권 이양의 최신성 기준이 독립적인지 확인."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 시험용 설정: 실제 비행에 적용하는 값은 아님
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        # PX4가 상태 메시지를 수신한 것으로 설정
        msg = VehicleStatus()
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(msg)

        # 마지막 수신이 1.0초 전이라고 가정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.0)
        )

        # 통신 감시 기준(1.5초): 아직 단절 아님
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양 기준(0.8초): 오래된 상태
        self.assertFalse(
            self.node._is_vehicle_status_fresh(
                self.node.handover_status_timeout_sec
            )
        )

    def test_vehicle_status_timeout_independence(self):
        """통신 감시와 제어권 이양의 최신성 기준이 독립적인지 확인."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 시험용 설정: 실제 비행에 적용하는 값은 아님
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        # PX4가 상태 메시지를 수신한 것으로 설정
        msg = VehicleStatus()
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(msg)

        # 마지막 수신이 1.0초 전이라고 가정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.0)
        )

        # 통신 감시 기준(1.5초): 아직 단절 아님
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양 기준(0.8초): 오래된 상태
        self.assertFalse(
            self.node._is_vehicle_status_fresh(
                self.node.handover_status_timeout_sec
            )
        )

    def test_vehicle_status_freshness(self):
        """PX4 상태 메시지의 수신 시각을 기준으로 최신성을 검사한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 1. 상태 메시지를 받은 적이 없다면 유효하지 않음
        self.assertIsNone(self.node.last_vehicle_status_time)
        self.assertFalse(
            self.node._is_vehicle_status_fresh()
        )

        # 2. 현재 시점에 상태 메시지를 수신
        msg = VehicleStatus()
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(msg)

        self.assertTrue(
            self.node._is_vehicle_status_fresh()
        )

        # 3. 1.6초 전에 받은 메시지는 오래된 정보
        now = self.node.get_clock().now()
        self.node.last_vehicle_status_time = (
            now - Duration(seconds=1.6)
        )

        self.assertFalse(
            self.node._is_vehicle_status_fresh()
        )

        # 4. 미래 시각으로 기록된 메시지도 거부
        self.node.last_vehicle_status_time = (
            now + Duration(seconds=1)
        )

        self.assertFalse(
            self.node._is_vehicle_status_fresh()
        )

    def test_land_mode_accepts_fresh_status(self):
        """명령 이후 수신한 최신 LAND 상태를 정상적으로 인정한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus, VehicleCommand

        # 단위 테스트에서만 적용하는 타임아웃
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        now = self.node.get_clock().now()

        # LAND 명령을 0.6초 전에 요청
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.node.pending_command_sent_time = (
            now - Duration(seconds=0.6)
        )

        # LAND 상태를 명령 이후인 0.3초 전에 수신
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self.node._cb_status(status)
        self.node.last_vehicle_status_time = (
            now - Duration(seconds=0.3)
        )

        # 통신 감시 기준에서도 유효
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양 기준에서도 유효
        self.assertTrue(
            self.node._is_pending_mode_active()
        )

    def test_land_mode_uses_handover_status_timeout(self):
        """LAND 모드 확인에 제어권 이양 전용 타임아웃을 적용한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus, VehicleCommand

        # 테스트에서만 서로 다른 타임아웃 사용
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        now = self.node.get_clock().now()

        # LAND 명령은 1.2초 전에 요청
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.node.pending_command_sent_time = (
            now - Duration(seconds=1.2)
        )

        # LAND 상태는 명령 이후인 1.0초 전에 수신
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self.node._cb_status(status)
        self.node.last_vehicle_status_time = (
            now - Duration(seconds=1.0)
        )

        # 통신 감시 기준으로는 아직 정상
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양 기준에서는 오래된 상태
        self.assertFalse(
            self.node._is_pending_mode_active()
        )

    def test_rtl_mode_accepts_fresh_status(self):
        """명령 이후 수신한 최신 RTL 상태는 정상적으로 인정한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus, VehicleCommand

        # 독립성 검증을 위한 시험용 값
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        now = self.node.get_clock().now()

        # RTL 명령을 0.6초 전에 요청
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )
        self.node.pending_command_sent_time = (
            now - Duration(seconds=0.6)
        )

        # 명령 이후인 0.3초 전에 RTL 상태를 수신
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)
        self.node.last_vehicle_status_time = (
            now - Duration(seconds=0.3)
        )

        # 통신 감시 기준에서도 유효
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양 기준에서도 유효
        self.assertTrue(
            self.node._is_pending_mode_active()
        )

    def test_rtl_mode_uses_handover_status_timeout(self):
        """RTL 모드 확인에 제어권 이양 전용 타임아웃을 적용한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus, VehicleCommand

        # 실제 설정 변경이 아닌 테스트용 값
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        # RTL 요청 시각을 1.2초 전으로 설정
        now = self.node.get_clock().now()
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )
        self.node.pending_command_sent_time = (
            now - Duration(seconds=1.2)
        )

        # RTL 상태 메시지는 1.0초 전에 수신했다고 가정
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)
        self.node.last_vehicle_status_time = (
            now - Duration(seconds=1.0)
        )

        # 통신 감시용 1.5초 기준에서는 정상
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 제어권 이양용 0.8초 기준에서는 오래된 상태
        self.assertFalse(
            self.node._is_pending_mode_active()
        )

    def test_old_status_cannot_confirm_new_rtl(self):
        """새 RTL 요청 이전에 수신된 PX4 상태는 인정하지 않는다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleCommand, VehicleStatus

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

        # 새로운 RTL 요청이 발생한 시각
        request_time = self.node.get_clock().now()

        self.node.pending_command = rtl
        self.node.pending_command_sent_time = request_time

        # PX4 상태값 자체는 AUTO_RTL
        self.node.vehicle_status.nav_state = (
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        )

        # 하지만 해당 상태는 요청 0.1초 전에 수신됨
        self.node.last_vehicle_status_time = (
            request_time - Duration(seconds=0.1)
        )

        # 최신 데이터라도 요청 이전의 상태이므로 거부
        self.assertTrue(self.node._is_vehicle_status_fresh())
        self.assertFalse(self.node._is_pending_mode_active())

        # 요청 이후 새 상태 메시지를 수신한 상황
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)

        # 이제 실제 RTL 모드 정보로 인정
        self.assertTrue(self.node._is_pending_mode_active())


    def test_handover_requires_ack_and_mode(self):
        """ACK 수락과 실제 PX4 모드 전환이 모두 필요하다."""
        from px4_msgs.msg import (
            VehicleCommand,
            VehicleCommandAck,
            VehicleStatus,
        )

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

        # 새로운 RTL 요청을 기다리는 상태
        self.node.pending_command = rtl
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        # 1. ACK도 없고 PX4도 OFFBOARD 상태
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.assertFalse(self.node._is_handover_confirmed())

        # 2. ACK는 수락했지만 PX4는 여전히 OFFBOARD
        ack = VehicleCommandAck()
        ack.command = rtl
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.node._cb_command_ack(ack)

        self.assertFalse(self.node._is_handover_confirmed())

        # 3. PX4가 실제 AUTO_RTL로 전환
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)

        self.assertTrue(self.node._is_handover_confirmed())

        # 4. ACK가 거부된 경우에는 모드가 맞아도 불인정
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        self.node._cb_command_ack(ack)

        self.assertFalse(self.node._is_handover_confirmed())


    def test_rtl_command_sent_only_once(self):
        """RTL 명령은 한 번만 보내고, 확인 전에는 DONE으로 전환하지 않는다."""
        from unittest.mock import patch

        self.node.state = FlightState.RTL
        self.node.handover_command_sent = False

        # 실제 PX4 명령 발행은 모의 함수로 대체
        with patch.object(self.node, "return_to_launch") as rtl_mock, \
             patch.object(
                 self.node, "_publish_offboard_mode"
             ), \
             patch.object(
                 self.node, "_is_handover_confirmed",
                 return_value=False
             ):

            # 첫 번째 제어 주기
            self.node._control_loop()

            rtl_mock.assert_called_once()
            self.assertTrue(self.node.handover_command_sent)
            self.assertEqual(self.node.state, FlightState.RTL)

            # 두 번째 제어 주기
            self.node._control_loop()

            # RTL 명령은 여전히 총 한 번만 호출되어야 함
            rtl_mock.assert_called_once()

            # PX4 확인이 없으므로 DONE이 되면 안 됨
            self.assertEqual(self.node.state, FlightState.RTL)


    def test_rtl_wait_publishes_zero_velocity(self):
        """RTL 대기 중 PX4가 OFFBOARD라면 제로 속도 목표를 발행한다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        # RTL 명령은 이미 전송했지만 제어권 이양은 미완료
        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True

        # PX4가 아직 OFFBOARD 상태라고 가정
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 실제 PX4 메시지 발행은 모의 함수로 대체
        with patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "return_to_launch"
        ) as rtl_mock, patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ):

            self.node._control_loop()

            # 인자 없는 호출은 속도 목표 0을 의미
            velocity_mock.assert_called_once_with()

            # 이미 전송한 RTL 명령은 반복하지 않음
            rtl_mock.assert_not_called()

            # 제어권 이양 전에는 DONE으로 전환하지 않음
            self.assertEqual(self.node.state, FlightState.RTL)


    def test_pending_command_rejection_validation(self):
        """현재 RTL 요청에 대한 거부·실패 ACK만 판별한다."""
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        land = VehicleCommand.VEHICLE_CMD_NAV_LAND

        self.node.pending_command = rtl
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        # 1. ACK가 없으면 거부로 판단하지 않음
        self.assertFalse(
            self.node._is_pending_command_rejected()
        )

        ack = VehicleCommandAck()

        # 2. 다른 명령의 거부 응답은 무시
        ack.command = land
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        self.node._cb_command_ack(ack)

        self.assertFalse(
            self.node._is_pending_command_rejected()
        )

        # 3. 현재 RTL 명령의 거부 응답
        ack.command = rtl
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        self.node._cb_command_ack(ack)

        self.assertTrue(
            self.node._is_pending_command_rejected()
        )

        # 4. 명령 실행 실패
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED
        self.node._cb_command_ack(ack)

        self.assertTrue(
            self.node._is_pending_command_rejected()
        )

        # 5. 정상 수락 응답은 거부가 아님
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.node._cb_command_ack(ack)

        self.assertFalse(
            self.node._is_pending_command_rejected()
        )


    def test_rtl_rejection_records_failure_once(self):
        """RTL 거부 시 실패를 기록하고 오류 로그를 반복하지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        # PX4가 아직 OFFBOARD 상태인 상황
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 외부 명령 발행 없이 거부 상황만 모의
        with patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ), patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=True
        ), patch.object(
            self.node.get_logger(), "error"
        ) as error_mock:

            # 첫 번째 제어 주기
            self.node._control_loop()

            self.assertTrue(self.node.handover_failed)
            self.assertEqual(self.node.state, FlightState.RTL)
            error_mock.assert_called_once()

            # 두 번째 제어 주기
            self.node._control_loop()

            # 실패 상태는 유지하고 로그는 추가 출력하지 않음
            self.assertTrue(self.node.handover_failed)
            self.assertEqual(self.node.state, FlightState.RTL)
            error_mock.assert_called_once()

    def test_handover_timeout_boundary(self):
        """종료 명령 전송 후 3초가 지나면 시간 초과로 판단한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleCommand

        rtl = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

        # 1. 아직 종료 명령이 없는 경우
        self.node.pending_command = None
        self.node.pending_command_sent_time = None

        self.assertFalse(self.node._is_handover_timed_out())

        # 2. RTL 명령 전송 후 2.9초 경과
        self.node.pending_command = rtl
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
            - Duration(seconds=2.9)
        )

        self.assertFalse(self.node._is_handover_timed_out())

        # 3. RTL 명령 전송 후 3.1초 경과
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
            - Duration(seconds=3.1)
        )

        self.assertTrue(self.node._is_handover_timed_out())

        # 4. 아직 명령을 보내지 않은 미래 시각은 무시
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
            + Duration(seconds=1.0)
        )

        self.assertFalse(self.node._is_handover_timed_out())


    def test_rtl_timeout_records_failure_once(self):
        """RTL 응답 시간 초과 시 실패를 한 번만 기록한다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        # PX4가 아직 OFFBOARD 상태
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 실제 PX4 명령 발행 없이 시간 초과만 모의
        with patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ), patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=True
        ), patch.object(
            self.node.get_logger(), "error"
        ) as error_mock:

            # 첫 번째 제어 주기
            self.node._control_loop()

            self.assertTrue(self.node.handover_failed)
            self.assertEqual(self.node.state, FlightState.RTL)
            error_mock.assert_called_once()

            # 두 번째 제어 주기
            self.node._control_loop()

            # 실패 상태 유지, 오류 로그 반복 출력 없음
            self.assertTrue(self.node.handover_failed)
            self.assertEqual(self.node.state, FlightState.RTL)
            error_mock.assert_called_once()

    def test_land_command_sent_only_once(self):
        """LAND 명령은 한 번만 전송하고 모드 확인 전에는 DONE이 되지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.LANDING
        self.node.handover_command_sent = False
        self.node.handover_failed = False

        # PX4가 아직 OFFBOARD 상태
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 실제 PX4 명령은 발행하지 않음
        with patch.object(
            self.node, "land"
        ) as land_mock, patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=False
        ):

            # 첫 번째 제어 주기
            self.node._control_loop()

            land_mock.assert_called_once()
            self.assertTrue(self.node.handover_command_sent)
            self.assertEqual(self.node.state, FlightState.LANDING)

            # 두 번째 제어 주기
            self.node._control_loop()

            # LAND 명령은 총 한 번만 호출
            land_mock.assert_called_once()

            # OFFBOARD 상태이므로 매 주기 제로 속도 목표 발행
            self.assertEqual(velocity_mock.call_count, 2)
            velocity_mock.assert_any_call()

            # PX4 모드 전환 전에는 DONE이 되면 안 됨
            self.assertEqual(self.node.state, FlightState.LANDING)

    def test_enter_handover_state_resets_only_on_transition(self):
        """종료 상태가 실제로 변경될 때만 요청 정보를 초기화한다."""
        from px4_msgs.msg import VehicleCommand, VehicleCommandAck

        self.node.state = FlightState.TRACK

        # 이전 종료 명령 정보가 남아 있는 상황
        self.node.handover_command_sent = True
        self.node.handover_failed = True
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        ack = VehicleCommandAck()
        self.node.last_command_ack = ack
        self.node.last_command_ack_time = (
            self.node.get_clock().now()
        )

        # 1. TRACK → RTL: 초기화
        self.node._enter_handover_state(FlightState.RTL)

        self.assertEqual(self.node.state, FlightState.RTL)
        self.assertFalse(self.node.handover_command_sent)
        self.assertFalse(self.node.handover_failed)
        self.assertIsNone(self.node.pending_command)
        self.assertIsNone(self.node.pending_command_sent_time)
        self.assertIsNone(self.node.last_command_ack)
        self.assertIsNone(self.node.last_command_ack_time)

        # 2. RTL → RTL: 같은 상태면 기존 값 유지
        self.node.handover_command_sent = True
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )

        self.node._enter_handover_state(FlightState.RTL)

        self.assertTrue(self.node.handover_command_sent)
        self.assertEqual(
            self.node.pending_command,
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
        )

        # 3. RTL → LANDING: 이전 RTL 요청 정보 초기화
        self.node._enter_handover_state(FlightState.LANDING)

        self.assertEqual(self.node.state, FlightState.LANDING)
        self.assertFalse(self.node.handover_command_sent)
        self.assertFalse(self.node.handover_failed)
        self.assertIsNone(self.node.pending_command)

    def test_wind_safety_enters_rtl_and_resets_handover(self):
        """풍속 임계치 초과 시 RTL 진입과 종료 변수 초기화를 확인한다."""
        from px4_msgs.msg import VehicleCommand

        # 추적 중인 상황
        self.node.state = FlightState.TRACK

        # 이전 종료 명령 정보가 남아 있다고 가정
        self.node.handover_command_sent = True
        self.node.handover_failed = True
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        # 풍속 추정값이 임계치에 도달한 상황
        self.node.wind_speed_est = self.node.wind_critical

        # 안전 모니터 실행
        result = self.node._check_wind_safety()

        # 임계치 초과로 정상적인 추적을 계속할 수 없음
        self.assertFalse(result)

        # RTL 상태로 변경
        self.assertEqual(self.node.state, FlightState.RTL)

        # 이전 종료 명령 정보 초기화
        self.assertFalse(self.node.handover_command_sent)
        self.assertFalse(self.node.handover_failed)
        self.assertIsNone(self.node.pending_command)
        self.assertIsNone(self.node.pending_command_sent_time)

    def test_attitude_safety_enters_landing_and_resets_handover(self):
        """자세 이상 지속 시 LANDING 진입과 종료 변수 초기화를 확인한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleCommand

        # RTL 요청을 이미 전송한 상황
        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = True
        self.node.pending_command = (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )
        self.node.pending_command_sent_time = (
            self.node.get_clock().now()
        )

        # 기체 기울기가 허용 범위를 초과
        self.node.tilt_deg = self.node.tilt_threshold + 5.0

        # 자세 이상이 허용 시간보다 오래 지속된 상황
        self.node.attitude_disturbed_since = (
            self.node.get_clock().now()
            - Duration(seconds=self.node.att_timeout + 1.0)
        )

        # 자세 안전 모니터 실행
        self.node._check_attitude_safety()

        # 비상 착륙 상태로 전환
        self.assertEqual(self.node.state, FlightState.LANDING)

        # 이전 RTL 요청 정보 초기화
        self.assertFalse(self.node.handover_command_sent)
        self.assertFalse(self.node.handover_failed)
        self.assertIsNone(self.node.pending_command)
        self.assertIsNone(self.node.pending_command_sent_time)

    def test_unknown_wind_does_not_trigger_rtl(self):
        """풍속 미측정 상태에서 오류나 잘못된 RTL 전환이 없어야 한다."""

        # 표적 추적 중인 상황
        self.node.state = FlightState.TRACK

        # 신뢰할 수 있는 풍속 정보가 없음
        self.node.wind_speed_est = None

        # 기존 종료 상태 정보
        self.node.handover_command_sent = False
        self.node.handover_failed = False

        # 풍속 안전 검사 실행
        result = self.node._check_wind_safety()

        # 예외 없이 검사 완료
        self.assertTrue(result)

        # 풍속을 모른다는 이유만으로 RTL에 진입하지 않음
        self.assertEqual(self.node.state, FlightState.TRACK)

        # 종료 명령 관련 상태도 변경되지 않음
        self.assertFalse(self.node.handover_command_sent)
        self.assertFalse(self.node.handover_failed)

    def test_local_velocity_is_not_treated_as_wind(self):
        """기체의 지면 기준 이동 속도를 풍속으로 사용하지 않는다."""
        from px4_msgs.msg import VehicleLocalPosition

        self.node.state = FlightState.TRACK
        self.node.wind_speed_est = None

        # 바람과 무관하게 기체가 빠르게 이동하는 상황
        msg = VehicleLocalPosition()
        msg.vx = 16.0
        msg.vy = 12.0
        msg.v_xy_valid = True

        # PX4 위치 메시지 수신
        self.node._cb_local_pos(msg)

        # 위치 메시지는 정상적으로 저장
        self.assertIs(self.node.local_pos, msg)

        # 지면속도가 풍속 추정값으로 저장되지 않아야 함
        self.assertIsNone(self.node.wind_speed_est)

        # 풍속 미측정을 이유로 잘못된 RTL이 발생하지 않음
        self.assertTrue(self.node._check_wind_safety())
        self.assertEqual(self.node.state, FlightState.TRACK)

    def test_landing_setpoint_uses_handover_timeout(self):
        """LANDING 제로 속도 발행에 제어권 이양용 최신성 기준을 적용한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 실제 비행 설정이 아닌 테스트용 값
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        self.node.state = FlightState.LANDING
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        # 마지막 PX4 상태는 OFFBOARD
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 상태 메시지를 1초 전에 수신했다고 가정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.0)
        )

        # 통신 감시 기준에서는 단절 아님
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as mode_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=False
        ):
            self.node._control_loop()

            # 현재 구현에서는 Offboard 모드 메시지 발행 유지
            mode_mock.assert_called()

            # 오래된 상태를 근거로 제로 속도 명령을 발행하지 않음
            velocity_mock.assert_not_called()

        # LANDING 대기 상태 유지
        self.assertEqual(self.node.state, FlightState.LANDING)

    def test_rtl_setpoint_uses_handover_timeout(self):
        """RTL 제로 속도 발행에 제어권 이양용 최신성 기준을 적용한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 실제 비행 설정과 무관한 테스트용 값
        self.node.vehicle_status_loss_timeout_sec = 1.5
        self.node.handover_status_timeout_sec = 0.8

        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        # 마지막 PX4 상태는 OFFBOARD
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 상태 메시지를 1.0초 전에 수신했다고 가정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.0)
        )

        # 통신 감시 기준에서는 단절 아님
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as mode_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=False
        ):
            self.node._control_loop()

            # 제어권 이양을 기다리는 동안 기존 Offboard 모드 발행 유지
            mode_mock.assert_called()

            # 오래된 OFFBOARD 상태로 제로 속도 명령을 발행하지 않음
            velocity_mock.assert_not_called()

        # RTL 대기 상태 유지
        self.assertEqual(self.node.state, FlightState.RTL)

    def test_rtl_stale_status_publishes_mode_without_setpoint(self):
        """RTL 대기 중 상태 정보가 오래되면 현재 발행 동작을 확인한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        # PX4는 마지막으로 OFFBOARD 상태를 보고했음
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 마지막 상태 수신이 1.6초 전이라고 가정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertFalse(
            self.node._is_vehicle_status_fresh()
        )

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as mode_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=False
        ), patch.object(
            self.node, "return_to_launch"
        ) as rtl_mock:

            self.node._control_loop()

            # 현재 구현은 Offboard 모드 메시지를 발행함
            mode_mock.assert_called_once_with(position=False)

            # 상태 메시지가 오래되면 제로 속도 목표는 발행하지 않음
            velocity_mock.assert_not_called()

            # RTL 명령은 반복 전송하지 않음
            rtl_mock.assert_not_called()

            # 제어권 이양 미확인
            self.assertEqual(self.node.state, FlightState.RTL)

    def test_vehicle_status_loss_detection(self):
        """PX4 상태 메시지의 미수신, 정상 지연, 수신 중단을 구분한다."""
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # 1. PX4 상태 메시지를 한 번도 받지 않은 경우
        self.node.last_vehicle_status_time = None

        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 2. 상태 메시지를 정상적으로 수신한 경우
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.assertTrue(
            self.node._is_vehicle_status_fresh()
        )
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 3. 실제 측정된 정상 수신 간격: 0.504초
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=0.504)
        )

        self.assertTrue(
            self.node._is_vehicle_status_fresh()
        )
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 4. 마지막 수신 후 1초 경과: 아직 정상
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.0)
        )

        self.assertTrue(
            self.node._is_vehicle_status_fresh()
        )
        self.assertFalse(
            self.node._is_vehicle_status_lost()
        )

        # 5. 마지막 수신 후 1.6초 경과: 통신 단절
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertFalse(
            self.node._is_vehicle_status_fresh()
        )
        self.assertTrue(
            self.node._is_vehicle_status_lost()
        )

    def test_vehicle_status_monitor_loss_and_recovery(self):
        """PX4 상태 메시지 단절·복구 로그를 중복 없이 기록한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.vehicle_status_lost_reported = False

        # 1. 상태 메시지를 한 번도 받지 않은 경우
        self.node.last_vehicle_status_time = None

        with patch.object(
            self.node.get_logger(), "error"
        ) as error_mock, patch.object(
            self.node.get_logger(), "info"
        ) as info_mock:

            self.node._monitor_vehicle_status()

            error_mock.assert_not_called()
            info_mock.assert_not_called()

            # 2. PX4 상태 메시지 정상 수신
            status = VehicleStatus()
            status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            self.node._cb_status(status)

            self.node._monitor_vehicle_status()

            error_mock.assert_not_called()
            info_mock.assert_not_called()

            # 3. 마지막 수신 후 1.6초 경과: 통신 단절
            self.node.last_vehicle_status_time = (
                self.node.get_clock().now()
                - Duration(seconds=1.6)
            )

            self.node._monitor_vehicle_status()

            self.assertTrue(
                self.node.vehicle_status_lost_reported
            )
            error_mock.assert_called_once()

            # 4. 계속 끊겨 있어도 오류 로그 반복 없음
            self.node._monitor_vehicle_status()

            error_mock.assert_called_once()

            # 5. PX4 상태 메시지 수신 복구
            self.node._cb_status(status)
            self.node._monitor_vehicle_status()

            self.assertFalse(
                self.node.vehicle_status_lost_reported
            )
            info_mock.assert_called_once()

            # 6. 정상 상태가 유지되면 복구 로그 반복 없음
            self.node._monitor_vehicle_status()

            info_mock.assert_called_once()

    def test_control_loop_calls_vehicle_status_monitor(self):
        """제어 루프에서 PX4 상태 감시 함수를 호출하는지 확인한다."""
        from unittest.mock import patch

        # 1. RTL 상태에서는 상태 감시 수행
        self.node.state = FlightState.RTL
        self.node.handover_command_sent = True
        self.node.handover_failed = False

        with patch.object(
            self.node, "_monitor_vehicle_status"
        ) as monitor_mock, patch.object(
            self.node, "_publish_offboard_mode"
        ) as mode_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ), patch.object(
            self.node, "_is_handover_confirmed",
            return_value=False
        ), patch.object(
            self.node, "_is_pending_command_rejected",
            return_value=False
        ), patch.object(
            self.node, "_is_handover_timed_out",
            return_value=False
        ):

            self.node._control_loop()

            monitor_mock.assert_called_once()
            mode_mock.assert_called_once_with(position=False)

            # 2. DONE 상태에서는 모든 제어 처리 중단
            self.node.state = FlightState.DONE

            self.node._control_loop()

            # 감시 함수와 Offboard 발행 횟수가 늘어나지 않아야 함
            monitor_mock.assert_called_once()
            mode_mock.assert_called_once_with(position=False)

    def test_track_stops_forward_command_when_vehicle_status_lost(self):
        """PX4 상태 통신 단절 중 TRACK 전진 명령을 차단한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        # TRACK 상태 설정
        self.node.state = FlightState.TRACK
        self.node.lock_on = True

        # 마지막 PX4 상태가 1.6초 전이라고 가정
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        # PX4 상태 통신 단절 확인
        self.assertTrue(
            self.node._is_vehicle_status_lost()
        )

        # YOLO 검출은 정상이라고 가정
        with patch.object(
            self.node, "_has_valid_detection",
            return_value=True
        ), patch.object(
            self.node, "_check_wind_safety",
            return_value=True
        ), patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 통신 단절 중 전진 속도 명령이 발행되면 안 됨
            for call in velocity_mock.call_args_list:
                args = call.args

                # 기본 인자 없이 호출했다면 제로 속도 명령
                if not args:
                    continue

                # NED 수평 속도 명령이 0인지 확인
                self.assertAlmostEqual(args[0], 0.0)
                self.assertAlmostEqual(args[1], 0.0)

    def test_search_stops_rotation_when_vehicle_status_lost(self):
        """PX4 상태 통신 단절 중 SEARCH 회전 명령을 차단한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.SEARCH

        # PX4 상태 메시지를 마지막으로 받은 지 1.6초 경과
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertTrue(
            self.node._is_vehicle_status_lost()
        )

        with patch.object(
            self.node, "_has_valid_detection",
            return_value=False
        ), patch.object(
            self.node, "_check_wind_safety",
            return_value=True
        ), patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 통신 단절 시 제로 속도 명령을 발행해야 함
            velocity_mock.assert_called_once_with()

    def test_intercept_stops_when_vehicle_status_lost(self):
        """PX4 상태 통신 단절 중 INTERCEPT 이동 명령을 차단한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.INTERCEPT

        # 마지막 PX4 상태 메시지 수신 후 1.6초 경과
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertTrue(self.node._is_vehicle_status_lost())

        with patch.object(
            self.node, "_has_valid_detection",
            return_value=True
        ), patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_compute_velocity_cmd",
            return_value=(4.0, 0.0, 0.0, 0.0)
        ):

            self.node._control_loop()

            # 통신 단절 시 제로 속도 명령을 발행해야 함
            velocity_mock.assert_called_once_with()

    def test_track_does_not_resume_after_vehicle_status_recovery(self):
        """PX4 통신 복구 직후 TRACK 전진 명령 자동 재개를 차단한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.TRACK
        self.node.lock_on = True

        # 1. 정상적인 PX4 상태 메시지 수신
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        # 2. 마지막 수신 시각을 1.6초 전으로 변경하여 통신 단절 재현
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertTrue(self.node._is_vehicle_status_lost())

        # 단절 상태를 감시 함수에 반영
        self.node._monitor_vehicle_status()
        self.assertTrue(self.node.vehicle_status_lost_reported)

        # 3. PX4 상태 메시지가 다시 도착했다고 가정
        self.node._cb_status(status)
        self.node._monitor_vehicle_status()

        self.assertTrue(self.node._is_vehicle_status_fresh())
        self.assertFalse(self.node.vehicle_status_lost_reported)

        # 4. 이전 표적 검출 정보가 여전히 유효한 상황 구성
        with patch.object(
            self.node, "_has_valid_detection",
            return_value=True
        ), patch.object(
            self.node, "_check_wind_safety",
            return_value=True
        ), patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_compute_velocity_cmd",
            return_value=(1.5, 0.0, 0.0, 0.0)
        ):

            self.node._control_loop()

            # 통신 복구 직후 전진 명령이 자동 재개되면 안 됨
            for call in velocity_mock.call_args_list:
                args = call.args
                if args:
                    self.assertAlmostEqual(args[0], 0.0)
                    self.assertAlmostEqual(args[1], 0.0)

            # 기존 추적 상태를 그대로 유지하면서 전진하지 않는지만
            # 확인하는 것이 아니라, 안전한 상태 전환도 요구한다.
            self.assertNotEqual(self.node.state, FlightState.TRACK)

    def test_recovery_hold_persists_after_status_recovery(self):
        """PX4 통신 복구 후에도 RECOVERY_HOLD를 유지하고 추적 명령을 차단한다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        # 이미 통신 장애로 안전 대기 상태에 진입한 상황
        self.node.state = FlightState.RECOVERY_HOLD
        self.node.lock_on = True
        self.node.near_2m = True

        # PX4 통신이 복구되어 상태 메시지가 정상 수신됨
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node._cb_status(status)

        self.node.vehicle_status_lost_reported = True

        with patch.object(
            self.node, "_has_valid_detection",
            return_value=True
        ), patch.object(
            self.node, "_publish_offboard_mode"
        ), patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock, patch.object(
            self.node, "_compute_velocity_cmd"
        ) as compute_mock:

            # 통신 복구 이후 제어 루프 5회 실행
            for _ in range(5):
                self.node._control_loop()

                # 매 주기 안전 대기 상태 유지
                self.assertEqual(
                    self.node.state,
                    FlightState.RECOVERY_HOLD
                )

            # 매 주기 제로 속도 목표를 정확히 한 번 발행
            self.assertEqual(velocity_mock.call_count, 5)

            for call in velocity_mock.call_args_list:
                self.assertEqual(call.args, ())
                self.assertEqual(call.kwargs, {})

            # 추적 및 요격 속도 계산은 실행하지 않아야 함
            compute_mock.assert_not_called()

        # 통신 복구 로그 처리 후에도 안전 대기 상태 유지
        self.assertFalse(self.node.vehicle_status_lost_reported)

    def test_recovery_hold_does_not_publish_offboard_when_px4_rtl(self):
        """PX4가 RTL 모드이면 RECOVERY_HOLD에서 Offboard 명령을 발행하지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        # 통신 장애 이후 안전 대기 상태
        self.node.state = FlightState.RECOVERY_HOLD

        # PX4 상태 메시지는 최신이며, 실제 모드는 AUTO_RTL
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)

        self.assertTrue(self.node._is_vehicle_status_fresh())

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as offboard_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # ROS 2 내부 상태는 안전 대기 유지
            self.assertEqual(
                self.node.state,
                FlightState.RECOVERY_HOLD
            )

            # PX4가 이미 RTL이면 새로운 Offboard 제어 신호 차단
            offboard_mock.assert_not_called()

            # 새로운 속도 목표도 발행하지 않음
            velocity_mock.assert_not_called()

    def test_recovery_hold_does_not_publish_offboard_when_px4_land(self):
        """PX4가 AUTO_LAND이면 RECOVERY_HOLD에서 Offboard 명령을 발행하지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RECOVERY_HOLD

        # 최신 PX4 상태 메시지: AUTO_LAND
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self.node._cb_status(status)

        self.assertTrue(self.node._is_vehicle_status_fresh())

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as offboard_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 내부 안전 대기 상태 유지
            self.assertEqual(
                self.node.state,
                FlightState.RECOVERY_HOLD
            )

            # PX4가 착륙 중이면 Offboard 신호를 발행하지 않음
            offboard_mock.assert_not_called()

            # 새로운 속도 목표도 발행하지 않음
            velocity_mock.assert_not_called()

    def test_recovery_hold_does_not_publish_offboard_when_px4_posctl(self):
        """PX4가 POSCTL이면 RECOVERY_HOLD에서 Offboard 명령을 발행하지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RECOVERY_HOLD

        # 최신 PX4 상태 메시지: POSCTL
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.node._cb_status(status)

        self.assertTrue(self.node._is_vehicle_status_fresh())

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as offboard_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 내부 안전 대기 상태 유지
            self.assertEqual(
                self.node.state,
                FlightState.RECOVERY_HOLD
            )

            # PX4가 위치 제어 모드이면 Offboard 신호를 발행하지 않음
            offboard_mock.assert_not_called()

            # 새로운 속도 목표도 발행하지 않음
            velocity_mock.assert_not_called()

    def test_recovery_hold_does_not_publish_offboard_when_px4_altctl(self):
        """PX4가 ALTCTL이면 RECOVERY_HOLD에서 Offboard 명령을 발행하지 않는다."""
        from unittest.mock import patch
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RECOVERY_HOLD

        # 최신 PX4 상태 메시지: ALTCTL
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_ALTCTL
        self.node._cb_status(status)

        self.assertTrue(self.node._is_vehicle_status_fresh())

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as offboard_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 내부 안전 대기 상태 유지
            self.assertEqual(
                self.node.state,
                FlightState.RECOVERY_HOLD
            )

            # PX4가 고도 제어 모드이면 Offboard 신호를 발행하지 않음
            offboard_mock.assert_not_called()

            # 새로운 속도 목표도 발행하지 않음
            velocity_mock.assert_not_called()

    def test_recovery_hold_with_stale_rtl_status_current_behavior(self):
        """오래된 RTL 상태 정보만 있을 때 현재 명령 발행 동작을 확인한다."""
        from unittest.mock import patch
        from rclpy.duration import Duration
        from px4_msgs.msg import VehicleStatus

        self.node.state = FlightState.RECOVERY_HOLD

        # 마지막으로 수신한 PX4 상태는 AUTO_RTL
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
        self.node._cb_status(status)

        # 마지막 상태 수신 시각을 1.6초 전으로 설정
        self.node.last_vehicle_status_time = (
            self.node.get_clock().now()
            - Duration(seconds=1.6)
        )

        self.assertFalse(self.node._is_vehicle_status_fresh())

        with patch.object(
            self.node, "_publish_offboard_mode"
        ) as offboard_mock, patch.object(
            self.node, "_publish_velocity_sp"
        ) as velocity_mock:

            self.node._control_loop()

            # 통신 단절 후에도 안전 대기 상태 유지
            self.assertEqual(
                self.node.state,
                FlightState.RECOVERY_HOLD
            )

            # 현재 구현의 명령 발행 동작 확인
            offboard_mock.assert_called_once_with(position=False)
            velocity_mock.assert_called_once()

if __name__ == "__main__":
    unittest.main(verbosity=2)
