YOLOv8n 기반 풍선 표적 인식 및 ROS2 Topic 출력 결과 정리
1. 작업 목적

본 작업의 목적은 Gazebo 시뮬레이션 환경에서 드론 카메라 영상을 이용하여 검붉은색 풍선 표적을 인식하고, 이후 제어 파트에서 사용할 수 있는 표적 위치 정보를 ROS2 topic으로 제공하는 것이다.

객체인식 파트의 최종 목표는 단순히 풍선 bbox를 화면에 표시하는 것이 아니라, 제어 파트가 활용할 수 있도록 다음 정보를 안정적으로 산출하는 것이다.

1. 표적 검출 여부
2. YOLO confidence
3. bbox 좌표
4. bbox 중심점
5. 화면 중심 대비 표적 오차 dx, dy
6. lock-on 여부
7. bbox 기반 거리 추정값
8. 2m 근접 알람
2. 개발 및 실행 환경
OS: Ubuntu 22.04
ROS2: Humble
Simulator: Gazebo Sim
PX4 SITL 사용
Model: YOLOv8n
GPU: NVIDIA GeForce RTX 4070
Input topic: /camera/image
Image size: 1280x960
Image encoding: rgb8
Camera rate: 약 30fps
3. 카메라 파이프라인 구축

초기에는 Gazebo 카메라 영상이 ROS2 /camera/image로 정상 수신되지 않는 문제가 있었다. 주요 원인은 다음 두 가지였다.

1. NVIDIA 드라이버 DKMS 빌드 실패로 인한 렌더링 환경 문제
2. Gazebo/PX4 실행 터미널과 확인용 터미널의 GZ_IP 설정 불일치

최종적으로 아래 환경변수를 설정하여 Gazebo transport 통신 문제를 해결했다.

export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1

해결 후 다음 사항을 확인했다.

- Gazebo 원본 카메라 topic에서 Hz 출력 확인
- ROS2 /camera/image topic 수신 확인
- rqt_image_view에서 드론 카메라 영상 확인
- image_probe에서 1280x960 rgb8, 약 30fps 확인

이를 통해 Gazebo 카메라에서 ROS2 topic까지의 영상 입력 파이프라인이 정상적으로 동작함을 확인했다.

4. 데이터셋 수집 및 자동 라벨링

수동 캡처 및 수동 라벨링 대신, ROS2 /camera/image를 구독하여 이미지를 자동 저장하고 bbox 라벨을 자동 생성하는 스크립트를 구현했다.

사용 스크립트:

record_balloon_dataset.py

주요 기능은 다음과 같다.

- /camera/image 구독
- ROS Image 메시지를 OpenCV 이미지로 변환
- 검붉은색 풍선 영역 탐지
- bbox 자동 생성
- YOLO 형식 txt 라벨 자동 저장

YOLO 데이터셋 구조는 다음과 같이 구성했다.

dataset/
  images/
    train/
    val/
  labels/
    train/
    val/
  data.yaml

라벨 형식은 YOLO 기본 형식을 사용했다.

class_id x_center y_center width height

풍선 클래스는 하나만 사용했기 때문에 class id는 0으로 설정했다.

0 = balloon
5. 데이터 보강

초기 학습 결과 풍선은 잘 검출되었으나, 일부 장면에서 풍선 그림자를 오검출하는 문제가 있었다. 또한 풍선이 2m 근처로 가까워져 화면에서 매우 크게 보일 경우 bbox가 불안정해지는 문제가 있었다.

이를 보완하기 위해 다음 데이터를 추가 수집 및 생성했다.

1. 풍선 + 그림자가 같이 보이는 이미지
   - 라벨은 풍선에만 부여

2. 그림자만 보이는 negative sample
   - 이미지는 저장
   - 라벨 txt 파일은 빈 파일로 생성

3. 풍선이 작게 보이는 이미지

4. 화면 가장자리 또는 일부 잘린 풍선 이미지

5. motion blur 증강 이미지

6. 근접 풍선 전체 이미지

7. 근접 풍선 일부 잘림 이미지

negative sample은 다음과 같은 방식으로 구성했다.

images/train/negative_000001.jpg
labels/train/negative_000001.txt  ← 빈 파일

이 방식은 YOLO에게 “그림자는 풍선이 아니다”라는 정보를 학습시키기 위한 hard negative sample로 사용했다.

6. YOLOv8n 학습 결과

YOLOv8n을 사용하여 풍선 표적 검출 모델을 학습했다. 초기 모델 학습 후 그림자 오검출 및 근접 풍선 검출 문제를 보완하기 위해 데이터셋을 확장하고 v2, v3 모델을 재학습했다.

최종 사용 모델:

runs/detect/balloon_yolov8n_v3/weights/best.pt

확인 결과는 다음과 같다.

- 검증 이미지에서 풍선 bbox 정상 검출
- 실시간 카메라 영상에서 풍선 bbox 정상 검출
- 그림자 오검출 감소 확인
- 2m 근처 근접 상황에서도 bbox 유지 확인

초기 모델에서는 그림자를 풍선으로 잡는 경우가 있었으나, negative sample과 그림자 포함 데이터를 추가한 뒤 해당 문제가 개선되었다.

7. 실시간 YOLO 검출 노드 구현

YOLO CLI는 ROS2 topic을 직접 입력으로 받을 수 없기 때문에, 별도의 ROS2 기반 실시간 검출 노드를 구현했다.

사용 스크립트:

yolo_live_detect.py

주요 기능은 다음과 같다.

- /camera/image 구독
- YOLOv8n best.pt 모델로 실시간 추론
- bbox 화면 표시
- bbox 중심점 계산
- 화면 중심과 bbox 중심의 오차 계산
- lock-on 판단
- bbox 기반 거리 추정
- 2m 근접 알람 판단
- 결과를 ROS2 topic으로 publish
8. bbox 안정화 및 오검출 필터링

실시간 검출 결과가 흔들리거나 오검출되는 문제를 줄이기 위해 다음 안정화 로직을 구현했다.

1. confidence threshold
2. bbox 내부 red_ratio 필터
3. bbox 크기 및 비율 필터
4. jump filter
5. EMA smoothing
6. miss 발생 시 tracking state reset

각 기능의 목적은 다음과 같다.

항목	목적
confidence threshold	낮은 신뢰도의 검출 제거
red_ratio 필터	그림자, 바닥 얼룩 등 비붉은 객체 제거
bbox 크기/비율 필터	비정상적으로 크거나 납작한 bbox 제거
jump filter	갑자기 튀는 bbox 제거
EMA smoothing	bbox 중심점 흔들림 완화
miss reset	표적을 놓친 뒤 다시 나타났을 때 추적 상태 복구

이를 통해 bbox가 더 안정적으로 표적을 따라가고, lock-on 상태가 순간적으로 흔들리는 문제를 줄였다.

9. Lock-on 정의

현재 구현한 lock-on은 거리 조건이 아니라, 카메라 화면 중심 기준 조준 안정 상태를 의미한다.

정의는 다음과 같다.

lock_on = 풍선이 YOLO로 안정적으로 검출되고,
          bbox 중심이 화면 중심 근처에 일정 시간 유지되는 상태

즉, lock_on과 near_2m은 서로 다른 의미를 가진다.

lock_on  = 표적이 화면 중심에 안정적으로 들어온 상태
near_2m  = 표적이 추정 거리 기준 2m 이내에 들어온 상태

예를 들어, 표적이 화면 중앙에 잘 잡혀 있지만 멀리 있다면 다음과 같이 판단된다.

lock_on = true
near_2m = false

반대로 표적이 가까이 있지만 화면 중심에서 벗어나 있다면 다음과 같이 판단될 수 있다.

lock_on = false
near_2m = true
10. bbox 기반 거리 추정 및 2m 알람

풍선의 실제 지름과 카메라 FOV를 이용하여 bbox 크기 기반 거리 추정을 구현했다.

기본 가정은 다음과 같다.

풍선 반지름: 0.5m
풍선 지름: 1.0m
카메라 horizontal FOV: 1.74 rad
이미지 너비: 1280 px

초점거리 픽셀값은 다음과 같이 계산한다.

fx = image_width / (2 * tan(horizontal_fov / 2))

거리 추정식은 다음과 같다.

distance_m = balloon_diameter_m * fx / bbox_width_px

즉, bbox가 커질수록 표적이 가까운 것으로 추정한다.

bbox width 증가 → distance_m 감소
bbox width 감소 → distance_m 증가

2m 알람은 다음 조건으로 판단한다.

near_2m = true if distance_m <= 2.0
near_2m = false if distance_m > 2.0 or target not detected

주의할 점은 이 거리값이 정밀 거리센서 값은 아니라는 것이다. 단안 카메라와 bbox 크기를 이용한 추정값이므로, 본 시스템에서는 2m 근접 여부 판단을 위한 보조 지표로 사용한다.

11. ROS2 Topic 명세
11.1 입력 Topic
/camera/image
Type: sensor_msgs/msg/Image
설명: Gazebo 카메라에서 bridge된 드론 카메라 영상
해상도: 1280x960
encoding: rgb8
주기: 약 30fps
11.2 출력 Topic
/target/balloon_bbox
Type: std_msgs/msg/Float32MultiArray
설명: 풍선 검출 결과와 제어용 정보를 포함하는 통합 topic

배열 구조는 다음과 같다.

data[0]  detected
data[1]  confidence
data[2]  x1
data[3]  y1
data[4]  x2
data[5]  y2
data[6]  cx
data[7]  cy
data[8]  dx
data[9]  dy
data[10] image_width
data[11] image_height
data[12] red_ratio
data[13] distance_m
data[14] near_2m
Index	이름	의미
0	detected	풍선 검출 여부. 검출 시 1.0, 미검출 시 0.0
1	confidence	YOLO confidence
2	x1	bbox 좌상단 x 좌표
3	y1	bbox 좌상단 y 좌표
4	x2	bbox 우하단 x 좌표
5	y2	bbox 우하단 y 좌표
6	cx	bbox 중심 x 좌표
7	cy	bbox 중심 y 좌표
8	dx	화면 중심 대비 x 방향 오차
9	dy	화면 중심 대비 y 방향 오차
10	image_width	이미지 가로 해상도
11	image_height	이미지 세로 해상도
12	red_ratio	bbox 내부 붉은 픽셀 비율
13	distance_m	bbox 기반 추정 거리, 단위 m
14	near_2m	2m 이내 여부. true면 1.0, false면 0.0
/target/lock_on
Type: std_msgs/msg/Bool
설명: 표적이 화면 중심 근처에 안정적으로 들어왔는지 여부

값 의미:

true  = 표적이 화면 중심 근처에서 안정적으로 검출됨
false = 표적이 없거나, 중심에서 벗어났거나, 안정적으로 유지되지 않음
/target/distance_m
Type: std_msgs/msg/Float32
설명: bbox 크기 기반 단안 거리 추정값
단위: meter

값 의미:

양수 값 = 추정 거리
-1.0 = 풍선 미검출 또는 거리 추정 불가

예:

data: 3.5   → 약 3.5m
data: 1.8   → 약 1.8m
data: -1.0  → 미검출 또는 거리 추정 불가
/target/near_2m
Type: std_msgs/msg/Bool
설명: 추정 거리 기준 2m 이내 접근 여부

값 의미:

true  = distance_m <= 2.0
false = distance_m > 2.0 또는 미검출
12. 제어 파트에서 활용 가능한 값

제어 파트에서 가장 직접적으로 활용할 값은 다음과 같다.

dx
dy
lock_on
distance_m
near_2m
dx, dy 의미

이미지 중심은 다음과 같다.

image_center_x = image_width / 2
image_center_y = image_height / 2

오차는 다음과 같이 계산된다.

dx = cx - image_center_x
dy = cy - image_center_y

해석:

dx > 0 → 표적이 화면 오른쪽에 있음
dx < 0 → 표적이 화면 왼쪽에 있음

dy > 0 → 표적이 화면 아래쪽에 있음
dy < 0 → 표적이 화면 위쪽에 있음

제어 파트에서는 이 값을 이용해 표적이 화면 중심에 오도록 yaw, pitch, 고도 또는 lateral 제어를 설계할 수 있다.

13. 동적 시나리오 검증

정적 장면뿐 아니라, 드론 이륙 후 호버링 상태에서 풍선을 수동으로 이동시키며 동적 입력 상황을 테스트했다.

테스트한 상황은 다음과 같다.

1. 드론 이륙 후 호버링 상태
2. 풍선이 화면 밖으로 완전히 나가는 상황
3. 풍선이 다시 화면 안으로 들어오는 상황
4. 풍선이 보이지 않을 정도로 멀어지는 상황
5. 풍선이 매우 가까워지는 상황
6. 풍선이 갑자기 시야 밖에서 등장하는 상황

검증 결과는 다음과 같다.

- 풍선이 사라질 경우 detected 값이 0으로 전환됨
- 풍선 미검출 시 distance_m = -1.0 출력
- 풍선이 다시 보이면 bbox가 복구됨
- bbox 중심 오차 dx, dy가 정상 출력됨
- 풍선이 가까워질수록 distance_m이 감소함
- 2m 근처에서 near_2m이 true로 전환됨
- lock_on이 중심 정렬 상태에서 true로 유지됨

따라서 객체인식 파트 단독 기준으로는 동적 입력 상황에 대한 1차 검증을 완료한 상태이다.

14. 현재 완료 상태
항목	상태
Gazebo 카메라 영상 수신	완료
ROS2 /camera/image 수신	완료
YOLOv8n 풍선 검출	완료
데이터셋 자동 수집/라벨링	완료
그림자 오검출 데이터 보강	완료
근접 풍선 데이터 보강	완료
실시간 bbox publish	완료
bbox 안정화	완료
lock-on 로직	완료
bbox 기반 거리 추정	완료
2m 근접 알람	완료
동적 입력 상황 테스트	완료
TensorRT 최적화	추후 진행
제어 파트 연동	대기
15. 남은 작업
15.1 제어 파트와 연동

객체인식 파트는 현재 제어 파트가 사용할 수 있는 입력값을 제공한다. 다음 단계는 제어 파트에서 아래 topic을 구독하여 실제 비행 제어에 활용하는 것이다.

/target/balloon_bbox
/target/lock_on
/target/distance_m
/target/near_2m

제어 파트는 우선 dx, dy를 이용해 표적을 화면 중심으로 정렬하는 로직을 구성할 수 있다. 이후 lock_on과 near_2m을 이용해 접근 및 요격 조건을 구성할 수 있다.

15.2 필요 시 전처리 및 필터 파라미터 조정

동적 제어 연동 과정에서 다음 문제가 발생할 수 있다.

bbox 중심 흔들림
일시적인 미검출
오검출
lock_on 상태 변동
거리 추정 오차

이 경우 다음 파라미터를 조정할 수 있다.

confidence threshold
red_ratio threshold
smooth_alpha
max_jump_px
center_tolerance
near_distance_m
15.3 TensorRT 최적화

TensorRT는 현재 즉시 진행하지 않는다. 최종 모델, 입력 해상도, 추론 주기, 탑재 하드웨어가 확정된 후 PyTorch 추론 속도가 부족할 경우 진행한다.

진행 시 목표는 다음과 같다.

PyTorch best.pt
→ ONNX 변환
→ TensorRT FP16 engine 변환
→ 추론 속도 비교
16. 최종 요약

현재 SW 객체인식 파트는 다음 흐름을 구현하고 검증했다.

Gazebo 카메라 영상
→ ROS2 /camera/image
→ YOLOv8n 풍선 검출
→ bbox 안정화 및 오검출 필터링
→ bbox 중심 오차 계산
→ lock-on 판단
→ bbox 기반 거리 추정
→ 2m 근접 알람
→ ROS2 topic publish

따라서 현재 상태는 제어 파트와 연동 가능한 객체인식 기반이 구축된 상태로 볼 수 있다.
