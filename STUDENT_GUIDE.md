# ROS 1 Noetic 터틀봇 표지판·신호등 인식 자율주행 패키지 제작 실습

## 1. 프로젝트 개요

### 1) 실습 목적
- YOLOv8 인식 결과를 커스텀 ROS 메시지(`DetectionArray`)로 퍼블리셔/서브스크라이버 간에 주고받는 구조를 이해합니다.
- 터틀봇 바퀴 모터 제어 표준 메시지인 `geometry_msgs/Twist`의 구조와 동작 방식을 이해합니다.
- 인식 결과(표지판/신호등)에 따라 상태머신이 판단해 자율주행하는 `sign_light_driving` 패키지를 완성합니다.

### 2) 시스템 통신 구조
- 노트북 (`detector_node.py`) : `/camera/image` 구독 → YOLO 추론 → `/sign_light/detections` 토픽(`DetectionArray`) 발행
- 무선 네트워크 : 노트북과 터틀봇이 서로 통신 가능한 같은 네트워크(공유기 Wi-Fi, VPN 등 무엇이든)에 연결된 ROS 멀티 머신 통신
- 터틀봇 (`controller_node.py`) : `/sign_light/detections` 토픽 수신 후 상태머신 판단 → `/cmd_vel` 토픽(`Twist`) 발행
- OpenCR 보드 (rosserial) : `/cmd_vel` 속도 지령값을 받아 터틀봇 좌/우 다이나믹셀 모터 구동
- 노트북 (`collector_node.py`) : 학습 데이터 수집 단계에서만 사용, `/camera/image`를 구독해 클래스별 이미지 저장

---

## 2. 패키지 구성 및 준비

### 1) 패키지 이름 : `sign_light_driving`
### 2) 대상 로봇 : 터틀봇 (TurtleBot3 Burger 호환, OpenCR + 다이나믹셀 XL430, Raspberry Pi 4)
### 3) 의존성 패키지 : `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `message_generation`/`message_runtime`

### 4) 노트북 측 필수 패키지 설치
노트북에서 YOLO 추론과 라벨링에 필요한 패키지를 설치합니다 (torch는 CPU판으로 함께 설치됩니다):

```bash
pip install ultralytics labelImg
```

### 5) 깃허브에서 패키지 다운로드 (Git Clone)
이미 완성된 저장소를 그대로 내려받아 실습할 경우 홈 디렉토리에서 실행합니다:

```bash
cd ~
git clone https://github.com/seongjun-k/tb_auto.git
```

### 6) 직접 패키지 생성하기 (처음부터 직접 만들 경우)
패키지를 기초부터 직접 만들어볼 경우 아래 명령어로 패키지와 폴더를 생성합니다:

```bash
cd ~/tb_auto/catkin_ws/src
catkin_create_pkg sign_light_driving rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_generation
cd sign_light_driving
mkdir scripts launch msg
```

---

## 3. 인식 결과 메시지 정의 (`Detection.msg` / `DetectionArray.msg`)

### 1) 소스 파일 경로 : `msg/Detection.msg`, `msg/DetectionArray.msg`
인식 노드와 제어 노드 사이를 오가는 커스텀 메시지입니다. 클래스명/신뢰도/bbox 비율(거리 대용값)을 담습니다:

```
# Detection.msg
string  label            # left_turn / right_turn / red_light / green_light
float32 confidence
float32 height_ratio     # bbox 높이 / 프레임 높이. 거리 대용값

# DetectionArray.msg
Header header
Detection[] detections
```

### 2) `CMakeLists.txt` / `package.xml` 반영
메시지를 빌드에 포함하려면 `CMakeLists.txt`에 `add_message_files`/`generate_messages`를,
`package.xml`에 `message_generation`(build_depend)/`message_runtime`(exec_depend)을 추가합니다
(저장소를 그대로 받았다면 이미 반영되어 있습니다).

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_generation
)
add_message_files(FILES Detection.msg DetectionArray.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime)
```

---

## 4. 노트북 데이터 수집 노드 작성 (`collector_node.py`)

### 1) 소스 파일 경로 : `scripts/collector_node.py`
### 2) 소스 코드 작성
로봇 카메라(`/camera/image`)를 구독해 화면에 보여주고, 숫자키로 클래스를 선택해 스페이스바로 저장하는 노드입니다:

```python
#!/usr/bin/env python3
"""데이터 수집 노드 - 노트북에서 실행.

영상은 로봇 카메라(aicon_camera.launch가 내보내는 /camera/image)를 구독한다.
화면 표시와 이미지 저장은 노트북에서 한다 - 찍는 장면을 보면서 모을 수 있고
이미지가 노트북에 바로 쌓여 scp가 필요 없다.

  roslaunch sign_light_driving collect.launch

키: 1~4 클래스 선택 / space 저장 / q 종료
"""
import os
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
DEFAULT_SAVE_DIR = "~/tb_auto/dataset_raw"


def main():
    rospy.init_node("collector")
    bridge = CvBridge()
    topic = rospy.get_param("~topic", "/camera/image")
    save_dir = os.path.expanduser(rospy.get_param("~save_dir", DEFAULT_SAVE_DIR))

    state = {"frame": None}
    rospy.Subscriber(topic, Image, lambda m: state.update(frame=bridge.imgmsg_to_cv2(m, "bgr8")))

    for c in CLASSES:
        os.makedirs(os.path.join(save_dir, c), exist_ok=True)
    # labelImg가 읽는 클래스 목록. 순서가 곧 YOLO 클래스 인덱스라 절대 바꾸지 말 것.
    with open(os.path.join(save_dir, "classes.txt"), "w") as f:
        f.write("\n".join(CLASSES) + "\n")
    counts = {c: len(os.listdir(os.path.join(save_dir, c))) for c in CLASSES}

    sel = 0
    rospy.loginfo("waiting for %s ... (save_dir=%s)", topic, save_dir)
    while not rospy.is_shutdown():
        frame = state["frame"]
        if frame is None:
            rospy.sleep(0.05)
            continue

        view = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        cv2.putText(view, "[%d] %s  saved=%d" % (sel + 1, CLASSES[sel], counts[CLASSES[sel]]),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("collect (1-4 class / space save / q quit)", view)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if ord("1") <= key <= ord("4"):
            sel = key - ord("1")
        elif key == ord(" "):
            name = CLASSES[sel]
            stamp = time.strftime("%Y%m%d_%H%M%S") + "_%03d" % (time.time() % 1 * 1000)
            path = os.path.join(save_dir, name, "%s_%s.jpg" % (name, stamp))
            cv2.imwrite(path, frame)   # 리사이즈 전 원본 320x240 저장
            counts[name] += 1
            rospy.loginfo("saved %s (%d)", path, counts[name])

    cv2.destroyAllWindows()
    print({c: counts[c] for c in CLASSES})


if __name__ == "__main__":
    main()
```

### 3) 실행 권한 부여 (필수)

```bash
chmod +x scripts/collector_node.py
```

---

## 5. 노트북 인식 노드 작성 (`detector_node.py`)

### 1) 소스 파일 경로 : `scripts/detector_node.py`
### 2) 소스 코드 작성
로봇 카메라 영상을 구독해 YOLO로 추론하고, `cmd_vel`은 만들지 않고 인식 결과만 발행하는 노드입니다:

```python
#!/usr/bin/env python3
"""인식 노드 - 노트북에서 실행 (로봇 roscore에 원격 접속).

로봇 카메라 영상을 구독해 YOLO로 추론하고 결과만 publish 한다.
cmd_vel은 만들지 않는다 - 그건 로봇의 controller_node 몫.

  roslaunch sign_light_driving detect.launch
"""
import os

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from sign_light_driving.msg import Detection, DetectionArray

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]


def main():
    rospy.init_node("detector")
    model_path = os.path.expanduser(
        rospy.get_param("~model", "~/tb_auto/runs/detect/runs/train/weights/best.pt"))
    model = YOLO(model_path)
    imgsz = rospy.get_param("~imgsz", 320)
    conf = rospy.get_param("~conf", 0.25)   # 최종 판단은 controller가 한다. 여기선 넉넉히.

    pub = rospy.Publisher("detections", DetectionArray, queue_size=1)
    bridge = CvBridge()

    # 첫 추론은 워밍업 때문에 수백 ms 걸린다. 미리 한 번 돌려서
    # 주행 시작 직후 controller 워치독이 헛발동하는 것을 막는다.
    model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=imgsz, verbose=False)

    def on_image(msg):
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        h = frame.shape[0]
        r = model.predict(frame, imgsz=imgsz, conf=conf, verbose=False)[0]
        out = DetectionArray()
        out.header = msg.header
        for b in r.boxes:
            out.detections.append(Detection(
                label=CLASSES[int(b.cls)],
                confidence=float(b.conf),
                height_ratio=float(b.xywh[0][3]) / h))
        pub.publish(out)

    rospy.Subscriber(rospy.get_param("~topic", "/camera/image"), Image, on_image, queue_size=1)
    rospy.loginfo("detector ready (%s)", model_path)
    rospy.spin()


if __name__ == "__main__":
    main()
```

### 3) 실행 권한 부여 (필수)

```bash
chmod +x scripts/detector_node.py
```

---

## 6. 터틀봇 제어 노드 작성 (`controller_node.py`)

### 1) 소스 파일 경로 : `scripts/controller_node.py`
### 2) 소스 코드 작성
인식 결과를 구독해 상태머신으로 판단하고 `/cmd_vel`을 발행하는 노드입니다. torch/ultralytics 의존이
없는 순수 로직이라 RPi4에서도 가볍게 돕니다:

```python
#!/usr/bin/env python3
"""제어 노드 - 터틀봇에서 실행.

노트북의 detector_node가 보내는 인식 결과를 구독해 cmd_vel을 만든다.
torch/ultralytics가 필요 없는 순수 로직이라 RPi4에서도 가볍게 돈다.

  roslaunch sign_light_driving control.launch

로직만 검증:  rosrun sign_light_driving controller_node.py --selftest
"""
import math
import sys

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]


class Controller:
    """검출 결과 -> (linear.x, angular.z). ROS 의존 없음 (그래서 테스트 가능).

    거리 센서 대신 bbox 높이 / 프레임 높이 비율을 근접도로 쓴다.
    파라미터 기본값은 실제 표지판 크기/조명에 맞춰 현장에서 튜닝할 것.
    """

    def __init__(self, conf=0.6, light_ratio=0.15, turn_ratio=0.30, debounce=3,
                 speed=0.1, turn_z=0.5, turn_angle=math.pi / 2, sign_cooldown=2.0):
        self.conf = conf
        self.ratio = {"red_light": light_ratio, "green_light": light_ratio,
                      "left_turn": turn_ratio, "right_turn": turn_ratio}
        self.debounce = debounce
        self.speed = speed
        self.turn_z = turn_z
        # ponytail: 오도메트리/IMU 피드백 없이 시간 적분으로 각도를 맞추는 열린루프 방식.
        # 바퀴 슬립 등으로 오차가 누적되면 실제 yaw 피드백(오도메트리) 기반 회전으로 교체.
        self.turn_time = turn_angle / turn_z
        self.sign_cooldown = sign_cooldown

        self.stopped = True  # 초록불을 봐야 출발. 빨간불/미검출 상태로 시작
        self.turn_until = None
        self.turn_dir = 0
        self.ignore_signs_until = 0.0
        self._streak_cls = None
        self._streak_n = 0

    def _confirmed(self, dets):
        """conf/근접도 통과 검출 중 최고 conf 클래스가 연속 debounce 프레임이면 반환."""
        ok = [d for d in dets if d[1] >= self.conf and d[2] >= self.ratio[d[0]]]
        cls = max(ok, key=lambda d: d[1])[0] if ok else None
        if cls == self._streak_cls:
            self._streak_n += 1
        else:
            self._streak_cls, self._streak_n = cls, 1
        return cls if cls is not None and self._streak_n >= self.debounce else None

    def step(self, dets, now):
        # 회전 중에는 새 검출 무시 (상태머신 락)
        if self.turn_until is not None:
            if now < self.turn_until:
                return (0.0, self.turn_dir * self.turn_z)
            self.turn_until = None
            # 돌고 난 뒤에도 같은 표지판이 시야에 남아 또 도는 것 방지
            self.ignore_signs_until = now + self.sign_cooldown

        cls = self._confirmed(dets)
        if cls == "red_light":
            self.stopped = True
        elif cls == "green_light":
            self.stopped = False
        elif cls in ("left_turn", "right_turn") and now >= self.ignore_signs_until:
            self.turn_until = now + self.turn_time
            self.turn_dir = 1 if cls == "left_turn" else -1
            self.stopped = False
            return (0.0, self.turn_dir * self.turn_z)

        return (0.0, 0.0) if self.stopped else (self.speed, 0.0)


def selftest():
    """ROS 없이 상태머신 로직만 검증. rosrun ... --selftest 로 실행."""
    def feed(c, dets, n, t0=0.0, dt=0.1):
        out = None
        for i in range(n):
            out = c.step(dets, t0 + i * dt)
        return out

    red = [("red_light", 0.9, 0.2)]
    green = [("green_light", 0.9, 0.2)]
    left = [("left_turn", 0.9, 0.4)]

    # 시작은 정지 상태. 초록불을 봐야 출발
    c = Controller()
    assert c.step([], 0.0) == (0.0, 0.0)
    assert feed(c, green, 3, t0=0.1) == (0.1, 0.0)

    # 빨간불 -> 정지, 좌회전은 신호와 무관하게 즉시 90도 회전 후 직진 복귀
    c = Controller()
    assert feed(c, left, 3) == (0.0, 0.5)
    assert c.step([], 1.0) == (0.0, 0.5)          # 회전 중 무시
    assert c.step([], 5.5) == (0.1, 0.0)          # 회전(pi/0.5초) 끝나면 직진

    print("selftest ok")


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from sign_light_driving.msg import DetectionArray

    rospy.init_node("controller")
    watchdog = rospy.get_param("~watchdog", 0.5)
    ctrl = Controller(
        conf=rospy.get_param("~conf", 0.6),
        light_ratio=rospy.get_param("~light_ratio", 0.15),
        turn_ratio=rospy.get_param("~turn_ratio", 0.30),
        debounce=rospy.get_param("~debounce", 3),
        speed=rospy.get_param("~speed", 0.1),
        turn_z=rospy.get_param("~turn_z", 0.5),
        turn_angle=rospy.get_param("~turn_angle", math.pi / 2),
    )

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    last_msg = [0.0]

    def send(lin, ang):
        t = Twist()
        t.linear.x, t.angular.z = lin, ang
        pub.publish(t)

    def on_detections(msg):
        now = rospy.get_time()
        last_msg[0] = now
        dets = [(d.label, d.confidence, d.height_ratio) for d in msg.detections]
        send(*ctrl.step(dets, now))

    def on_watchdog(_):
        # 무선이 끊기거나 노트북 추론이 멈추면 로봇이 마지막 명령으로 계속 달린다.
        # 정지 판단을 로봇 자신이 하므로 링크가 죽어도 확실히 선다.
        if rospy.get_time() - last_msg[0] > watchdog:
            send(0.0, 0.0)

    rospy.Subscriber("detections", DetectionArray, on_detections, queue_size=1)
    rospy.Timer(rospy.Duration(0.1), on_watchdog)
    rospy.on_shutdown(lambda: send(0.0, 0.0))
    rospy.loginfo("controller ready (watchdog %.2fs)", watchdog)
    rospy.spin()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main()
```

### 3) 실행 권한 부여 (필수)

```bash
chmod +x scripts/controller_node.py
```

### 4) 제어 로직 단독 검증 (ROS 없이)
모터를 실제로 움직이기 전에 상태머신 로직만 따로 검증할 수 있습니다:

```bash
rosrun sign_light_driving controller_node.py --selftest
```

---

## 7. 런치(Launch) 파일 작성

### 1) 데이터 수집용 런치 파일 : `launch/collect.launch`
노트북에서 실행하며, 로봇 카메라 영상을 구독해 `collector_node`를 띄웁니다:

```xml
<launch>
  <!-- 데이터 수집. 노트북에서 실행한다.
       카메라는 로봇의 /camera/image, 화면과 저장은 노트북. -->
  <arg name="topic"    default="/camera/image"/>
  <arg name="save_dir" default="$(env HOME)/tb_auto/dataset_raw"/>

  <node pkg="sign_light_driving" type="collector_node.py" name="collector" output="screen" required="true">
    <param name="topic"    value="$(arg topic)"/>
    <param name="save_dir" value="$(arg save_dir)"/>
  </node>
</launch>
```

### 2) 인식용 런치 파일 : `launch/detect.launch`
노트북에서 실행하며, YOLO 추론 노드를 띄우고 결과를 `/sign_light/detections`로 리맵합니다:

```xml
<launch>
  <!-- 인식(추론). 노트북에서 실행한다. GPU가 없어도 320px에서 29fps 나온다. -->
  <arg name="model" default="$(env HOME)/tb_auto/runs/detect/runs/train/weights/best.pt"/>
  <arg name="topic" default="/camera/image"/>
  <arg name="imgsz" default="320"/>
  <arg name="conf"  default="0.25"/>

  <node pkg="sign_light_driving" type="detector_node.py" name="detector" output="screen" required="true">
    <param name="model" value="$(arg model)"/>
    <param name="topic" value="$(arg topic)"/>
    <param name="imgsz" value="$(arg imgsz)"/>
    <param name="conf"  value="$(arg conf)"/>
    <remap from="detections" to="/sign_light/detections"/>
  </node>
</launch>
```

### 3) 제어용 런치 파일 : `launch/control.launch`
터틀봇에서 실행하며, 인식 결과를 받아 실제 `cmd_vel`을 만드는 `controller_node`를 띄웁니다:

```xml
<launch>
  <!-- 주행 제어. 터틀봇에서 실행한다. 인식 결과를 받아 cmd_vel을 만든다.
       torch 불필요 - 순수 파이썬 상태머신이라 RPi4에서 가볍다. -->
  <arg name="conf"        default="0.6"/>
  <arg name="light_ratio" default="0.15"/>
  <arg name="turn_ratio"  default="0.30"/>
  <arg name="debounce"    default="3"/>
  <arg name="speed"       default="0.1"/>
  <arg name="turn_z"      default="0.5"/>
  <!-- 좌/우회전은 항상 이 각도(라디안, 기본 90도)만큼 회전한다 -->
  <arg name="turn_angle"  default="1.5707963"/>
  <!-- 인식 결과가 이 시간 이상 끊기면 로봇이 스스로 정지. 무선이 끊겨도 확실히 선다. -->
  <arg name="watchdog"    default="0.5"/>

  <node pkg="sign_light_driving" type="controller_node.py" name="controller" output="screen" required="true">
    <param name="conf"        value="$(arg conf)"/>
    <param name="light_ratio" value="$(arg light_ratio)"/>
    <param name="turn_ratio"  value="$(arg turn_ratio)"/>
    <param name="debounce"    value="$(arg debounce)"/>
    <param name="speed"       value="$(arg speed)"/>
    <param name="turn_z"      value="$(arg turn_z)"/>
    <param name="turn_angle"  value="$(arg turn_angle)"/>
    <param name="watchdog"    value="$(arg watchdog)"/>
    <remap from="detections" to="/sign_light/detections"/>
  </node>
</launch>
```

---

## 8. 패키지 빌드 및 환경변수 등록 (Build)

### 1) 워크스페이스로 이동하여 빌드 수행 (`catkin_make`)
패키지(특히 커스텀 메시지) 작성이 끝나면 반드시 catkin 워크스페이스 루트에서 빌드해야 ROS가 인식합니다:

```bash
cd ~/tb_auto/catkin_ws
catkin_make
```

### 2) 빌드 환경변수 적용 (source)
빌드 후 생성된 실행 환경을 현재 터미널에 반영합니다:

```bash
source devel/setup.bash
```

### 3) 패키지 인식 확인 (`rospack`)
ROS 패키지 시스템에 `sign_light_driving` 패키지가 정상적으로 등록되었는지 확인합니다:

```bash
rospack find sign_light_driving
```
- 출력 결과로 `/home/.../tb_auto/catkin_ws/src/sign_light_driving` 경로가 나타나면 정상적으로 빌드 및 등록이 완료된 것입니다.

---

## 9. 네트워크 환경 설정 (노트북과 터틀봇 무선 연결)

### 1) 내 IP 확인하기
노트북과 터틀봇 각각 터미널에서 자신의 IP 주소를 확인합니다:

```bash
hostname -I
```

### 2) 환경변수 설정 원리
- `ROS_MASTER_URI` : roscore가 실행 중인 컴퓨터의 IP 주소 (양쪽 모두 동일하게 입력)
- `ROS_IP` : 현재 명령어를 입력하고 있는 컴퓨터 본인의 IP 주소
- 주의: 터틀봇에도 `ROS_IP`를 반드시 걸어야 합니다. 안 걸면 로봇 노드가 자기 주소를 `http://ubuntu:<port>/`로
  광고하는데, 노트북이 `ubuntu`를 이름풀이하지 못해 `rostopic list`엔 토픽이 보여도 메시지가 한 장도
  들어오지 않습니다.

### 3) 터틀봇(roscore) 환경변수 등록

```bash
export ROS_MASTER_URI=http://<터틀봇IP>:11311
export ROS_IP=<터틀봇IP>
```

### 4) 노트북 환경변수 등록

```bash
export ROS_MASTER_URI=http://<터틀봇IP>:11311
export ROS_IP=<노트북IP>
```

---

## 10. 데이터 수집 → 라벨링 → 분할 → 학습 실습

### 1) 로봇 브링업 및 데이터 수집

```bash
# 터틀봇
roslaunch aicon_bringup aicon_robot.launch
roslaunch aicon_bringup aicon_camera.launch
```

```bash
# 노트북
roslaunch sign_light_driving collect.launch
```
키: `1`~`4` 클래스 선택 / `space` 저장 / `q` 종료 → `dataset_raw/{class}/*.jpg`. 목표: 클래스당 200장.

### 2) 라벨링 (LabelImg)

```bash
labelImg dataset_raw dataset_raw/classes.txt dataset_raw/labels
```
포맷을 **YOLO**로 설정하고 `Auto Save mode`를 켠 뒤, **`Use default label`**에 현재 폴더 클래스명을
넣고 `w`(박스)/`d`(다음)/`a`(이전)로 진행합니다. `classes.txt`를 인자로 넘기지 않으면 세션마다 클래스
인덱스가 달라져 데이터셋이 조용히 망가지므로 반드시 넘겨야 합니다.

### 3) train/val 분할

```bash
python3 split.py        # dataset/{images,labels}/{train,val} + data.yaml, 8:2
```

### 4) YOLO 학습

```bash
yolo detect train model=yolov8n.pt data=dataset/data.yaml \
  epochs=100 imgsz=320 batch=16 patience=20 device=cpu workers=8
```
`imgsz=320`은 카메라 원본 해상도와 동일합니다. 결과: `runs/detect/train/weights/best.pt`
직접 학습이 어렵다면 학습된 체크포인트(mAP50 0.995)를 GitHub Release에서 바로 받을 수 있습니다:
https://github.com/seongjun-k/tb_auto/releases/tag/v1.0-model

---

## 11. 터틀봇 자율주행 실습

### 1) 터틀봇에서 제어 노드 실행
터틀봇에 SSH 접속해 아래 명령을 실행합니다:

```bash
roslaunch sign_light_driving control.launch
```

### 2) 노트북에서 인식 노드 실행
노트북 터미널에서 학습된 모델로 인식 노드를 실행합니다:

```bash
roslaunch sign_light_driving detect.launch model:=<best.pt 경로>
```

### 3) 인식 결과 → 동작 가이드
`controller_node`가 인식 결과를 받아 아래 표대로 판단해 즉시 반응합니다(엔터/키 입력 없이 자동):

| 인식 클래스 | 발동 조건 | 동작 | 상세 설명 |
|:---|:---|:---|:---|
| 미검출 | - | 직진 유지 | 직전 상태(정지/직진)를 그대로 유지 |
| **red_light** | conf≥0.6, bbox비율≥0.15, 연속 3프레임 | 정지 | `linear.x=0, angular.z=0` |
| **green_light** | conf≥0.6, bbox비율≥0.15, 연속 3프레임 | 출발/재출발 | `linear.x=speed(0.1m/s)`로 직진 시작 |
| **left_turn** | conf≥0.6, bbox비율≥0.30, 연속 3프레임 | 좌회전 | 신호와 무관하게 즉시 제자리 좌회전, `turn_angle`(기본 90도)만큼 |
| **right_turn** | conf≥0.6, bbox비율≥0.30, 연속 3프레임 | 우회전 | 신호와 무관하게 즉시 제자리 우회전, `turn_angle`(기본 90도)만큼 |

- 로봇은 시작하면 정지 상태이며 초록불을 봐야 출발합니다.
- 좌/우회전은 회전 중 새 검출을 무시하고, 회전 직후 2초 쿨다운을 둬 같은 표지판이 다시 회전을
  발동시키지 않습니다.

### 4) 안전 워치독
인식 메시지가 0.5초 이상 끊기면(무선 끊김, 노트북 다운, 추론 정지 등) `controller_node`가 스스로
정지 명령을 보냅니다. 원격 추론 구조이므로 이 로직은 절대 건드리지 않습니다.
