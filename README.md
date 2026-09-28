# tb_auto: 터틀봇 표지판/신호등 자율주행

## 개요

터틀봇 카메라 영상으로 `left_turn`, `right_turn`, `red_light`, `green_light`
4개 클래스를 학습한 YOLOv8 검출기를 이용해 로봇이 스스로 주행하는 ROS 1
Noetic 패키지다. 추론은 노트북에서 원격으로 돈다(터틀봇 RPi4로는 실시간 YOLO가
무리). 실제 모터 제어는 의존성 없는 순수 상태머신으로 로봇 위에서 직접
돌기 때문에, 무선 연결이 끊겨도 로봇이 스스로 안전하게 정지한다. 계획/결정
과정은 `PLAN.md`에 있고, 이 파일은 구조와 실사용 절차만 다룬다.

## 주요 특징

- **노드 3분할 구조**: `collector_node`/`detector_node`(노트북, torch/
  ultralytics 필요)와 `controller_node`(로봇, 순수 파이썬)가
  `/sign_light/detections` 토픽으로만 통신한다. 로봇 쪽에는 GPU도 무거운
  ML 스택도 필요 없다.
- **거리센서 없이 근접도 판단**: 거리 센서 대신 bbox 높이/프레임 높이
  비율로 근접도를 대신하고, 클래스별 임계값은 실제 카메라 화면 기준으로
  튜닝한다.
- **디바운스 처리**: 같은 클래스가 N프레임 연속으로 잡혀야 동작이
  발동한다. 단발성 오검출 하나로 회전하거나 멈추지 않는다.
- **정확한 각도 회전**: `left_turn`/`right_turn`은 정확히
  `turn_angle / turn_z`초 동안 회전한다(기본값 90도). 회전이 끝나면
  다음 프레임부터 다시 신호등 상태에 따라 동작한다.
- **워치독으로 스스로 정지**: `watchdog`초 동안 검출 메시지가 안 오면
  (무선 끊김, 노트북 다운, 추론 멈춤 등) 로봇이 스스로 속도를 0으로
  만든다. 원격 쪽이 "정지"를 알려주길 기다리지 않는다.
- **명령 단위 데이터셋 파이프라인**: 수집/라벨링/train-val 분할이 각각
  독립적으로 반복 실행 가능한 단계로 나뉘어 있다
  (`collect.launch` -> LabelImg -> `split.py` -> `yolo detect train`).

## 레포 구조

```
catkin_ws/src/sign_light_driving/
  scripts/collector_node.py    데이터 수집 노드 (노트북)
  scripts/detector_node.py     YOLO 인식 노드, 감지 결과만 publish (노트북)
  scripts/controller_node.py   감지 결과 -> cmd_vel 상태머신 (로봇)
  launch/collect.launch
  launch/detect.launch
  launch/control.launch
split.py  autolabel.py         데이터셋 도구 (ROS 노드 아님)
dataset_raw/  dataset/  runs/  데이터와 학습 결과
```

워크스페이스 빌드 (노트북과 로봇 양쪽, 최초 1회):
```bash
cd ~/tb_auto/catkin_ws && catkin_make
echo 'source ~/tb_auto/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

## 실행 순서

**1. 로봇 띄우기** (`ssh ubuntu@100.88.31.34`, 터미널 2개).
`ROS_IP`는 로봇에도 반드시 export 해야 한다:
```bash
export ROS_IP=100.88.31.34
roslaunch aicon_bringup aicon_robot.launch
```
```bash
export ROS_IP=100.88.31.34
roslaunch aicon_bringup aicon_camera.launch      # /camera/image, 320x240@30
```

**2. 노트북 터미널 설정** (터미널마다 반복):
```bash
source /opt/ros/noetic/setup.bash
source ~/tb_auto/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://100.88.31.34:11311
export ROS_IP=100.76.204.28                      # 노트북 tailscale IP
```
최초 1회 설치: `pip install ultralytics labelImg` (torch는 CPU판으로 딸려옴).

**3. 수집** (노트북, 이미지가 바로 로컬에 쌓여 scp 불필요):
```bash
roslaunch sign_light_driving collect.launch
```
키: `1`~`4` 클래스 선택, `space` 저장, `q` 종료 -> `dataset_raw/{class}/*.jpg`.
목표: 클래스당 200장, 각도·거리·조명 섞어서.

**4. 라벨링**:
```bash
labelImg dataset_raw dataset_raw/classes.txt dataset_raw/labels
```
포맷을 **YOLO**로 바꾸고(좌하단 버튼, 기본값은 PascalVOC), `View > Auto Save
mode`를 켜고, **`Use default label`**을 체크해서 `d`/`a`로 넘기는 동안 박스마다
라벨이 자동으로 붙게 한다(방향키는 박스를 미세이동시킬 뿐 다음 이미지로
안 넘어간다). `classes.txt`는 반드시 인자로 넘겨야 한다 — 안 넘기면
LabelImg가 세션마다 "처음 본 순서"로 클래스 번호를 매겨서 데이터셋이
조용히 망가진다.

**5. 분할**:
```bash
python3 split.py        # dataset/{images,labels}/{train,val} + data.yaml, 8:2
```

**6. 학습** (세션 끊겨도 안 죽는 터미널에서 실행 — 시간이 오래 걸린다):
```bash
yolo detect train model=yolov8n.pt data=dataset/data.yaml \
  epochs=100 imgsz=320 batch=16 patience=20 device=cpu workers=8
```
`imgsz=320`은 카메라 원본 해상도와 일치시킨 값이다. 640은 정보량 증가
없이 업스케일만 하는 셈이라 손해. 결과: `runs/detect/train/weights/best.pt`.

**7. 주행**:
```bash
roslaunch sign_light_driving detect.launch model:=<best.pt 경로>   # 노트북
roslaunch sign_light_driving control.launch                       # 로봇
```
동작: 시작하면 정지 상태(초록불을 봐야 출발) / `red_light` -> 정지 /
`green_light` -> 재출발 / `left_turn`·`right_turn` -> 신호 상태와 무관하게
즉시 `turn_angle`만큼 제자리 회전 후 직진 복귀(회전 중엔 새 검출 무시,
회전 직후 쿨다운으로 같은 표지판이 다시 회전을 발동시키지 않음).

제어 로직만 ROS 없이 검증:
```bash
rosrun sign_light_driving controller_node.py --selftest
```

## 환경 설정 시 주의점

**`ROS_IP`는 노트북뿐 아니라 로봇에도 걸어야 한다.** 안 걸면 로봇 노드가
자기 주소를 `http://ubuntu:<port>/`로 광고하는데, 노트북이 `ubuntu`를
이름풀이하지 못해서 **`rostopic list`엔 토픽이 보여도 메시지는 한 장도
안 들어온다.** 로봇에 `ROS_IP=100.88.31.34`를 걸면 IP로 광고해서 바로
붙는다.

카메라 bringup이 `cv camera open failed: device_id 0`로 죽으면 이전
`cv_camera_node`가 아직 `/dev/video0`을 놓지 않은 것이다. `pkill -f
cv_camera_node` 후 몇 초 뒤 다시 띄우면 된다.

튜닝 파라미터 (전부 `control.launch`의 arg, `roslaunch ... 이름:=값`으로
덮어쓴다):

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `conf` | 0.6 | 이 미만 신뢰도의 검출은 폐기 |
| `light_ratio` | 0.15 | 신호등 bbox높이/240이 이 값 이상이어야 반응 |
| `turn_ratio` | 0.30 | 표지판이 이만큼 가까워야(bbox 비율) 회전 발동 |
| `debounce` | 3 | 연속 N프레임 같은 클래스여야 동작 발동 |
| `speed` | 0.1 | 기본 직진 속도, m/s |
| `turn_z` | 0.5 | 회전 속도, rad/s |
| `turn_angle` | pi/2 | 목표 회전 각도, rad (회전 시간 = turn_angle/turn_z) |
| `watchdog` | 0.5 | 검출 메시지가 이 시간(초) 이상 안 오면 강제 정지 |

**워치독 로직은 건드리지 말 것.** 추론이 무선으로 원격에서 돌기 때문에
링크가 끊기면 로봇이 마지막 상태로 스스로 정지해야 한다 — 오지 않을 수도
있는 메시지를 기다리면 안 된다.
