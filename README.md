# tb_auto — 터틀봇 표지판/신호등 자율주행

계획/결정사항은 `PLAN.md`. 여기는 실행 순서만.

## 구성

```
catkin_ws/src/sign_light_driving/     ROS1 패키지
  scripts/collector_node.py           데이터 수집 노드
  scripts/drive_node.py               인식-제어 자율주행 노드
  launch/collect.launch
  launch/drive.launch
split.py  autolabel.py                데이터셋 도구 (ROS 노드 아님)
dataset_raw/  dataset/  runs/         데이터와 학습 결과
```

워크스페이스 빌드 (노트북, 최초 1회):
```bash
cd ~/tb_auto/catkin_ws && catkin_make
echo 'source ~/tb_auto/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

추론은 **노트북에서 원격**으로 돈다 (로봇 RPi4 2GB로는 실시간 YOLO 무리).
로봇은 카메라/모터만 띄우고, 노트북이 로봇 roscore에 붙는다.

## 0. 환경

로봇 (`ssh ubuntu@100.88.31.34`), 터미널 2개. **`ROS_IP` 먼저 export 할 것:**
```bash
export ROS_IP=100.88.31.34                       # 로봇 tailscale IP. 빠뜨리면 노트북에서 영상 못 받음
roslaunch aicon_bringup aicon_robot.launch
```
```bash
export ROS_IP=100.88.31.34
roslaunch aicon_bringup aicon_camera.launch      # /camera/image, 320x240@30
```

> **`ROS_IP`를 왜 로봇에도 걸어야 하나 (실측으로 확인한 함정)**
> 안 걸면 로봇 노드가 자기 주소를 `http://ubuntu:41671/` 로 광고한다. 노트북은 `ubuntu`를
> 이름풀이 못 해서 **토픽 목록엔 보이는데 메시지는 한 장도 안 들어온다.** 조용히 실패해서
> 원인 찾기 고약하다. 로봇에 `ROS_IP=100.88.31.34`를 걸면 IP로 광고해서 바로 붙는다.

> **`cv camera open failed: device_id 0`** 로 죽으면 이전 `cv_camera_node`가 `/dev/video0`을
> 아직 안 놓은 것이다. `pkill -f cv_camera_node` 하고 몇 초 뒤 다시 띄우면 된다.

노트북, 매 터미널마다:
```bash
source /opt/ros/noetic/setup.bash
source ~/tb_auto/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://100.88.31.34:11311
export ROS_IP=100.76.204.28                      # 노트북 tailscale IP
```

노트북 1회 설치:
```bash
pip install ultralytics labelImg                 # torch는 CPU판으로 딸려옴
```

## 1. 수집 — 노트북에서 실행, 이미지도 노트북에 바로 쌓인다 (scp 불필요)

영상은 **터틀봇 카메라**(`aicon_camera.launch`의 `/camera/image`)에서 온다. 노트북은 구독만 한다.
실측: Tailscale 경유 **30.1 fps / 6.93 MB/s, 손실 없음** — 원본 30fps 그대로 받는다.
(`/camera/image/compressed`는 로봇에 없다. 실측상 raw로 충분해서 필요 없음)
```bash
roslaunch sign_light_driving collect.launch
```
`1~4` 클래스 선택 / `space` 저장 / `q` 종료 → `dataset_raw/{class}/*.jpg`
목표: 클래스당 200장. 각도·거리·조명 섞을 것.

## 2. 라벨링
```bash
labelImg dataset_raw dataset_raw/classes.txt dataset_raw/labels
```
`dataset_raw`를 os.walk로 훑어서 4개 폴더 400장을 **한 세션에 다 연다.** 폴더별로 따로 열지 말 것.

켜자마자 할 것:
1. 좌하단 포맷 버튼을 **YOLO**로 (기본이 PascalVOC라 txt가 안 나온다)
2. 메뉴 `View > Auto Save mode` 체크 — **켜면 Ctrl+S 자체를 안 누른다** (`d`로 넘길 때 자동 저장)
3. 좌측 **`Use default label`** 체크 + 옆칸에 현재 폴더 클래스명 입력
   → 박스만 그리면 라벨이 자동으로 붙는다. 이미지가 폴더 순으로 정렬되니
     폴더가 바뀌는 지점에서만 이 칸을 고쳐주면 된다 (총 3번)

단축키: `w` 박스 그리기 / **`d` 다음 / `a` 이전**
**방향키는 다음 사진으로 안 간다** — 방향키는 선택한 박스를 미세이동시키는 키다. 반드시 `d`/`a`.

> **Ctrl+S 누를 때 "어디에 저장?" 창이 뜬다면** 세 번째 인자(`dataset_raw/labels`)를
> 안 넘긴 것이다. 저장 디렉토리가 없으면 labelImg가 매장 파일 다이얼로그를 띄운다
> (`labelImg.py:1399-1405`). 인자를 주면 안 뜨고, Auto Save까지 켜면 저장 조작이 아예 없어진다.
> txt는 `dataset_raw/labels/`에 모이고 `split.py`가 거기서 찾는다.

> **`classes.txt` 인자를 반드시 넘길 것.** 안 넘기면 labelImg가 클래스 인덱스를
> "그 세션에서 처음 본 순서"로 매긴다. 세션을 나눠 열면 `left_turn`과 `red_light`가
> 둘 다 0번이 되어 **데이터셋이 조용히 망가진다.** 인자로 넘기면 항상
> `0=left_turn 1=right_turn 2=red_light 3=green_light`로 고정된다.
> (labelImg가 각 폴더에 `classes.txt`를 새로 써놓는데, 내용이 같으므로 무시해도 된다)

`split.py`는 txt를 `dataset_raw/labels/`와 이미지 옆 양쪽에서 찾으므로 어느 쪽에 저장돼도 된다.
또 라벨 인덱스와 소속 폴더가 어긋난 파일을 잡아내서 알려준다.

## 3. 분할
```bash
python3 split.py        # dataset/{images,labels}/{train,val} + data.yaml, 8:2
```

## 4. 학습 — Orca 터미널 탭에서 (세션 끊겨도 안 죽게)
```bash
yolo detect train model=yolov8n.pt data=dataset/data.yaml \
  epochs=100 imgsz=320 batch=16 patience=20 device=cpu workers=8
```
`imgsz=320`은 카메라 원본 해상도. 640은 업스케일일 뿐이라 손해.
CPU라 1.5~3시간 예상(미실측). 너무 느리면 dataset/ 압축해서 Colab T4로.
결과: `runs/detect/train/weights/best.pt`, `results.png`, `confusion_matrix.png`

## 5. 주행
```bash
roslaunch sign_light_driving drive.launch
```
모델 경로가 다르면 `model:=<경로>`를 붙인다.

현장 튜닝은 `roslaunch ... 이름:=값`으로 (전부 `drive.launch`의 arg). 기본값과 의미:

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `conf` | 0.6 | 이 미만 검출 폐기 |
| `light_ratio` | 0.15 | 신호등 bbox높이/240 이 값 이상이어야 반응 |
| `turn_ratio` | 0.30 | 표지판 반응 임계 (더 가까이 와야 돈다) |
| `debounce` | 3 | 연속 N프레임 같은 클래스여야 발동 (단발 오검출 차단) |
| `speed` | 0.1 | 기본 직진 m/s |
| `turn_z` | 0.5 | 회전 rad/s |
| `turn_time` | 3.1 | 회전 지속 s (1.57/0.5 ≈ 90도) |
| `watchdog` | 0.5 | 프레임 N초 끊기면 즉시 정지 |

동작: 미검출→직진 / `red_light`→정지 / `green_light`→재출발 /
`left_turn`·`right_turn`→제자리 회전 후 직진 복귀(회전 중 검출 무시, 이후 2초 쿨다운).

**워치독은 건드리지 말 것.** 무선 원격 추론이라 끊기면 로봇이 마지막 명령으로 계속 달린다.

제어 로직만 검증:
```bash
rosrun sign_light_driving drive_node.py --selftest
```
