# 터틀봇 표지판/신호등 인식 자율주행 프로젝트 - 전체 계획

## 0. 프로젝트 개요
- 터틀봇 카메라로 4종 클래스(`left_turn`, `right_turn`, `red_light`, `green_light`)를 YOLO로 인식
- 인식 결과에 따라 로봇이 **직접 자율주행** (좌/우회전 표지판 → 방향 전환, 빨간불 → 정지, 초록불 → 주행)
- 최종 산출물: 학습된 YOLO 모델 + 인식-제어 연동 노드 + 발표자료(PPT)
- 참고 원본 문서: `~/Downloads/프로젝트_확정사항.md` (데이터 수집/학습 파이프라인은 유효, "자율주행 없음" 서술은 무효)
- 네이밍 규칙: 새로 만드는 패키지/파일/디렉토리에 "notie" 사용 금지

## 1. 환경 / 로봇 정보 (2026-09-21 실측)

### 터틀봇
- SSH: `ubuntu@100.88.31.34` (hostname `ubuntu`)
- 하드웨어: **Raspberry Pi 4 Model B Rev 1.5, aarch64, 4코어, RAM 1.8GB**
- ROS Noetic, Python 3.8.10, 워크스페이스 `~/catkin_ws/src` (aicon, aicon_msgs, ld08_driver, ydlidar, DynamixelSDK, srobot)
- 로봇 구동: `roslaunch aicon_bringup aicon_robot.launch` (core + lidar + diagnostics)
- 카메라 구동: `roslaunch aicon_bringup aicon_camera.launch`
- **카메라 드라이버: `cv_camera` / `cv_camera_node`**
  - 토픽: **`/camera/image`** (`sensor_msgs/Image`) — `/cv_camera/image_raw`에서 리맵됨
  - `/camera/camera_info` — `/cv_camera/camera_info`에서 리맵됨
  - **해상도 320x240, 30fps**, frame_id `camera`
  - **`/camera/image/compressed`는 존재하지 않음** (로봇에 compressed_image_transport 플러그인 미적재). raw만 나온다 — 실측상 충분해서 불필요.
  - 로봇 로컬 실측: 29.98 Hz, 5.87 MB/s, 프레임당 230400 bytes (bgr8)

### 노트북 (로컬)
- 작업 디렉토리 `~/tb_auto`, ROS Noetic 설치됨, Python 3.8.10
- **GPU: Intel Iris Xe (00:02.0, 9a49) 내장그래픽만. NVIDIA/CUDA 없음**
- CPU 8코어, RAM 16GB
- torch / ultralytics 미설치 (환경 구축 필요)

### 네트워크
- Tailscale 경유 RTT 평균 **11.7ms** (min 6.1 / max 16.8, 손실 0%)
- **노트북에서 `/camera/image` 실수신 실측: 30.1 fps / 6.93 MB/s, 손실 없음** → 원격 수집·추론 검증 완료
- **원격 추론 실측 (yolov8n, imgsz=320, CPU): 29.0 fps, latency mean 19.2ms / p50 16.8 / p95 25.4ms**
  → 카메라 30fps를 사실상 실시간으로 따라감. 원격 추론 결정 검증 완료.
  (max 796ms 스파이크 1회 = 최초 워밍업. 워치독 0.5s가 이보다 짧으니 주행 시작 직후 1회 정지 튈 수 있음 → drive.py가 첫 프레임 추론을 미리 돌리거나 watchdog을 1.0s로 올리는 것 검토)
- **함정: 로봇에서도 `export ROS_IP=100.88.31.34` 필수.** 안 하면 노드가 `http://ubuntu:PORT`로 자기를 광고하고, 노트북은 `ubuntu`를 resolve 못 해 **토픽 목록엔 보이는데 메시지가 0장** 들어온다 (조용한 실패)
- 함정: `cv camera open failed: device_id 0` = 이전 `cv_camera_node`가 `/dev/video0` 미반환. `pkill -f cv_camera_node` 후 재시도

## 2. 전체 단계

### 2-1. 데이터 수집 (터틀봇 본체에서 수행)
- ROS1 패키지 작성 (예: `sign_light_collector`, 원본 doc의 `notie_data_collector`와 동일한 역할이나 이름만 변경)
  - `scripts/image_collector.py` — 모니터+키보드 직결용 (cv2 GUI)
  - `scripts/headless_image_collector.py` — SSH 헤드리스용 (터미널 키입력)
- `/camera/image` (`sensor_msgs/Image`) 구독, 숫자키(1~4)로 클래스 선택 → 스페이스바로 프레임 저장
- 저장 구조: `save_dir/{class_name}/{class_name}_{timestamp}_{ms}.jpg`
- **목표 수집량 (확정): 클래스당 200장, 총 800장.** 다양한 각도/거리/조명 조건
  - 근거: 4클래스에 형태 구분이 뚜렷(화살표 방향 / 색상)해서 난이도가 낮음. 200장이면 mAP50 0.9대 충분히 나옴. 부족하면 실패 클래스만 추가 수집.
- 카메라 토픽명 확인 완료 → `/camera/image` (별도 `rostopic list` 불필요)

### 2-2. 데이터 전송 & 라벨링
- 터틀봇 → 노트북(`~/tb_auto`)으로 scp/rsync 전송
- **라벨링 도구 (확정): LabelImg 로컬 사용** (`pip install labelImg`, YOLO 포맷 직접 저장)
  - 근거: 800장 규모라 Roboflow의 자동분할/증강 이점이 계정·업로드·다운로드 왕복 비용을 못 넘음. 오프라인으로 끝나고 산출물이 바로 YOLO txt임.
  - train/val 분할은 python 한 줄 스크립트로 처리
- train/val 8:2 분할, 구조:
  ```
  tb_auto/dataset/
    images/{train,val}/*.jpg
    labels/{train,val}/*.txt
    data.yaml
  ```
- `data.yaml` 클래스: 0=left_turn, 1=right_turn, 2=red_light, 3=green_light

### 2-3. YOLO 학습 (노트북 CPU에서 수행)
- **CUDA GPU가 없으므로 CPU 학습.** 대신 원본 해상도가 320x240이라 입력 크기를 줄일 수 있어 감당 가능.
- 모델: `yolov8n.pt` (nano 고정. CPU + 실시간 추론 요구라 s 이상 불필요)
- **설정 (확정): `epochs=100, imgsz=320, batch=16, patience=20, device='cpu', workers=8`**
  - `imgsz=320`: 카메라가 320x240이라 640은 업스케일일 뿐 정보량 증가 없음. 학습·추론 속도 4배 이득.
  - 예상 소요: 800장 기준 CPU에서 대략 1.5~3시간 (추정, 실측 필요)
- **학습은 Bash 툴이 아니라 Orca 터미널 탭에서 실행** (세션 종료로 잘리지 않게)
- 대안: 시간이 너무 오래 걸리면 Google Colab 무료 T4로 이전 (dataset zip 업로드 → 10분 내 완료)
- 결과물: `best.pt`, `results.png`, `confusion_matrix.png`
- 배포용: `model.export(format="onnx")` — 단, 아래 2-4 결정에 따라 필수는 아님

### 2-4. 인식-제어 연동 (자율주행 로직) — 신규 추가 단계
- 원본 문서에는 없던 단계. 실제 목표가 자율주행이므로 반드시 필요.
- ROS 노드 설계 (예: `sign_light_controller`):
  - YOLO 추론 노드가 카메라 이미지를 구독 → 감지 결과(class, confidence, bbox) publish
  - 제어 노드가 감지 결과를 구독 → `cmd_vel` (Twist) publish
  - 로직 초안:
    - `red_light` 감지 & 근접 → 정지 (`linear.x = 0`)
    - `green_light` 감지 → 직진 재개
    - `left_turn` / `right_turn` 감지 & 근접 → 해당 방향 회전 후 직진 복귀
    - 아무 것도 감지 안 됨 → 기본 직진 또는 정지(안전 정책 결정 필요)
#### 추론 위치 (확정): **노트북 원격 추론**
- 근거: RPi4 2GB RAM / 4코어 aarch64에서 YOLOv8n 실시간 추론은 무리(수 fps + 메모리 압박, 로봇 제어 노드와 자원 경쟁). 네트워크 RTT 11.7ms는 30fps 주기(33ms)보다 짧아 원격이 유리.
- 구성:
  - 터틀봇: `aicon_robot.launch` + `aicon_camera.launch` 만 실행 (roscore = 로봇)
  - 노트북: `export ROS_MASTER_URI=http://100.88.31.34:11311`, `export ROS_IP=<노트북 tailscale IP>`
  - 노트북에서 `/camera/image`(가능하면 `/camera/image/compressed`) 구독 → YOLO 추론 → `/cmd_vel` publish
- 장점: `best.pt` scp 불필요, onnx 변환 불필요, 모델 교체가 노트북에서 즉시 반영
- 리스크: 무선 끊김 시 로봇이 마지막 명령으로 계속 주행 → **워치독 필수** (아래)

#### 판단 기준 (확정)
프레임 240px 높이 기준 bbox 높이 비율(`bbox_h / 240`)을 거리 대용으로 사용. 별도 거리센서 불필요.
- `conf >= 0.6` 미만 검출은 버림
- **디바운스: 동일 클래스가 연속 3프레임 잡혀야 액션 발동** (단발 오검출 차단)
- `red_light`: 비율 >= 0.15 → 정지 (`linear.x=0`). 멀리서도 일찍 멈추게 낮게 잡음
- `green_light`: 정지 상태에서만 의미. 검출 시 직진 재개
- `left_turn` / `right_turn`: 비율 >= 0.30 → 해당 방향 제자리 회전 (`angular.z=±0.5`, 90도 도달까지 `1.57/0.5≈3.1s`) 후 직진 복귀
  - 회전 중에는 새 검출 무시 (상태머신 락)
- **아무것도 검출 안 됨 → 기본 직진 `linear.x=0.1 m/s`** (표지판 사이 구간 주행용)
- **안전 워치독: 0.5초간 이미지 프레임 미수신 → 즉시 정지.** 무선 끊김/노트북 다운 대비. 이건 타협 없음.
- 위 숫자는 전부 rosparam으로 빼서 현장에서 튜닝 (실제 표지판 크기/조명에 맞춰 조정 필요)

### 2-5. 발표자료 (PPT)
1. 프로젝트 개요
2. 문제 정의 및 YOLO 선택 이유
3. 데이터 수집 방법 (터틀봇 + 자체 제작 ROS1 패키지)
4. 데이터 라벨링
5. 학습 설정 및 과정
6. 결과 (mAP, PR, confusion matrix, 샘플 추론 이미지)
7. 인식-제어 연동 로직 (자율주행 구현 방식)
8. 실주행 데모
9. 한계 및 향후 계획
10. Q&A

## 3. 미정 사항

### 해결됨 (2026-09-21)
| 항목 | 결론 |
|---|---|
| 노트북 GPU | NVIDIA 없음 (Intel Iris Xe). CPU 학습 + `imgsz=320`으로 대응 |
| 카메라 드라이버/토픽 | `cv_camera` → `/camera/image`, 320x240@30fps |
| 라벨링 도구 | LabelImg 로컬 |
| 수집 수량 | 클래스당 200장 (총 800) |
| 추론 위치 | 노트북 원격 추론 (ROS_MASTER_URI로 로봇 마스터 접속) |
| 정지/회전 기준 | bbox 높이비율 + conf 0.6 + 3프레임 디바운스 (2-4 참조) |

### 남은 미정 (사용자 결정 필요)
- **발표 일정** — 이것만 알면 수집/학습/튜닝 일정 역산 가능
- **실시간 데모 환경** — 데모 장소에 노트북-로봇 같은 네트워크(Tailscale/공용 WiFi) 확보 가능한지. 원격 추론 구성이라 여기에 의존함. 불가하면 온보드 fallback을 별도 설계해야 하므로 빨리 확인 필요
- 표지판/신호등 실물 준비 여부 (출력물? 실제 모형?) — 수집 시작 전제조건

## 4. 구현 (2026-09-21 완료)

catkin 패키지 없이 노트북에서 도는 순수 python 스크립트 4개. 실행 순서는 `README.md`.

| 파일 | 역할 |
|---|---|
| `collect.py` | 노트북에서 로봇 roscore에 붙어 `/camera/image` 구독, cv2 창에서 1~4 클래스 선택 + space 저장 → `dataset_raw/{class}/`. `classes.txt`도 생성 |
| `split.py` | labelImg가 남긴 txt와 짝 맞춰 `dataset/{images,labels}/{train,val}` + `data.yaml` 8:2 분할 (seed 고정) |
| `drive.py` | YOLO 추론 + `Controller` 상태머신 → `/cmd_vel`. 워치독 포함. `--selftest`로 제어 로직 단독 검증 |
| `README.md` | 0~5단계 실행 순서, 튜닝 파라미터 표 |

학습 스크립트는 안 만듦 — `yolo detect train ...` CLI 한 줄이면 된다 (README 4단계).

**당초 계획 대비 바뀐 점**: 로봇용 ROS 패키지(`sign_light_collector`) 배포를 없앴다.
수집기를 노트북에서 돌리면 로봇에 catkin 빌드·배포할 게 없고, 이미지가 노트북에 바로 쌓여서
2-2의 scp/rsync 전송 단계 자체가 사라진다. 헤드리스 수집기도 불필요 (노트북에 화면이 있음).

### 다음 액션
1. ~~`pip install ultralytics labelImg`~~ → 완료 (torch 2.4.1 CPU, ultralytics 8.4.157, labelImg)
2. ~~로봇 bringup 띄우고 `collect.py` 연결 확인~~ → 완료 (30.1fps 수신 확인)
3. 데이터 수집 (클래스당 200장)
