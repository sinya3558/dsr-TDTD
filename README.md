# DSR-Project2


## 📁 프로젝트 디렉토리 구조

```text
dsr-TDTD/                           # DOOSAN ROBOTICS PROJECT 2
├── Object_Detection/               # Computer Vision 
│   ├── Hand-Detection/
│   └── Bottle-Detection/
│
├── Robot_Control/                  # ROS2
|   └── main_controller.py          # State Machine (Voice -> Vision -> Robot) 
│
├── Voice_Processing/               # NLP
│
├── recycle_interfaces/             # User Defined Msg/Srv/Action Interfaces (사용하실 인터페이스 있다면 여기 모아주세요)
│
└── resource/                       # Shared Resources
    ├── hello_rokey_8332_32.tflite  # Wake-up word file
    └── voice

```

## 1. Project Overview
- TBD

<br>

## 2. Logic Sequence Overview

전체 시스템 흐름은 다음 순서로 동작합니다.

1. 사용자의 음성 명령 입력 (**voice standby**)

2. 음성 인식 노드에서 타겟 객체 키워드 추출 (**keyword extraction**)

3. 객체 인식 노드에서 타겟 객체의 3D 좌표 추적 (**object detection**)

4. 로봇 제어 노드가 좌표를 기반으로 로봇을 제어 (**robot execution**)

5. 인식 결과와 로봇 상태를 실시간으로 모니터링


<br>

## 3. Communication Interface Overview

| 구분      | 송신 노드 (Client / Pub) | 수신 노드 (Server / Sub) | 인터페이스 이름 (Topic / Srv / Action) | 데이터 내용                                  |
| ------- | -------------------- | -------------------- | ------------------------------- | --------------------------------------- |
| Service | dsr_control          | dsr_voice            | GetKeyword.srv                  | 사용자의 음성 명령에서 추출된 타겟 물체명 (예: `"bottle"`) |
| Action  | dsr_control          | dsr_perception       | GetObjectCoords.action          | 목표 물체의 실시간 3D 좌표 (`x, y, z`) 및 검출 상태    |
| Topic   | dsr_perception       | Rviz2 / Monitor      | `/detection_result`             | YOLO 기반 객체 검출 결과(BBox)가 포함된 이미지 스트림     |
| Topic   | dsr_control          | Doosan Robot         | `/dsr/joint_states`             | 로봇의 현재 관절 각도 및 상태 정보                    |

<br>

## 4. Technical Skills

- Language: Python 3.10

- Framework: ROS 2 Humble

- CV: YOLOv8, v11, v26

- Hardware: Doosan M0609, Intel RealSense D435, OnRobot RG2
- 
<br>