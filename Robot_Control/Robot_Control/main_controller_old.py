import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import os
import numpy as np
from scipy.spatial.transform import Rotation
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

# 사용자 정의 모듈 임포트
from Voice_Processing.wakeup_word import WakeupWord
from Voice_Processing.get_keyword import GetKeyword
from Voice_Processing.MicController import MicController
from Voice_Processing.stt import STT
from Object_Detection.yolo import YoloModel
from Object_Detection.realsense import ImgNode

# 로봇 제어 관련 (DR_init 설정 필수)
import DR_init
from robot_control.onrobot import RG

# 로봇 설정 (robot_control.py 기반)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# Doosan Robot 라이브러리 임포트
try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait
except ImportError as e:
    print(f"DSR_ROBOT2 임포트 에러: {e}")

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # 1. 환경 설정 및 API 키 로드
        self._load_env_keys()
        self.package_path = get_package_share_directory("Robot_Control")
        
        # 2. 로봇 및 그리퍼 초기화
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = self
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        
        # 3. 음성 및 시각 지능 객체 생성
        self.mic = MicController()
        self.mic.open_stream()
        self.detector = WakeupWord(buffer_size=1280)  
        self.detector.set_stream(self.mic.stream)
        self.stt = STT(self.api_key)                    
        self.keyword_extractor = GetKeyword()
        
        self.yolo = YoloModel()
        self.img_node = ImgNode()

        # 로봇 초기 위치로 이동
        self.init_robot()
        self.get_logger().info("MainController 및 로봇 초기화 완료.")

    def _load_env_keys(self):
        load_dotenv() # 시스템 환경변수 우선
        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            vp_share = get_package_share_directory('Voice_Processing')
            load_dotenv(os.path.join(vp_share, 'resource', '.env'))
            self.api_key = os.getenv('OPENAI_API_KEY')

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        """로봇의 현재 포즈를 4x4 변환 행렬로 변환 (ZYZ 오일러 각 사용)"""
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords):
        """카메라 좌표(x,y,z)를 로봇 베이스 좌표계로 변환"""
        # 1. 외부 캘리브레이션 파일 로드 (Gripper to Camera)
        calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        if not os.path.exists(calib_path):
            self.get_logger().error(f"캘리브레이션 파일을 찾을 수 없음: {calib_path}")
            return None
        
        gripper2cam = np.load(calib_path)
        
        # 2. 카메라 좌표를 동차 좌표(Homogeneous)로 변환
        coord_cam = np.append(np.array(camera_coords), 1)

        # 3. 로봇의 현재 좌표(Base to Gripper) 가져오기
        robot_posx = get_current_posx()[0] # [x, y, z, rx, ry, rz]
        base2gripper = self.get_robot_pose_matrix(*robot_posx)

        # 4. 좌표 변환 연산: Base_Coord = T_base2gripper * T_gripper2cam * Cam_Coord
        base2cam = base2gripper @ gripper2cam
        coord_base = np.dot(base2cam, coord_cam)

        # 5. 최종 좌표 및 오프셋 적용
        td_coord = coord_base[:3]
        td_coord[2] += DEPTH_OFFSET
        td_coord[2] = max(td_coord[2], MIN_DEPTH)

        # 최종 목표 포즈: 변환된 [x, y, z] + 원래 로봇의 [rx, ry, rz]
        target_pose = list(td_coord) + robot_posx[3:]
        return target_pose

    def init_robot(self):
        """로봇을 대기 위치로 이동시키고 그리퍼를 엽니다."""
        self.get_logger().info("로봇 초기화 이동 중...")
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        self.gripper.open_gripper()
        mwait()

    def pick_and_place(self, target_pose):
        """물체를 집고 초기 위치로 돌아옵니다."""
        self.get_logger().info(f"물체 이동 시작: {target_pose}")
        movel(target_pose, vel=VELOCITY, acc=ACC)
        mwait()
        
        self.gripper.close_gripper()
        # 그리퍼가 동작 중인지 확인 (onrobot.py의 status bit 0 사용)
        time.sleep(1.0) 
        while self.gripper.get_status()[0]: 
            time.sleep(0.2)
        mwait()
        
        # 물체를 집은 후 다시 초기 위치로 이동하여 작업 완료 표시
        self.init_robot()

    def process(self):
        self.get_logger().info("=== 시스템 시작: 'Hello Rokey'를 말하세요 ===")
        
        while rclpy.ok():
            if self.detector.is_wakeup():
                self.get_logger().info("음성 인식 시작...")
                self.mic.close_stream()
                user_speech = self.stt.speech2text()
                self.get_logger().info(f"인식된 문장: {user_speech}")

                self.mic.open_stream()
                self.detector.set_stream(self.mic.stream)
                
                if user_speech:
                    keywords = self.keyword_extractor.extract_keyword(user_speech)
                    if keywords:
                        target_name = keywords[0] # "bottle" 등
                        self.get_logger().info(f"'{target_name}' 탐지 중...")
                        
                        # YOLO 탐지
                        box, score = self.yolo.get_best_detection(self.img_node, target_name)
                        
                        if box:
                            # 1. 카메라 좌표계 기준 3D 좌표 구하기 (이전 코드 로직 활용)
                            cx, cy = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)
                            depth_frame = self.img_node.get_depth_frame()
                            intrinsics = self.img_node.get_camera_intrinsic()
                            
                            if depth_frame is not None and intrinsics is not None:
                                cz = float(depth_frame[cy, cx])
                                if cz > 0:
                                    cam_x = (cx - intrinsics['ppx']) * cz / intrinsics['fx']
                                    cam_y = (cy - intrinsics['ppy']) * cz / intrinsics['fy']
                                    
                                    # 2. 카메라 좌표 -> 로봇 베이스 좌표 변환
                                    target_pose = self.transform_to_base([cam_x, cam_y, cz])
                                    
                                    if target_pose:
                                        self.pick_and_place(target_pose)
                                        self.get_logger().info("임무 완료.")
                                    else:
                                        self.get_logger().error("좌표 변환 실패.")
                                else:
                                    self.get_logger().warn("Depth 값이 유효하지 않습니다.")
                        else:
                            self.get_logger().warn(f"'{target_name}'을 찾지 못했습니다.")
                else:
                    self.get_logger().warn("음성이 들리지 않습니다.")
            
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()

    if DR_init:
        DR_init.__dsr__node = node

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.img_node)

    thread = threading.Thread(target=node.process, daemon=True)
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.gripper.close_connection()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()