import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import DR_init
import os
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

# 각각의 패키지에서 기능 임포트
from Voice_Processing.wakeup_word import WakeupWord
from Voice_Processing.get_keyword import GetKeyword
from Voice_Processing.MicController import MicController, MicConfig
from Voice_Processing.stt import STT
from Object_Detection.yolo import YoloModel
from Object_Detection.realsense import ImgNode
from Robot_Control.robot_control import RobotController 
from od_msg.srv import SrvDepthPosition


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # .env 에서 키 가져오기
        # 1. API 키 로드 로직 (우선순위: 환경변수 > .env 파일)
        api_key = os.getenv('OPENAI_API_KEY')
        
        if not api_key:
            try:
                # 패키지의 share 디렉토리(install 폴더)에서 .env 찾기
                package_share_directory = get_package_share_directory('Voice_Processing')
                env_path = os.path.join(package_share_directory, 'resource', '.env')
                
                if os.path.exists(env_path):
                    load_dotenv(env_path)
                    api_key = os.getenv('OPENAI_API_KEY')
                    self.get_logger().info(f".env 로드 성공: {env_path}")
                else:
                    src_env_path = os.path.expanduser("~/cobot_ws/src/dsr-TDTD/Voice_Processing/resource/.env")
                    load_dotenv(src_env_path)
                    api_key = os.getenv('OPENAI_API_KEY')
                    self.get_logger().info(f"src에서 .env 로드 시도: {src_env_path}")
            except Exception as e:
                self.get_logger().error(f"키 로드 중 예외 발생: {e}")

        # 2. 키 검증 및 객체 생성
        if not api_key:
            self.get_logger().error("CRITICAL: API 키를 찾을 수 없습니다!")
            raise RuntimeError("OPEN AI API 키가 없음.")
        
        # 1. 초기화
        self.mic = MicController()
        self.mic.open_stream()
        self.detector = WakeupWord(buffer_size=3840)  
        self.detector.set_stream(self.mic.stream)
        self.stt = STT(api_key)                    
        self.keyword_extractor = GetKeyword()
        self.yolo = YoloModel()                # 객체 탐지
        self.img_node = ImgNode()              # 카메라 데이터
        self.robot = RobotController()         # 로봇 컨트롤
        self.depth_client = self.create_client(
            SrvDepthPosition,
            'get_3d_position'
        )                                       # Object Detection
        while not self.depth_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for object detection service...")

        

    def process(self):
        self.get_logger().info("시스템 안정화 중 (2초)...")
        time.sleep(2.0)
        self.get_logger().info("=== 시스템 시작: 'Hello Rokey'를 말해보세요 ===")
        
        position_map = {
            "pos1": [607.81, -155.19, 350.52, 91.42, 92.53, 88.92], # pos_up1
            
        }

        '''
        POS_GRASP = [607.81, -155.19, 155.52, 91.42, 92.53, 88.92]
        POS_UP1   = [607.81, -155.19, 350.52, 91.42, 92.53, 88.92]
        '''

        while rclpy.ok():
            # 단계 1: Wakeup Word 대기
            # wakeup_word.py의 구조에 따라 is_wakeup() 혹은 __call__을 사용하세요.
            if self.detector.stream is None:
                self.get_logger().error("마이크 스트림이 연결되지 않았습니다.")
                time.sleep(1.0)
                continue

            if self.detector.is_wakeup(): 
                self.get_logger().info("네, 듣고 있어요! 어떤 물체를 찾을까요?")
                
                # 단계 2: 녹음 및 STT
                # 호출어를 들었으니, 이제 5초간 녹음하여 파일로 만듭니다.
                self.mic.record_audio()
                # wav_data = self.mic.get_wav_data()

                # 단계 2-1: STT (음성을 텍스트로 변환)
                user_speech = self.stt.speech2text()
                self.get_logger().info(f"인식된 문장: {user_speech}")

                if user_speech:
                    # 단계 2-2: GetKeyword의 extract_keyword 메서드를 호출하여 리스트 추출
                    # "bottle을 pos1에 둬" -> ['bottle', 'pos1'] 반환
                    # target_list = self.keyword_extractor.extract_keyword(user_speech)
                    objects, targets = self.keyword_extractor.extract_keyword(user_speech)
                    if not objects:
                        self.get_logger().warn("no keywords for object detection")
                        continue
                    # target 없는 경우, 기본 pos1로 채움
                    if len(targets) < len(objects):
                        targets += ["pos1"] * (len(objects) - len(targets))

                    for obj, pos in zip(objects, targets):

                        target_name = obj # 첫 번째 타겟 물체 선택
                        target_pos = pos
                        self.get_logger().info(f"추출된 키워드: {target_name}. 탐지를 시작합니다.")
                        
                        # 단계 3: YOLO 객체 탐지
                        # rclpy.spin_once를 통해 카메라 노드 데이터 갱신
                        rclpy.spin_once(self.img_node, timeout_sec=0.1)
                        
                        box, score = self.yolo.get_best_detection(self.img_node, target_name)
                        
                        if box:
                            self.get_logger().info(f"탐지 성공 ({score:.2f}). 로봇 이동을 시작합니다.")
                            
                            # 단계 4: 좌표 변환 및 로봇 동작
                            # 주의: ImgNode에 pixel_to_robot_coords가 구현되어 있어야 합니다.
                            # 만약 없다면 이전에 작성한 transform_to_base 로직을 여기에 넣어야 합니다.
                            target_pose = self.img_node.pixel_to_robot_coords(box) 
                            
                            # robot_control.py에 정의된 로직 호출
                            self.get_logger().info("집기 시작")
                            self.robot.pick_up(target_pose)

                            time.sleep(0.5)

                            if target_pos not in position_map:
                                self.get_logger().warn(f"{target_pos} 위치 정보가 없음 → pos1 사용")
                                target_pos = "pos1"
                            
                            # 단계 5: place
                            # place_pose = position_map[target_pos[i]]   # test
                            # self.get_logger().info(f"{target_pos[i]} 위치로 이동")
                            place_pose = position_map[target_pos]
                            self.get_logger().info(f"{target_pos} 위치로 이동")

                            self.robot.move_to(place_pose)

                            self.robot.release()
                            ##
                            
                            self.get_logger().info("작업 완료! 다시 대기합니다.")
                        else:
                            self.get_logger().warn(f"'{target_name}'을 화면에서 찾을 수 없습니다.")
                    # else:
                    #     self.get_logger().warn("문장에서 유효한 도구 이름을 찾지 못했습니다.")
                else:
                    self.get_logger().warn("음성이 인식되지 않았습니다.")
            
            time.sleep(0.1)

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    # DR_init 에러 디버깅
    try:
        import DR_init
    except ImportError:
        print("경고: DR_init 모듈을 찾을 수 없습니다. 관련 기능을 건너뜁니다.")
        DR_init = None

    node = MainController()

    if DR_init:
        DR_init.__dsr__node = node

    # 멀티 데이터 처리용
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.img_node)
    executor.add_node(node.robot)

    if hasattr(node, 'img_node'):
        executor.add_node(node.img_node)
    if hasattr(node, 'robot'):
        executor.add_node(node.robot)

    # 메인로직 실행 시키기
    thread = threading.Thread(target=node.process, daemon=True)
    thread.start()

    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 키보드 입력, 시스템 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()