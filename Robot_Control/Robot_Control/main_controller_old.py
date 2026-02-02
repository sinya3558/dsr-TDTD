import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import os
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

# 커스텀 패키지 임포트
from Voice_Processing.wakeup_word import WakeupWord
from Voice_Processing.get_keyword import GetKeyword
from Voice_Processing.MicController import MicController
from Voice_Processing.stt import STT
from Object_Detection.yolo import YoloModel
from Object_Detection.realsense import ImgNode
from Robot_Control.robot_control import RobotController 

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # 1. API 키 로드
        self.api_key = self._load_api_key()
        
        # 2. 모듈 초기화
        self.mic = MicController()
        self.mic.open_stream()
        
        self.detector = WakeupWord(buffer_size=1280)  
        self.detector.set_stream(self.mic.stream)
        
        self.stt = STT(self.api_key)                    
        self.keyword_extractor = GetKeyword()
        self.yolo = YoloModel()
        
        # 다른 노드 참조 (Executor에 의해 함께 spin됨)
        self.img_node = ImgNode()
        self.robot = RobotController()

    def _load_api_key(self):
        load_dotenv() # 시스템 환경변수 우선
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            try:
                pkg_path = get_package_share_directory('Voice_Processing')
                env_path = os.path.join(pkg_path, 'resource', '.env')
                load_dotenv(env_path)
                api_key = os.getenv('OPENAI_API_KEY')
            except Exception:
                pass
        
        if not api_key:
            self.get_logger().error("API 키를 찾을 수 없습니다!")
            raise RuntimeError("API Key Missing")
        return api_key

    def process(self):
        """메인 로직 루프 (별도 스레드에서 실행)"""
        self.get_logger().info("시스템 안정화 중 (2초)...")
        time.sleep(2.0)
        self.get_logger().info("=== 시스템 시작: 'Hello Rokey'를 말해보세요 ===")
        
        while rclpy.ok():
            # 1단계: 호출어 감지
            if self.detector.is_wakeup(): 
                self.get_logger().info("호출어 감지! 듣고 있습니다...")
                
                # 2단계: 음성 녹음 및 STT
                self.mic.record_audio() # 녹음 완료될 때까지 블로킹
                wav_data = self.mic.get_wav_data()
                user_speech = self.stt.speech2text(wav_data)
                self.get_logger().info(f"인식 결과: {user_speech}")

                if user_speech:
                    # 3단계: 키워드 추출
                    target_list = self.keyword_extractor.extract_keyword(user_speech)
                    
                    if target_list:
                        target_name = target_list[0]
                        self.get_logger().info(f"타겟 탐지 시작: {target_name}")
                        
                        # 4단계: YOLO 탐지 (img_node의 최신 프레임 사용)
                        # spin_once 대신 executor가 백그라운드에서 계속 업데이트하는 self.img_node를 사용
                        detection_result = self.yolo.get_best_detection(self.img_node, target_name)
                        
                        if detection_result and detection_result[0] is not None:
                            box, score = detection_result
                            self.get_logger().info(f"[{target_name}] 탐지 성공! 신뢰도: {score:.2f}")

                            # 5단계: 좌표 변환 (2D -> 3D Robot Base)
                            # ImgNode 내부에 depth 정보를 이용한 변환 메서드가 있다고 가정
                            world_pose = self.img_node.pixel_to_robot_coords(box)
                            
                            if world_pose:
                                self.get_logger().info(f"이동 좌표: {world_pose}")
                                # 6단계: 로봇 제어 (Pick up)
                                self.robot.pick_up(world_pose)
                                self.get_logger().info("작업 완료. 대기 상태로 전환합니다.")
                            else:
                                self.get_logger().error("좌표 변환에 실패했습니다.")
                        else:
                            self.get_logger().warn(f"화면 내에 {target_name}이(가) 없습니다.")
                    else:
                        self.get_logger().warn("명령어에서 키워드를 찾지 못했습니다.")
                else:
                    self.get_logger().warn("음성 인식 실패.")
            
            time.sleep(0.05) # CPU 점유율 방지

def main(args=None):
    rclpy.init(args=args)

    # 1. 메인 컨트롤러 노드 생성
    main_node = MainController()

    # 2. 멀티스레드 실행기 설정
    # 여러 노드(Main, Img, Robot)가 동시에 메시지를 처리할 수 있게 함
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(main_node.img_node)
    executor.add_node(main_node.robot)

    # 3. 로직 루프를 별도 스레드에서 실행 (Spin 방해 금지)
    process_thread = threading.Thread(target=main_node.process, daemon=True)
    process_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        main_node.get_logger().info("시스템 종료 요청")
    finally:
        main_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()