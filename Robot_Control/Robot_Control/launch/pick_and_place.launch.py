import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 각 패키지의 절대 경로 설정 (src 기준)
    # 현재 환경에 맞춰 ~/cobot_ws/src/dsr-TDTD 경로를 기준으로 합니다.
    base_path = os.path.expanduser("~/cobot_ws/src/dsr-TDTD")
    
    voice_path = os.path.join(base_path, "Voice_Processing")
    detection_path = os.path.join(base_path, "Object_Detection")
    robot_path = os.path.join(base_path, "Robot_Control")

    # 2. PYTHONPATH 환경 변수 구성 (패키지 임포트 가능하도록)
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{voice_path}:{detection_path}:{robot_path}:" + env.get("PYTHONPATH", "")
    
    # 3. OpenAI API Key 확인 (터미널에 없으면 직접 입력 가능)
    if not os.getenv('OPENAI_API_KEY'):
        env['OPENAI_API_KEY'] = 'your-actual-api-key-here' # 혹은 직접 입력

    return LaunchDescription([
        Node(
            package='Robot_Control',
            executable='main',  # setup.py의 entry_points에 정의된 'main'
            name='main_system',
            output='screen',
            env=env, # 구성한 환경 변수 주입
            parameters=[{
                'model_path': '/home/sinya/cobot_ws/src/dsr-TDTD/Object_Detection/resource/best_yolov8_epoch100.pt'
            }]
        )
    ])