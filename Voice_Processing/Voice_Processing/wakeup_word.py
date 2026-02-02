# wakeup_word.py

import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

package_path = get_package_share_directory("Voice_Processing")
MODEL_NAME = "hello_rokey_8332_32.tflite"
MODEL_PATH = os.path.join(package_path, f"resource/{MODEL_NAME}")


class WakeupWord:
    def __init__(self, buffer_size=3840): # 48kHz에서 1280개를 만들기 위해 3840으로 변경
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        # 48000Hz 기준 3840개 샘플을 읽어야 리샘플링 후 1280개가 됩니다.
        self.buffer_size = 3840 

    def is_wakeup(self):
        try:
            # 1. 48kHz 데이터 읽기
            raw_data = self.stream.read(self.buffer_size, exception_on_overflow=False)
            audio_chunk = np.frombuffer(raw_data, dtype=np.int16)

            # 2. 리샘플링 (48000 -> 16000)
            # 3840 -> 1280개로 딱 떨어짐
            num_samples = 1280
            audio_resampled = resample(audio_chunk, num_samples)
            
            # 3. 데이터 정제 (float 변환 후 다시 int16으로)
            audio_resampled = np.clip(audio_resampled, -32768, 32767).astype(np.int16)

            # 4. 모델 예측
            # 만약 학습된 모델이 특수한 경우, predict_clip을 써보는 것도 방법입니다.
            outputs = self.model.predict(audio_resampled)
            confidence = outputs.get(self.model_name, 0)

            # 로그 출력 (변화 감지용)
            if confidence > 0.0001:
                print(f"Confidence: {confidence:.4f}")

            if confidence > 0.3:
                print(">>> [SUCCESS] Hello Rokey Detected! <<<")
                return True
                
        except Exception as e:
            print(f"Error: {e}")
        return False
    def set_stream(self, stream):
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.stream = stream
