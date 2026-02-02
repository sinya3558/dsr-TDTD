from setuptools import find_packages, setup  # 반드시 find_packages가 있어야 함

package_name = 'Voice_Processing'

setup(
    name=package_name,
    version='0.0.0',
    # ---------------------------------------------------------
    # 기존: packages=[package_name] -> 최상위 폴더만 인식함
    # 변경: find_packages() -> 안쪽의 Voice_Processing 폴더를 자동으로 찾아줌
    # ---------------------------------------------------------
    packages=find_packages(), 
    
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/hello_rokey_8332_32.tflite']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sinya',
    maintainer_email='sinya3443@gmail.com',
    description='Voice Processing for Robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'wakeup = Voice_Processing.wakeup_word:main',
        ],
    },
)
