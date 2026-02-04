from setuptools import find_packages, setup
import os

package_name = 'Object_Detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), 
        ['resource/best_yolov8_epoch100.pt', 
        'resource/best_yolov26_epoch100.pt', 
        'resource/class_name_tool.json']),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sinya',
    maintainer_email='sinya3443@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "yolo = Object_Detection.yolo:main",
            "detection = Object_Detection.detection:main",
            "check_model = Object_Detection.check_model:main",
        ],
    },
)
