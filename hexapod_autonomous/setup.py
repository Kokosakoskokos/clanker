from setuptools import setup

package_name = 'hexapod_autonomous'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/' + f for f in [
            'hexapod_bringup.launch.py',
            'hexapod_control.launch.py',
            'hexapod_vision.launch.py',
            'hexapod_navigation.launch.py',
            'hexapod_ai.launch.py',
            'hexapod_full.launch.py',
        ]]),
        ('share/' + package_name + '/config', ['config/' + f for f in [
            'servos.yaml',
            'gait_params.yaml',
            'camera.yaml',
            'navigation.yaml',
            'ai_config.yaml',
        ]]),
        ('share/' + package_name + '/urdf', ['urdf/' + f for f in [
            'hexapod.urdf.xacro',
        ]]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous Hexapod Robot System with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hexapod_bringup = hexapod_autonomous.hexapod_bringup.main:main',
            'hexapod_control = hexapod_autonomous.hexapod_control.servo_controller:main',
            'hexapod_gait = hexapod_autonomous.hexapod_control.gait_controller:main',
            'hexapod_vision = hexapod_autonomous.hexapod_vision.camera_node:main',
            'hexapod_object_detect = hexapod_autonomous.hexapod_vision.object_detector:main',
            'face_recognition = hexapod_autonomous.hexapod_vision.face_recognition:main',
            'hexapod_ultrasonic = hexapod_autonomous.hexapod_perception.ultrasonic_node:main',
            'hexapod_navigation = hexapod_autonomous.hexapod_navigation.navigator:main',
            'person_follower = hexapod_autonomous.hexapod_navigation.person_follower:main',
            'hexapod_voice = hexapod_autonomous.hexapod_ai.voice_node:main',
            'hexapod_ai_brain = hexapod_autonomous.hexapod_ai.ai_brain:main',
            'memory_node = hexapod_autonomous.hexapod_ai.memory_node:main',
        ],
    },
)
