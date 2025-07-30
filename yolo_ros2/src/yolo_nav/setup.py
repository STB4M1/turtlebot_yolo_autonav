from setuptools import find_packages, setup

package_name = 'yolo_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/yolo_nav.launch.py',
            'launch/my_world.launch.py',  
            'launch/full_launch.py',
        ]),
        ('share/' + package_name + '/my_worlds', ['my_worlds/mixed_room.world']),
        ('share/' + package_name + '/launch', ['launch/yolo_nav.launch.py']),      # launchファイルを追加
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOによる物体認識ナビゲーションノード',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_nav.yolo_node:main',
            'obstacle_avoidance_node = yolo_nav.obstacle_avoidance_node:main',
        ],
    },
)

