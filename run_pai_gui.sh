#!/bin/bash

# Docker コンテナを起動するスクリプト（AIロボット教材用 ROS2 環境）

docker run -it \
--name yolo_ros2 \
--privileged \
--shm-size=512m \
-e RESOLUTION=1920x1080 \
-p 15900:5900 \
-p 13389:3389 \
-p 6080:80 \
-p 9090:9090 \
-v $(pwd)/yolo_ros2:/home/ubuntu/yolo_ros2 \
airobotbook/ros2-desktop-ai-robot-book-humble
