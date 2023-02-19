FROM ros:humble-ros-base-jammy

# create a basic workspace
RUN ["/bin/bash", "-c", "mkdir -p /app/ws/src"]
WORKDIR /app/ws

# install some basic toolsRUN apt update 
RUN apt update && apt upgrade -y
RUN apt install -y --no-install-recommends git curl 
RUN rm -rf /var/lib/apt/lists/*
RUN apt update && apt upgrade -y
		
# Install basic messaging, image processing 
RUN apt install -y --no-install-recommends ffmpeg libsm6 libxext6 ros-humble-cv-bridge ros-humble-vision-msgs python3-natsort ros-humble-vision-opencv python3-pip ros-humble-sensor-msgs-py 
# Install naviagtion tools
RUN apt install -y --no-install-recommends ros-humble-navigation2 ros-humble-nav2-bringup
RUN rm -rf /var/lib/apt/lists/*
		
# Utilize pip to install
RUN python3 -m pip install --upgrade pip

#install yolo+requirements
RUN git clone https://github.com/ultralytics/yolov5 
RUN pip install -r yolov5/requirements.txt  # install

#copy the custom yolov5 model
COPY ./custom_model.pt /app/ws/yolov5

#copy and build the ros2_yolov5 pakage
COPY ./src/navi/.  /app/ws/src/navi
COPY ./src/ros2_yolov5/.  /app/ws/src/ros2_yolov5
RUN . /opt/ros/humble/setup.sh && colcon build

