# YOLOv5 Computer Vision Robot

This repository contains a ROS2 package for inference on custom YOLOv5 models, a diff-drive robot named Hubert and slam/nav2 config and launch files to let Hubert navigate with YOLOv5-perception.

## The Problem 

Let's say you want to give your autonomous robot the ability to perceive specific objects that are flat or level with the ground, e..g. to perceive floor markings for navigation, or to avoid it rolling over or stepping on dangerous areas like water or oil puddles, or valuable objects like the receipt for that laptop you wanted to return, which you later find in the dirt container of your vacuum robot. In this project, we explore how YOLO computer vision can be deployed for perception within the ROS2 framework.

In this toy example, we meet Hubert, the cleaning robot residing in our institute's common room. We simulate navigation tasks where Hubert must avoid Doppelkopf playing cards that someone dropped carelessly onto the ground.

![robo](https://user-images.githubusercontent.com/90965716/219977326-e1d704a0-006b-49f4-ad47-14b808e3534c.gif)

Left: costmap, right: gazebo simulation. The robot navigates, sees an obstacle and reroutes.

## Installation

We ran Hubert on ros2-foxy, with packages ros-foxy-xacro, ros-foxy-gazebo-ros-pkgs. The navigation and YOLOv5 perception ran on ros2-humble (we refer to the Dockerfile for installation). Note: We expect everything (also Hubert) to run on humble (that has not been tested though).

## Running the Simulation

1. Run the robot simulation in gazebo (foxy)
  ```      
    ros2 launch hubert launch_sim.launch.py use_sim_time:=true world:=./src/hubert/worlds/fslightcards.world 
  ```
2. Run Rviz (foxy)
  ```
    rviz2 ./src/hubert/config/rviz.rviz 
  ```
3. Run the YOLOv5 inference (humble)
  ```
    ros2 run ros2_yolov5 inferator
  ```
4. Run mapping and localization (humble)
  ```
    ros2 launch navi online_async_launch.py use_sim_time:=true
  ```
5. Run navigation (humble)
  ```
     ros2 launch navi navigation_launch.py use_sim_time:=true
  ```
If everything started successfully, you can now use the RViz-GUI to submit navigation tasks. 


## Note about YOLOv5

If you are just here hoping to find out how to deploy your custom YOLOv5 model in your ROS2 project, have a look at the ros2_yolov5 package. The weights "custom_model.pt" provided here detect and classify german Doppelkopf playing cards. But the model should be interchangeable with any other weights you trained. 


## Conclusion

The main goal of this project was to provide an exercise to learn about the ROS2 framework and gazebo simulations while finding more applications for our self-trained computer vision model I had laying around. Therefore, Hubert has some obvious design flaws simply to ease the workload. We have a few comments on these and other problems that became apparent in the simulation:

* Objects that are not playing cards and not on a level with the lidar are not detected. Adding a bumper sensor or using the depth camera to detect obstacles under the lidar would be better. 
* A camera stick on a vacuum robot is a bad idea. All robot components should be kept below the lidar. To still guarantee the necessary field of view, multiple cameras could be installed. 
* A depth camera might not be necessary. If we assume that the objects we are looking for lie flat on the ground, then projective geometry provides enough tools to locate each pixel. 
* The inference takes a considerable amount of time (>200ms). In the current configuration, Hubert can only view the area very close to him. Hence, Hubert has a low speed limit.
* Hubert sometimes gets confused when there are multiple ways of similar length around an obstacle or finds himself with his way blocked by a card he forgot too quickly. The nav2 params still need some tweaking. 

## Acknowledgements

I thank Josh Evans (https://articulatedrobotics.xyz/). His tutorials helped me a lot to get started with ros2/gazebo. 
