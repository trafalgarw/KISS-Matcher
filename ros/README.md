<div align="center">
    <h1>KISS-Matcher-SAM</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
</div>

______________________________________________________________________

## ROS2 KISS-Matcher-SAM

This repository provides LiDAR SLAM using KISS-Matcher.
If you just want to **perform registration using ROS2**, see [README_ROS2_REGISTRATION.md](<>).

## :gear: How To Build & RUN

1. Put this repository in your colcon workspace, and then build this repository as follows:

```bash
cd ${YOUR_ROS2_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd ..
colcon build --packages-select kiss_matcher_ros
```

And then source your workspace using `source install/setup.bash` or `source install/setup.zsh` depending on your shell.

2. Then, run the command below:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

If you want to run it on your own dataset, make sure to set the `/cloud` and `/odom` topics appropriately using:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  odom_topic:=<YOUR_TOPIC> scan_topic:=<YOUR_TOPIC>
```

## How to Tune Parameters?

More details can be found in [config/slam_config.yaml](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/ros/config/slam_config.yaml).
