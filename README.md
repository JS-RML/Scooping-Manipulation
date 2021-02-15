# Scooping
## 1. Overview
A novel dexterous robotic manipulation technique which we called Scooping is implemented by this package. The work implies autonoumus picking of thin profile objects from the flat surface and in a dense clutter environment, plastic cards, domino blocks, Go stones for example, using two-finger parallel jaw gripper having one length-controllable digit. The technique presents the ability to carry out complete bin picking: from the first to the last one. The package includes gripper design used, pre-scoop planning, object detection and instance segmentation by Mask-RCNN, scoop-grasp manipulation package and collision check.

### Scooping from Flat Support Surface
<p align = "center">
<img src="files/Github_domino-long.gif" width="360" height="202">   
<img src="files/Github_domino-short.gif" width="360" height="202">   
<img src="files/Github_plastic-card.gif" width="360" height="202">   
<img src="files/Github_go-stone.gif" width="360" height="202"> 
<img src="files/Github_triangular-prism.gif" width="360" height="202">
</p>


### Application to Bin Picking
<p align = "center">
<img src="files/Github_plastic-card_bin.gif" width="360" height="202">   
<img src="files/Github_domino_bin.gif" width="360" height="202">   
<img src="files/Github_go-stone_bin.gif" width="360" height="202"> 
<img src="files/Github_triangular-prism_bin.gif" width="360" height="202">
</p>


## 2. Prerequisites
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
- [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- [**RealSense Camera SR300**](https://github.com/IntelRealSense/realsense-ros)
<!-- - [**Customized Finger design**](https://github.com/HKUST-RML/dig-grasping/tree/master/fingertip%20design) features fingertip concavity---
- [**Extendable Finger**](https://github.com/HKUST-RML/extendable_finger) for realizing finger length differences during digging -->


### 2.2 Software
This implementation requires the following dependencies (tested on Ubuntu 16.04 LTS):
- [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation)
- [**Urx**](https://github.com/SintefManufacturing/python-urx) for UR10 robot control
- [**robotiq_2finger_grippers**](https://github.com/chjohnkim/robotiq_2finger_grippers.git): ROS driver for Robotiq Adaptive Grippers
<!-- - [**Mask R-CNN**](https://github.com/matterport/Mask_RCNN) for instance segmentation (also see the dependencies therein). Also download the trained weights for [Go stone](https://hkustconnect-my.sharepoint.com/:u:/g/personal/ztong_connect_ust_hk/Eb7z0WBHf8BOgLfkGKQf1wsBcZgVAwpUTJP7Q9u0y8h5Kw?e=15cEsA) and [capsule](https://hkustconnect-my.sharepoint.com/:u:/g/personal/yhngad_connect_ust_hk/EY5C4hAOm-xNoQ1oHyhArtgBe91wuVaWSf3N2D1fJmcERg?e=aHRnJa).-->
- [**OpenCV 3.4.1**](https://pypi.org/project/opencv-python/3.4.1.15/) and [**Open3D 0.7.0.0**](http://www.open3d.org/docs/0.7.0/getting_started.html)
- [**PyBullet**](https://pybullet.org/wordpress/) for collision check

**Note**: The online compiler [**Jupyter Notebook**](https://jupyter.org/) is needed to run our program.

## 3. Real Experiments to Practice Scooping
### 3.1 Build on ROS
In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-RML/Scooping.git
cd ..
catkin_make
source devel/setup.bash
```
Activate robotiq 2-fingered gripper and RealSense Camera in two separate terminals:

