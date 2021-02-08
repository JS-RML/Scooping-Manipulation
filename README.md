# Scooping
## 1. Overview
A novel dexterous robotic manipulation technique which we called Scooping is implemented by this package. The work implies autonoumus picking of thin profile objects from the flat surface and in a dense clutter environment, plastic cards, domino blocks, Go stones for example, using two-finger parallel jaw gripper having one length-controllable digit. The technique presents the ability to carry out complete bin picking: from the first to the last one. The package includes finger design used, object detection and instance segmentation by Mask-RCNN, scoop-grasp manipulation package.



## 2. Prerequisite
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
- [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- [**RealSense Camera SR300**](https://github.com/IntelRealSense/realsense-ros)
<!-- - [**Customized Finger design**](https://github.com/HKUST-RML/dig-grasping/tree/master/fingertip%20design) features fingertip concavity---
- [**Extendable Finger**](https://github.com/HKUST-RML/extendable_finger) for realizing finger length differences during digging -->


### 2.2 Software
This implementation requires the following dependencies (tested on Ubuntu 16.04 LTS):
