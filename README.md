# Simulation platform for active human reconstruction and motion capture

This repository is the implementation of simulation platform for active human reconstruction and motion capture in following publication / pre-print

* iHuman3D: Intelligent Human Body 3D Reconstruction using a Single Flying Camera 
  * [Project Page](http://www.luvision.net/iHuman3D/) 
  * [Paper](http://www.luvision.net/iHuman3D/Fig/iHuman3D_camear_ready.pdf)
  * [Code / Executable Files (Reconstruction)](https://github.com/wchengad/iHuman3D) coming soon!

* FlyFusion: Realtime Dynamic Scene Reconstruction Using a Flying Depth Camera
  * [Project Page](http://www.luvision.net/FlyFusion_tvcg/) 
  * [Paper](http://www.luvision.net/FlyFusion_tvcg/Fig/FlyFusion-tvcg.pdf)
  * [Code / Executable Files (MoCap)](https://github.com/wchengad/FlyFusion) coming soon!

# install and requirement
* [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu) (only Ros indigo with Ubuntu 14.04 LTS tested)
* [armadillo](http://arma.sourceforge.net/)
* [blender](http://download.blender.org/release/Blender2.78/blender-2.78a-linux-glibc211-x86_64.tar.bz2)
* [libzmq](https://github.com/zeromq/libzmq)
* python3
  * numpy
  * opencv-python 

# run
roslaunch simulator.launch

# reference
* Synthetic Human Model: [Learning from Synthetic Humans (SURREAL)](https://github.com/gulvarol/surreal)
* Trajectory Planner: [Online Safe Trajectory Generation For Quadrotors Using Fast Marching Method and Bernstein Basis Polynomial](https://github.com/HKUST-Aerial-Robotics/Btraj)
