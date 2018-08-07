hdl_graph_slam_package
===========================

### DESCRIPTION
This package is used to execute a lidar slam.

### PACKAGE NEEDED
[calibration] (http://gitlab.hirain.com/feng.ding1/hdl_calibration.git) (provide an initializaition calibration yaml file)
[ndt_omp] (http://gitlab.hirain.com/feng.ding1/ndt_omp.git) (provide a multi-thread ndt method)


### BUILD

* create a new folder and a src folder in it `mkdir example`  `mkdir example/src`
* clone the project in the src folder `git clone http://gitlab.hirain.com/feng.ding1/hdl_mapping.git`
* `catkin_make -DCMAKE_BUILD_TYPE=Release`

### EXECUTE
* `roslaunch hdl_graph_slam hi_slam.launch bag_filename:=[bag file path]`

* `roslaunch hdl_graph_slam hi_slam_multi_frame.launch bag_filename:=[bag file path]`
