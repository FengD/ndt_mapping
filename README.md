hdl_graph_slam_package
===========================

### DESCRIPTION
This package is used to execute a lidar slam.

### RELATED Package
[calibration] (https://github.com/FengD/lidar_init_calibration) (provide an initializaition calibration yaml file)
[ndt_omp] (http://gitlabout.hirain.com/feng.ding1/hdl_mapping-ndt-omp.git) (provide a multi-thread ndt method)

### Dependencies
[g2o] (https://github.com/RainerKuemmerle/g2o.git): please roll back to the commit `a48ff8c42136f18fbe215b02bfeca48fa0c67507` and build. 

### BUILD

* create a new folder and a src folder in it `mkdir example`  `mkdir example/src`
* clone the project in the src folder `git clone http://gitlab.hirain.com/feng.ding1/hdl_mapping.git`
* `catkin_make -DCMAKE_BUILD_TYPE=Release`

### EXECUTE
* `roslaunch hdl_graph_slam hi_slam.launch bag_filename:=[bag file path]`

* `roslaunch hdl_graph_slam hi_slam_multi_frame.launch bag_filename:=[bag file path]`
