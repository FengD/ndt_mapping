#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <hdl_graph_slam/yaml_editor.hpp>

namespace hdl_graph_slam {

class InitCloudNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  InitCloudNodelet() {}
  virtual ~InitCloudNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();
    init_Calibration();


    points_sub = nh.subscribe(points_sub_topic, 64, &InitCloudNodelet::cloud_callback, this);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>(points_pub_topic, 32);
  }

private:
  void initialize_params() {
    points_sub_topic = private_nh.param<std::string>("points_sub_topic", "velodyne_points");
    points_pub_topic = private_nh.param<std::string>("points_pub_topic", "/transformed_velodyne_points");
    calibration_file_path = private_nh.param<std::string>("calibration_file_path", "");
  }

  void init_Calibration() {
    CalibrationYaml cYaml;
    cYaml.yamlRead(calibration_file_path);
    cb_init = cYaml.getCalibrationLidar();
  }

  pcl::PointCloud<PointT>::ConstPtr cloud_tranformer(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(calibration_file_path.empty()) {
      return cloud;
    }
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    Eigen::Translation3f tl(cb_init.x, cb_init.y, cb_init.z);                 // tl: translation
    Eigen::AngleAxisf rot_x(cb_init.roll, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y(cb_init.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(cb_init.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_rot_matrix = (tl * rot_x * rot_y * rot_z).matrix();
    pcl::transformPointCloud(*cloud, *filtered, tf_rot_matrix);

    filtered->header = cloud->header;

    return filtered;
  }



  void cloud_callback(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) {
    if(src_cloud->empty()) {
      return;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = cloud_tranformer(src_cloud);

    points_pub.publish(filtered);
  }


private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  std::string points_sub_topic, points_pub_topic, calibration_file_path;
  Calibration_Lidar cb_init;

};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::InitCloudNodelet, nodelet::Nodelet)
