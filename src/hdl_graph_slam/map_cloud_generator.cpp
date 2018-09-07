#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <pcl/octree/octree_search.h>

namespace hdl_graph_slam {

MapCloudGenerator::MapCloudGenerator() {
}

MapCloudGenerator::~MapCloudGenerator() {

}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution, float xRange, float yRange) const {
  if(keyframes.empty()) {
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    for(const auto& src_pt : keyframe->cloud->points) {
      if(fabs(src_pt.x) > xRange || fabs(src_pt.y) > yRange) {
        continue;
      }
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generateRawMap(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, float xRange, float yRange, float intensityScale) const {
  if(keyframes.empty()) {
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    for(const auto& src_pt : keyframe->cloud->points) {
      if(fabs(src_pt.x) > xRange || fabs(src_pt.y) > yRange) {
        continue;
      }
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity * intensityScale;
      if(dst_pt.intensity > 255) {
        dst_pt.intensity = 255;
      }
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  return cloud;
}

}
