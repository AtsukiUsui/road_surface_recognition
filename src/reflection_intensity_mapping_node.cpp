#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "geometry_msgs/Point32.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_srvs/Empty.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "sensor_msgs/point_cloud_conversion.h"
#include <laser_geometry/laser_geometry.h>

#include <assert.h>
#include <time.h>

#include <road_surface_recognition/map.h>

#include <iostream>
#include <string>

class ReflectionIntensityMappingNode {
public:
  ReflectionIntensityMappingNode();

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud2);
  void requestMap();
  void map_update_cell2(double gx, double gy, double value);
  void makingOccupancyGridMap();
  map_t *convertMap(const nav_msgs::OccupancyGrid &map_msg);
  bool pubmapCallback(std_srvs::Empty::Request &request,
                      std_srvs::Empty::Response &response);

  /* Variable for map_update_cell2 function. */
  int *data1;
  int *data2;
  int map_H, map_W;
  double map_R;
  double map_CX, map_CY;

private:
  ros::Subscriber scan_sub;
  ros::Subscriber point_cloud_sub;
  ros::Publisher map_cloud_pub;
  ros::Publisher occupancyGrid_pub;
  ros::ServiceServer pub_server_;

  tf::TransformListener tf_;
  sensor_msgs::PointCloud2 cp_in_cloud;
  laser_geometry::LaserProjection projector_;

  std::string global_frame_id_, point_cloud_topic_name_;

  map_t *map_;

  ros::NodeHandle private_nh_;
};

ReflectionIntensityMappingNode::ReflectionIntensityMappingNode() {
  ros::NodeHandle nh;

  std::string point_cloud_topic_name;
  private_nh_.param("point_cloud_topic_name", point_cloud_topic_name_,
                    std::string("/disting_cloud2"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  point_cloud_sub =
      nh.subscribe(point_cloud_topic_name_, 10,
                   &ReflectionIntensityMappingNode::pointcloudCallback, this);
  map_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("map_cloud", 2, true);
  occupancyGrid_pub =
      nh.advertise<nav_msgs::OccupancyGrid>("lawnOccupancyGrid", 2, true);
  pub_server_ = nh.advertiseService(
      "up_map", &ReflectionIntensityMappingNode::pubmapCallback, this);

  requestMap();
}

void ReflectionIntensityMappingNode::pointcloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud2) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_point_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  ROS_INFO("Function in pointcloudCallback");

  /* Convert PointCloud2 to pcl data */
  cp_in_cloud = *point_cloud2;
  cp_in_cloud.fields[3].name = "intensity";
  pcl::fromROSMsg(cp_in_cloud, *pcl_point_cloud);

  /* Run an endpoint update until you run 'rosservice call /up_map'. */
  for (int i = 0; i < pcl_point_cloud->points.size(); i++) {
    ROS_INFO("map_update_cell");
    map_update_cell(map_, pcl_point_cloud->points[i].x,
                    pcl_point_cloud->points[i].y,
                    pcl_point_cloud->points[i].intensity);
  }
}

/* When rosservice call /up_map run, this function is called. */
bool ReflectionIntensityMappingNode::pubmapCallback(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  sensor_msgs::PointCloud map_cloud;
  sensor_msgs::ChannelFloat32 channel;
  sensor_msgs::PointCloud2 cloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);

  channel.name = std::string("intensity");
  channel.values.shrink_to_fit();
  map_cloud.channels.push_back(channel);

  /* Make a grid pcl data by converting occupancy grid map coordinate. */
  for (int i = 0; i < map_->size_x; i++) {
    ROS_INFO("Convert map data to pcd data");
    for (int j = 0; j < map_->size_y; j++) {
      if (MAP_VALID(map_, i, j)) {
        if (map_->cells[MAP_INDEX(map_, i, j)].average > 0.0) {
          geometry_msgs::Point32 point;
          point.x = MAP_WXGX(map_, i);
          point.y = MAP_WXGX(map_, j);
          map_cloud.points.push_back(point);
          map_cloud.channels.at(map_cloud.channels.size() - 1)
              .values.push_back(map_->cells[MAP_INDEX(map_, i, j)].average);
        }
      }
    }
  }

  map_cloud.header.frame_id = global_frame_id_;
  map_cloud.header.stamp = ros::Time::now();
  convertPointCloudToPointCloud2(map_cloud, cloud2);
  pcl::fromROSMsg(cloud2, *pcl_cloud);

  // Delete except data being intensity 100.
  for (int i = 0; i < pcl_cloud->points.size(); i++) {
    if (pcl_cloud->points[i].intensity != 100) {
      pcl_cloud->points[i].x = 0;
      pcl_cloud->points[i].y = 0;
      pcl_cloud->points[i].z = 0;
      pcl_cloud->points[i].intensity = 0;
    }
  }

  // Save raw point cloud data.
  pcl::io::savePCDFileASCII(
      "/home/atsuki/lab_ws/data/pcd/making_envir_cloud/test_raw.pcd",
      *pcl_cloud);
  ROS_INFO("Saving a raw pcd file Succeeded\n");
  ROS_INFO("Please wait because a raw pcd file is filtered now ...");

  /* Run an Outlier filter */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(pcl_cloud);
  sor.setMeanK(100);
  sor.setStddevMulThresh(2.0);
  sor.filter(*cloud_filtered);

  // Save filtered point cloud data.
  pcl::io::savePCDFileASCII(
      "/home/atsuki/lab_ws/data/pcd/making_envir_cloud/test_filtered.pcd",
      *cloud_filtered);
  ROS_INFO("Saving a filtered pcd file Succeeded");

  map_cloud_pub.publish(map_cloud);
  ROS_INFO("Published a PointCloud data of which topic name is map_cloud");

  /* Run endpoint update again because grid pcl data was filtered */
  for (int i = 0; i < cloud_filtered->points.size(); i++) {
    ROS_INFO("map_update_cell2");
    map_update_cell2(cloud_filtered->points[i].x, cloud_filtered->points[i].y,
                     cloud_filtered->points[i].intensity);
  }

  makingOccupancyGridMap();

  return true;
}

map_t *ReflectionIntensityMappingNode::convertMap(
    const nav_msgs::OccupancyGrid &map_msg) {
  map_t *map = map_alloc();

  map->size_x = map_W = map_msg.info.width;
  map->size_y = map_H = map_msg.info.height;
  map->scale = map_R = map_msg.info.resolution * 1.0;
  map->origin_x = map_CX =
      map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_CY =
      map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  map->cells =
      (map_cell_t *)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

  data2 = new int[map_W * map_H];
  data1 = new int[map_W * map_H];

  for (int i = 0; i < map_msg.data.size(); i++)
    data1[i] = map_msg.data[i];

  return map;
}

void ReflectionIntensityMappingNode::requestMap() {
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response resp;

  ROS_INFO("Requesting the map...");
  while (!ros::service::call("static_map", req, resp)) {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }

  /* Import map_server data */
  map_ = convertMap(resp.map);
}

void ReflectionIntensityMappingNode::makingOccupancyGridMap() {
  nav_msgs::OccupancyGrid lawn_occupancy;

  lawn_occupancy.header.frame_id = global_frame_id_;
  lawn_occupancy.info.width = map_W;
  lawn_occupancy.info.height = map_H;
  lawn_occupancy.info.resolution = map_R;
  lawn_occupancy.info.origin.position.x = map_CX - (map_W / 2) * map_R;
  lawn_occupancy.info.origin.position.y = map_CY - (map_H / 2) * map_R;
  lawn_occupancy.info.origin.position.z = 0.0;

  /* Make occupancy grid map which has lawn data.
   * This map is made from grid pcl data and a published occupancy grid map from
   * topic named map_server. */
  for (int i = 0; i < map_W * map_H; i++) {
    lawn_occupancy.data.push_back((int)data1[i]);
    if (lawn_occupancy.data[i] != 100 && (int)data2[i] == 100)
      lawn_occupancy.data[i] = (int)data2[i];
  }

  // If you want to save occupancy grid map that has lawn place data regarded as
  // occupation, Please run a following command. 'rosrun map_server map_saver -f
  // mapfilename map:=lawnOccupancyGrid'
  occupancyGrid_pub.publish(lawn_occupancy);
  ROS_INFO(
      "Published an OccupancyGrid data of which topic name is occupancyGrid");

  sleep(3);

  // 以下、自動でマップを保存してくれる機能
  ros::NodeHandle nh;
  // map_file_path パラメータの値を取得
  std::string map_file;
  if (!nh.getParam("/reflection_intensity_mapping_node/map_file_path",
                   map_file)) {
    ROS_ERROR("Failed to get 'map_file_path' parameter.");
  } else {
    // 拡張子が .yaml の場合、削除する
    if (map_file.size() >= 5 &&
        map_file.substr(map_file.size() - 5) == ".yaml") {
      map_file = map_file.substr(0, map_file.size() - 5);
    }
    // ROS_INFO("Map file path without .yaml extension: %s", map_file.c_str());
  }

  // ディレクトリとファイル名に分割
  size_t lastSlash = map_file.find_last_of("/"); // 最後の/のインデックスを示す
  if (lastSlash == std::string::npos) {
    ROS_ERROR("Invalid 'map_file' format.");
  }

  std::string directory = map_file.substr(0, lastSlash); // ディレクトリ
  std::string filename = map_file.substr(lastSlash + 1); // ファイル名

  // パラメータの値を取得
  double coefficient_0_double, coefficient_x_double, coefficient_x2_double;
  if (!nh.getParam("/making_envir_cloud/coefficient_0", coefficient_0_double) ||
      !nh.getParam("/making_envir_cloud/coefficient_x", coefficient_x_double) ||
      !nh.getParam("/making_envir_cloud/coefficient_x2",
                   coefficient_x2_double)) {
    ROS_ERROR("Failed to get coefficients from ROS parameters.");
  }

  // double 型を int 型に変換
  int coefficient_0 = static_cast<int>(coefficient_0_double);
  int coefficient_x = static_cast<int>(coefficient_x_double);
  int coefficient_x2 = static_cast<int>(coefficient_x2_double);

  // ファイル名を構築
  filename = filename + "__" + std::to_string(coefficient_x2) + "x^2+" +
             std::to_string(coefficient_x) + "x+" +
             std::to_string(coefficient_0);

  ROS_INFO("map_file_directory: %s", directory.c_str());
  ROS_INFO("map_file_name: %s", filename.c_str());

  // ディレクトリを移動
  if (chdir(directory.c_str()) != 0) {
    perror("chdir");
  }

  // 外部コマンドを実行
  std::string command = "rosrun map_server map_saver -f " + filename +
                        "_lawnOccupancyGridmap map:=lawnOccupancyGrid";
  int result = system(command.c_str());

  // 外部コマンドの実行結果をチェック
  if (result == 0) {
    ROS_INFO("map_saver executed successfully");
  } else {
    ROS_ERROR("map_saver execution failed");
  }

  sleep(10);

  return;
}

// Endpoint update function 2.
void ReflectionIntensityMappingNode::map_update_cell2(double gx, double gy,
                                                      double value) {
  double x = gx - map_CX;
  double y = gy - map_CY;
  int ix = (int)(x / map_R + map_W / 2 + 0.5);
  int iy = (int)(y / map_R + map_H / 2 + 0.5);
  // int ix = (int)((x + map_R / 2) / map_R + map_W / 2);
  // int iy = (int)((y + map_R / 2) / map_R + map_H / 2);

  // チェックして配列の範囲外アクセスを防止
  if (ix >= 0 && ix < map_W && iy >= 0 && iy < map_H) {
    int i = map_W * iy + ix;
    data2[abs(i)] = value;
  }

  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reflection_intensity_mapping");
  ReflectionIntensityMappingNode ri;

  ros::spin();

  return 0;
}
