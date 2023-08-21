#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "osk/kitti.h"
#include "osk/osk.h"
#include "osk/timekeeper.h"
#include "osk/useful_tools.h"
#include "osk/data_reader.h"

std::string fpath_gt_pose, fpath_lidar;
std::string lidar_info_path;
double xy_leaf_size, z_leaf_size;
double landmark_range_threshold;
int landmark_occupancy_threshold;
double landmark_mask_radius;
int lsh_band_length;
int lsh_band_num;
double occupancy_context_max_range;
double occupancy_context_redius_resolution;
double occupancy_context_angle_resolution;
bool enable_pre_downsample;
double overlap_threshold;
std::string result_save_path;

ros::Publisher pub_cloud_this;
ros::Publisher pub_ground;
ros::Publisher pub_object;
ros::Publisher pub_object_less;
ros::Publisher pub_landmark;
ros::Publisher pub_path;
ros::Publisher pub_loop;
ros::Publisher pub_link_marker;
ros::Publisher pub_cloud_match;
ros::Publisher pub_matched_keypoints;
ros::Publisher pub_cloud_match_origin;
ros::Publisher pub_cloud_match_origin_add;
ros::Publisher pub_cloud_match_transform;

std::atomic<bool> is_running{true};
void PauseControl() {
  while (ros::ok()) {
    char input;
    std::cin >> input;

    if (input == 'p') {
      is_running = false;
    } else if (input == 'r') {
      is_running = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void RunOSKSearch() {
  DataReader reader(lidar_info_path);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_this{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_landmark{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_less{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_match_raw{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_match_transform{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_match_origin{
          new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_match_origin_add{
          new pcl::PointCloud<pcl::PointXYZI>};

  std::string fp_des = "/media/zz/new/myMidImg/des/";

  visualization_msgs::MarkerArray loop_marker;
  visualization_msgs::MarkerArray link_marker;

  OSKManager osk_manager{xy_leaf_size,
                         z_leaf_size,
                         landmark_range_threshold,
                         landmark_occupancy_threshold,
                         landmark_mask_radius,
                         lsh_band_length,
                         lsh_band_num,
                         occupancy_context_max_range,
                         occupancy_context_redius_resolution,
                         occupancy_context_angle_resolution};

  Timekeeper timer;

  nav_msgs::Path path;
  path.header.frame_id = "world";
  std_msgs::Header header, header_world;

  tf::TransformBroadcaster broadcaster_this, broadcaster_match;

  ros::Rate r(2);
  int scan_num = 0;
  while (ros::ok()) {
    while (!is_running) {
      if (!ros::ok()) {
        return;
      }
      std::this_thread::yield();
    }

    bool has_new_scan = reader.MoveToNextScan();
    if (!has_new_scan) {
      std::cout << "all scan has been processed, exit." << std::endl;
      break;
    }

    ReadLidarFromKittiBin(reader.GetCurrentScanInfo().file_path, *cloud_raw);

    PublishTF(broadcaster_this,
              reader.GetCurrentScanInfo().transformation.cast<float>(),
              "world", "lidar");

    // add path
    auto pose = reader.GetCurrentScanInfo().transformation;
    Eigen::Vector3d translation = pose.block<3, 1>(0,3);
    geometry_msgs::Point point;
    point.x = translation.x();
    point.y = translation.y();
    point.z = translation.z();
    path.poses.emplace_back();
    path.poses.back().pose.position = point;
    path.header.stamp = ros::Time::now();

    int curr_scan_id = reader.GetCurrentScanInfo().scan_id;
    std::string des_path =
        fp_des + std::to_string(reader.GetCurrentScanInfo().scan_id) + ".txt";

    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(0.4, 0.4, 0.4);

    Eigen::Matrix4f T_this_to_world =
        reader.GetCurrentScanInfo().transformation.cast<float>();
    Eigen::Matrix4f T_match_to_world = Eigen::Matrix4f::Identity();

    header.stamp = ros::Time::now();
    bool work_in_world_frame = false;
    if (work_in_world_frame) {
      pcl::transformPointCloud(*cloud_raw, *cloud_this, T_this_to_world);
      header.frame_id = "world";
    } else {
      (*cloud_this) = (*cloud_raw);
      header.frame_id = "lidar";
    }
    
    header_world = header;
    header_world.frame_id = "world";

    filter.setInputCloud(cloud_this);
    filter.filter(*cloud_ds);
    if (enable_pre_downsample) {
      cloud_this = cloud_ds;
    }

    if (work_in_world_frame) {
      osk_manager.set_lidar_pose(T_this_to_world);
    }

    timer.Resume();
    osk_manager.InsertPointCloud(*cloud_this, curr_scan_id);
    timer.Pause();
    double t1 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    osk_manager.FindLandmarkAndObjectPoints();
    timer.Pause();
    double t2 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    osk_manager.MakeDescriptor();
    timer.Pause();
    double t3 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    osk_manager.VoteByOSK();
    timer.Pause();
    double t4 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    auto results = osk_manager.GeometryCheck();
    timer.Pause();
    double t5 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    osk_manager.AddCurrentScanToDatabase();
    timer.Pause();
    double t6 = timer.GetLastElapsedTime() / 1000;

    std::cout << "Time cost: t1 = " << t1 << " t2 = " << t2 << " t3 = " << t3
              << " t4 = " << t4 << " t5 = " << t5 << " t6 = " << t6
              << " this total = " << (t1 + t2 + t3 + t4 + t5 + t6)
              << " total avg = " << timer.GetOverallTime() / 1000 / scan_num
              << std::endl;

    // result process
    if (!results.empty()) {
      std::cout << "=== current scan id = " << reader.GetCurrentScanInfo().scan_id
                << std::endl;
      for (const auto& result : results) {
        auto& matched_scan_id = result.first;
        auto& score = result.second;
        std::cout << "Scan ID: " << matched_scan_id << ", Score: " << score << std::endl;
      }
    }

    std::vector<Eigen::Vector3f> landmark_this;
    std::vector<Eigen::Vector3f> landmark_match;
    std::vector<Eigen::Vector3f> keypoints_match;
    auto best_id = osk_manager.GetBestOverlapMatchPairs(landmark_this, landmark_match);
    osk_manager.GetBestMatchLandmarks(keypoints_match);
    std::cout << " find  size = " << landmark_this.size() << " matching paris. "
              << " matched frame has " << keypoints_match.size() << " keypoints"
              << std::endl;

    bool detect_loop = false;
    if (!results.empty() && results.front().second > overlap_threshold) {
      detect_loop = true;
    }

    link_marker.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar";
    marker.ns = "link";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;  // Clear old markers
    link_marker.markers.push_back(marker);
    if (detect_loop) {
      ReadLidarFromKittiBin(reader.GetScanInfo(best_id).file_path,
                            *cloud_match_raw);
      T_match_to_world =
          reader.GetScanInfo(best_id).transformation.cast<float>();
      // convert to world frame
      pcl::transformPointCloud(*cloud_match_raw, *cloud_match_raw,
                               T_match_to_world);

      osk_manager.GetHistoricalRegistrationCloud(best_id, *cloud_match_origin);

      auto T_match_to_this = osk_manager.GetBestRelativeTransform();
      pcl::transformPointCloud(*cloud_match_origin, *cloud_match_transform,
                               T_match_to_this);

      // landmark correlation.
      Eigen::Vector3f t_add = Eigen::Vector3f{0, 0, 10};
      for (auto& point : landmark_match) {
        point += t_add;
      }

      Eigen::Matrix4f T_match_add = Eigen::Matrix4f::Identity();
      T_match_add.block<3, 1>(0, 3) = t_add;
      pcl::transformPointCloud(*cloud_match_origin, *cloud_match_origin_add,
                               T_match_add);

      GeneratePointCorrelationMarkers(landmark_this, landmark_match, link_marker,
                                      "lidar");

      loop_marker.markers.push_back(GenerateLoopMarker(
          T_this_to_world, T_match_to_world, "world", scan_num));
    }

    osk_manager.GetDownsampledCloud(*cloud_ds);
    osk_manager.GetLandmarkCloud(*cloud_landmark);
    osk_manager.GetObjectCloud(*cloud_object);
    osk_manager.GetGroundCloud(*cloud_ground);
    osk_manager.GetObjectCloud(*cloud_object_less, false);
    std::cout << "find " << cloud_landmark->size() << " keypoints." << std::endl;

    if (scan_num % 20 == 0) {
      osk_manager.ReportParameters();
    }

    // current
    PublishCloud(*cloud_this, pub_cloud_this, header);
    PublishCloud(*cloud_object, pub_object, header);
    PublishCloud(*cloud_landmark, pub_landmark, header);
    PublishCloud(*cloud_ground, pub_ground, header);
    PublishCloud(*cloud_object_less, pub_object_less, header);

    // match
    PublishCloud(*cloud_match_raw, pub_cloud_match, header_world);
    PublishCloud(*cloud_match_origin, pub_cloud_match_origin, header);
    PublishCloud(*cloud_match_transform, pub_cloud_match_transform, header);
    PublishCloud(*cloud_match_origin_add, pub_cloud_match_origin_add, header);
    cloud_match_raw->clear();
    cloud_match_origin->clear();
    cloud_match_transform->clear();
    cloud_match_origin_add->clear();

    pub_link_marker.publish(link_marker);
    pub_loop.publish(loop_marker);
    pub_path.publish(path);

    scan_num += 1;
    std::cout << "frame " << scan_num << " ds size = " << cloud_ds->size()
              << std::endl;
    osk_manager.RecordSearchResult();
  }
  osk_manager.WriteSearchResult(result_save_path);
  std::cout << "write ok." << std::endl;

  while (ros::ok()) {
    // current
    PublishCloud(*cloud_this, pub_cloud_this, header);
    PublishCloud(*cloud_object, pub_object, header);
    PublishCloud(*cloud_landmark, pub_landmark, header);
    PublishCloud(*cloud_ground, pub_ground, header);

    // match
    PublishCloud(*cloud_match_raw, pub_cloud_match, header_world);
    PublishCloud(*cloud_match_origin, pub_cloud_match_origin, header);
    PublishCloud(*cloud_match_transform, pub_cloud_match_transform, header);
    PublishCloud(*cloud_match_origin_add, pub_cloud_match_origin_add, header);

    pub_link_marker.publish(link_marker);
    pub_loop.publish(loop_marker);
    pub_path.publish(path);

    std::cout << "path length = " << path.poses.size() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "run_osk");
  ros::NodeHandle nh;

  nh.param<std::string>("fpath_gt_pose", fpath_gt_pose, "");
  nh.param<std::string>("fpath_lidar", fpath_lidar, "");
  nh.param<std::string>("lidar_info_path", lidar_info_path, "");

  nh.param<double>("xy_leaf_size", xy_leaf_size, 0.3);
  nh.param<double>("z_leaf_size", z_leaf_size, 0.3);
  nh.param<int>("landmark_occupancy_threshold", landmark_occupancy_threshold,
                4);
  nh.param<double>("landmark_range_threshold", landmark_range_threshold, 40);
  nh.param<double>("landmark_mask_radius", landmark_mask_radius, 2);
  nh.param<int>("lsh_band_length", lsh_band_length, 5);
  nh.param<int>("lsh_band_num", lsh_band_num, 10);
  nh.param<double>("occupancy_context_max_range", occupancy_context_max_range,
                   20);
  nh.param<double>("occupancy_context_redius_resolution",
                   occupancy_context_redius_resolution, 2);
  nh.param<double>("occupancy_context_angle_resolution",
                   occupancy_context_angle_resolution, 6);

  nh.param<bool>("enable_pre_downsample", enable_pre_downsample, false);
  nh.param<double>("overlap_threshold", overlap_threshold, 0.5);
  nh.param<std::string>("result_save_path", result_save_path, "");

  pub_cloud_this = nh.advertise<sensor_msgs::PointCloud2>("cloud_this", 10);
  pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground", 100);
  pub_object = nh.advertise<sensor_msgs::PointCloud2>("object", 100);
  pub_object_less = nh.advertise<sensor_msgs::PointCloud2>("object_less", 100);
  pub_landmark = nh.advertise<sensor_msgs::PointCloud2>("landmark", 100);
  pub_path = nh.advertise<nav_msgs::Path>("path", 10);
  pub_loop = nh.advertise<visualization_msgs::MarkerArray>("loop_markers", 10);
  pub_link_marker =
      nh.advertise<visualization_msgs::MarkerArray>("line_markers", 10);
  pub_matched_keypoints =
      nh.advertise<sensor_msgs::PointCloud2>("matched_keypoints", 100);

  pub_cloud_match = nh.advertise<sensor_msgs::PointCloud2>("cloud_match", 100);
  pub_cloud_match_origin =
      nh.advertise<sensor_msgs::PointCloud2>("match_origin", 100);
  pub_cloud_match_transform =
      nh.advertise<sensor_msgs::PointCloud2>("match_transform", 100);
  pub_cloud_match_origin_add =
      nh.advertise<sensor_msgs::PointCloud2>("match_origin_add", 100);

  std::thread thread_pause_control(PauseControl);

  RunOSKSearch();

  thread_pause_control.join();

  return 0;
}