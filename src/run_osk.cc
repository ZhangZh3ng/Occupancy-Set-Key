#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <csignal>
#include <unistd.h>

#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "osk/data_reader.h"
#include "osk/kitti.h"
#include "osk/osk.h"
#include "osk/timekeeper.h"
#include "osk/useful_tools.h"

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
double overlap_grid_size;
int num_exclude_near_scan;
bool enable_pre_downsample;
double overlap_threshold;
std::string result_save_path;
std::string time_save_path;
std::string descriptor_save_folder;

double loop_dist_threshold;

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
ros::Publisher pub_path2;
ros::Publisher pub_loop_fp;
ros::Publisher pub_tp_points;
ros::Publisher pub_fp_points;
ros::Publisher pub_fn_points;

bool flag_termiate = false;

void SignalHandler(int signum) {
  flag_termiate = true;
  std::cout << "Interrupt signal (" << signum << ") received.\n";
  // exit(signum);
}

std::atomic<bool> is_running{false};
void PauseControl() {
  char c;
  while (ros::ok() && flag_termiate == false && read(STDIN_FILENO, &c, 1) == 1) {
    if (c == ' ') {
      is_running = !is_running;
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

  visualization_msgs::MarkerArray loop_marker, loop_marker_fp;
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
                         occupancy_context_angle_resolution,
                         overlap_grid_size,
                         num_exclude_near_scan};

  Timekeeper timer;

  Eigen::Vector3f t_add = Eigen::Vector3f{0, 0, 30};
  nav_msgs::Path path, path2;
  path.header.frame_id = "world";
  path2.header.frame_id = "world";
  std_msgs::Header header, header_world;

  tf::TransformBroadcaster broadcaster_this, broadcaster_match;

  std::vector<std::vector<double>> time_costs;

  pcl::PointCloud<pcl::PointXYZI>::Ptr position_cloud{
      new pcl::PointCloud<pcl::PointXYZI>};
  pcl::KdTreeFLANN<pcl::PointXYZI> position_kdtree;

  pcl::PointCloud<pcl::PointXYZ> tp_points;
  pcl::PointCloud<pcl::PointXYZ> fp_points;
  pcl::PointCloud<pcl::PointXYZ> fn_points;

  auto pub_messages = [&]() {
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

    pub_link_marker.publish(link_marker);
    pub_loop.publish(loop_marker);
    pub_loop_fp.publish(loop_marker_fp);
    pub_path.publish(path);
    pub_path2.publish(path2);

    PublishTF(broadcaster_this,
              reader.GetCurrentScanInfo().transformation.cast<float>(), "world",
              "lidar");
  };

  int scan_num = 0;
  while (ros::ok()) {
    while (!is_running) {
      if (!ros::ok()) {
        return;
      }
      std::this_thread::yield();
    }

    if (!reader.CurrentScanIsValid()) {
      std::cout << "all scan has been processed, exit." << std::endl;
      break;
    }

    ReadLidarFromKittiBin(reader.GetCurrentScanInfo().file_path, *cloud_raw);

    PublishTF(broadcaster_this,
              reader.GetCurrentScanInfo().transformation.cast<float>(), "world",
              "lidar");

    // add path
    auto pose = reader.GetCurrentScanInfo().transformation;
    Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
    geometry_msgs::Point point, point2;
    point.x = translation.x();
    point.y = translation.y();
    point.z = translation.z();
    path.poses.emplace_back();
    path.poses.back().pose.position = point;
    path.header.stamp = ros::Time::now();

    // make path2 higher
    point2.x = point.x + t_add.x();
    point2.y = point.y + t_add.y();
    point2.z = point.z + t_add.z();
    path2.poses.emplace_back();
    path2.poses.back().pose.position = point2;
    path2.header = path.header;

    int curr_scan_id = reader.GetCurrentScanInfo().scan_id;
    // std::string des_path =
    //     fp_des + std::to_string(reader.GetCurrentScanInfo().scan_id) + ".txt";

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
    auto results = osk_manager.VoteByOSK();
    timer.Pause();
    double t4 = timer.GetLastElapsedTime() / 1000;

    timer.Resume();
    // auto results = osk_manager.GeometryCheck();
    osk_manager.GeometryCheck();
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
    
    // cost_format: [scan_id, t1, t2+t3, t4+t5, t6]
    std::vector<double> cost{double(scan_num), t1, (t2+t3), (t4+t5), t6};
    time_costs.emplace_back(cost);

    // result process
    if (!results.empty()) {
      std::cout << "=== current scan id = "
                << reader.GetCurrentScanInfo().scan_id << std::endl;
      for (const auto& result : results) {
        auto& matched_scan_id = result.first;
        auto& score = result.second;
        std::cout << "Scan ID: " << matched_scan_id << ", Score: " << score
                  << std::endl;
      }
    }

    std::vector<Eigen::Vector3f> landmark_this;
    std::vector<Eigen::Vector3f> landmark_match;
    std::vector<Eigen::Vector3f> keypoints_match;
    auto best_id =
        osk_manager.GetBestOverlapMatchPairs(landmark_this, landmark_match);
    osk_manager.GetBestMatchLandmarks(keypoints_match);
    std::cout << " find  size = " << landmark_this.size() << " matching paris. "
              << " matched frame has " << keypoints_match.size() << " keypoints"
              << std::endl;

    auto best_result = osk_manager.GetBestScanID();

    bool detect_loop = false;
    if (best_result.second > overlap_threshold) {
      detect_loop = true;
    }

    link_marker.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar";
    marker.ns = "link";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;  // Clear old markers
    link_marker.markers.push_back(marker);

    pcl::PointXYZ position_curr{T_this_to_world(0, 3), T_this_to_world(1, 3),
                                T_this_to_world(2, 3)};
    bool has_true_loop = false;
    pcl::PointXYZI position_id;
    position_id.x = T_this_to_world(0, 3);
    position_id.y = T_this_to_world(1, 3);
    position_id.z = T_this_to_world(2, 3);
    position_id.intensity = curr_scan_id;  // use intensity field store scan_id
    // update 
    position_cloud->push_back(position_id);
    if (scan_num % 20 == 0) {
      position_kdtree.setInputCloud(position_cloud);
    }
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    if (position_kdtree.radiusSearch(position_id, loop_dist_threshold,
                                     point_indices, point_distances)) {
      const auto& cloud = position_kdtree.getInputCloud();
      for (size_t i = 0; i < point_indices.size(); ++i) {
        int point_index = point_indices[i];

        // Access the nearby point and its distance
        auto nearby_position_scan_id = cloud->points[point_index].intensity;
        if (curr_scan_id - nearby_position_scan_id > num_exclude_near_scan) {
          has_true_loop = true;
          break;
        }
      }
    }

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
      for (auto& point : landmark_match) {
        point += t_add;
      }

      Eigen::Matrix4f T_match_add = Eigen::Matrix4f::Identity();
      T_match_add.block<3, 1>(0, 3) = t_add;
      pcl::transformPointCloud(*cloud_match_origin, *cloud_match_origin_add,
                               T_match_add);

      GeneratePointCorrelationMarkers(landmark_this, landmark_match,
                                      link_marker, "lidar", Eigen::Vector4f{0, 0, 1, 1});

      // make node match higher for visualization.
      bool is_tp = (T_match_to_world.block<3, 1>(0, 3) -
                    T_this_to_world.block<3, 1>(0, 3))
                       .norm() < loop_dist_threshold;
      auto T_this_to_world_add = T_this_to_world;
      T_this_to_world_add.block<3, 1>(0, 3) += t_add;


      if (is_tp) {
        tp_points.emplace_back(position_curr);
      } else {
        fp_points.emplace_back(position_curr);
      }
    }
    if (has_true_loop && !detect_loop) {
      fn_points.emplace_back(position_curr);
    }

    osk_manager.GetDownsampledCloud(*cloud_ds);
    osk_manager.GetLandmarkCloud(*cloud_landmark);
    osk_manager.GetObjectCloud(*cloud_object);
    osk_manager.GetGroundCloud(*cloud_ground);
    osk_manager.GetObjectLessCloud(*cloud_object_less);

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
    pub_loop_fp.publish(loop_marker_fp);
    pub_path.publish(path);
    pub_path2.publish(path2);

    // marker:
    PublishCloud(tp_points, pub_tp_points, header_world);
    PublishCloud(fp_points, pub_fp_points, header_world);
    PublishCloud(fn_points, pub_fn_points, header_world);

    scan_num += 1;
    std::cout << "Frame " << scan_num << " ds size = " << cloud_ds->size()
              << " landmark: " << cloud_landmark->size() 
              << " object: " << cloud_object->size()
              << std::endl;
    osk_manager.RecordSearchResult();

    // write descriptor
    std::string des_path1 = descriptor_save_folder + "/" +
                           std::to_string(reader.GetCurrentScanInfo().scan_id) +
                           "_curr.txt";
    std::string des_path2 = descriptor_save_folder + "/" +
                           std::to_string(reader.GetCurrentScanInfo().scan_id) +
                           "_match.txt";
    osk_manager.WriteDescriptor2(des_path1, des_path2);

    // go to next scan
    if (!reader.TryMoveToNextScan()) {
      std::cout << "all scan has been processed, exit." << std::endl;
      break;
    }
  }
  osk_manager.WriteSearchResult(result_save_path);
  // write time cost
  std::ofstream file(time_save_path);
    for (auto& entry : time_costs) {
      for (auto& val : entry) {
        file << val << " ";
      }
      file << std::endl;
    }
  file.close();

  std::cout << "write ok." << std::endl;

  while (ros::ok() && flag_termiate == false) {
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
    pub_loop_fp.publish(loop_marker_fp);
    pub_path.publish(path);
    pub_path2.publish(path2);

    std::cout << "path length = " << path.poses.size() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "run_osk");
  ros::NodeHandle nh;
  signal(SIGINT, SignalHandler);

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
  nh.param<double>("overlap_grid_size", overlap_grid_size, 0.5);
  nh.param<int>("num_exclude_near_scan", num_exclude_near_scan, 300);

  nh.param<bool>("enable_pre_downsample", enable_pre_downsample, false);
  nh.param<double>("overlap_threshold", overlap_threshold, 0.5);
  nh.param<std::string>("result_save_path", result_save_path, "");
  nh.param<double>("loop_dist_threshold", loop_dist_threshold, 10);

  nh.param<std::string>("time_save_path", time_save_path, "");
  nh.param<std::string>("descriptor_save_folder", descriptor_save_folder, "");

  pub_cloud_this = nh.advertise<sensor_msgs::PointCloud2>("cloud_this", 10);
  pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground", 100);
  pub_object = nh.advertise<sensor_msgs::PointCloud2>("object", 100);
  pub_object_less = nh.advertise<sensor_msgs::PointCloud2>("object_less", 100);
  pub_landmark = nh.advertise<sensor_msgs::PointCloud2>("landmark", 100);
  pub_path = nh.advertise<nav_msgs::Path>("path", 10);
  pub_path2 = nh.advertise<nav_msgs::Path>("path2", 10);
  pub_loop = nh.advertise<visualization_msgs::MarkerArray>("loop_markers", 10);
  pub_loop_fp =
      nh.advertise<visualization_msgs::MarkerArray>("loop_markers_fp", 10);
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

  pub_tp_points = nh.advertise<sensor_msgs::PointCloud2>("tp_points", 100);
  pub_fp_points = nh.advertise<sensor_msgs::PointCloud2>("fp_points", 100);
  pub_fn_points = nh.advertise<sensor_msgs::PointCloud2>("fn_points", 100);

  std::thread thread_pause_control(PauseControl);

  RunOSKSearch();

  return 0;
}