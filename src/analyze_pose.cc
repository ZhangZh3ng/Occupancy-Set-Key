#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include "osk/data_reader.h"
#include "osk/kitti.h"
#include "osk/useful_tools.h"
#include "osk/timekeeper.h"

ros::Publisher pub_cloud_query;
ros::Publisher pub_cloud_match;
ros::Publisher pub_cloud_match_icp;
ros::Publisher pub_cloud_match_ransac;
ros::Publisher pub_cloud_match_ransac_icp;

std::string lidar_info_path;
std::string result_save_path;
std::string err_result_save_path;
double overlap_threshold;

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

struct FileRow {
  int queryFrameId;
  int matchFrameId;
  double similarityScore;
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
};

std::vector<FileRow> readFile(const std::string& filename) {
  std::vector<FileRow> data;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return data;
  }

  while (!file.eof()) {
    FileRow row;
    file >> row.queryFrameId >> row.matchFrameId >> row.similarityScore;
    for (int i = 0; i < 12; i++) {
      file >> row.transformation(i / 4, i % 4);
    }
    data.push_back(row);
  }

  file.close();
  return data;
}

float ComputeYawError(const Eigen::Matrix4f transformation) {
  return abs(atan2(transformation(1, 0), transformation(0, 0))) / M_PI * 180;
}

void Run() {
  DataReader reader(lidar_info_path);
  std::ofstream file(err_result_save_path);

  auto search_result = readFile(result_save_path);
  Timekeeper timer;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(0.5, 0.5, 0.5);

  int num_tp = 0;
  std::for_each(
      search_result.begin(), search_result.end(), [&](FileRow& entry) {
        while (!is_running) {
          if (!ros::ok()) {
            return;
          }
          std::this_thread::yield();
        }

        if (entry.similarityScore < overlap_threshold) {
          return;
        }

        auto query_frame_id = entry.queryFrameId;
        auto match_frame_id = entry.matchFrameId;

        ReadLidarFromKittiBin(reader.GetScanInfo(match_frame_id).file_path,
                              *cloud_source);
        ReadLidarFromKittiBin(reader.GetScanInfo(query_frame_id).file_path,
                              *cloud_target);
        Eigen::Matrix4f world_T_query =
            reader.GetScanInfo(query_frame_id).transformation.cast<float>();
        Eigen::Matrix4f world_T_match =
            reader.GetScanInfo(match_frame_id).transformation.cast<float>();
        Eigen::Matrix4f query_T_match =
            GetInverseTransformMatrix(world_T_query) * world_T_match;

        auto ransac_pose = entry.transformation;
        pcl::PointCloud<pcl::PointXYZ> cloud_match_ransac;
        pcl::transformPointCloud(*cloud_source, cloud_match_ransac, ransac_pose);

        filter.setInputCloud(cloud_source);
        filter.filter(*cloud_source);
        filter.setInputCloud(cloud_target);
        filter.filter(*cloud_target);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp, icp2;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cloud_target);

        pcl::PointCloud<pcl::PointXYZ> transformed_source;
        timer.Resume();
        icp.align(transformed_source);
        timer.Pause();
        auto t0 = timer.GetLastElapsedTime() / 1000;
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        Eigen::Matrix4f err = GetInverseTransformMatrix(query_T_match) * transformation;


        // icp with initial guess
        icp2.setInputSource(cloud_source);
        icp2.setInputTarget(cloud_target);

        pcl::PointCloud<pcl::PointXYZ> transformed_source2;
        timer.Resume();
        icp2.align(transformed_source2, ransac_pose.cast<float>());
        timer.Pause();
        auto t1 = timer.GetLastElapsedTime() / 1000;
        Eigen::Matrix4f transformation1 = icp2.getFinalTransformation();
        Eigen::Matrix4f err1 = GetInverseTransformMatrix(query_T_match) * transformation1;

        std_msgs::Header header;
        header.frame_id = "query";
        header.stamp = ros::Time::now();

        PublishCloud(*cloud_source, pub_cloud_match, header);
        PublishCloud(*cloud_target, pub_cloud_query, header);
        PublishCloud(cloud_match_ransac, pub_cloud_match_ransac, header);
        PublishCloud(transformed_source, pub_cloud_match_icp, header);
        PublishCloud(transformed_source2, pub_cloud_match_ransac_icp, header);

        std::cout << "t0 = " << t0 << " t1 = " << t1 << std::endl;
        std::cout << "err0 " << err.block<3, 1>(0, 3).norm() << " " << ComputeYawError(err) << " err1 : " <<err1.block<3, 1>(0, 3).norm() << " " << ComputeYawError(err1);

        file << err.block<3, 1>(0, 3).norm() << " " << ComputeYawError(err) << " " << err1.block<3, 1>(0, 3).norm() << " " << ComputeYawError(err1) << std::endl;


        num_tp += 1;
        std::cout << entry.queryFrameId << " " << entry.matchFrameId << " "
                  << entry.similarityScore << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      });

  std::cout << "read " << search_result.size() << " outcomes." << std::endl;

  std::cout << search_result.front().queryFrameId << std::endl;
  std::cout << search_result.back().queryFrameId << std::endl;
  std::cout << "num_tp " << num_tp << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "analyze_pose");
  ros::NodeHandle nh;

  nh.param<std::string>("lidar_info_path", lidar_info_path, "");
  nh.param<std::string>("result_save_path", result_save_path, "");
  nh.param<std::string>("err_result_save_path", err_result_save_path, "");
  nh.param<double>("overlap_threshold", overlap_threshold, 0.25);

  pub_cloud_query = nh.advertise<sensor_msgs::PointCloud2>("cloud_query", 10);
  pub_cloud_match = nh.advertise<sensor_msgs::PointCloud2>("cloud_match", 10);
  pub_cloud_match_icp =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_match_icp", 10);
  pub_cloud_match_ransac =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_match_ransac", 10);
  pub_cloud_match_ransac_icp =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_match_ransac_icp", 10);

  std::thread thread_pause(PauseControl);

  Run();

  return 0;
}