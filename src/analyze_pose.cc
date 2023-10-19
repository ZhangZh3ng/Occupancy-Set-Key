#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "osk/data_reader.h"

std::string lidar_info_path;
std::string result_save_path;
double overlap_threshold;

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

void Run() {
  DataReader reader(lidar_info_path);

  auto search_result = readFile(result_save_path);

  std::cout << "read " << search_result.size() << " outcomes." << std::endl;

  std::cout << search_result.front().queryFrameId << std::endl;
  std::cout << search_result.back().queryFrameId << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "analyze_pose");
  ros::NodeHandle nh;

  nh.param<std::string>("lidar_info_path", lidar_info_path, "");
  nh.param<std::string>("result_save_path", result_save_path, "");
  nh.param<double>("overlap_threshold", overlap_threshold, 0.25);

  Run();

  return 0;
}