/*
  Author: Zheng Zhang
  Created: 2023-3-20
  Description: Tool function for read lidar point cloud form .bin file.
*/

#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// read KITTI's point cloud with '.bin' file
// Note: also work for Mulran dataset's .bin point cloud file
bool ReadLidarFromKittiBin(const std::string lidar_data_path,
                           pcl::PointCloud<pcl::PointXYZI>& cloud) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Empty file read End..." << std::endl;
    return false;
  }

  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));

  cloud.clear();
  pcl::PointXYZI point;
  for (std::size_t i = 0; i < lidar_data_buffer.size(); i += 4) {
    point.x = lidar_data_buffer[i];
    point.y = lidar_data_buffer[i + 1];
    point.z = lidar_data_buffer[i + 2];
    point.intensity = lidar_data_buffer[i + 3];
    cloud.push_back(point);
  }

  return true;
}

// not read 'intensity' field
bool ReadLidarFromKittiBin(const std::string lidar_data_path,
                           pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Empty file read End..." << std::endl;
    return false;
  }

  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));

  cloud.clear();
  pcl::PointXYZ point;
  for (std::size_t i = 0; i < lidar_data_buffer.size(); i += 4) {
    point.x = lidar_data_buffer[i];
    point.y = lidar_data_buffer[i + 1];
    point.z = lidar_data_buffer[i + 2];
    cloud.push_back(point);
  }

  return true;
}

// read KITTI's label and object id from '.label' file
bool ReadKittiLabel(const std::string label_data_path,
                    std::vector<uint16_t>& labels,
                    std::vector<uint16_t>& obj_ids) {
  std::ifstream file(label_data_path, std::ifstream::in | std::ifstream::binary);
  if (!file) {
    std::cout << label_data_path << " open failed !" << std::endl;
    return false;
  }

  // Stop eating new lines in binary mode!!!
  file.unsetf(std::ios::skipws);

  // get its size:
  std::streampos element_number;
  file.seekg(0, std::ios::end);
  element_number = file.tellg() / sizeof(uint32_t);
  file.seekg(0, std::ios::beg);

  //  a file XXXXXX.label in the labels folder that contains for each point a
  //  label in binary format. The label is a 32-bit unsigned integer (aka
  //  uint32_t) for each point, where the lower 16 bits correspond to the label.
  //  The upper 16 bits encode the instance id, which is temporally consistent
  //  over the whole sequence, i.e., the same object in two different scans gets
  //  the same id. This also holds for moving cars, but also static objects seen
  //  after loop closures.
  std::vector<uint16_t> vec(2 * element_number);
  file.read(reinterpret_cast<char*>(vec.data()),
            element_number * sizeof(uint32_t));
  labels.clear();
  obj_ids.clear();
  labels.reserve(element_number);
  obj_ids.reserve(element_number);
  for (int i = 0; i < element_number; ++i) {
    labels.push_back(vec[2 * i]);
    obj_ids.push_back(vec[2 * i + 1]);
  }

  return true;
}

// Generate label file's path for sementic kitti
std::string GenerateKittiLabelPath(const std::string& folder_path,
                                   const int& id) {
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0')
     << id;  // Format ID with leading zeros

  std::string file_name = ss.str();
  std::string file_path = folder_path + "/" + file_name + ".label";
  return file_path;
}