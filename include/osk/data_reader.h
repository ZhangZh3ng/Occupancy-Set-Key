#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

class DataReader {
 public:
  struct ScanInfo {
    double timestamp = -1;
    int scan_id = -1;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    std::string file_path;
  };

  DataReader(const std::string& file_path) { ParseFile(file_path); }

  const auto& GetCurrentScanInfo() const { return *iter_curr_; }

  bool CurrentScanIsValid() const {
    return iter_curr_ != scan_infos_.end();
  }

  bool TryMoveToNextScan() {
    if (iter_curr_ == scan_infos_.end()) {
      return false;
    }

    ++iter_curr_;
    return true;
  }

  auto GetScanInfo(const int scan_id) {
    auto result = scan_infos_dictionary_[scan_id];
    return result;
  }

 private:
  void ParseFile(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cerr << "Failed to open file: " << file_path << std::endl;
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      ScanInfo scan_info;
      iss >> scan_info.scan_id >> scan_info.timestamp;

      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
          iss >> scan_info.transformation(i, j);
        }
      }

      iss >> scan_info.file_path;
      scan_infos_dictionary_[scan_info.scan_id] = scan_info;
      scan_infos_.emplace_back(scan_info);
    }

    iter_curr_ = scan_infos_.begin();
    std::cout << "Load " << scan_infos_.size() << " frames." << std::endl;
    file.close();
  }

  std::vector<ScanInfo> scan_infos_;
  std::unordered_map<int, ScanInfo> scan_infos_dictionary_;
  std::vector<ScanInfo>::iterator iter_curr_;
};