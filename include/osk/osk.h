/*
  Author: Zheng Zhang
  Created: 2023-6-2
  Description: A point cloud based place recognition algrithm.
*/

#pragma once

#include <algorithm>
#include <bitset>
#include <execution>
#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include "osk/hash_combine.h"
#include "osk/lsh.h"
#include "osk/ransac.h"
#include "osk/useful_tools.h"
#include "osk/voxel_id.h"

class OSKManager {
 public:
  using ScanID = int;
  using ScoreType = double;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  struct Voxel {
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f point_avg{0, 0, 0};
    int num_point = 0;
  };

  struct Pixel {
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f point_top;
    Eigen::Vector3f point_bottom;
    Eigen::Vector3f point_avg;
  };

  struct LandmarkPoint {
    LandmarkPoint(const Eigen::Vector3f p, const int num_t) : point(p) {
      fill_rate.resize(num_t, 0.0);
    }

    Eigen::Vector3f point;

    std::vector<float> fill_rate;
    std::unordered_set<ID3D> occupied_ids;
    std::vector<std::size_t> set;
    std::vector<std::size_t> signature;
    std::vector<std::size_t> query_key;
  };

  struct LandmarkLocation {
    LandmarkLocation(const int arg_scan_id, const int arg_point_id)
        : scan_id(arg_scan_id), point_id(arg_point_id) {}

    bool operator==(const LandmarkLocation& other) const {
      return (scan_id == other.scan_id && point_id == other.point_id);
    }

    struct Hash {
      std::size_t operator()(const LandmarkLocation& obj) const {
        std::size_t seed = 0;
        hash_combine(seed, obj.scan_id, obj.point_id);
        return seed;
      }
    };

    ScanID scan_id;
    int point_id;
  };

  struct CorrespondencePair {
    CorrespondencePair(const int curr_id, const int match_id, const double s)
        : curr_point_id(curr_id), match_point_id(match_id), score(s) {}
    int curr_point_id;
    int match_point_id;
    ScoreType score;
  };

  struct HistoricalScan {
    std::vector<Eigen::Vector3f> landmarks;
    std::vector<Eigen::Vector3f> registration_points;
  };

  struct QueryItem {
    std::vector<LandmarkLocation> locations;
    int historical_scan_num = 0;
    double score = 0.0;  // inverse document frequency(idf)
  };

  struct CandidateLog {
    ScoreType vote_score = 0;
    ScoreType overlap_score = 0;
    int num_ransac_inliers = 0;
    Eigen::Matrix4f relative_transformation = Eigen::Matrix4f::Identity();
  };

  struct SearchResult {
    SearchResult(const ScanID arg_curr_scan_id,
                 const ScanID arg_best_match_scan_id = -1,
                 const ScoreType arg_score = 0,
                 const Eigen::Matrix4f arg_T = Eigen::Matrix4f::Identity())
        : curr_scan_id(arg_curr_scan_id),
          best_match_scan_id(arg_best_match_scan_id),
          score(arg_score),
          relative_tranform(arg_T) {}
    ScanID curr_scan_id;
    ScanID best_match_scan_id;
    ScoreType score;
    Eigen::Matrix4f relative_tranform;
  };

  OSKManager(const double xy_leaf_size = 0.5,
             const double z_leaf_size = 0.5,
             const double landmark_range_threshold = 40,
             const int landmark_occupancy_threshold = 4,
             const double landmark_mask_radius = 5.0,
             const std::size_t lsh_band_length = 5,
             const std::size_t lsh_band_num = 5,
             const double occupancy_context_max_range = 20,
             const double occupancy_context_redius_resolution = 2,
             const double occupancy_context_angle_resolution = 6)
      : xy_leaf_size_(xy_leaf_size),
        z_leaf_size_(z_leaf_size),
        landmark_range_threshold_(landmark_range_threshold),
        landmark_occupancy_threshold_(landmark_occupancy_threshold),
        lsh_band_length_(lsh_band_length),
        lsh_band_num_(lsh_band_num),
        occupancy_context_max_range_(occupancy_context_max_range),
        occupancy_context_redius_resolution_(
            occupancy_context_redius_resolution),
        occupancy_context_angle_resolution_(occupancy_context_angle_resolution),
        min_hash_(lsh_band_length_ * lsh_band_num_),
        lidar_pose_(Eigen::Matrix4f::Identity()) {
    landmark_mask_area_ = ID2D::DisntanceInRadius(landmark_mask_radius);
    query_dictionaries_.resize(lsh_band_num_);

    occupancy_context_num_bins_redius_ = static_cast<int>(
        occupancy_context_max_range_ / occupancy_context_redius_resolution_);
    occupancy_context_num_bins_angle_ =
        static_cast<int>(std::ceil(360 / occupancy_context_angle_resolution_));

    std::cout << "min hash has " << min_hash_.num_hash_function()
              << " hash function." << std::endl;
    std::cout << "masked neighbor has " << landmark_mask_area_.size()
              << " elements."
              << " num bin radius = " << occupancy_context_num_bins_redius_
              << " num bin angle = " << occupancy_context_num_bins_angle_
              << std::endl;
  }

  // the overall searching process
  template<typename T>
  auto FindLoop(const pcl::PointCloud<T>& cloud_in, const ScanID curr_scan_id) {
    InsertPointCloud(cloud_in, curr_scan_id);
    FindLandmarkAndObjectPoints();
    MakeDescriptor();
    VoteByOSK();
    std::vector<std::pair<ScanID, ScoreType>> results = GeometryCheck();
    return results;
  }

  template <typename T>
  void InsertPointCloud(const pcl::PointCloud<T>& cloud_in,
                        const ScanID curr_scan_id) {
    curr_scan_id_ = curr_scan_id;

    map_.clear();
    bev_.clear();
    for (const auto& pt : cloud_in.points) {
      int idx = static_cast<int>(pt.x / xy_leaf_size_);
      int idy = static_cast<int>(pt.y / xy_leaf_size_);
      int idz = static_cast<int>(pt.z / z_leaf_size_);

      ID3D voxel_id(idx, idy, idz);
      auto iter = map_.find(voxel_id);
      if (iter == map_.end()) {
        iter = map_.emplace(voxel_id, Voxel{}).first;
      }
      auto& voxel = iter->second;

      // voxel.points.emplace_back(pt.x, pt.y, pt.z);
      voxel.point_avg += Eigen::Vector3f{pt.x, pt.y, pt.z};
      voxel.num_point += 1;
    }

    for (auto& entry : map_) {
      auto& voxel_id = entry.first;
      auto& voxel = entry.second;

      // voxel.point_avg.setZero();
      // for (auto& point : voxel.points) {
      //   voxel.point_avg += point;
      // }
      voxel.point_avg /= voxel.num_point;
      auto& point = voxel.point_avg;

      ID2D bev_id{voxel_id.x(), voxel_id.y()};
      auto iter = bev_.find(bev_id);
      if (iter == bev_.end()) {
        iter = bev_.emplace(bev_id, Pixel{}).first;
        auto& bev_pixel = iter->second;
        bev_pixel.point_bottom = point;
        bev_pixel.point_top = point;
        bev_pixel.points.push_back(point);
      } else {
        auto& bev_pixel = iter->second;
        bev_pixel.points.push_back(point);
        if (point.z() > bev_pixel.point_top.z()) {
          bev_pixel.point_top = point;
        } else if (point.z() < bev_pixel.point_bottom.z()) {
          bev_pixel.point_bottom = point;
        }
      }
    }
  }

  void FindLandmarkAndObjectPoints() {
    landmarks_.clear();
    object_points_.clear();
    registration_points_.clear();
    ground_points_.clear();

    landmarks_.reserve(map_.size());
    object_points_.reserve(map_.size());
    registration_points_.reserve(map_.size());

    // const iters directing to bev_'s elements.
    std::vector<decltype(bev_.cbegin())> iters;
    for (auto iter = bev_.cbegin(); iter != bev_.cend(); ++iter) {
      iters.push_back(iter);
    }
    std::sort(iters.begin(), iters.end(),
              [&](const auto& lhs, const auto& rhs) {
                return lhs->second.points.size() > rhs->second.points.size();
              });

    std::unordered_set<ID2D> masked_ids;
    for (auto& iter : iters) {
      auto& bev_id = iter->first;
      auto& bev_pixel = iter->second;

      // object points used in overlap computation.
      int num_object_points = 0;
      for (auto& point : bev_pixel.points) {
        if (point.z() - bev_pixel.point_bottom.z() > z_leaf_size_) {
          registration_points_.emplace_back(point);
          ++num_object_points;
        } else {
          ground_points_.emplace_back(point);
        }
      }

      // filter ground area
      if (num_object_points < 2) {
        continue;
      }

      // Only use the top and bottom point of each pixel grid for computing
      // occpancy context to reduce computational complexity and the
      // registration_points_ is what we called object points.
      object_points_.emplace_back(bev_pixel.point_top);
      object_points_.emplace_back(bev_pixel.point_bottom);

      if (num_object_points < landmark_occupancy_threshold_) {
        continue;
      }

      // Find landmark, firstly check if fullfill landmark condition
      auto dist_to_center =
          (bev_pixel.point_bottom - lidar_pose_.block<3, 1>(0, 3)).norm();
      if (bev_pixel.points.size() < landmark_occupancy_threshold_ ||
          dist_to_center > landmark_range_threshold_) {
        continue;
      }
      // check if it has been masked before.
      if (masked_ids.find(bev_id) != masked_ids.end()) {
        continue;
      }

      // all condition pass, add landmark
      landmarks_.emplace_back(
          (bev_pixel.point_bottom + bev_pixel.point_top) / 2.0,
          occupancy_context_num_bins_angle_);

      // mask neighbors
      for (auto& delta_id : landmark_mask_area_) {
        masked_ids.insert(bev_id + delta_id);
      }
    }
  }

  void MakeDescriptor() {
    std::for_each(
        std::execution::par_unseq, landmarks_.begin(), landmarks_.end(),
        [&](LandmarkPoint& landmark) {
          for (auto& objpoint : object_points_) {
            const Eigen::Vector2f& p1 = landmark.point.block<2, 1>(0, 0);
            const Eigen::Vector2f& p2 = objpoint.block<2, 1>(0, 0);
            int z = 1;  // current, not use z
            // z = static_cast<int>(
            //     (objpoint.z() - landmark.point.z() / z_leaf_size_));

            float dist = (p2 - p1).norm();
            float angle = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());

            int bin_angle_id =
                static_cast<int>(std::floor((angle + M_PI) / (2.0 * M_PI) *
                                            occupancy_context_num_bins_angle_));
            if (bin_angle_id < 0 ||
                bin_angle_id >= occupancy_context_num_bins_angle_) {
              continue;
            }

            // Calculate radial bin index
            int bin_radius_id = static_cast<int>(
                std::floor(dist / occupancy_context_redius_resolution_));
            if (bin_radius_id < 0 ||
                bin_radius_id >= occupancy_context_num_bins_redius_) {
              continue;
            }

            ID3D bin_id{bin_radius_id, bin_angle_id, z};
            auto success_insert = landmark.occupied_ids.insert(bin_id).second;
            if (success_insert) {
              landmark.fill_rate[bin_angle_id] += 1;
            }
          }

          // Create a vector of column indices for find pivot column
          std::vector<int> column_ids(occupancy_context_num_bins_angle_);
          std::iota(column_ids.begin(), column_ids.end(), 0);

          // Sort the column indices based on their fill rate using a lambda
          // function
          std::sort(
              column_ids.begin(), column_ids.end(), [&](int col1, int col2) {
                return landmark.fill_rate[col1] > landmark.fill_rate[col2];
              });

          // shift along with pivot column
          for (auto& first_col : column_ids) {
            for (auto& bin_id : landmark.occupied_ids) {
              const auto& row = bin_id.x();
              const auto& col = bin_id.y();
              // type is used to distinguish different types of information
              const auto& type = 1;
              int col_shifted = col - first_col;
              if (col_shifted < 0) {
                col_shifted += occupancy_context_num_bins_angle_;
              }
              // assign an unique ID for the occpancy property using hash trick.
              std::size_t seed = 0;
              hash_combine(seed, type, row, col_shifted);
              landmark.set.emplace_back(seed);
            }
            break; // only try once(the max occupancy direction)
          }

          // compute osk
          landmark.signature = min_hash_(landmark.set);
          landmark.query_key = BandLSH(landmark.signature, lsh_band_length_);
        });
  }

  std::vector<std::pair<ScanID, ScoreType>> VoteByOSK() {
    historical_scan_correspondences_.clear();

    std::vector<std::size_t> curr_point_ids(landmarks_.size());
    std::iota(curr_point_ids.begin(), curr_point_ids.end(), 0);

    std::vector<
        std::unordered_map<LandmarkLocation, ScoreType, LandmarkLocation::Hash>>
        matches_of_points(landmarks_.size());

    std::for_each(
        std::execution::par_unseq, curr_point_ids.begin(), curr_point_ids.end(),
        [&](const std::size_t curr_point_id) {
          const auto& keypoint = landmarks_[curr_point_id];
          auto& matches = matches_of_points[curr_point_id];
          for (int i = 0; i < lsh_band_num_; ++i) {
            const auto& key = keypoint.query_key[i];
            const auto& base = query_dictionaries_[i];

            const auto iter = base.find(key);
            if (iter == base.end()) {
              continue;
            }

            const auto& queried_item = iter->second;
            for (const auto& location : queried_item.locations) {
              if (curr_scan_id_ - location.scan_id < num_exclude_near_frame_) {
                continue;
              }

              // two point may match multiple times in different dictionary,
              // we will sum the score in voting, since that indicate more
              // similar.
              if (matches.find(location) == matches.end()) {
                matches[location] = queried_item.score;
              } else {
                matches[location] += queried_item.score;
              }
            }
          }
        });

    for (auto curr_point_id : curr_point_ids) {
      for (auto& entry : matches_of_points[curr_point_id]) {
        auto& location = entry.first;
        auto& score = entry.second;

        auto iter = historical_scan_correspondences_.find(location.scan_id);
        if (iter == historical_scan_correspondences_.end()) {
          std::vector<CorrespondencePair> correlations;
          iter = historical_scan_correspondences_
                     .emplace(location.scan_id, correlations)
                     .first;
        }

        auto& correspondences = iter->second;
        correspondences.emplace_back(curr_point_id, location.point_id, score);
      }
    }

    candidate_scan_vote_scores_.clear();
    for (auto& entry : historical_scan_correspondences_) {
      auto& scan_id = entry.first;
      auto& match_pairs = entry.second;

      ScoreType score = 0;
      for (auto& pair : match_pairs) {
        score += pair.score;
      }

      candidate_scan_vote_scores_.emplace_back(scan_id, score);
    }

    std::sort(candidate_scan_vote_scores_.begin(),
              candidate_scan_vote_scores_.end(),
              [&](auto& lhs, auto& rhs) { return lhs.second > rhs.second; });

    if (candidate_scan_vote_scores_.size() > max_candidate_num_) {
      candidate_scan_vote_scores_.resize(max_candidate_num_);
    }

    // update candidate log
    candidate_logs_.clear();
    for (auto& entry : candidate_scan_vote_scores_) {
      auto& scan_id = entry.first;
      auto& vote_score = entry.second;

      CandidateLog log;
      log.vote_score = vote_score;
      candidate_logs_.emplace(scan_id, log);
    }

    return candidate_scan_vote_scores_;
  }

  std::vector<std::pair<ScanID, ScoreType>> GeometryCheck() {
    if (candidate_scan_vote_scores_.empty()) {
      return {};
    }

    std::vector<std::pair<ScanID, ScoreType>> scan_overlap_scores;
    for (auto& entry : candidate_scan_vote_scores_) {
      auto& scan_id = entry.first;
      scan_overlap_scores.emplace_back(scan_id, 0);
    }

    auto occupy_set_curr = ConvertToOccupySet(registration_points_);
    std::vector<int> indices(candidate_scan_vote_scores_.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(
        std::execution::par_unseq, indices.begin(), indices.end(),
        [&](const int i) {
          auto& scan_id = candidate_scan_vote_scores_[i].first;
          auto& correspondences = historical_scan_correspondences_[scan_id];
          auto& matched_scan = *(historical_scans_[scan_id]);
          auto& log = candidate_logs_[scan_id];

          if (correspondences.size() < 4) {
            return;
          }

          std::vector<Eigen::Vector2f> source;
          std::vector<Eigen::Vector2f> target;
          target.reserve(correspondences.size());
          source.reserve(correspondences.size());
          for (auto& pair : correspondences) {
            source.emplace_back(
                matched_scan.landmarks[pair.match_point_id].head<2>());
            target.emplace_back(landmarks_[pair.curr_point_id].point.head<2>());
          }

          int num_ransac_inliers = 0;
          auto T2D = RANSAC(source, target, 30, 1.0, num_ransac_inliers);
          auto T3D = Convert2DTransformationTo3D(T2D);
          log.num_ransac_inliers = num_ransac_inliers;
          log.relative_transformation = T3D;

          auto& cloud = historical_scans_[scan_id]->registration_points;
          auto transformed_cloud = TransformPoints(cloud, T3D);

          auto occupy_set_match = ConvertToOccupySet(transformed_cloud);
          double overlap_score =
              ComputeJaccardSimilarity(occupy_set_match, occupy_set_curr);
          scan_overlap_scores[i].second = overlap_score;

          log.overlap_score = overlap_score;
        });

    std::sort(scan_overlap_scores.begin(), scan_overlap_scores.end(),
              [&](auto& lsh, auto& rhs) { return lsh.second > rhs.second; });

    candidate_scan_overlap_scores_ = scan_overlap_scores;
    if (scan_overlap_scores.empty()) {
      best_match_scan_id_ = -1;
    } else {
      best_match_scan_id_ = scan_overlap_scores.front().first;
    }

    return scan_overlap_scores;
  }

  void AddCurrentScanToDatabase() {
    int total_scan_num = historical_scans_.size() + 1;

    std::vector<std::unordered_set<std::size_t>> visited_keys_of_lsh_part(
        lsh_band_num_);
    // update query dictionary
    int point_id = 0;
    for (auto landmark : landmarks_) {
      for (int i = 0; i < lsh_band_num_; ++i) {
        auto& dictionary = query_dictionaries_[i];
        auto& key = landmark.query_key[i];
        auto& visted_key = visited_keys_of_lsh_part[i];

        auto iter = dictionary.find(key);
        if (iter == dictionary.end()) {
          iter = dictionary.emplace(key, QueryItem()).first;
        }
        auto& item = iter->second;
        item.locations.emplace_back(curr_scan_id_, point_id);
        visted_key.insert(key);
      }
      ++point_id;
    }

    // update score(inverse document frequency)
    std::vector<int> lsh_part_ids(lsh_band_num_);
    std::iota(lsh_part_ids.begin(), lsh_part_ids.end(), 0);
    std::for_each(std::execution::par_unseq, lsh_part_ids.begin(),
                  lsh_part_ids.end(), [&](const int i) {
                    auto& dictionary = query_dictionaries_[i];
                    auto& visted_keys = visited_keys_of_lsh_part[i];
                    for (auto& key : visted_keys) {
                      auto& item = dictionary[key];
                      item.historical_scan_num += 1;
                      item.score = std::log(double(total_scan_num) /
                                            double(item.historical_scan_num));
                    }
                  });

    // save point info of current scan
    historical_scans_[curr_scan_id_] = std::make_unique<HistoricalScan>();
    auto& curr_frame_info = *(historical_scans_[curr_scan_id_]);
    curr_frame_info.landmarks.reserve(landmarks_.size());
    for (auto& keypoint : landmarks_) {
      curr_frame_info.landmarks.emplace_back(keypoint.point);
    }
    curr_frame_info.registration_points = registration_points_;
  }

  ScanID GetBestVoteMatchPairs(std::vector<Eigen::Vector3f>& points_curr,
                               std::vector<Eigen::Vector3f>& points_match) {
    points_curr.clear();
    points_match.clear();
    if (candidate_scan_vote_scores_.empty()) {
      return -1;
    }

    const auto& best_match_id = candidate_scan_vote_scores_.front().first;
    const auto& matched_pairs = historical_scan_correspondences_[best_match_id];
    const auto& matched_database = historical_scans_[best_match_id];
    for (const auto& pair : matched_pairs) {
      points_curr.emplace_back(landmarks_[pair.curr_point_id].point);
      points_match.emplace_back(
          matched_database->landmarks[pair.match_point_id]);
    }

    return best_match_id;
  }

  ScanID GetBestOverlapMatchPairs(std::vector<Eigen::Vector3f>& points_curr,
                                  std::vector<Eigen::Vector3f>& points_match) {
    points_curr.clear();
    points_match.clear();
    if (candidate_scan_overlap_scores_.empty()) {
      return -1;
    }

    const auto& best_match_id = candidate_scan_overlap_scores_.front().first;
    const auto& matched_pairs = historical_scan_correspondences_[best_match_id];
    const auto& matched_database = historical_scans_[best_match_id];
    for (const auto& pair : matched_pairs) {
      points_curr.emplace_back(landmarks_[pair.curr_point_id].point);
      points_match.emplace_back(
          matched_database->landmarks[pair.match_point_id]);
    }

    return best_match_id;
  }

  void GetBestMatchLandmarks(std::vector<Eigen::Vector3f>& landmarks) {
    if (best_match_scan_id_ == -1) {
      return;
    }

    landmarks.clear();
    auto& best_match_frame = historical_scans_[best_match_scan_id_];
    for (auto& point : best_match_frame->landmarks) {
      landmarks.emplace_back(point);
    }
  }

  std::unordered_set<ID3D> ConvertToOccupySet(
      const std::vector<Eigen::Vector3f>& points) {
    std::unordered_set<ID3D> occupy_area;

    for (const auto& point : points) {
      int x = int(point.x() / overlap_grid_size_);
      int y = int(point.y() / overlap_grid_size_);
      int z = int(point.z() / overlap_grid_size_);
      ID3D id(x, y, z);
      occupy_area.insert(id);
    }
    return occupy_area;
  }

  template <typename T>
  double ComputeJaccardSimilarity(const std::unordered_set<T>& set1,
                                  const std::unordered_set<T>& set2) {
    int num_intersection = 0;
    for (const auto& entry : set2) {
      if (set1.find(entry) != set1.end()) {
        num_intersection += 1;
      }
    }
    int num_union = set1.size() + set2.size() - num_intersection;
    double jaccard_similarity = double(num_intersection) / double(num_union);
    return jaccard_similarity;
  }

  void set_lidar_pose(const Eigen::Matrix4f& pose) { lidar_pose_ = pose; }

  void ReportParameters() {
    std::cout << "REPORT: ====  ===== ==== " << std::endl;
    std::cout << "xy_leaf_size = " << xy_leaf_size_ << std::endl;
    std::cout << "z_leaf_size = " << z_leaf_size_ << std::endl;
    std::cout << "landmark_occupancy_threshold = "
              << landmark_occupancy_threshold_ << std::endl;
    std::cout << "landmark_range_threshold = " << landmark_range_threshold_
              << std::endl;
    std::cout << "occupancy_context_max_range = "
              << occupancy_context_max_range_ << std::endl;
    std::cout << "occupancy_context_redius_resolution = "
              << occupancy_context_redius_resolution_ << std::endl;
    std::cout << "occupancy_context_angle_resolution_= "
              << occupancy_context_redius_resolution_ << std::endl;
    std::cout << "occupancy_context size angle = "
              << occupancy_context_num_bins_angle_
              << " radius = " << occupancy_context_num_bins_redius_
              << std::endl;
    std::cout << "lsh_band_length = " << lsh_band_length_ << std::endl;
    std::cout << "lsh_band_num = " << lsh_band_num_ << std::endl;
    std::cout << "Report Done === " << std::endl;
  }

  void GetDownsampledCloud(Cloud& cloud_out) {
    cloud_out.clear();
    cloud_out.reserve(map_.size());
    pcl::PointXYZI pt;
    std::hash<int> int_hash;
    for (auto& entry : map_) {
      auto& voxel_id = entry.first;
      auto& voxel = entry.second;

      pt.x = voxel.point_avg.x();
      pt.y = voxel.point_avg.y();
      pt.z = voxel.point_avg.z();
      cloud_out.push_back(pt);
    }
  }

  void GetObjectCloud(Cloud& cloud_out, bool output_all = true) {
    cloud_out.clear();
    cloud_out.reserve(map_.size());
    pcl::PointXYZI pt;
    std::hash<int> int_hash;

    if (output_all) {
      for (auto& point : registration_points_) {
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        cloud_out.push_back(pt);
      }
    } else {
      for (auto& point : object_points_) {
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        cloud_out.push_back(pt);
      }
    }
  }

  void GetGroundCloud(Cloud& cloud_out) {
    cloud_out.clear();
    cloud_out.reserve(map_.size());
    pcl::PointXYZI pt;

    for (auto& point : ground_points_) {
      pt.x = point.x();
      pt.y = point.y();
      pt.z = point.z();
      cloud_out.push_back(pt);
    }
  }

  Eigen::Matrix4f GetBestRelativeTransform() {
    if (best_match_scan_id_ == -1) {
      std::cout << "relative transform empty" << std::endl;
      return Eigen::Matrix4f::Identity();
    }
    return candidate_logs_[best_match_scan_id_].relative_transformation;
  }

  void GetLandmarkCloud(Cloud& cloud_out) {
    cloud_out.clear();
    cloud_out.reserve(map_.size());
    pcl::PointXYZI pt;
    std::hash<int> int_hash;
    for (auto& keypoint : landmarks_) {
      pt.x = keypoint.point.x();
      pt.y = keypoint.point.y();
      pt.z = keypoint.point.z();
      cloud_out.push_back(pt);
    }
  }

  void GetHistoricalRegistrationCloud(const int scan_id, Cloud& cloud_out) {
    cloud_out.clear();
    if (historical_scans_.find(scan_id) == historical_scans_.end()) {
      return;
    }

    auto& cloud = historical_scans_[scan_id]->registration_points;
    for (auto& point : cloud) {
      pcl::PointXYZI pt;
      pt.x = point.x();
      pt.y = point.y();
      pt.z = point.z();
      cloud_out.emplace_back(pt);
    }
  }

  void GetHistoricalLandmarkCloud(const int scan_id, Cloud& cloud_out) {
    cloud_out.clear();
    if (historical_scans_.find(scan_id) == historical_scans_.end()) {
      return;
    }

    auto& cloud = historical_scans_[scan_id]->landmarks;
    for (auto& point : cloud) {
      pcl::PointXYZI pt;
      pt.x = point.x();
      pt.y = point.y();
      pt.z = point.z();
      cloud_out.emplace_back(pt);
    }
  }

  void RecordSearchResult() {
    SearchResult result(curr_scan_id_);
    if (best_match_scan_id_ != -1) {
      result.best_match_scan_id = best_match_scan_id_;
      result.score = candidate_scan_overlap_scores_.front().second;
      result.relative_tranform =
          candidate_logs_[best_match_scan_id_].relative_transformation;
    }
    search_results_.emplace_back(result);
  }

  void WriteSearchResult(const std::string of_path) {
    std::ofstream file(of_path);
    for (auto& result : search_results_) {
      file << result.curr_scan_id << " " << result.best_match_scan_id << " "
           << result.score;
      Eigen::Matrix<float, 3, 4> upper3x4 =
          result.relative_tranform.topLeftCorner<3, 4>();
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
          file << " " << upper3x4(row, col);
        }
      }
      file << std::endl;
    }
    file.close();
  }

 private:
  double xy_leaf_size_;
  double z_leaf_size_;
  Eigen::Matrix4f lidar_pose_;
  std::unordered_map<ID3D, Voxel> map_;
  std::unordered_map<ID2D, Pixel> bev_;

  float occupancy_context_redius_resolution_;
  float occupancy_context_max_range_;
  float occupancy_context_angle_resolution_;
  int occupancy_context_num_bins_angle_;
  int occupancy_context_num_bins_redius_;

  double landmark_occupancy_threshold_;
  double landmark_range_threshold_;
  std::vector<ID2D> landmark_mask_area_;

  std::vector<LandmarkPoint> landmarks_;
  std::vector<Eigen::Vector3f> object_points_;
  std::vector<Eigen::Vector3f> ground_points_;
  std::vector<Eigen::Vector3f> registration_points_;

  ScanID curr_scan_id_ = -1;
  ScanID best_match_scan_id_ = -1;
  int num_exclude_near_frame_ = 300;
  int max_candidate_num_ = 10;
  double overlap_grid_size_ = 0.5;
  std::unordered_map<ScanID, std::vector<CorrespondencePair>>
      historical_scan_correspondences_;
  std::vector<std::pair<ScanID, ScoreType>> candidate_scan_vote_scores_;
  std::vector<std::pair<ScanID, ScoreType>> candidate_scan_overlap_scores_;
  std::unordered_map<ScanID, CandidateLog> candidate_logs_;

  std::vector<std::unordered_map<std::size_t, QueryItem>> query_dictionaries_;
  std::unordered_map<ScanID, std::unique_ptr<HistoricalScan>> historical_scans_;

  std::vector<SearchResult> search_results_;

  std::size_t lsh_band_length_;
  std::size_t lsh_band_num_;
  MinHash min_hash_;
};