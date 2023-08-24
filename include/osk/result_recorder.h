#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>

class ResultRecorder {
 public:
  using ScanID = int;
  using ScoreType = double;

  struct ResultItem {
    ResultItem(const ScanID query_id_in,
               const ScanID match_id_in = -1,
               const ScoreType score_in = 0,
               const Eigen::Matrix4f m_to_q = Eigen::Matrix4f::Identity())
        : query_id(query_id_in),
          match_id(match_id_in),
          score(score_in),
          T_match_to_query(m_to_q) {}
    ScanID query_id;
    ScanID match_id;
    ScoreType score;
    Eigen::Matrix4f T_match_to_query;
  };

  void AddResult(const ScanID query_id_in,
                 const ScanID match_id_in = -1,
                 const ScoreType score_in = 0,
                 const Eigen::Matrix4f m_to_q = Eigen::Matrix4f::Identity()) {
    results_.emplace_back(query_id_in, match_id_in, score_in, m_to_q);
  }

  void WriteSearchResult(const std::string of_path) {
    std::ofstream file(of_path);
    for (auto& result : results_) {
      file << result.query_id << " " << result.match_id << " "
           << result.score;
      Eigen::Matrix<float, 3, 4> upper3x4 =
          result.T_match_to_query.topLeftCorner<3, 4>();
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
  std::vector<ResultItem> results_;
};
