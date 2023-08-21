#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class RandomPermutationHash {
 public:
  RandomPermutationHash(const std::size_t i, std::size_t prime = 999999999989)
      : prime_(prime) {
    std::random_device rd;
    std::mt19937_64 generator(rd());
    std::uniform_int_distribution<std::size_t> distribution(1, prime - 1);

    a_ = distribution(generator);
    b_ = distribution(generator);
  }

  std::size_t operator()(std::size_t identifier) const {
    return (a_ * identifier + b_) % prime_;
  }

 private:
  std::size_t a_;
  std::size_t b_;
  std::size_t prime_;
};

class MinHash {
 public:
  MinHash(const std::size_t num_hash_function,
          const std::size_t max_identifier = 999999999989)
      : num_hash_function_(num_hash_function) {
    for (std::size_t i = 0; i < num_hash_function; ++i) {
      RandomPermutationHash hash(max_identifier);
      hash_functions_.push_back(hash);
    }
  }

  std::vector<std::size_t> operator()(
      const std::vector<std::size_t>& identifiers) const {
    std::size_t num_identifiers = identifiers.size();
    std::vector<std::size_t> signature(num_hash_function_,
                                       std::numeric_limits<std::size_t>::max());

    for (std::size_t i = 0; i < num_hash_function_; ++i) {
      const auto& hash = hash_functions_[i];
      for (std::size_t j = 0; j < num_identifiers; ++j) {
        std::size_t identifier = identifiers[j];
        std::size_t val = hash(identifier);

        if (val < signature[i]) {
          signature[i] = val;
        }
      }
    }

    return signature;
  }

  std::size_t num_hash_function() const { return num_hash_function_; }

 private:
  std::vector<RandomPermutationHash> hash_functions_;
  std::size_t num_hash_function_;
};

// Will return a std::pair<std::size_t, double>
// frist: number of same item, second: Jaccard similarity score.
std::pair<std::size_t, double> ComputeSetSimilarityScore(
    const std::vector<std::size_t>& set1,
    const std::vector<std::size_t>& set2,
    const std::pair<bool, bool> need_sort = std::make_pair(true, true)) {
  std::vector<std::size_t> sorted_set1(set1);
  std::vector<std::size_t> sorted_set2(set2);

  if (need_sort.first) {
    std::sort(sorted_set1.begin(), sorted_set1.end());
  }
  if (need_sort.second) {
    std::sort(sorted_set2.begin(), sorted_set2.end());
  }

  std::size_t size1 = sorted_set1.size();
  std::size_t size2 = sorted_set2.size();

  std::size_t intersection_size = 0;
  std::size_t i = 0;
  std::size_t j = 0;

  while (i < size1 && j < size2) {
    if (sorted_set1[i] < sorted_set2[j]) {
      ++i;
    } else if (sorted_set1[i] > sorted_set2[j]) {
      ++j;
    } else {
      ++intersection_size;
      ++i;
      ++j;
    }
  }

  std::size_t union_size = size1 + size2 - intersection_size;
  double score =
      static_cast<double>(intersection_size) / static_cast<double>(union_size);
  return std::make_pair(intersection_size, score);
}

// Two input signature must have same size, if not will retrun (0, -1)
// Will return a std::pair<std::size_t, double>
// frist: number of same item, second: similarity ratio.
std::pair<std::size_t, double> ComputeSignatureSimilaritySocre(
    const std::vector<std::size_t>& sig1,
    const std::vector<std::size_t>& sig2) {
  if (sig1.size() != sig2.size()) {
    return std::make_pair(0, -1);
  }

  std::size_t intersection_size = 0;
  std::size_t total_size = sig1.size();
  for (int i = 0; i < total_size; ++i) {
    if (sig1[i] == sig2[i]) {
      intersection_size += 1;
    }
  }

  double score =
      static_cast<double>(intersection_size) / static_cast<double>(total_size);
  return std::make_pair(intersection_size, score);
}

class BandLocalitySensitiveHash {
 public:
  BandLocalitySensitiveHash(const std::size_t num_bands,
                            const std::size_t num_rows)
      : num_bands_(num_bands), num_rows_(num_rows) {
    signature_size_ = num_bands_ * num_rows_;
  }

  std::vector<std::size_t> operator()(
      const std::vector<std::size_t>& signature) const {
    if (signature.size() < signature_size_) {
      return {};
    }

    std::vector<std::size_t> result(num_bands_);
    for (std::size_t id_band = 0; id_band < num_bands_; ++id_band) {
      // key of this band
      std::size_t key = 0;
      for (std::size_t id_row = 0; id_row < num_rows_; ++id_row) {
        std::size_t index = id_band * num_rows_ + id_row;
        std::size_t value = signature[index];
        // hash_combine is used for compress a signature segment into a key
        key = hash_combine(key, value);
      }
      result[id_band] = key;
    }

    return result;
  }

 private:
  std::size_t num_bands_;
  std::size_t num_rows_;
  std::size_t signature_size_;

  std::size_t hash_combine(std::size_t seed, std::size_t value) const {
    // Combine the hash using a simple bitwise XOR operation
    return seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2));
  }
};

class SetSimilarityRetrievaler {
 public:
  struct SetInfo {
    std::size_t set_id;
    std::vector<std::size_t> set;
    std::vector<std::size_t> signature;
  };

  SetSimilarityRetrievaler(const std::size_t num_bands,
                           const std::size_t num_rows,
                           const double threshold)
      : num_bands_(num_bands),
        num_rows_(num_rows),
        signature_size_(num_bands * num_rows),
        threshold_(threshold),
        min_hash_(num_bands * num_rows),
        band_lsh_(num_bands, num_rows),
        band_hash_tables_(num_bands) {}

  void AddSet(const std::vector<std::size_t>& set, const int set_id) {
    SetInfo set_info;
    set_info.set = set;
    set_info.set_id = set_id;

    std::vector<std::size_t> signature = min_hash_(set);
    set_info.signature = signature;

    // update retrival band hash table
    std::vector<std::size_t> band_keys = band_lsh_(signature);
    for (std::size_t i = 0; i < num_bands_; ++i) {
      std::size_t band_key = band_keys[i];
      band_hash_tables_[i][band_key].insert(set_id);
    }

    original_sets_[set_id] = set_info;
    last_set_id_ = set_id;
  }

  std::vector<std::pair<int, double>> RetrieveSimilarSets(
      const std::vector<std::size_t>& query) {
    std::vector<std::size_t> query_signature = min_hash_(query);
    std::vector<std::size_t> query_band_keys = band_lsh_(query_signature);
    std::vector<std::pair<int, double>> results;

    // first: set_id, second: collision counter
    std::unordered_map<int, int> candidates;
    // first: set_id, second: similarity_score
    std::unordered_map<int, double> similarity_scores;

    for (std::size_t i = 0; i < num_bands_; ++i) {
      std::size_t band_key = query_band_keys[i];
      // hash table of current band
      auto& current_table = band_hash_tables_[i];
      if (current_table.find(band_key) == current_table.end()) {
        // no match element
        continue;
      }

      auto& current_bucket = current_table[band_key];
      for (const auto& set_id : current_bucket) {
        if (last_set_id_ - set_id < num_exclude_frame_) {
          continue;
        }

        if (candidates.find(set_id) == candidates.end()) {
          candidates.insert(std::make_pair(set_id, 0));
        }
        candidates[set_id] += 1;
      }
    }

    // first: set_id, second: collision number
    std::vector<std::pair<int, int>> candidates_for_sort(candidates.begin(),
                                                         candidates.end());
    std::sort(
        candidates_for_sort.begin(), candidates_for_sort.end(),
        [&](const std::pair<int, int> lhs, const std::pair<int, int> rhs) {
          return lhs.second > rhs.second;
        });
    for (auto& entry : candidates_for_sort) {
      auto& set_id = entry.first;
      auto& collision_count = entry.second;

      const auto& set_match = original_sets_[set_id].set;
      const auto& candidate_signature = original_sets_[set_id].signature;
      auto [num_intersection, score] =
          ComputeSignatureSimilaritySocre(query_signature, candidate_signature);
      if (score < threshold_) {
        continue;
      }

      results.push_back(std::make_pair(set_id, score));
      if (results.size() >= max_similarity_check_num_) {
        break;
      }
    }

    // sort by score
    std::sort(results.begin(), results.end(),
              [&](const std::pair<int, double> lhs,
                  const std::pair<int, double> rhs) {
                return lhs.second > rhs.second;
              });
    if (results.size() > max_result_size_) {
      results.resize(max_result_size_);
    }
    return results;
  }

  auto& GetSet(const int set_id) { return original_sets_[set_id].set; }

 private:
  // first: band_key, second: all set_id that include this key
  std::vector<std::unordered_map<std::size_t, std::unordered_set<int>>>
      band_hash_tables_;
  std::unordered_map<int, SetInfo> original_sets_;
  std::size_t num_bands_;
  std::size_t num_rows_;
  std::size_t signature_size_;
  double threshold_;
  MinHash min_hash_;
  BandLocalitySensitiveHash band_lsh_;
  int max_similarity_check_num_ = 1000;
  int max_result_size_ = 10;
  int num_exclude_frame_ = 100;
  int last_set_id_ = 0;
};
