/*
  Author: Zheng Zhang
  Created: 2023-6-15
  Description: A simple Locality Sensitive Hashing(LSH) class for searching sets
  with high similarity
*/

#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "osk/hash_combine.h"

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

// Band Locality Sensitive Hashing for convert MinHash signature into query key
std::vector<std::size_t> BandLSH(const std::vector<std::size_t>& signature,
                                 const int band_length = 5) {
  std::vector<std::size_t> query_key;
  for (std::size_t i = 0; i <= signature.size() - band_length;
       i += band_length) {
    std::size_t seed = 0;
    for (int j = 0; j < band_length; ++j) {
      hash_combine(seed, signature[i + j]);
    }
    query_key.emplace_back(seed);
  }
  return query_key;
}