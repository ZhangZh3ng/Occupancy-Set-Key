/*
  Author: Zheng Zhang
  Created: 2023-5-11
  Description: A simple index to indicate 3D voxel's ID
*/

#pragma once

#include <vector>

#include "osk/hash_combine.h"

struct ID2D {
 public:
  ID2D(const int x = 0, const int y = 0) : x_(x), y_(y) {}

  bool operator==(const ID2D& other) const {
    return x_ == other.x() && y_ == other.y();
  }

  ID2D operator+(const ID2D& other) const {
    return ID2D{x_ + other.x(), y_ + other.y()};
  }

  ID2D operator-(const ID2D& other) const {
    return ID2D{x_ - other.x(), y_ - other.y()};
  }

  // 4 neighbor
  std::vector<ID2D> Neighbors() const {
    std::vector<ID2D> result;
    result.reserve(4);
    result.emplace_back(x_ + 1, y_);
    result.emplace_back(x_ - 1, y_);
    result.emplace_back(x_, y_ + 1);
    result.emplace_back(x_, y_ - 1);
    return result;
  }

  // 8 neighbor
  std::vector<ID2D> Neighbors8() const {
    std::vector<ID2D> result;
    result.reserve(8);
    result.emplace_back(x_ + 1, y_);
    result.emplace_back(x_ - 1, y_);
    result.emplace_back(x_, y_ + 1);
    result.emplace_back(x_, y_ - 1);
    result.emplace_back(x_ + 1, y_ + 1);
    result.emplace_back(x_ - 1, y_ + 1);
    result.emplace_back(x_ - 1, y_ - 1);
    result.emplace_back(x_ + 1, y_ - 1);
    return result;
  }

  static std::vector<ID2D> DisntanceInRadius(const float dist) {
    std::vector<ID2D> result;
    if (dist < 1) {
      return {ID2D{0, 0}};
    }

    const float dist2 = dist * dist;
    const int d = static_cast<int>(std::floor(dist));

    for (int i = -d; i <= d; ++i) {
      for (int j = -d; j <= d; ++j) {
        if (float(i) * float(i) + float(j) * float(j) < dist2) {
          result.emplace_back(i, j);
        }
      }
    }

    return result;
  }

  int x() const { return x_; }
  int y() const { return y_; }

 private:
  int x_ = 0;
  int y_ = 0;
};

struct ID3D {
 public:
  ID3D(const int x = 0, const int y = 0, const int z = 0)
      : x_(x), y_(y), z_(z) {}

  bool operator==(const ID3D& other) const {
    return x_ == other.x() && y_ == other.y() && z_ == other.z();
  }

  ID3D operator+(const ID3D& other) const {
    return ID3D{x_ + other.x(), y_ + other.y(), z_ + other.z()};
  }

  ID3D operator-(const ID3D& other) const {
    return ID3D{x_ - other.x(), y_ - other.y(), z_ - other.z()};
  }

  std::vector<ID3D> Neighbors() const {
    std::vector<ID3D> result;
    result.reserve(6);
    result.emplace_back(x_ + 1, y_, z_);
    result.emplace_back(x_ - 1, y_, z_);
    result.emplace_back(x_, y_ + 1, z_);
    result.emplace_back(x_, y_ - 1, z_);
    result.emplace_back(x_, y_, z_ + 1);
    result.emplace_back(x_, y_, z_ - 1);
    return result;
  }

  std::vector<ID3D> NeighborsInManhattanDist(const int dist) const {
    std::vector<ID3D> result;
    for (int i = -dist; i <= dist; ++i) {
      for (int j = -dist; j <= dist; ++j) {
        for (int k = -dist; k <= dist; ++k) {
          if (std::abs(i) + std::abs(j) + std::abs(k) <= dist) {
            result.emplace_back(x_ + i, y_ + j, z_ + k);
          }
        }
      }
    }
    return result;
  }

  std::vector<ID3D> NeighborsAtSpecificManhattanDist(const int dist) const {
    std::vector<ID3D> result;
    for (int i = -dist; i <= dist; ++i) {
      for (int j = -dist; j <= dist; ++j) {
        for (int k = -dist; k <= dist; ++k) {
          if (std::abs(i) + std::abs(j) + std::abs(k) == dist) {
            result.emplace_back(x_ + i, y_ + j, z_ + k);
          }
        }
      }
    }
    return result;
  }

  std::vector<ID3D> NeighborsInBoxRadius(const int radius) const {
    std::vector<ID3D> result;
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        for (int k = -radius; k <= radius; ++k) {
            result.emplace_back(x_ + i, y_ + j, z_ + k);
        }
      }
    }
    return result;
  }

  std::vector<ID3D> NeighborsInBoxRadius(const int rx,
                                         const int ry,
                                         const int rz) const {
    std::vector<ID3D> result;
    for (int i = -rx; i <= rx; ++i) {
      for (int j = -ry; j <= ry; ++j) {
        for (int k = -rz; k <= rz; ++k) {
          result.emplace_back(x_ + i, y_ + j, z_ + k);
        }
      }
    }
    return result;
  }

  static std::vector<ID3D> DisntanceInRadius(const float dist) {
    std::vector<ID3D> result;
    if (dist < 1) {
      return {ID3D{0, 0, 0}};
    }

    const float dist2 = dist * dist;
    const int d = static_cast<int>(std::floor(dist));

    for (int i = -d; i <= d; ++i) {
      for (int j = -d; j <= d; ++j) {
        for (int k = -d; k <= d; ++k) {
          if (float(i) * float(i) + float(j) * float(j) + float(k) * float(k) <
              dist2) {
            result.emplace_back(i, j, k);
          }
        }
      }
    }

    return result;
  }

  int x() const { return x_; }
  int y() const { return y_; }
  int z() const { return z_; }

 private:
  int x_, y_, z_;
};


namespace std {
template <>
struct hash<ID2D> {
  std::size_t operator()(const ID2D& h) const {
    std::size_t seed = 0;
    hash_combine(seed, h.x(), h.y());
    return seed;
  }
};

template <>
struct hash<ID3D> {
  std::size_t operator()(const ID3D& h) const {
    std::size_t seed = 0;
    hash_combine(seed, h.x(), h.y(), h.z());
    return seed;
  }
};

}  // namespace std
