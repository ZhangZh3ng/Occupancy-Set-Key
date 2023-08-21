#pragma once

#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

// Function to compute the transformation matrix given two sets of 3D points
Eigen::Matrix4f ComputeRelativeTransformOfPoints(
    const std::vector<Eigen::Vector3f>& points_source,
    const std::vector<Eigen::Vector3f>& points_target) {
  // Assuming points_source and points_target are of the same size
  int num_points = points_source.size();

  // Construct matrices to store the coordinates of points_source and
  // points_target
  Eigen::MatrixXf mat_source(3, num_points);
  Eigen::MatrixXf mat_target(3, num_points);
  for (int i = 0; i < num_points; ++i) {
    mat_source.col(i) = points_source[i];
    mat_target.col(i) = points_target[i];
  }

  // Compute the centroids of points_source and points_target
  Eigen::Vector3f centroid_source = mat_source.rowwise().mean();
  Eigen::Vector3f centriod_target = mat_target.rowwise().mean();

  // Subtract the centroids to make the points zero-mean
  mat_source.colwise() -= centroid_source;
  mat_target.colwise() -= centriod_target;

  // Compute the covariance matrix
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (int i = 0; i < num_points; ++i) {
    covariance += mat_source.col(i) * mat_target.col(i).transpose();
  }

  // Perform singular value decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Matrix3f V = svd.matrixV();

  Eigen::Matrix3f R = V * U.transpose();
  Eigen::Vector3f t = centriod_target - R * centroid_source;

  // Build the transformation matrix
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = t;
  return transform;
}

// Function to implement RANSAC algorithm for finding the best transformation
Eigen::Matrix4f RANSAC(const std::vector<Eigen::Vector3f>& cloud_source,
                       const std::vector<Eigen::Vector3f>& cloud_traget,
                       int num_iterations,
                       double inlier_threshold,
                       int& num_best_match) {
  int num_correspondences = cloud_source.size();

  // Minimum number of correspondences needed to compute the transform
  const int min_correspondences = 3;

  Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
  int max_inliers = 0;

  // Random number generator
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> distribution(0, num_correspondences - 1);

  for (int iteration = 0; iteration < num_iterations; ++iteration) {
    // Randomly select min_correspondences correspondences
    std::vector<Eigen::Vector3f> sampled_cloud1;
    std::vector<Eigen::Vector3f> sampled_cloud2;

    for (int i = 0; i < min_correspondences; ++i) {
      int random_idx = distribution(rng);
      sampled_cloud1.push_back(cloud_source[random_idx]);
      sampled_cloud2.push_back(cloud_traget[random_idx]);
    }

    // Compute the transformation using the sampled correspondences
    Eigen::Matrix4f transform =
        ComputeRelativeTransformOfPoints(sampled_cloud1, sampled_cloud2);

    // Count inliers using the computed transformation
    int inliers = 0;
    for (int i = 0; i < num_correspondences; ++i) {
      Eigen::Vector4f point1;
      point1 << cloud_source[i], 1.0f;
      Eigen::Vector4f transformed_point = transform * point1;

      double distance = (transformed_point.head<3>() - cloud_traget[i]).norm();
      if (distance < inlier_threshold) {
        inliers++;
      }
    }

    // Update the best transformation if this iteration has more inliers
    if (inliers > max_inliers) {
      max_inliers = inliers;
      best_transform = transform;
    }
  }

  num_best_match = max_inliers;
  return best_transform;
}

// Function to compute the transformation matrix given two sets of 2D points
Eigen::Matrix3f ComputeRelativeTransformOfPoints(
    const std::vector<Eigen::Vector2f>& points_source,
    const std::vector<Eigen::Vector2f>& points_target) {
  // Assuming points_source and points_target are of the same size
  int num_points = points_source.size();

  // Construct matrices to store the coordinates of points_source and
  // points_target
  Eigen::MatrixXf mat_source(2, num_points);
  Eigen::MatrixXf mat_target(2, num_points);
  for (int i = 0; i < num_points; ++i) {
    mat_source.col(i) = points_source[i];
    mat_target.col(i) = points_target[i];
  }

  // Compute the centroids of points_source and points_target
  Eigen::Vector2f centroid_source = mat_source.rowwise().mean();
  Eigen::Vector2f centroid_target = mat_target.rowwise().mean();

  // Subtract the centroids to make the points zero-mean
  mat_source.colwise() -= centroid_source;
  mat_target.colwise() -= centroid_target;

  // Compute the covariance matrix
  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (int i = 0; i < num_points; ++i) {
    covariance += mat_source.col(i) * mat_target.col(i).transpose();
  }

  // Perform singular value decomposition
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2f U = svd.matrixU();
  Eigen::Matrix2f V = svd.matrixV();

  Eigen::Matrix2f R = V * U.transpose();
  Eigen::Vector2f t = centroid_target - R * centroid_source;

  // Build the transformation matrix
  Eigen::Matrix3f transform = Eigen::Matrix3f::Identity();
  transform.block<2, 2>(0, 0) = R;
  transform.block<2, 1>(0, 2) = t;
  return transform;
}

// Function to implement RANSAC algorithm for finding the best transformation
Eigen::Matrix3f RANSAC(const std::vector<Eigen::Vector2f>& cloud_source,
                       const std::vector<Eigen::Vector2f>& cloud_target,
                       int num_iterations,
                       double inlier_threshold,
                       int& num_best_match) {
  int num_correspondences = cloud_source.size();

  // Minimum number of correspondences needed to compute the transform
  const int min_correspondences = 3;

  Eigen::Matrix3f best_transform = Eigen::Matrix3f::Identity();
  int max_inliers = 0;

  // Random number generator
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> distribution(0, num_correspondences - 1);

  for (int iteration = 0; iteration < num_iterations; ++iteration) {
    // Randomly select min_correspondences correspondences
    std::vector<Eigen::Vector2f> sampled_cloud1;
    std::vector<Eigen::Vector2f> sampled_cloud2;

    for (int i = 0; i < min_correspondences; ++i) {
      int random_idx = distribution(rng);
      sampled_cloud1.push_back(cloud_source[random_idx]);
      sampled_cloud2.push_back(cloud_target[random_idx]);
    }

    // Compute the transformation using the sampled correspondences
    Eigen::Matrix3f transform =
        ComputeRelativeTransformOfPoints(sampled_cloud1, sampled_cloud2);

    // Count inliers using the computed transformation
    int inliers = 0;
    for (int i = 0; i < num_correspondences; ++i) {
      Eigen::Vector3f point1;
      point1 << cloud_source[i], 1.0f;
      Eigen::Vector3f transformed_point = transform * point1;

      double distance = (transformed_point.head<2>() - cloud_target[i]).norm();
      if (distance < inlier_threshold) {
        inliers++;
      }
    }

    // Update the best transformation if this iteration has more inliers
    if (inliers > max_inliers) {
      max_inliers = inliers;
      best_transform = transform;
    }
  }

  num_best_match = max_inliers;
  return best_transform;
}

Eigen::Matrix4f Convert2DTransformationTo3D(
    const Eigen::Matrix3f& transform2D) {
  Eigen::Matrix4f transform3D = Eigen::Matrix4f::Identity();

  // Copy the 2D transformation part
  transform3D.block<2, 2>(0, 0) = transform2D.block<2, 2>(0, 0);

  // Copy the translation part
  transform3D.block<2, 1>(0, 3) = transform2D.block<2, 1>(0, 2);

  return transform3D;
}
