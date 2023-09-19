#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>

template <typename PointType>
pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertToPclPointCloud(
    const std::vector<PointType>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_out->reserve(points.size());

  for (const auto& point : points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    cloud_out->push_back(pcl_point);
  }

  return cloud_out;
}

template <typename T>
double GetRosTime(const T& msg) {
  return msg.header.stamp.toSec();
}

template <typename T>
void PublishCloud(const pcl::PointCloud<T>& cloud,
                  ros::Publisher& publisher,
                  const std_msgs::Header& header) {
  sensor_msgs::PointCloud2 cloud_pub;
  pcl::toROSMsg(cloud, cloud_pub);
  cloud_pub.header = header;
  publisher.publish(cloud_pub);
}

void PublishTF(tf::TransformBroadcaster& broadcaster,
               const Eigen::Matrix4f& transform,
               const std::string& parent_frame,
               const std::string& child_frame) {
  Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);

  // Convert Eigen translation and rotation to ROS tf data types
  tf::Vector3 tf_translation(translation(0), translation(1), translation(2));
  Eigen::Quaternionf quat(rotation);
  tf::Quaternion tf_quaternion(quat.x(), quat.y(), quat.z(), quat.w());

  // Create the tf transform object
  tf::Transform tf_transform;
  tf_transform.setOrigin(tf_translation);
  tf_transform.setRotation(tf_quaternion);

  // Publish the tf message
  broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(),
                                                 parent_frame, child_frame));
}

Eigen::Matrix4f GetInverseTransformMatrix(
    const Eigen::Matrix4f& transformMatrix) {
  Eigen::Matrix3f rotationMatrix = transformMatrix.block<3, 3>(0, 0);
  Eigen::Vector3f translation = transformMatrix.block<3, 1>(0, 3);

  Eigen::Matrix3f inverseRotationMatrix = rotationMatrix.transpose();
  Eigen::Vector3f inverseTranslation = -inverseRotationMatrix * translation;

  Eigen::Matrix4f inverseTransformMatrix = Eigen::Matrix4f::Identity();
  inverseTransformMatrix.block<3, 3>(0, 0) = inverseRotationMatrix;
  inverseTransformMatrix.block<3, 1>(0, 3) = inverseTranslation;

  return inverseTransformMatrix;
}

std::vector<Eigen::Vector3f> TransformPoints(
    const std::vector<Eigen::Vector3f>& points,
    const Eigen::Matrix4f& transformMatrix) {
  std::vector<Eigen::Vector3f> transformedPoints;
  for (const auto& point : points) {
    // Convert point to homogeneous coordinates (4x1 vector)
    Eigen::Vector4f homogeneousPoint;
    homogeneousPoint << point, 1.0f;

    // Apply the transformation matrix to the point
    Eigen::Vector4f transformedHomogeneousPoint =
        transformMatrix * homogeneousPoint;

    // Convert back to Cartesian coordinates (3x1 vector)
    Eigen::Vector3f transformedPoint = transformedHomogeneousPoint.head<3>();

    transformedPoints.push_back(transformedPoint);
  }

  return transformedPoints;
}

template <typename ResultType>
void WriteSimiarityResult(const std::string of_path,
                          const ResultType& results) {
  std::ofstream file(of_path);
  for (const auto& entry : results) {
    const auto& scan_id = entry.first;
    const auto& score = entry.second;

    file << scan_id << " " << score << std::endl;
  }

  file.close();
}

visualization_msgs::Marker GenerateLoopMarker(
    const Eigen::Matrix4f& current_pose,
    const Eigen::Matrix4f& history_pose,
    const std::string& frame_id,
    const int id,
    const Eigen::Vector4f rgba = Eigen::Vector4f{0, 1, 0, 1}) {
  visualization_msgs::Marker loop_marker;
  loop_marker.header.frame_id =
      frame_id;  // Set the frame in which the marker will be displayed
  loop_marker.header.stamp = ros::Time::now();
  loop_marker.ns = "loop_closures";
  loop_marker.id = id;  // Unique ID for the marker
  loop_marker.type = visualization_msgs::Marker::LINE_LIST;
  loop_marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the line (e.g., line width)
  loop_marker.scale.x = 0.3;  // Line width

  // Set the color (RGBA) of the marker (green in this example)
  loop_marker.color.r = rgba[0];
  loop_marker.color.g = rgba[1];
  loop_marker.color.b = rgba[2];
  loop_marker.color.a = rgba[3];

  // Set the two endpoints of the line representing the loop closure
  loop_marker.points.resize(2);
  loop_marker.points[0].x = current_pose(0, 3);
  loop_marker.points[0].y = current_pose(1, 3);
  loop_marker.points[0].z = current_pose(2, 3);
  loop_marker.points[1].x = history_pose(0, 3);
  loop_marker.points[1].y = history_pose(1, 3);
  loop_marker.points[1].z = history_pose(2, 3);
  loop_marker.pose.orientation.w = 1;
  loop_marker.pose.orientation.x = 0;
  loop_marker.pose.orientation.y = 0;
  loop_marker.pose.orientation.z = 0;

  return loop_marker;
}

template <typename MatrixType>
MatrixType MatrixColumnShift(const MatrixType& matrix, int shift_amount) {
  MatrixType shifted_matrix(matrix.rows(), matrix.cols());
  if (shift_amount >= matrix.cols()) {
    return shifted_matrix;
  }

  // Perform the column shift
  shifted_matrix << matrix.block(0, shift_amount, matrix.rows(),
                                 matrix.cols() - shift_amount),
      matrix.block(0, 0, matrix.rows(), shift_amount);

  return shifted_matrix;
}

// ------------------ visualization tools --------------------------
void GeneratePointCorrelationMarkers(
    const std::vector<Eigen::Vector3f>& points1,
    const std::vector<Eigen::Vector3f>& points2,
    visualization_msgs::MarkerArray& markers,
    const std::string frame_id,
    const Eigen::Vector4f rgba = Eigen::Vector4f{0, 1, 0, 1}) {
  markers.markers.clear();
  // Set up markerArray for points and lines
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = frame_id;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::POINTS;
  point_marker.scale.x = 0.5;
  point_marker.scale.y = 0.5;
  point_marker.color.r = 1.0;
  point_marker.color.a = 1.0;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "lines";
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.6;
  line_marker.color.r = rgba[0];
  line_marker.color.g = rgba[1];
  line_marker.color.b = rgba[2];
  line_marker.color.a = rgba[3];

  // Add points and lines to markerArray
  for (int i = 0; i < points1.size(); i++) {
    // Add point from point set 1
    geometry_msgs::Point point1;
    point1.x = points1[i](0);
    point1.y = points1[i](1);
    point1.z = points1[i](2);
    point_marker.points.push_back(point1);

    // Add point from point set 2
    geometry_msgs::Point point2;
    point2.x = points2[i](0);
    point2.y = points2[i](1);
    point2.z = points2[i](2);
    point_marker.points.push_back(point2);

    // Add line between the two points
    line_marker.points.push_back(point1);
    line_marker.points.push_back(point2);
  }

  // Add markers to markerArray
  markers.markers.push_back(point_marker);
  markers.markers.push_back(line_marker);
}

void GeneratePointCorrelationMarkers(
    const std::vector<Eigen::Vector3f>& points1,
    const std::vector<Eigen::Vector3f>& points2,
    const std::vector<int>& is_true_positive,
    visualization_msgs::MarkerArray& markers,
    const std::string frame_id) {
  markers.markers.clear();
  // Set up markerArray for points and lines
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = frame_id;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::POINTS;
  point_marker.scale.x = 0.05;
  point_marker.scale.y = 0.05;
  point_marker.color.r = 1.0;
  point_marker.color.a = 1.0;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "lines_true_positive";
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.05;
  line_marker.color.g = 1.0;
  line_marker.color.a = 1.0;

  visualization_msgs::Marker line_marker_fp;
  line_marker_fp.header.frame_id = frame_id;
  line_marker_fp.header.stamp = ros::Time::now();
  line_marker_fp.ns = "lines_false_positive";
  line_marker_fp.action = visualization_msgs::Marker::ADD;
  line_marker_fp.pose.orientation.w = 1.0;
  line_marker_fp.id = 1;
  line_marker_fp.type = visualization_msgs::Marker::LINE_LIST;
  line_marker_fp.scale.x = 0.05;
  line_marker_fp.color.r = 1.0;
  line_marker_fp.color.a = 1.0;

  // Add points and lines to markerArray
  for (int i = 0; i < points1.size(); i++) {
    // Add point from point set 1
    geometry_msgs::Point point1;
    point1.x = points1[i](0);
    point1.y = points1[i](1);
    point1.z = points1[i](2);
    point_marker.points.push_back(point1);

    // Add point from point set 2
    geometry_msgs::Point point2;
    point2.x = points2[i](0);
    point2.y = points2[i](1);
    point2.z = points2[i](2);
    point_marker.points.push_back(point2);

    // Add line between the two points
    if (is_true_positive[i] == 1) {
      line_marker.points.push_back(point1);
      line_marker.points.push_back(point2);
    } else {
      line_marker_fp.points.push_back(point1);
      line_marker_fp.points.push_back(point2);
    }
  }

  // Add markers to markerArray
  markers.markers.push_back(point_marker);
  markers.markers.push_back(line_marker);
  markers.markers.push_back(line_marker_fp);
}

void AddPointCorrelationMarkers(const std::vector<Eigen::Vector3f>& points1,
                                const std::vector<Eigen::Vector3f>& points2,
                                visualization_msgs::MarkerArray& markers,
                                const std::string frame_id) {
  // Set up markerArray for points and lines
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = frame_id;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::POINTS;
  point_marker.scale.x = 0.05;
  point_marker.scale.y = 0.05;
  point_marker.color.r = 1.0;
  point_marker.color.a = 1.0;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "lines";
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.5;
  line_marker.color.g = 1.0;
  line_marker.color.a = 1.0;

  // Add points and lines to markerArray
  for (int i = 0; i < points1.size(); i++) {
    // Add point from point set 1
    geometry_msgs::Point point1;
    point1.x = points1[i](0);
    point1.y = points1[i](1);
    point1.z = points1[i](2);
    point_marker.points.push_back(point1);

    // Add point from point set 2
    geometry_msgs::Point point2;
    point2.x = points2[i](0);
    point2.y = points2[i](1);
    point2.z = points2[i](2);
    point_marker.points.push_back(point2);

    // Add line between the two points
    line_marker.points.push_back(point1);
    line_marker.points.push_back(point2);
  }

  // Add markers to markerArray
  markers.markers.push_back(point_marker);
  markers.markers.push_back(line_marker);
}
