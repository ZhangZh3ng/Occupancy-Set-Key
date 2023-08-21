#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <vector>

void GeneratePointCorrelationMarkers(
    const std::vector<Eigen::Vector3f>& points1,
    const std::vector<Eigen::Vector3f>& points2,
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
  line_marker.ns = "lines";
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.scale.x = 0.3;
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

void AddPointCorrelationMarkers(
    const std::vector<Eigen::Vector3f>& points1,
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
