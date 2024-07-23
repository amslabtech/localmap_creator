/**
 * @file virtual_road_projector.h
 * @author amsl
 * @brief Virtual road projector class
 * @copyright Copyright (c) 2024
 */

#ifndef LOCALMAP_CREATOR_VIRTUAL_ROAD_PROJECTOR_H
#define LOCALMAP_CREATOR_VIRTUAL_ROAD_PROJECTOR_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <optional>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/Road.h"

/**
 * @struct Param
 * @brief Parameters for VirtualRoadProjector
 */
struct Param
{
  std::string global_frame_id;
  int allowable_num_of_not_received;
  float robot_radius;
  float wall_thickness;
};

/**
 * @class VirtualRoadProjector
 * @brief Virtual road projector class
 */
class VirtualRoadProjector
{
public:
  /**
   * @brief Construct a new Virtual Road Projector object
   */
  VirtualRoadProjector(void);

private:
  /**
   * @brief Callback function for map
   * @param msg OccupancyGrid message
   */
  void map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

  /**
   * @brief Callback function for pose
   * @param msg PoseWithCovarianceStamped message
   */
  void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) { pose_ = *msg; }

  /**
   * @brief Callback function for road information
   * @param msg Road message
   */
  void road_info_callback(const amsl_navigation_msgs::RoadConstPtr &msg);

  /**
   * @brief Project road to map
   * @param map OccupancyGrid message
   * @param road Road message
   * @return nav_msgs::OccupancyGrid Projected map
   */
  nav_msgs::OccupancyGrid project_road(const nav_msgs::OccupancyGrid &map, amsl_navigation_msgs::Road road);

  /**
   * @brief Check if the point is inside the road
   * @param road Road message
   * @param point Point message
   * @return true if the point is inside the road
   * @return false if the point is outside the road
   */
  bool inside_road(const amsl_navigation_msgs::Road &road, const geometry_msgs::Point &point);

  /**
   * @brief Convert index to point
   * @param map OccupancyGrid message
   * @param index Index
   * @return geometry_msgs::Point Point message
   */
  geometry_msgs::Point index_to_point(const nav_msgs::OccupancyGrid &map, const int index);

  /**
   * @brief Check if the point is edge of the road
   * @param point Point message
   * @param road Road message
   * @return true if the point is edge of the road
   * @return false if the point is not edge of the road
   */
  bool is_edge_of_road(const geometry_msgs::Point &point, const amsl_navigation_msgs::Road &road);

  /**
   * @brief Calculate distance to the path
   * @param edge_point0 Edge point 0
   * @param edge_point1 Edge point 1
   * @param target_point Target point
   * @return float Distance to the path
   */
  float calc_dist_to_path(
      const geometry_msgs::Point &edge_point0, const geometry_msgs::Point &edge_point1,
      const geometry_msgs::Point &target_point);

  Param param_;
  bool road_updated_ = false;
  int count_of_not_received_road_ = 0;
  std::optional<amsl_navigation_msgs::Road> road_;
  std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher map_pub_;
  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber road_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // LOCALMAP_CREATOR_VIRTUAL_ROAD_PROJECTOR_H
