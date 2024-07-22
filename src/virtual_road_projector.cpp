#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <optional>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include "amsl_navigation_msgs/Road.h"

struct Param
{
  std::string global_frame_id;
  int allowable_num_of_not_received;
  float robot_radius;
  float wall_thickness;
};

class VirtualRoadProjector
{
public:
  VirtualRoadProjector(void);

private:
  void map_callback(const nav_msgs::OccupancyGridConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void road_info_callback(const amsl_navigation_msgs::RoadConstPtr &msg);
  nav_msgs::OccupancyGrid project_road(const nav_msgs::OccupancyGrid &map, amsl_navigation_msgs::Road road);
  bool inside_road(const amsl_navigation_msgs::Road &road, const geometry_msgs::Point &point);
  geometry_msgs::Point index_to_point(const nav_msgs::OccupancyGrid &map, const int index);
  bool is_edge_of_road(const geometry_msgs::Point &point, const amsl_navigation_msgs::Road &road);
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

VirtualRoadProjector::VirtualRoadProjector(void) : private_nh_("~"), tf_listener_(tf_buffer_)
{
  private_nh_.param<std::string>("global_frame_id", param_.global_frame_id, std::string("map"));
  private_nh_.param<int>("allowable_num_of_not_received", param_.allowable_num_of_not_received, 5);
  private_nh_.param<float>("robot_radius", param_.robot_radius, 0.3);
  private_nh_.param<float>("wall_thickness", param_.wall_thickness, 0.05);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map/projected", 1);
  map_sub_ = nh_.subscribe(
      "/local_map", 1, &VirtualRoadProjector::map_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  pose_sub_ = nh_.subscribe(
      "/localized_pose", 1, &VirtualRoadProjector::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  road_sub_ = nh_.subscribe(
      "/node_edge_map/road", 1, &VirtualRoadProjector::road_info_callback, this,
      ros::TransportHints().reliable().tcpNoDelay());
}

void VirtualRoadProjector::map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (!road_updated_)
    count_of_not_received_road_++;
  if (param_.allowable_num_of_not_received < count_of_not_received_road_)
    road_.reset();

  if (!road_.has_value())
    map_pub_.publish(*msg);
  else
    map_pub_.publish(project_road(*msg, road_.value()));

  road_updated_ = false;
}

void VirtualRoadProjector::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) { pose_ = *msg; }

void VirtualRoadProjector::road_info_callback(const amsl_navigation_msgs::RoadConstPtr &msg)
{
  road_ = *msg;
  road_updated_ = true;
  count_of_not_received_road_ = 0;
}

nav_msgs::OccupancyGrid
VirtualRoadProjector::project_road(const nav_msgs::OccupancyGrid &map, amsl_navigation_msgs::Road road)
{
  nav_msgs::OccupancyGrid projected_map = map;

  // transform road points to global frame
  geometry_msgs::TransformStamped transform_stamped;
  while (ros::ok())
  {
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(map.header.frame_id, param_.global_frame_id, ros::Time(0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.5).sleep();
    }
  }
  tf2::doTransform(road.point0, road.point0, transform_stamped);
  tf2::doTransform(road.point1, road.point1, transform_stamped);

  if (pose_.has_value())
    if (!inside_road(road, pose_.value().pose.pose.position))
      return map;

  // project road to map
  for (int i = 0; i < projected_map.data.size(); i++)
  {
    const geometry_msgs::Point point = index_to_point(projected_map, i);
    if (is_edge_of_road(point, road))
    {
      if (hypot(point.x, point.y) <= param_.robot_radius)
        return map;
      projected_map.data[i] = 100;
    }
  }

  return projected_map;
}

bool VirtualRoadProjector::inside_road(const amsl_navigation_msgs::Road &road, const geometry_msgs::Point &point)
{
  const Eigen::Vector3d reference_vector(road.point1.x - road.point0.x, road.point1.y - road.point0.y, 0.0);
  const Eigen::Vector3d target_vector(point.x - road.point0.x, point.y - road.point0.y, 0.0);
  const float dist_to_path = calc_dist_to_path(road.point0, road.point1, point);
  if (reference_vector.cross(target_vector).z() < 0.0)
    return dist_to_path < road.distance_to_right;
  else
    return dist_to_path < road.width - road.distance_to_right;
}

geometry_msgs::Point VirtualRoadProjector::index_to_point(const nav_msgs::OccupancyGrid &map, const int index)
{
  geometry_msgs::Point point;
  const int grid_index_x = index % map.info.width;
  const int grid_index_y = static_cast<int>(index / map.info.width);
  point.x = grid_index_x * map.info.resolution + map.info.origin.position.x + map.info.resolution / 2.0;
  point.y = grid_index_y * map.info.resolution + map.info.origin.position.y + map.info.resolution / 2.0;

  return point;
}

bool VirtualRoadProjector::is_edge_of_road(const geometry_msgs::Point &point, const amsl_navigation_msgs::Road &road)
{
  const Eigen::Vector3d reference_vector(road.point1.x - road.point0.x, road.point1.y - road.point0.y, 0.0);
  const Eigen::Vector3d target_vector(point.x - road.point0.x, point.y - road.point0.y, 0.0);
  const float dist_to_path = calc_dist_to_path(road.point0, road.point1, point);
  if (reference_vector.cross(target_vector).z() < 0.0)
  {
    return road.distance_to_right <= dist_to_path && dist_to_path < road.distance_to_right + param_.wall_thickness;
  }
  else
  {
    const float distance_to_left = road.width - road.distance_to_right;
    return distance_to_left <= dist_to_path && dist_to_path < distance_to_left + param_.wall_thickness;
  }
}

float VirtualRoadProjector::calc_dist_to_path(
    const geometry_msgs::Point &edge_point0, const geometry_msgs::Point &edge_point1,
    const geometry_msgs::Point &target_point)
{
  const float a = edge_point1.y - edge_point0.y;
  const float b = -(edge_point1.x - edge_point0.x);
  const float c = -a * edge_point0.x - b * edge_point0.y;

  return fabs(a * target_point.x + b * target_point.y + c) / (hypot(a, b) + DBL_EPSILON);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "road_projector");
  VirtualRoadProjector road_projector;
  ros::spin();

  return 0;
}
