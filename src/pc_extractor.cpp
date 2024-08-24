#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

class PcExtractor
{
public:
  PcExtractor(void);

protected:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void passthrough_filter(PointCloudT::Ptr cloud, const std::string &axis, const float min, const float max);

  std::string target_frame_;
  float max_height_;
  float min_height_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  tf::TransformListener tf_listener_;
};

PcExtractor::PcExtractor(void) : private_nh_("~")
{
  private_nh_.param<std::string>("target_frame", target_frame_, {"base_link"});
  private_nh_.param<float>("max_height", max_height_, 1.5);
  private_nh_.param<float>("min_height", min_height_, 0.05);

  ROS_INFO("=== PC Extractor ===");
  ROS_INFO_STREAM("target_frame: " << target_frame_);
  ROS_INFO_STREAM("max_height: " << max_height_);
  ROS_INFO_STREAM("min_height: " << min_height_);

  point_cloud_sub_ =
      nh_.subscribe("/cloud_in", 1, &PcExtractor::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
}

void PcExtractor::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);

  // transform
  pcl_ros::transformPointCloud(target_frame_, *cloud, *cloud, tf_listener_);

  // filter
  passthrough_filter(cloud, "z", min_height_, max_height_);

  // publish
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = msg->header.stamp;
  cloud_msg.header.frame_id = target_frame_;
  point_cloud_pub_.publish(cloud_msg);
}

void PcExtractor::passthrough_filter(PointCloudT::Ptr cloud, const std::string &axis, const float min, const float max)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.filter(*cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_extractor");
  PcExtractor pc_extractor;
  ros::spin();

  return 0;
}
