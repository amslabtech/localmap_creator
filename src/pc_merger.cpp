/**
 * @file pc_merger.cpp
 * @author amsl
 * @brief PointCloud merger class
 * @copyright Copyright (c) 2024
 */

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

class PcMerger
{
public:
  PcMerger(void);
  void process(void);

protected:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

  int hz_;
  int cloud_num_;
  int cloud_count_;
  int cloud_count_th_;
  std::string target_frame_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::vector<ros::Subscriber> point_cloud_subs_;
  ros::Publisher point_cloud_pub_;
  PointCloudT cloud_merged_;
  tf::TransformListener tf_listener_;
};

PcMerger::PcMerger(void) : private_nh_("~"), cloud_count_(0)
{
  private_nh_.param<std::string>("target_frame", target_frame_, {"base_link"});
  private_nh_.param<int>("hz", hz_, 10);
  private_nh_.param<int>("cloud_num", cloud_num_, 2);
  private_nh_.param<int>("cloud_count_th", cloud_count_th_, 2);

  ROS_INFO("=== PC Merger ===");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("target_frame: " << target_frame_);
  ROS_INFO_STREAM("cloud_num: " << cloud_num_);
  ROS_INFO_STREAM("cloud_count_th: " << cloud_count_th_);

  point_cloud_subs_.resize(cloud_num_);
  for (int i = 0; i < cloud_num_; i++)
  {
    point_cloud_subs_[i] = nh_.subscribe(
        "/cloud" + std::to_string(i), 1, &PcMerger::cloud_callback, this,
        ros::TransportHints().reliable().tcpNoDelay());
  }
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/merged", 1);
}

void PcMerger::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);
  pcl_ros::transformPointCloud(target_frame_, *cloud, *cloud, tf_listener_);
  cloud_merged_.points.insert(cloud_merged_.points.end(), cloud->begin(), cloud->end());
  cloud_count_++;
}

void PcMerger::process(void)
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (cloud_count_th_ <= cloud_count_)
    {
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud_merged_, cloud_msg);
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = target_frame_;
      point_cloud_pub_.publish(cloud_msg);
    }
    cloud_merged_.points.clear();
    loop_rate.sleep();
    cloud_count_ = 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_merger");
  PcMerger pc_merger;
  pc_merger.process();

  return 0;
}
