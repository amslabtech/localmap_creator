#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

struct Params
{
  std::string target_frame;
  bool remove_underground;
  float ramdom_sample_percent;
  bool use_dummy_ground_points;
  int dummy_ground_points_num;
  float dummy_ground_points_size;
  int outlier_removal_num_neighbors;
  float outlier_removal_std_dev_mul_thresh;
};

class OutlierRemovalFilter
{
public:
  OutlierRemovalFilter(void);

protected:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  void load_params(void);
  void print_params(void);
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void passthrough_filter(PointCloudT::Ptr cloud, const std::string &axis, const float min, const float max);
  void random_sample_filter(PointCloudT::Ptr cloud, const int size);
  void add_dummy_ground_points(PointCloudT::Ptr cloud, const int point_num, const float size);
  void outlier_removal_filter(PointCloudT::Ptr cloud, const int num_neighbors, const float std_dev_mul_thresh);

  Params params_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  tf::TransformListener tfListener_;
};

OutlierRemovalFilter::OutlierRemovalFilter(void) : private_nh_("~")
{
  load_params();
  print_params();

  point_cloud_sub_ = nh_.subscribe(
      "/cloud", 1, &OutlierRemovalFilter::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/filtered", 1);
}

void OutlierRemovalFilter::load_params(void)
{
  private_nh_.param<std::string>("target_frame", params_.target_frame, {"base_link"});
  private_nh_.param<bool>("remove_underground", params_.remove_underground, {false});
  private_nh_.param<float>("ramdom_sample_percent", params_.ramdom_sample_percent, {0.1});
  private_nh_.param<bool>("dummy_ground_points/enable", params_.use_dummy_ground_points, {false});
  private_nh_.param<int>("dummy_ground_points/num", params_.dummy_ground_points_num, {100});
  private_nh_.param<float>("dummy_ground_points/size", params_.dummy_ground_points_size, {2.0});
  private_nh_.param<int>("outlier_removal/num_neighbors", params_.outlier_removal_num_neighbors, {50});
  private_nh_.param<float>("outlier_removal/std_dev_mul_thresh", params_.outlier_removal_std_dev_mul_thresh, {1.0});
}

void OutlierRemovalFilter::print_params(void)
{
  ROS_INFO("=== Realsense Outlier Removal Filter ===");
  ROS_INFO_STREAM("target_frame: " << params_.target_frame);
  ROS_INFO_STREAM("remove_underground: " << params_.remove_underground);
  ROS_INFO_STREAM("ramdom_sample_percent: " << params_.ramdom_sample_percent);
  ROS_INFO_STREAM("dummy_ground_points/enable: " << params_.use_dummy_ground_points);
  ROS_INFO_STREAM("dummy_ground_points/num: " << params_.dummy_ground_points_num);
  ROS_INFO_STREAM("dummy_ground_points/size: " << params_.dummy_ground_points_size);
  ROS_INFO_STREAM("outlier_removal/num_neighbors: " << params_.outlier_removal_num_neighbors);
  ROS_INFO_STREAM("outlier_removal/std_dev_mul_thresh: " << params_.outlier_removal_std_dev_mul_thresh);
}

void OutlierRemovalFilter::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert sensor_msgs::PointCloud2 to pcl::PointCloud
  PointCloudT::Ptr pc_ptr(new PointCloudT);
  pcl::fromROSMsg(*msg, *pc_ptr);

  // transform
  PointCloudT::Ptr pc_transformed_ptr(new PointCloudT);
  pcl_ros::transformPointCloud(target_frame_, *pc_ptr, *pc_transformed_ptr, tfListener_);

  // preprocess
  if (params_.remove_underground)
    passthrough_filter(pc_transformed_ptr, "z", -0.05, 3.0);
  random_sample_filter(
      pc_transformed_ptr, static_cast<int>(pc_transformed_ptr->size() * params_.ramdom_sample_percent / 100));

  // add dummy data
  if (params_.use_dummy_ground_points)
    add_dummy_ground_points(pc_transformed_ptr, params_.dummy_ground_points_num, params_.dummy_ground_points_size);

  // outlier removal
  outlier_removal_filter(
      pc_transformed_ptr, params_.outlier_removal_num_neighbors, params_.outlier_removal_std_dev_mul_thresh);

  // convert pcl::PointCloud to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*pc_transformed_ptr, cloud_msg);

  // publish
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = target_frame_;
  point_cloud_pub_.publish(cloud_msg);
}

void OutlierRemovalFilter::passthrough_filter(
    PointCloudT::Ptr cloud, const std::string &axis, const float min, const float max)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.filter(*cloud);
}

void OutlierRemovalFilter::random_sample_filter(PointCloudT::Ptr cloud, const int size)
{
  pcl::RandomSample<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setSample(size);
  sor.filter(*cloud);
}

void OutlierRemovalFilter::add_dummy_ground_points(PointCloudT::Ptr cloud, const int point_num, const float size)
{
  pcl::PointCloud<PointT> dummy_cloud;
  dummy_cloud.points.resize(point_num);
  for (auto &point : dummy_cloud)
  {
    point.x = size * rand() / (RAND_MAX + 1.0f) - size / 2.0;
    point.y = size * rand() / (RAND_MAX + 1.0f) - size / 2.0;
    point.z = 0.0;
  }
  cloud->insert(cloud->end(), dummy_cloud.begin(), dummy_cloud.end());
}

void OutlierRemovalFilter::outlier_removal_filter(
    PointCloudT::Ptr cloud, const int num_neighbors, const float std_dev_mul_thresh)
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(num_neighbors);
  sor.setStddevMulThresh(std_dev_mul_thresh);
  sor.filter(*cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_outlier_removal_filter");
  OutlierRemovalFilter outlier_removal_filter;
  ros::spin();

  return 0;
}
