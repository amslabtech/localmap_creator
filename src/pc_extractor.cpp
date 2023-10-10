#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

class PcExtractor
{
public:
    PcExtractor();
    void process();

protected:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void extract_points(const PointCloudType &cloud);

    double max_height_;
    double min_height_;
    std::string target_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    PointCloudType cloud_;
    tf::TransformListener tfListener_;
};

PcExtractor::PcExtractor() : private_nh_("~")
{
    private_nh_.param<std::string>("target_frame", target_frame_, {"base_link"});
    private_nh_.param("max_height", max_height_, 1.5);
    private_nh_.param("min_height", min_height_, 0.05);

    ROS_INFO("=== PC Extractor ===");
    ROS_INFO_STREAM("target_frame: " << target_frame_);
    ROS_INFO_STREAM("max_height: " << max_height_);
    ROS_INFO_STREAM("min_height: " << min_height_);

    point_cloud_sub_ = nh_.subscribe(
        "/cloud_in", 1, &PcExtractor::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);

    cloud_.header.frame_id = target_frame_;
}

void PcExtractor::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudTypePtr pc_ptr(new PointCloudType);
    pcl::fromROSMsg(*msg, *pc_ptr);
    PointCloudTypePtr pc_transformed_ptr(new PointCloudType);
    pcl_ros::transformPointCloud(target_frame_, *pc_ptr, *pc_transformed_ptr, tfListener_);
    extract_points(*pc_transformed_ptr);
    cloud_.header.stamp = pc_transformed_ptr->header.stamp;
    point_cloud_pub_.publish(cloud_);
    cloud_.points.clear();
}

void PcExtractor::extract_points(const PointCloudType &cloud)
{
    for (int i = 0; i < cloud.points.size(); i++)
        if (min_height_ < cloud.points[i].z && cloud.points[i].z < max_height_)
            cloud_.points.push_back(cloud.points[i]);
}

void PcExtractor::process() { ros::spin(); }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_extractor");
    PcExtractor pc_extractor;
    pc_extractor.process();

    return 0;
}
