#include <boost/mpl/min_max.hpp>
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
    void extract_points(PointCloudType &cloud);

    double max_height_;
    double min_height_;
    std::string target_frame_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
};

PcExtractor::PcExtractor() : private_nh_("~")
{
    private_nh_.param("max_height", max_height_, 1.5);
    private_nh_.param("min_height", min_height_, 0.05);

    ROS_INFO("=== PC Extractor ===");
    ROS_INFO_STREAM("max_height: " << max_height_);
    ROS_INFO_STREAM("min_height: " << min_height_);

    point_cloud_sub_ = nh_.subscribe(
        "/cloud_in", 1, &PcExtractor::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
}

void PcExtractor::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudType pc;
    pcl::fromROSMsg(*msg, pc);
    extract_points(pc);
    point_cloud_pub_.publish(pc);
}

void PcExtractor::extract_points(PointCloudType &cloud)
{
    ROS_WARN_STREAM("cloud size: " << cloud.points.size());
    for (int i = 0; i < cloud.points.size(); i++)
    {
        ROS_WARN_STREAM("cloud z: " << cloud.points[i].z);
        ROS_WARN_STREAM("judege: " << (cloud.points[i].z < min_height_ || max_height_ < cloud.points[i].z));
        if (cloud.points[i].z < min_height_ || max_height_ < cloud.points[i].z)
            cloud.points.erase(cloud.points.begin() + i);
    }
    ROS_WARN("extract");
    ROS_WARN_STREAM("cloud size: " << cloud.points.size());
    ROS_WARN("---");
}

void PcExtractor::process() { ros::spin(); }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_extractor");
    PcExtractor pc_extractor;
    pc_extractor.process();

    return 0;
}
