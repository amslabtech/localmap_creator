#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

class PclMerger
{
public:
    PclMerger();
    void process();

protected:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    int hz_;
    int cloud_num_;
    std::string target_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::vector<ros::Subscriber> point_cloud_subs_;
    ros::Publisher point_cloud_pub_;
    PointCloudType cloud_;
    tf::TransformListener tfListener_;
};

PclMerger::PclMerger() : private_nh_("~")
{
    private_nh_.param<std::string>("target_frame", target_frame_, {"base_link"});
    private_nh_.param<int>("hz", hz_, 10);
    private_nh_.param<int>("cloud_num", cloud_num_, 2);

    point_cloud_subs_.resize(cloud_num_);
    for (int i = 0; i < cloud_num_; i++)
    {
        point_cloud_subs_[i] = private_nh_.subscribe(
            "/cloud" + std::to_string(i), 1, &PclMerger::cloud_callback, this,
            ros::TransportHints().reliable().tcpNoDelay());
    }
    point_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);

    cloud_.header.frame_id = target_frame_;
}

void PclMerger::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudTypePtr pc_ptr(new PointCloudType);
    pcl::fromROSMsg(*msg, *pc_ptr);
    PointCloudTypePtr pc_transformed_ptr(new PointCloudType);
    pcl_ros::transformPointCloud(target_frame_, *pc_ptr, *pc_transformed_ptr, tfListener_);
    cloud_.points.insert(cloud_.points.end(), pc_transformed_ptr->begin(), pc_transformed_ptr->end());
}

void PclMerger::process()
{
    ros::Rate loop_rate(hz_);
    while (ros::ok())
    {
        ros::spinOnce();
        point_cloud_pub_.publish(cloud_);
        cloud_.points.clear();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_merger");
    PclMerger pcl_merger;
    pcl_merger.process();

    return 0;
}
