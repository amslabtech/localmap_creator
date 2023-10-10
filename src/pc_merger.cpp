#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

class PcMerger
{
public:
    PcMerger();
    void process();

protected:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;

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
    PointCloudType cloud_;
    tf::TransformListener tfListener_;
};

PcMerger::PcMerger() : private_nh_("~"), cloud_count_(0)
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
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);

    cloud_.header.frame_id = target_frame_;
}

void PcMerger::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudTypePtr pc_ptr(new PointCloudType);
    pcl::fromROSMsg(*msg, *pc_ptr);
    PointCloudTypePtr pc_transformed_ptr(new PointCloudType);
    pcl_ros::transformPointCloud(target_frame_, *pc_ptr, *pc_transformed_ptr, tfListener_);
    cloud_.points.insert(cloud_.points.end(), pc_transformed_ptr->begin(), pc_transformed_ptr->end());
    cloud_count_++;
}

void PcMerger::process()
{
    ros::Rate loop_rate(hz_);
    while (ros::ok())
    {
        ros::spinOnce();
        if (cloud_count_th_ <= cloud_count_)
            point_cloud_pub_.publish(cloud_);
        cloud_.points.clear();
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
