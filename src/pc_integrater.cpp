#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

void syncMsgsCB(const sensor_msgs::PointCloud2::ConstPtr &pc1, const sensor_msgs::PointCloud2::ConstPtr &pc2, const sensor_msgs::PointCloud2::ConstPtr &pc3)
{

}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pc_integrater");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc1(nh, "pc1", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc2(nh, "pc2", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc3(nh, "pc3", 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc1, pc2, pc3);
    sync.registerCallback(boost::bind(&syncMsgsCB, _1, _2, _3));

    ros::spin();

    return 0;
}
