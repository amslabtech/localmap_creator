
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle nh;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};
My_Filter::My_Filter(){
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 1);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	sensor_msgs::PointCloud2 cloud;
	try{
		projector_.transformLaserScanToPointCloud("/base_link", *scan, cloud, tfListener_);
		point_cloud_publisher_.publish(cloud);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2pcl");

    My_Filter filter;

    ros::spin();

    return 0;
}
