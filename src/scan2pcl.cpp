/**
* @file scan2pcl.cpp
* @brief C++ scan data to xy pcl data
* @author AMSL
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>

/**
* Class for scan to pcl
*/
class ScanToPcl {
     public:
         /**
         * Constructor
         */
        ScanToPcl();
        /**
         * set scan data
         */
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle nh; //NodeHandle
        laser_geometry::LaserProjection projector_; //package for 2DLaser scan to point cloud

        tf::TransformListener tfListener_; //receive Transform

        ros::Publisher point_cloud_pub_; //Publisher
        ros::Subscriber scan_sub_; //Subscriber
};

/**
 * subscribe and publish
 *
 */
ScanToPcl::ScanToPcl(){
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &ScanToPcl::scan_callback, this); //Subscriber
    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 1); //Publisher
    tfListener_.setExtrapolationLimit(ros::Duration(0.1)); //set the distance which tf is allow to extrapolate
}

/**
 * set scan data
 *
 */
void ScanToPcl::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	sensor_msgs::PointCloud2 cloud;
    //code that may raoise an exception
	try{
		projector_.transformLaserScanToPointCloud("/base_link", *scan, cloud, tfListener_); //projection of laser
		point_cloud_pub_.publish(cloud); //publish point cloud
	}
    //exception handling
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what()); //error message
	}
}

/**
 * main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2pcl");

    ScanToPcl scan_to_pcl;

    ros::spin();

    return 0;
}
