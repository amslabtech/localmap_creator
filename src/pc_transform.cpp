#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

class PointCloudTransform{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		tf::TransformListener tflistener;
		ros::Subscriber sub;
		ros::Publisher pub;
		sensor_msgs::PointCloud pc_;
		std::string target;

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

PointCloudTransform::PointCloudTransform()
    : nhPrivate("~")
{
	sub = nh.subscribe("/cloud", 1, &PointCloudTransform::Callback, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/transformed", 1);
	nhPrivate.getParam("target", target);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	sensor_msgs::PointCloud pc_in;
	sensor_msgs::PointCloud pc_trans;
	sensor_msgs::PointCloud2 pc2_out;
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_in);

	try{
		tflistener.waitForTransform(target, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud(target, msg->header.stamp, pc_in, msg->header.frame_id, pc_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_out);
		pub.publish(pc2_out);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_transform");
	
	PointCloudTransform transform;
	ros::spin();
}
