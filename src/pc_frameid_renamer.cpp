#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudTransform{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		ros::Subscriber sub;
		ros::Publisher pub;
		std::string new_id;

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

PointCloudTransform::PointCloudTransform()
	: nhPrivate("~")
{
	sub = nh.subscribe("/cloud", 1, &PointCloudTransform::Callback, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/renamed_frame", 1);
	nhPrivate.getParam("new_id", new_id);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	sensor_msgs::PointCloud2 pc = *msg;
	pc.header.frame_id = new_id;
	pub.publish(pc);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_frameid_rename");
	
	PointCloudTransform transform;
	ros::spin();
}
