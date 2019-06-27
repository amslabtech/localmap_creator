#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>



class PointCloudTransform{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;
		ros::Subscriber sub;
		ros::Publisher pub_curv;
		ros::Publisher pub_plane;
		
		sensor_msgs::PointCloud pc_;
		sensor_msgs::PointCloud2 curv;
		sensor_msgs::PointCloud2 plane;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curv_cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};

		double UPPER_LIMIT;
		double UNDER_LIMIT;
		double VIEW_RANGE_X_MIN;
		double VIEW_RANGE_X_MAX;
		double VIEW_RANGE_Y_MIN;
		double VIEW_RANGE_Y_MAX;
		double VIEW_RANGE_Z_MIN;
		double VIEW_RANGE_Z_MAX;

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Passthrough(void);
		// void SetCurvature(void);
};

PointCloudTransform::PointCloudTransform()
	: private_nh("~")
{
	private_nh.param("UPPER_LIMIT", UPPER_LIMIT, {0.1});
	private_nh.param("UNDER_LIMIT", UNDER_LIMIT, {0.1});
	private_nh.param("VIEW_RANGE_X_MIN", VIEW_RANGE_X_MIN, {0.3});
	private_nh.param("VIEW_RANGE_X_MAX", VIEW_RANGE_X_MAX, {1.0});
	private_nh.param("VIEW_RANGE_Y_MIN", VIEW_RANGE_Y_MIN, {-0.8});
	private_nh.param("VIEW_RANGE_Y_MAX", VIEW_RANGE_Y_MAX, {0.8});
	private_nh.param("VIEW_RANGE_Z_MIN", VIEW_RANGE_Z_MIN, {-0.5});
	private_nh.param("VIEW_RANGE_Z_MAX", VIEW_RANGE_Z_MAX, {0.5});

	std::cout << "UPPER_LIMIT 		: " << UPPER_LIMIT << std::endl;
	std::cout << "UNDER_LIMIT 		: " << UNDER_LIMIT << std::endl;
	std::cout << "VIEW_RANGE_X_MIN 	: " << VIEW_RANGE_X_MIN << std::endl;
	std::cout << "VIEW_RANGE_X_MAX 	: " << VIEW_RANGE_X_MAX << std::endl;
	std::cout << "VIEW_RANGE_Y_MIN 	: " << VIEW_RANGE_Y_MIN << std::endl;
	std::cout << "VIEW_RANGE_Y_MAX 	: " << VIEW_RANGE_Y_MAX << std::endl;
	std::cout << "VIEW_RANGE_Z_MIN 	: " << VIEW_RANGE_Z_MIN << std::endl;
	std::cout << "VIEW_RANGE_Z_MAX 	: " << VIEW_RANGE_Z_MAX << std::endl;


	sub = nh.subscribe("/cloud", 1, &PointCloudTransform::Callback, this);
	pub_curv = nh.advertise<sensor_msgs::PointCloud2>("/cloud/curv", 1);
	pub_plane = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pcl_cloud);

	Passthrough();

	pcl::toROSMsg(*curv_cloud, curv);
	pcl::toROSMsg(*plane_cloud, plane);
	curv.header.frame_id = msg->header.frame_id;
	plane.header.frame_id = msg->header.frame_id;
	curv.header.stamp = msg->header.stamp;
	plane.header.stamp = msg->header.stamp;

	pub_curv.publish(curv);
	pub_plane.publish(plane);
}

void PointCloudTransform::Passthrough(void)
{
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(pcl_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(VIEW_RANGE_X_MIN, VIEW_RANGE_X_MAX);
	pass.filter(*pcl_cloud);
	pass.setInputCloud(pcl_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(VIEW_RANGE_Y_MIN, VIEW_RANGE_Y_MAX);
	pass.filter(*pcl_cloud);
	pass.setInputCloud(pcl_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(VIEW_RANGE_Z_MIN, VIEW_RANGE_Z_MAX);
	pass.filter(*pcl_cloud);

	pass.setInputCloud(pcl_cloud);
	pass.setFilterLimits(UNDER_LIMIT, UPPER_LIMIT);
	pass.filter(*plane_cloud);
	
	pass.setFilterLimitsNegative(true);
	pass.setInputCloud(pcl_cloud);
	pass.setFilterLimits(UNDER_LIMIT, UPPER_LIMIT);
	pass.filter(*curv_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_object");
	PointCloudTransform transform;
	ros::spin();
	
	return 0;
}
