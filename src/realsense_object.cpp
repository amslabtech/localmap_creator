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
		ros::Subscriber sub;
		ros::Publisher pub_curv;
		ros::Publisher pub_plane;
		
		sensor_msgs::PointCloud pc_;
		sensor_msgs::PointCloud2 curv;
		sensor_msgs::PointCloud2 plane;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr curv_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cpy {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Passthrough(void);
		// void SetCurvature(void);
};

PointCloudTransform::PointCloudTransform()
{
	sub = nh.subscribe("/cloud", 1, &PointCloudTransform::Callback, this);
	pub_curv = nh.advertise<sensor_msgs::PointCloud2>("/cloud/curv", 1);
	pub_plane = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pcl_cloud);

	pcl::copyPointCloud(*pcl_cloud, *cloud_cpy);
	Passthrough();
	// SetCurvature();

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
	pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
	pass.setInputCloud(cloud_cpy);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.3, 1.2);
	pass.filter(*cloud_cpy);
	pass.setInputCloud(cloud_cpy);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.8, 0.8);
	pass.filter(*cloud_cpy);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.5, 0.5);
	pass.filter(*cloud_cpy);

	pass.setFilterLimits(-0.10, 0.10);
	pass.filter(*plane_cloud);
	pass.setFilterLimitsNegative(true);
	pass.filter(*curv_cloud);
	
}

// void PointCloudTransform::SetCurvature(void)
// {
// 	pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
// 	ne.setInputCloud (cloud_cpy);
// 	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
// 	ne.setSearchMethod(tree);
// 	ne.setRadiusSearch(0.3);
// 	ne.compute(*cloud_cpy);
// 	
// 	const double CURVATURE_THRESHOLD = 3.0e-4;
//
// 	int loop_lim = cloud_cpy->points.size();
// 	for(int i=0; i<loop_lim; i++){
// 		if(cloud_cpy->points[i].curvature>CURVATURE_THRESHOLD){
// 			curv_cloud->points.push_back(cloud_cpy->points[i]);
// 		}else{
// 			plane_cloud->points.push_back(cloud_cpy->points[i]);
// 			
// 		}
// 	}
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_object");
	PointCloudTransform transform;
	ros::spin();
	
	return 0;
}
