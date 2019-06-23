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
#include <tf/transform_listener.h>



class PointCloudTransform{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		ros::Subscriber sub;
		ros::Publisher pub_curv;
		ros::Publisher pub_plane;
		
		sensor_msgs::PointCloud pc_;

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void Passthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc);
		void SetCurvature(	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pc,
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& plane,
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& curv);
};

PointCloudTransform::PointCloudTransform()
    : nhPrivate("~")
{
	sub = nh.subscribe("/cloud", 1, &PointCloudTransform::Callback, this);
	pub_curv = nh.advertise<sensor_msgs::PointCloud2>("/cloud/curv", 1);
	pub_plane = nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	sensor_msgs::PointCloud2 curv;
	sensor_msgs::PointCloud2 plane;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr curv_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cpy {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
	pcl::fromROSMsg(*msg, *pcl_cloud);

	Passthrough(pcl_cloud);
	pcl::copyPointCloud(*pcl_cloud, *cloud_cpy);
	SetCurvature(cloud_cpy,plane_cloud,curv_cloud);

	pcl::toROSMsg(*curv_cloud, curv);
	pcl::toROSMsg(*plane_cloud, plane);
	curv.header.frame_id = msg->header.frame_id;
	plane.header.frame_id = msg->header.frame_id;
	curv.header.stamp = msg->header.stamp;
	plane.header.stamp = msg->header.stamp;
	pub_curv.publish(curv);
	pub_plane.publish(plane);
}

void PointCloudTransform::Passthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
{
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0, 2.0);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.0, 1.0);
	pass.filter(*pc);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0, 1.0);
	pass.filter(*pc);
}

void PointCloudTransform::SetCurvature(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& pc,
										pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& plane,
										pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& curv)
{
	pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
	ne.setInputCloud (pc);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.1);
	ne.compute(*pc);
	
	const double CURVATURE_THRESHOLD = 3.0e-4;

	int loop_lim = pc->points.size();
	for(int i=0; i<loop_lim; i++){
		if(pc->points[i].curvature>CURVATURE_THRESHOLD){
			curv->points.push_back(pc->points[i]);
		}else{
			plane->points.push_back(pc->points[i]);
		}
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_curvature");
	PointCloudTransform transform;
	ros::spin();
	
	return 0;
}
