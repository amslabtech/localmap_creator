//subscribe:
//		/pointcloud/velodyne/rm_ground
//		/pointcloud/velodyne/ground/stored
//		/pointcloud/hokuyo
//		/pointcloud/realsense
//		(all of frameid = base_link)
//
//publish:
//		/occupancygrid/localmap

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

class MakingLocalmap{
    private:
        ros::NodeHandle nh;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
																sensor_msgs::PointCloud2, 
																sensor_msgs::PointCloud2, 
																sensor_msgs::PointCloud2> making_localmap_sync_subs;

        message_filters::Subscriber<sensor_msgs::PointCloud2> ground_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> rmground_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> hokuyo_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> realsense_sub;
        message_filters::Synchronizer<making_localmap_sync_subs> making_localmap_sync;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr rmground_ {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_ {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr hokuyo_ {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_ {new pcl::PointCloud<pcl::PointXYZ>};
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_all_minusone;

		const double w = 20.0;	//x[m]
		const double h = 20.0;	//y[m]
		const double resolution = 0.1;	//[m]
		
		std::string pub_frameid;
		ros::Time pub_stamp;
        ros::Publisher pub;

    public:
        MakingLocalmap();
		void GridInitialization(void);
        void Callback(const sensor_msgs::PointCloud2::ConstPtr&, 
					  const sensor_msgs::PointCloud2::ConstPtr&, 
					  const sensor_msgs::PointCloud2::ConstPtr&, 
					  const sensor_msgs::PointCloud2::ConstPtr&);
        void making_localmap(const sensor_msgs::PointCloud2::ConstPtr&, 
							 const sensor_msgs::PointCloud2::ConstPtr&, 
							 const sensor_msgs::PointCloud2::ConstPtr&, 
							 const sensor_msgs::PointCloud2::ConstPtr&);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
		void InputGrid(void);
		int MeterpointToIndex(double x, double y);
		void Publication(void);
};


MakingLocalmap::MakingLocalmap()
    : nh("~"),
      ground_sub(nh, "/ground", 10), 
	  rmground_sub(nh, "/rm_ground", 10), 
	  hokuyo_sub(nh, "/hokuyo", 10), 
	  realsense_sub(nh, "/realsense", 10),
      making_localmap_sync(making_localmap_sync_subs(10), ground_sub, rmground_sub, hokuyo_sub, realsense_sub)
{
    making_localmap_sync.registerCallback(boost::bind(&MakingLocalmap::Callback, this, _1, _2, _3, _4));
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/localmap", 10);
	GridInitialization();
}


void MakingLocalmap::GridInitialization(void)
{
	grid.info.resolution = resolution;
	grid.info.width = w/resolution + 1;
	grid.info.height = h/resolution + 1;
	grid.info.origin.position.x = -w/2.0;
	grid.info.origin.position.y = -h/2.0;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;
	grid.info.origin.orientation.w = 1.0;
	for(int i=0;i<grid.info.width*grid.info.height;i++)	grid.data.push_back(-1);
	// frame_id is same as the one of subscribed pc
	grid_all_minusone = grid;
}

void MakingLocalmap::Callback(const sensor_msgs::PointCloud2::ConstPtr& ground,
                             	   const sensor_msgs::PointCloud2::ConstPtr& rmground,
                             	   const sensor_msgs::PointCloud2::ConstPtr& hokuyo,
								   const sensor_msgs::PointCloud2::ConstPtr& realsense)
{
    making_localmap(ground, rmground, hokuyo, realsense);
}


void MakingLocalmap::making_localmap(const sensor_msgs::PointCloud2::ConstPtr& ground,    
                                      	  const sensor_msgs::PointCloud2::ConstPtr& rmground,  
                                      	  const sensor_msgs::PointCloud2::ConstPtr& hokuyo,    
                                          const sensor_msgs::PointCloud2::ConstPtr& realsense)
{
	pcl::fromROSMsg(*ground, *ground_);
	pcl::fromROSMsg(*rmground, *rmground_);
	pcl::fromROSMsg(*hokuyo, *hokuyo_);
	pcl::fromROSMsg(*realsense, *realsense_);

	ExtractPCInRange(ground_);
	ExtractPCInRange(rmground_);
	ExtractPCInRange(hokuyo_);
	ExtractPCInRange(realsense_);

	grid = grid_all_minusone;

	pub_frameid = ground->header.frame_id;
	pub_stamp = ground->header.stamp;
	
	InputGrid();

	Publication();
}

void MakingLocalmap::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w/2.0, w/2.0);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h/2.0, h/2.0);
	pass.filter(*pc);
}

void MakingLocalmap::InputGrid(void)
{
	grid = grid_all_minusone;
	//ground
	for(size_t i=0;i<ground_->points.size();i++){
		grid.data[MeterpointToIndex(ground_->points[i].x, ground_->points[i].y)] = 0;
	}
	//rmground
	for(size_t i=0;i<rmground_->points.size();i++){
		grid.data[MeterpointToIndex(rmground_->points[i].x, rmground_->points[i].y)] = 100;
	}
	//hokuyo
	for(size_t i=0;i<hokuyo_->points.size();i++){
		grid.data[MeterpointToIndex(hokuyo_->points[i].x, hokuyo_->points[i].y)] = 100;
	}
	//realsense
	for(size_t i=0;i<realsense_->points.size();i++){
		grid.data[MeterpointToIndex(realsense_->points[i].x, realsense_->points[i].y)] = 100;
	}
}

int MakingLocalmap::MeterpointToIndex(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void MakingLocalmap::Publication(void)
{
	grid.header.frame_id = pub_frameid;
	grid.header.stamp = pub_stamp;
	pub.publish(grid);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MakingLocalmap_neo");

    MakingLocalmap making_localmap;

    ros::spin();

    return 0;
}
