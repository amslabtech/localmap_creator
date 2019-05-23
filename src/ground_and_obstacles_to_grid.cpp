#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class OccupancyGridLidar{
	private:
		ros::NodeHandle nh;
		
		/*subscribe*/
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_ground;
		
		/*publish*/
		ros::Publisher pub;
		
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground {new pcl::PointCloud<pcl::PointXYZINormal>};
		/*grid*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_all_minusone;
		
		/*tf*/
		tf::TransformListener tflistener;
		
		/*publish infomations*/
		std::string pub_frameid;
		ros::Time pub_stamp;
		
		/*const values*/
		const double w = 20.0;	//x[m]
		const double h = 20.0;	//y[m]
		const double resolution = 0.1;	//[m]
		// const double range_road_intensity[2] = {5, 15};
		
		bool first_callback_ground = true;


	public:
		OccupancyGridLidar();
		void GridInitialization(void);
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void InputGrid(void);
		int MeterpointToIndex(double x, double y);
		void Publication(void);
};


OccupancyGridLidar::OccupancyGridLidar()
{
	sub_rmground = nh.subscribe("/rm_ground", 1, &OccupancyGridLidar::CallbackRmGround, this);
	sub_ground = nh.subscribe("/ground", 1, &OccupancyGridLidar::CallbackGround, this);
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar", 1);
	GridInitialization();
}

void OccupancyGridLidar::GridInitialization(void)/*{{{*/
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
}/*}}}*/

void OccupancyGridLidar::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	
	sensor_msgs::PointCloud2 pc2_out;
	try{
		pcl_ros::transformPointCloud("/base_link", *msg, pc2_out, tflistener);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	pcl::fromROSMsg(pc2_out, *rmground);

	ExtractPCInRange(rmground);

	pub_frameid = "/base_link";
	pub_stamp = msg->header.stamp;

	InputGrid();
	if(!first_callback_ground)
		Publication();
}/*}}}*/

void OccupancyGridLidar::CallbackGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	sensor_msgs::PointCloud2 pc2_out;
	try{
		pcl_ros::transformPointCloud("/base_link", *msg, pc2_out, tflistener);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	pcl::fromROSMsg(pc2_out, *tmp_pc);
	ExtractPCInRange(tmp_pc);
	
	pcl::copyPointCloud(*tmp_pc, *ground);
	first_callback_ground = false;
}/*}}}*/

void OccupancyGridLidar::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)/*{{{*/
{
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w/2.0, w/2.0);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h/2.0, h/2.0);
	pass.filter(*pc);
}/*}}}*/

void OccupancyGridLidar::InputGrid(void)
{
	grid = grid_all_minusone;
	//intensity
	for(size_t i=0;i<ground->points.size();i++){
		// if(ground->points[i].intensity<range_road_intensity[0] || ground->points[i].intensity>range_road_intensity[1])	grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = 50;
		// else	grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = 0;
		
		grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = 0;
	}
	//obstacle
	for(size_t i=0;i<rmground->points.size();i++){
		grid.data[MeterpointToIndex(rmground->points[i].x, rmground->points[i].y)] = 100;
	}
}

int OccupancyGridLidar::MeterpointToIndex(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void OccupancyGridLidar::Publication(void)
{
	grid.header.frame_id = pub_frameid;
	grid.header.stamp = pub_stamp;
	pub.publish(grid);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_and_obstacle_to_grid");
	std::cout << "= ground_and_obstacle_to_grid =" << std::endl;
	
	OccupancyGridLidar occupancygrid_lidar;

	ros::spin();
}
