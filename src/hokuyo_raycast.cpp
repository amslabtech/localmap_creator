#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <tf/tf.h>

class OccupancyGridLidar{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub;
		/*publish*/
		ros::Publisher pub;
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		/*grid*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_all_zero;
		/*publish infomations*/
		std::string pub_frameid;
		ros::Time pub_stamp;
		/*const values*/
		const double w = 20.0;	//x[m]
		const double h = 20.0;	//y[m]
		const double resolution = 0.1;	//[m]
		// const double range_road_intensity[2] = {5, 15};
	public:
		OccupancyGridLidar();
		void GridInitialization(void);
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc);
		void InputGrid(void);
		int MeterpointToIndex(double x, double y);
		void Publication(void);
};


OccupancyGridLidar::OccupancyGridLidar()
{
	sub = nh.subscribe("/hokuyo_points", 1, &OccupancyGridLidar::CallbackRmGround, this);
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar", 1);
	GridInitialization();
}

void OccupancyGridLidar::GridInitialization(void)/*{{{*/
{
	grid.info.resolution = resolution;
	grid.info.width = w/resolution + 1;
	grid.info.height = h/resolution + 1;
	grid.info.origin.position.x = -w*0.5;
	grid.info.origin.position.y = -h*0.5;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;
	grid.info.origin.orientation.w = 1.0;
	int loop_lim = grid.info.width*grid.info.height;
	for(int i=0;i<loop_lim;i++)	grid.data.push_back(0);
	// frame_id is same as the one of subscribed pc
	grid_all_zero = grid;
}/*}}}*/

void OccupancyGridLidar::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	pcl::fromROSMsg(*msg, *rmground);

	ExtractPCInRange(rmground);

	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;

	InputGrid();
	Publication();
}/*}}}*/

void OccupancyGridLidar::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)/*{{{*/
{
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w*0.5, w*0.5);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h*0.5, h*0.5);
	pass.filter(*pc);
}/*}}}*/

void OccupancyGridLidar::InputGrid(void)
{
	grid = grid_all_zero;
	//obstacle
	size_t loop_lim = rmground->points.size();
	for(size_t i=0;i<loop_lim;i++){
		grid.data[MeterpointToIndex(rmground->points[i].x, rmground->points[i].y)] = 100;
	}
}

int OccupancyGridLidar::MeterpointToIndex(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width*0.5;
	int y_ = y/grid.info.resolution + grid.info.height*0.5;
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
    ros::init(argc, argv, "hokuyo_raycast");
	std::cout << "= hokuyo_raycast =" << std::endl;
	
	OccupancyGridLidar occupancygrid_lidar;

	ros::spin();
}
