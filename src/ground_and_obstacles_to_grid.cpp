#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
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
		ros::NodeHandle private_nh;
		
		/*subscribe*/
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_ground;
		ros::Subscriber sub_curv;
		
		/*publish*/
		ros::Publisher pub;
		ros::Publisher pub_grid;
		ros::Publisher pub_grass;
		ros::Publisher pub_downsampled_grass;
		
		/*cloud*/
		sensor_msgs::PointCloud2 grass;
		sensor_msgs::PointCloud2 downsampled;

		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr curv {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr grass_points {new pcl::PointCloud<pcl::PointXYZINormal>};
		/*grid*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_filtered;
		nav_msgs::OccupancyGrid grid_all_minusone;
		
		/*tf*/
		tf::TransformListener tflistener;
		
		/*publish infomations*/
		std::string pub_frameid;
		ros::Time pub_stamp;
		
		/*const values*/
		double w;	//x[m]
		double h;	//y[m]
		double resolution;	//[m]
		double resolution_rec;	//[m]
		int width;	//
		double width_rec;	//
		int height;	//
		const int grass_score = 50;//
		
		
		double ROAD_INTENSITY_MIN;
		double ROAD_INTENSITY_MAX;
		double ZEROCELL_RATIO;
		int FILTER_RANGE;
		double range_road_intensity[2] = {ROAD_INTENSITY_MIN, ROAD_INTENSITY_MAX};
		
		bool first_callback_ground = true;
		bool first_callback_curv = true;


	public:
		OccupancyGridLidar();
		void GridInitialization(void);
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackCurv(const sensor_msgs::PointCloud2ConstPtr& msg);
		bool CellIsInside(nav_msgs::OccupancyGrid &grid, int x, int y);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc);
		void Filter(void);
		void InputGrid(void);
		int MeterpointToIndex(double x, double y);
		void IndexToPoint(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y);
		int PointToIndex(nav_msgs::OccupancyGrid &grid, int x, int y);
		void Publication(void);
};


OccupancyGridLidar::OccupancyGridLidar()
	: private_nh("~")
{
	private_nh.param("resolution", resolution, {0.1});
	private_nh.param("w", w, {20.0});
	private_nh.param("h", h, {20.0});
	private_nh.param("ROAD_INTENSITY_MIN", ROAD_INTENSITY_MIN, {1});
	private_nh.param("ROAD_INTENSITY_MAX", ROAD_INTENSITY_MAX, {15});
	private_nh.param("FILTER_RANGE", FILTER_RANGE, {3});
	private_nh.param("ZEROCELL_RATIO", ZEROCELL_RATIO, {0.4});

	std::cout << "resolution         : " << resolution << std::endl;
	std::cout << "w                  : " << w << std::endl;
	std::cout << "h                  : " << h << std::endl;
	std::cout << "ROAD_INTENSITY_MIN : " << ROAD_INTENSITY_MIN << std::endl;
	std::cout << "ROAD_INTENSITY_MAX : " << ROAD_INTENSITY_MAX << std::endl;
	std::cout << "FILTER_RANGE       : " << FILTER_RANGE << std::endl;
	std::cout << "ZEROCELL_RATIO     : " << ZEROCELL_RATIO << std::endl;
	
	width = w/resolution+1;	//
	height = h/resolution+1;	//
	width_rec = 1/(double)width;	//
	resolution_rec = 1 / resolution;

	sub_rmground = nh.subscribe("/rm_ground", 1, &OccupancyGridLidar::CallbackRmGround, this);
	sub_ground = nh.subscribe("/ground", 1, &OccupancyGridLidar::CallbackGround, this);
	sub_curv = nh.subscribe("/curv", 1, &OccupancyGridLidar::CallbackCurv, this);
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar", 1);
	pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar/raw", 1);
	pub_grass = nh.advertise<sensor_msgs::PointCloud2>("/grass_points", 1);
	pub_downsampled_grass = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_points", 1);
	range_road_intensity[0] = ROAD_INTENSITY_MIN;
	range_road_intensity[1] = ROAD_INTENSITY_MAX;
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
	grid_all_minusone = grid;
}/*}}}*/

void OccupancyGridLidar::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	std::cout << "=====================" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc_ {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	sensor_msgs::PointCloud2 pc2_out;
	double time = ros::Time::now().toSec();
	pcl::fromROSMsg(*msg, *tmp_pc_);
    
	pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud (tmp_pc_);  
    vg.setLeafSize (0.05f, 0.05f, 100.0f);
    vg.filter (*tmp_pc);
	std::cout << "delay of downsampling rm_ground : " 
		<< ros::Time::now().toSec() - time << std::endl;
	time = ros::Time::now().toSec();
	
	try{
		pcl_ros::transformPointCloud("/base_link", *tmp_pc, *tmp_pc, tflistener);
		std::cout << "delay of rmground transform : " 
			<< ros::Time::now().toSec() - time << std::endl;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	time = ros::Time::now().toSec();

	ExtractPCInRange(tmp_pc);
	std::cout << "delay of extract obstacle in range : " 
		<< ros::Time::now().toSec() - time << std::endl;
	time = ros::Time::now().toSec();
	pcl::copyPointCloud(*tmp_pc, *rmground);
	
	pub_frameid = "/base_link";
	pub_stamp = msg->header.stamp;

	InputGrid();
	std::cout << "delay of input grid : " 
		<< ros::Time::now().toSec() - time << std::endl;
	time = ros::Time::now().toSec();
	
	grid.header.frame_id = pub_frameid;
	grid.header.stamp = pub_stamp;
	pub_grid.publish(grid);
	
	Filter();
	std::cout << "delay of filter : " 
		<< ros::Time::now().toSec() - time << std::endl;

	if(!first_callback_ground){
		Publication();
	}


}/*}}}*/

void OccupancyGridLidar::CallbackGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc_ {new pcl::PointCloud<pcl::PointXYZI>};
	double time = ros::Time::now().toSec();
	pcl::fromROSMsg(*msg, *tmp_pc_);

	pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud (tmp_pc_);  
    vg.setLeafSize (0.05f, 0.05f, 100.0f);
    vg.filter (*tmp_pc);
	std::cout << "delay of downsampling ground: " 
		<< ros::Time::now().toSec() - time << std::endl;
	pcl::toROSMsg(*tmp_pc, downsampled);
	pub_stamp = msg->header.stamp;
	downsampled.header.frame_id = pub_frameid;
	downsampled.header.stamp = pub_stamp;
	pub_downsampled_grass.publish(downsampled);
	time = ros::Time::now().toSec();

	try{
		pcl_ros::transformPointCloud("/base_link", *tmp_pc, *tmp_pc, tflistener);
		std::cout << "delay of ground transform	: " 
			<< ros::Time::now().toSec() - time << std::endl;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	time = ros::Time::now().toSec();
	ExtractPCInRange(tmp_pc);
	std::cout << "delay of extract ground in range : " 
		<< ros::Time::now().toSec() - time << std::endl;
	time = ros::Time::now().toSec();
	
	pcl::copyPointCloud(*tmp_pc, *ground);
	first_callback_ground = false;
}/*}}}*/

void OccupancyGridLidar::CallbackCurv(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	double time = ros::Time::now().toSec();
	try{
		pcl::fromROSMsg(*msg, *tmp_pc);

		pcl_ros::transformPointCloud("/base_link", *tmp_pc, *tmp_pc, tflistener);
		std::cout << "delay of curvcloud transform	: " 
			<< ros::Time::now().toSec() - time << std::endl;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	ExtractPCInRange(tmp_pc);
	
	pcl::copyPointCloud(*tmp_pc, *curv);
	first_callback_curv = false;
}/*}}}*/
void OccupancyGridLidar::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc)/*{{{*/
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

bool OccupancyGridLidar::CellIsInside(nav_msgs::OccupancyGrid &grid, int x, int y)/*{{{*/
{   
	int w = grid.info.width;
	int h = grid.info.height;
	if(x<-w*0.5)  return false;
	if(x>w*0.5-1) return false;
	if(y<-h*0.5)  return false;
	if(y>h*0.5-1) return false;
	return true;
}/*}}}*/



void OccupancyGridLidar::Filter(void)
{
	grid_filtered = grid;
	grass_points->points.clear();
	pcl::PointXYZINormal pt;
	pt.z=0;
	size_t loop_lim = grid.data.size();
	for(size_t i=0;i<loop_lim;i++){
		if(grid.data[i]==grass_score){
			int x, y;
			IndexToPoint(grid, i, x, y);
			int count_zerocell = 0; 
			int count_grasscell = 0; 
			for(int j=-FILTER_RANGE;j<=FILTER_RANGE;j++){
				for(int k=-FILTER_RANGE;k<=FILTER_RANGE;k++){
					if(CellIsInside(grid, x+j, y+k)){ 
						if(grid.data[PointToIndex(grid, x+j, y+k)]==0)	
							count_zerocell++;
						else if(grid.data[PointToIndex(grid, x+j, y+k)]==grass_score)
							count_grasscell++;
					}
				}
			}
			int num_cells = count_zerocell+count_grasscell;
			double threshold = num_cells*ZEROCELL_RATIO;
			// double distance = sqrt(x*x + y*y) * resolution;
			// double fanction =  0.06 * distance + 0.1;
			// double threshold = num_cells * fanction;
			if(count_zerocell>=threshold){
				grid_filtered.data[i] = 0;
			}else{
				pt.x=x*resolution;
				pt.y=y*resolution;
				grass_points->points.push_back(pt);
			}
		}
	}
	// std::cout << "grass_points.size : " << grass_points->points.size() << std::endl;
}

void OccupancyGridLidar::InputGrid(void)
{
	grid = grid_all_minusone;
	//intensity
	size_t loop_lim = ground->points.size();
	for(size_t i=0;i<loop_lim;i++){
		// if(ground->points[i].intensity<range_road_intensity[0] ||
		// ground->points[i].intensity>range_road_intensity[1]){
		// 	grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = grass_score;
		// 	
		// }else{
		// 	grid.data[Meterpoin
		// }
		grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = grass_score;
	}
	//obstacle
	loop_lim = rmground->points.size();
	for(size_t i=0;i<loop_lim;i++){
		grid.data[MeterpointToIndex(rmground->points[i].x, rmground->points[i].y)] = 100;
	}
	// curv
	loop_lim = curv->points.size();
	for(size_t i=0;i<loop_lim;i++){
		grid.data[MeterpointToIndex(curv->points[i].x, curv->points[i].y)] = 100;
	}
}



int OccupancyGridLidar::MeterpointToIndex(double x, double y)
{
	int x_ = x*resolution_rec + width*0.5;
	int y_ = y*resolution_rec + height*0.5;
	int index = y_*width + x_;
	return index;
}

void OccupancyGridLidar::IndexToPoint(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y)
{
	x = index%width - width*0.5;
	y = index*width_rec - height*0.5;
}


int OccupancyGridLidar::PointToIndex(nav_msgs::OccupancyGrid &grid, int x, int y)
{
	int x_ = x + width*0.5;
	int y_ = y + height*0.5;
	return	y_*width + x_;
}

void OccupancyGridLidar::Publication(void)
{
	grid_filtered.header.frame_id = pub_frameid;
	grid_filtered.header.stamp = pub_stamp;
	pub.publish(grid_filtered);

	pcl::toROSMsg(*grass_points, grass);
	grass.header.frame_id = pub_frameid;
	grass.header.stamp = pub_stamp;
	pub_grass.publish(grass);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_and_obstacle_to_grid");
	std::cout << "= ground_and_obstacle_to_grid =" << std::endl;
	
	OccupancyGridLidar occupancygrid_lidar;

	ros::spin();
}
