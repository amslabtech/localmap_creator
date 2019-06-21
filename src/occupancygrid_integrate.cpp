#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>

class OccupancyGridCombination{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;
		/*subscribe*/
		ros::Subscriber sub_grid_lidar;
		ros::Subscriber sub_grid_realsense;
		ros::Subscriber sub_grid_hokuyo;
		/*publish*/
		ros::Publisher pub;
		ros::Publisher pub_expand;
		/*objects*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_expand;
		nav_msgs::OccupancyGrid grid_lidar;
		nav_msgs::OccupancyGrid grid_realsense;
		nav_msgs::OccupancyGrid grid_hokuyo;
		/*flags*/
		bool first_callback_grid_lidar = true;
		bool first_callback_grid_realsense = true;
		bool first_callback_grid_hokuyo = true;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		ros::Time time_pub;
		/*param*/
		int EXPAND_RANGE;

	public:
		OccupancyGridCombination();
		void CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackGridRealsense(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackGridHokuyo(const nav_msgs::OccupancyGridConstPtr& msg);
		bool CellIsInside(nav_msgs::OccupancyGrid &grid, int x, int y);
		void CombineGrids(void);
		void Expand(void);
		void IndexToPoint(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y);
		int PointToIndex(nav_msgs::OccupancyGrid &grid, int x, int y);
		void Publication(void);
};

OccupancyGridCombination::OccupancyGridCombination()
	: private_nh("~")
{
	private_nh.param("EXPAND_RANGE", EXPAND_RANGE, {4});
	std::cout << "EXPAND_RANGE : " << EXPAND_RANGE << std::endl;


	sub_grid_lidar = nh.subscribe("/occupancygrid/lidar/stored", 1, 
			&OccupancyGridCombination::CallbackGridLidar, this);
	sub_grid_realsense = nh.subscribe("/occupancygrid/realsense", 1, 
			&OccupancyGridCombination::CallbackGridRealsense, this);
	sub_grid_hokuyo = nh.subscribe("/occupancygrid/hokuyo", 1, 
			&OccupancyGridCombination::CallbackGridHokuyo, this);
	
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
	pub_expand = nh.advertise<nav_msgs::OccupancyGrid>("/local_map/expand",1);
}

void OccupancyGridCombination::CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg)
{
	std::cout<<"CallbackGridLidar"<<std::endl;	
	grid_lidar = *msg;

	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid = *msg;
		grid_expand =*msg;
		for(size_t i=0;i<grid_realsense.data.size();i++)	grid.data[i] = -1;
	}
	first_callback_grid_lidar = false;

	time_pub = msg->header.stamp;
	CombineGrids();
	if(!grid.data.empty())	Publication();
}

void OccupancyGridCombination::CallbackGridRealsense(const nav_msgs::OccupancyGridConstPtr& msg)
{
	std::cout<<"CallbackGridRealsense"<<std::endl;	
	grid_realsense = *msg;
	
	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid = *msg;
		grid_expand =*msg;
		for(size_t i=0;i<grid_lidar.data.size();i++)	grid.data[i] = -1;
	}

	first_callback_grid_realsense = false;

	time_pub = msg->header.stamp;
	CombineGrids();
	if(!grid.data.empty())	Publication();
}

void OccupancyGridCombination::CallbackGridHokuyo(const nav_msgs::OccupancyGridConstPtr& msg)
{
	std::cout<<"CallbackGridHokuyo"<<std::endl;	
	grid_hokuyo = *msg;

	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid = *msg;
		grid_expand =*msg;
		for(size_t i=0;i<grid_hokuyo.data.size();i++)	grid.data[i] = -1;
	}
		
	first_callback_grid_hokuyo = false;
	
	time_pub = msg->header.stamp;
	CombineGrids();
	if(!grid.data.empty())	Publication();
}

void OccupancyGridCombination::Expand(void)
{
	int loop_lim = grid.data.size();
	for(size_t i=0;i<loop_lim;i++){
		if(grid.data[i]==100){
			int x, y;
			IndexToPoint(grid, i, x, y);
			for(int j=-EXPAND_RANGE;j<=EXPAND_RANGE;j++){
				for(int k=-EXPAND_RANGE;k<=EXPAND_RANGE;k++){
					if(CellIsInside(grid, x+j, y+k)){ 
						grid_expand.data[PointToIndex(grid, x+j,y+k)] = 100;
					}
				}
			}
		}
	}
}

void OccupancyGridCombination::CombineGrids(void)
{
	for(size_t i=0;i<grid.data.size();i++){
		if(!first_callback_grid_lidar)grid.data[i] = grid_lidar.data[i];
		if(!first_callback_grid_realsense && grid_realsense.data[i]!=-1) grid.data[i] = grid_realsense.data[i];
		if(!first_callback_grid_hokuyo && grid.data[i]<=0) grid.data[i] = grid_hokuyo.data[i];
	}
	
	for(size_t i=0;i<grid.data.size();i++){
		grid_expand.data[i] = grid.data[i];
	}
	Expand();
}

void OccupancyGridCombination::IndexToPoint(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width*0.5;
	y = index/grid.info.width - grid.info.height*0.5;
}


int OccupancyGridCombination::PointToIndex(nav_msgs::OccupancyGrid &grid, int x, int y)
{
	int x_ = x + grid.info.width*0.5;
	int y_ = y + grid.info.height*0.5;
	return	y_*grid.info.width + x_;
}

bool OccupancyGridCombination::CellIsInside(nav_msgs::OccupancyGrid &grid, int x, int y)
{   
	int w = grid.info.width;
	int h = grid.info.height;
	if(x<-w*0.5)  return false;
	if(x>w*0.5-1) return false;
	if(y<-h*0.5)  return false;
	if(y>h*0.5-1) return false;
	return true;
}


void OccupancyGridCombination::Publication(void)
{
	std::cout<<"Publication"<<std::endl;	
	grid.header.stamp = time_pub;
	grid_expand.header.stamp = time_pub;
	pub.publish(grid);
	pub_expand.publish(grid_expand);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_integrate");
	std::cout << "= occupancygrid_integrate =" << std::endl;

	OccupancyGridCombination occupancygrid_combine;

	ros::spin();
}
