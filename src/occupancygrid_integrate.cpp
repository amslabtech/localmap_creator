#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>

class OccupancyGridCombination{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_grid_lidar;
		ros::Subscriber sub_grid_realsense;
		ros::Subscriber sub_grid_hokuyo;
		/*publish*/
		ros::Publisher pub;
		/*objects*/
		nav_msgs::OccupancyGrid grid;
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
		/*node numbers*/
	public:
		OccupancyGridCombination();
		void CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackGridRealsense(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackGridHokuyo(const nav_msgs::OccupancyGridConstPtr& msg);
		void CombineGrids(void);
		void Publication(void);
};

OccupancyGridCombination::OccupancyGridCombination()
{
	sub_grid_lidar = nh.subscribe("/occupancygrid/lidar/stored", 1, 
			&OccupancyGridCombination::CallbackGridLidar, this);
	sub_grid_realsense = nh.subscribe("/occupancygrid/realsense", 1, 
			&OccupancyGridCombination::CallbackGridRealsense, this);
	sub_grid_hokuyo = nh.subscribe("/occupancygrid/hokuyo", 1, 
			&OccupancyGridCombination::CallbackGridHokuyo, this);
	
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
}

void OccupancyGridCombination::CallbackGridLidar(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_lidar = *msg;
	if(grid.data.empty())	grid = *msg;

	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid_realsense = *msg;
		for(size_t i=0;i<grid_realsense.data.size();i++)	grid_realsense.data[i] = -1;
	}
		
	first_callback_grid_lidar = false;

	time_pub = msg->header.stamp;
	if(!grid_lidar.data.empty() && !grid_realsense.data.empty() && !grid_hokuyo.data.empty()){
		CombineGrids();
	}
	if(!grid.data.empty())	Publication();
}

void OccupancyGridCombination::CallbackGridRealsense(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_realsense = *msg;
	if(grid.data.empty())	grid = *msg;
	
	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid_lidar = *msg;
		for(size_t i=0;i<grid_lidar.data.size();i++)	grid_lidar.data[i] = -1;
	}

	first_callback_grid_realsense = false;

	time_pub = msg->header.stamp;
	if(!grid_lidar.data.empty() && !grid_realsense.data.empty() && !grid_hokuyo.data.empty()){
		CombineGrids();
	}
	if(!grid.data.empty())	Publication();
}

void OccupancyGridCombination::CallbackGridHokuyo(const nav_msgs::OccupancyGridConstPtr& msg)
{
	grid_hokuyo = *msg;
	if(grid.data.empty())	grid = *msg;

	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid_hokuyo = *msg;
		for(size_t i=0;i<grid_hokuyo.data.size();i++)	grid_hokuyo.data[i] = -1;
	}
		
	first_callback_grid_lidar = false;
	
	time_pub = msg->header.stamp;
	if(!grid_lidar.data.empty() && !grid_realsense.data.empty() && !grid_hokuyo.data.empty()){
		CombineGrids();
	}
	if(!grid.data.empty())	Publication();
}


void OccupancyGridCombination::CombineGrids(void)
{

	for(size_t i=0;i<grid.data.size();i++){
		grid.data[i] = grid_lidar.data[i];
		if(grid_realsense.data[i]!=-1) grid.data[i] = grid_realsense.data[i];
		if(grid.data[i]!=100) grid.data[i] = grid_hokuyo.data[i];
	}
}



void OccupancyGridCombination::Publication(void)
{
	grid.header.stamp = time_pub;
	pub.publish(grid);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_integrate");
	std::cout << "= occupancygrid_integrate =" << std::endl;

	OccupancyGridCombination occupancygrid_combine;

	ros::spin();
}
