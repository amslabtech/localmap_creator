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
	std::cout<<"CallbackGridLidar"<<std::endl;	
	grid_lidar = *msg;

	if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
		grid = *msg;
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
		for(size_t i=0;i<grid_hokuyo.data.size();i++)	grid.data[i] = -1;
	}
		
	first_callback_grid_hokuyo = false;
	
	time_pub = msg->header.stamp;
	CombineGrids();
	if(!grid.data.empty())	Publication();
}


void OccupancyGridCombination::CombineGrids(void)
{

	for(size_t i=0;i<grid.data.size();i++){
		if(!first_callback_grid_lidar)grid.data[i] = grid_lidar.data[i];
		if(!first_callback_grid_realsense && grid_realsense.data[i]!=-1) grid.data[i] = grid_realsense.data[i];
		if(!first_callback_grid_hokuyo && grid.data[i]<=0) grid.data[i] = grid_hokuyo.data[i];
	}
}



void OccupancyGridCombination::Publication(void)
{
	std::cout<<"Publication"<<std::endl;	
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
