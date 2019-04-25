#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

class OccupancyGridStore{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_grid;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub;
		/*objects*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_all_minusone;
		nav_msgs::Odometry odom;
		double theta;
		double delta_x;
		double delta_y;
		int count_same_odom;
		/*flags*/
		bool first_callback_grid = true;
		bool first_callback_odom = true;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		double time_moving;
		double time_nomove;
	public:
		OccupancyGridStore();
		void CallbackGrid(const nav_msgs::OccupancyGridConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void MoveCells(double dt);
		void ClearObstacles(void);
		int PointToIndex(nav_msgs::OccupancyGrid grid, int x, int y);
		void IndexToMeterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y);
		int MeterpointToIndex(nav_msgs::OccupancyGrid grid, double x, double y);
		bool MeterPointIsInside(nav_msgs::OccupancyGrid grid, double x, double y);
		void Publication(void);
};

OccupancyGridStore::OccupancyGridStore()
{
	sub_grid = nh.subscribe("/occupancygrid/lidar", 1, &OccupancyGridStore::CallbackGrid, this);
	sub_odom = nh.subscribe("/tinypower/odom", 1, &OccupancyGridStore::CallbackOdom, this);
	pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar/stored",1);
	theta = 0.0;
	delta_x = 0.0;
	delta_y = 0.0;
	count_same_odom = 0;
}

void OccupancyGridStore::CallbackGrid(const nav_msgs::OccupancyGridConstPtr& msg)
{
	if(first_callback_grid){
		grid = *msg;
		grid_all_minusone = *msg;
		for(size_t i=0;i<grid_all_minusone.data.size();i++)	grid_all_minusone.data[i] = -1;
	}
	else{
		for(size_t i=0;i<grid.data.size();i++){
			if(msg->data[i]!=-1)	grid.data[i] = msg->data[i];
		}
	}

	first_callback_grid = false;
}

void OccupancyGridStore::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	bool encoder_works = true;
	if(msg->twist.twist.linear.x>1.0e-3 && fabs(msg->twist.twist.linear.x-odom.twist.twist.linear.x)<1.0e-3)	count_same_odom++;
	else	count_same_odom = 0;
	if(count_same_odom>5)	encoder_works = false;

	odom = *msg;

	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;
	
	if(msg->twist.twist.linear.x>1.0e-3){
		time_nomove = 0.0;
		time_moving += dt;
	}
	else{
		time_nomove += dt;
		time_moving = 0.0;
	}

	if(!first_callback_grid && !grid.data.empty()){
        if(encoder_works)	MoveCells(dt);
	    Publication();
    }
	
	const double time_shrink = 3.0;	//[s]
	const double time_clear = 5.0;	//[s]
	if(!grid.data.empty()){
		if(time_nomove>time_clear || first_callback_odom)	ClearObstacles();
	}

	first_callback_odom = false;
}

void OccupancyGridStore::MoveCells(double dt)
{
	theta += odom.twist.twist.angular.z*dt;
	theta = atan2(sin(theta), cos(theta));
	
	delta_x += odom.twist.twist.linear.x*dt * cos(theta);
	delta_y += odom.twist.twist.linear.x*dt * sin(theta);
	
	if(fabs(delta_x)>grid.info.resolution || fabs(delta_y)>grid.info.resolution){
		int delta_x_ = delta_x/grid.info.resolution;
		int delta_y_ = delta_y/grid.info.resolution;

		nav_msgs::OccupancyGrid tmp_grid = grid_all_minusone;
		for(size_t i=0;i<grid.data.size();i++){
			double x, y;
			IndexToMeterpoint(grid, i, x, y);
			x -= delta_x_*grid.info.resolution;
			y -= delta_y_*grid.info.resolution;	
			int index_moved = MeterpointToIndex(grid, x, y);
			if(MeterPointIsInside(grid, x, y))	tmp_grid.data[index_moved] = grid.data[i];
		}
		delta_x -= delta_x_*grid.info.resolution;
		delta_y -= delta_y_*grid.info.resolution;
		
		grid = tmp_grid;
	}

}

void OccupancyGridStore::ClearObstacles(void)
{
	const double range_meter = 2.3;	//[m]
	int range_cell = range_meter/grid.info.resolution;
	for(int i=-range_cell;i<=range_cell;i++){
		for(int j=-range_cell;j<=range_cell;j++){
			grid.data[PointToIndex(grid, i, j)] = 0;
		}
	}
}

int OccupancyGridStore::PointToIndex(nav_msgs::OccupancyGrid grid, int x, int y)
{
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

void OccupancyGridStore::IndexToMeterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y)
{
	x = (index%grid.info.width - grid.info.width/2.0 + 0.5)*grid.info.resolution;
	y = (index/grid.info.width - grid.info.height/2.0 + 0.5)*grid.info.resolution;
}

int OccupancyGridStore::MeterpointToIndex(nav_msgs::OccupancyGrid grid, double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

bool OccupancyGridStore::MeterPointIsInside(nav_msgs::OccupancyGrid grid, double x, double y)
{
	if(fabs(x)>grid.info.width*grid.info.resolution/2.0)	return false;
	if(fabs(y)>grid.info.height*grid.info.resolution/2.0)	return false;
	return true;
}

void OccupancyGridStore::Publication(void)
{
	grid.header.stamp = odom.header.stamp;
	pub.publish(grid);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_store");
	std::cout << "= occupancygrid_store =" << std::endl;

	OccupancyGridStore occupancygrid_store;

	ros::spin();
}
