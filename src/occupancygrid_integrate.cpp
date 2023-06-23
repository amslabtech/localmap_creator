/**
 * @file occupancygrid_integrate.cpp
 * @brief C++ merged gird maps
 * @author AMSL
 */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

class OccupancyGridCombination{
    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        /*subscribe*/
        ros::Subscriber sub_grid_lidar;
        ros::Subscriber sub_grid_realsense;
        ros::Subscriber sub_grid_hokuyo;
        ros::Subscriber sub_expand_flag;
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
        bool expand_minimize_flag = false;
        /*time*/
        ros::Time time_odom_now;
        ros::Time time_odom_last;
        ros::Time time_pub;
        /*param*/
        double EXPAND_RANGE;
        double EXPAND_RANGE_MINI;
        double expand_range;

    public:
        /**
         * constructor
         */
        OccupancyGridCombination();
        /**
         * set expand flag
         */
        void expand_flag_callback(const std_msgs::BoolConstPtr& msg);
        /**
         * set grid map from Lidar
         */
        void grid_lidar_callback(const nav_msgs::OccupancyGridConstPtr& msg);
        /**
         * set grid map from realsense
         */
        void grid_realsense_callback(const nav_msgs::OccupancyGridConstPtr& msg);
        /**
         * set grid map from hokuyo
         */
        void grid_hokuyo_callback(const nav_msgs::OccupancyGridConstPtr& msg);
        /**
         * check whether cell is in map
         *
         * @param[in] x coordinate x
         * @param[in] y coordinate y
         */
        bool cell_is_inside(nav_msgs::OccupancyGrid &grid, int x, int y);
        /**
         * combine three grid maps
         */
        void combine_grids(void);
        /**
         * expand map
         */
        void expand(void);
        /**
         * set coordinate from index
         *
         * @param index grid map index
         * @param x map coordinate x
         * @param y map coordinate y
         */
        void index_to_point(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y);
        /**
         * set grid map index from obstacle coordinate
         *
         * @param[in] x obstacle coordinate x
         * @param[in] y obstacle coordinate y
         * @return int index of grid map
         */
        int point_to_index(nav_msgs::OccupancyGrid &grid, int x, int y);
        /**
         * publish merged grid map
         */
        void publication(void);
};

/**
 * constructor
 */
OccupancyGridCombination::OccupancyGridCombination()
    : private_nh("~")
{
    private_nh.param("EXPAND_RANGE", EXPAND_RANGE, {0.4});
    private_nh.param("EXPAND_RANGE_MINI", EXPAND_RANGE_MINI, {0.2});
    std::cout << "EXPAND_RANGE 		: " << EXPAND_RANGE << std::endl;
    std::cout << "EXPAND_RANGE_MINI : " << EXPAND_RANGE_MINI << std::endl;

    expand_range = EXPAND_RANGE;


    sub_grid_lidar = nh.subscribe("/occupancygrid/lidar/stored", 1,
            &OccupancyGridCombination::grid_lidar_callback, this);
    sub_grid_realsense = nh.subscribe("/occupancygrid/realsense", 1,
            &OccupancyGridCombination::grid_realsense_callback, this);
    sub_grid_hokuyo = nh.subscribe("/occupancygrid/hokuyo", 1,
            &OccupancyGridCombination::grid_hokuyo_callback, this);
    sub_expand_flag = nh.subscribe("/expand_minimize_flag", 1,
            &OccupancyGridCombination::expand_flag_callback, this);

    pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",1);
    pub_expand = nh.advertise<nav_msgs::OccupancyGrid>("/local_map/expand",1);
}

/**
 * set expand flag
 */
void OccupancyGridCombination::expand_flag_callback(const std_msgs::BoolConstPtr& msg)
{
    expand_minimize_flag = msg->data;
    std::cout << "ExpandFlag : " << expand_minimize_flag << std::endl;
    expand_range = EXPAND_RANGE;
    if(expand_minimize_flag)expand_range = EXPAND_RANGE_MINI;
}

/**
 * set grid map from Lidar
 *
 * set grid map from Lidar and combine grid maps and publish
 */(
void OccupancyGridCombination::grid_lidar_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // std::cout<<"grid_lidar_callback"<<std::endl;
    grid_lidar = *msg;

    /** initialize grid data to -1*/
    if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
        grid = *msg;
        grid_expand =*msg;
        size_t loop_lim = grid_realsense.data.size();
        for(size_t i=0;i<loop_lim;i++)	grid.data[i] = -1;
    }
    first_callback_grid_lidar = false;

    time_pub = msg->header.stamp;
    combine_grids();
    if(!grid.data.empty())	publication();
}

/**
 * set grid map from realsense
 */
void OccupancyGridCombination::grid_realsense_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // std::cout<<"grid_realsense_callback"<<std::endl;
    grid_realsense = *msg;

    /** initialize grid data to -1*/
    if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
        grid = *msg;
        grid_expand =*msg;
        size_t loop_lim = grid_lidar.data.size();
        for(size_t i=0;i<loop_lim;i++)	grid.data[i] = -1;
    }

    first_callback_grid_realsense = false;
}

/**
 * set grid map from hokuyo
 */
void OccupancyGridCombination::grid_hokuyo_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // std::cout<<"grid_hokuyo_callback"<<std::endl;
    grid_hokuyo = *msg;

    /** initialize grid data to -1*/
    if(first_callback_grid_lidar && first_callback_grid_realsense && first_callback_grid_hokuyo){
        grid = *msg;
        grid_expand =*msg;
        size_t loop_lim = grid_hokuyo.data.size();
        for(size_t i=0;i<loop_lim;i++)	grid.data[i] = -1;
    }

    first_callback_grid_hokuyo = false;
}

/**
 * expand map
 */
void OccupancyGridCombination::expand(void)
{
    int loop_lim = grid.data.size();
    double reso = grid.info.resolution;
    double reso_rec = 1/reso;
    int expand_range_int = int(expand_range*reso_rec);
    int Data = 0;
    for(size_t i=0;i<loop_lim;i++){
        if(grid.data[i]){
            Data = grid.data[i];
            int x, y;　
            index_to_point(grid, i, x, y);
            /**expand map to range limit */
            for(int j=-expand_range_int;j<=expand_range_int;j++){
                for(int k=-expand_range_int;k<=expand_range_int;k++){
                    if(cell_is_inside(grid, x+j, y+k)){
                        double distance = sqrt((x+j+0.5)*(x+j+0.5) + (y+k+0.5)*(y+k+0.5)); //distance on the expand map
                        distance *= reso;
                        if(distance > EXPAND_RANGE || Data==100){
                            if(sqrt(j*j+k*k)*reso<=expand_range){
                                if(grid_expand.data[point_to_index(grid, x+j,y+k)]<=Data){
                                    grid_expand.data[point_to_index(grid, x+j,y+k)] = Data;
                                }
                            }
                        }else if(grid_expand.data[point_to_index(grid, x+j,y+k)] != 100){
                            grid_expand.data[point_to_index(grid, x+j,y+k)] = 0;
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief combine three grid maps
 */
void OccupancyGridCombination::combine_grids(void)
{
    size_t loop_lim = grid.data.size();
    for(size_t i=0;i<loop_lim;i++){
        if(!first_callback_grid_lidar)grid.data[i] = grid_lidar.data[i]; //input grid data from lidar
        if(!first_callback_grid_realsense && grid_realsense.data[i]==100) grid.data[i] = grid_realsense.data[i]; //input grid data from realsense
        if(!first_callback_grid_hokuyo && grid.data[i]<=0) grid.data[i] = grid_hokuyo.data[i]; //input grid data from hokuyo
    }

    for(size_t i=0;i<loop_lim;i++){
        grid_expand.data[i] = grid.data[i];
    }
    expand();
}

/**
 * @brief set coordinate from index
 *
 * @param index grid map index
 * @param x map coordinate x
 * @param y map coordinate y
 */
void OccupancyGridCombination::index_to_point(nav_msgs::OccupancyGrid &grid, int index, int& x, int& y)
{
    x = index%grid.info.width - grid.info.width*0.5; //calculate point x
    y = index/grid.info.width - grid.info.height*0.5; //calculate point y
}

/**
 * @brief set grid map index from obstacle coordinate
 *
 * @param[in] x obstacle coordinate x
 * @param[in] y obstacle coordinate y
 * @return int index of grid map
 */
int OccupancyGridCombination::point_to_index(nav_msgs::OccupancyGrid &grid, int x, int y)
{
    int x_ = x + grid.info.width*0.5; //calculate index x
    int y_ = y + grid.info.height*0.5; //calculate index y
    return	y_*grid.info.width + x_; //calculate index
}

/**
 * @brief check whether cell is in map
 *
 * if cell is in map return true
 *
 * @param[in] x coordinate x
 * @param[in] y coordinate y
 */
bool OccupancyGridCombination::cell_is_inside(nav_msgs::OccupancyGrid &grid, int x, int y)
{
    int w = grid.info.width;
    int h = grid.info.height;
    if(x<-w*0.5)  return false;
    if(x>w*0.5-1) return false;
    if(y<-h*0.5)  return false;
    if(y>h*0.5-1) return false;
    return true;
}

/**
 * @brief publish merged grid map
 */
void OccupancyGridCombination::publication(void)
{
    // std::cout<<"publication"<<std::endl;
    grid.header.stamp = time_pub;
    grid_expand.header.stamp = time_pub;
    pub.publish(grid);
    pub_expand.publish(grid_expand);
}

/**
 * @brief main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancygrid_integrate");
    std::cout << "= occupancygrid_integrate =" << std::endl;

    OccupancyGridCombination occupancygrid_combine;

    ros::spin();
}
