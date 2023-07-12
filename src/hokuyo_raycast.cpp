/**
 * @file hokuyo_raycast.cpp
 * @brief C++ make grid map from lidar data
 * @author AMSL
 */
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

namespace hokuyo_raycast
{
    /**
     * @brief class for making occupancy grid map from lidar
     */
    class OccupancyGridLidar{
        private:
            /*NodeHandle*/
            ros::NodeHandle nh;
            ros::NodeHandle private_nh;
            /*subscribe*/
            ros::Subscriber obstacle_sub_;
            /*publish*/
            ros::Publisher grid_map_pub_;
            /*cloud*/
            pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
            /*grid*/
            nav_msgs::OccupancyGrid grid;
            nav_msgs::OccupancyGrid grid_all_zero;
            /*publish infomations*/
            std::string pub_frameid;
            ros::Time pub_stamp;
            /*const values*/
            double width;	//x[m]
            double height;	//y[m]
            double resolution;	//[m]
                                // const double range_road_intensity[2] = {5, 15};
        public:
            /**
             * @brief Constructor
             */
            OccupancyGridLidar();
            /**
             * @brief initialize grid map
             */
            void grid_initialization(void);
            /**
             * @brief set obstacle data
             */
            void rm_ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
            /**
             * @brief set obstacle data
             */
            void ground_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
            /**
             * @brief limited search range
             */
            void exstract_pc_in_range(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc);
            /**
             * @brief input obstacle information to grid map
             */
            void input_grid(void);
            /**
             * @brief set grid map index from obstacle coordinate
             *
             * @param[in] x obstacle coordinate x
             * @param[in] y obstacle coordinate y
             * @return int index of grid map
             */
            int meterpoint_to_index(double x, double y);
            /**
             * @brief publish grid map
             */
            void publication(void);
    };


    /**
     * @brief Constructor
     */
    OccupancyGridLidar::OccupancyGridLidar()
        : private_nh("~")
    {
        private_nh.param("resolution", resolution, {0.1});
        private_nh.param("width", width, {20.0});
        private_nh.param("height", height, {20.0});

        std::cout << "resolution : " << resolution << std::endl;
        std::cout << "width          : " << width << std::endl;
        std::cout << "height          : " << height << std::endl;

        obstacle_sub_ = nh.subscribe("/hokuyo_points", 1, &OccupancyGridLidar::rm_ground_callback, this);
        grid_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar", 1);
        grid_initialization();
    }

    /**
     * @brief initialize grid map
     */
    void OccupancyGridLidar::grid_initialization(void)/*{{{*/
    {
        grid.info.resolution = resolution;
        grid.info.width = width/resolution + 1;
        grid.info.height = height/resolution + 1;
        grid.info.origin.position.x = -width*0.5;
        grid.info.origin.position.y = -height*0.5;
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

    /**
     * @brief set obstacle data
     *
     * recive obstacle data and make gridmap and publish
     */
    void OccupancyGridLidar::rm_ground_callback(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
    {
        pcl::fromROSMsg(*msg, *rmground);

        exstract_pc_in_range(rmground);

        pub_frameid = msg->header.frame_id;
        pub_stamp = msg->header.stamp;

        input_grid();
        publication();
    }/*}}}*/

    /**
     * @brief limited search range
     */
    void OccupancyGridLidar::exstract_pc_in_range(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc)/*{{{*/
    {
        pcl::PassThrough<pcl::PointXYZI> pass; //class methods to pass through all data  that satisfies the user given constraints.
        /**
         * limite x range
         */
        pass.setInputCloud(pc);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-width*0.5, width*0.5);
        pass.filter(*pc);
        /**
         * limite y range
         */
        pass.setInputCloud(pc);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-height*0.5, height*0.5);
        pass.filter(*pc);
    }/*}}}*/

    /**
     * @brief input obstacle information to grid map
     */
    void OccupancyGridLidar::input_grid(void)
    {
        grid = grid_all_zero;
        size_t loop_lim = rmground->points.size();
        for(size_t i=0;i<loop_lim;i++){
            grid.data[meterpoint_to_index(rmground->points[i].x, rmground->points[i].y)] = 100; //input onstacle coordinate in map
        }
    }

    /**
     * @brief set grid map index from obstacle coordinate
     *
     * @param[in] x obstacle coordinate x
     * @param[in] y obstacle coordinate y
     * @return int index of grid map
     */
    int OccupancyGridLidar::meterpoint_to_index(double x, double y)
    {
        int x_ = x/grid.info.resolution + grid.info.width*0.5; //calculate index x
        int y_ = y/grid.info.resolution + grid.info.height*0.5; //calculate index y
        int index = y_*grid.info.width + x_;
        return index;
    }

    /**
     * @brief publish grid map
     */
    void OccupancyGridLidar::publication(void)
    {
        grid.header.frame_id = pub_frameid; //publish tf frame
        grid.header.stamp = pub_stamp; //publish time stamp
        grid_map_pub_.publish(grid); //publish grid map
    }
}

/**
 * @brief main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hokuyo_raycast");
    std::cout << "= hokuyo_raycast =" << std::endl;

    hokuyo_raycast::OccupancyGridLidar occupancygrid_lidar;

    ros::spin();
}
