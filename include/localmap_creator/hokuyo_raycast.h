#ifndef HOKUYO_RAYCAST_H
#define HOKUYO_RAYCAST_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <tf/tf.h>

namespace hokuyo_raycast
{
    struct Param
    {
        int hz;
        double map_width;
        double map_height;
        double map_resolution;
    };


    class OccupancyGridLidar
    {
        public:
            OccupancyGridLidar(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void grid_initialization();
            void init_map();
            void input_grid();
            int xy_to_index(double x, double y);
            void publication();
            void process();

        private:
            //subscriber
            ros::Subscriber sub;

            //publisher
            ros::Publisher pub;

            //cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};

            //grid
            nav_msgs::OccupancyGrid grid;

            //publish information
            std::string pub_frameid;
            ros::Time pub_stamp;

            Param param_;

    };
}
#endif
