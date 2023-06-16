#include "localmap_creator/hokuyo_raycast.h"

namespace hokuyo_raycast
{
    OccupancyGridLidar::OccupancyGridLidar(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param("hz", param_.hz, 10);
        private_nh.param("map_width", param_.map_width, 20.0);
        private_nh.param("map_height", param_.map_height, 20.0);
        private_nh.param("map_resolution", param_.map_resolution, 1.0);

        pub = nh.advertise<nav_msgs::OccupancyGrid>("/OccupancyGridLidar/Lidar", 1);
    }

    void OccupancyGridLidar::grid_initialization()
    {
        grid.info.resolution = param_.map_resolution;
        grid.info.width = param_.map_width / param_.map_resolution;
        grid.info.height = param_.map_height / param_.map_resolution;
        grid.info.origin.position.x = -param_.map_width / 2.0;
        grid.info.origin.position.y = -param_.map_height / 2.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
    }

    void OccupancyGridLidar::init_map()
    {
        grid.data.clear();

        const int size = grid.info.width * grid.info.height;
        for(int i=0; i<size; i++)
        {
            grid.data.push_back(-1);
        }
    }

    void OccupancyGridLidar::input_grid()
    {
        size_t loop_lim = rmground->points.size();

        for(size_t i=0; i<loop_lim; i++)
        {
            const int grid_index = xy_to_index();
        }

    }

    int OccupancyGridLidar::xy_to_index(double x, double y)
    {
        int x_ = x/grid.info.resolution + grid.info.width/2;
    }

    void OccupancyGridLidar::publication()
    {
        grid.header.frame_id = pub_frameid;
        grid.header.stamp = pub_stamp;
        pub.publish(grid);
    }

    void OccupancyGridLidar::process()
    {
        ros::Rate loop_rate(param_.hz);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

    }
}
