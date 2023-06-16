#include "localmap_creator/hokuyo_raycast.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hokuyo_raycast");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    hokuyo_raycast::OccupancyGridLidar occupancygrid_lidar(nh, private_nh);

    occupancygrid_lidar.process();

    return 0;
}


