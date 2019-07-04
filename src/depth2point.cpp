  
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <limits>
class Depth2Point 
{
     public:
        Depth2Point();
        void frameCallback(const sensor_msgs::Image::ConstPtr& msg);
        void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		void depth_to_pointcloud();
		void publication();
     private:
        ros::NodeHandle nh;
        ros::Publisher pub_pcl;
        ros::Subscriber sub_image;
        ros::Subscriber sub_camera_info;

		sensor_msgs::CameraInfo cam;
		sensor_msgs::Image image;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl {new pcl::PointCloud<pcl::PointXYZ>()};
		sensor_msgs::PointCloud2 pc2;

		bool cam_flag = false;
		ros::Time pub_time;
};

Depth2Point::Depth2Point()
{
    sub_image = nh.subscribe<sensor_msgs::Image> ("/image",1, &Depth2Point::frameCallback, this);
    sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo> ("/camera_info",1, &Depth2Point::CameraInfoCallback, this);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 1);
}


void Depth2Point::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	cam = *msg;
	cam_flag = true;
}


void Depth2Point::frameCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	image = *msg;
	pc2.header.frame_id = msg->header.frame_id;
	pub_time = msg->header.stamp;
	if(cam_flag){
		depth_to_pointcloud();
	}
	publication();
}

void Depth2Point::depth_to_pointcloud()
{
	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(cam);

	float center_x = model.cx();
	float center_y = model.cy();
	
	//encoding 16UC1
	double unit_scaling = 0.001f;
	float constant_x = unit_scaling / model.fx();
	float constant_y = unit_scaling / model.fy();
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	sensor_msgs::PointCloud2Iterator<float> iter_x(pc2, "x");	
	sensor_msgs::PointCloud2Iterator<float> iter_y(pc2, "y");	
	sensor_msgs::PointCloud2Iterator<float> iter_z(pc2, "z");	

	const uint16_t* depth_row = reinterpret_cast<uint16_t*>(image.data[0]); 

	int row_step = image.step / sizeof(uint16_t);
	int height =  static_cast<int>(image.height);
	int width =  static_cast<int>(image.width);
	double range_max = 0.0;

	for(int v=0; v < height; ++v, depth_row += row_step){
		for(int u=0; u < width; ++u, ++iter_x, ++iter_y, ++iter_z){
			uint32_t depth = depth_row[u];

			if(!depth){
				if(range_max != 0.0){
					depth = (range_max * 1000.0f) + 0.5f;
				}else{
					*iter_x = *iter_y = *iter_z = bad_point;
					continue;
				}
			}

			*iter_x = (u - center_x) * depth * constant_x;
			*iter_y = (v - center_y) * depth * constant_y;
			*iter_z = depth * 0.001f;
		}
	}

}


void Depth2Point::publication()
{
	pc2.header.stamp = pub_time;
	pub_pcl.publish(pc2);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth2point");

    Depth2Point depth2point;

    ros::spin();

    return 0;
}
