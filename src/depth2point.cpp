  
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <limits>
class Depth2Point 
{
     public:
        Depth2Point();
        void frameCallback(const sensor_msgs::Image::ConstPtr& msg);
        void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
		void depth_to_pointcloud(sensor_msgs::PointCloud2& cloud_msg);
		void publication();
     private:
        ros::NodeHandle nh;
        ros::Publisher pub_pcl;
        ros::Subscriber sub_image;
        ros::Subscriber sub_camera_info;

		sensor_msgs::CameraInfo cam;
		sensor_msgs::Image image;
		sensor_msgs::Image compressed_image;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl {new pcl::PointCloud<pcl::PointXYZ>()};
		sensor_msgs::PointCloud2 pc2;

		bool cam_flag = false;
		ros::Time pub_time;
		std::string pub_frameid;
};

Depth2Point::Depth2Point()
{
    sub_image = nh.subscribe<sensor_msgs::Image> ("/image",1, &Depth2Point::frameCallback, this);
    sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo> ("/camera_info",1, &Depth2Point::CameraInfoCallback, this);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 1);
}


void Depth2Point::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	// std::cout << "-----camera info call back-----" << std::endl;
	cam = *msg;
	cam_flag = true;
}


void Depth2Point::frameCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	// std::cout << "-----frame call back-----" << std::endl;
	image = *msg;
	std::cout << msg->encoding << std::endl;
	
	pub_frameid = msg->header.frame_id;
	pub_time = msg->header.stamp;
	

	// sensor_msgs::PointCloud2 pc2;
	if(cam_flag){
		depth_to_pointcloud(pc2);
	}

	publication();
}

void Depth2Point::depth_to_pointcloud(sensor_msgs::PointCloud2& cloud_msg)
{
	std::cout << "-----Depth to Point-----" << std::endl;


	cv_bridge::CvImagePtr depth_img_cv;
	cv::Mat depth_mat;
	// Get the ROS image to openCV
	try{
		depth_mat = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	// depth_img_cv = cv_bridge::toCvShare (image, sensor_msgs::image_encodings::TYPE_16UC1);
	// Convert the uints to floats
	// depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001);



	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(cam);

	double SCALING = 0.001;
	double FX = model.fx();
	double FY = model.fy();
	// std::cout << "FX:" << FX << ", FY:" << FY << std::endl;
	double FX_ = 1/FX;
	double FY_ = 1/FY;
	float CX = model.cx();
	float CY = model.cy();
	// std::cout << "CX:" << CX << ", CY:" << CY << std::endl;
	int height = image.height;
	int width = image.width;
	// std::cout << "height:" << height << ", width:" << width << std::endl;
	pcl->points.clear();
	for(int v=0; v<height; v++){
		for(int u=0; u<width; u++){
			// std::cout << v*width+u<< " < " << image.data.size() << std::endl;
			float d = float(image.data[v*width+u]*SCALING);
			// std::cout << "d = "<<d << std::endl;
			if(!d){
				continue;
			}
			pcl::PointXYZ pt;	
			pt.z = double(d);
			pt.x = (u-CX)*pt.z*FX_;
			pt.y = (v-CY)*pt.z*FY_;
			// std::cout << "z:" << double(pt.z) << ", x:" << pt.x << ", y:" << pt.y <<std::endl;
			pcl->points.push_back(pt);
		}
	}

	pcl::toROSMsg(*pcl, cloud_msg);
}


void Depth2Point::publication()
{
	pc2.header.stamp = pub_time;
	// pc2.header.frame_id = pub_frameid;
	pc2.header.frame_id = "base_link";
	pub_pcl.publish(pc2);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth2point");

    Depth2Point depth2point;

    ros::spin();

    return 0;
}
