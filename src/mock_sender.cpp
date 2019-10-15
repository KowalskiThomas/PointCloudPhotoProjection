#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Image.h"
#include "image_projector.h"
#include "point_projector.h"
#include "const_data.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char **argv)
{
  std::cout << "start" << std::endl;
  ros::init(argc, argv, "mock_sender");
  ros::NodeHandle n;
  ros::Publisher pub_images = n.advertise<sensor_msgs::Image>("images", 1000);
  ros::Publisher pub_clouds = n.advertise<sensor_msgs::PointCloud2>("points", 1000);
  ros::Rate loop_rate(10);

  int i = 0;
  while (ros::ok())
  {
	  ++i;
	  auto mat = cv::imread("/home/kowalski/Desktop/input.png");
	cv_bridge::CvImage image_data;
	image_data.encoding = sensor_msgs::image_encodings::BGR8;
	image_data.image = mat;

	pub_images.publish(image_data);
	ros::spinOnce();
	loop_rate.sleep();
	std::cout << "Published" << i << std::endl;

  }

  return 0;
}
