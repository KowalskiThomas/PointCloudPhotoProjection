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

point_projector p_projector { rotation_matrix, translation, intrinsics_matrix };
cv::Mat image;

std::vector<cv::Vec3f> convert_points(const sensor_msgs::PointCloud2& msg)
{
	auto result = std::vector<cv::Vec3f>{};
	auto in_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
	auto in_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
	auto in_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");

	for(; in_x != in_x.end() && in_y != in_y.end() && in_z != in_z.end(); ++in_x, ++in_y, ++in_z)
	{
		result.emplace_back(cv::Vec3f(*in_x, *in_y, *in_z));
	}

	auto projected_points = p_projector.project_points(result);

	auto projector = image_projector{};
	projector.project_image("output.png", image, result);


	return result;
}

void callback_images(const sensor_msgs::Image& img)
{
	auto image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	image_ptr->image.copyTo(image);
}

void callback(const sensor_msgs::PointCloud2& msg)
{
	std::cout << "I GOT SOMETHING" << std::endl;
	auto cloud = convert_points(msg);
	std::cout << "PC Conversion Finished" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_projector");
  ros::NodeHandle n;
  ros::Subscriber sub_points = n.subscribe("image_points_couples", 1000, callback);
  ros::Subscriber sub_images = n.subscribe("images", 1000, callback_images);
  ros::spin();

  return 0;
}
