#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Image.h"
#include "image_projector.h"
#include "point_projector.h"
// #include "const_data.h"
#include "cv_bridge/cv_bridge.h"

cv::Mat image(1, 1, CV_32F);

std::vector<cv::Vec3f> convert_points(const sensor_msgs::PointCloud2& msg)
{
float r_data[] = {7.533745e-03, -9.999714e-01, -6.166020e-04, 0,
                  1.480249e-02, 7.280733e-04, -9.998902e-01, 0,
                  9.998621e-01, 7.523790e-03, 1.480755e-02, 0};

auto rotation_matrix = cv::Mat(3, 4, CV_32F, r_data);

float i_data[] = {7.215377e+02, 0.000000e+00, 6.095593e+02,
                  4.485728e+01, 0.000000e+00, 7.215377e+02,
                  1.728540e+02, 2.163791e-01, 0.000000e+00,
                  0.000000e+00, 1.000000e+00, 2.745884e-03};
auto intrinsics_matrix = cv::Mat(3, 4, CV_32F, i_data);

const float t_data [] = {-4.069766e-03, -7.631618e-02, -2.717806e-01};
auto translation = cv::Mat(3, 1, CV_32F);
for(size_t i = 0; i < 3; i++)
	translation.at<float>(i) = *(t_data + i);

	point_projector p_projector { rotation_matrix, translation, intrinsics_matrix };
	if (image.rows == 1)
		return {};

	auto result = std::vector<cv::Vec3f>{};
	auto in_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
	auto in_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
	auto in_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");

	for(; in_x != in_x.end() && in_y != in_y.end() && in_z != in_z.end(); ++in_x, ++in_y, ++in_z)
	{
		result.push_back(cv::Vec3f(*in_x, *in_y, *in_z));
	}

for(auto point : result)
{
//	std::cout << "UNPROJECTED" << point << std::endl;
}

	auto projected_points = p_projector.project_points(result);

//for(auto point : projected_points)
//	std::cout << "PROJECTED" << point << std::endl;

	std::cout << "FRAME" << std::endl;
	std::cout << result.back() << std::endl;
	std::cout << projected_points.back() << std::endl;

	auto projector = image_projector{};
	projector.project_image("/home/kowalski/Desktop/output.png", image, projected_points);

	return result;
}

void callback_images(const sensor_msgs::Image& img)
{
	std::cout << "Received image" << std::endl;
	auto image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	image_ptr->image.copyTo(image);
}

void callback(const sensor_msgs::PointCloud2& msg)
{
	std::cout << "Received points" << std::endl;
	auto cloud = convert_points(msg);
	std::cout << "PC Conversion Finished" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_projector");
  ros::NodeHandle n;
  ros::Subscriber sub_points = n.subscribe("points", 1000, callback);
  ros::Subscriber sub_images = n.subscribe("images", 1000, callback_images);
  ros::spin();

  return 0;
}
