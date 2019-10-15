#include "point_cloud_loader.h"
#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

#include "defines.h"

class pcl_loader {
	public:
	static pcl::PointCloud<pcl::PointXYZ> load_from_file(const fs::path& path)
	{
		auto cloud = pcl::PointCloud<pcl::PointXYZ>{};
		auto all_points = point_cloud_loader::load(path);
		for(auto& point : all_points)
		{
			cloud.points.emplace_back(pcl::PointXYZ{*(point.val), *(point.val + 1), *(point.val + 2)});
		}
		return cloud; //_points;
	}
};
