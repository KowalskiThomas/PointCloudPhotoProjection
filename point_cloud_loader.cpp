#include "point_cloud_loader.h"

std::vector<cv::Vec3f> point_cloud_loader::load(const fs::path& path) {
    auto loader = point_cloud_loader{path};
    return loader.load();
}