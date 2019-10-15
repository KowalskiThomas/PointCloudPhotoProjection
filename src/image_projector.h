#pragma once

#include "defines.h"

class image_projector {
    static constexpr const bool prune_points = false;
    static constexpr const size_t point_rate = 2;

public:
    // TODO: Make that return an image instead of writing it to disk
    static void project_image(fs::path file_name, const cv::Mat &image, std::vector <cv::Vec3f> points);
};

