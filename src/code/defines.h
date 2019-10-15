#pragma once

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>


namespace fs = boost::filesystem;

struct compare_point {
    bool operator()(const cv::Point &p1, const cv::Point &p2) const {
        if (p1.x < p2.x)
            return true;
        return p1.y < p2.y;
    }
};