#pragma once

#include "defines.h"

class point_projector {
    cv::Mat transform_matrix;
    cv::Mat intrinsics_matrix;;

public:
    point_projector(cv::Mat rotation_matrix, cv::Mat translation, cv::Mat intr) {
        transform_matrix = cv::Mat(3, 4, CV_32F);

        intr.copyTo(intrinsics_matrix);

        for (auto i = 0; i < 3; i++)
            for (auto j = 0; j < 3; j++)
                transform_matrix.at<float>(i, j) = rotation_matrix.at<float>(i, j);

        for (auto i = 0; i < translation.rows; i++)
            transform_matrix.at<float>(i, 3) = translation.at<float>(i);
    }

    std::vector <cv::Vec3f> project_points(const std::vector <cv::Vec3f> &point_cloud);
};
