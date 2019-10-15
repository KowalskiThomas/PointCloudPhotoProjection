#include "point_projector.h"

std::vector <cv::Vec3f> point_projector::project_points(const std::vector <cv::Vec3f> &point_cloud) {
    if (intrinsics_matrix.cols > 3) {
        intrinsics_matrix = intrinsics_matrix(
                cv::Range(0, intrinsics_matrix.rows),
                cv::Range(0, intrinsics_matrix.cols - 1)
        );
    }

    auto projected_points = std::vector<cv::Vec3f>();
    for (const auto &point : point_cloud) {
        auto projective_point = cv::Mat(4, 1, CV_32F);
        for (int i = 0; i < 3; i++) {
            projective_point.at<float>(i, 0) = point[i];
        }

        projective_point.at<float>(3) = 1;
        cv::Mat result = intrinsics_matrix * transform_matrix * projective_point;
        assert(result.rows == 3);
        assert(result.cols == 1);
        cv::Vec3f vector{result.at<float>(0), result.at<float>(1), result.at<float>(2)};
        projected_points.push_back(vector);
    }
    return projected_points;
}
