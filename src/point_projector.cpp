#include "point_projector.h"

std::vector<cv::Vec3f> point_projector::project_points(const std::vector<cv::Vec3f> &point_cloud) {
    if (intrinsics_matrix.cols > 3) {
        intrinsics_matrix = intrinsics_matrix(
                cv::Range(0, intrinsics_matrix.rows),
                cv::Range(0, intrinsics_matrix.cols - 1)
        );
    }

    auto projected_points = std::vector<cv::Vec3f>();
    for (const auto &point : point_cloud) {
        auto projective_point = cv::Mat(1, 4, CV_32F, {point.val[0],
                                          point.val[1],
                                          point.val[2],
                                          1});
        cv::Mat result = intrinsics_matrix * transform_matrix * projective_point;
        cv::Vec3f vector = result;
        projected_points.push_back(result);
    }
    return projected_points;
}
