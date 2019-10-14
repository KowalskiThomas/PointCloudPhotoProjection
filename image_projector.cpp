#include "image_projector.h"

void image_projector::project_image(fs::path file_name, const cv::Mat &image, std::vector<cv::Vec3f> points) {
    if constexpr (prune_points) {
        auto selected_points = decltype(points){};
        for (size_t i = 0; i < points.size(); i++) {
            if (i % point_rate == 0)
                selected_points.push_back(points[i]);
        }
        points = selected_points;
    }

    // Remove points behind us
    points.erase(std::remove_if(points.begin(), points.end(), [](decltype(points[0]) point) {
                     return point.val[2] < 3;
                 }),
                 points.end());

    // Divide points by their third coordinate
    std::transform(points.begin(), points.end(), points.begin(), [](decltype(points[0]) point) {
        point.val[0] = point.val[0] / point.val[2],
        point.val[1] = point.val[1] / point.val[2],
        point.val[2] = point.val[2];
        return point;
    });

    // Get maximum third coordinate and create the colors
    float max_z = 0;
    auto result = std::max_element(points.begin(), points.end(),
                                   [](decltype(points[0]) &pt1, decltype(points[0]) &pt2) {
                                       return pt1.val[2] <= pt2.val[2];
                                   });
    assert(result != points.end());
    max_z = (*result).val[2];

    std::transform(points.begin(), points.end(), points.begin(), [](decltype(points[0]) &point) {
        auto z = point.val[2];
        auto new_z = std::sqrt(std::abs(z));
        point.val[2] = new_z;
        return point;
    });

    auto colors = std::map<cv::Point, cv::Vec3b, compare_point>{};
    std::for_each(points.begin(), points.end(), [&colors, max_z](decltype(points[0]) point) {
        float z = point.val[2];
        auto blue = 30 * 255 * std::abs(z - 3) / max_z;
        auto colour = cv::Vec3b(0,
                                cv::saturate_cast<unsigned char>(blue),
                                255);
        auto key = cv::Point{static_cast<int>(point.val[0]), static_cast<int>(point.val[1])};
        colors.insert(std::make_pair(key, colour));
    });

    auto output = image.clone();
    for (auto &pair : colors) {
        if (pair.first.x < 0 || pair.first.x >= output.cols)
            continue;

        if (pair.first.y < 0 || pair.first.y >= output.rows)
            continue;

        output.at<cv::Vec3b>(pair.first.y, pair.first.x) = pair.second;
    }
    auto output_path = fs::path{"output"} / file_name.replace_extension("png");
    cv::imwrite(output_path.string(), output);
}