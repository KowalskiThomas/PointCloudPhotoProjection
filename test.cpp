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

cv::Mat load_image(const fs::path &path) {
    auto image = cv::imread(path.string().c_str());
    assert(!image.empty());
    return image;
}

std::vector<cv::Vec3f> load_point_cloud(const fs::path &path) {
    // TODO: Make an okay version of this function

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    int *data = (int *) malloc(num * sizeof(int));

    // pointers
    int *px = data + 0;
    int *py = data + 1;
    int *pz = data + 2;
    int *pr = data + 3;

    // load point cloud
    FILE *stream;
    std::cout << path << std::endl;
    stream = fopen(path.string().c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    auto point_cloud = std::vector<cv::Vec3f>{};
    for (int32_t i = 0; i < num; i++) {
        point_cloud.emplace_back(cv::Vec3f(*px, *py, *pz));
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
    }
    fclose(stream);

    assert(!point_cloud.empty());
    return point_cloud;
}

std::vector<cv::Vec3f> project_points(const cv::Mat &image, const std::vector<cv::Vec3f> &point_cloud) {
    ///////// FIRST GET THE CALIBRATION RIGHT ///////
    float r_data[] = {7.533745e-03, -9.999714e-01, -6.166020e-04, 0,
                      1.480249e-02, 7.280733e-04, -9.998902e-01, 0,
                      9.998621e-01, 7.523790e-03, 1.480755e-02, 0,
                      0, 0, 0, 0};
    auto rotation_matrix = cv::Mat(4, 4, CV_32F, r_data);

    auto translation = cv::Mat(std::vector<float>{-4.069766e-03, -7.631618e-02, -2.717806e-01}).reshape(1, 3);
    for (auto i = 0; i < translation.cols; i++)
        rotation_matrix.at<float>(i, 3) = translation.at<float>(i);

    //////// NOW DO THE PROJECTION STUFF ////////
    float i_data[] = {7.215377e+02, 0.000000e+00, 6.095593e+02,
                      4.485728e+01, 0.000000e+00, 7.215377e+02,
                      1.728540e+02, 2.163791e-01, 0.000000e+00,
                      0.000000e+00, 1.000000e+00, 2.745884e-03};
    auto intrinsics_matrix = cv::Mat(3, 4, CV_32F, i_data);
    intrinsics_matrix(
            cv::Range(0, intrinsics_matrix.rows - 1),
            cv::Range(0, intrinsics_matrix.cols)
    );

    auto projected_points = std::vector<cv::Vec3f>();
    for (const auto &point : point_cloud) {
        auto projective_point = cv::Vec4f(point.val[0],
                                          point.val[1],
                                          point.val[2],
                                          1);
        cv::Mat result = intrinsics_matrix * rotation_matrix * projective_point;
        cv::Vec3f vector = result;
        projected_points.push_back(result);
    }
    return projected_points;
}

constexpr const bool prune_points = true;
constexpr const size_t point_rate = 2;

void project_image(const cv::Mat &image, std::vector<cv::Vec3f> points) {
    if constexpr (prune_points) {
        auto selected_points = decltype(points){};
        for (size_t i = 0; i < points.size(); i++) {
            if (i % point_rate == 0)
                selected_points.push_back(points[i]);
        }
        points = selected_points;
    }

/*    for(auto point : points)
    {
        std::cout << point << std::endl;
    }*/

    // Remove points behind us
    points.erase(std::remove_if(points.begin(), points.end(), [](decltype(points[0]) point) {
                     return point.val[2] < 3;
                 }),
                 points.end());
    // Divide points by their third coordinate
    std::transform(points.begin(), points.end(), points.begin(), [](decltype(points[0]) point) {
        return cv::Vec3f (
                point.val[0] / point.val[2],
                point.val[1] / point.val[2],
                point.val[2]);
    });

    // Get maximum third coordinate and create the colors
    int max_z;
    auto result = std::max_element(points.begin(), points.end(),
                                   [](decltype(points[0]) &pt1, decltype(points[0]) &pt2) {
                                       return pt1.val[2] < pt2.val[2];
                                   });
    if (result != points.end())
        max_z = (*result).val[2];

    std::transform(points.begin(), points.end(), points.begin(), [](decltype(points[0]) &point) {
        auto z = point.val[2];
        auto new_z = std::sqrt(std::abs(z));
        return new_z;
    });

    auto colors = std::map<cv::Point, cv::Vec3b, compare_point>{};
    std::for_each(points.begin(), points.end(), [&colors, max_z](decltype(points[0]) point) {
        auto z = point.val[2];
        auto colour = cv::Vec3b{1,
                                static_cast<unsigned char>(std::min(1, static_cast<int>(
                                                                               static_cast<float>(std::abs(z - 3)) /
                                                                               max_z) % 30)),
                                0};
        auto key = cv::Point{static_cast<int>(point.val[0]), static_cast<int>(point.val[1])};
        colors.insert(std::make_pair(key, colour));
    });

    for (auto &pair : colors) {
        std::cout << pair.first.x << " / " << pair.first.y << " : " << pair.second << std::endl;
    }

    auto output = image.clone();
    for (int i = 0; i < output.rows; i++) {
        for (int j = 0; j < output.cols; j++) {
            if (colors.find({i, j}) != colors.end()) {
                output.at<cv::Vec3b>(cv::Point{i, j}) = colors[{i, j}];
            }
        }
    }
    cv::imwrite("output.png", output);
}

void create_animation() {
    auto path = fs::path{"/Users/kowalski/Downloads/LIDAR/2011_09_26 2/2011_09_26_drive_0005_sync/"};
    auto points_path = path / "/velodyne_points/data/";
    auto images_path = path / "/image_03/data/";

    auto iterator = fs::directory_iterator(points_path);
    std::vector<fs::path> binary_files;
    for (; iterator != fs::directory_iterator{}; iterator++) {
        auto file = *iterator;
        auto path_string = file.path().string();
        if (path_string.find(".bin") != -1)
            binary_files.push_back(file);
    }

    for (const auto &file_path : binary_files) {
        auto file_name = file_path.filename();
        auto image_file = file_name.replace_extension("png");
        auto image_path = images_path / image_file;
        auto image = load_image(image_path);
        auto point_cloud = load_point_cloud(file_path);
        auto projected_cloud = project_points(image, point_cloud);
        project_image(image, projected_cloud);
    }
}

int main() {
    create_animation();
    return 0;
}
