#include "defines.h"

// TODO: Move that to an image_loader class
cv::Mat load_image(const fs::path &path) {
    auto image = cv::imread(path.c_str());
    assert(!image.empty());
    return image;
}

// TODO: Move that to a point_cloud_loader class
std::vector<cv::Vec3f> load_point_cloud(const fs::path &path) {
    // TODO: Make an okay version of this function

    int32_t num = 1000000;
    float *data = (float *) malloc(num * sizeof(float));

    // pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    // load point cloud
    FILE *stream;
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

// TODO: Move that to a point_projector class
std::vector<cv::Vec3f> project_points(const cv::Mat &image, const std::vector<cv::Vec3f> &point_cloud) {
    ///////// FIRST GET THE CALIBRATION RIGHT ///////
    float r_data[] = {7.533745e-03, -9.999714e-01, -6.166020e-04, 0,
                      1.480249e-02, 7.280733e-04, -9.998902e-01, 0,
                      9.998621e-01, 7.523790e-03, 1.480755e-02, 0};

    auto rotation_matrix = cv::Mat(3, 4, CV_32F, r_data);

    auto translation = cv::Mat(std::vector<float>{-4.069766e-03, -7.631618e-02, -2.717806e-01}).reshape(1, 3);
    for (auto i = 0; i < translation.rows; i++)
        rotation_matrix.at<float>(i, 3) = translation.at<float>(i);


    //////// NOW DO THE PROJECTION STUFF ////////
    float i_data[] = {7.215377e+02, 0.000000e+00, 6.095593e+02,
                      4.485728e+01, 0.000000e+00, 7.215377e+02,
                      1.728540e+02, 2.163791e-01, 0.000000e+00,
                      0.000000e+00, 1.000000e+00, 2.745884e-03};
    auto intrinsics_matrix = cv::Mat(3, 4, CV_32F, i_data);
    intrinsics_matrix = intrinsics_matrix(
            cv::Range(0, intrinsics_matrix.rows),
            cv::Range(0, intrinsics_matrix.cols - 1)
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

// TODO: Move that to an image_renderer class
void project_image(fs::path file_name, const cv::Mat &image, std::vector<cv::Vec3f> points) {
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

// TODO: Move that to an animator class
void create_animation() {
    auto path = fs::path{"/Users/kowalski/Downloads/LIDAR/2011_09_26 2/2011_09_26_drive_0005_sync/"};
    auto points_path = path / "/velodyne_points/data/";
    auto images_path = path / "/image_02/data/";

    auto iterator = fs::directory_iterator(points_path);
    std::vector<fs::path> binary_files;
    for (; iterator != fs::directory_iterator{}; iterator++) {
        auto file = *iterator;
        auto path_string = file.path().string();
        if (path_string.find(".bin") != -1)
            binary_files.push_back(file);
    }

    std::sort(binary_files.begin(), binary_files.end());
    for (const auto &file_path : binary_files) {
        auto file_name = file_path.filename();
        auto image_file = file_name.replace_extension("png");
        auto image_path = images_path / image_file;
        auto image = load_image(image_path);
        auto point_cloud = load_point_cloud(file_path);
        auto projected_cloud = project_points(image, point_cloud);
        project_image(file_name, image, projected_cloud);
        std::cout << file_path << std::endl;
    }
}

int main() {
    create_animation();
    return 0;
}
