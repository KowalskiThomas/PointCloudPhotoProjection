#include "defines.h"
#include "point_cloud_loader.h"
#include "image_projector.h"
#include "point_projector.h"

float r_data[] = {7.533745e-03, -9.999714e-01, -6.166020e-04, 0,
                  1.480249e-02, 7.280733e-04, -9.998902e-01, 0,
                  9.998621e-01, 7.523790e-03, 1.480755e-02, 0};

auto rotation_matrix = cv::Mat(3, 4, CV_32F, r_data);

float i_data[] = {7.215377e+02, 0.000000e+00, 6.095593e+02,
                  4.485728e+01, 0.000000e+00, 7.215377e+02,
                  1.728540e+02, 2.163791e-01, 0.000000e+00,
                  0.000000e+00, 1.000000e+00, 2.745884e-03};
auto intrinsics_matrix = cv::Mat(3, 4, CV_32F, i_data);

auto translation = cv::Mat(std::vector<float>{-4.069766e-03, -7.631618e-02, -2.717806e-01}).reshape(1, 3);


// TODO: Move that to an image_loader class
cv::Mat load_image(const fs::path &path) {
    auto image = cv::imread(path.c_str());
    assert(!image.empty());
    return image;
}

// TODO: Move that to an animator class
void create_animation() {
    auto path = fs::path{"/Users/kowalski/Desktop/Imperial/Projet/projection/data"};
    auto points_path = path / "points/data";
    auto images_path = path / "images/data";

    auto iterator = fs::directory_iterator(points_path);
    std::vector<fs::path> binary_files;
    for (; iterator != fs::directory_iterator{}; iterator++) {
        auto file = *iterator;
        auto path_string = file.path().string();
        if (path_string.find(".bin") != -1)
            binary_files.push_back(file);
    }

    auto projector = point_projector{rotation_matrix, translation, intrinsics_matrix};

    std::sort(binary_files.begin(), binary_files.end());
    for (const auto &file_path : binary_files) {
        auto file_name = file_path.filename();
        auto image_file = file_name.replace_extension("png");
        auto image_path = images_path / image_file;
        auto image = load_image(image_path);
        auto point_cloud = point_cloud_loader::load(file_path);
        auto projected_cloud = projector.project_points(image, point_cloud);
        image_projector::project_image(file_name, image, projected_cloud);
        std::cout << file_path << std::endl;
    }
}

int main() {
    create_animation();
    return 0;
}
