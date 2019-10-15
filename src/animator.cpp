#include "animator.h"
#include "point_projector.h"
#include "const_data.h"
#include "image_loader.h"
#include "point_cloud_loader.h"
#include "image_projector.h"

void animator::create_animation(fs::path output_directory) {
    auto points_path = path / "points/data";
    auto images_path = path / "images/data";

    auto iterator = fs::directory_iterator(points_path);
    std::vector <fs::path> binary_files;
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
        auto image = image_loader::load_image(image_path);
        auto point_cloud = point_cloud_loader::load(file_path);
        auto projected_cloud = projector.project_points(point_cloud);
        image_projector::project_image(file_name, image, projected_cloud);
        std::cout << "Done with " << file_path << std::endl;
    }
    std::cout << "ALL DONE" << std::endl;
}
