#pragma once

#include "defines.h"

class image_loader {
public:
    static cv::Mat load_image(const fs::path &path) {
        auto image = cv::imread(path.c_str());
        assert(!image.empty());
        return image;
    }
};
