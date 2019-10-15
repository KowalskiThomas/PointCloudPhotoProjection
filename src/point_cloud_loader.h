#pragma once

#include "defines.h"


class point_cloud_loader {
private:
    fs::path path;

public:
    point_cloud_loader(fs::path path)
            : path(path) {
    }

    std::vector <cv::Vec3f> load() {
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
        auto point_cloud = std::vector < cv::Vec3f > {};
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

    static std::vector <cv::Vec3f> load(const fs::path &path);
};
