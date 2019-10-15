#pragma once

#include "defines.h"

class animator {
    fs::path path;

public:
    animator(fs::path path)
        : path(path)
    {
    }

    void create_animation(fs::path output_directory);

};

