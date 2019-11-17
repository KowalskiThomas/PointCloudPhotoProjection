#include "defines.h"
#include "animator.h"

int main(int argc, char** argv) {
    std::string path = "/Users/kowalski/Desktop/Imperial/Projet/projection/data";
    if (argc > 1)
        path = argv[1];
    
    std::cout << "Taking images and points from " << path << std::endl;
    
    if (!fs::exists(path)) {
        std::cerr << "The directory doesn't exist!" << std::endl;
        return 1;
    }

    auto anim = animator(path);
    anim.create_animation("output");
    return 0;
}
