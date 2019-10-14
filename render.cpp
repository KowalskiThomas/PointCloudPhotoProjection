#include "defines.h"
#include "animator.h"

int main() {
    if (!fs::is_directory("output"))
        fs::create_directories("output");
    assert(fs::is_directory("output"));
    auto anim = animator("/Users/kowalski/Desktop/Imperial/Projet/projection/data");
    anim.create_animation("output");
    return 0;
}
