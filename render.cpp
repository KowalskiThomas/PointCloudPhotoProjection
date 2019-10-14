#include "defines.h"
#include "animator.h"

int main() {
    auto anim = animator("/Users/kowalski/Desktop/Imperial/Projet/projection/data");
    anim.create_animation("output");
    return 0;
}
