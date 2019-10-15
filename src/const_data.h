#include "defines.h"
#pragma once


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
