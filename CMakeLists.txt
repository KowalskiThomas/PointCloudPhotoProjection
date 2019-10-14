cmake_minimum_required(VERSION 3.0)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3")

find_package(OpenCV REQUIRED)

set(boost_min_ver 1.50.0)
set(boost_libs system filesystem)
find_package(Boost ${boost_min_ver} REQUIRED)

if(Boost_FOUND)
    find_package(Boost ${boost_min_ver} COMPONENTS ${boost_libs})
endif()

add_executable(render render.cpp defines.h point_cloud_loader.cpp point_cloud_loader.h)
target_link_libraries(render ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(tests tests.cpp defines.h)
target_link_libraries(tests ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(stream stream.cpp defines.h)
target_link_libraries(stream ${OpenCV_LIBS} ${Boost_LIBRARIES})
