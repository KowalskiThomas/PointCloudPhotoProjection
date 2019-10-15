# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(thomas)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "-O3")

find_package(OpenCV REQUIRED)

set(boost_min_ver 1.50.0)
set(boost_libs system filesystem)
find_package(Boost ${boost_min_ver} REQUIRED)

if(Boost_FOUND)
    find_package(Boost ${boost_min_ver} COMPONENTS ${boost_libs})
endif()

add_executable(render
        src/render.cpp
        src/defines.h
        src/point_cloud_loader.cpp
        src/point_cloud_loader.h
        src/image_projector.cpp
        src/image_projector.h
        src/point_projector.cpp
        src/point_projector.h
        src/animator.cpp
        src/animator.h
        src/image_loader.cpp
        src/image_loader.h
        src/const_data.h)
target_link_libraries(render ${OpenCV_LIBS} ${Boost_LIBRARIES})

# add_executable(tests tests.cpp defines.h)
# target_link_libraries(tests ${OpenCV_LIBS} ${Boost_LIBRARIES})


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener
        src/listener.cpp
        src/defines.h
        src/point_cloud_loader.cpp
        src/point_cloud_loader.h
        src/image_projector.cpp
        src/image_projector.h
        src/point_projector.cpp
        src/point_projector.h
        src/image_loader.cpp
        src/image_loader.h
        src/const_data.h)

target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(mock_sender
        src/mock_sender.cpp
        src/defines.h
        src/point_cloud_loader.cpp
        src/point_cloud_loader.h
        src/image_projector.cpp
        src/image_projector.h
        src/point_projector.cpp
        src/point_projector.h
        src/image_loader.cpp
        src/image_loader.h
        src/const_data.h)

target_link_libraries(mock_sender ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(mock_sender beginner_tutorials_generate_messages_cpp)







# %EndTag(FULLTEXT)%

