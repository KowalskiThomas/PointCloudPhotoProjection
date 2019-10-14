cmake_minimum_required(VERSION 3.0)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS x"-O3")

find_package( OpenCV REQUIRED )
set(boost_min_ver 1.50.0)
set(boost_libs system filesystem)
find_package(Boost ${boost_min_ver})

if(Boost_FOUND)
    find_package(Boost ${boost_min_ver} COMPONENTS ${boost_libs})
endif()


add_executable(coucou coucou.cpp)
target_link_libraries(coucou ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_executable(tests test.cpp)
target_link_libraries(tests ${OpenCV_LIBS} ${Boost_LIBRARIES})
