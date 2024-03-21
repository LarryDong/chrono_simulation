cmake_minimum_required(VERSION 3.0.2)
project(create_rosbag)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    pcl_ros
)

catkin_package()
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_directories()

add_executable(create_bag src/main.cpp)
target_link_libraries(create_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

