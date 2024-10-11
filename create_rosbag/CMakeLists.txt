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

add_executable(time_export src/bag_time_export.cpp)
target_link_libraries(time_export ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(create_bag_noisy src/create_bag_with_noise.cpp)
target_link_libraries(create_bag_noisy ${catkin_LIBRARIES} ${PCL_LIBRARIES})
