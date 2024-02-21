#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/time.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pack_data_to_rosbag");
    ROS_INFO("--> Create rosbag from chrono output.");
    ros::NodeHandle nh;

    std::string base_path = "/home/larrydong/Desktop/chrono_output/";
    std::string lidar_folder = base_path + "lidar/";
    std::string imu_path = base_path + "imu.csv";
    std::string gt_path = base_path + "gt.csv";

    std::string output_bag_filename = "/home/larrydong/Desktop/output.bag";

    rosbag::Bag bag;
    bag.open(output_bag_filename, rosbag::bagmode::Write);

    ros::Time begin_time = ros::Time::now();

    ROS_INFO_STREAM("Input folder: " << base_path);
    ROS_INFO_STREAM("Output file: " << output_bag_filename);

    // Load and write Lidar data
    float lidar_frequency = 10.0;           // lidar 10Hz.
    int lidar_scan_counter = 0;
    for (int i = 0;; ++i) {
        std::string file_name = lidar_folder + "frame_" + std::to_string(i) + ".csv";
        std::ifstream lidar_file(file_name);
        if (!lidar_file) {
            // ROS_WARN_STREAM("Cannot find lidar file: " << file_name);
            ROS_INFO_STREAM("Total Lidar scan: " << i);
            ROS_WARN_STREAM("Estimated simulation time: " << i / 10.0f << "s.");
            break;
        }
        pcl::PointCloud<pcl::PointXYZI> cloud;
        std::string line;
        std::getline(lidar_file, line); // Skip the first empty line
        while (std::getline(lidar_file, line)) {
            std::istringstream iss(line);
            std::string token;
            pcl::PointXYZI point;
            std::getline(iss, token, ','); point.x = std::stof(token);
            std::getline(iss, token, ','); point.y = std::stof(token);
            std::getline(iss, token, ','); point.z = std::stof(token);
            std::getline(iss, token, ','); point.intensity = std::stof(token);
            cloud.push_back(point);
        }
        lidar_scan_counter++;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "chrono";
        output.header.stamp = begin_time + ros::Duration(i * 1.0 / lidar_frequency);
        bag.write("/lidar", ros::Time::now(), output);
    }
    ROS_INFO_STREAM("<-- Write Lidar scans: " << lidar_scan_counter);

    // Load and write IMU data
    std::ifstream imu_file(imu_path);
    if(!imu_file.is_open()){
        cout <<"Error. Cannot open IMU file: "<< imu_path << endl;
        std::abort();
    }
    int imu_counter = 0;
    std::string line;
    std::getline(imu_file, line); // Skip header
    while (std::getline(imu_file, line)) {
        std::istringstream iss(line);
        sensor_msgs::Imu imu_msg;
        std::string token;
        std::getline(iss, token, ','); 
        double dt = std::stod(token);
        imu_msg.header.stamp = begin_time + ros::Duration(dt);
        imu_msg.header.frame_id = "chrono";
        std::getline(iss, token, ','); imu_msg.linear_acceleration.x = std::stod(token);
        std::getline(iss, token, ','); imu_msg.linear_acceleration.y = std::stod(token);
        std::getline(iss, token, ','); imu_msg.linear_acceleration.z = std::stod(token);
        std::getline(iss, token, ','); imu_msg.angular_velocity.x = std::stod(token);
        std::getline(iss, token, ','); imu_msg.angular_velocity.y = std::stod(token);
        std::getline(iss, token, ','); imu_msg.angular_velocity.z = std::stod(token);
        imu_counter++;
        bag.write("/imu", imu_msg.header.stamp, imu_msg);
    }
    ROS_INFO_STREAM("<-- Write IMU data: " << imu_counter);

    // Read and pack GT data
    std::ifstream gt_file(gt_path);
    std::getline(gt_file, line); // Skip header
    size_t gt_counter = 0;
    while (std::getline(gt_file, line)) {
        std::istringstream iss(line);
        nav_msgs::Odometry odom_msg;
        std::string token;
        std::getline(iss, token, ','); 
        double dt = std::stod(token);
        odom_msg.header.stamp = begin_time + ros::Duration(dt);
        odom_msg.header.frame_id = "chrono";
        std::getline(iss, token, ','); odom_msg.pose.pose.position.x = std::stod(token);
        std::getline(iss, token, ','); odom_msg.pose.pose.position.y = std::stod(token);
        std::getline(iss, token, ','); odom_msg.pose.pose.position.z = std::stod(token);
        std::getline(iss, token, ','); double qw;
        std::getline(iss, token, ','); double qx;
        std::getline(iss, token, ','); double qy;
        std::getline(iss, token, ','); double qz;
        odom_msg.pose.pose.orientation.w = qw;
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        gt_counter++;
        bag.write("/gt", odom_msg.header.stamp, odom_msg);
    }
    ROS_INFO_STREAM("<-- Write Gt pose: " << gt_counter);

    bag.close();
    ROS_WARN_STREAM("<== Saved bag into: " << output_bag_filename);
    return 0;
}

