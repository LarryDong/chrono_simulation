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


// convert to velodyne point format.
struct VelodynePoint{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))


#define LIDAR_FREQUENCY (10)
#define PI (3.1415926f)
#define OUTPUT_SCAN_LINE (32)         // how many line output.
const double g_fov_top = 16.0;
const double g_fov_bottom = -16.0;


int main(int argc, char** argv) {
    ros::init(argc, argv, "pack_data_to_rosbag");
    ROS_INFO("--> Create rosbag from chrono output.");
    ros::NodeHandle nh("~");

    std::string base_path = "default";
    std::string output_bag_filename = "default";

    nh.getParam("input_folder", base_path);
    nh.getParam("output_file", output_bag_filename);

    std::string lidar_folder = base_path + "lidar/";
    std::string imu_path = base_path + "imu.csv";
    std::string gt_path = base_path + "gt.csv";


    rosbag::Bag bag;
    bag.open(output_bag_filename, rosbag::bagmode::Write);

    ros::Time begin_time = ros::Time::now();
    double init_wait_time = 3.0;            // wait 3s to start recording.

    ROS_INFO_STREAM("Input folder: " << base_path);
    ROS_INFO_STREAM("Lidar folder: " << lidar_folder);
    ROS_INFO_STREAM("IMU file: " << imu_path);
    ROS_INFO_STREAM("GT file: " << gt_path);
    ROS_INFO_STREAM("Output file: " << output_bag_filename);
    ROS_INFO_STREAM("Wait time: " << init_wait_time <<" s");


    // Load and write Lidar data
    int lidar_scan_counter = 0;
    const double vertical_angle_resolution = (g_fov_top - g_fov_bottom)/(OUTPUT_SCAN_LINE-1);
    for (int i = 0;; ++i) {
        // skip init time.
        double dt = i * 1.0 / LIDAR_FREQUENCY;
        if(dt < init_wait_time)
            continue;

        std::string file_name = lidar_folder + "frame_" + std::to_string(i) + ".csv";
        std::ifstream lidar_file(file_name);
        if (!lidar_file) {
            // ROS_WARN_STREAM("Cannot find lidar file: " << file_name);
            ROS_INFO_STREAM("Total Lidar scan: " << i);
            ROS_WARN_STREAM("Estimated simulation time: " << i / 10.0f << "s.");
            break;
        }
        
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointCloud<VelodynePoint> velo_cloud;

        std::string line;
        std::getline(lidar_file, line); // Skip the first empty line
        while (std::getline(lidar_file, line)) {
            std::istringstream iss(line);
            std::string token;
            // pcl::PointXYZI point;
            VelodynePoint vp;
            double x,y,z,intensity;

            std::getline(iss, token, ','); x = std::stof(token);
            std::getline(iss, token, ','); y = std::stof(token);
            std::getline(iss, token, ','); z = std::stof(token);
            std::getline(iss, token, ','); intensity = std::stof(token);

            if(abs(x)<0.01 || abs(y) < 0.01 || std::isnan(x) || std::isnan(y) || isnan(z))            // skip too-near points.
                continue;

            vp.x = x;
            vp.y = y;
            vp.z = z;
            vp.intensity = intensity;
            const double rad2deg = 180 / PI;
            double pitch = atan2(z, sqrt(x * x + y * y)) * rad2deg;
            int ring = int((pitch - g_fov_bottom) / vertical_angle_resolution);
            // cout << "i: " << i << ", x: " << x << ", y: " << y << ", z: " << z << ", pitch: " << pitch << ", ring" << ring << endl;
            assert(ring >= 0 && ring < OUTPUT_SCAN_LINE);
            vp.ring = ring;
            vp.time = 0.0;           // no offset time.

            // cloud.push_back(point);
            velo_cloud.push_back(vp);
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(velo_cloud, output);
        output.header.frame_id = "chrono";
        lidar_scan_counter++;

        output.header.stamp = begin_time + ros::Duration(dt);
        bag.write("/lidar", output.header.stamp, output);
    }
    ROS_INFO_STREAM("<-- Write Lidar scans: " << lidar_scan_counter);



    // Load and write IMU data

    std::ifstream imu_file(imu_path);
    if(!imu_file.is_open()){
        ROS_ERROR_STREAM("Error. Cannot open IMU file: "<< imu_path);
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

        if(dt < init_wait_time)     // skip init time;
            continue;

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
    if(!gt_file.is_open()){
        ROS_ERROR_STREAM("Error. Cannot open GT file: "<< gt_path);
        std::abort();
    }
    std::getline(gt_file, line); // Skip header
    size_t gt_counter = 0;
    while (std::getline(gt_file, line)) {
        std::istringstream iss(line);
        nav_msgs::Odometry odom_msg;
        std::string token;
        std::getline(iss, token, ','); 
        double dt = std::stod(token);

        if(dt<init_wait_time)
            continue;

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

