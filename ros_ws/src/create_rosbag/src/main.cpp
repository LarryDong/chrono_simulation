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


// define Ouster32 point
struct OusterPoint{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t  ring;      
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)



#define LIDAR_FREQUENCY (10.0f)
#define PI (3.1415926535f)
#define OUTPUT_SCAN_LINE (32)         // how many line output.
#define POINT_PER_LINE (1800)

const double g_fov_top = 16.0;
const double g_fov_bottom = -15.0;


double g_vlp_time[OUTPUT_SCAN_LINE][POINT_PER_LINE];


void create_vlp_time(void){
    double dt_between_ring = 1.0f / LIDAR_FREQUENCY / OUTPUT_SCAN_LINE;
    double dt_in_ring = dt_between_ring * 1.0f / POINT_PER_LINE;
    for (unsigned int h = 0; h < OUTPUT_SCAN_LINE; h++) {
        for (unsigned int w = 0; w < POINT_PER_LINE; w++) {
            double offset_time = h * dt_between_ring + w * dt_in_ring;
            g_vlp_time[h][w] = offset_time;
        }
    }
    ROS_INFO_STREAM("Create point time. dt_ring: " << dt_between_ring << ", dt_point: " << dt_in_ring);
}

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
    double init_wait_time = 5.0;            // wait 5s to start recording.

    ROS_INFO_STREAM("Input folder: " << base_path);
    ROS_INFO_STREAM("Lidar folder: " << lidar_folder);
    ROS_INFO_STREAM("IMU file: " << imu_path);
    ROS_INFO_STREAM("GT file: " << gt_path);
    ROS_INFO_STREAM("Output file: " << output_bag_filename);
    ROS_INFO_STREAM("Wait time: " << init_wait_time <<" s");

    create_vlp_time();

    // Load and write Lidar data
    int lidar_scan_counter = 0;
    const double vertical_angle_resolution = (g_fov_top - g_fov_bottom)/(OUTPUT_SCAN_LINE-1);
    double debug_cnt = 0;
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
        
        std::string line;
        std::getline(lidar_file, line); // Skip the first empty line
        int counter = 0;
        OusterPoint vp, vp_old;
        int skip_counter = 0;

        std::vector<OusterPoint> v_pc_rings[OUTPUT_SCAN_LINE];   // use N lines to save each ring first, and then save them to a new point cloud.

        int line_cnt = 0;
        while (std::getline(lidar_file, line)) {
            line_cnt++;
            std::istringstream iss(line);
            std::string token;
            // pcl::PointXYZI point;
            double x,y,z,intensity;

            std::getline(iss, token, ','); x = std::stof(token);
            std::getline(iss, token, ','); y = std::stof(token);
            std::getline(iss, token, ','); z = std::stof(token);
            std::getline(iss, token, ','); intensity = std::stof(token);

            if(abs(x)<0.01 || abs(y) < 0.01 || std::isnan(x) || std::isnan(y) || isnan(z)){            // skip too-near points.
                // ROS_INFO_STREAM("idx: "<< i <<", Skip counter: " << skip_counter++ );
                continue;
            }
            else{
                vp.x = x;
                vp.y = y;
                vp.z = z;
                vp.intensity = intensity;
                const double rad2deg = 180 / PI;
                double pitch = atan2(z, sqrt(x * x + y * y)) * rad2deg;
                int ring = int((pitch - g_fov_bottom) / vertical_angle_resolution + 0.5);
                // ROS_INFO_STREAM("ring: " << ring <<", pitch: " << pitch << ", g_fov_bottom: " << g_fov_bottom);
                if(!(ring >=0 && ring < OUTPUT_SCAN_LINE)){
                    ROS_WARN_STREAM("Invalid ring: " << ring << ", line id: "<< line_cnt << ", pitch: " << pitch << ", g_fov_bottom: " << g_fov_bottom);
                    continue;
                }
                assert(ring >= 0 && ring < OUTPUT_SCAN_LINE);
                vp.ring = ring;
                
                // For chrono simulation, the rotation: (x-, y=0) -> (x=0, y-) -> (x+, y=0) -> (x=0, y+)
                // atan2: from [-pi, pi] during the rotation.
                double angle = atan2(y, x);
                if(angle < 0)
                    angle += 2*PI;

                int in_ring_idx = int (angle / (2*PI) * POINT_PER_LINE + 0.5);
                vp.t = g_vlp_time[ring][in_ring_idx];

                v_pc_rings[ring].push_back(vp);  // first save points into rings, then sort.
            }
        }

        // merge each ring into the full pc;
        pcl::PointCloud<OusterPoint> velo_cloud;
        // sort each ring's timestamp to avoid CHRONO simulation issue.
        for(int r=0; r<OUTPUT_SCAN_LINE; ++r){
            auto &v_ring = v_pc_rings[r];
            // sort(v_ring.begin(), v_ring.end(), [](const VelodynePoint& p1, const VelodynePoint& p2){    // sort by yaw-angle
            //     return p1.time < p2.time;
            // });
            sort(v_ring.begin(), v_ring.end(), [](const OusterPoint& p1, const OusterPoint& p2){    // sort by yaw-angle
                return p1.t < p2.t;
            });
            for(auto p : v_ring){
                velo_cloud.push_back(p);        // save into clouds.
            }
        }

        debug_cnt = 0;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(velo_cloud, output);
        output.header.frame_id = "chrono";
        lidar_scan_counter++;

        output.header.stamp = begin_time + ros::Duration(dt);
        bag.write("/ouster/points", output.header.stamp, output);
        // bag.write("/ouster/points", output.header.stamp+ros::Duration(0.1), output);
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
        // add cov
        std::vector<double> cov{1,0,0,0,1,0,0,0,1};
        std::copy(cov.begin(), cov.end(), imu_msg.linear_acceleration_covariance.begin());
        std::copy(cov.begin(), cov.end(), imu_msg.angular_velocity_covariance.begin());
        imu_counter++;

        // // TODO: disable IMU
        // ROS_WARN("Now diable IMU");
        // imu_msg.linear_acceleration.x = 0;
        // imu_msg.linear_acceleration.y = 0;
        // imu_msg.linear_acceleration.z = 9.8;
        // imu_msg.angular_velocity.x = 0;
        // imu_msg.angular_velocity.y= 0;
        // imu_msg.angular_velocity.z = 0;

        // bag.write("/ouster/imu", imu_msg.header.stamp+ros::Duration(0.01), imu_msg);
        bag.write("/ouster/imu", imu_msg.header.stamp, imu_msg);
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

