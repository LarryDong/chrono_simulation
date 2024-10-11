#include <iomanip> // 为了使用std::setprecision

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <sstream>



std::string bag_filename = "/home/larrydong/clins_ws/data/kaist-urban-07.bag";
std::string output_filename = "/home/larrydong/Desktop/kaist.csv";
std::string imu_topic = "/imu/data";
std::string lidar_topic = "/velodyne_left_points";


struct PointWithRingTime : public pcl::PointXYZI {
    uint16_t ring;
    float time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointWithRingTime,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))



void processBagFile(const std::string &bag_file) {
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics = {imu_topic, lidar_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::ofstream output_file(output_filename);
    output_file << "ros_ts,  type,  timestamp,  min_time,  max_time\n";

    ros::Time bag_start_time = view.getBeginTime();

    for (const rosbag::MessageInstance &m : view) {
        if (m.getTopic() == lidar_topic) {
            sensor_msgs::PointCloud2::ConstPtr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc2 != nullptr) {
                pcl::PointCloud<PointWithRingTime> cloud;
                pcl::fromROSMsg(*pc2, cloud);

                float min_time = std::numeric_limits<float>::max();
                float max_time = std::numeric_limits<float>::min();
                for (const auto &point : cloud) {
                    min_time = std::min(min_time, point.time);
                    max_time = std::max(max_time, point.time);
                }

                // 使用 std::fixed 和 std::setprecision 控制浮点数的输出格式
                output_file << std::fixed << std::setprecision(3)
                            << (m.getTime() - bag_start_time).toSec() << ",  Lid,  "
                            << pc2->header.stamp.toSec()-bag_start_time.toSec() << ",  "
                            << min_time << ",  " << max_time << "\n";
            }
        } else if (m.getTopic() == imu_topic) {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != nullptr) {
                output_file << std::fixed << std::setprecision(3)
                            << (m.getTime() - bag_start_time).toSec() << ",  IMU,  "
                            << imu->header.stamp.toSec()-bag_start_time.toSec() << ",  0,  0\n";
            }
        }
    }

    output_file.close();
    bag.close();
}

// 主函数不变

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_processor");

    processBagFile(bag_filename);

    return 0;
}