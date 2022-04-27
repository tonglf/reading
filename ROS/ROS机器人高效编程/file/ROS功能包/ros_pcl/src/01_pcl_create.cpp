#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctime>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcl_create");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    cloud.width = 100;
    cloud.height = 1;
    cloud.resize(cloud.width * cloud.height);

    srand((unsigned)time(nullptr));

    for (size_t i = 0; i < cloud.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}