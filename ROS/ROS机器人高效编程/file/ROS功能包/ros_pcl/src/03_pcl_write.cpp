#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::io::savePCDFileASCII("/home/tong/文档/ROS-2022.4/FileSystem/src/ros_pcl/data/write.pcd", *cloud);
}

int main(int argc,  char* argv[])
{
    ros::init(argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe<sensor_msgs::PointCloud2>("pcl_output", 10, cloudCB);

    ros::spin();

    return 0;
}