#include <iostream>
#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler {
public:
    cloudHandler(): viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("pcl_output", 10, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0, 1), &cloudHandler::timerCB, this);
    }

    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);
        viewer.showCloud(cloud);
    }

    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcl_visualize");

    cloudHandler handler;

    ros::spin();

    return 0;
}