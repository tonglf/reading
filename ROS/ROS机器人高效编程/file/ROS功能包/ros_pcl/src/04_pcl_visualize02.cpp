#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

class cloudHandler
{  
public:
    cloudHandler() {
        output_sub = nh.subscribe("pcl_output", 1, &cloudHandler::outputCB, this);
        filtered_sub = nh.subscribe("pcl_filter", 1, &cloudHandler::filteredCB, this);
        downsampled_sub = nh.subscribe("pcl_downsampled", 1, &cloudHandler::downsampledCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

        viewer.createViewPort(0.0, 0.0,0.33, 1.0, output_view);
        viewer.setBackgroundColor(0, 0, 0, output_view);

        viewer.createViewPort(0.33, 0.0,0.66, 1.0, filtered_view);
        viewer.setBackgroundColor(0, 0, 0, filtered_view);

        viewer.createViewPort(0.66, 0.0,1.0, 1.0, downsampled_view);
        viewer.setBackgroundColor(0, 0, 0, downsampled_view);

        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
    }

    void outputCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        viewer.removeAllPointClouds(output_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "output", output_view);
    }

    void filteredCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        viewer.removeAllPointClouds(filtered_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "filtered", filtered_view);
    }

    void downsampledCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        viewer.removeAllPointClouds(downsampled_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, "downsampled", downsampled_view);
    }

    void timerCB(const ros::TimerEvent&)
    {
        viewer.spinOnce();
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

private:    
    ros::NodeHandle nh;
    ros::Subscriber output_sub, filtered_sub, downsampled_sub;
    pcl::visualization::PCLVisualizer viewer;
    int output_view, filtered_view, downsampled_view;
    ros::Timer viewer_timer;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcl_visualize2");

    cloudHandler handler;

    ros::spin();

    return 0;
}