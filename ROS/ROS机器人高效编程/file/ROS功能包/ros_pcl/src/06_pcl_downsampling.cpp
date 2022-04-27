#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class cloudHandler {
public:
    cloudHandler() {
        pcl_sub = nh.subscribe("pcl_filter", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

        pcl::fromROSMsg(*input, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud);
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(*cloud_downsampled);

        pcl::toROSMsg(*cloud_downsampled, *output);
        pcl_pub.publish(*output);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pcl_downsampling");

    cloudHandler handler;

    ros::spin();

    return 0;
}