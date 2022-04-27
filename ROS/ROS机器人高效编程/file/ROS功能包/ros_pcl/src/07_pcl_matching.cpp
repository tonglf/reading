#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>

class cloudHandler {
public:
    cloudHandler() {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::clouCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
    }

    void clouCB(const sensor_msgs::PointCloud2::ConstPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

        pcl::fromROSMsg(*input, *cloud_in);

        *cloud_out = *cloud_in;

        for (size_t i = 0; i < cloud_in->size(); ++i)
        {
            cloud_in->at(i).x = cloud_in->at(i).x + 0.7f;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_out);
        icp.setInputTarget(cloud_in);

        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(*cloud_aligned);

        pcl::toROSMsg(*cloud_aligned, *output);
        output->header.frame_id = "odom";
        pcl_pub.publish(*output);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl_matching");

    cloudHandler handler;

    ros::spin();

    return 0;
}