#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler {
public:
    cloudHandler() {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_plane", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

        pcl::fromROSMsg(input, *cloud);

        cloud->width = 500;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->resize(cloud->width * cloud->height);

        std::vector<int> inliers;

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();
        ransac.getInliers(inliers);

        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
        pcl::toROSMsg(*final, *output);
        pcl_pub.publish(output);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcl_model_estimation");

    cloudHandler handler;

    ros::spin();

    return 0;
}