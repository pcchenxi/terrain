#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "pointshape_processor.h"

tf::TransformListener* tfListener = NULL;

ros::Publisher  pub_continuity, pub_cross, pub_ground, pub_costmap, pub_ground_obstacle, pub_free;
Pointshape_Processor *ps_processor;

void publish(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> cloud, int type = 2)
{
    sensor_msgs::PointCloud2 pointlcoud2;
    pcl::toROSMsg(cloud, pointlcoud2);

    if(type == 2)
    {
        pub.publish(pointlcoud2);
    }
    else
    {
        sensor_msgs::PointCloud pointlcoud;
        sensor_msgs::convertPointCloud2ToPointCloud(pointlcoud2, pointlcoud);

        pointlcoud.header = pointlcoud2.header;
        pub.publish(pointlcoud);
    }

}


pcl::PointCloud<pcl::PointXYZRGB> generate_costmap_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::PointXYZRGB> &cloud_free)
{
    pcl::PointCloud<pcl::PointXYZRGB> obs_points;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        int r = cloud.points[i].r;
        pcl::PointXYZRGB point;
        point = cloud.points[i];

        if(r != 0 )
        {
            point.z = 0;
            obs_points.points.push_back(point);
        }
        else
        {
            point.z = 0;
            cloud_free.points.push_back(point);
        }
//        else
//        {
//            point.z = 0.0;
//            obs_points.points.push_back(point);
//        }
    }

    obs_points.header.frame_id = cloud.header.frame_id;

    return obs_points;
}



void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGB> filtered_single_scan;
    filtered_single_scan = ps_processor->process_velodyne(cloud_in, tfListener);
    filtered_single_scan.header.frame_id = "base_link";

    publish(pub_ground_obstacle, filtered_single_scan);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointshape_based_processor");

    ros::NodeHandle node;
 
    float cell_size = 0.2;
    node.getParam("/cell_size", cell_size);

    ps_processor = new Pointshape_Processor(360*4, cell_size);

    // ros::Subscriber sub_velodyne_right = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_right", 1, callback_velodyne);
    // ros::Subscriber sub_velodyne_left = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_left", 1, callback_velodyne);
    ros::Subscriber sub_velodyne_left  = node.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, callback_velodyne);

    // pub_continuity = node.advertise<sensor_msgs::PointCloud2>("/continuity_filtered", 1);
    // pub_cross      = node.advertise<sensor_msgs::PointCloud2>("/cross_section_filtered", 1);
    // pub_ground     = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points_ground", 1);
    pub_ground_obstacle = node.advertise<sensor_msgs::PointCloud2>("/ground_obstacle",1);
    // pub_free       = node.advertise<sensor_msgs::PointCloud2>("/velodyne_points_free",1);

    // pub_costmap    = node.advertise<sensor_msgs::PointCloud>("/costmap_cloud", 1);


    // ros::Subscriber sub_odom = node.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 1, callback_odom);
    // ros::Subscriber sub_odom_icp = node.subscribe<nav_msgs::Odometry >("/icp_odom", 1, callback_odom_icp);


    tfListener = new (tf::TransformListener);
    ros::spin();

    return 0;
}
