#include "ros/ros.h"

#include "functions/continuity_filter.h"
#include "functions/cross_section_filter.h"
#include "functions/xi_functions.h"

#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>


tf::TransformListener* tfListener = NULL;
float sensor_height = 1.0;
int point_num_h = 360*4;

ros::Publisher  pub_continuity, pub_cross, pub_ground, pub_costmap, pub_ground_obstacle, pub_free;
Filter_Continuity       filter_continutiy(point_num_h);
Filter_Cross_Section    filter_crosssection(point_num_h);

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

void seperate_velodyne_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud
                                           , pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
                                           , Feature **feature_sets)
{

    float toDegree = 180/M_PI;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;

        if(x < 0)
            continue;

        float r = sqrt(x*x + y*y);
        float h = cloud.points[i].z;

        if(r > 20)
            continue;

        double angle_v = atan2(h, r) * toDegree + 15.0;
        double angle_h = (atan2(y, x) + M_PI) * toDegree;

        int index_v = angle_v/2.0;
        int index_h = angle_h/0.5;
        float mod = angle_v - (index_v * 2.0);
        if(mod > 1)
            index_v += 1;

        velodyne_sets[index_v].points[index_h] = cloud.points[i];
        feature_sets[index_v][index_h].radius = r;
    }
}

/**
 * @brief seperate_velodyne_cloud seprate points into 16 vectors sorted by azimuth (one for each altidue form Velodyne16)
 * @param cloud
 * @param cloud_transformed
 * @param velodyne_sets
 * @param feature_sets
 */
void seperate_velodyne_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud
                           , pcl::PointCloud<pcl::PointXYZRGB> cloud_transformed
                           , pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
                           , Feature **feature_sets
                           , pcl::PointCloud<pcl::PointXYZI> intensity_pcl)
{

    float toDegree = 180/M_PI;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;

       if(x < 1 || x > 5 || abs(y) > 5)
           continue;

    //    if(x <  0 || abs(y) > 0.5)
    //        continue;
        float r = sqrt(x*x + y*y);
        float h = cloud.points[i].z;

        if(r < 1.5)
            continue;

        float x_trans = cloud_transformed.points[i].x;
        float y_trans = cloud_transformed.points[i].y;
        float r_trans = sqrt(x_trans*x_trans + y_trans*y_trans);

        double angle_v = atan2(h, r) * toDegree + 15.0;
        double angle_h = (atan2(y, x) + M_PI) * toDegree;

        int index_v = angle_v/2.0;
        int index_h = angle_h/0.5;
        float mod = angle_v - (index_v * 2.0);
        if(mod > 1)
            index_v += 1;

        //16 x point_num_h
        if(index_v < 0) {
            //std::cerr << "index_v = " << index_v << std::endl;
            index_v = 0;
        }
        if(index_v > 15){
            //std::cerr << "index_v = " << index_v << std::endl;
            index_v = 15;
        }
        if(index_h < 0) {
            //std::cout << "index_h = " << index_h << std::endl;
            index_h = 0;
        }
        if(index_h > point_num_h-1) {
            //std::cout << "index_h = " << index_h << std::endl;
            index_h = point_num_h-1;
        }
        velodyne_sets[index_v].points[index_h] = cloud_transformed.points[i];
        feature_sets[index_v][index_h].radius = r_trans;
        feature_sets[index_v][index_h].intensity = intensity_pcl.points[i].intensity;
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
    sensor_msgs::PointCloud2 cloud_base;

    tf::StampedTransform velodyne_to_base;

    string target_name = "/base_link";
    // cout << cloud_in->header.frame_id << endl;
    //string target_name = "/odom";

    //tfListener->waitForTransform(target_name, cloud_in->header.frame_id, ros::Time::now(), ros::Duration(2.0));
    try {
        tfListener->lookupTransform(target_name, cloud_in->header.frame_id, ros::Time(0), velodyne_to_base);
    }catch(std::exception & e) {
       // ROS_WARN_STREAM("TF error! " << e.what());
        return ;
    }

    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (velodyne_to_base, eigen_transform);
    pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_base);

    cloud_base.header.frame_id = target_name;

    // initialize velodyne group space
    pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets[16];
    Feature **feature_sets = new Feature*[16];
    //pcl::PointCloud<pcl::PointXYZRGB> feature_sets[16];
    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i].points.resize(point_num_h);
        feature_sets[i] = new Feature[point_num_h];
        //feature_sets[i].points.resize(point_num_h);
    }

    for(int i = 0; i< 16; i++)
        for(int j = 0; j < point_num_h; j++)
        {
            feature_sets[i][j].radius = 0;
            feature_sets[i][j].continuity_prob = 0;
            feature_sets[i][j].cross_section_prob = 0;
            feature_sets[i][j].sum = 0;

            feature_sets[i][j].mean_height = 0;
            feature_sets[i][j].mean_slope = 0;
            feature_sets[i][j].height_variance = 0;
            feature_sets[i][j].vertical_acc = 0;
            feature_sets[i][j].roughness = 0;
            feature_sets[i][j].max_height_diff = 0;
            // feature_sets[i][j].is_selected = false;
        }

    // for velodyne frame, used for point classification
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_base;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_base_rgb, cloud_inlier;
    copyPointCloud(pcl_cloud, cloud);

    // for target frame, used for sending result to cost map
    pcl::fromROSMsg(cloud_base, pcl_cloud_base);
    copyPointCloud(pcl_cloud_base, cloud_base_rgb);
    cloud_base_rgb.header.frame_id = target_name;

    // get intensity value
    pcl::PointCloud<pcl::PointXYZI> intensity_pcl;
    pcl::fromROSMsg(*cloud_in,intensity_pcl);

    //seperate_velodyne_cloud(cloud, velodyne_sets, feature_sets);
    seperate_velodyne_cloud(cloud, cloud_base_rgb, velodyne_sets, feature_sets, intensity_pcl);

    pcl::PointCloud<pcl::PointXYZRGB> result, cloud_free, reformed_height;

    // for(int i = 0; i < point_num_h; i++)
    // {
    //     if(velodyne_sets[0][i].z != 0)
    //         cout << velodyne_sets[0][i].z*100/2.54 << endl;
    // }    
    // //////////////////////////////////////////////////////////////////////////////////////////////////
    feature_sets = filter_crosssection.filtering_all_sets(velodyne_sets, feature_sets);
    result = filter_crosssection.color_all_sets(velodyne_sets, feature_sets);

    feature_sets = filter_continutiy.filtering_all_sets(velodyne_sets, feature_sets);
    result       = filter_continutiy.color_all_sets(velodyne_sets, feature_sets, reformed_height);
    // //////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZRGB> costmap_cloud = generate_costmap_cloud(result, cloud_free);

    costmap_cloud.header.frame_id = target_name;
    cloud_free.header.frame_id = target_name;
    result.header.frame_id =  target_name;

    // cout << "frame: " <<result.header.frame_id << endl;
    // publish(pub_ground, costmap_cloud);
    // publish(pub_free, cloud_free);
    publish(pub_ground_obstacle, result);


    for(int i = 0; i<16; i++)
    {
        delete [] feature_sets[i];
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointshape_based_processor");

    ros::NodeHandle node;

    float cell_size = 0.2;
    node.getParam("/cell_size", cell_size);
    filter_continutiy.set_cell_size(cell_size);
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
