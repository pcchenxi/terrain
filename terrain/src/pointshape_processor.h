
#include "functions/continuity_filter.h"
#include "functions/cross_section_filter.h"
#include "functions/xi_functions.h"

#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>

int point_num_h      = 360*4;;
Filter_Continuity filter_continutiy(point_num_h);
Filter_Cross_Section filter_crosssection(point_num_h);

class Pointshape_Processor
{
    public:
        float m_cell_size;

        string base_frame_;
        Feature *cloud_feature;

        pcl::PointCloud<pcl::PointXYZRGB> frontp_roughness, velodyne_cost;
        
        Pointshape_Processor(int point_num, float cell_size);
        ~Pointshape_Processor();
        void seperate_velodyne_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud
                           , pcl::PointCloud<pcl::PointXYZRGB> cloud_base
                           , pcl::PointCloud<pcl::PointXYZRGB> cloud_map
                           , pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
                           , Feature **feature_sets
                           , pcl::PointCloud<pcl::PointXYZI> intensity_pcl);

        pcl::PointCloud<pcl::PointXYZRGB> process_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in, tf::TransformListener* tfListener);

        void get_robotlocation(float &robot_x, float &robot_y, tf::TransformListener* tfListener);

};

Pointshape_Processor::Pointshape_Processor(int point_num, float cell_size)
{
    m_cell_size = cell_size;
    base_frame_ = "base_link";
    point_num_h = point_num;
    cloud_feature = new Feature[16*point_num_h];
}

Pointshape_Processor::~Pointshape_Processor()
{
    delete [] cloud_feature;
}
/**
 * @brief seperate_velodyne_cloud seprate points into 16 vectors sorted by azimuth (one for each altidue form Velodyne16)
 * @param cloud
 * @param cloud_transformed
 * @param velodyne_sets
 * @param feature_sets
 */
void Pointshape_Processor::seperate_velodyne_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud
                           , pcl::PointCloud<pcl::PointXYZRGB> cloud_base
                           , pcl::PointCloud<pcl::PointXYZRGB> cloud_map
                           , pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
                           , Feature **feature_sets
                           , pcl::PointCloud<pcl::PointXYZI> intensity_pcl)
{
    filter_continutiy.set_cell_size(m_cell_size);
    float toDegree = 180/M_PI;

    for(int i = 0; i<cloud.points.size(); i++)
    {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;

       if(x < 0 || x > 8 || abs(y) > 8)
           continue;

    //    if(x <  0 || abs(y) > 2)
    //        continue;
        float r = sqrt(x*x + y*y);
        float h = cloud.points[i].z;

        if(r < 1.5 || r > 30)
            continue;

        // float x_trans = cloud_transformed.points[i].x;
        // float y_trans = cloud_transformed.points[i].y;
        // float r_trans = sqrt(x_trans*x_trans + y_trans*y_trans);

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
        velodyne_sets[index_v].points[index_h] = cloud_base.points[i];

        feature_sets[index_v][index_h].map_x = cloud_map.points[i].x;
        feature_sets[index_v][index_h].map_y = cloud_map.points[i].y;
        feature_sets[index_v][index_h].map_z = cloud_map.points[i].z;

        feature_sets[index_v][index_h].radius = r;
        feature_sets[index_v][index_h].intensity = intensity_pcl.points[i].intensity;
    }
}

void Pointshape_Processor::get_robotlocation(float &robot_x, float &robot_y, tf::TransformListener* tfListener)
{
    robot_x = 0, robot_y = 0;
    tf::StampedTransform transform;
    try
    {
        //ROS_INFO("Attempting to read pose...");
        tfListener->lookupTransform("/map","/base_link",ros::Time(0), transform);
        robot_x = transform.getOrigin().x();
        robot_y = transform.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("error while reading map fram! %s", ex.what());
    } 
}



pcl::PointCloud<pcl::PointXYZRGB> Pointshape_Processor::process_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in, tf::TransformListener* tfListener)
{
    pcl::PointCloud<pcl::PointXYZRGB> result, cloud_free, reformed_height;
    sensor_msgs::PointCloud2 cloud_base, cloud_map;

    tf::StampedTransform velodyne_to_base, velodyne_to_map;
    bool map_avaiable = true;

    // base_frame_ = "/base_link";
    // cout << cloud_in->header.frame_id << endl;
    //string base_frame_ = "/odom";

    //tfListener->waitForTransform(base_frame_, cloud_in->header.frame_id, ros::Time::now(), ros::Duration(2.0));
    try {
        tfListener->lookupTransform(base_frame_, cloud_in->header.frame_id, ros::Time(0), velodyne_to_base);
    }catch(std::exception & e) {
       ROS_WARN_STREAM("TF error! " << e.what());
        return result;
    }

    try {
        tfListener->lookupTransform("map", cloud_in->header.frame_id, ros::Time(0), velodyne_to_map);
    }catch(std::exception & e) {
        map_avaiable = false;
        ROS_WARN_STREAM("TF error! " << e.what());
    }

    if(map_avaiable)
    {
        Eigen::Matrix4f eigen_transform_map;
        pcl_ros::transformAsMatrix (velodyne_to_map, eigen_transform_map);
        pcl_ros::transformPointCloud (eigen_transform_map, *cloud_in, cloud_map);
        cloud_map.header.frame_id = "map";
    }
    // else
    {
        Eigen::Matrix4f eigen_transform;
        pcl_ros::transformAsMatrix (velodyne_to_base, eigen_transform);
        pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_base);
        cloud_base.header.frame_id = base_frame_;
    }

    // initialize velodyne group space
    pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets[16];
    Feature **feature_sets = new Feature*[16];

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
            feature_sets[i][j].roughness = -1;
            // feature_sets[i][j].is_selected = false;
        }

    // for velodyne frame, used for point classification
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_base, pcl_cloud_map;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_base_rgb, cloud_map_rgb, cloud_inlier;
    copyPointCloud(pcl_cloud, cloud);

    // for target frame, used for sending result to cost map
    if(map_avaiable)
    {
        pcl::fromROSMsg(cloud_map, pcl_cloud_map);
        copyPointCloud(pcl_cloud_map, cloud_map_rgb);
        cloud_map_rgb.header.frame_id = "map";
    }
    // else
    {
        pcl::fromROSMsg(cloud_base, pcl_cloud_base);
        copyPointCloud(pcl_cloud_base, cloud_base_rgb);
        cloud_base_rgb.header.frame_id = base_frame_;
    }

    // get intensity value
    pcl::PointCloud<pcl::PointXYZI> intensity_pcl;
    pcl::fromROSMsg(*cloud_in,intensity_pcl);

    //seperate_velodyne_cloud(cloud, velodyne_sets, feature_sets);
    if(map_avaiable)
        seperate_velodyne_cloud(cloud, cloud_base_rgb, cloud_map_rgb, velodyne_sets, feature_sets, intensity_pcl);
    else
        seperate_velodyne_cloud(cloud, cloud_base_rgb, cloud_base_rgb, velodyne_sets, feature_sets, intensity_pcl);


    // //////////////////////////////////////////////////////////////////////////////////////////////////
    feature_sets        = filter_crosssection.filtering_all_sets(velodyne_sets, feature_sets);
    result              = filter_crosssection.color_all_sets(velodyne_sets, feature_sets);

    feature_sets        = filter_continutiy.filtering_all_sets(velodyne_sets, feature_sets, cloud_in->header.stamp);
    result              = filter_continutiy.color_all_sets(velodyne_sets, feature_sets, cloud_feature); // all points with z as height

    frontp_roughness    = filter_continutiy.frontp_roughness;   // selected points infront of the robot to show the cost value
    velodyne_cost       = filter_continutiy.velodyne_cost;      // all points with z as cost 
    // //////////////////////////////////////////////////////////////////////////////////////////////////

    // pcl::PointCloud<pcl::PointXYZRGB> costmap_cloud = generate_costmap_cloud(result, cloud_free);

    // costmap_cloud.header.frame_id = base_frame_;
    // cloud_free.header.frame_id = base_frame_;

    if(map_avaiable)
    {    
        frontp_roughness.header.frame_id = "map";
        result.header.frame_id =  "map";
        velodyne_cost.header.frame_id = "map";
    }
    else
    {    
        frontp_roughness.header.frame_id = base_frame_;
        result.header.frame_id =  base_frame_;
        velodyne_cost.header.frame_id = base_frame_;
    }


    // cout << "frame: " <<result.header.frame_id << endl;
    // publish(pub_ground, costmap_cloud);
    // publish(pub_free, cloud_free);
    // publish(pub_ground_obstacle, result);

    // Eigen::Matrix4f eigen_transform_target;
    // pcl_ros::transformAsMatrix (base_to_target, eigen_transform_target);
    // pcl::transformPointCloud(result, result, eigen_transform_target);
    // result.header.frame_id =  base_frame_;

    for(int i = 0; i<16; i++)
    {
        delete [] feature_sets[i];
    }

    return result;
}
