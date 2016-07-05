#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "terrain_function_sturcture.h"
#include "xi_functions.h"

using namespace std;


class Filter_Cross_Section
{
    public:
        float max_cross;

        Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);
        // pcl::PointCloud<pcl::PointXYZRGB> filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, vector<Feature> feature_set);
        float filtering_one_set(int index, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, vector<Feature> feature_set);

        pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
        pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);

};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Cross_Section::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{
    for(int i = 0; i<720; i++)
    {
        if(feature_set[i].radius == 0)
            continue;

        if(velodyne_sets.points[i].r == 255)
            continue;
//        velodyne_sets.points[i].r = 0;
//        velodyne_sets.points[i].g = 0;
//        velodyne_sets.points[i].b = 0;

        float cross_section_prob = feature_set[i].cross_section_prob;
        velodyne_sets.points[i].r = cross_section_prob/max_cross * 255;
    }

    return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Cross_Section::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    pcl::PointCloud<pcl::PointXYZRGB> result;

    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
        result += velodyne_sets[i];
    }

    return result;
}

Feature ** Filter_Cross_Section::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    max_cross = 0;

    pcl::PointCloud<pcl::PointXYZRGB> point_all;
    pcl::PointCloud<pcl::PointXYZRGB> point_selected;

    for(int i = 0; i < 720; i = i+1)
    {
        pcl::PointCloud<pcl::PointXYZRGB> point_selected;
        pcl::PointCloud<pcl::PointXYZRGB> result;
        vector<Feature> point_feature;
        point_feature.clear();

        for(int j = 0; j < 16; j ++)
        {
            point_selected.points.push_back(velodyne_sets[j].points[i]);
            point_feature.push_back(feature_set[j][i]);

            //cout << velodyne_sets[j].points[i].x << "  " << velodyne_sets[j].points[i].y << "  " << velodyne_sets[j].points[i].z << endl;
        }

//        for(int j = 0; j < 16; j ++)
//        {
//            if(i > 718)
//                continue;
//
//            point_selected.points.push_back(velodyne_sets[j].points[i+1]);
//            point_feature.push_back(feature_set[j][i+1]);
//        }

        for(int j = 0; j < point_selected.points.size(); j ++)
        {
            float prob = filtering_one_set(j, point_selected, point_feature);
            if(j < 16)
                feature_set[j][i].cross_section_prob = prob;
//            else
//            {
//                feature_set[j-16][i+1].cross_section_prob = prob;
//            }
        }
    }

    return feature_set;
}

float get_difficult_value(float diff_height)
{
    float difficulity = 0;

    if(diff_height < 0.05)
        difficulity = 2 * diff_height;
    else if(diff_height < 0.15)
        difficulity = 0.1 + 3 * diff_height;
    else if(diff_height < 0.2)
        difficulity = 0.4 + 12 * diff_height;
    else if(diff_height < 1)
        difficulity = 1;
    else if(diff_height < 1.5)
        difficulity = 1 - 2 * diff_height;

    if(difficulity < 0)
        difficulity = 0;

    // cout << "diff_h: " << diff_height << endl;
    // cout << "difficulity: " << difficulity << endl;

    return difficulity;
}

float Filter_Cross_Section::filtering_one_set(int index, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, vector<Feature> feature_set)
{
    float color_step = 255 * 255;
    float sum_d = 0;

    float c_radius  = feature_set[index].radius;
    float c_x       = velodyne_set.points[index].x;
    float c_y       = velodyne_set.points[index].y;
    float c_z       = velodyne_set.points[index].z;

    pcl::PointXYZRGB c_point = velodyne_set.points[index];

    for(int i = 0; i < velodyne_set.points.size(); i++ )
    {
        pcl::PointXYZRGB point = velodyne_set.points[i];
        float diff_z = abs(c_z - point.z);

        float radius = feature_set[i].radius;
        float diff_r = abs(radius - c_radius);

        if(diff_r < 0.5 && diff_z > diff_r)
        {
            sum_d = 1;
            break;
        }

//
//        //cout << "r1: " << radius << "  r2: " << c_radius << endl;
//
//        if(diff_r < 0.5)
//        {
//            float diff_h = abs(velodyne_set.points[i].z - c_point.z);
//            float difficulity = get_difficult_value(diff_h);
//
//            sum_d += difficulity;
//        }
    }

    if(max_cross < sum_d)
        max_cross = sum_d;

    return sum_d;
}
