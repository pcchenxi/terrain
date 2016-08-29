#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

// #include <unsupported/Eigen/FFT>
//#include "svm.h"

#include "terrain_function_sturcture.h"

using namespace std;

class Filter_Continuity
{
public:
	// Eigen::FFT<float> fft;
	// struct svm_model* m_svm_model;

	Filter_Continuity(int num_one_set);
	int point_num_h;
	float m_smooth_size;
	float m_cell_size;
	float m_max_roughness;
	ros::Time frame_time;

	Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> &velodyne_set, Feature *feature_set);
	Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, ros::Time time);

	pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
	pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, Feature *cloud_feature);
	void set_cell_size(float cell_size);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZRGB> reform_cloud_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
	pcl::PointXYZRGB get_mean_point(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
	void  get_windowboundory(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, float size, int point_index, int &end_index);
	float get_mean_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
	float get_mean_slope(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
	float get_height_variance(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
	float get_roughness(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
	void  compute_terrain_feature(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int point_index, int end_index);
	float get_height_diff(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{
	for (int i = 0; i<point_num_h; i++)
	{
		if (feature_set[i].radius == 0)
			continue;
		
		if(velodyne_sets.points[i].r != 0)
		{
			// velodyne_sets.points[i].z = 255;
			continue;
		}

		float continuity_prob = feature_set[i].roughness;

		// velodyne_sets.points[i].z = continuity_prob;
		

		float threshold_1 = 0.4;
		float threshold_2 = 3.5;
		// float threshold_1 = 0.01;
		// float threshold_2 = 0.1;

		if (continuity_prob < threshold_1)                               // 0.5 for fft[0]   0.04 for fft[1]
		{
			float color_b = continuity_prob / threshold_1 * 255;
			velodyne_sets.points[i].r = 0;
			velodyne_sets.points[i].g = 0;
			velodyne_sets.points[i].b = color_b;
			// cout << "color: " << color <<endl;
		}
		else if (continuity_prob < threshold_2)                             // 1 for fft[0]   0.1 for fft[1]
		{
			float color_g = continuity_prob / threshold_2 * 255;
			velodyne_sets.points[i].r = 0;
			velodyne_sets.points[i].g = color_g;
			velodyne_sets.points[i].b = 255;
		}
		else
		{
			velodyne_sets.points[i].r = 255;
			velodyne_sets.points[i].g = 0;
			velodyne_sets.points[i].b = 0;
		}
	}

	return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, Feature *cloud_feature)
{
	int feature_index = 0;
	pcl::PointCloud<pcl::PointXYZRGB> result;

	for (int i = 0; i < 15; i++)
	{
		velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
		result += velodyne_sets[i];

		for(int j = 0; j < velodyne_sets[i].points.size(); j++)
		{
			cloud_feature[feature_index] = feature_set[i][j];
			feature_index ++;
		}
	}

	return result;
}


Filter_Continuity::Filter_Continuity(int num_one_set)
{
	point_num_h = num_one_set;

	m_smooth_size = 0.1;
	m_cell_size   = 0.2;
}

void Filter_Continuity::set_cell_size(float cell_size)
{
	m_cell_size   = cell_size;
	// cout << "cell_size: " << m_cell_size << endl;
}

Feature ** Filter_Continuity::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, ros::Time time)
{
	m_max_roughness = 0;
	pcl::PointCloud<pcl::PointXYZRGB> result;
	for (int i = 0; i<15; i++)
	{
		feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i]);
	}

	frame_time = time; 
	return feature_set;
}

pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
	point.r = r;
	point.g = g;
	point.b = b;

	return point;
}


pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::reform_cloud_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
	pcl::PointCloud<pcl::PointXYZRGB> new_set = velodyne_set;
	for (int i = 1; i < (int)velodyne_set.points.size(); i = i + 1)
	{
		if (feature_set[i].radius == 0)
			continue;

		new_set.points[i] = get_mean_point(feature_set, velodyne_set, i, i+6);
	}

	return new_set;
}

void Filter_Continuity::get_windowboundory(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, float size, int point_index, int &end_index)
{
	float radius 			= feature_set[point_index].radius;
	end_index 				= point_index;
	
	for (int i = point_index; (int)i < velodyne_set.points.size(); i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		// feature_set[point_index].is_selected = true;		
		if (r == 0 || diff_r > size)
			continue;

		float diff_y	 	= velodyne_set.points[i].y - velodyne_set.points[point_index].y;
		float diff_x 		= velodyne_set.points[i].x - velodyne_set.points[point_index].x;

		// cout << diff_y << " " << diff_x << " " << size << endl;
		// feature_set[point_index].is_selected = true;
		if(abs(diff_y) > size || abs(diff_x) > size)
		{
			end_index = i;
			// cout << "break" << endl;
			break;
		}	
	}

}

float Filter_Continuity::get_mean_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float sum_height 		= 0;
	float point_count 		= 0;
	float radius = feature_set[start].radius;
	for (int i = start; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5)
			continue;

		// cout << " point height: " << velodyne_set[i].z << endl;
		sum_height 			+= velodyne_set[i].z;
		point_count++;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_height 		= sum_height / point_count;
	return mean_height;
}


float Filter_Continuity::get_mean_slope(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float sum_slope 		= 0;
	float point_count 		= 0;
	float radius 			= feature_set[start].radius;
	for (int i = start+1; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5)
			continue;

		float diff_z	 	= velodyne_set.points[i].z - velodyne_set.points[start].z;
		float diff_x 		= velodyne_set.points[i].x - velodyne_set.points[start].x;
		float diff_y 		= velodyne_set.points[i].y - velodyne_set.points[start].y;

		float dist			= sqrt(diff_x*diff_x + diff_y*diff_y);

		float slope			= diff_z / dist;
		sum_slope			+= slope;

		/////////////////////////////  compute vetical acceleration ////////////////////////////
		float slope_sq		= slope * slope;
		float sin_sq 		= slope_sq / (1 + slope_sq);
		float vacc			= 9.8 * sin_sq;
		feature_set[i].vertical_acc	= vacc;

		point_count++;
	}

	if(point_count < 5)
	return 0;

	// cout << point_count << endl;
	float mean_slope 		= sum_slope / point_count;
	return mean_slope;
}

float Filter_Continuity::get_height_variance(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	int   point_count 		= 0;
	float sum_variance		= 0;
	float radius 			= feature_set[start].radius;
	float mean_height		= feature_set[start].mean_height;
	for (int i = start; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5)
			continue;

		// height varience
		float height_diff 	= (mean_height - velodyne_set.points[i].z);

		sum_variance 		+= height_diff * height_diff;
		point_count++;
	}

	 if(point_count < 5)
	 	return 0;

	float variance 			=  sqrt(sum_variance / point_count);
	return variance;
}

float Filter_Continuity::get_height_diff(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float max_height_diff	= 0;
	float radius 			= feature_set[start].radius;

	for (int i = start; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5)
			continue;

		// height varience
		float height_diff 	= abs(velodyne_set.points[start].z - velodyne_set.points[i].z);
		if(height_diff > max_height_diff)
			max_height_diff = height_diff;
	}

	return max_height_diff;
}

float Filter_Continuity::get_roughness(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float sum_vacc_sq 		= 0;
	float point_count 		= 0;
	float radius = feature_set[start].radius;
	for (int i = start+1; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float vacc			= feature_set[i].vertical_acc;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5 || vacc == 0)
			continue;

		sum_vacc_sq 		+= vacc * vacc;

		point_count++;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_vacc_sq 		= sum_vacc_sq / point_count;
	float roughness 		= sqrt(mean_vacc_sq);

	if(point_count < 5)
		roughness = 0;

	return roughness;
}

pcl::PointXYZRGB Filter_Continuity::get_mean_point(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float sum_z		 		= 0;
	float sum_x				= 0;
	float sum_y				= 0;
	float point_count 		= 0;
	float radius = feature_set[start].radius;

	pcl::PointXYZRGB point;
	for (int i = start; i < end; i++)
	{
		float r 	 		= feature_set[i].radius;
		float diff_r 		= abs(r - radius);
		if (r == 0 || diff_r > 0.5)
			continue;

		// cout << " point height: " << velodyne_set[i].z << endl;
		sum_z 				+= velodyne_set[i].z;
		sum_x 				+= velodyne_set[i].x;
		sum_y 				+= velodyne_set[i].y;

		if(velodyne_set[i].r != 0)
			point.r = velodyne_set[i].r;

		point_count++;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;

	point.z			 		= sum_z / point_count;
	point.x			 		= sum_x / point_count;
	point.y			 		= sum_y / point_count;

	return point;
}

void Filter_Continuity::compute_terrain_feature(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int point_index, int end_index)
{
	// feature_set[point_index].mean_height		= get_mean_height		(feature_set, velodyne_set, point_index, end_index);
	// feature_set[point_index].height_variance	= get_height_variance	(feature_set, velodyne_set, point_index, end_index);
	
	// if(feature_set[point_index].height_variance == 0)
		// return;

	// feature_set[point_index].max_height_diff	= get_height_diff		(feature_set, velodyne_set, point_index, end_index);
	feature_set[point_index].mean_slope			= get_mean_slope		(feature_set, velodyne_set, point_index, end_index);
	feature_set[point_index].roughness			= get_roughness			(feature_set, velodyne_set, point_index, end_index);


	// cout << feature_set[point_index].roughness << endl;
	cout << frame_time << " " << velodyne_set.points[point_index].z << " " << feature_set[point_index].mean_height << " " << feature_set[point_index].height_variance << " " 
	<< feature_set[point_index].mean_slope << " " << feature_set[point_index].max_height_diff << " " << feature_set[point_index].roughness << endl;

	if(m_max_roughness < feature_set[point_index].roughness)
		m_max_roughness = feature_set[point_index].roughness;
}

Feature* Filter_Continuity::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> &velodyne_set, Feature *feature_set)
{

	velodyne_set = reform_cloud_height(velodyne_set, feature_set);  // smooth height

	for (int i = 0; i < velodyne_set.points.size(); i = i + 1)
	// for (int i = 350; i < velodyne_set.points.size()-15; i = i + 1)
	{
		if (feature_set[i].radius == 0 || velodyne_set.points[i].r != 0)
			continue;

		int end_index;
		// end_index = i + 11;
		get_windowboundory(feature_set, velodyne_set, m_cell_size, i, end_index);
		compute_terrain_feature(feature_set, velodyne_set, i, end_index);
	}
	
	// cout << endl;
	return feature_set;
}
