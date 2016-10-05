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

	pcl::PointCloud<pcl::PointXYZRGB> velodyne_cost;
	pcl::PointCloud<pcl::PointXYZRGB> frontp_roughness;
	pcl::PointXYZRGB 				  front_point;

	Feature* 	filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_sets, int beam_index);
	Feature** 	filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_sets, ros::Time time);

	pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_set, Feature  *feature_set, int beam_index);
	pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, Feature *cloud_feature);
	void 		set_cell_size(float cell_size);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZRGB> reform_cloud_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
	pcl::PointXYZRGB get_mean_point(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);

	void 		get_windowboundory(Feature **feature_sets, pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
											, float size, int beam_index, int point_index
											, pcl::PointCloud<pcl::PointXYZRGB> &selected_points, vector<Feature> &selected_features);

	float 		get_mean_height			(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
	float 		get_mean_slope			(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
	float 		get_height_variance		(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
	float 		get_roughness			(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
	Feature  	compute_terrain_feature	(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
	float 		get_height_diff			(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points);
};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set, int beam_index)
{
	for (int i = 0; i<point_num_h; i++)
	{
		if (feature_set[i].radius == 0 || (feature_set[i].roughness == -1 && velodyne_set.points[i].r == 0))
		{
			velodyne_set.points[i].x = 0;
			velodyne_set.points[i].y = 0;
			velodyne_set.points[i].z = 0;
			continue;
		}

		
		velodyne_set.points[i].x = feature_set[i].map_x;
		velodyne_set.points[i].y = feature_set[i].map_y;
		velodyne_set.points[i].z = feature_set[i].map_z;

		if(velodyne_set.points[i].r != 0)
		{
			velodyne_set.points[i].z = 5;
			continue;
		}

		float continuity_prob = feature_set[i].roughness;

		velodyne_set.points[i].z = continuity_prob;

		float threshold_1 = 0.4;
		float threshold_2 = 0.45;
		// float threshold_1 = 0.01;
		// float threshold_2 = 0.1;

		// if (continuity_prob < threshold_1)                               // 0.5 for fft[0]   0.04 for fft[1]
		// {
		// 	float color_b = continuity_prob / threshold_1 * 255;
		// 	velodyne_set.points[i].r = 0;
		// 	velodyne_set.points[i].g = 0;
		// 	velodyne_set.points[i].b = color_b;
		// 	// cout << "color: " << color <<endl;
		// }
		// else if (continuity_prob < threshold_2)                             // 1 for fft[0]   0.1 for fft[1]
		// {
		// 	float color_g = continuity_prob / threshold_2 * 255;
		// 	velodyne_set.points[i].r = 0;
		// 	velodyne_set.points[i].g = color_g;
		// 	velodyne_set.points[i].b = 255;
		// }
		// else
		// {
		// 	float color_r = continuity_prob / 10 * 255;
		// 	velodyne_set.points[i].r = color_r;
		// 	velodyne_set.points[i].g = 255;
		// 	velodyne_set.points[i].b = 255;
		// }

		if (continuity_prob < threshold_1)                               // 0.5 for fft[0]   0.04 for fft[1]
		{
			float color_b = continuity_prob / threshold_1 * 255;
			velodyne_set.points[i].r = 50;
			velodyne_set.points[i].g = 50;
			velodyne_set.points[i].b = 255;
			// cout << "color: " << color <<endl;
		}
		else if (continuity_prob < threshold_2)                             // 1 for fft[0]   0.1 for fft[1]
		{
			float color_g = continuity_prob / threshold_2 * 255;
			velodyne_set.points[i].r = 50;
			velodyne_set.points[i].g = 50;
			velodyne_set.points[i].b = 255;
		}
		else
		{
			float color_r = continuity_prob / 10 * 255;
			velodyne_set.points[i].r = 254;
			velodyne_set.points[i].g = 255;
			velodyne_set.points[i].b = 50;
		}


		if(beam_index == 1 && i > 200 && i < 480)
		{

			front_point = velodyne_set.points[i];
			// front_point.x = feature_set[i].map_x;
			// front_point.y = feature_set[i].map_y;

			front_point.z = feature_set[i].roughness - 3;

			frontp_roughness.points.push_back(front_point);
			// cout << velodyne_set.points[i].x << " " << velodyne_set.points[i].y << " " << velodyne_set.points[i].z << " " << feature_set[i].roughness << endl;
			
		}
	}

	return velodyne_set;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, Feature *cloud_feature)
{
	int feature_index = 0;
	velodyne_cost.points.clear();
	pcl::PointCloud<pcl::PointXYZRGB> result;
	pcl::PointCloud<pcl::PointXYZRGB> colored_set;

	for (int i = 0; i < 15; i++)
	{
		colored_set = color_one_set(velodyne_sets[i], feature_set[i], i);
		velodyne_cost += colored_set;

		for(int j = 0; j < velodyne_sets[i].points.size(); j++)
		{
			// cloud_feature[feature_index] = feature_set[i][j];
			if (feature_set[i][j].radius == 0 || (feature_set[i][j].roughness == -1 && velodyne_sets[i].points[j].r == 0))
				continue;
			colored_set.points[j].z = feature_set[i][j].map_z;

			feature_index ++;
		}

		result += colored_set;
	}

	return result;
}


Filter_Continuity::Filter_Continuity(int num_one_set)
{
	point_num_h = num_one_set;

	m_smooth_size = 0.1;
	m_cell_size   = 0.2;

	frontp_roughness.points.clear();
}

void Filter_Continuity::set_cell_size(float cell_size)
{
	m_cell_size   = cell_size;
	// cout << "cell_size: " << m_cell_size << endl;
}

Feature ** Filter_Continuity::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_sets, ros::Time time)
{
	m_max_roughness = 0;
	pcl::PointCloud<pcl::PointXYZRGB> result;

	for (size_t i = 0; i<15; i++)
	{
		velodyne_sets[i] = reform_cloud_height(velodyne_sets[i], feature_sets[i]);  // smooth height
	}

	for (size_t i = 0; i<15; i++)
	{
		feature_sets[i] = filtering_one_set(velodyne_sets, feature_sets, i);
	}

	frame_time = time; 
	return feature_sets;
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


float Filter_Continuity::get_mean_height(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	float sum_height 		= 0;
	float point_count 		= 0;

	for (size_t i = 1; i < selected_points.points.size(); i++)
	{

		// cout << " point height: " << velodyne_set[i].z << endl;
		sum_height 			+= selected_points.points[i].z;
		point_count++;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_height 		= sum_height / point_count;
	return mean_height;
}


float Filter_Continuity::get_mean_slope(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	float sum_slope 		= 0;
	float point_count 		= 0;

	for (size_t i = 1; i < selected_points.points.size(); i++)
	{
		float diff_z	 	= selected_points.points[i].z - selected_points.points[0].z;
		float diff_x 		= selected_points.points[i].x - selected_points.points[0].x;
		float diff_y 		= selected_points.points[i].y - selected_points.points[0].y;

		float dist			= sqrt(diff_x*diff_x + diff_y*diff_y);

		float slope			= diff_z / dist;
		sum_slope			+= slope;

		/////////////////////////////  compute vetical acceleration ////////////////////////////
		float slope_sq		= slope * slope;
		float sin_sq 		= slope_sq / (1 + slope_sq);
		float vacc			= 9.8 * sin_sq;
		selected_features[i].vertical_acc	= vacc;

		point_count++;
	}

	// cout << point_count << endl;
	float mean_slope 		= sum_slope / point_count;
	return mean_slope;
}

float Filter_Continuity::get_height_variance(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	int   point_count 		= 0;
	float sum_variance		= 0;

	float mean_height		= selected_features[0].mean_height;
	for (size_t i = 1; i < selected_points.points.size(); i++)
	{
		// height varience
		float height_diff 	= (mean_height - selected_points.points[i].z);

		sum_variance 		+= height_diff * height_diff;
		point_count++;
	}

	float variance 			=  sqrt(sum_variance / point_count);
	return variance;
}

float Filter_Continuity::get_height_diff(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	float max_height_diff	= 0;

	for (size_t i = 0; i < selected_points.points.size(); i++)
	{
		// height varience
		float height_diff 	= abs(selected_points.points[0].z - selected_points.points[i].z);
		if(height_diff > max_height_diff)
			max_height_diff = height_diff;
	}

	return max_height_diff;
}

float Filter_Continuity::get_roughness(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	float sum_vacc_sq 		= 0;
	float point_count 		= 0;

	for (size_t i = 1; i < selected_points.points.size(); i++)
	{
		float vacc			= selected_features[i].vertical_acc;
		sum_vacc_sq 		+= vacc * vacc;

		point_count++;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_vacc_sq 		= sum_vacc_sq / point_count;
	float roughness 		= sqrt(mean_vacc_sq);


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

Feature Filter_Continuity::compute_terrain_feature(vector<Feature> &selected_features, pcl::PointCloud<pcl::PointXYZRGB> &selected_points)
{
	int point_index = 0;
	// selected_features[point_index].mean_height = 0;
	// selected_features[point_index].height_variance = 0;
	// selected_features[point_index].max_height_diff = 0;
	// selected_features[point_index].mean_slope = 0;
	// selected_features[point_index].roughness = 0;

	if(selected_points.points.size() < 5)
	{
		selected_features[point_index].roughness = 0;
		return selected_features[point_index];
	}
		

	// selected_features[point_index].mean_height		= get_mean_height		(selected_features, selected_points);
	// selected_features[point_index].height_variance	= get_height_variance	(selected_features, selected_points);

	// selected_features[point_index].max_height_diff	= get_height_diff		(selected_features, selected_points);
	selected_features[point_index].mean_slope		= get_mean_slope		(selected_features, selected_points);
	selected_features[point_index].roughness		= get_roughness			(selected_features, selected_points);

	// cout << selected_points.points.size() << " " << selected_features.size() << endl;
	// cout << selected_features[point_index].roughness << endl;
	// cout << frame_time << " " << selected_points.points[point_index].z << " " << selected_features[point_index].mean_height << " " << selected_features[point_index].height_variance << " " 
	// << selected_features[point_index].mean_slope << " " << selected_features[point_index].max_height_diff << " " << selected_features[point_index].roughness << endl;

	if(m_max_roughness < selected_features[point_index].roughness)
		m_max_roughness = selected_features[point_index].roughness;

	return selected_features[point_index];
}

void Filter_Continuity::get_windowboundory(Feature **feature_sets, pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets
											, float size, int beam_index, int point_index 
											, pcl::PointCloud<pcl::PointXYZRGB> &selected_points, vector<Feature> &selected_features)
{
	selected_points.points.clear();
	selected_features.clear();
	
	selected_points.points.push_back(velodyne_sets[beam_index].points[point_index]);
	selected_features.push_back(feature_sets[beam_index][point_index]);

	for (size_t beam_num = 0; beam_num < 15; beam_num ++)
	{	
		for (size_t i = point_index; i < velodyne_sets[beam_num].points.size(); i++)
		{
			if(beam_num == beam_index && i == point_index)
				continue; 

			float r 	 	= feature_sets[beam_num][i].radius;
			float diff_z 	= velodyne_sets[beam_num].points[i].z - velodyne_sets[beam_index].points[point_index].z;

			if (r == 0 || diff_z > 0.5)
				continue;

			float diff_y	= velodyne_sets[beam_num].points[i].y - velodyne_sets[beam_index].points[point_index].y;
			float diff_x 	= velodyne_sets[beam_num].points[i].x - velodyne_sets[beam_index].points[point_index].x;


			// cout << diff_y << " " << diff_x << " " << size << endl;
			// feature_set[point_index].is_selected = true;
			if(abs(diff_y) > size || abs(diff_x) > size)
			{
				break;
			}	

			selected_points.points.push_back(velodyne_sets[beam_num].points[i]);
			selected_features.push_back(feature_sets[beam_num][i]);
		}
	}

}

Feature* Filter_Continuity::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_sets, int beam_index)
{

	for (int i = 0; i < velodyne_sets[beam_index].points.size(); i = i + 3)
	// for (int i = 100; i <700; i = i + 3)
	{
		if (feature_sets[beam_index][i].radius == 0 || velodyne_sets[beam_index].points[i].r != 0)
			continue;

		pcl::PointCloud<pcl::PointXYZRGB> 	selected_points;
		vector<Feature> 					selected_features;
		
		get_windowboundory(feature_sets, velodyne_sets, m_cell_size, beam_index, i, selected_points, selected_features);
		feature_sets[beam_index][i] = compute_terrain_feature(selected_features, selected_points);
	}
	
	// cout << endl;
	return feature_sets[beam_index];
}
