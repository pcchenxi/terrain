  #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <unsupported/Eigen/FFT>
//#include "svm.h"

#include "terrain_function_sturcture.h"

using namespace std;


class Filter_Continuity
{
public:
	float max_continuity;
	Eigen::FFT<float> fft;
	struct svm_model* m_svm_model;

	Filter_Continuity(int num_one_set);
	int point_num_h;
	Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> &velodyne_set, Feature *feature_set);
	Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);

	float get_varience(Feature *feature_set, int start_index, int size);
	float get_varience_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size);

	float get_mean_intensity(Feature *feature_set, int start, int end);
	float get_mean_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);
    float get_mean_diff_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end, float height);

	pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
	pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, pcl::PointCloud<pcl::PointXYZRGB> &cloud_reformed_height);


	/////////////////////////////////// reform height ////////////////////////////////////////
	float get_previous_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set, int index);
	pcl::PointCloud<pcl::PointXYZRGB> reform_cloud_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);

	void get_points_slope(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
	float get_point_slope(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set, int index);
};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{
	//cout << max_continuity << endl;

	for (int i = 0; i<point_num_h; i++)
	{
		if (feature_set[i].radius == 0 || velodyne_sets.points[i].r != 0)
			continue;

		float continuity_prob = feature_set[i].continuity_prob;

		// float color = continuity_prob/max_continuity * 255;

		// if(continuity_prob == 1)
		// {
		//     velodyne_sets.points[i].r = 200;
		//     velodyne_sets.points[i].g = 200;
		//     velodyne_sets.points[i].b = 200;	
		// }
		// else if(continuity_prob == 3)
		// {
		//     velodyne_sets.points[i].r = 0;
		//     velodyne_sets.points[i].g = 200;
		//     velodyne_sets.points[i].b = 0;	
		// }
		// else if(continuity_prob == 4)
		// {
		//     velodyne_sets.points[i].r = 200;
		//     velodyne_sets.points[i].g = 200;
		//     velodyne_sets.points[i].b = 0;	
		// }
		// else if(continuity_prob == 5)
		// {
		//     velodyne_sets.points[i].r = 200;
		//     velodyne_sets.points[i].g = 0;
		//     velodyne_sets.points[i].b = 200;	
		// }
		// else 
		// {
		//     velodyne_sets.points[i].r = 0;
		//     velodyne_sets.points[i].g = 0;
		//     velodyne_sets.points[i].b = 0;	
		// }

		float threshold_1 = 0.01;
		float threshold_2 = 0.65;
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
			float color_g = 150 + continuity_prob / threshold_2 * 155;
			velodyne_sets.points[i].r = 0;
			velodyne_sets.points[i].g = color_g;
			velodyne_sets.points[i].b = 0;
		}
		else
		{
			velodyne_sets.points[i].r = 255;
			velodyne_sets.points[i].g = 0;
			velodyne_sets.points[i].b = 0;
		}

		// velodyne_sets.points[i].r = 0;
		// velodyne_sets.points[i].g = 0;
		// velodyne_sets.points[i].b = color;	
	}

	return velodyne_sets;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set, pcl::PointCloud<pcl::PointXYZRGB> &cloud_reformed_height)
{
	pcl::PointCloud<pcl::PointXYZRGB> result;
	pcl::PointCloud<pcl::PointXYZRGB> reformed_height;
	cloud_reformed_height.points.clear();

	for (int i = 0; i<16; i++)
	{
		velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
		result += velodyne_sets[i];

		reformed_height = velodyne_sets[i];
		for (int j = 0; j < reformed_height.size(); j++)
		{
			reformed_height.points[j].z = feature_set[i][j].reformed_height;
			// cout << feature_set[i][j]->reformed_height << endl;
		}
		cloud_reformed_height += reformed_height;
	}

	return result;
}


Filter_Continuity::Filter_Continuity(int num_one_set)
{
	max_continuity = 0;
	point_num_h = num_one_set;
}

float Filter_Continuity::get_mean_intensity(Feature *feature_set, int start, int end)
{
	float sum_intensity = 0;
	float value_count = 0;
	for (int i = start; i < end; i++)
	{
		float r = feature_set[i].radius;
		if (r == 0)
			continue;

		value_count++;

		sum_intensity += feature_set[i].intensity;
	}

	return sum_intensity / value_count;
}

float Filter_Continuity::get_mean_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
	float sum_height = 0;
	float value_count = 0;
	for (int i = start; i < end; i++)
	{
		float r = feature_set[i].radius;
		if (r == 0)
			continue;

		value_count++;

		// cout << " point height: " << velodyne_set[i].z << endl;
		sum_height += velodyne_set[i].z;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_height = sum_height / value_count;
	return mean_height;
}


float Filter_Continuity::get_mean_diff_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end, float height)
{
	float sum_diff_height = 0;
	float value_count = 0;
	for (int i = start; i < end; i++)
	{
		float r = feature_set[i].radius;
		if (r == 0)
			continue;

		value_count++;

		// cout << " point height: " << velodyne_set[i].z << endl;
        float diff_height = abs(height - velodyne_set[i].z);

		sum_diff_height += diff_height;
	}

	// cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
	float mean_height = sum_diff_height / value_count;
	return mean_height;
}

float Filter_Continuity::get_varience_height(Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size)
{
	std::vector<float> timevec;
	std::vector<std::complex<float> > freqvec;

	float varience = 0;
	float valud_count = 0;

	int start = start_index - size / 2;
	int end = start_index + size / 2;

	if (start < 0)
		start = 0;
	if (end > velodyne_set.points.size() - 1)
		end = velodyne_set.points.size() - 1;


	//////////////////////////////////////////////////
	float cen_radius = feature_set[start_index].radius;
	// float cen_intensity  = feature_set[start_index].intensity;
	float var_intensity = 0;
	float mean_intensity = get_mean_intensity(feature_set, start, end);
	float mean_height = get_mean_height(feature_set, velodyne_set, start, end);
	float cen_height = velodyne_set.points[start_index].z;
    float base_length = cen_radius * size/2.0 *M_PI/180;

	feature_set[start_index].reformed_height = cen_height - mean_height;

	// // cout << "mean height: " << mean_height << endl;
	for (int i = start; i < end; i++)
	{
		float r = feature_set[i].radius;
		if (r == 0)
		{
			int time_size = timevec.size();
			if (time_size >0)
				timevec.push_back(timevec[time_size - 1]);
			continue;
		}


		float diff = (r - cen_radius);
		float diff_abs = abs(diff);

		if (diff < 0 && diff_abs > 0.1)
		{
			int time_size = timevec.size();
			if (time_size >0)
				timevec.push_back(timevec[time_size - 1]);
			continue;
		}

		// height varience
		float dist = (mean_height - velodyne_set.points[i].z);
        float mean_diff_height = get_mean_diff_height(feature_set, velodyne_set, start, end, velodyne_set.points[i].z);

		// float dist = velodyne_set.points[i].z;
		float dist_abs = abs(mean_diff_height)/base_length;
		// cout << dist << " ";
		valud_count++;

		timevec.push_back(dist);
		varience += dist*dist;

		// intensity varience
		float intensity = feature_set[i].intensity;
		float diff_intensity = abs(intensity - mean_intensity);
		var_intensity += diff_intensity*diff_intensity;
	}
	// cout << endl;
	// if(valud_count < size)
	//     return 0;

	varience = sqrt(varience) / valud_count;
	var_intensity = sqrt(var_intensity) / valud_count;

	// cout << "before fft" << timevec.size() << endl;
	fft.fwd(freqvec, timevec);
	// cout << "after" << freqvec.size() << endl;
	// cout << "5 ";

	vector<float> features;
	float sum_magnitude = 0;
	for (int i = 0; i < freqvec.size() / 4; i++)
		// for(int i = 1; i < 6; i++)
	{
		float magnitude = sqrt(freqvec[i].real()*freqvec[i].real() + freqvec[i].imag()*freqvec[i].imag());
		sum_magnitude += magnitude;
		// cout << i+1 << ":" << magnitude << " ";
		// cout << magnitude << " ";
		features.push_back(magnitude);
		//cout << freqvec[i] << " real: " << freqvec[i].real() << " imag: " << freqvec[i].imag() << " magnitude: " << magnitude << endl;
	}
	// int index_mid       = freqvec.size()/2;
	// float magnitude_mid = sqrt(freqvec[index_mid].real()*freqvec[index_mid].real() + freqvec[index_mid].imag()*freqvec[index_mid].imag());
	// cout << magnitude_mid << " ";
	// cout << endl;        
	// cout << cen_intensity << " " << mean_intensity << " " << var_intensity << " " << cen_radius << endl;
	// cout << "3:" << mean_intensity << " 4:" << varience << endl;

	// features.push_back(mean_intensity);
	// features.push_back(varience);

	float avg_magnitude = sum_magnitude / features.size();
	if (features.size() == 0)
		cout << " !!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
	// return avg_magnitude; 
	return varience;

	// return 0;
}

float Filter_Continuity::get_varience(Feature *feature_set, int start_index, int size)
{
	float varience = 0;
	int   valud_count = 0;

	float avg = feature_set[start_index].radius;

	int start = start_index - size / 2;
	int end = start_index + size / 2 + 1;

	if (start < 0)
		start = start + point_num_h;
	if (end >= point_num_h)
		end = point_num_h - 1;

	//cout << "start: " << start << "  end: " << end << endl;
	for (int i = start; i < end; i++)
	{
		int index = i % point_num_h;
		float r = feature_set[index].radius;
		//cout << "avg: " << avg << "  r: " << r << endl;
		if (r == 0)
			continue;

		valud_count++;

		float diff = (r - avg);
		float diff_abs = abs(diff);

		if (diff < 0 && diff_abs > 0.1)
			continue;

		if (diff_abs > 0.4)
		{
			diff = 0.4;
			//  break;
		}
		varience += diff*diff;
	}

	//  if(valud_count < 3)
	//      varience = 0;
	//  cout << "varience: " << varience << endl;
	//   if(varience > 0.025)
	//	varience = 1;
	return varience;
}

Feature ** Filter_Continuity::filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
	max_continuity = 0;
	pcl::PointCloud<pcl::PointXYZRGB> result;
	for (int i = 0; i<16; i++)
	{
		feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i]);
	}

	return feature_set;
}

pcl::PointXYZRGB set_point_color(pcl::PointXYZRGB point, int r, int g, int b)
{
	point.r = r;
	point.g = g;
	point.b = b;

	return point;
}

float Filter_Continuity::get_previous_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set, int index)
{
	float height = 0.0;
	for (int i = index - 1; i >= 0; i = i - 1)
	{
		if (feature_set[i].radius == 0)
			continue;

		height = velodyne_set[i].z;
		// cout << index << " " << i << " " << height;
		break;
	}

	if (height > 2)
		height = 2;
	return height;
}

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::reform_cloud_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
	pcl::PointCloud<pcl::PointXYZRGB> new_set = velodyne_set;
	for (int i = 1; i < velodyne_set.points.size(); i = i + 1)
	{
		if (feature_set[i].radius == 0)
			continue;

		float pre_height = get_previous_height(velodyne_set, feature_set, i);
		float new_height = velodyne_set.points[i].z - pre_height;

		// cout << "pre height: " << pre_height << " curent_hei: " << velodyne_set.points[i].z << " new_h: " << new_height << endl;

		// pcl::PointXYZRGB point = velodyne_set.points[i];
		// // point.z = new_height;

		// new_set.points.push_back(point);
		new_set.points[i].z = new_height;
	}

	return new_set;
}

float Filter_Continuity::get_point_slope(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set, int index)
{
	float ver_acc = 0;
    float sin_a = 0;

	pcl::PointXYZRGB p, p1, p2;
	p1 = velodyne_set.points[index];

	float height = 0.0;
	for (int i = index+1; i < velodyne_set.points.size(); i ++)
	{
		if (feature_set[i].radius == 0)
			continue;

		p2 = velodyne_set.points[i];
		p.x = p2.x - p1.x;
		p.y = p2.y - p1.y;
		p.z = p2.z - p1.z;

		float length = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
		float sin_a = p.z / length;
		float sin_a_sqr = sin_a * sin_a;

		ver_acc = 9.8 * sin_a_sqr;

		break;
	}

    cout << "ver_acc: " << ver_acc <<  " "  << asin(sin_a) * 180/M_PI << endl;
	return ver_acc;
}

void Filter_Continuity::get_points_slope(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
	for (int i = 1; i < velodyne_set.points.size(); i = i + 1)
	{
		if (feature_set[i].radius == 0)
			continue;

		feature_set[i].ver_acc = get_point_slope(velodyne_set, feature_set, i);
	}
}


Feature * Filter_Continuity::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> &velodyne_set, Feature *feature_set)
{

	// velodyne_set = reform_cloud_height(velodyne_set, feature_set);
    // get_points_slope(velodyne_set, feature_set);
	for (int i = 0; i < velodyne_set.points.size(); i = i + 1)
	{
		feature_set[i].reformed_height = velodyne_set.points[i].z;
		float varience = 0;
		if (feature_set[i].radius == 0 || feature_set[i].radius > 10 || velodyne_set.points[i].r != 0)
		{
			continue;
		}

		if (velodyne_set.points[i].z < 2.0)
		{
			//float varience_1 = get_varience(feature_set, i, 7);
			float varience_2 = get_varience_height(feature_set, velodyne_set, i, 16);
			varience = varience_2;
			// cout << i << " " << varience << endl;
		}



		// if(varience > 0.02)
		//     varience = 0.02;
		feature_set[i].continuity_prob = varience;


		if (varience > max_continuity)
			max_continuity = varience;

		// cout << "type: " << varience << endl;
	}

	// cout << endl;
	return feature_set;
}