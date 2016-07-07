#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unsupported/Eigen/FFT>
#include "svm.h"

#include "terrain_function_sturcture.h"

using namespace std;
using namespace cv;

struct Vec3
{
    float x;
    float y;
    float z;
 
    const Vec3 operator-( const Vec3& a )
    {
        Vec3 v;
        v.x = x - a.x;
        v.y = y - a.y;
	v.z = z - a.z;
 
        return v;
    }
 
    const Vec3 operator*( float a ) const
    {
        Vec3 v;
        v.x = x * a;
        v.y = y * a;
	v.z = z * a;
 
        return v;
    }
};
 
float Dot( const Vec3& a, const Vec3& b )
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float DistancePtLine( Vec3 a, Vec3 b, Vec3 p )
{
    Vec3 n = b - a;
    Vec3 pa = a - p;
    Vec3 c = n * (Dot( pa, n ) / Dot( n, n ));
    Vec3 d = pa - c;
    return abs(d.z);
    //return sqrt( Dot( d, d ) );
}

//////////////////////////////////////////////  SVM ////////////////////////////////////////////////

// int SvmPredict( const char* modelAdd, vector<float> features )
int SvmPredict(struct svm_model* svm_model, vector<float> features )
{
	struct svm_node *testX;
    
    // struct svm_model* testModel;
	// testModel = svm_load_model( modelAdd );

	int featureDim = features.size();
	testX = new struct svm_node[featureDim + 1];

	for ( int i = 0; i < featureDim; i++ )
	{
		testX[i].index	= i + 1;
		testX[i].value	= features[i];
	}

	testX[featureDim].index = -1;
	double p = svm_predict( svm_model, testX );

//	svm_free_and_destroy_model( &testModel );
	delete[] testX;
    // cout << "type: " << p << endl;
	// if ( p > 0.5 )
	// {
	// 	return(1);
	// }else {
	// 	return(-1);
	// }

    return p;
}





class Filter_Continuity
{
    public:
        float max_continuity;
        Eigen::FFT<float> fft;
        struct svm_model* m_svm_model;

        Filter_Continuity(int num_one_set );
        int point_num_h;
        Feature* filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set);
        Feature** filtering_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);

        float get_varience( Feature *feature_set, int start_index, int size);
        float get_varience_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size);

        void  get_min_max_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set
                , int start_index, int size, float &min_height, float &max_height);

        float get_mean_intensity( Feature *feature_set, int start, int end);
        float get_mean_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end);

        pcl::PointCloud<pcl::PointXYZRGB> color_one_set(pcl::PointCloud<pcl::PointXYZRGB>  velodyne_sets, Feature  *feature_set);
        pcl::PointCloud<pcl::PointXYZRGB> color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set);


};

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_sets, Feature *feature_set)
{
    //cout << max_continuity << endl;
    float color_step = 255 * 255;
    for(int i = 0; i<720; i++)
    {
        if(feature_set[i].radius == 0 || velodyne_sets.points[i].r != 0)
            continue;

        float continuity_prob = feature_set[i].continuity_prob;

        float color = continuity_prob/max_continuity * 255;

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

        float threshold_1 = 0.06;
        float threshold_2 = 0.15;
        if(continuity_prob < threshold_1)                               // 0.5 for fft[0]   0.04 for fft[1]
        {
            float color_b = continuity_prob/threshold_1 * 255;
            velodyne_sets.points[i].r = 0;
            velodyne_sets.points[i].g = 0;
            velodyne_sets.points[i].b = color_b;	
            // cout << "color: " << color <<endl;
        }
        else if(continuity_prob < threshold_2)                             // 1 for fft[0]   0.1 for fft[1]
        {
            float color_g = continuity_prob/threshold_2 * 255;
            velodyne_sets.points[i].r = 255;
            velodyne_sets.points[i].g = color_g;
            velodyne_sets.points[i].b = 255;	
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

pcl::PointCloud<pcl::PointXYZRGB> Filter_Continuity::color_all_sets(pcl::PointCloud<pcl::PointXYZRGB> *velodyne_sets, Feature **feature_set)
{
    pcl::PointCloud<pcl::PointXYZRGB> result;

    for(int i = 0; i<16; i++)
    {
        velodyne_sets[i] = color_one_set(velodyne_sets[i], feature_set[i]);
        result += velodyne_sets[i];
    }

    return result;
}


Filter_Continuity::Filter_Continuity(int num_one_set = 720)
{
    max_continuity = 0;
    point_num_h = num_one_set;
    m_svm_model= svm_load_model( "model/velodyne_4_50000.model" );
   //     m_svm_model= svm_load_model( "model/velodyne_4_full.model" );
}

void Filter_Continuity::get_min_max_height(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set
                , int start_index, int size, float &min_height, float &max_height)
{
    min_height = 999;
    max_height = -999;

    int start = start_index - size/2;
    int end = start_index + size/2 + 1;

    if(start < 0)
        start = start + velodyne_set.points.size();
    if(end > velodyne_set.points.size()-1)
        end = velodyne_set.points.size()-1;

    for(int i = start; i < end; i++)
    {
        int index = i % velodyne_set.points.size();

        if(velodyne_set.points[index].x == 0 && velodyne_set.points[index].y == 0)
            continue;

        float height = velodyne_set.points[index].z;

        if(height < min_height)
            min_height = height;
        if(height > max_height)
            max_height = height;
    }
}

float Filter_Continuity::get_mean_intensity( Feature *feature_set, int start, int end)
{
    float sum_intensity;
    float value_count = 0;
    for(int i = start; i < end; i++)
    {
        float r = feature_set[i].radius;
        if(r == 0)
            continue;

        value_count ++;

        sum_intensity += feature_set[i].intensity;
    }

    return sum_intensity/value_count;
}

float Filter_Continuity::get_mean_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start, int end)
{
    float sum_height;
    float value_count = 0;
    for(int i = start; i < end; i++)
    {
        float r = feature_set[i].radius;
        if(r == 0)
            continue;

        value_count ++;

        // cout << " point height: " << velodyne_set[i].z << endl;
        sum_height += velodyne_set[i].z;
    }

    // cout << "sum_height: " << sum_height << "  count: " << value_count << endl;
    float mean_height = sum_height/value_count;
    return mean_height;
}

float Filter_Continuity::get_varience_height( Feature *feature_set, pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, int start_index, int size)
{
    std::vector<float> timevec;
    std::vector<std::complex<float> > freqvec;

    Mat height_diff(5, size+1, CV_8UC1, Scalar( 0));

    float sum = 0;
    float varience = 0;
    float valud_count = 0;

    int img_index = -1;
    int start = start_index - size/2;
    int end = start_index + size/2;

    if(start < 0)
        start = 0;
    if(end > velodyne_set.points.size()-1)
        end = velodyne_set.points.size()-1;

    Vec3 a, b; 
    a.x = velodyne_set.points[start].x;
    a.y = velodyne_set.points[start].y;
    a.z = velodyne_set.points[start].z;

    b.x = velodyne_set.points[end].x;
    b.y = velodyne_set.points[end].y;
    b.z = velodyne_set.points[end].z;


//////////////////////////////////////////////////
    float cen_radius     = feature_set[start_index].radius;
    float cen_intensity  = feature_set[start_index].intensity;
    float var_intensity  = 0;
    float mean_intensity = get_mean_intensity(feature_set, start, end);
    float mean_height    = get_mean_height(feature_set, velodyne_set, start, end);
    float cen_height     = velodyne_set.points[start_index].z;

    // cout << "mean height: " << mean_height << endl;
    for(int i = start; i < end; i++)
    {
	    img_index ++;
        float r = feature_set[i].radius;
        if(r == 0)
            continue;
        
        float diff = (r - cen_radius);
	    float diff_abs = abs(diff);

	    if(diff < 0 && diff_abs > 0.1)
	        continue;

        // height varience
        Vec3 p;
        p.x = velodyne_set.points[i].x;
        p.y = velodyne_set.points[i].y;
        p.z = velodyne_set.points[i].z;

        // float dist = DistancePtLine(a, b, p);
        float dist = (cen_height - p.z);
        float dist_abs = abs(dist);
        // if(dist_abs > 1)
        //     continue;


        valud_count ++;
        // // cout << "dist: " << dist << " point_h: " << p.z<< endl;
        // if(dist > 0.1)
        //     dist = 0.1;
        // if(dist < -0.1)
        //     dist = -0.1;            

        timevec.push_back(dist_abs);
        varience += dist*dist;

        // intensity varience
        float intensity      = feature_set[i].intensity;
        float diff_intensity = abs(intensity - mean_intensity);
        var_intensity        += diff_intensity*diff_intensity;
    }

    if(valud_count < size/2)
        return 0;

    varience        = sqrt(varience)/valud_count;
    var_intensity   = sqrt(var_intensity)/valud_count;

    fft.fwd( freqvec,timevec);

    // cout << "5 ";

    vector<float> features;
    float sum_magnitude = 0;
    for(int i = 1; i < freqvec.size()/4; i++)
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

    // int type = SvmPredict(m_svm_model, features);

    float avg_magnitude = sum_magnitude/features.size();
    return sum_magnitude; // fft[0] fft[1] mean_intensity varience
}

float Filter_Continuity::get_varience( Feature *feature_set, int start_index, int size)
{
    float sum = 0;
    float varience = 0;
    int   valud_count = 0;

    float avg = feature_set[start_index].radius;

    int start = start_index - size/2;
    int end = start_index + size/2 + 1;

    if(start < 0)
        start = start + point_num_h;
    if(end >= point_num_h)
        end = point_num_h - 1;

    //cout << "start: " << start << "  end: " << end << endl;
    for(int i = start; i < end; i++)
    {
        int index = i % point_num_h;
        float r = feature_set[index].radius;
        //cout << "avg: " << avg << "  r: " << r << endl;
        if(r == 0)
            continue;

        valud_count ++;

        float diff = (r - avg);
	    float diff_abs = abs(diff);

	    if(diff < 0 && diff_abs > 0.1)
	        continue;

        if(diff_abs > 0.4)
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
    for(int i = 0; i<16; i++)
    {
        feature_set[i] = filtering_one_set(velodyne_sets[i], feature_set[i]);
        //result += velodyne_sets[i];
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

Feature * Filter_Continuity::filtering_one_set(pcl::PointCloud<pcl::PointXYZRGB> velodyne_set, Feature *feature_set)
{
    int start_index = 0;
    int pre_index = 0;
    float color_step = 255 * 255;

    for(int i = 0; i < velodyne_set.points.size(); i = i+1)
    {
        float varience = 0;
        if(feature_set[i].radius == 0 || velodyne_set.points[i].r != 0)
            continue;

        float min_height, max_height;

        // get_min_max_height(velodyne_set, i, 5, min_height, max_height);
        // float diff_height   = abs( min_height - max_height);


	    if(velodyne_set.points[i].z < 2.0)
        {
            //float varience_1 = get_varience(feature_set, i, 7);
            float varience_2 = get_varience_height(feature_set, velodyne_set, i, 8);
            varience = varience_2;
            cout << i << " " << varience << endl;
	    }



        // if(varience > 0.02)
	    //     varience = 0.02;
        feature_set[i].continuity_prob = varience;


        if(varience > max_continuity)
            max_continuity = varience;

        // cout << "type: " << varience << endl;
    }

   // cout << endl;
    return feature_set;
}
