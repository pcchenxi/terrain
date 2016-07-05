#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/rsd.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>

//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"

//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>

#include <opencv2/ximgproc.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <slic.h>

#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace cv::xfeatures2d;

//using namespace pcl;


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Dependency Injection Interface ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

class Detector
{
public:
    string currentTaskName;
    Detector()
    {
        currentTaskName = "Base";
    }
    virtual Mat startDetect(Mat img)
    {
        cout << "base detector \n";
 //       return inputImg;
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Types of the feature need to be detected /////////////////////

class SIFT_Detector : public Detector
{
public:
    SIFT_Detector()
    {
        currentTaskName = "SIFT";
    }

    Mat startDetect(Mat img);
};

class SURF_Detector : public Detector
{
public:
    SURF_Detector()
    {
        currentTaskName = "SURF";
    }
    Mat startDetect(Mat img);
};

class SLIC_SuperPixel_Detector : public Detector
{
public:
    SLIC_SuperPixel_Detector()
    {
        currentTaskName = "SLIC_SP";
    }
    Mat startDetect(Mat img);
};

class OpenCV_SuperPixel_Detector : public Detector
{
public:
    OpenCV_SuperPixel_Detector()
    {
        currentTaskName = "CV_SP";
    }
    Mat startDetect(Mat img);
};

class CV_DFT : public Detector
{
public:
    CV_DFT()
    {
        currentTaskName = "DFT";
    }
    Mat startDetect(Mat img);
};

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Setter Injection Class /////////////////////////////////

class FeatureDetectorControlor
{
public:
    Detector *_detectMethod;

    string get_currentTaskName()
    {
        return _detectMethod->currentTaskName;
    }

    void set_detectMethod(Detector *detectMethod)
    {
        this->_detectMethod = detectMethod;
//        cout << "Link to : " << _detectMethod->currentTaskName << "\n";
    }

    Mat startDetect(Mat img)
    {
        Mat result = _detectMethod->startDetect(img);
        return result;
    }
};




///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Feature Detection /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

static const string window_name = "Feature Detector";
static const string window_control_name = "Control";

static Ptr<SuperpixelSEEDS> seeds;
VideoCapture                cap;

FeatureDetectorControlor    detector;
Detector                    *sift_detector;
Detector                    *surf_detector;
Detector                    *slic_sp_detector;
Detector                    *cv_sp_detector;
Detector                    *cv_dft;
Mat                         inputImg;

ros::Subscriber sub;
ros::Publisher  pub;
ros::Publisher  pub2;
ros::Publisher marker_pub;
visualization_msgs::Marker marker;

pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
//pcl::visualization::PCLHistogramVisualizer viewer_pfh;
bool    viewer_pfh_isexist = false;

Mat     intiImage(int argc, char** argv, VideoCapture &cap);
Mat     getNextImage(Mat inputImage, VideoCapture &cap);
void    callback(const sensor_msgs::PointCloud2ConstPtr &msgs);
//void    gpucallback(const sensor_msgs::PointCloud2ConstPtr &msgs);

bool    init = false;
bool    image_node = false;
void    resetTask(int, void*);
void    superPixelNumChanged(int, void*);
void    videoControl(int, void*);

void    initDisplayWindow(string window_name);
void    initParamater();
void    dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);

void    publishLabeledCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr, ros::Publisher inputPub);

////////////////////////////////////Point cloud processing///////////////////////////////////////////////
//pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormals(float radiusSearch, pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);

//////////////// paramaters for superPixel opencv  //////////////////////////
int     num_superpixels;
int     num_levels;
int     prior;
int     num_iterations;

int     CV_SuperPixel_on;
int     SLIC_SuperPixel_on;
int     SIFT_on;
int     SURF_on;
int     topicInput_on;
int     CV_DFT_on;


///////////////////////////// point cloud processing //////////////////////////////////////
pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                         std::vector<int> indices,
                                                         float searchRadius );

pcl::PointCloud<pcl::PFHSignature125>::Ptr calculatePFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                         std::vector<int> indices,
                                                         float searchRadius );

pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr calculateRSD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                         float searchRadius );

void setTerrainPointColor(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr,
                            pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsdFeatures);













