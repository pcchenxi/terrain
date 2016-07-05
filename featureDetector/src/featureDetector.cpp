#include "featureDetector.h"


int main(int argc, char** argv )
{
    /* ros init */
    ros::init(argc, argv, "featureDetector");
	ros::NodeHandle n;

	sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/kinect2/sd/points2", 1);
    pub2 = n.advertise<sensor_msgs::PointCloud2>("/kinect2/sd/points3", 1);

	ros::Rate loop_rate(10);

//    pcl::gpu::setDevice(0);
//    pcl::gpu::printShortCudaDeviceInfo(0);

    Mat  nextFrame, result;

    /* need to call initParamater() before initDisplayWindow()  */
    inputImg = intiImage(argc, argv, cap);
    initParamater();
    initDisplayWindow(window_name);

    while (ros::ok())
	{
        nextFrame = getNextImage(inputImg, cap);

        if(topicInput_on == 0)
            imshow(window_name, nextFrame);

        if(topicInput_on == 0 && detector._detectMethod != NULL)
        {
            result = detector.startDetect(nextFrame);
            imshow(window_name, result);
        }

        /* press any key to stop and press again to continue  */
        int c = waitKey(10);
		if( c >= 0) { waitKey(0); }

        ros::spinOnce();
		loop_rate.sleep();
    }

    if(cap.isOpened())
        cap.release();

	return 1;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &msgs)
{
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::fromROSMsg(*msgs, point_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prt (new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud));

    std::vector<int> indices_normal;
    for (size_t i = point_cloud.height*3/9; i < point_cloud.height*7/9; i=i+2)
    for (size_t j = point_cloud.width*1/9; j < point_cloud.width*8/9; j=j+2)
    {
        int inx = i*point_cloud.width + j;
        indices_normal.push_back(inx);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_point_prt (new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud, indices_normal));
    publishLabeledCloud(normal_point_prt, pub2);


    return;

////////////////////////////////calculate surface normal//////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    cloud_normals = calculateSurfaceNormal(cloud_prt, indices_normal, 0.04);  // radius 0.04, Ksearch 25

    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(normal_point_prt, cloud_normals, 20, 0.03, "normal");


///////////////////////////////////////// PFH  ////////////////////////////////////////////////////////
//    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeatures(new pcl::PointCloud<pcl::PFHSignature125>);
//    pfhFeatures = calculatePFH(point_cloud, cloud_normals, indices, 0.05);
//
//    pfhFeatures->points[0].histogram[124] = 150.0;
//    if(viewer_pfh_isexist == false)
//    {
//        viewer_pfh.addFeatureHistogram(*pfhFeatures, 150, "pfh");
//        viewer_pfh_isexist = true;
//    }
//    else
//        viewer_pfh.updateFeatureHistogram(*pfhFeatures, 150, "pfh");


//////////////////////////////////////// RSD ////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsdFeatures(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    rsdFeatures = calculateRSD(normal_point_prt, cloud_normals, 0.07); //// radius 0.05, Ksearch 49

//////////////////////////////////////// set color /////////////////////////////////////////////////////
    setTerrainPointColor(normal_point_prt, rsdFeatures);
    for(int i = 0; i<normal_point_prt->points.size(); i=i+4)
    {
        normal_point_prt->points[i].x += 1.5;
    }
    publishLabeledCloud(normal_point_prt, pub);


//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(selected_point_ptr);
//    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler (selected_point_ptr, 255, 0, 0);
//    viewer.addPointCloud<pcl::PointXYZRGB> (selected_point_ptr, color_handler, "original_cloud");
//    viewer.addCoordinateSystem (1.0, 0.0);
//    viewer.setBackgroundColor(0.5, 0.5, 0.5, 0); // Setting background to a dark grey
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
//
//
/////////////////////////////////////   set all points to 0, for sphere display   ////////////////////////////
    int sphereRadius = 0.3;
    for (size_t i = 0; i < (*normal_point_prt).points.size (); i++)
    {
        (*normal_point_prt).points[i].x = cloud_normals->points[i].normal[0] - sphereRadius;
        (*normal_point_prt).points[i].y = cloud_normals->points[i].normal[1] - sphereRadius;
        (*normal_point_prt).points[i].z = cloud_normals->points[i].normal[2] - sphereRadius;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(normal_point_prt, cloud_normals, 20, 0.01, "sphere1");

    viewer.spinOnce ();
//    viewer_pfh.spinOnce();
//
    viewer.removePointCloud ("original_cloud");
    viewer.removePointCloud ("normal");
    viewer.removePointCloud ("sphere1");
    viewer.removePointCloud ("sphere2");
 }


pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                         std::vector<int> indices,
                                                         float searchRadius )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_cloud_ptr;

    if(indices.size() == 0)
        selected_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> (*point_cloud));
    else
        selected_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> (*point_cloud, indices));

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (selected_cloud_ptr);
    ne.setSearchSurface(point_cloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals4 (new pcl::PointCloud<pcl::Normal>);

    //ne.setKSearch (50);
    ne.setRadiusSearch (searchRadius);
    ne.compute (*cloud_normals);

    return cloud_normals;

    //viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(selected_point_ptr, cloud_normals, 20, 0.03, "normal");
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr calculatePFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                         std::vector<int> indices,
                                                         float searchRadius )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_cloud_ptr;

    if(indices.size() == 0)
        selected_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> (*point_cloud));
    else
        selected_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> (*point_cloud, indices));

    std::vector<int> pointIds(1);
    pointIds[0] = indices.size()/2;

    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfhEstimation;
    pfhEstimation.setInputCloud (selected_cloud_ptr);
    pfhEstimation.setSearchSurface(point_cloud);
    pfhEstimation.setInputNormals(cloud_normals);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pfhEstimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeatures(new pcl::PointCloud<pcl::PFHSignature125>);
    pfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));
    pfhEstimation.setRadiusSearch (searchRadius);  // 0.1

    // Actually compute the features
//    std::cout << "Computing features..." << std::endl;
    pfhEstimation.compute (*pfhFeatures);

//    std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    //  pcl::PFHSignature125 descriptor = pfhFeatures->points[0];
    //  std::cout << descriptor << std::endl;

    return pfhFeatures;
}



pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr calculateRSD(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                                                            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                            float searchRadius )
{
   // RSD estimation object.
	pcl::RSDEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
	pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());

    std::vector<int> indices_rsd;
    for (size_t i = 0; i < point_cloud->points.size (); i = i+4)
    {
        indices_rsd.push_back(i);
    }
    std::vector<int> (indices); //!!!!!!!!!!!!!!!!!!!!!!!!!
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    rsd.setIndices (indicesptr);

	rsd.setInputCloud(point_cloud);
//	rsd.setSearchSurface(point_cloud);
	rsd.setInputNormals(cloud_normals);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	rsd.setSearchMethod(tree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	rsd.setRadiusSearch(searchRadius);
	// Plane radius. Any radius larger than this is considered infinite (a plane).
	rsd.setPlaneRadius(10);
	// Do we want to save the full distance-angle histograms?
	rsd.setSaveHistograms(false);

	rsd.compute(*descriptors);

    int ind = descriptors->points.size()/2;
   // for(int i = 0; i<descriptors->points.size(); i++)
    {
        cout << descriptors->points[ind-10].r_min << "," << descriptors->points[ind-10].r_max << endl;
        cout << descriptors->points[ind].r_min << "," << descriptors->points[ind].r_max << endl;
        cout << descriptors->points[ind+10].r_min << "," << descriptors->points[ind+10].r_max << endl;
    }

    return descriptors;
}


void setTerrainPointColor(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr,
                            pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsdFeatures)
{
    for(int i = 0; i<point_cloud_ptr->points.size(); i++)
    {
        float r_max = rsdFeatures->points[i].r_max;
        float r_min = rsdFeatures->points[i].r_min;
        if(r_max > 1.4)   // flat
        {
            point_cloud_ptr->points[i].r = 0;
            point_cloud_ptr->points[i].g = 0;
            point_cloud_ptr->points[i].b = 150;//255-(100*r_max);
        }
        else if(r_max > 0.7)  // grass
        {
            point_cloud_ptr->points[i].r = 0;
            point_cloud_ptr->points[i].g = 150;//255-(100*r_max);
            point_cloud_ptr->points[i].b = 0;
        }
        else // stone
        {
            point_cloud_ptr->points[i].r = 150;//255-(100*r_max);
            point_cloud_ptr->points[i].g = 0;
            point_cloud_ptr->points[i].b = 0;
        }
//        else if(r_max >= 1.8)
//        {
//            point_cloud_ptr->points[i].r = 0;
//            point_cloud_ptr->points[i].g = 0;
//            point_cloud_ptr->points[i].b = 50*r_max;
//        }

//        point_cloud_ptr->points[i].r = 255-(100*r_max);
//        point_cloud_ptr->points[i].g = 0;
//        point_cloud_ptr->points[i].b = 0;
    }
}



void publishLabeledCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr, ros::Publisher inputPub)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*point_cloud_ptr, cloud_msg);
    inputPub.publish(cloud_msg);
}



Mat getNextImage(Mat inputImg, VideoCapture &cap)
{
    Mat nextFrame;
    if(image_node)
        return inputImg;

    if(topicInput_on == 0 && !cap.isOpened())
        cap.open(0);

    if(topicInput_on == 0 && cap.isOpened())
        cap >> nextFrame;

    return nextFrame;
}


Mat intiImage(int argc, char** argv, VideoCapture &cap)
{
    Mat inputImg;

    if( argc >= 2 )
    {
        inputImg = imread(argv[1]);
        image_node = true;
    }
//    else if()
//    {
//        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
//        cap >> inputImg;
//    }

    if(argc >= 2 && inputImg.empty())
    {
        cout << "can not open Image, program terminate \n";
        if(cap.isOpened())
        cap.release();

        exit(1);
    }

    return inputImg;
}

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    tmp.copyTo(out);
 //   cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
}
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Feature Detecting Functions ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

Mat SIFT_Detector::startDetect(Mat inputImg)
{
//    cout << "SIFT_Detector \n";
    Mat result;
    vector<KeyPoint>keypoints_1, keypoints_2;


    Ptr<Feature2D> f2d = SIFT::create();

 //   f2d->detectAndCompute(inputImg, NULL);
    f2d->detect( inputImg, keypoints_1 );
//    f2d->detect( inputImg, keypoints_2 );

    Mat descriptors_1, descriptors_2;
    f2d->compute( inputImg, keypoints_1, descriptors_1 );


    f2d->compute( inputImg, keypoints_2, descriptors_2 );

//    BFMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_1, descriptors_2, matches );

    drawKeypoints(inputImg, keypoints_1, result);

 //   imshow("sift", result);
    return result;
}

Mat SURF_Detector::startDetect(Mat inputImg)
{
 //   cout << "SURF_Detector \n";
    Mat result;
    vector<KeyPoint>keypoints_1, keypoints_2;


    Ptr<Feature2D> f2d = SURF::create();
    f2d->detect( inputImg, keypoints_1 );

    Mat descriptors_1, descriptors_2;
    f2d->compute( inputImg, keypoints_1, descriptors_1 );

    drawKeypoints(inputImg, keypoints_1, result);

    return result;
}

Mat SLIC_SuperPixel_Detector::startDetect(Mat inputImg)
{
    SLIC slic;
	Mat result;

	slic.GenerateSuperpixels(inputImg, num_superpixels);

    if (inputImg.channels() == 3)
		return slic.GetImgWithContours(Scalar(0, 0, 255));
	else
		return slic.GetImgWithContours(Scalar(128));
}

Mat OpenCV_SuperPixel_Detector::startDetect(Mat inputImg)
{
//    cout << "OpenCV_SuperPixel_Detector \n";
	int width, height;
	Mat mask, result;

	if( !init )
	{
//	    cout << "number of superPixel: " << num_superpixels << "\n";
//		cout << "inside init \n";
		bool double_step = false;
        int num_histogram_bins = 5;

		width = inputImg.size().width;
		height = inputImg.size().height;
		seeds = createSuperpixelSEEDS(width, height, inputImg.channels(), num_superpixels, num_levels, prior, num_histogram_bins, double_step);

		init = true;
	}

	Mat converted;
	cvtColor(inputImg, converted, COLOR_BGR2HSV);

	double t = (double) getTickCount();

	seeds->iterate(converted, num_iterations);
	result = inputImg.clone();

	t = ((double) getTickCount() - t) / getTickFrequency();
//	printf("SEEDS segmentation took %i ms with %3i superpixels\n",
//			(int) (t * 1000), seeds->getNumberOfSuperpixels());

	/* retrieve the segmentation result */
	Mat labels;
	seeds->getLabels(labels);

	/* get the contours for displaying */
	seeds->getLabelContourMask(mask, false);

    result.setTo(Scalar(0, 0, 255), mask);

	return result;
}

Mat CV_DFT::startDetect(Mat inputImg)
{
//    cout << "DFT \n";
    Mat padded, gray;
    cvtColor(inputImg, gray, CV_BGR2GRAY);                          //expand input image to optimal size
    int m = getOptimalDFTSize( gray.rows );
    int n = getOptimalDFTSize( gray.cols ); // on the border add zero values
    copyMakeBorder(gray, padded, 0, m - gray.rows, 0, n - gray.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix
    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).

//    imshow("Input Image"       , I   );    // Show the result
//    imshow("spectrum magnitude", magI);
//    waitKey(0);
    return magI;
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Display Window Related Functions  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void initDisplayWindow(string window_name)
{
    namedWindow(window_name, WINDOW_AUTOSIZE );
    namedWindow(window_control_name, WINDOW_AUTOSIZE );
 //   resizeWindow( window_control_name, 50, 500 );

    moveWindow( window_name, 0, 0 );
    moveWindow( window_control_name, 1000, 0 );

    createTrackbar("Number",    window_name,          &num_superpixels,   1000,   superPixelNumChanged);

    createTrackbar("Topic_In",  window_control_name,  &topicInput_on,     1,      videoControl);

    createTrackbar("CV_SP",     window_control_name,  &CV_SuperPixel_on,  1,      resetTask);
    createTrackbar("SLIC_SP",   window_control_name,  &SLIC_SuperPixel_on,1,      resetTask);
    createTrackbar("SIFT",      window_control_name,  &SIFT_on           ,1,      resetTask);
    createTrackbar("SURF",      window_control_name,  &SURF_on           ,1,      resetTask);

    createTrackbar("DFT",       window_control_name,  &CV_DFT_on           ,1,      resetTask);
}

void initParamater()
{
    sift_detector       = new SIFT_Detector();
    surf_detector       = new SURF_Detector();
    slic_sp_detector    = new SLIC_SuperPixel_Detector();
    cv_sp_detector      = new OpenCV_SuperPixel_Detector();
    cv_dft              = new CV_DFT();

    detector._detectMethod = NULL;

    CV_SuperPixel_on    = 0;
    SLIC_SuperPixel_on  = 0;
    SIFT_on             = 0;
    SURF_on             = 0;
    topicInput_on       = 1;
    CV_DFT_on           = 0;

    num_superpixels     = 50;
    num_levels          = 2;
    prior               = 4;
    num_iterations      = 4;

    init = false;
}

void videoControl(int, void*)
{
    if(image_node == true)
        return;

    cout << "topic ON = " << topicInput_on << endl;
    cout << "cat is opened: " << cap.isOpened() << endl;

    if(topicInput_on == 0 && cap.isOpened() == false)  /* use camera */
        cap.open(0);

    if(topicInput_on == 1 && cap.isOpened() == true)
     {
        cout << "turn of camera " << endl;
        cap.release();
     }
}

void superPixelNumChanged(int, void*)
{
    init = false;
}

/* trackbarChanged_callBack(int, void*) */
void resetTask(int, void*)
{
    string currentRunningTaskName;

    if(detector._detectMethod != NULL)
    {
        currentRunningTaskName = detector.get_currentTaskName();
        setTrackbarPos(currentRunningTaskName, window_control_name, 0);
        detector._detectMethod = NULL;
    }

    if(CV_SuperPixel_on == 1)
        detector.set_detectMethod(cv_sp_detector);

    if(SLIC_SuperPixel_on == 1)
        detector.set_detectMethod(slic_sp_detector);

    if(SIFT_on == 1)
        detector.set_detectMethod(sift_detector);

    if(SURF_on == 1)
        detector.set_detectMethod(surf_detector);

    if(CV_DFT_on == 1)
        detector.set_detectMethod(cv_dft);

}
