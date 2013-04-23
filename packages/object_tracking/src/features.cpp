#include <object_tracking/features.h>

  features::features()
  {
      featurePool1 = cv::Mat(1, 27 + 48, CV_16S);
  }

  features::~features()
  {
  }

  void features::computePool1(cv::Mat clr, CvRect roi)
  {
	  cv::Mat gray;
	cv::cvtColor(clr,gray,CV_BGR2GRAY);
    haarSignFeatures(gray, roi);  // first 174 features of pool 1
    regionColorFeatures(clr, roi);
  }

  void features::haarSignFeatures(cv::Mat gray, CvRect roi)
  {
    // scale to 4x4 to get the 27 features
    cv::Mat object = gray(roi).clone();
    cv::Mat temp;
    cv::resize(object,temp,cv::Size(4,4),0,0,CV_INTER_AREA);
    temp.convertTo(temp,CV_16S);

    int idx = 0;
    // Horizontal features
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, temp.at<int16_t>(i,j)
        + temp.at<int16_t>(i+1,j)
        - temp.at<int16_t>(i,j+1)
        - temp.at<int16_t>(i+1,j+1));

    // Vertical features
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, temp.at<int16_t>(i,j)
        - temp.at<int16_t>(i+1,j)
        + temp.at<int16_t>(i,j+1)
        - temp.at<int16_t>(i+1,j+1));

    // Diagonal features
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, temp.at<int16_t>(i,j)
        - temp.at<int16_t>(i+1,j)
        - temp.at<int16_t>(i,j+1)
        + temp.at<int16_t>(i+1,j+1));
  }

  void features::regionColorFeatures(cv::Mat clr, CvRect roi)
  {
	  // Scale to 4x4 to get 48 features (each box: R, G, B > 128)
	  cv::Mat object = clr(roi).clone();
	  cv::Mat temp;
	  cv::resize(object,temp,cv::Size(4,4),0,0,CV_INTER_AREA);

	  int idx = 27; // start where haar sign features leave off
	  for(int i = 0; i < 4; i++) for (int j = 0; j < 4; j++)
	  {
		  featurePool1.at<int16_t>(0,idx++) = (temp.at<cv::Vec3b>(i,j)[0] > 128);
		  featurePool1.at<int16_t>(0,idx++) = (temp.at<cv::Vec3b>(i,j)[1] > 128);
		  featurePool1.at<int16_t>(0,idx++) = (temp.at<cv::Vec3b>(i,j)[2] > 128);
	  }
  }

  void features::computeMaps(cv::Mat gray)
  {
    // kernels
    float diag_data[] = {1, -1, -1, 1};
    float horz_data[] = {1, -1, 1, -1};
    float vert_data[] = {1, 1, -1, -1};

    cv::Mat diag_kernel = cv::Mat(2,2, CV_16S, diag_data);
    cv::Mat horz_kernel = cv::Mat(2,2, CV_16S, horz_data);
    cv::Mat vert_kernel = cv::Mat(2,2, CV_16S, vert_data);

    cv::filter2D(gray, diagMap, CV_16S, diag_kernel, cv::Point(0,0));
    cv::filter2D(gray, horzMap, CV_16S, horz_kernel, cv::Point(0,0));
    cv::filter2D(gray, vertMap, CV_16S, vert_kernel, cv::Point(0,0));
  }

  void features::writePool1FromMaps(int i, int j)
  {
    int idx = 0;
    // Horizontal features
    for(int ii = 0; ii < 3; ii++)
      for(int jj = 0; jj < 3; jj++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, horzMap.at<int16_t>(i+ii,j+jj));
    // Vertical features
    for(int ii = 0; ii < 3; ii++)
      for(int jj = 0; jj < 3; jj++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, vertMap.at<int16_t>(i+ii,j+jj));
    // Diagonal features
    for(int ii = 0; ii < 3; ii++)
      for(int jj = 0; jj < 3; jj++)
        featurePool1.at<int16_t>(0,idx++) = copysign(1, diagMap.at<int16_t>(i+ii,j+jj));
  }

