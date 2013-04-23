/**
 * @file /include/HaarDispAda/haardispada.hpp
 *
 * @brief Communications central!
 *
 * @date November 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HAARDISPADA_HPP
#define HAARDISPADA_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <map>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <roi_msgs/Rois.h>
#include <roi_msgs/RoiRect.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HaarDispAda {
using  std::string;
using  std::vector;
using  cv::Mat;
using  cv::Rect;
using  cv::Range;
using  cv::Size;
using  cv::INTER_AREA;

  /*****************************************************************************
   ** Class
   *****************************************************************************/

  class HaarDispAdaClassifier{

  public:
    HaarDispAdaClassifier();
    HaarDispAdaClassifier(string file_name);
    void init();
    bool useMissingDataMask_;
    bool loaded;
    void load(string filename);
    int addToTraining(vector<Rect> &R_in, vector<int> &Labels_in, Mat &Disparity_in);
    void train(string filename);
    void detect(vector<Rect> &R_in, 
		vector<int> &L_in,
		Mat &D_in, 
		vector<Rect> &R_out, 
		vector<int> &L_out,
		bool label_all);

    int test();
    int numSamples_;    
    float HaarDispAdaPrior_;
    void setMaxSamples(int n);
    int getMaxSamples(){ return(maxSamples_);};

  private:
    CvBoost HDAC_;
    string classifier_filename_; // for loading and saving
    int maxSamples_;
    int num_filters_;
    Mat integralImage_;
    Mat trainingSamples_;
    Mat trainingLabels_;
    Mat trainingMissingMask_;
    Mat AvePosTrainingImg_;
    
    // images set by alpah_map() 
    Mat map; // set by alpha_map() in haar_response()

    // images set by setDImageRoi()
    Mat Image4Haar;  
    Mat haar16x16;
    Mat haar8x8;
    Mat haar4x4;

    // private functions
    void setDImageRoi(Rect &R_in, Mat &I_in);
    void setDImageROI_fast(Rect & R_in, Mat &I_in);
    void alpha_map(int idx);
    void print_map(int k);// a debugging tool
    void print_scaled_map(cv::Mat &A);// a debugging tool
    void print_Image4Haar();// a debugging tool
    int haar_features(Mat & HF, Mat & MH);
    int haar_features_fast(Mat & HF);
    void mask_scale(Mat & input, Mat & output);
    float find_central_disparity(int x, int y, int height, int width, Mat& D_in);
  };


}  // namespace HaarDispAda

#endif /*  HAARDISPADA_HPP */

