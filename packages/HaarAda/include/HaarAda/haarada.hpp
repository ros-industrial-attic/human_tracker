/**
 * @file /include/HaarAda/haarAda.hpp
 *
 * @brief Communications central!
 *
 * @date November 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HAARADA_HPP
#define HAARADA_HPP

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

namespace HaarAda {
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

  class HaarAdaClassifier{

  public:
    HaarAdaClassifier();
    HaarAdaClassifier(string file_name);
    void init();
    bool loaded;
    void load(string filename);
    int addToTraining(vector<Rect> R_in, vector<int> Labels_in, Mat Image_in);
    void train(string filename);
    void detect(vector<Rect> &R_in, 
		vector<int> &L_in,
		Mat &I_in, 
		vector<Rect> &R_out, 
		vector<int> &L_out,
		bool label_all);

    int test();
    int numSamples_;    
    float HaarAdaPrior_;
    void setMaxSamples(int n);
    int getMaxSamples(){ return(maxSamples_);};
  private:
    CvBoost HAC_;
    string classifier_filename_; // for loading and saving
    int maxSamples_;
    int num_filters_;
    Mat integralImage_;
    Mat trainingSamples_;
    Mat trainingLabels_;
    Mat AvePosTrainingImg_;
    
    // images used by alpah_map() 
    Mat map; // set by alpha_map() in haar_response()
    Mat WaveletResponse; // set by resize of map in haar_response()

    // images used by setImageRoi()
    Mat Scaled_Image; // resize of Cropped image in setImageRoi()to scale to 64x64
    Mat Bit8_Image; // convertTo of Scaled_Image in  SetImageRoi() to make 8 bits
    Mat Equal_Gray_Image; // equalizeHist of Bit8_image in SetImageRoi()
    Mat Image4Haar;  // cvtTo of Equal_Gray_Image of in SetImageRoi() to make 16 bits

    // faster haar
    Mat haar16x16;
    Mat haar8x8;
    Mat haar4x4;

    // private functions
    void setImageRoi(Rect R_in, Mat I_in);
    void setImageROI_fast(Rect R_in, Mat I_in);
    void alpha_map(int idx);
    void print_map(int k);// a debugging tool
    int haar_features(Mat & HF);
    int haar_features_fast(Mat &HF);
    void computeIntegralImage(Mat image_in); // currently unused
  };


}  // namespace HaarAda

#endif /*  HAARADA_HPP */

