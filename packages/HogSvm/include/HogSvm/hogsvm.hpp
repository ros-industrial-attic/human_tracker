/**
 * @file /include/HogSvm/hogsvm.hpp
 *
 * @brief Communications central!
 *
 * @date November 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HOGSVM_HPP
#define HOGSVM_HPP

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

namespace HogSvm {
using  std::string;
using  std::vector;
using  cv::Mat;
using  cv::Rect;
using  cv::Range;
using  cv::Size;
using  cv::INTER_AREA;
using  cv::FileStorage;

  /*****************************************************************************
   ** Class
   *****************************************************************************/

  class HogSvmClassifier{

  public:
    HogSvmClassifier();
    void init(string cf, string bf);
    bool loaded;
    void load();
    int addToTraining(vector<Rect> &R_in, vector<int> &Labels_in,  Mat &Image_in);
    void train(string filename);
    void detect(vector<Rect> &R_in, 
		vector<int>  &L_in,
		Mat &I_in, 
		vector<Rect> &R_out, 
		vector<int> &L_out,
		bool label_all);

    int test();
    int numSamples_;    
    bool initialized_;
    bool trained_;
    float HogSvmThreshold_;
    bool Save_Average_Roi_; // used to accumulate an averager image for setting hog blocks
    string Ave_Roi_File_;	// full path name of where to store results
    void setMaxSamples(int n);
    int getMaxSamples(){return(maxSamples_);};

  private:
    int cell_size;
    CvSVM HSC_;
    string detectDir_; // base directory containing both classifier and blocks xml files
    string classifier_filename_; // for loading and saving
    string blocks_filename_;     // defines Hog blocks
    int maxSamples_;
    int num_features_;
    Mat integralImage_;
    Mat trainingSamples_;
    Mat trainingLabels_;
    Mat AvePosTrainingImg_;
    
    // images used by alpah_map() 
    Mat map; // set by alpha_map() in hog_response()
    Mat WaveletResponse; // set by resize of map in hog_response()

    // images used by setImageRoi()
    Mat Scaled_Image; // resize of Cropped image in setImageRoi()to scale to 64x64
    Mat Bit8_Image; // convertTo of Scaled_Image in  SetImageRoi() to make 8 bits
    Mat Equal_Gray_Image; // equalizeHist of Bit8_image in SetImageRoi()
    Mat Ave_Roi;	  // average of Equal_Gray_Image
    Mat Image4Hog;  // cvtTo of Equal_Gray_Image of in SetImageRoi() to make 16 bits
    Mat DirImage;   // direction image
    Mat MagImage;   // magnitude image
    int num_ave_rois_;          // counter for averaging

    Mat HB_; // locations in the ROI, scaled to 64x64 where to features
    // private functions
    void setImageRoi(Rect R_in, Mat I_in);
    void alpha_map(int idx, Mat output);
    void print_map(int k, Mat map);// a debugging tool
    int hog_features(Mat & HF);
    void compute_hog_feature_block(int x, int width, int y, int height, Mat &HF);
    void compute_hog_feature_cell(int x, int y,Mat &hcf);
  };


}  // namespace HogSvm

#endif /*  HOGSVM_HPP */

