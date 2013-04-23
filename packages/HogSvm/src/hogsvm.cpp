/*
Software License Agreement (BSD License)
 
Copyright (c) 2013, Southwest Research Institute
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "HogSvm/hogsvm.hpp"
#include <stdlib.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace HogSvm {

/*****************************************************************************
** Implementation
*****************************************************************************/
  HogSvmClassifier::HogSvmClassifier()
  {
    cell_size = 4;
    initialized_ = false;
    trained_     = false;
    HogSvmThreshold_=0.0;
    num_ave_rois_ = 0;
  }

  void HogSvmClassifier::detect(vector<Rect> &R_in, 
				vector<int> &L_in,
				Mat &I_in, 
				vector<Rect> &R_out, 
				vector<int> &L_out,
				bool label_all)
  {
    if(!trained_) return;
    int count =0;
    Mat HF(1,num_features_,CV_32F);

    R_out.clear();
    L_out.clear();
    for(unsigned int i=0;i<R_in.size();i++){
      float result = 0;
      if(R_in[i].width > 2 && R_in[i].height > 2){
	setImageRoi(R_in[i],I_in);
	if(hog_features(HF)== 1){
	  result = HSC_.predict(HF,true);
	}
        else{
	  ROS_ERROR("Couldn't compute hog features???");
          result = 0;
	}
      }
      if(result<HogSvmThreshold_ || label_all == true){
	R_out.push_back(R_in[i]);
	if(result<HogSvmThreshold_){
	  if(label_all) L_out.push_back(1);
	  if(!label_all) L_out.push_back(L_in[i]);
	  count++;
	}
	else{
	  if(label_all) L_out.push_back(0);
	  if(!label_all) L_out.push_back(L_in[i]);
	}
      }
    }
  }

  int HogSvmClassifier::addToTraining(vector<Rect> &R_in, vector<int> &L_in, Mat &I_in)
  {
    if(!initialized_) return(0);
    Mat HF(1,num_features_,CV_32F);
    for(unsigned int i = 0; i<R_in.size(); i++){
      if(R_in[i].width < 2 || R_in[i].height <2){
	// do nothing with really small rois
      }
      else if(R_in[i].x+R_in[i].width > I_in.cols ||R_in[i].y+R_in[i].height>I_in.rows){
	ROS_ERROR("invalid ROI %d %d %d %d",R_in[i].x,R_in[i].y,R_in[i].width,R_in[i].height);
      }
      else if(R_in[i].x<0 ||R_in[i].width <0 || R_in[i].y<0 || R_in[i].height<0){
	ROS_ERROR("negative ROI %d %d %d %d",R_in[i].x,R_in[i].y,R_in[i].width,R_in[i].height);
      }
      else if(numSamples_<maxSamples_){// not too many samples already
	setImageRoi(R_in[i],I_in);
	if(Save_Average_Roi_ && L_in[i] == 1){
	  num_ave_rois_++;
	  for(int q = 0; q<64; q++){
	    for(int k = 0; k<64; k++){
	      Ave_Roi.at<float>(q,k) += MagImage.at<float>(q,k);
	    }
	  }
	}
	int rtn = hog_features(HF);
	if(rtn){
	  for(int j=0;j<num_features_;j++){// copy the subset of samples 
	    trainingSamples_.at<float>(numSamples_,j) = HF.at<float>(0,j);
	  }
	  if(L_in[i] <= 0 ){
	    trainingLabels_.at<int>(numSamples_,0) = 0;//classes: 0,1  not -1,1 
	  }
	  else{
	    trainingLabels_.at<int>(numSamples_,0) = L_in[i];
	  }
	  numSamples_++;
	}// end if successful compute feature
      }// end of if not too many samples already
    }// end each roi
    return(numSamples_);
  }// end addToTraining

  void HogSvmClassifier::setImageRoi(Rect R_in, Mat I_in)
  {
    if(!initialized_) return;
    int xmin = R_in.x;
    int xmax = xmin+R_in.width;
    int ymin = R_in.y;
    int ymax = ymin+R_in.height;
    Mat Cropped_Image = I_in(Range(ymin,ymax),Range(xmin,xmax)); // extract ROI
    resize(Cropped_Image,Scaled_Image, Size(64,64), 0, 0,INTER_AREA ); // resize to 64x64
    Scaled_Image.convertTo(Bit8_Image,CV_8UC1); // convert to  8 bit gray scale
    equalizeHist(Bit8_Image, Equal_Gray_Image); // do histogram normalization

    // compute magnitude and direction images over ROI
    Mat Sx(64,64,CV_32FC1);
    Mat Sy(64,64,CV_32FC1);
    cv::Sobel(Equal_Gray_Image,Sx,CV_32FC1,1,0,3);
    cv::Sobel(Equal_Gray_Image,Sy,CV_32FC1,0,1,3);

    bool in_degrees=true;
    cv::cartToPolar(Sx,Sy,MagImage,DirImage,in_degrees);
    
  }

  void HogSvmClassifier::train(string filename)
  {
    if(!initialized_) return;
    CvMat* cWeights = cvCreateMat(1, 2, CV_32FC1);
    cvSet2D(cWeights, 0, 0, cvScalar(1));
    cvSet2D(cWeights, 0, 1, cvScalar(1));

    //  add | CV_TERMCRIT_ITER to first argument if training fails
    CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS, 100, 0.001); 
    CvSVMParams svmparams = CvSVMParams(CvSVM::C_SVC, CvSVM::POLY, 2, 1, 1, 1,
					0.5, 0.1, cWeights, criteria);

    // copy sub matrix for training 
    Mat VarIdx;
    Mat Features(numSamples_,num_features_,CV_32F);
    for(int i=0;i<numSamples_;i++){
      for(int j=0;j<num_features_;j++){
	Features.at<float>(i,j) = trainingSamples_.at<float>(i,j);
      }
    }
    Mat Responses(numSamples_,1,CV_32S);
    for(int i=0; i<numSamples_;i++){
      if(trainingLabels_.at<int>(i,0) == -1) trainingLabels_.at<int>(i,0) = 0;//classes: 0,1  not -1,1 
      Responses.at<int>(i,0) = trainingLabels_.at<int>(i,0);
    }
    HSC_.train(Features, Responses,Mat(),Mat(),svmparams);
    trained_ = true;
    ROS_ERROR("saving trained classifier to %s",filename.c_str());
    HSC_.save(filename.c_str());
    if(Save_Average_Roi_){
      Ave_Roi = Ave_Roi/num_ave_rois_;
      Mat tmp = cv::Mat::zeros(64,64,CV_8U);
      Ave_Roi.convertTo(tmp, CV_8U);
      cv::imwrite(Ave_Roi_File_.c_str(),tmp);
    }
    // Determine Recall Statistics
    int num_TP     = 0;
    int num_FP     = 0;
    int num_people = 0;
    int num_neg    = 0;
    int num_TN     = 0;
    int num_FN     = 0;
    for(int i=0;i<Features.rows;i++){
      float result = HSC_.predict(Features.row(i),true);
      if(Responses.at<int>(i,0) == 1) num_people++;
      if(Responses.at<int>(i,0) != 1) num_neg++;
      if(result<HogSvmThreshold_ && Responses.at<int>(i,0) == 1) num_TP++; // true pos
      else if(result<HogSvmThreshold_ && Responses.at<int>(i,0) == 0) num_FP++; // false pos
      else if(result>HogSvmThreshold_ && Responses.at<int>(i,0) == 0) num_TN++; // true neg
      else if(result>HogSvmThreshold_ && Responses.at<int>(i,0) == 1) num_FN++; // false neg
    }
    float percent = (float)num_TP/(float)num_people*100.0;
    ROS_ERROR("Recall = %6.2f%c",percent,'%');
    percent = (float)num_FP/(float)num_neg*100.0;
    ROS_ERROR("False Positives = %6.2f%c",percent,'%');
  }

  void HogSvmClassifier::setMaxSamples(int n)
  {
    maxSamples_ = n;
    numSamples_ =0;
    trainingSamples_.create(maxSamples_,num_features_,CV_32FC1);
    trainingLabels_.create(maxSamples_,1,CV_32SC1);
  }
  void HogSvmClassifier::init(string cf, string bf )
  {
    classifier_filename_ = cf;//"/HogSvm.xml";
    blocks_filename_     = bf;//"/Blocks.xml";
    num_features_ = 1200;
    setMaxSamples(350);
    AvePosTrainingImg_.create(64,64,CV_64F);
    
    // image of setImageRoi() set here since known size and type to avoid realloc
    Scaled_Image.create(64,64,CV_8UC1);
    Bit8_Image.create(64,64,CV_8UC1); 
    Equal_Gray_Image.create(64,64,CV_8UC1);
    Ave_Roi.create(64,64,CV_32FC1);
    DirImage.create(64,64,CV_32FC1);
    MagImage.create(64,64,CV_32FC1);

    // load blocks
    FileStorage fs;
    fs = FileStorage(blocks_filename_.c_str(), FileStorage::READ);
    if (!fs.isOpened())
    {
      ROS_ERROR("Failed to open blocks file %s.",blocks_filename_.c_str());
    }
    HB_.create(30,4,CV_8UC1);
    fs["HOG_MAT"] >> HB_;
    fs.release();

    // load classifier
    FileStorage fs2;
    fs2 = FileStorage(classifier_filename_.c_str(), FileStorage::READ);
    if (fs2.isOpened())
    {
      HSC_.load(classifier_filename_.c_str());      
      trained_ = true;
      fs2.release();
    }
    else{
      ROS_ERROR("COULD NOT LOAD %s",classifier_filename_.c_str());
    }

    initialized_ = true;
  }

  void HogSvmClassifier::load()
  {
    ROS_ERROR("loading classifier from %s",classifier_filename_.c_str());
    HSC_.load(classifier_filename_.c_str());
  }

  int HogSvmClassifier::test()
  {
    return(1);
  }

  int HogSvmClassifier::hog_features(Mat &HF)
  {
    if(!initialized_){
      ROS_ERROR("Not initialized in hog_features()");
      return(0);
    }
    if(DirImage.rows != 64 || DirImage.cols !=64)
      {
	ROS_ERROR("Wrong sz Polar Image hog_response %dX%d ",DirImage.rows,DirImage.cols);
	return(0);
      }
    if(HB_.cols != 4)
      {
	ROS_ERROR("Wrong sz of Hog Blocks %dX%d ",HB_.rows,HB_.cols);
	return(0);
      }
    Mat Block_feature(40,1,CV_32F);
    for (int k = 0; k < HB_.rows; k++) // do each block
      {
	int x      = (int)HB_.at<char>(k,0)*cell_size;	// must be less than 60
	int y      = (int)HB_.at<char>(k,1)*cell_size;  // must be less than 60
	int width  = (int)HB_.at<char>(k,2);            // must be either 1 or 2 (cells)
	int height = (int)HB_.at<char>(k,3);            // must be either 1 or 2 (cells)
	//	ROS_ERROR("Block %d x=%d y=%d h=%d w=%d",k,x,y,height,width);
	if(x>60 || y>60 || height*width !=4){
	  ROS_ERROR("INCORRECT BLOCK %d %d %d %d",x,width,y,height);
	  ROS_ERROR("HB_.type() = %d k = %d",HB_.type(),k);
	}
	else{
	  compute_hog_feature_block(x,width,y,height,Block_feature);
	  // copy into overall vec
	  for(int i=0;i<40;i++) HF.at<float>(0,k*40+i) = Block_feature.at<float>(i,0);
	}
      } 
    return(1);
  }// hog_features

  void HogSvmClassifier::compute_hog_feature_block(int x,int width,int y,int height,Mat &BF)
  {
    if(!initialized_){
      ROS_ERROR("NOT initialized in compute_hog_feature_block");
      return;
    }
    int num_cell_bins=10; // TODO should be configurable
    int q=0;
    for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
	Mat hcf(1,num_cell_bins,CV_32F);
        int x_index = x+i*cell_size;
	int y_index = y+j*cell_size;
	//	ROS_ERROR("cell loc xi= %d yi=%d",x_index,y_index);
	compute_hog_feature_cell(x_index,y_index,hcf);
        for(int k=0;k<num_cell_bins;k++) BF.at<float>(q++,0) = hcf.at<float>(0,k);
      }
    }
  }
  void HogSvmClassifier::compute_hog_feature_cell(int x, int y,Mat &hcf)
  {
    if(!initialized_){
      ROS_ERROR("NOT initialized in compute_hog_feature_cell");
      return;
    }

    #define PI 3.14159265359
    int num_cell_bins = 10;
    float ang_res = 360.0/num_cell_bins;    
    // this code assumes the DirImage and MagImage contain the magnitude and direction in radians
    // of the gradient of the 64x64 Image which is a ROI from the original image
    // NOTE, the direction is -pi to pi
    hcf = Mat::zeros(1,hcf.cols,CV_32F);
    for(int i=x;i<x+cell_size;i++){
      for(int j=y;j<y+cell_size;j++){
	unsigned int angle_index =(unsigned int)((DirImage.at<float>(i,j))/ang_res); // find index
	if(angle_index >= (unsigned int) hcf.cols){
	  ROS_ERROR("angle_index = %d is too big %f ",angle_index,DirImage.at<float>(i,j));
	}
	else{
	  //	  ROS_ERROR("Adding MagImage val = %12.2f",MagImage.at<float>(i,j));
	  hcf.at<float>(0,angle_index) += MagImage.at<float>(i,j);
	}
      }
    }
  }
  
} // namespace HogSvm
