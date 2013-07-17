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

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <stdlib.h>
#include <algorithm>
#include <functional>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <consistency/consistency.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using namespace std;
using namespace cv;

namespace Cten {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  Consistency::Consistency(){

    minAspect = 0.30;
    maxAspect = 1.30;
    stepAspect = 0.85;

    minDisparity = 3.0;

    minX=10;
    maxX=630;
    minY=180;
    maxY=400;
    stepX=0.33;			
    stepY=0.33;

    yhLowerM=0.0;
    yhLowerB=180.0;
    yhUpperM=-0.33;
    yhUpperB=300.0;

    dhLowerM=0.13;
    dhLowerB=-2.0;
    dhUpperM=0.28;
    dhUpperB=2.0;

    minHeight = 64;
    maxHeight = 281;
    stepHeight = 0.9;

    TD_.clear();
  }

  void Consistency::load(){

    FileStorage fs(ymlFilename.c_str(),FileStorage::READ);

    fs["minAspect"]>>minAspect;
    fs["maxAspect"]>>maxAspect;
    fs["stepAspect"]>>stepAspect;

    fs["minDisparity"]>>minDisparity;

    fs["minX"]>>minX;
    fs["maxX"]>>maxX;
    fs["stepX"]>>stepX;
    fs["minY"]>>minY;
    fs["maxY"]>>maxY;
    fs["stepY"]>>stepY;



    fs["yhLowerM"]>>yhLowerM;
    fs["yhLowerB"]>>yhLowerB;
    fs["yhUpperM"]>>yhUpperM;
    fs["yhUpperB"]>>yhUpperB;

    fs["dhLowerM"]>>dhLowerM;
    fs["dhLowerB"]>>dhLowerB;
    fs["dhUpperM"]>>dhUpperM;
    fs["dhUpperB"]>>dhUpperB;

    fs["minHeight"]>>minHeight;
    fs["maxHeight"]>>maxHeight;
    fs["stepHeight"]>>stepHeight;

    if(minX<0) minX=0.0;
    if(minY<0) minY=0.0;
    if(minHeight<0) minHeight=0.0;
    if(minAspect<0.01) minAspect=.01;

    if(stepX<=0.0 || stepX>1.0) stepX = 0.33;
    if(stepY<=0.0 || stepY>1.0) stepY = 0.33;
    if(stepHeight<=0.0 || stepHeight>1.0) stepHeight = .9;
    if(stepAspect<=0.0 || stepAspect>1.0) stepAspect = .9;
  }
  void Consistency::save(){

    FileStorage fs(ymlFilename.c_str(),FileStorage::WRITE);

    fs<<"minAspect"<<minAspect;
    fs<<"maxAspect"<<maxAspect;
    fs<<"stepAspect"<<stepAspect;

    fs<<"minDisparity"<<minDisparity;

    fs<<"minX"<<minX;
    fs<<"maxX"<<maxX;
    fs<<"stepX"<<stepX;
    fs<<"minY"<<minY;
    fs<<"maxY"<<maxY;
    fs<<"stepY"<<stepY;

    fs<<"yhLowerM"<<yhLowerM;
    fs<<"yhLowerB"<<yhLowerB;
    fs<<"yhUpperM"<<yhUpperM;
    fs<<"yhUpperB"<<yhUpperB;

    fs<<"dhLowerM"<<dhLowerM;
    fs<<"dhLowerB"<<dhLowerB;
    fs<<"dhUpperM"<<dhUpperM;
    fs<<"dhUpperB"<<dhUpperB;

    fs<<"minHeight"<<minHeight;
    fs<<"maxHeight"<<maxHeight;
    fs<<"stepHeight"<<stepHeight;

  }
  void Consistency::accumulate()
  {
    cvRoisOutput.clear();
    labelsOutput.clear();

    for(unsigned int i=0;i<cvRoisInput.size();i++){
      cvRoisOutput.push_back(cvRoisInput[i]);
      labelsOutput.push_back(labelsInput[i]);
      if(labelsInput[i] == 1){		// only retain positive examples
        int centerx = cvRoisInput[i].x + cvRoisInput[i].width/2;
        int centery = cvRoisInput[i].y + cvRoisInput[i].height/2;
        if(cvRoisInput[i].x<0 ||cvRoisInput[i].y<0){
          ROS_ERROR("Invalid ROI, negative x or y");
        }
        else if(cvRoisInput[i].width + cvRoisInput[i].x > disparity_image.cols){
          ROS_ERROR("Invalid ROI, width");
        }
        else if(cvRoisInput[i].height<=0 || cvRoisInput[i].width<=0){
          ROS_ERROR("Invalid ROI, negative height or width");
        }
        else if(centerx > disparity_image.cols || centery > disparity_image.rows){
          ROS_ERROR("Invalid ROI, center %d %d",centerx,centery);
        }
        else{
          training_info tf;
          tf.x            = cvRoisInput[i].x;
          tf.y            = cvRoisInput[i].y;
          tf.height       = cvRoisInput[i].height;
          tf.width        = cvRoisInput[i].width;
          tf.aspect_ratio = ((float)tf.width)/((float)tf.height);
          //	  tf.disparity_c  = (int)disparity_image.at<float>(centery,centerx);
          tf.disparity_c  = (int)find_central_disparity(tf.x,tf.y,tf.height,tf.width);
          tf.disparity_a  = find_average_disparity(tf.x,tf.y,tf.height,tf.width);
          if(tf.disparity_c >= minDisparity) TD_.push_back(tf);
        }
      }
    }
  }
  float Consistency::find_average_disparity(int x, int y, int height, int width)
  {
    int sum = 0;
    int count = 0;

    // Insure its a reasonable roi
    if(x+width>disparity_image.cols || y+height>disparity_image.rows) return(0.0);

    for(int i=0;i<height;i++){// y is rows
      for(int j=0;j<width;j++){// x is cols
        sum +=(int)disparity_image.at<float>(y+i,x+j);
        count++;
      }
    }
    float ad = (float) sum/ (float)count;
    return(ad);
  }

  float Consistency::find_central_disparity(int x, int y, int height, int width)
  {
    // Insure its a reasonable roi
    if(x+width>disparity_image.cols || y+height>disparity_image.rows) return(0.0);

    float ad=0.0;
    int j=width/2.0;// use central value for x
    for(int i=0;i<height/3;i++){// use middle third of y range
      if(disparity_image.at<float>(y+i,x+j)>ad){
        ad = disparity_image.at<float>(y+i,x+j);
      }
    }

    return(ad);
  }

  void Consistency::train()
  {
    // this function does the following
    // determines range of x,y,h and aspect ratio
    // determines heteroscedastic models for disparity vs height 
    // determines heteroscedastic models for         y vs height 
    // this fills in the following constraints
    // minX,maxX,minY,maxY,minAspect,maxAspect,minH,maxH
    // dhLowerM,dhUpperM,dhLowerB,dhUpperB
    // yhLowerM,yhUpperM,yhLowerB,yhUpperB

    minX = TD_[0].x;
    maxX = TD_[0].x;
    minY = TD_[0].y;
    maxY = TD_[0].y;
    vector<float> all_aspects;
    // find range in x and y and create a vector of aspect ratios
    for(unsigned int i=0;i<TD_.size();i++){
      if(minX>TD_[i].x) minX=TD_[i].x;
      if(minY>TD_[i].y) minY=TD_[i].y;
      if(maxX<TD_[i].x) maxX=TD_[i].x;
      if(maxY<TD_[i].y) maxY=TD_[i].y;
      float ar = (float)TD_[i].width/((float)TD_[i].height);
      all_aspects.push_back(ar);
    }
    float sum = std::accumulate(all_aspects.begin(),all_aspects.end(),0.0);
    float aspect_mean  = sum/all_aspects.size();
    float sq_sum = std::inner_product(all_aspects.begin(),all_aspects.end(),all_aspects.begin(),0.0);
    float aspect_sigma = sqrt(sq_sum/all_aspects.size()-aspect_mean*aspect_mean);
    minAspect = aspect_mean - 2*aspect_sigma;
    maxAspect = aspect_mean + 2*aspect_sigma;

    // model center disparity vs height
    cv::Mat samples(2,TD_.size(),CV_32FC1);
    for(unsigned int i=0;i<TD_.size();i++){
      samples.at<float>(0,i) = (float) TD_[i].height; // set x values to model
      samples.at<float>(1,i) = (float) TD_[i].disparity_c; // set y values to model
    }
    float nsd = 4.0;
    heteroscedastic_regression(samples,nsd,dhUpperM,dhUpperB,dhLowerM,dhLowerB,minHeight,maxHeight);
    std::string cd(getenv("NIST_CLASSIFIERS"));
    std::string fn;
    fn = cd + "/dvsh.m";
    h_model2Matlab(fn,"Disparity",samples,nsd,dhUpperM,dhUpperB,dhLowerM,dhLowerB,minHeight,maxHeight);

    // model y vs height
    for(unsigned int i=0;i<TD_.size();i++){
      samples.at<float>(0,i) = (float) TD_[i].height; // set x values to model
      samples.at<float>(1,i) = (float) TD_[i].y; // set y values to model
    }
    heteroscedastic_regression(samples,nsd,yhUpperM,yhUpperB,yhLowerM,yhLowerB,minHeight,maxHeight);

    fn = cd + "/yvsh.m";
    h_model2Matlab(fn,"Y value",samples,nsd,yhUpperM,yhUpperB,yhLowerM,yhLowerB,minHeight,maxHeight);

    save();			// write the classifier info

    // go ahead and copy the input rois to the output so they get passed along
    cvRoisOutput.clear();
    labelsOutput.clear();
    for(unsigned int i=0;i<cvRoisInput.size();i++){
      cvRoisOutput.push_back(cvRoisInput[i]);
      labelsOutput.push_back(labelsInput[i]);
    }
  }
  void  Consistency::h_model2Matlab(string filename, string yaxis, cv::Mat &samples, float &num_std_dev,
      float &m1, float &b1,
      float &m2, float &b2,
      float &min_x, float &max_x)
  {
    FILE *fp;
    fp = fopen(filename.c_str(),"w");
    fprintf(fp,"num_std_dev = %f;\n",num_std_dev);
    fprintf(fp,"m1 = %f;\n",m1);
    fprintf(fp,"b1 = %f;\n",b1);
    fprintf(fp,"m2 = %f;\n",m2);
    fprintf(fp,"b2 = %f;\n",b2);
    fprintf(fp,"min_x = %f;\n",min_x);
    fprintf(fp,"max_x = %f;\n",max_x);
    fprintf(fp,"samples = [\n");
    for(int i=0;i<samples.cols;i++){
      fprintf(fp,"%f %f;\n",samples.at<float>(0,i),samples.at<float>(1,i));
    }
    fprintf(fp,"];\n");
    fprintf(fp,"x=min_x:1:max_x;\n");
    fprintf(fp,"y1=m1*x+b1;\n");
    fprintf(fp,"y2=m2*x+b2;\n");
    fprintf(fp,"figure()\n");
    fprintf(fp,"plot(samples(:,1),samples(:,2),'+')\n");
    fprintf(fp,"hold\n");
    fprintf(fp,"plot(x,y1,'r')\n");
    fprintf(fp,"plot(x,y2,'b')\n");
    fprintf(fp,"xlabel('Height of ROI');\n");
    fprintf(fp,"ylabel('%s');\n",yaxis.c_str());
    fclose(fp);


  }


  void  Consistency::heteroscedastic_regression(cv::Mat &samples, float &num_std_dev,
      float &m1, float &b1,
      float &m2, float &b2,
      float &min_x, float &max_x)
  {
    cv::Mat A;
    cv::Mat B;

    // do a line fit on the data and find min and max values
    max_x=samples.at<float>(0,0);
    min_x=samples.at<float>(0,0);
    if(samples.rows==2){ // each column is a sample
      A.create(samples.cols,2,CV_32FC1);
      B.create(samples.cols,1,CV_32FC1);
      for(int i=0;i<samples.cols;i++){
        A.at<float>(i,0) = 1;
        A.at<float>(i,1) = samples.at<float>(0,i);
        B.at<float>(i,0) = samples.at<float>(1,i);
        if(max_x<samples.at<float>(0,i)) max_x = samples.at<float>(0,i);
        if(min_x>samples.at<float>(0,i)) min_x = samples.at<float>(0,i);
      }
    }
    else if(samples.cols==2){ // each row is a sample
      A.create(samples.rows,2,CV_32FC1);
      B.create(samples.rows,1,CV_32FC1);
      for(int i=0;i<samples.cols;i++){
        A.at<float>(i,0) = 1;
        A.at<float>(i,1) = samples.at<float>(i,0);
        B.at<float>(i,0) = samples.at<float>(i,1);
        if(max_x<samples.at<float>(i,0)) max_x = samples.at<float>(i,0);
        if(min_x>samples.at<float>(i,0)) min_x = samples.at<float>(i,0);
      }
    }
    else{
      ROS_ERROR("INVALID DATA in Heteroscedastic_regression()");
      return;
    }
    cv::Mat X(2,1,CV_32FC1);	// X is the terms(m & b) for y = b + m * x 
    cv::Mat R;			// R is the terms(m & b) for e = b + m * x 
    float e;
    bool is_ok = cv::solve(A,B,X,cv::DECOMP_SVD);

    // Now we compute the linear regression model for the error of this linear model of the data
    // that's a whole lot of repeated words, I hope they make sense
    if(is_ok){
      for(int i=0;i<B.rows;i++){
        // compute error of each term
        e = X.at<float>(0,0)+X.at<float>(1,0)*A.at<float>(i,1) - B.at<float>(i,0);
        // A stays the same
        B.at<float>(i,0) = fabs(e);
      }
      is_ok = cv::solve(A,B,R,cv::DECOMP_SVD);
    }
    else{
      ROS_ERROR("Initial REGRESSION FAILED IN Heteroscedastic_regression()");
    }
    // This second line shows how the error in the model varies with x
    // Nice data would have zero slope, and the y intercept is sigma
    // The two desired models models are determined from this as follows
    m1 = X.at<float>(0,1) + fabs(R.at<float>(0,1));// upper bounds
    b1 = X.at<float>(0,0) + num_std_dev*fabs(R.at<float>(0,0));
    m2 = X.at<float>(0,1) - fabs(R.at<float>(0,1)); //lower bounds
    b2 = X.at<float>(0,0) - num_std_dev*fabs(R.at<float>(0,0));

  }
  void Consistency::detect()
  {
    // create rois that are consistent with constraints throughout the image
    cvRoisOutput.clear();
    labelsOutput.clear();
    static int q=0;
    int y_offset,x_offset;
    if(q==0){
      y_offset=0;
      x_offset=0;
    }
    if(q==1){
      y_offset = 1;
      x_offset = 0;
    }
    if(q==2){
      y_offset = 0;
      x_offset = 1;
    }
    if(q==3){
      y_offset = 1;
      x_offset = 1;
    }
    q++;
    if(q>=4) q=0;

    for(float ar=maxAspect;ar>=minAspect;ar*=stepAspect){ // iterate over range of aspect ratios
      for(int h=maxHeight;h>=minHeight;h*=stepHeight){ // iterate over range of heights
        int uly = (int)( yhUpperB+yhUpperM*h);// find upper bounds on y
        int lly = (int)( yhLowerB+yhLowerM*h);// find lower bounds on y
        if(lly<0) lly=0;
        if(uly>=disparity_image.rows-h) uly=disparity_image.rows-h;
        float ul = dhUpperB + dhUpperM*h;
        float ll = dhLowerB + dhLowerM*h;
        int w = (int)(h*ar);
        for(int y=lly+y_offset*h*stepY;y<=uly;y+=2*stepY*h){ // iterate over shortend range in y
          for(int x=minX+x_offset*w*stepX;x<=maxX;x+=2*stepX*w){ // iterate over range of x values
            // window is at x,y with h and aspect ratio
            if(x+w<disparity_image.cols &&  y+h<disparity_image.rows){
              //	      int centerx = x + w/2;
              //	      int centery = y + h/2;
              //	      float cd = disparity_image.at<float>(centery,centerx);
              float cd = find_central_disparity(x,y,h,w);
              if(cd <= ul && cd >= ll && cd >= minDisparity){
                // create a roi and add to list
                CvRect cvRoi;
                cvRoi.x = (int)x;
                cvRoi.y = y;
                cvRoi.height = h;
                cvRoi.width = w;
                cvRoisOutput.push_back(cvRoi);
                labelsOutput.push_back(label);
              }
            }// end if reasonable x,y,h,w
          }// end iterations over y
        }// end eterations over x
      }// end iterations over height
    }// end iterations over aspect ratio

  }// end detect



} // namespace Cten
