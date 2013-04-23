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

#ifndef consistency_CONSISTENCY_HPP_
#define consistency_CONSISTENCY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <ros/ros.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Cten {

/*****************************************************************************
** Class
*****************************************************************************/

class Consistency{
 private:
  struct training_info{
    int x,y,height,width;
    int disparity_c;		// center disparity
    int disparity_a;		// average disparity
    float aspect_ratio;
  };

public:
  enum {MAX_LOOP_ITS=10};
  std::vector<training_info> TD_;
  int max_training_data;
  Consistency();
  void accumulate();
  void train();
  void detect();
  
  ///////////////////////////////////////////////////////////////////
  
  // Classifier Functions
  void load();
  void save();
  
  // Load message inputs
  std::vector<CvRect> cvRoisInput;
  std::vector<int>    labelsInput;

  cv::Mat color_image;
  cv::Mat disparity_image;
	
  // Message Outputs
  std::vector<CvRect> cvRoisOutput;
  std::vector<int>    labelsOutput;

  // Load Classifier Configuration
  std::string mode;
  int label;

  // YAML File Parameters
  std::string ymlFilename;		
	
  // NOTE: These could probably be private variables
  // Force Aspect Ratio	
  float minAspect;
  float maxAspect;
  float stepAspect;

  // Force Disparity Constraint
  float minDisparity;

  // Force ROI Constraint
  float minX;
  float maxX;
  float stepX;
  float minY;
  float maxY;
  float stepY;

  // Spatial Y vs Height Constraints
  float yhLowerM;
  float yhLowerB;
  float yhUpperM;
  float yhUpperB;

  // Disparity vs Height Constraints
  float dhLowerM;
  float dhLowerB;
  float dhUpperM;
  float dhUpperB;
	
  // Height Constraints
  float minHeight;
  float maxHeight;
  float stepHeight;
	
private:
  void  heteroscedastic_regression(cv::Mat &samples, float &num_std_dev,// input data
				   float &m1, float &b1,// upper linear model for data
				   float &m2, float &b2,// lower linear model for data
				   float &min_x, float &max_x);// range of linear model for data
  void  h_model2Matlab(std::string filename,std::string yaxis,cv::Mat &samples, float &num_std_dev,// input data
		       float &m1, float &b1,// upper linear model for data
		       float &m2, float &b2,// lower linear model for data
		       float &min_x, float &max_x);// range of linear model for data
  float find_average_disparity(int x, int y, int height, int width);
  float find_central_disparity(int x, int y, int height, int width);
  

};

}  // namespace consistency

#endif /* consistency_CONSISTENCY_HPP_ */


