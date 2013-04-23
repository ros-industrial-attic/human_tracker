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
#ifndef FEATURES_H_
#define FEATURES_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>

  /**
   * @brief Calculate and store features for online learning.
   */
  class features{

  public:
    cv::Mat featurePool1;
    cv::Mat horzMap;
    cv::Mat vertMap;
    cv::Mat diagMap;
    /**
     * @brief Create features
     */
    features();

    /**
     * @brief destroy the class
     */
    ~features();

    /**
     * @brief Compute the feature pools
     * @param[in] gray The input image
     * @param[in] roi The input roi
     */
    void computePool1(cv::Mat gray, CvRect roi);

    /**
     * @brief Compute haar sign features
     * @param[in] gray The input image
     * @param[in] roi The input roi
     */
    void haarSignFeatures(cv::Mat gray, CvRect roi);

    /**
     * @brief Compute the color features
     * @param[in] clr The input image
     * @param[in] roi The input roi
     */
    void regionColorFeatures(cv::Mat clr, CvRect roi);

    /**
     * @brief Compute maps for pool features
     */
    void computeMaps(cv::Mat gray);

    /**
     * @brief Compute the feature pools
     * @param[in] i index of map to find features for
     * @param[in] j index of map to find features for
     */
    void writePool1FromMaps(int i, int j);
  private:
  };  // class features

#endif // FEATURES_H_
