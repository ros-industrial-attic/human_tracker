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
// Online learning and tracking of a specific object

// Tracking function should take x-z map and image itself for observations

// initialization should establish object size, aspect ratio

// particle tracker will calculate probability based on x-z map, appearance (each xz has a box associated with it)

// interface with node will be Initialize, Override, Update

// biggest question is how override should be done- could use it to amplify x-z map and then do regular update,
// could provide tools to score the override and let the node decide if it wants to override or not

// Need to include particle filter, features, forests - this is a particle tracking TLD implementation
// Will still add a raw detect, but the only purpose is to prune trees


#ifndef PARTICLE_TRACK_H_
#define PARTICLE_TRACK_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>

// Package files
#include <object_tracking/particle_filter.h>
#include <object_tracking/features.h>
#include <object_tracking/forest.h>


using namespace cv;
using namespace std;

  /**
   * @brief Track objects in disparity space with particle filter.
   */

  class ParticleTrack : public ParticleFilter
  {
    // TLD Stuff
    forest forest1;
    features features1;

    // Stereo setup stuff
    cv::Mat xz_map_;
    double xz_map_res_, xz_map_offset_;
    double angle_;
    cv::Mat R;
    double Q_32, Q_03, Q_13, Q_23, Q_33;

    // Object properties
    double h;
    double ar_;

    struct pos_time
    {
    	double dt;
    	cv::Point2f position;
    };
    std::vector<pos_time> position_history;
  public:
    double size_;

    cv::Point objectLocation;
    CvRect objectROI;
    CvRect searchROI;
    cv::Mat hist_;
    uint64_t uid;

    bool debug_mode_;  // if true, console output and visualization is active

    /**
     * @brief Construct a tracking object
     */
    ParticleTrack();

    /**
     * @brief Initialize a tracker
     * @param[in] input The input image
     * @param[in] inROI the tracking box
     */
    void initialize(cv::Mat& input, cv::Mat& disp, CvRect inROI, std::string filterDir,
        cv::Mat& Q, double angle);

    void revive(cv::Mat& input, cv::Mat& xz, CvRect inROI);

    /**
     * @brief Update the tracker state
     * @param[in] input The input image (next image in sequence)
     */
    int update_state(cv::Mat& input, cv::Mat& xz, double dt);

    /**
     * @brief draw the tracker box
     * @param[in] image The image to draw on
     * @param[in] clr The color of the box
     */
    void draw_box(cv::Mat& image, CvScalar clr);

    /**
     * @brief Score an object against the tracker
     * @param[in] image The image to draw on
     * @param[in] disp The disparity map
     * @param[in] roi The roi of the candidate object
     * @retval a score for the object
     */
    double score(cv::Mat& image, cv::Mat& disp, cv::Rect roi);

    /**
     * @brief compute velocity of tracked object from position history
     * @retval object x and z velocity
     */
    cv::Point2f compute_velocity();

    /**
     * @brief compute speed of tracked object from position history
     * @retval magnitude of velocity
     */
    double compute_speed();

    /**
     * @brief return a bounding box in the image from 3D coordinates
     * @param[in] xz X and Z coordinates in 3D
     * @retval Rect containing bounding box coordinates in the image
     */
    cv::Rect xz_to_box(cv::Point2f xz);

    /**
     * @brief return the estimated y-location of the object
     * @retval estimated y-location of object
     */
    double get_height(){
      return h;
    }

    /**
     * @brief return the person box size (z * h)
     * @retval tracker box size
     */
    double get_box_size(){
      return size_;
    }

    /**
     * @brief return the person box aspect ratio
     * @retval tracker box size aspect ratio
     */
    double get_ar(){
      return ar_;
    }

  private:
    double likelihood(cv::Mat& input, Particle states);
    void raw_detect(cv::Mat& input);
    void grow(cv::Mat& input);
    void prune();
    cv::Point2f find_location(cv::Mat& disp, cv::Rect roi);
    void make_xz_map(cv::Mat& disp, cv::Rect* roi = NULL);
    void make_xz_map_weighted(cv::Mat& image, cv::Mat& disp, cv::Rect* roi);
    void make_object_hist(cv::Mat& image, cv::Mat& disp, cv::Point maxLoc);
  };



#endif // PARTICLE_TRACK_H_

