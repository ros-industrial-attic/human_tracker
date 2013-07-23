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
#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

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

using namespace cv;
using namespace std;

  struct Particle{
    Mat x;
    double weight;
    double prevWeight;
  };

  /**
   * @brief Implements sequential importance resampling particle filter.
   */

  class ParticleFilter{
    cv::RNG rng;
    double neff;
  public:
    double age;
    int N; //number of particles
    Mat A; //linear motion model, 6x6
    Mat B; //variances of each state variable
    bool initialized;
    bool is_dead_;
    Particle current_map_;
    vector<Particle> particle_model; //length N
    bool console_output_;

    /**
     * @brief Construct a filter object
     */
    ParticleFilter();

    /**
     * @brief Initialize a tracker
     * @param[in] input The input image
     * @param[in] initial_contour the initial 3 points of contour spline
     */
    void initialize(Particle initial_state);

    /**
     * @brief Load the motion model
     * @param[in] perceptionPath The path containing the covariance data
     */
    void load_model(const char* perceptionPath);

    /**
     * @brief Update the tracker state
     * @param[in] input The input image (next image in sequence)
     */
    void updateState(cv::Mat& input, double dt);

    /**
     * @brief Calculate the covariance matrix of particles in filter
     * @returns a Mat object covariance matrix
     */
    cv::Mat calculate_covariance();

  private:
    void resample(); // resample according to weights to update particle_model
    void drift_diffuse_sample(double dt); // apply x = Ax + Bw to each member of particle_model
    void observation_density_reweight(cv::Mat& input); // Reweight every particle in model by its observation density
    virtual double likelihood(cv::Mat& input, Particle states); //Calculate likelihood of a particular possible state

  };

#endif // PARTICLE_FILTER_H_
