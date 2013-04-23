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
