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
