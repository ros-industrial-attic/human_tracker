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
#ifndef POINTCLOUD_UTILS_H_
#define POINTCLOUD_UTILS_H_

#include <cv.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

  /**
   * @brief Provide methods for converting rgb and disparity images to colored point clouds.
   * @author Matteo Munaro - matteo.munaro@dei.unipd.it
   * @date 16th July 2013
   */
  class PointcloudUtils{

  public:

    /**
     * @brief Create an instance of the class
     */
    PointcloudUtils ();

    /**
     * @brief destroy the class
     */
    ~PointcloudUtils ();

    /**
     * @brief Set intrinsic parameters for the rgb and disparity images
     * @param[in] baseline Cameras baseline
     * @param[in] rgb_intrinsics_ RGB intrinsic parameters matrix
     * @param[in] disparity_intrinsics_ Disparity intrinsic parameters matrix
     * @param[in] min_disparity Minimum value of disparity allowed
     */
    void
    setIntrinsics (float& baseline, Eigen::Matrix3f& rgb_intrinsics, Eigen::Matrix3f& disparity_intrinsics, float min_disparity);

    /**
     * @brief Set extrinsic parameters for the rgb and disparity reference frames
     * @param[in] translation_disp_rgb Translation between RGB and disparity reference frames
     * @param[in] rotation_angles_disp_rgb Rotation between RGB and disparity reference frames
     */
    void
    setExtrinsics (Eigen::Vector3f& translation_disp_rgb, Eigen::Vector3f& rotation_angles_disp_rgb);

    /**
     * @brief Compute a depth image from a disparity image
     * @param[in] disparity_image Disparity image
     * @param[in] intrinsics Intrinsic parameters matrix of the disparity image
     * @param[in] min_disparity Minimum disparity allowed
     * @param[out] depth_image Depth image corresponding to the disparity image
     */
    void
    depthFromDisparity (cv::Mat& disparity_image, Eigen::Matrix3f& intrinsics, float min_disparity, cv::Mat& depth_image);

    /**
     * @brief Compute a XYZ pointcloud corresponding a depth image
     * @param[in] depth_image Depth image
     * @param[in] depth_intrinsics Intrinsic parameters matrix of the depth image
     * @param[out] output_cloud Pointcloud corresponding to the depth image
     */
    void
    pointcloudFromDepthImage (cv::Mat& depth_image, Eigen::Matrix3f& depth_intrinsics, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

    /**
     * @brief Apply an affine transform to a XYZ pointcloud
     * @param[in] xyz_cloud_in Input point
     * @param[out] xyz_cloud_out Output point
     * @param[in] transform Affine transform
     */
    void
    transformPointcloudWithAffineTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_out, Eigen::Affine3f& transform);

    /**
     * @brief Project an organized pointcloud to a depth image
     * @param[in] cloud Input point cloud
     * @param[in] intrinsic_matrix Intrisic parameters matrix of the depth image
     * @param[out] depth_image Output depth image corresponding to the pointcloud
     */
    void
    createDepthImageFromOrganizedPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix3f& intrinsic_matrix, cv::Mat& depth_image);

    /**
     * @brief Compute a depth image registered to a rgb image
     * @param[in] depth_image Input depth image
     * @param[out] registered_depth_image Depth image registered to the rgb image
     */
    void
    depthToRGBRegistration (cv::Mat& depth_image, cv::Mat& registered_depth_image);

    /**
     * @brief Compute a XYZRGB pointcloud from a rgb and a depth image
     * @param[in] depth_image Input depth image
     * @param[in] rgb_image Input rgb image
     * @param[in] intrinsic_matrix Intrisic parameters matrix of the two images
     * @param[out] cloud Output xyzrgb pointcloud
     */
    void
    createPointcloudFromRGBAndDepthImage (cv::Mat& rgb_image, cv::Mat& depth_image, Eigen::Matrix3f& intrinsic_matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    /**
     * @brief Compute a XYZRGB pointcloud from a rgb and a disparity image (version 25% faster)     *
     * @param[in] rgb_image Input rgb image
     * @param[in] disparity_image Input disparity image
     * @param[out] cloud Output xyzrgb pointcloud
     */
    void
    computePointcloudFast (cv::Mat& rgb_image, cv::Mat& disparity_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    /**
     * @brief Compute a colored point cloud from a pair of corresponding rgb and disparity images
     * @param[in] rgb_image The input rgb image
     * @param[in] disparity_image The input disparity image
     * @param[out] xyzrgb_cloud Output xyzrgb pointcloud
     */
    void
    computePointcloud (cv::Mat& rgb_image, cv::Mat& disparity_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzrgb_cloud);

  protected:
    /** @brief camera baseline */
    float baseline_;

    /** @brief RGB intrinsica parameters matrix */
    Eigen::Matrix3f rgb_intrinsics_;

    /** @brief disparity intrinsic parameters matrix */
    Eigen::Matrix3f disparity_intrinsics_;

    /** @brief minimum disparity allowed */
    float min_disparity_;

    /** @brief translation between RGB and disparity reference frames */
    Eigen::Vector3f translation_disp_rgb_;

    /** @brief rotation between RGB and disparity reference frames */
    Eigen::Vector3f rotation_angles_disp_rgb_;

    /** @brief affine transform between RGB and disparity reference frames */
    Eigen::Affine3f depth_to_rgb_transform_;
  };  // class PointcloudUtils

#endif // POINTCLOUD_UTILS_H_
