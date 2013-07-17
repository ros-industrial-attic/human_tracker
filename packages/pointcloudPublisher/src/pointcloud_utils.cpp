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
#include <pointcloudPublisher/pointcloud_utils.h>

PointcloudUtils::PointcloudUtils ()
{
}

PointcloudUtils::~PointcloudUtils ()
{
}

void
PointcloudUtils::setIntrinsics (float& baseline, Eigen::Matrix3f& rgb_intrinsics, Eigen::Matrix3f& disparity_intrinsics, float min_disparity)
{
  baseline_ = baseline;
  rgb_intrinsics_ = rgb_intrinsics;
  disparity_intrinsics_ = disparity_intrinsics;
  min_disparity_ = min_disparity;
}

void
PointcloudUtils::setExtrinsics (Eigen::Vector3f& translation_disp_rgb, Eigen::Vector3f& rotation_angles_disp_rgb)
{
  translation_disp_rgb_ = translation_disp_rgb;
  rotation_angles_disp_rgb_ = rotation_angles_disp_rgb;

  // Affine transformation between disparity and rgb frames:
  pcl::getTransformation(translation_disp_rgb(0), translation_disp_rgb(1), translation_disp_rgb(2), rotation_angles_disp_rgb(0),
      rotation_angles_disp_rgb(1), rotation_angles_disp_rgb(2), depth_to_rgb_transform_);
}

void
PointcloudUtils::depthFromDisparity (cv::Mat& disparity_image, Eigen::Matrix3f& intrinsics, float min_disparity, cv::Mat& depth_image)
{
  depth_image = cv::Mat::zeros(disparity_image.rows, disparity_image.cols, CV_32F);
  for (int i = 0; i < disparity_image.rows; i++)
  {
    for (int j = 0; j < disparity_image.cols; j++)
    {
      float disparity_value = disparity_image.at<float>(i,j);
      if (disparity_value > min_disparity)
      {
        depth_image.at<float>(i,j) = baseline_ * intrinsics(0) / disparity_value;
      }
    }
  }
}

void
PointcloudUtils::pointcloudFromDepthImage (cv::Mat& depth_image, Eigen::Matrix3f& depth_intrinsics, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
{
  // For point clouds XYZ
  float depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
  float depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy

  pcl::PointXYZ new_point;

  output_cloud->points.resize(depth_image.cols*depth_image.rows, new_point);
  output_cloud->width = depth_image.cols;
  output_cloud->height = depth_image.rows;
  output_cloud->is_dense = false;
  for (int i=0;i<depth_image.rows;i++)
  {
    for (int j=0;j<depth_image.cols;j++)
    {
      float depth_value = depth_image.at<float>(i,j);

      if (depth_value > 0)
      {
        // Find 3D position with respect to depth frame:
        new_point.z = depth_value;
        new_point.x = (j - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
        new_point.y = (i - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
        output_cloud->at(j,i) = new_point;
      }
      else
      {
        new_point.z = std::numeric_limits<float>::quiet_NaN();
        new_point.x = std::numeric_limits<float>::quiet_NaN();
        new_point.y = std::numeric_limits<float>::quiet_NaN();
        output_cloud->at(j,i) = new_point;
      }
    }
  }
}

void
PointcloudUtils::transformPointcloudWithAffineTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_out, Eigen::Affine3f& transform)
{
  pcl::PointXYZ current_point, registered_point;
  xyz_cloud_out->points.resize(xyz_cloud_in->width * xyz_cloud_in->height, registered_point);
  xyz_cloud_out->width = xyz_cloud_in->width;
  xyz_cloud_out->height = xyz_cloud_in->height;
  xyz_cloud_out->is_dense = false;

  for (int i=0;i<xyz_cloud_in->height;i++)
  {
    for (int j=0;j<xyz_cloud_in->width;j++)
    {
      current_point = xyz_cloud_in->at(j,i);
      registered_point = pcl::transformPoint(current_point, transform);
      xyz_cloud_out->at(j,i) = registered_point;
    }
  }
}

void
PointcloudUtils::createDepthImageFromOrganizedPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Matrix3f& intrinsic_matrix, cv::Mat& depth_image)
{
  // Compute projection matrix:
  Eigen::MatrixXf projection_matrix;
  Eigen::MatrixXf m(3,4);
  m << 1,0,0,0,
     0,1,0,0,
     0,0,1,0;
  projection_matrix = intrinsic_matrix * m;

  Eigen::Vector4f homogeneous_point;
  Eigen::Vector3f projected_point;
  depth_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32F);
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    pcl::PointXYZ* current_point = &cloud->points[i];
    if (!isnan(current_point->z))
    {
      homogeneous_point << current_point->x, current_point->y, current_point->z, 1;
      projected_point = projection_matrix * homogeneous_point;
      projected_point /= projected_point(2);
      if ((projected_point(0) >= 0) && (projected_point(1) >= 0) && (projected_point(0) < depth_image.cols)
          && (projected_point(1) < depth_image.rows))
      {
        depth_image.at<float>(projected_point(1),projected_point(0)) = current_point->z;
      }
    }
  }
}

void
PointcloudUtils::depthToRGBRegistration (cv::Mat& depth_image, cv::Mat& registered_depth_image)
{
  // Create XYZ pointcloud:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloudFromDepthImage(depth_image, disparity_intrinsics_, cloud);

  // Tranform pointcloud to rgb reference frame:
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  transformPointcloudWithAffineTransform(cloud, registered_cloud, depth_to_rgb_transform_);

  // Project pointcloud to an image:
  createDepthImageFromOrganizedPointcloud(registered_cloud, rgb_intrinsics_, registered_depth_image);

}

void
PointcloudUtils::createPointcloudFromRGBAndDepthImage(cv::Mat& rgb_image, cv::Mat& depth_image, Eigen::Matrix3f& intrinsic_matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // For point clouds XYZRGB
  float rgb_focal_inverted_x = 1/intrinsic_matrix(0,0);  // 1/fx
  float rgb_focal_inverted_y = 1/intrinsic_matrix(1,1);  // 1/fy

  pcl::PointXYZRGB new_point;

  cloud->points.resize(depth_image.cols*depth_image.rows, new_point);
  cloud->width = depth_image.cols;
  cloud->height = depth_image.rows;
  cloud->is_dense = false;
  for (int i=0;i<depth_image.rows;i++)
  {
    for (int j=0;j<depth_image.cols;j++)
    {
      float depth_value = depth_image.at<float>(i,j);

      if (depth_value == depth_value)
      {
        // Find 3D position respect to rgb frame:
        new_point.z = depth_value;
        new_point.x = (j - intrinsic_matrix(0,2)) * new_point.z * rgb_focal_inverted_x;
        new_point.y = (i - intrinsic_matrix(1,2)) * new_point.z * rgb_focal_inverted_y;
        new_point.r = rgb_image.at<cv::Vec3b>(i,j)[2];
        new_point.g = rgb_image.at<cv::Vec3b>(i,j)[1];
        new_point.b = rgb_image.at<cv::Vec3b>(i,j)[0];
        cloud->at(j,i) = new_point;
      }
      else
      {
        new_point.z = std::numeric_limits<float>::quiet_NaN();
        new_point.x = std::numeric_limits<float>::quiet_NaN();
        new_point.y = std::numeric_limits<float>::quiet_NaN();
        new_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
        new_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
        new_point.b = std::numeric_limits<unsigned char>::quiet_NaN();
        cloud->at(j,i) = new_point;
      }
    }
  }
}

void
PointcloudUtils::computePointcloudFast (cv::Mat& rgb_image, cv::Mat& disparity_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // For point clouds XYZRGB
  float disparity_focal_inverted_x = 1/disparity_intrinsics_(0,0);  // 1/fx
  float disparity_focal_inverted_y = 1/disparity_intrinsics_(1,1);  // 1/fy

  // Compute projection matrix:
  Eigen::MatrixXf projection_matrix;
  Eigen::MatrixXf m(3,4);
  m << 1,0,0,0,
      0,1,0,0,
      0,0,1,0;
  projection_matrix = rgb_intrinsics_ * m;

  Eigen::Vector4f homogeneous_point;
  Eigen::Vector3f projected_point;
  pcl::PointXYZRGB nan_point;
  pcl::PointXYZ new_point;
  pcl::PointXYZ registered_point;

  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();
  cloud->points.resize(disparity_image.cols*disparity_image.rows, nan_point);
  cloud->width = disparity_image.cols;
  cloud->height = disparity_image.rows;
  cloud->is_dense = false;

  // Fill pointcloud with color:
  for (int i=0;i<rgb_image.rows;i++)
  {
    for (int j=0;j<rgb_image.cols;j++)
    {
      cloud->at(j,i).r = rgb_image.at<cv::Vec3b>(i,j)[2];
      cloud->at(j,i).g = rgb_image.at<cv::Vec3b>(i,j)[1];
      cloud->at(j,i).b = rgb_image.at<cv::Vec3b>(i,j)[0];
    }
  }

  // Fill pointcloud with 3D coordinates:
  for (int i=0;i<disparity_image.rows;i++)
  {
    for (int j=0;j<disparity_image.cols;j++)
    {
      float disparity_value = disparity_image.at<float>(i,j);
      float depth_value = 0;
      if (disparity_value > min_disparity_)
      {
        depth_value = baseline_ * rgb_intrinsics_(0) / disparity_value;
      }

      if (depth_value > 0)
      {
        // Find 3D position with respect to depth frame:
        new_point.z = depth_value;
        new_point.x = (j - disparity_intrinsics_(0,2)) * new_point.z * disparity_focal_inverted_x;
        new_point.y = (i - disparity_intrinsics_(1,2)) * new_point.z * disparity_focal_inverted_y;

        // Transform to rgb reference frame:
        registered_point = pcl::transformPoint(new_point, depth_to_rgb_transform_);

        // Project to the rgb image:
        homogeneous_point << registered_point.x, registered_point.y, registered_point.z, 1;
        projected_point = projection_matrix * homogeneous_point;
        projected_point /= projected_point(2);
        if ((projected_point(0) >= 0) && (projected_point(1) >= 0) && (projected_point(0) < disparity_image.cols)
            && (projected_point(1) < disparity_image.rows))
        {
          // Change 3D coordinates of the point:
          cloud->at(projected_point(0),projected_point(1)).x = registered_point.x;
          cloud->at(projected_point(0),projected_point(1)).y = registered_point.y;
          cloud->at(projected_point(0),projected_point(1)).z = registered_point.z;
        }
      }
    }
  }
}

void
PointcloudUtils::computePointcloud (cv::Mat& rgb_image, cv::Mat& disparity_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzrgb_cloud)
{
  // Create a depth image from a disparity image:
  cv::Mat depth_image;
  depthFromDisparity(disparity_image, disparity_intrinsics_, min_disparity_, depth_image);

  // Register the depth image to the rgb reference frame:
  cv::Mat registered_depth_image;
  depthToRGBRegistration (depth_image, registered_depth_image);

  // Create the xyzrgb pointcloud from the rgb and the registered depth image:
  createPointcloudFromRGBAndDepthImage(rgb_image, registered_depth_image, rgb_intrinsics_, xyzrgb_cloud);
}

