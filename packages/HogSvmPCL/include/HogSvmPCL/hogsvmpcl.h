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

#ifndef HOGSVMPCL_H
#define HOGSVMPCL_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/people/hog.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HogSvmPCL {

  /*****************************************************************************
   ** Class
   *****************************************************************************/
  /**
     * @brief Provide methods for classifying portion of an image with a Support Vector Machine
     * @author Matteo Munaro - matteo.munaro@dei.unipd.it
     * @date 19th July 2013
     */
  class HogSvmPCLClassifier{

  public:
      /**
       * @brief Create an instance of the class
       */
      HogSvmPCLClassifier ();

      /**
       * @brief destroy the class
       */
      ~HogSvmPCLClassifier ();

      /**
       * @brief Load SVM parameters from a text file.
       *
       * @param[in] svm_filename Filename containing SVM parameters.
       *
       * @return true if SVM has been correctly set, false otherwise.
       */
      bool
      loadSVMFromFile (std::string svm_filename);

      /**
       * @brief Set trained SVM for person confidence estimation.
       *
       * @param[in] window_height Detection window height.
       * @param[in] window_width Detection window width.
       * @param[in] SVM_weights SVM weights vector.
       * @param[in] SVM_offset SVM offset.
       */
      void
      setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset);

      /**
       * @brief Get trained SVM for person confidence estimation.
       *
       * @param[out] window_height Detection window height.
       * @param[out] window_width Detection window width.
       * @param[out] SVM_weights SVM weights vector.
       * @param[out] SVM_offset SVM offset.
       */
      void
      getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset);

      /**
       * @brief Create an instance of the class
       *
       * @param
       */
      float
      evaluate (cv::Rect& R_in, cv::Mat& image);

  protected:

      /** \brief Height of the image patch to classify. */
      int window_height_;

      /** \brief Width of the image patch to classify. */
      int window_width_;

      /** \brief SVM offset. */
      float SVM_offset_;

      /** \brief SVM weights vector. */
      std::vector<float> SVM_weights_;

  };


}  // namespace HogSvmPCL

#endif /*  HOGSVMPCL_H */

