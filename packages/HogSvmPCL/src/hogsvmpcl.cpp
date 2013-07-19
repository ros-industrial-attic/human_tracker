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

#include "HogSvmPCL/hogsvmpcl.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace HogSvmPCL {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/
  HogSvmPCLClassifier::HogSvmPCLClassifier()
  {
  }

  bool
  HogSvmPCLClassifier::loadSVMFromFile (std::string svm_filename)
  {
    std::string line;
    std::ifstream SVM_file;
    SVM_file.open(svm_filename.c_str());

    getline (SVM_file,line);      // read window_height line
    size_t tok_pos = line.find_first_of(":", 0);  // search for token ":"
    window_height_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

    getline (SVM_file,line);      // read window_width line
    tok_pos = line.find_first_of(":", 0);  // search for token ":"
    window_width_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

    getline (SVM_file,line);      // read SVM_offset line
    tok_pos = line.find_first_of(":", 0);  // search for token ":"
    SVM_offset_ = std::atof(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

    getline (SVM_file,line);      // read SVM_weights line
    tok_pos = line.find_first_of("[", 0);  // search for token "["
    size_t tok_end_pos = line.find_first_of("]", 0);  // search for token "]" , end of SVM weights
    size_t prev_tok_pos;
    while (tok_pos < tok_end_pos) // while end of SVM_weights is not reached
    {
      prev_tok_pos = tok_pos;
      tok_pos = line.find_first_of(",", prev_tok_pos+1);  // search for token ","
      SVM_weights_.push_back(std::atof(line.substr(prev_tok_pos+1, tok_pos-prev_tok_pos-1).c_str()));
    }
    SVM_file.close();

    if (SVM_weights_.size() == 0)
    {
      return (false);
    }
    else
    {
      return (true);
    }
  }

  void
  HogSvmPCLClassifier::setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset)
  {
    window_height_ = window_height;
    window_width_ = window_width;
    SVM_weights_ = SVM_weights;
    SVM_offset_ = SVM_offset;
  }

  void
  HogSvmPCLClassifier::getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset)
  {
    window_height = window_height_;
    window_width = window_width_;
    SVM_weights = SVM_weights_;
    SVM_offset = SVM_offset_;
  }

  float
  HogSvmPCLClassifier::evaluate (cv::Rect& R_in, cv::Mat& image)
  {
    int left = 0;
    int right = 0;
    int top = 0;
    int bottom = 0;

    if(R_in.x < 0)
      left = -R_in.x;
    if(R_in.x + R_in.width > image.cols)
      right = R_in.x + R_in.width - image.cols;
    if(R_in.y < 0)
      top = -R_in.y;
    if(R_in.y + R_in.height > image.rows)
      bottom = R_in.y + R_in.height - image.rows;

    //Reduce the image to match the correct size
    //if near the border, fill with black
    cv::Mat box;
    cv::copyMakeBorder(image(cv::Rect(R_in.x + left, R_in.y + top, R_in.width - right - left, R_in.height - bottom - top)), box, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::Mat sample;
    cv::resize(box, sample, cv::Size(window_width_, window_height_));

    //Convert the image to array of float
    float* sample_float = new float[sample.cols * sample.rows * 3];
    int delta = sample.rows * sample.cols;
    for(int row = 0; row < sample.rows; row++)
    {
      unsigned char* ptr = (unsigned char*) (sample.datastart + row * sample.step);
      for(int col = 0; col < sample.cols; col++)
      {
        sample_float[row + sample.rows * col] = (float) ptr[col * 3 + 2];
        sample_float[row + sample.rows * col + delta] = (float) ptr[col * 3 + 1];
        sample_float[row + sample.rows * col + delta * 2] = (float) ptr[col * 3];
      }
    }

    // Calculate HOG descriptor:
    pcl::people::HOG hog;
    float *descriptor = (float*) calloc(SVM_weights_.size(), sizeof(float));
    hog.compute(sample_float, descriptor);

    // Calculate confidence value by dot product:
    float confidence = 0.0;
    for(unsigned int j = 0; j < SVM_weights_.size(); j++)
    {
      confidence += SVM_weights_[j] * descriptor[j];
    }
    // Confidence correction:
    confidence -= SVM_offset_;

    delete[] descriptor;
    delete[] sample_float;

    return confidence;
  }

} // namespace HogSvmPCL
