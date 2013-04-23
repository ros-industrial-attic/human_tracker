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
#ifndef FOREST_H_
#define FOREST_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>

using namespace cv;
using namespace std;

#define VAGUE_LEARN false
#define TREE_DEPTH 18
#define N_TREES 8

#include <object_tracking/swri_tree.h>

  /**
   * @brief Online forest class
   */

  class forest{

    /**
     * @brief Node class for decision tree
     */
    struct fTree{
      int features[TREE_DEPTH];
      swriTree* dTree;
      int treeVote;
    };

    fTree trees[N_TREES];
    cv::Mat h;
  public:
    /**
     * @brief Create a decision tree
     */
    forest();

    /**
     * @brief destroy a tree
     */
    ~forest();

    /**
     * @brief Predict class of an input sequence
     * @param[in] sequence The input feature vector
     * @retval    Return the number of votes for class 1
     */
    int classify(cv::Mat& sequence);

    /**
     * @brief Add a sample to the forest
     * @brief sequence the sample to add
     * @brief protect whether or not to protect the leaves from pruning
     */
    void addToTrees(cv::Mat& sequence, bool protect = false);

    /**
     * @brief Remove a sample from the forest
     * @brief sequence the sample to remove
     * @brief protect whether or not to protect the leaf from growing
     */
    void removeFromTrees(cv::Mat& sequence, bool protect = false);

    void subFeature(cv::Mat& sequence, int i, cv::Mat& h);

    void randomFeatures(int poolSize);

  };  // class forest


#endif // FOREST_H_
