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
#include <object_tracking/forest.h>
#include <ros/ros.h>


    forest::forest()
  {
    h = cv::Mat::zeros(1, TREE_DEPTH, CV_16S);
    for (int i = 0; i < N_TREES; i++)
    {
      trees[i].dTree = new swriTree;
    }
  }

  forest::~forest()
  {
    for (int i = 0; i < N_TREES; i++)
    {
      delete trees[i].dTree;
    }
  }

  int forest::classify(cv::Mat& sequence)
  {
    int votes = 0;
    // For each tree, construct feature vector and get vote
    for (int i = 0; i < N_TREES; i++)
    {
      subFeature(sequence, i, h);
      trees[i].treeVote = trees[i].dTree->predict(h);
      //std::cout << "v " << trees[i].treeVote << std::endl;
      if (trees[i].treeVote == 1)
      {
        votes++;
      }
    }

    return votes;
  }

  void forest::addToTrees(cv::Mat& sequence, bool protect)
  {
    for (int i = 0; i < N_TREES; i++)
    {  // for every tree in the forest
      subFeature(sequence, i, h);
      trees[i].dTree->addToTree(h, protect);  // Builds the trees with
                                              // single branches
    }
  }

  void forest::removeFromTrees(cv::Mat& sequence, bool protect)
  {
    for (int i = 0; i < N_TREES; i++)
    {  // for every tree in the forest
      subFeature(sequence, i, h);
      trees[i].dTree->removeFromTree(h, protect);  // Builds the trees with
                                                   // single branch
    }
  }

  void forest::subFeature(cv::Mat& sequence, int i, cv::Mat& h)
  {
    for (int j = 0; j < TREE_DEPTH; j++)
    {
      h.at<int16_t>(0,j) = sequence.at<int16_t>(0,trees[i].features[j]);
    }
  }

  void forest::randomFeatures(int poolSize)
  {
    for (int i = 0; i < N_TREES; i++)
      for (int j = 0; j < TREE_DEPTH; j++)
      {
        trees[i].features[j] = rand() % poolSize;
      }
  }
