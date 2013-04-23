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
