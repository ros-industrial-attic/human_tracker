#ifndef SWRI_TREE_H_
#define SWRI_TREE_H_

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

  /**
   * @brief Simple decision tree class
   * The decision tree makes binary decisions that lead to an end node.
   * The end node (leaf) will have a class value of non-object or object
   */

  class swriTree{

    /**
     * @brief Node class for decision tree
     */
    class node{
    public:
      node* parent;
      node* left;
      node* right;
      int nClass;
      bool isProtected;

      /**
       * @brief Constructor for node
       * Children/parent are initially null, nClass initially is 2 (non-leaf node)
       */
      node(){
        parent = NULL;
        left = NULL;
        right = NULL;
        nClass = 2;
        isProtected = false;
      }
      ~node(){
        delete left;
        delete right;
      }
    };
    node* root;
    int depth;
  public:
    /**
     * @brief Create a decision tree
     */
    swriTree();

    /**
     * @brief destroy a tree
     */
    ~swriTree();

    /**
     * @brief Predict class of an input sequence
     * @param[in] sequence The input feature vector
     * @retval    Return the tree's prediction
     */
    int predict(cv::Mat& sequence);

    /**
     * @brief Add a sample to a tree
     * @param[in] sequence the sample to add
     * @param[in] protect whether or not to protect the leaf from pruning
     */
    void addToTree(cv::Mat& sequence,bool protect = false);

    /**
     * @brief Remove a sample to a tree
     * @param[in] sequence the sample to remove
     * @param[in] protect whether or not to protect the leaf from growing
     */
    void removeFromTree(cv::Mat& sequence, bool protect = false);

    bool isDead(node* loc);
  }; //class swriTree

#endif // SWRI_TREE_H_
