#include <object_tracking/swri_tree.h>


  swriTree::swriTree()
  {
    root = new node;
  }  // swriTree()

  swriTree::~swriTree()
  {  // Free all memory for the tree...
    delete root;
  }  // ~swriTree

  int swriTree::predict(cv::Mat& sequence)
  {
    node* loc = root;
    int i = 0;
    while ((loc->nClass == 0)&&(i < sequence.cols))
    {
      if (sequence.at<int16_t>(0,i) == 1)
      {  // is left
        loc = loc->left;
      }
      else
      {
        loc = loc->right;  // is right
      }
      i = i + 1;
    }  // while
    return loc->nClass;
  }  // predict

  void swriTree::addToTree(cv::Mat& sequence, bool protect)
  {  // this is how we initialize a tree also, after creating root (root has
     // null children, class of 2 (non-object)
    node* loc = root;
    bool shouldBreak = false;
    for (int i = 0; i < sequence.cols; i++)
    {
      if (loc->nClass == 2)
      {  // if this was a non-object, it now is just a node and we will create
         // children for it, which we will move to in the next iteration
        loc->nClass = 0;  // zero indicates a node instead of a leaf
        loc->left = new node;  // the new nodes both initialize to be
                               // non-objects
        loc->left->parent = loc;
        loc->right = new node;
        loc->right->parent = loc;
        if (VAGUE_LEARN)
        {  // if VAGUE_LEARN is true, we only want to make the tree one node
           // deeper
          shouldBreak = true;
        }
      }  // if

      if (sequence.at<int16_t>(0,i) == 1)  // is left
      {
        loc = loc->left;
      }
      else
      {
        loc = loc->right;  // is right
      }
      if (shouldBreak)
      {
        break;
      }
    }  // for

    if (!loc->isProtected)  // object could be a protected negative
    {
      loc->nClass = 1;  // object
      loc->isProtected = protect;
    }
  }  // addToTree

  void swriTree::removeFromTree(cv::Mat& sequence, bool protect)
  {
    node* loc = root;
    int i = 0;
    while (loc->nClass == 0)
    {
      if (sequence.at<int16_t>(0,i) == 1)  // is left
      {
        loc = loc->left;
      }
      else
      {
        loc = loc->right;  // is right
      }
      i = i + 1;
    }  // while
    if (VAGUE_LEARN)  // if VAGUE_LEARN is set, we
    {
      if (i < sequence.cols)  // create a child
      {
        loc->nClass = 0;  // zero indicates a node instead of a leaf
        loc->left = new node;  // the new nodes both initialize to be
                               // non-objects
        loc->left->parent = loc;
        loc->right = new node;
        loc->right->parent = loc;
        if (sequence.at<int16_t>(0,i) == 1)  // is left
        {
          loc = loc->left;
          loc->nClass = 2;  // this is the offender and is given non-object
                            // status
          loc->parent->right->nClass = 1;  // the other can remain
        }
        else
        {
          loc = loc->right;  // is right
          loc->nClass = 2;
          loc->parent->left->nClass = 1;
        }
      }
      loc->nClass = 2;  // assign non-object value
    }
    else
    {
      if (!loc->isProtected)
      {
        loc->nClass = 2;  // assign non-object value
        loc->isProtected = protect;
      }
    }
  }  // removeFromTree

  bool swriTree::isDead(node* loc)
  {
    // Find nodes that have identical branches upstream on left and right splits
    // This will allow for finding which features are not pulling their weight
    node* locCompare1 = loc->left;
    node* locCompare2 = loc->right;

    // choose a random path to follow through in each child
    bool isDead = false;
    while (1)
    {
      if ((locCompare1->left != NULL) && (locCompare2->left != NULL))
      {  // always try to move left first... maybe should make random
        locCompare1 = locCompare1->left;
        locCompare2 = locCompare2->left;
        if ((locCompare1->nClass = 1)&&(locCompare2->nClass = 1))
        {
          isDead = true;
        }
      }
      else if ((locCompare1 ->right != NULL) && (locCompare2->right != NULL))
      {
        locCompare1 = locCompare1->right;
        locCompare2 = locCompare2->right;
        if ((locCompare1->nClass = 1) && (locCompare2->nClass = 1))
        {
          isDead = true;
        }
      }
      else
      {
        break;
      }
    }  // while

    // if same path results in detection in both cases, the node is  dead
    return isDead;
  }  // findDeadNodes

