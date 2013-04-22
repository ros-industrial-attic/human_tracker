Human Detection and Tracking
============================

This software is designed to detect and track humans using both an intensity image and a disparity image from either a pair of stereo cameras or from a structured lighting sensor such as the Xtion Pro from ASUS. The software first detects and then tracks people in the field of view of the sensors using machine learning techniques. 
The detection system is a cascade of successively more complex classifiers in the following order:
 1.  Simple geometric consistency test
 1.	Ada Boost Classifier using 174 Haar-like features on the intensity image
 1.	Ada Boost Classifier using 174 Haar-like features on the disparity image
 1.	Support Vector Machine using 174 Haar-like features on the intensity image
 1.	Support Vector Machine using Histogram of Oriented Gradient features

The tracking software implements a particle filter in the x-y plane, and learns on-line using color histograms computed over vertical columns as features.  A rudimentary graphical user interface is included which allows users to label collected imagery for re-training the system.  Scripts, in the form of ROS launch files are included for all of the following:
 1.	Collecting Data from either the ASUS sensor or a pair of Basler Ace Cameras.
 1.	Dumping the Imagery to a directory for Labeling.
 1.	Training each classifier using a directory filled with labeled imagery.
 1.	Running the system from a ROS bag, from a directory or live.

