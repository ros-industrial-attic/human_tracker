#ifndef OVERLAP_HPP
#define OVERLAP_HPP

#include <roi_msgs/RoiRect.h>
#include <roi_msgs/Rois.h>

using roi_msgs::RoiRect;
using roi_msgs::Rois;

float calcOverlapMax(RoiRect r1, RoiRect r2);
void remove_overlap_Rois(Rois& rois_msg_input, Rois& rois_msg_output);

#endif
