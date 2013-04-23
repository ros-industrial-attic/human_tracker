#include <ros/ros.h>
#include <roi_msgs/overlap.hpp>

using roi_msgs::RoiRect;
using roi_msgs::RoisConstPtr;
using roi_msgs::RoisPtr;

float calcOverlapMax(RoiRect r1, RoiRect r2)
{
  float max_x = (r1.x > r2.x) ? r1.x : r2.x;
  float max_y = (r1.y > r2.y) ? r1.y : r2.y;
  float min_x = (r1.x + r1.width < r2.x + r2.width) ? r1.x + r1.width : r2.x + r2.width;
  float min_y = (r1.y + r1.height < r2.y + r2.height) ? r1.y + r1.height : r2.y + r2.height;
  float A1 = r1.width * r1.height;
  float A2 = r2.width * r2.height;
  float minArea = (A1 < A2) ? A1 : A2;
  if ((min_x > max_x) && (min_y > max_y) && (minArea > 0.0))
  {
    return (min_x - max_x) * (min_y - max_y) / minArea;
  }
  else
    return 0.0;
}  // calcOverlapMax

void remove_overlap_Rois(Rois& rois_msg_input, Rois& rois_msg_output)
{
  float o;
  int n = (int) rois_msg_input.rois.size();
  rois_msg_output.rois.clear();
  if(n>0)rois_msg_output.rois.push_back(rois_msg_input.rois[0]);
  for(int i = 1;i<n;i++){
    bool overlap = false;
    for(unsigned int j = 0;j<rois_msg_output.rois.size();j++){ // check against those on list
      if ((o=calcOverlapMax(rois_msg_input.rois[i],rois_msg_output.rois[j])) > .80){
	//	ROS_ERROR("overlap %d %d = %lf",i,j,o);
	overlap = true;
	break;
      }
    }
    if(!overlap) {
      //      ROS_ERROR("NO OVERLAP %d",i);
      rois_msg_output.rois.push_back(rois_msg_input.rois[i]);
    }
  }
}

