#include <stdio.h>
#include <roi_msgs/overlap.hpp>
using roi_msgs::RoiRect;
using roi_msgs::RoisConstPtr;
using roi_msgs::RoisPtr;

main()
{
  RoiRect r1,r2;
  calcOverlapMax(r1,r2);
  printf("shit this is stupid\n");
}
