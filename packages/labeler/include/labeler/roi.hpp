/**
 * @file /include/labeler/roi.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef labeler_ROI_HPP_
#define labeler_ROI_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <stdlib.h>
#include <algorithm>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace labeler {

/*****************************************************************************
** Class
*****************************************************************************/

class Roi{

public:
	Roi( );
	bool init();
	bool init(int x1, int y1, int x2, int y2, int roiV);
	bool init(int roiV, int width, int height);
	bool init(int x1, int y1, int x2, int y2, int roiV, bool isRandom);
        bool pinit(Roi R, float percent);
        int startX();
	int startY();
	int endX();
	int endY();
	int label();
	bool isRandom();
        int width();
        int height();
	int area();
        float aspect() {
	  float a = (float)width()/(float)height();
	  return(a);
	};
	void offset(int offsetX, int offsetY);
	void bounds(int width, int height);
	int overlapArea(Roi checkRoi);	
        void grow_force_aspect(float a, float percent);
        

private:
	int upperLeftX;
	int upperLeftY;
	int lowerRightX;
	int lowerRightY;
	int roiValue;
	bool randomlyGenerated;
};

}  // namespace labeler

#endif /* labeler_ROI_HPP_ */
