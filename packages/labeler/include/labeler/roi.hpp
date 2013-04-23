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
