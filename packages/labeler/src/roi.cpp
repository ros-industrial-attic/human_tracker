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

#include "../include/labeler/roi.hpp"
#include <stdlib.h>
#include <algorithm>

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace std;

namespace labeler {

/*****************************************************************************
** Implementation
*****************************************************************************/

Roi::Roi(){

}

bool Roi::init() {
	upperLeftX = 0;
	upperLeftY =0;
	lowerRightX = 0;
	lowerRightY = 0;
	roiValue = 0;
	randomlyGenerated = false;
	return true;
}

bool Roi::init(int roiV, int width, int height){
        // This function randomly generates ROI
	int x1;
	int x2;
	int y1;
	int y2;
	
	x1 = rand() % width;
        y1 = rand() % height;
        x2 = rand() % width;
        y2 = rand() % height;
       
	init(x1, y1, x2, y2, roiV);
        randomlyGenerated = true;

        return true;
}
bool Roi::pinit(Roi R, float percent)
{
  float t = 2*percent/100*R.height(); // twice the percent is the total range of the random number
  int max_twice_offset = (int) t; // truncate
  int ulxoffset = rand()%max_twice_offset - t/2; // allow both positive and negative offsets
  int ulyoffset = rand()%max_twice_offset - t/2; // allow both positive and negative offsets
  int lrxoffset = rand()%max_twice_offset - t/2; // allow both positive and negative offsets
  int lryoffset = rand()%max_twice_offset - t/2; // allow both positive and negative offsets
  
  upperLeftX  = R.startX() + ulxoffset;
  upperLeftY  = R.startY() + ulyoffset;
  lowerRightX = R.endX()   + lrxoffset;
  lowerRightY = R.endY()   + lryoffset;
  roiValue    = R.label();
  randomlyGenerated = true;
  return(true);
}

bool Roi::init(int x1, int y1, int x2, int y2, int roiV){
	
	// Logic to determine upperLeft and lowerRight given two opposite rect corners (x1, y1) and (x2, y2)
	if (x1 < x2){
		upperLeftX = x1;
        	lowerRightX = x2;
        }
       	else
        {
        	upperLeftX = x2;
        	lowerRightX = x1;
        }
        if (y1 < y2){
		upperLeftY = y1;
        	lowerRightY = y2;
        }
        else
        {
		upperLeftY = y2;
		lowerRightY = y1;
        }
	roiValue = roiV;
	randomlyGenerated = false;
	return true;
}

bool Roi::init(int x1, int y1, int x2, int y2, int roiV, bool isRandom){

        // Logic to determine upperLeft and lowerRight given two opposite rect corners (x1, y1) and (x2, y2)
       	init(x1,y1,x2,y2,roiV);
	randomlyGenerated = isRandom;
        return true;
}


int Roi::overlapArea(Roi checkRoi){
	int overlap = 0;
	// Calulate the intersection of the two roi
	overlap = max(0,min(lowerRightX, checkRoi.endX())-max(upperLeftX,checkRoi.startX())) * max(0,min(lowerRightY, checkRoi.endY())-max(upperLeftY,checkRoi.startY()));
	//overlap = max(0,min(upperLeftX, checkRoi.startX())-max(lowerRightX,checkRoi.endX())) * max(0,min(upperLeftY, checkRoi.startY())-max(lowerRightY,checkRoi.endY()));
	return overlap;
}

int Roi::startX(){
        return upperLeftX;
}

int Roi::startY(){
        return upperLeftY;
}

int Roi::endX(){
        return lowerRightX;
}

int Roi::endY(){
        return lowerRightY;
}

int Roi::label(){
	return roiValue;
}

bool Roi::isRandom(){
	return randomlyGenerated;
}


int Roi::width(){

	return (lowerRightX - upperLeftX);
}

int Roi::height(){
	return (lowerRightY-upperLeftY);
}

int Roi::area() {
        return ((lowerRightX - upperLeftX) * (lowerRightY - upperLeftY));
}

void Roi::offset(int offsetX, int offsetY){

	upperLeftX -=offsetX;
	lowerRightX -=offsetX;
	upperLeftY -=offsetY;
	lowerRightY -=offsetY;
}
void Roi::grow_force_aspect(float a, float percent)
{
  int height = lowerRightY-upperLeftY;
  int middleX = (lowerRightX+upperLeftX)/2.0;
  int middleY = (lowerRightY+upperLeftY)/2.0;
  height = height*(1.0+percent/100.0);
  int width  = height*a;
  lowerRightX = middleX + width/2;
  upperLeftX  = middleX - width/2;
  lowerRightY = middleY + height/2;
  upperLeftY  = middleY - height/2;
}
void Roi::bounds(int width, int height){
	if(upperLeftX<0)
        	upperLeftX=0;
        if(upperLeftX>=width)
              	upperLeftX = width-1;
        if(upperLeftY<0)
        	upperLeftY=0;
        if(upperLeftY>=height)
                upperLeftY = height-1;

        if(lowerRightX<0)
        	lowerRightX=0;
        if(lowerRightX>=width)
        	lowerRightX = width-1;
        if(lowerRightY<0)
        	lowerRightY=0;
        if(lowerRightY>=height)
        	lowerRightY = height-1;
}

}  // namespace labeler
