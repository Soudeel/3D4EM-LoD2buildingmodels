
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : March 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Segmentation into quadric patches.
*
*--------------------------------------------------------------------*/
#ifndef __QUADRIC__SEGMENTAION__h__
#define __QUADRIC__SEGMENTAION__h__
#include "LaserPoints.h"
#include "Types.h"

				
/** Segment into quadric patches.
	
*/	
void
SegmentIntoQuadrics(
			const LaserPoints& laserPoints,
			SegmentsVector& segmentedIndices,				
			const int kNN=5,
			const bool useStrictNormalSimilarity=false);
			
vector<int>
GrowFromSeed(const LaserPoints& laserPoints,
			const LaserPoints& normals,
			int seedPointIndex,
			double maxAngleInDegrees,
			double maxSizeOfRegion,//In terms of distance from central point.
			const int kNN,
			double angleVariationForNeighbours = 10,
			bool breakOnMaxAngle=true);			
			
#endif //__QUADRIC__SEGMENTAION__h__


