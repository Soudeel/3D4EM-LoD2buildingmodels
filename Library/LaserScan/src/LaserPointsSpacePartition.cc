
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Position2D.h" // local
#include "LineTopologies.h" // local
#include "Plane.h" // local
#include "DataBoundsLaser.h" // local
#include "LaserPoints.h" // local
#include "PointNumberLists.h"
#include "Adjustment.h"
#include "Rotation2D.h"
#include "stdmath.h"


#include "LineSegment2D.h"
#include <numeric>
LineSegment2D LaserPoints::FitLineSegment(const PointNumberList& pnlPnts) const
{
	Position2D objPnt;
	Positions2D temObjPnts;
	LaserPoints::const_iterator pnt;

	for (unsigned int i=0; i<pnlPnts.size(); i++) {
		pnt = this->begin()+pnlPnts[i].Number();
		objPnt.X() = pnt->X();
		objPnt.Y() = pnt->Y();
		temObjPnts.push_back(objPnt);
	}

	Line2D line = Line2D(temObjPnts);
	double scalarMax, scalarMin, scalarTem;
	scalarMax = -10e10;
	scalarMin = +10e10;

	for (unsigned int i=0; i<pnlPnts.size(); i++) {
		pnt = this->begin()+pnlPnts[i].Number();
		scalarTem = line.Scalar(pnt->Position2DOnly());

		if (scalarTem<scalarMin) 
			scalarMin = scalarTem;

		if (scalarTem>scalarMax) 
			scalarMax = scalarTem;
	}

	return LineSegment2D(line,scalarMin, scalarMax);
}


LineSegment2D LaserPoints::FitLineSegment(const PointNumberList& pnlPnts, const Vector2D& dir) const
{
	if (pnlPnts.empty()) return LineSegment2D();
	LaserPoints::const_iterator pnt = this->begin()+pnlPnts[0].Number();

	Line2D line = Line2D(pnt->Position2DOnly(), pnt->Position2DOnly()+dir);
	double meanDist = 0.0;

	for (unsigned int i=0; i<pnlPnts.size(); i++) {
		pnt = this->begin()+pnlPnts[i].Number();
		meanDist += line.DistanceToPointSigned(pnt->Position2DOnly());
	}
	line.SetDistanceToOrigin(line.DistanceToOrigin()+meanDist/pnlPnts.size());

	double scalarMax, scalarMin, scalarTem;
	scalarMax = -10e10;
	scalarMin = +10e10;

	for (unsigned int i=0; i<pnlPnts.size(); i++) {
		pnt = this->begin()+pnlPnts[i].Number();
		scalarTem = line.Scalar(pnt->Position2DOnly());

		if (scalarTem<scalarMin) scalarMin = scalarTem;
		if (scalarTem>scalarMax) scalarMax = scalarTem;
	}

	return LineSegment2D(line, scalarMin, scalarMax);
}


