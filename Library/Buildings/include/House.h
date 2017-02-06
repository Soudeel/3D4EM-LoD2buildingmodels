
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



#ifndef _House_h_
#define _House_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  House  - A list of corners of a house

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/
#include "ObjectPoints2D.h"
#include "LineTopology.h"
#include "Segment2D.h"
#include "Segment.h"
#include "Segment3D.h"

struct CollinSegs;

/*
--------------------------------------------------------------------------------
                                    House
--------------------------------------------------------------------------------
*/


class House : public FeatureNumber, public ObjectPoints2D
{

public:
	House(int n = 0)	/* Default constructor                       */	
	  { num = n; } 
	  
	House(int n, const LineTopology &lin, const ObjectPoints2D &pts); 
	
	House(int n, const ObjectPoints2D &pts)
		: FeatureNumber(n), ObjectPoints2D(pts) {}; 
			  
	~House() {};		/* Destructor                        */
	
	House& operator=(const House& house);    /* Copy assignament         */
	
	int IsComplet() const;

    void MergeClosePoints(double dist);			
        
    void DeleteCollinearPoints();
	
	void Convert2LineTop(LineTopology *, ObjectPoints2D *) const;		

    void Print() const;

	/// check if the given object point is a map corner	
	bool IsHouseCorner(const ObjectPoint2D &pt) const;

	/// check if the given object point is a map corner	
    /// returns the index of the corresponding corner
	int GetHouseCorner(const Position2D &pt) const;

    /// returns the index of the house segment containing the given point
	int GetHouseSegment(const Position2D &pt) const;

	/// check if two object points are neighbors in the map
	bool AreNeighbourCorners(const ObjectPoint2D &pt1, 
			const ObjectPoint2D &pt2) const;
	
	/// return the line segment corresp to the map line number 
    void GetHouseLine(int lin_no, Segment2D &seg) const; 
        
    /// return the 2D object point corresp to the given map corner
    const ObjectPoint2D &GetHouseCorner(int corner_no) const;
        
    /// select the segments which can corresp to the given map line
    void GetMapLineSegments(int lin_no, Segments3D &seg) const;
        
    /// test if a point is inside of the house plan
    bool PointInsideHouse(const Position2D &point) const;

	/// test if a segment intersects the house boundary
	int SegmentIntersectHouse(const Segment &seg) const; 

	/// test if a segment intersects the house boundary
    /// the intersection point has to be different than the endpoints of seg
	int SegmentIntersectHouseDiff(const Segment &seg) const; 
        
    /// divide the plan of a house into rectangles
	/// ptno - PointNumber for newly generated corners
    void DivideHousePlan(std::vector<ObjectPoints2D> &rect_list, 
					PointNumber &ptno) const;       

/*        void SplitHouse(const Line2D &lin, std::vector<House> &house_parts) const;
        void GenerateHouseParts(ObjectPoints2D &pts, std::vector<int> online, 
			std::vector<House> &house_parts) const;
*/
    void GetHouseSegments(std::vector<Segment> &lins) const;

	/// finds all collinear segments
   	void GetCollinearSegments(std::vector<Segment> &collin_lins) const;

	/// looks for segments collinear with the given segment
    /// returns true if a segment is found 
   	bool CollectCollinearSegments(const Segment &seg, 
					std::vector<Segment> &collin_lins) const;

    void ShiftHouse(double offsetx, double offsety);
};





#endif /* _House_h_ */   /* Do NOT add anything after this line */
