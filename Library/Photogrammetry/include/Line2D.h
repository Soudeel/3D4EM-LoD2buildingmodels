
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



/*!
 * \file
 * \brief Interface to Class Line2D - A list of the parameters of a plane line
 *
 */
/*!
 * \class Line2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Line2D - A list of the parameters of a plane line
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg and \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li The line is parameterised by
 * <br>
 * d = x * cos(phi) + y * sin(phi)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Line2D_h_
#define _Line2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  Line2D  - A list of the parameters of a plane line
  
  d = x * cosphi + y * sinphi
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <math.h>
#include "Positions2D.h"

//------------------------------------------------------------------------------
// A two-dimensional line
/* The line is parameterised by
    d = x * cos(phi) + y * sin(phi)
*/
//------------------------------------------------------------------------------

class Line3D;
class Position3D;
class DataBounds2D;

class Line2D
{
  protected:
    /// Cosine and sine of line normal direction
	double sinphi, cosphi;
	
    /// Distance of line to origin
	double d;
  
  public:
    /// Default constructor             	
	Line2D()			
	  {}; 
	  
	/// Construct from specified values
    /** @param sine     Sine of the line normal direction
        @param cosine   Cosine of the line normal direction
        @param distance Perpendicular distance of line to origin
    */
	Line2D(double sine, double cosine, double distance);

	/// Construct a line from two positions
	Line2D(const Position2D &pt1, const Position2D &pt2);

	/// Construct a line from two 3D positions
	Line2D(const Position3D &pt1, const Position3D &pt2);

	/// Copy constructor
	Line2D(const Line2D &line)
      { *this = line; }

	/// Construct a line from a 3D line
	Line2D(const Line3D &line);

	/// Construct a line by a least squares fit to a set of points
	Line2D(const Positions2D &pts);
				
	/// Destructor  
	~Line2D() 
	  {}; 
	  
	/// Copy assignment          			
	Line2D& operator=(const Line2D&);     			
	  	
        // Not documented. Usage of Line2DReference const should be preferred
	const Line2D &lin() const
	  { return *this; }
	
        // Not documented. Usage of Line2DReference should be preferred
	Line2D &lin()
	  { return *this; }  
	  
        /// Return the readable reference
	const Line2D &Line2DReference() const
	  { return *this; }
	
        /// Return the writable reference
	Line2D &Line2DReference()
	  { return *this; }  
	  
	/// Return the cosine of the line's normal direction
    double Getcosphi() const
      { return cosphi; }
          
	/// Return the sine of the line's normal direction
    double Getsinphi() const
      { return sinphi; }
          
    /// Return the perpendicular distance of the line to the origin
    double GetDisto() const
      { return d; }            
  	
  	/// Return the distance of the line to the origin
  	double DistanceToOrigin() const
  	  { return d; }
  	  
  	/// Set the distance of the line to the origin
  	void SetDistanceToOrigin(double dist)
  	  { d = dist; }
  	  
  	/// Get the Y-coordinate for the given X-coordinate
    /** If the line is parallel to the Y-axis, the distance of the line
        to the origin is returned.
    */
	double Y(double X) const;

	/// Return the distance of a point to the line
	double DistanceToPoint(const Position2D &pos) const;

        /// Return the signed distance of a point to the line
	double DistanceToPointSigned(const Position2D &pos) const;
		
	/// Test if a point is on the line		
	bool PointOnLine(const Position2D &pt, double err_dist) const;
				     	
	/// Construct a line that is parallel with this line and which goes through the specified position
    /** @param pos Position on the line to be constructed
        @param lin The line to be constructed
    */
	void FindParallelLine(const Position2D &pos, Line2D &lin) const;

	/// Construct two lines at a specified distance and parallel with this line 
    /** @param dist  Distance of the lines to be constructed to this line
        @param posm  Some position
        @param lin1  First line to be constructed (reduced distance)
        @param posm1 Point posm moved by 0.5 in direction of the first line
        @param lin2  Second line to be constructed (increased distance)
        @param posm2 Point posm moved by 0.5 in direction of the second line
    */
    void FindParallelLines(double dist, const Position2D &posm, 
      		               Line2D &lin1, Position2D &posm1, 
                           Line2D &lin2, Position2D &posm2) const;

	/// Return the perpendicular line to this line through a given point
	Line2D PerpendicularLine(const Position2D &pos) const;
	
	/// Return the angle of the line with X-axis
    double AngleOx() const;

    /// Return the normal direction in radians according to the line definition
    double NormalAngle() const
      {return(atan2(sinphi, cosphi));}

	/// Construct two points which are at a given distance in the line direction to a given point        
    /** @param pt   Given point
        @param dist Distance of new points to given point
        @param pt1  First constructed point 
        @param pt2  Second constructed point 
    */
    bool PointsAtDistance(const Position2D &pt, double dist, 
       		              Position2D &pt1, Position2D &pt2) const;
        
    /// Return the angle between two lines in radians
    friend double Angle2Lines(const Line2D &lin1, const Line2D &lin2);

	/// Determine the intersection point of two lines
    /** @param lin1 First line
        @param lin2 Second line
        @param pos  Constructed intersection point
        @return 0 - Parallel lines, no intersection, 1 - Success
    */
  	friend bool Intersection2Lines(const Line2D &lin1, 
  			                       const Line2D &lin2, Position2D &pos);	
       
    /// Fast intersection of two lines, no test for parallel lines!
    friend void Intersection2NonParallelLines(const Line2D &lin1,
                                              const Line2D &lin2, 
                                              Position2D &pos);
   				       
   	/// Construct the bisector of two lines
   	friend Line2D & Bisector(const Line2D &line1, const Line2D &line2);
   	
	/// Check if two lines are parallel
    /** @param lin1      First line
        @param lin2      Second line
        @param err_angle Allowed angle between the two lines
    */
    friend bool ParallelLines(const Line2D &lin1, const Line2D &lin2, 
        		              double err_angle);    			
        			     
	/// Check if two lines are collinear         			     
    /** @param lin1      First line
        @param lin2      Second line
        @param err_angle Allowed angle between the two lines
        @param err_dist  Allowed difference between distance to origin
    */
    friend bool CollinearLines(const Line2D &lin1, const Line2D &lin2, 
        		               double err_angle, double err_dist);    				     

    /// Test on equality of all class parameters
	friend bool operator==(const Line2D &lin1, const Line2D &lin2);   				     

    /// Print all class parameters to stdout
    void Print() const;   				  				

// Additional functions with footpoints and directions

    /// Return the line's footpoint (point nearest to origin)
    Position2D FootPoint() const;

    /// Return the line's direction (perpendicular to normal direction)
    Vector2D Direction() const;

    /// Return the line's normal direction
    Vector2D Normal() const;

    /// Project a point onto the line
    Position2D Project(const Position2D &pos) const;

    /// Return the scalar of a point's projection onto the line
    /** The scalar is defined as the signed distance of a point to the
        origin in the direction of the line.
    */
    double Scalar(const Position2D &pos) const;

    /// Position of a point on the line derived from the scalar
    /** The scalar is defined as the signed distance of a point to the
        origin in the direction of the line. For scalar 0, the point equals
        the foot point of the line.
    */
    Position2D Position(double scalar) const;
    
    /// Swap the normal direction
    void SwapNormal();
    
    /// Determine the bounding box of the rotated rectangle
    /** @param scalar_min Minimum scalar of line part
        @param scalar_max Maximum scalar of line part
        @param max_dist   Half width of the rotated rectangle
        @return Bounding box of the rotated rectangle
    */
    DataBounds2D & BoundingBox(double scalar_min, double scalar_max,
                               double max_dist) const;
};

#endif /* _Line2D_h_ */   /* Do NOT add anything after this line */
