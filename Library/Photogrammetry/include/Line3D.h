
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
 * \brief Interface to Class Line3D - A list of the parameters of a plane in 3D
 *
 */
/*!
 * \class Line3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Line3D - A list of the parameters of a plane in 3D
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg and \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A three-dimensional line
 * The line is defined by a foot point and a line direction vector:
 * <br><br>
 *   X-line = X-footpoint + X-direction * t
 * <br><br>
 *   where t is a scalar.
 * <br>
 *   Constraints are not enforced. I.e. the direction vector is not necessarily
 *   normalised and the footpoint vector is not necessarily perpendicular to the
 *   direction vector.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Line3D_h_
#define _Line3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  Line3D  - A list of the parameters of a plane in 3D
  
  x = x[0] + x[1] * t;
  y = y[0] + y[1] * t;
  z = z[0] + z[1] * t;
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position3D.h"
#include "Rotation3D.h"
#include "Line2D.h"

//------------------------------------------------------------------------------
// A three-dimensional line
/* The line is defined by a foot point and a line direction vector:

    X-line = X-footpoint + X-direction * t

    where t is a scalar.

    Constraints are not enforced. I.e. the direction vector is not necessarily
    normalised and the footpoint vector is not necessarily perpendicular to the
    direction vector.
*/
//------------------------------------------------------------------------------


class Line3D
{

friend class Plane3Points;

protected:
    /// X, Y, and Z coordinates footpoint and the direction vector
    double x[2], y[2], z[2];

    /// Number of points used for sum and moment values
    int num_pts;

    /// Sum of the coordinates of the points in the plane
    Vector3D *coord_sum;

    /// Square sums of the coordinates of the points in the plane
    double *moments;
 	
    /// Offset for moment computations
    Vector3D *offset;
    

public:
    /// Default constructor
	Line3D();
		
    /// Construct from specified values
	Line3D(double xfp, double yfp, double zfp,
               double xdir, double ydir, double zdir) 	
	  { InitialisePointers();
        Initialise();
        x[0] = xfp; x[1] = xdir; 
	    y[0] = yfp; y[1] = ydir;
	    z[0] = zfp; z[1] = zdir; }

	/// Construct from two positions
	Line3D(const Position3D &pt1, const Position3D &pt2);
				
	/// Construct from a position and direction
	Line3D(const Position3D &pt, const Vector3D &dir);
	
    /// Copy constructor
    Line3D(const Line3D &line)
      { *this = line; }
      			
    /// Copy assignment
	Line3D& operator=(const Line3D&);
	  
    /// Default destructor
	~Line3D();

    /// Initialise all pointers of vectors to moment data
    void InitialisePointers();
    
    /// Initialise all variables
    void Initialise();
    
    // Not documented. Usage of Line3DReference const is preferred
	const Line3D &lin() const 
	  { return *this; }
			     		
    // Not documented. Usage of Line3DReference is preferred
    Line3D &lin() 
      { return *this; }

    /// Return readable reference
	const Line3D &Line3DReference() const 
	  {return(*this);}

    /// Return writable reference
	Line3D &Line3DReference()
	  {return(*this);}

    /// Return coordinate sums (used for debugging)
    Vector3D * CoordSum() {return coord_sum;}
      
    /// Return offsets (used for debugging)
    Vector3D * Offsets() {return offset;}
      
    /// Return pointer to moment data (used for debugging)
    double * Moments() {return moments;}
    
    /// Return the foot point of the line
    Position3D FootPoint() const
      {return(Position3D(x[0], y[0], z[0]));}

    /// Set the foot point of the line
    void FootPoint(const Position3D &fp)
      {x[0] = fp.X();  y[0] = fp.Y();  z[0] = fp.Z();}

    /// Set the foot point of the line
    void FootPoint(double xfp, double yfp, double zfp)
      {x[0] = xfp;  y[0] = yfp;  z[0] = zfp;}

    /// Return the direction of the line
    Vector3D Direction() const
      {return(Vector3D(x[1], y[1], z[1]));}

    /// Set the direction of the line
    void Direction(const Vector3D &dir)
      {x[1] = dir.X();  y[1] = dir.Y();  z[1] = dir.Z();}
          
    /// Check if a line is horizontal
    bool IsHorizontal(double angle_tolerance) const
      {return Direction().IsHorizontal(angle_tolerance);}

    /// Set the direction of the line
    void Direction(double xdir, double ydir, double zdir)
      {x[1] = xdir;  y[1] = ydir;  z[1] = zdir;}

    /// Return the position of a point on the line based on the scalar t
    Position3D Position(double t) const
      {return(Position3D(x[0]+t*x[1], y[0]+t*y[1], z[0]+t*z[1]));}

    /// Project a point onto a line
    Position3D Project(const Position3D &pos) const;

    /// Return scalar of a point's projection onto the line
    double Scalar(const Position3D &pos) const;

    /// Return the Y-coordinate for the specified X-coordinate
    /** @return If the line is parallel to the X-axis, a warning is printed
                and the return value is set to 0.
    */
    double Y(double X) const;

    /// Return the Z-coordinate for the specified X-coordinate
    /** @return If the line is parallel to the X-axis, a warning is printed
                and the return value is set to 0.
    */
    double Z(double X) const;
	
    /// Determine the position on the line for which the X-coordinate corresponds to the specified value
    /** @return If the line is parallel to the X-axis, a warning is printed
                and the returned position is set to (-1, -1, -1).
    */
    Position3D DetPositionX(double X) const;

    /// Determine the position on the line for which the Y-coordinate corresponds to the specified value
    /** @return If the line is parallel to the Y-axis, a warning is printed
                and the returned position is set to (-1, -1, -1).
    */
	Position3D DetPositionY(double Y) const;

    /// Determine the position on the line for which the Z-coordinate corresponds to the specified value
    /** @return If the line is parallel to the Z-axis, a warning is printed
                and the returned position is set to (-1, -1, -1).
    */
	Position3D DetPositionZ(double Z) const;	
	
	/// Return the distance of a point to the line
	double DistanceToPoint(const Position3D &pt) const;
	
    /// Test if a point is within a specified distance from the line
	bool PointOnLine(const Position3D &point, double distance) const;
	
	/// Find the intersection point in 3D between a 3D line and a 2D line 
    /** The function intersects in 2D the projection of the 3D line onto the
        XOY plane with the specified 2D line. For this XY location, the
        Z-coordinate of the 3D line is determined. These X, Y, and
        Z-coordinates are returned as the intersection point.
        @return 0 if the projection of the 3D line onto the XOY plane is
                  parallel to the specified line, 1 otherwise
    */
    bool FindIntersection(const Line2D &line, Position3D &intersection) const;         

    /// Project line onto the XOY plane
    Line2D ProjectOntoXOYPlane() const;
    
    /// Print footpoint and direction vectors to stdout
    void Print() const;
        
    /// Print footpoint and direction vectors multiplied by 1000 as integers to stdout
    void PrintInt() const;
       
    //void Rotate(const Rotation3D &rot);
    //void Translate(const Vector3D &trans);

    /// Add a point to the line and (optionally) recalculate the parameters
    bool AddPoint(const Position3D &pos, bool recalculate=true);

    /// Remove a point from the line and (optionally) recalculate 
    bool RemovePoint(const Position3D &pos, bool recalculate=true);

    /// Add or remove a point from the line and (optionally) recalculate 
    bool ChangePoint(const Position3D &pos, bool add,
                     bool recalculate=true);

    /// Recalculate line parameters from moments
    bool Recalculate();
    
    /// Return the number of points in a line
    int NumberOfPoints() const
      {return num_pts;}

    /// Return the noise standard deviation of the line fitting
    double SigmaLineFitting() const;
    
    /// Erase line data
    void Erase();

    /// Return the centre of gravity of points added to the line
    /** If no points have been added using AddPoint, the position returned
        contains coordinates INF
    */
    Position3D CentreOfGravity() const;
    
    /// Return the angle between two lines in radians
    friend double Angle2Lines(const Line3D &line1, const Line3D &line2);

    /// Return the distance between two lines
    friend double Distance2Lines(const Line3D &line1, const Line3D &line2);        
	
	/// Intersect two lines 
    /** WARNING: THIS FUNCTION NEEDS TO BE REWRITTEN!
        The function actually intersects the projections of the two lines
        in the X0Y plane and takes the Z-coordinate of the first line
        at this XY-location as the Z-coordinate of the intersection point.
        THIS FUNCTION WILL FAIL FOR LINES PERPENDICULAR TO THE X0Y PLANE!
    */
  	friend bool Intersection2Lines(const Line3D &line1,
                                   const Line3D &line2, Position3D &pos);	  			
  			
    /// Test if the footpoints and direction vectors of two lines are identical
	friend bool operator==(const Line3D &line1, const Line3D &line2);   				       			
};

#endif /* _Line3D_h_ */   /* Do NOT add anything after this Line3D */
