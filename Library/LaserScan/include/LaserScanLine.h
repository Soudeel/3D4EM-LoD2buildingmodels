
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
 * \brief Interface to Class LaserScanLine - Information on a scan line
 *
 */
/*!
 * \class LaserScanLine
 * \ingroup LDataOrg
 * \brief Interface to Class LaserScanLine - Information on a scan line
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */



#ifndef _LaserScanLine_h_
#define _LaserScanLine_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserScanLine                       - Information on a scan line

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "PointNumber.h"
#include "LaserPoints.h"

//------------------------------------------------------------------------------
///                             Laser scan line
//------------------------------------------------------------------------------

class LaserScanLine
{
  protected:
    /// Point number of scan line begin
    PointNumber beginning;

    /// Point number of scan line end
    PointNumber ending;

  public:

    /// Default constructor
    LaserScanLine() {};

    /// Constructor
    LaserScanLine(int b, int e)
      {beginning = PointNumber(b);
       ending    = PointNumber(e);}

    /// Constructor
    LaserScanLine(const PointNumber &b, const PointNumber &e)
      {beginning = b; ending = e;}

    /// Construct from LaserPoint::iterators
    LaserScanLine(const LaserPoints &points,
	              LaserPoints::const_iterator b,
				  LaserPoints::const_iterator e);
				  
    /// Default destructor
    ~LaserScanLine() {};

    /// Return readable reference
	const LaserScanLine &LaserScanLineRef() const 
	  {return(*this);}

    /// Return writable reference
	LaserScanLine &LaserScanLineRef()
	  {return(*this);}

    /// Set begin and end of scan line
    void SetBeginAndEnd(const LaserPoints &points,
	                    LaserPoints::const_iterator b,
			     	    LaserPoints::const_iterator e);

    /// Return the first point number
    PointNumber Begin() const
      {return(beginning);}

    /// Return the last point number
    PointNumber End() const
      {return(ending);}
      
    /// Return the first point
    LaserPoints::const_iterator begin(const LaserPoints &points,
                                      int first_number=0) const;

    /// Return the last point
    LaserPoints::const_iterator end(const LaserPoints &points,
                                    int first_number=0) const;

    /// Return the first point
    LaserPoints::iterator begin(LaserPoints &points,
                                int first_number=0) const;

    /// Return the last point
    LaserPoints::iterator end(LaserPoints &points,
                              int first_number=0) const;

    /// Return the first point
    LaserPoints::const_iterator begin(const LaserPoints *points,
                                      int first_number=0) const;

    /// Return the last point
    LaserPoints::const_iterator end(const LaserPoints *points,
                                    int first_number=0) const;

    /// Return the first point
    LaserPoints::const_iterator begin(LaserPoints::const_iterator first_point_of_set,
                                      int first_number=0) const;

    /// Return the last point
    LaserPoints::const_iterator end(LaserPoints::const_iterator first_point_of_set,
                                    int first_number=0) const;

    /// Return the first point
    LaserPoints::iterator begin(LaserPoints::iterator first_point_of_set,
                                int first_number=0) const;

    /// Return the last point
    LaserPoints::iterator end(LaserPoints::iterator first_point_of_set,
                              int first_number=0) const;

    /// Print the scan line
    void Print() const;

    /// Return the number of points on this scan line
    int NumberOfPoints() const
      { return(ending.Number() - beginning.Number() + 1); }
      
    /// Length of scan line
    double Length(const LaserPoints &points) const;

    /// Length of scan line
    double Length(LaserPoints::const_iterator first_point_of_set) const;

    /// Reconstruct the scanner position
    /** Three methods are available for the reconstruction of the scanner
        position.
        Method 0 (default) determines the X- and Y-coordinate by taking the
        averages of the two end points of the scan line. The user-specified
        height is used as the Z-coordinate.
        Method 1 and 2 reconstruct the scanner position by estimating the
        location at which the angles between two consecutive laser beams is
        best approximated by a constant. I.e. the assumption is that the mirror
        speed as well as the laser pulse frequency is constant. In the case
        of method 1, this constant value is estimated. In the case of method 2,
        the user-specified value is used. This method is recommended for
        scanners with small opening angles, as the angle increment is highly
        correlated with the flight height. 
        @param points The laser points of the strip
        @param height The approximated height of the scanner
        @param scanner_position The reconstructed scanner position
        @param method The method number (0 or 1)  
        @param angle_increment Fixed user-specified angle increment
        @return true if the reconstruction succeeded, false otherwise.
    */
    bool ScannerPosition(const LaserPoints &points, double height,
                         Position3D &scanner_position, int method=0,
                         double angle_increment=0.0) const;

    /// Estimate the scan angle increment between consecutive points
    /** This is done by computing all angle increments and taking the
        percentile specified value of the sorted increments.
        @param points The laser points of the strip
        @param scanner_position Approximate position of the laser scanner
        @param percentage (0-100) percentile to be taken from the sorted angles
        @return The computed incremental scan angle
    */
    double ScanAngleIncrement(const LaserPoints &points,
                              const Position3D &scanner_position,
                              double percentage) const;
};

#endif /* _LaserScanLine_h_ */  /* Don't add after this point */
