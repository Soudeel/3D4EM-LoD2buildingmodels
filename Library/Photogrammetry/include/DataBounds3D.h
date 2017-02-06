
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
 * \brief Interface to Class DataBounds3D - Data bounds of a 3D dataset
 *
 */
/*!
 * \class DataBounds3D
 * \ingroup Photogrammetry
 * \brief Interface to Class DataBounds3D - Data bounds of a 3D dataset
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _DataBounds3D_h_
#define _DataBounds3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  DataBounds3D                     - Data bounds of a 3D dataset

--------------------------------------------------------------------------------
*/

#include "Position3D.h"
#include "ObjectPoints.h"
#include "DataBounds2D.h"

//------------------------------------------------------------------------------
/// Data bounds of a 3D dataset
//------------------------------------------------------------------------------

class DataBounds3D
{
  protected:

    /// Minimum coordinates
    Position3D minimum;

    /// Maximum coordinates
    Position3D maximum;

    /// 0/1 switch for (un)known minima
    int min_is_set[3];

    /// 0/1 switch for (un)known maxima
    int max_is_set[3];

  public:

    /// Default constructor
    DataBounds3D() { Initialise(); }

    /// Default destructor
    ~DataBounds3D() {};

    /// Copy assignment
    DataBounds3D operator = (const DataBounds3D &);

    /// Return the minimum coordinates
    Position3D Minimum() {return(minimum);}

    /// Return the minimum coordinates
    const Position3D Minimum() const {return(minimum);}

    /// Return the maximum coordinates
    Position3D Maximum() {return(maximum);}

    /// Return the maximum coordinates
    const Position3D Maximum() const {return(maximum);}

    /// Set all values to unknown
    void Initialise();

    /// Set the minimum X coordinate
    void SetMinimumX(double Xmin);
    /// Set the minimum Y coordinate
    void SetMinimumY(double Ymin);
    /// Set the minimum Z coordinate
    void SetMinimumZ(double Zmin);
    /// Set the maximum X coordinate
    void SetMaximumX(double Xmax);
    /// Set the maximum Y coordinate
    void SetMaximumY(double Ymax);
    /// Set the maximum Z coordinate
    void SetMaximumZ(double Zmax);

    /// Check if the minimum X coordinate is set
    int MinimumXIsSet() const;
    /// Check if the minimum Y coordinate is set
    int MinimumYIsSet() const;
    /// Check if the minimum Z coordinate is set
    int MinimumZIsSet() const;
    /// Check if the maximum X coordinate is set
    int MaximumXIsSet() const;
    /// Check if the maximum Y coordinate is set
    int MaximumYIsSet() const;
    /// Check if the maximum Z coordinate is set
    int MaximumZIsSet() const;

    /// Check if all X and Y bounds are set
    int XYBoundsSet() const;
    /// Check if all X, Y, and Z bounds are set
    int XYZBoundsSet() const;

    /// Return the X coordinate range
    double XRange() const;
    /// Return the Y coordinate range
    double YRange() const;
    /// Return the Z coordinate range
    double ZRange() const;

    /// Update the data bounds with another set of data bounds
    DataBounds3D Update(const DataBounds3D &bounds);
    /// Update the data bounds with values of a 3D position
    DataBounds3D Update(const Position3D &pos);
    /// Update the data bounds with a set of object points
    DataBounds3D Update(const ObjectPoints &points);

    /// Check if some data bound is set
    int HasSomeData() const;

    /// Check if a point is inside the X and Y bounds
    bool InsideXY(const Position3D &pos) const;
    /// Check if a point is inside the X and Y bounds relaxed with a margin
    bool InsideXY(const Position3D &pos, double margin) const;
    /// Check if a point is inside all set bounds
    bool Inside(const Position3D &pos) const;
    /// Check if a point is inside all set bounds relaxed with a margin
    bool Inside(const Position3D &pos, double margin) const;

    /// Return the centre of the bounding box
    Position3D MidPoint() const;

    /// Extract 2D bounds
    DataBounds2D & Bounds2D() const;
    
    /// Check if two data bounds show an overlap
    friend bool Overlap(const DataBounds3D &bounds1,
                        const DataBounds3D &bounds2);

    /// Check if two data bounds show an overlap in X and Y coordinates
    friend bool OverlapXY(const DataBounds3D &bounds1,
                          const DataBounds3D &bounds2);

    /// Check if two data bounds are the same
    friend bool operator == (const DataBounds3D &bounds1,
                             const DataBounds3D &bounds2);
};
#endif /* _DataBounds3D_h_ */  /* Don't add after this point */
