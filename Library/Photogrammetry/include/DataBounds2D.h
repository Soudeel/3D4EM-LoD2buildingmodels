
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
 * \brief Interface to Class DataBounds2D - Data bounds of a 2D dataset
 *
 */
/*!
 * \class DataBounds2D
 * \ingroup Photogrammetry
 * \brief Interface to Class DataBounds2D - Data bounds of a 2D dataset
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


#ifndef _DataBounds2D_h_
#define _DataBounds2D_h_

/*
--------------------------------------------------------------------------------
  DataBounds2D                     - Data bounds of a 2D dataset
--------------------------------------------------------------------------------
*/

#include "Position2D.h"
#include "Position3D.h"

//------------------------------------------------------------------------------
/// Data bounds of a 2D dataset
//------------------------------------------------------------------------------

class DataBounds2D
{
  protected:

    /// Minimum coordinates
    Position2D minimum;

    /// Maximum coordinates
    Position2D maximum;

    /// 0/1 switch for (un)known minima
    int min_is_set[2];

    /// 0/1 switch for (un)known maxima
    int max_is_set[2];

  public:

    /// Default constructor
    DataBounds2D() { Initialise(); }

    /// Default destructor
    ~DataBounds2D() {};

    /// Copy assignment
    DataBounds2D operator = (const DataBounds2D &);

    /// Return the minimum coordinates
    Position2D Minimum() {return(minimum);}

    /// Return the minimum coordinates
    const Position2D Minimum() const {return(minimum);}

    /// Return the maximum coordinates
    Position2D Maximum() {return(maximum);}

    /// Return the maximum coordinates
    const Position2D Maximum() const {return(maximum);}

    /// Set all extrema values to unknown
    void Initialise();

    /// Set the minimum X coordinate
    void SetMinimumX(double Xmin);
    /// Set the minimum Y coordinate
    void SetMinimumY(double Ymin);
    /// Set the maximum X coordinate
    void SetMaximumX(double Xmax);
    /// Set the maximum Y coordinate
    void SetMaximumY(double Ymax);

    /// Check if the minimum X coordinate is set
    int MinimumXIsSet() const;
    /// Check if the minimum Y coordinate is set
    int MinimumYIsSet() const;
    /// Check if the maximum X coordinate is set
    int MaximumXIsSet() const;
    /// Check if the maximum Y coordinate is set
    int MaximumYIsSet() const;

    /// Check if all X and Y bounds are set
    int XYBoundsSet() const;

    /// Return the X coordinate range
    double XRange() const;
    /// Return the Y coordinate range
    double YRange() const;

    /// Update the data bounds with another set of data bounds
    DataBounds2D Update(const DataBounds2D &bounds);
    /// Update the data bounds with values of a 2D position
    DataBounds2D Update(const Position2D &pos);

    /// Check if some data bound is set
    int HasSomeData() const;

    /// Check if a point is inside the bounds
    int Inside(const Position2D &pos) const;

    /// Check if a point is inside the bounds
    int Inside(const Position3D &pos) const;

    /// Return the centre of the bounding box
    Position2D MidPoint() const;

    /// Check if two data bounds overlap
    int Overlap(const DataBounds2D &bounds2) const;

    /// Check if two data bounds overlap
    friend int Overlap(const DataBounds2D &bounds1,
                       const DataBounds2D &bounds2);
};
#endif /* _DataBounds2D_h_ */  /* Don't add after this point */
