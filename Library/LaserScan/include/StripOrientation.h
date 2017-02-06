
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
 * \brief Interface to Class StripOrientation - Position and rotation of strip system
 *
 */
/*!
 * \class StripOrientation
 * \ingroup LDataOrg
 * \brief Interface to Class StripOrientation - Position and rotation of strip system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		05-07-2001 (Created)
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



#ifndef _StripOrientation_h_
#define _StripOrientation_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 StripOrientation                 - Position and rotation of strip system

 Initial creation
 Author : George Vosselman
 Date   : 05-07-2001
--------------------------------------------------------------------------------
*/

#include "Orientation3D.h"
#include "ObjectPoint.h"
#include "Positions3D.h"

class LaserUnit;

//------------------------------------------------------------------------------
///         Position and rotation of strip coordinate system
//------------------------------------------------------------------------------

class StripOrientation : public Orientation3D {

  protected:
    /// Known orientation flag
    int known_orientation;

  public:
    /// Default constructor
    StripOrientation()
      {known_orientation = 0;}
 	
    /// Construct from flight path
    StripOrientation(const Positions3D &flight_path_points);

    /// Construct from the moments of the points of a laser strip
    StripOrientation(double m00, double m10, double m01,
                     double m20, double m11, double m02);

    /// Initialisation
    void Initialise();

    /// Return known orientation flag
    int IsKnown() const
      {return(known_orientation);}

    /// Transform terrain point to strip point
    ObjectPoint TerrainToStrip(const ObjectPoint &terrain_point) const;

    /// Transform strip point to terrain point
    ObjectPoint StripToTerrain(const ObjectPoint &strip_point) const;

    /// Return the readable reference
    const StripOrientation &StripOrientationRef() const
      {return(*this);}

    /// Return the writable reference
    StripOrientation &StripOrientationRef()
      {return(*this);}

    /// Read strip orientation from a strip meta data file
    void Read(FILE *file);

    /// Write strip orientation to a strip meta data file
    /** @param file File descriptor
        @param indent Number of characters to indent in BNF file
    */
    void Write(FILE *file, int indent) const;
};

#endif /* _StripOrientation_h_ */  /* Don't add after this point */
