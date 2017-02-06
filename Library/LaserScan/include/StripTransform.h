
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
 * \brief Interface to Class StripTransform - Transformation between distorted strip coordinate
 *
 */
/*!
 * \class StripTransform
 * \ingroup LDataOrg
 * \brief Interface to Class StripTransform - Transformation between distorted strip coordinate
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		05-07-2001 (Created)
 *
 * \remark \li StripTransform - Transformation between distorted strip coordinate
 *                              system and the terrain coordinate system
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



#ifndef _StripTransform_h_
#define _StripTransform_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 StripTransform             - Transformation between distorted strip coordinate
                              system and the terrain coordinate system

 Initial creation
 Author : George Vosselman
 Date   : 05-07-2001

--------------------------------------------------------------------------------
*/

#include "StripOrientation.h"
#include "StripErrors.h"

//------------------------------------------------------------------------------
///                     Transform in the strip coordinate system
//------------------------------------------------------------------------------

class StripTransform {

  protected:
    /// Strip orientation
    StripOrientation orientation;

    /// Strip errors
    StripErrors      errors;
  
  public:
    /// Default constructor
    StripTransform() {};
 	
    /// Constructor with initialisation
    StripTransform(const StripOrientation &ori, const StripErrors &err)
      {orientation = ori;  errors = err;}

    /// Initialisation
    void Initialise()
      {orientation.Initialise();  errors.Initialise();}

    /// Number of error parameters
    int NumErrorParms(int error_model) const
      {return(errors.NumErrorParms(error_model));}

    /// Transform from terrain to strip coordinate system
    /** @param terrain_point A point in the terrain coordinate system
        @param error_model Error mode to be applied (1 or 2).
                           See LaserBlock::AdjustStrips. If 0, no systematic
                           errors are added
        @return A point in the (distorted) strip coordinate system
    */
    ObjectPoint TerrainToStrip(const ObjectPoint &terrain_point,
                               int error_model) const;

    /// Transform from strip to terrain coordinate system
    /** @param strip_point A point in a (possibly distorted) strip coordinate
                           system.
        @param error_model Error mode to be applied (1 or 2).
                           See LaserBlock::AdjustStrips. If 0, systematic
                           errors are not removed (strip point is assumed to
                           have none).
        @return A point in the terrain coordinate system
    */
    ObjectPoint StripToTerrain(const ObjectPoint &strip_point,
                               int error_model) const;

    /// Return the readable strip orientation
    const StripOrientation &Orientation() const
      {return(orientation);}

    /// Return the writeble strip orientation
    StripOrientation &Orientation()
      {return(orientation);}

    /// Return the readable strip errors
    const StripErrors &Errors() const
      {return(errors);}

    /// Return the writeble strip errors
    StripErrors &Errors()
      {return(errors);}

    /// Return the readable reference
    const StripTransform &StripTransformRef() const
      {return(*this);}

    /// Return the writable reference
    StripTransform &StripTransformRef()
      {return(*this);}

   /// Partial derivatives of strip point to errors in terrain coordinate system
    /** @param strip_point Point in a strip coordinate system
        @param a Vector with NumErrorParms doubles to avoid reallocation
                 If NULL, a vector will be allocated.
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
        @return The row of the A-matrix with partial derivatives of the
                strip point with respect to the error parameters in the
                terrain coordinate system
    */
    double *PartialDerivatives(const ObjectPoint &strip_point,
                               double *a, int error_model) const;

    /// Correct observed terrain point for (approximate) strip errors
    /** @param point_with_errors Terrain point with systematic errors
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
        @return Terrain point corrected for systematic errors
    */
    ObjectPoint CorrectTerrainPoint(const ObjectPoint &point_with_errors,
                                    int error_model) const;
};

#endif /* _StripTransform_h_ */  /* Don't add after this point */
