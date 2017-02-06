
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
 * \brief Interface to Class StripErrors - Errors in the strip coordinate system
 *
 */
/*!
 * \class StripErrors
 * \ingroup LDataOrg
 * \brief Interface to Class StripErrors - Errors in the strip coordinate system
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



#ifndef _StripErrors_h_
#define _StripErrors_h_

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 This file contains the definitions of the following classes:

 StripErrors                 - Errors in the strip coordinate system

 Initial creation
 Author : George Vosselman
 Date   : 05-07-2001

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#include "vMatrix.h"
#include "Vector3D.h"
#include "ObjectPoint.h"
#include "RotationParameter.h"


//------------------------------------------------------------------------------
///                     Errors in the strip coordinate system
//------------------------------------------------------------------------------

class StripErrors {

  protected:
    /// Translation error
    Vector3D      trans_error;

    /// Rotation error
    EulerRotation rot_error;

    /// Time dependent rotation error
    EulerRotation rot_error_time_dep;
  
  public:
    /// Default constructor
    StripErrors() {};
 	
    /// Constructor with initialisation
    /** @param te Translation error
        @param re Rotation error
        @param retd Time dependent rotation error (drift)
    */
    StripErrors(const Vector3D &te, const EulerRotation &re,
                const EulerRotation &retd)
      {trans_error = te;  rot_error = re;  rot_error_time_dep = retd;}

    /// Initialisation
    void Initialise();

    /// Number of error parameters
    int NumErrorParms(int error_model) const;

    /// Apply error correction to a distorted strip point
    /** @param point_with_error Original point
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
        @return Point corrected for systematic errors
    */
    ObjectPoint CorrectStripPoint(const ObjectPoint &point_with_error,
                                  int error_model) const;

    /// Distort a correct strip point
    /** @param good_point Point without systematic errors
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
        @return Point with systematic errors
    */
    ObjectPoint DistortStripPoint(const ObjectPoint &good_point,
                                  int error_model) const;

    /// Return the readable reference
    const StripErrors &StripErrorsRef() const
      {return(*this);}

    /// Return the writable reference
    StripErrors &StripErrorsRef()
      {return(*this);}

    /// Partial derivatives of strip point to errors in strip coordinate system
    /** @param strip_point Point in a strip coordinate system
        @param a Vector with NumErrorParms doubles to avoid reallocation
                 If NULL, a vector will be allocated.
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
        @return The row of the A-matrix with partial derivatives
    */
    double *PartialDerivatives(const ObjectPoint &strip_point, double *a,
                               int error_model) const;

    /// Update the errors
    /** Add the estimated increments of the errors to the error values
        @param increments Estimated increments
        @param error_model Error model (1 or 2). See LaserBlock::AdjustStrips.
    */
    void Update(const vMatrix &increments, int error_model);

    /// Return offsets
    const Vector3D &Offsets() const
      {return(trans_error);}

    /// Return rotation errors
    const EulerRotation &RotError() const
      {return(rot_error);}

    /// Return time dependent rotation errors
    const EulerRotation &TimeDepRotError() const
      {return(rot_error_time_dep);}
};

#endif /* _StripErrors_h_ */  /* Don't add after this point */
