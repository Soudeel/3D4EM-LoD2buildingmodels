
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
 * \brief Interface to Class Adjustment - Class for a generic least squares adjustment
 *
 */
/*!
 * \class Adjustment
 * \ingroup Adjustment
 * \brief Interface to Class Adjustment - Class for a generic least squares adjustment
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		06-07-2001 (Created)
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

#ifndef _Adjustment_h_
#define _Adjustment_h_

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 This file contains the definitions of the following classes:

 Adjustment                 - Class for a generic least squares adjustment

 Initial creation
 Author : George Vosselman
 Date   : 06-07-2001

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#include <stdio.h>
#include <math.h>
#include "vMatrix.h"

//------------------------------------------------------------------------------
///           Matrices for a generic least squares adjustment
//------------------------------------------------------------------------------

class Adjustment {

  protected:
    /// Normal matrix
    vMatrix ata;

    /// Observations premultiplied by design matrix
    vMatrix aty;

    /// Estimated unknowns
    vMatrix x;

    /// Covariances of estimated unknowns
    vMatrix Qxx;
  
  public:
    /// Default constructor
    Adjustment() {};
 	
    /// Constructor with initialisation
    Adjustment(int unknowns)
      {Initialise(unknowns);}

    /// Initialisation and memory allocation
    void Initialise(int);

    /// Clear all arrays
    void Clear();

    /// Number of unknowns
    int NumberOfUnknowns() const
      {return(ata.NumRows());}

    /// Return the readable reference
    const Adjustment &AdjustmentRef() const
      {return(*this);}

    /// Return the writable reference
    Adjustment &AdjustmentRef()
      {return(*this);}

    /// Print the normal matrix
    void PrintNormalMatrix(char *format) const
      {ata.Print(format);}

    /// Add an observation (vector) to the equation system
    int AddObservations(vMatrix &, vMatrix &, vMatrix &);

    /// Solve the normal equation system
    int SolveEquationSystem();

    /// Return the normal equation matrix
    const vMatrix &NormalMatrix() const
      {return(ata);}

    /// Return the estimated parameters
    const vMatrix &EstPar() const
      {return(x);}

    /// Return the covariance matrix of the estimated parameters
    const vMatrix &CovEstPar() const
      {return(Qxx);}
};

#endif /* _Adjustment_h_ */  /* Don't add after this point */
