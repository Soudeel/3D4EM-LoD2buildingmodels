
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
 * \brief Interface to Class ModelPoints - A set of stochastic three-dimensional model points
 *
 */
/*!
 * \class ModelPoints
 * \ingroup Photogrammetry
 * \brief Interface to Class ModelPoints - A set of stochastic three-dimensional model points
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

#ifndef _ModelPoints_h_
#define _ModelPoints_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ModelPoints  - A set of stochastic three-dimensional model points

--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "ModelPoint.h"
#include "Database4Cpp.h"


//------------------------------------------------------------------------------
/// A vector of model points
//------------------------------------------------------------------------------

class ModelPoints : public VectorPoint<ModelPoint>
{
  public:

    /// Default constructor
    ModelPoints() : VectorPoint<ModelPoint>() {}

    /// Construct by converting a C object
    ModelPoints(ModelPts *modelpts) : VectorPoint<ModelPoint>()
       { C2Cpp(modelpts); }

    /// Destructor
    ~ModelPoints() {};

    /// Conversion from C to C++ object
    void C2Cpp(ModelPts *);

    /// Conversion from C++ to C object
    void Cpp2C(ModelPts **) const;

    /// Read the model points from a file
    /** @param filename File with model points
        @return 0 - failure, 1 - success
    */
    int Read(char *filename);

    /// Write the model points to a file
    /** @param filename File for model points
        @return 0 - failure, 1 - success
    */
    int Write(char *filename) const;

    /// Print the model points to stdout
    void Print() const;
};
#endif /* _ModelPoints_h_ */   /* Do NOT add anything after this line */
