
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
 * \brief Interface to Class RelativeOrientation - Relative orientation between two images
 *
 */
/*!
 * \class RelativeOrientation
 * \ingroup POrientation
 * \brief Interface to Class RelativeOrientation - Relative orientation between two images
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Parameters of the relative orientation
 * This class still needs to be set up properly. The Read and Write functions
 *   read and write two sets of exterior orientation parameters that describe
 *   the poses of two camera coordinate systems with respect to the model
 *   coordinate system. The do not make use of any class variable!
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _RelativeOrientation_h_
#define _RelativeOrientation_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  RelativeOrientation   - Relative orientation between two images

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include "Pose3D.h"
#include "ExteriorOrientation.h"

//------------------------------------------------------------------------------
/// Parameters of the relative orientation
/** This class still needs to be set up properly. The Read and Write functions
    read and write two sets of exterior orientation parameters that describe
    the poses of two camera coordinate systems with respect to the model
    coordinate system. The do not make use of any class variable!
*/
//------------------------------------------------------------------------------

class RelativeOrientation : public Pose3D {

  public:

    /// Default constructor
    RelativeOrientation() : Pose3D() {}

    /// Default destructor
    ~RelativeOrientation() {};

    /// Read two exterior orientations
    /** NOTE: Nothing is stored in the class variables!
        @param filename Name of the file with relative orientation data
        @param extor1   Pose of the first camera in the model coordinate system
        @param extor2   Pose of the second camera in the model coordinate system
        @return Success status, 0 - failure, 1 - success
    */
    int Read(char *filename, ExteriorOrientation *extor1,
             ExteriorOrientation *extor2);

    /// Write two exterior orientations
    /** NOTE: No data of the class variables is used!
        @param filename Name of the file for the relative orientation data
        @param extor1   Pose of the first camera in the model coordinate system
        @param extor2   Pose of the second camera in the model coordinate system
        @return Success status, 0 - failure, 1 - success
    */
    int Write(char *filename, const ExteriorOrientation *extor1,
              const ExteriorOrientation *extor2) const;		     	
};
#endif /* _RelativeOrientation_h_ */   /* Do NOT add anything after this line */
