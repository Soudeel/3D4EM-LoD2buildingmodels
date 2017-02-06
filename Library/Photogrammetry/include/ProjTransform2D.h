
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
 * \brief Interface to Class ProjectiveTransform2D - Projective transformation between two two-dimensional coordinate systems
 *
 */
/*!
 * \class ProjectiveTransform2D
 * \ingroup Photogrammetry
 * \brief Interface to Class ProjectiveTransform2D - Projective transformation between two two-dimensional coordinate systems
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Projective transformation between two two-dimensional coordinate systems
 * This transformation is defined by
 * <br><br>
 *       p0 * x + p1 * y + p2           p3 * x + p4 * y + p5 <br>
 *  x' = --------------------      y' = -------------------- <br>
 *       p6 * x + p7 * y + 1            p6 * x + p7 * y + 1
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ProjTransform2D_h_
#define _ProjTransform2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ProjectiveTransform2D - Projective transformation between two two-dimensional
                          coordinate systems

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
// Projective transformation between two two-dimensional coordinate systems
/* This transformation is defined by

        p0 * x + p1 * y + p2             p3 * x + p4 * y + p5
   x' = --------------------        y' = --------------------
        p6 * x + p7 * y + 1              p6 * x + p7 * y + 1

*/

//------------------------------------------------------------------------------

class ProjectiveTransform2D {

  protected:

    /// The eight parameters of the projective transformation
    double par[8];

  public:

    /// Default constructor
    ProjectiveTransform2D() {};

    /// Construct by reading from a file
    /** @param filename Name of a file with projective transformation parameters
        @param success  Status of the read function. 0 - failure, 1 - success.
    */
    ProjectiveTransform2D(const char *filename, int *success)
      {*success = Read(filename);}

    /// Default destructor
    ~ProjectiveTransform2D() {};

    /// Read projective transformation parameters from a file
    /** @param filename Name of a file with projective transformation parameters
        @return Success status. 0 - failure, 1 - success.
    */
    int Read(const char *filename);

    /// Write projective transformation parameters to a file
    /** @param filename Name of a file for projective transformation parameters
        @return Success status. 0 - failure, 1 - success.
    */
    int Write(const char *filename) const;

    /// Derive the reverse projective transformation
    /** @param invertible This flag is set to 0 if the projective transformation                          could not be inverted. Otherwise, 1 is returned.
        @return The projective transformation parameters of the inverted
                transformation.
    */
    ProjectiveTransform2D Inverse(int &invertible) const;
};
#endif /* _ProjTransform2D_h_ */   /* Do NOT add anything after this line */
