
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
 * \brief Interface to Class TIN - A triangular irregular network
 *
 */
/*!
 * \class TIN
 * \ingroup TIN
 * \brief Interface to Class TIN - A triangular irregular network
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A triangular irregular network based on the Delaunay triangulation
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _TIN_h_
#define _TIN_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  TIN        - A triangular irregular network

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                  Include files
--------------------------------------------------------------------------------
*/

#include "TINMesh.h"
#include "Positions2D.h"
#include "Database4Cpp.h"

//------------------------------------------------------------------------------
/// A triangular irregular network based on the Delaunay triangulation
//------------------------------------------------------------------------------

class TIN : public std::vector<TINMesh>
{
  public:

    /// Default constructor
    TIN(void) : std::vector<TINMesh>() {}
    
    /// Copy constructor
    TIN(TIN &tin) : std::vector<TINMesh>()
      {*this = tin;}

    /// Construct by reading from a file
    /** @param filename Name of the file with a TIN
        @param success  Return status of the read function.
                        0 - failure, 1 - success
    */
    TIN(char *filename, int *success);

    /// Construct by converting a C object
    TIN(Triangles *triangles);
    
    /// Construct by triangulating points with constraints and holes
    /** See Derive() for a description of the parameters.
    */
    TIN(double *coord, int num_pts, int *edges=NULL, int num_edges=0,
        double *hole_coord=NULL, int num_holes=0)
      {Derive(coord, num_pts, edges, num_edges, hole_coord, num_holes);}

    /// Default destructor
    ~TIN();

    /// Return the writable reference
    TIN &TINReference()
      {return(*this);}

    /// Return the readable reference
    const TIN &TINReference() const
      {return(*this);}

    /// Copy assignment
    TIN & operator = (const TIN &);

    /// Conversion from C to C++ object
    void C2Cpp(Triangles *triangles);

    /// Conversion from C++ to C object
    void Cpp2C(Triangles **triangles);

    /// Read the TIN from a file
    /** @param filename Name of the file with a TIN
        @return Success status, 0 - failure, 1 - success
    */
    int Read(char *filename);

    /// Write the TIN to a file
    /** @param filename Name of the file for a TIN
        @return Success status, 0 - failure, 1 - success
    */
    int Write(char *filename);

    /// Write the TIN to a compressed file
    /** @param filename Name of the file for a TIN
        @param compress If true, the file will be gnu-zipped after writing
        @return Success status, 0 - failure, 1 - success
    */
    int Write(char *filename, int compress);

    /// Erase the TIN and free the memory
    void Erase();

    /// Derive the TIN from a coordinate list
    /** Delaunay triangulation of points. The triangulation can be constrained
        by specified closed polygons of the triangulation and by holes. In case
        of a constrained triangulations, points outside the specified polygons
        will result in an invalid triangulation. The polygons are specified by
        index pairs of the points of the polygon edges. When using holes, the
        edges of the holes need to be part of the list of polygon edges. In
        addition, a position inside each hole needs to be specified. Note that
        polygons and holes are optional. Calling this function with only the
        first two arguments will result in an unconstrained triangulation.

        @param coord      An array with X- and Y-coordinates of the points to be
                          triangulated, ordered as X0, Y0, X1, Y1, X2, Y2, ...
        @param num_pts    Number of points in the coordinate array
        @param edges      An array with pairs of point indices of the edges of
                          constraining polygons. These can be both edges of
                          polygons that surround the TIN and edges of holes in
                          the TIN. The index of the first point is 0. The edges
                          of a polygon through points 0, 1, and 2, need to be
                          specified as 0 1 1 2 2 0, i.e. three edges with the
                          indices of the end nodes. In this case, num_edges
                          should be set to 3.
        @param num_edges  Number of polygon edges
        @param hole_coord For each hole this array should contain the X- and Y-
                          coordinate of a position in the hole.
        @param num_holes  Number of holes.
    */
    void Derive(double *coord, int num_pts, int *edges=NULL, int num_edges=0,
                double *hole_coord=NULL, int num_holes=0);

    /// Write the TIN meshes to a VRML file
    /** The topology of all TIN meshes is written as IndexedFaceSet's to the
        VRML file. It is assumed that the coordinates are already written
        to that file. See e.g. ObjectPoints::VRML_Write(FILE *).
        @param fd File descriptor of an open VRML file
    */
    void VRML_Write(FILE *fd) const;
    
    /// Return the highest node number in the TIN
    PointNumber HighestNodeNumber() const;
};
#endif /* _TIN_h_ */   /* Do NOT add anything after this line */
