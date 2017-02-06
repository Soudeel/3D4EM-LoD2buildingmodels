
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
 * \brief Interface to Class TINMesh - A triangular mesh of a TIN
 *
 */
/*!
 * \class TINMesh
 * \ingroup TIN
 * \brief Interface to Class TINMesh - A triangular mesh of a TIN
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A triangular mesh of a TIN
 * The mesh is described by the point numbers of the three mesh corners and
 *   the mesh numbers of the three adjacent meshes. If one of the mesh edges
 *   is on the convex hull of the TIN, there is no adjacent mesh on the other
 *   side of that edge. In that case the mesh number of the adjacent mesh is
 *   set to -1.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _TINMesh_h_
#define _TINMesh_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  TINMesh              - A triangular mesh of a TIN

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Database4Cpp.h"
#include "PointNumber.h"
#include "MeshNumber.h"

//------------------------------------------------------------------------------
/// A triangular mesh of a TIN
/** The mesh is described by the point numbers of the three mesh corners and
    the mesh numbers of the three adjacent meshes. If one of the mesh edges
    is on the convex hull of the TIN, there is no adjacent mesh on the other
    side of that edge. In that case the mesh number of the adjacent mesh is
    set to -1.
*/
//------------------------------------------------------------------------------

class TINMesh {
  protected:

    /// The nodes of the triangle
    PointNumber node[3];

    /// Indices of the neighbouring triangles
    MeshNumber nb[3];

  public:

    /// Default constructor
    TINMesh() {};

    /// Construct from node and mesh numbers
    TINMesh(int node0, int node1, int node2, int mesh0, int mesh1, int mesh2)  
      {node[0] = PointNumber(node0); node[1] = PointNumber(node1);
       node[2] = PointNumber(node2);
       nb[0] = MeshNumber(mesh0);    nb[1] = MeshNumber(mesh1);
       nb[2] = MeshNumber(mesh2);};

    /// Construct from node and mesh numbers
    TINMesh(PointNumber node0, PointNumber node1, PointNumber node2,     
	    MeshNumber mesh0, MeshNumber mesh1, MeshNumber mesh2)  
      {node[0] = node0;   node[1] = node1;   node[2] = node2;
       nb[0]   = mesh0;   nb[1]   = mesh1;   nb[2]   = mesh2;};

    /// Default destructor
    ~TINMesh() {};

    /// Conversion of C++ to C object
    void Cpp2C(Triangle **tri) const;

    /// Conversion of C to C++ object
    void C2Cpp(Triangle *tri);

    /// Return the pointer to the array of the three node numbers
    PointNumber *Nodes()
      { return node;}
    	  
    /// Return the pointer to the array of the three node numbers
    const PointNumber *Nodes() const
      { return node;}
    	  
    /// Return the pointer to the array with the numbers of the three adjacent meshes
    MeshNumber *Neighbours()
      { return nb;}

    /// Return the pointer to the array with the numbers of the three adjacent meshes
    const MeshNumber *Neighbours() const
      { return nb;}

    /// Write a mesh to a VRML file
    /** Write an IndexedFaceSet of the triangle to the VRML file. Note that the 
        point coordinates should already have been written in a point list.
        See e.g. void ObjectPoints::VRML_Write(FILE *).
        @param fd File descriptor of an open VRML file
    */
    void VRML_Write(FILE *fd) const;
};
#endif /* _TINMesh_h_ */   /* Do NOT add anything after this line */
