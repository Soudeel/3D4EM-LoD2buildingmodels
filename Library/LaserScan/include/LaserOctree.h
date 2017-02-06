
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
 * \brief Interface to Class LaserOctree - Laser points organised in an octree
 *
 */
/*!
 * \class DataBoundsLaser
 * \ingroup LDataOrg
 * \brief Interface to Class LaserOctree - Laser points organised in an octree
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		07-10-2004 (Created)
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

#ifndef _LaserOctree_h_
#define _LaserOctree_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserOctree                     - Octree of laser data

--------------------------------------------------------------------------------
*/

#include "LaserPoints.h"

//------------------------------------------------------------------------------
/*                  Octree of laser data                                     */
//------------------------------------------------------------------------------

class LaserOctree
{
  protected:

    /// Bounds of the cube
    DataBounds3D bounds;

    /// Border margin outside cube
    double margin;

    /// Child cubes
    std::vector <LaserOctree> children;

    /// Laser point numbers
    PointNumberList numbers;

    /// Maximum number of points per cube
    int max_num_pts;

  public:

    /// Default constructor
    LaserOctree() { Initialise(); }

    /// Construct from 3D data bounds and a margin
    LaserOctree(const DataBounds3D &new_bounds, double new_margin,
                int new_max_num_pts)
      { Initialise(new_bounds, new_margin, new_max_num_pts); }

    /// Default destructor
    ~LaserOctree() {};

    /// Return the bounds
    const DataBounds3D &Bounds() const {return bounds;}

    /// Return the margin
    const double &Margin() const {return margin;}

    /// Set bounds to unknown and margin to zero and delete old data if present
    void Initialise();

    /// Initialise bounds and margin and delete old data if present
    void Initialise(const DataBounds3D &new_bounds, double new_margin,
                    int new_max_num_pts);

    /// Erase old data
    void Erase();

    /// Return the reference
    LaserOctree &LaserOctreeRef() {return *this;}

    /// Return the const reference
    const LaserOctree &LaserOctreeRef() const {return *this;}

    /// Insert a laser point
    void Insert(const LaserPoints &points, const PointNumber &number);

    /// Insert all laser points
    void Insert(const LaserPoints &points);

    /// Split into children
    void Split(const LaserPoints &points);

    /// Count the number of cubes
    int NumberOfCubes(bool leaves_only=false) const;

    /// Depth of the octree
    int Depth() const;

    /// Number of points in the octree
    int NumberOfPoints() const;

    /// Derive neighbourhood edges
    TINEdges * NeighbourhoodEdges3D(const LaserPoints &points,
                                    double range) const;

    /// Collect all points within some distance
    void Neighbourhood(PointNumberList &neighbourhood, const Position3D &pos,
                       const LaserPoints &points, double range) const;

    /// Return the point number list
    const PointNumberList & PointNumbers() const { return numbers; }

    /// Locate the leaf cube of a specified position
    const LaserOctree * LeafCube(const Position3D &pos) const; 

  private:
    /// Add points to a neighbourhood
    void AddNearPoints(PointNumberList &neighbourhood, const Position3D &pos,
                       const DataBounds3D &bounding_box,
                       const LaserPoints &points, double sqrange) const;
};
#endif /* _LaserOctree_h_ */  /* Don't add after this point */
