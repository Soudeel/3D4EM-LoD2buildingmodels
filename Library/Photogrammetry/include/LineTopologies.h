
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
 * \brief Interface to Class LineTopologies - A set of stochastic three-dimensional control points
 *
 */
/*!
 * \class LineTopologies
 * \ingroup Photogrammetry
 * \brief Interface to Class LineTopologies - A set of stochastic three-dimensional control points
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

#ifndef _LineTopologies_h_
#define _LineTopologies_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineTopologies  - A set of stochastic three-dimensional control points

--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "LineTopology.h"
#include <vMatrix.h> 
#include <fstream>

class ObjectPoints2D;
class ObjectPoints;
class TIN;
class DataBounds3D;

//------------------------------------------------------------------------------
/// A vector of line topologies
//------------------------------------------------------------------------------

class LineTopologies : public std::vector<LineTopology>
{
  protected:

    /// File name of the data set
    char *name;

  public:

    /// Default constructor
    LineTopologies() : std::vector<LineTopology>()
      { name = NULL; } 

    /// Construct by reading line topologies from a file
    /** @param filename Name of file with line topologies
        @param success  Return value of the read function
                        0 - failure, 1 - success
    */
    LineTopologies(const char *filename, int *success)
      : std::vector<LineTopology>()
      { name = NULL; *success = Read(filename, false); }

    /// Construct by converting a C object
    LineTopologies(LineTops *linetops) : std::vector<LineTopology>()
      { name = NULL; C2Cpp(linetops); }    

    /// Copy constructor
    LineTopologies(const LineTopologies &tops) : std::vector<LineTopology> ()
      { name = NULL; *this = tops; }

    /// Default destructor
    ~LineTopologies()
	{ delete [] name; }

    /// Copy assignment
    LineTopologies & operator = (const LineTopologies &);

    /// Return the writable reference
    LineTopologies & LineTopologiesReference()
      {return *this;}

    /// Return the readable reference
    const LineTopologies & LineTopologiesReference() const
      {return *this;}

    /// Conversion from C to C++ object
    void C2Cpp(LineTops *);

    /// Conversion from C++ to C object
    void Cpp2C(LineTops **) const;

    /// Read line topologies from a file
    /** @param filename Name of the file with line topologies
        @return 0 - failure, 1 - success
    */
    int Read(const char *filename, bool printlog=true);
    
    
    /// Read line topologies from a file with older topology version
    /** @param filename Name of the file with line topologies
        @return 0 - failure, 1 - success
    */
    int ReadV0(const char *filename);
    
    /// Read line topologies from a file with new topology version
    /** @param filename Name of the file with line topologies
        @return 0 - failure, 1 - success
    */
    int ReadV1(const char *filename, bool printlog=true);
    
    /// Write line topologies to a file
    /** @param filename Name of the file for the line topologies
        @return 0 - failure, 1 - success
    */
    int Write(const char *filename, bool printlog=true) const;
    
    /// Write line topologies to a file with new line topology version
    /** @param filename Name of the file for the line topologies
        @return 0 - failure, 1 - success
    */
    int WriteV1(const char *filename, bool printlog=true) const;

    /// Print the line topologies to stdout
    void Print() const;

    /// Set an attribute value for all line topologies
    void SetAttribute(const LineTopologyTag tag, const int value);

   /// Select attributed lines from line topologies
    LineTopologies SelectAttributedLines(const LineTopologyTag tag, const int value);
    
   std::vector<int> AttributedValues(const LineTopologyTag tag);

    /// Return the name of the data set
    char *Name() const
      {return name;}

    /// Write the line topologies to a VRML file
    /** A list of object coordinates should have been written to the VRML
        file before invoking this function. see e.g. ObjectPoints::Write_VRML
        @param fd           File descriptor of an open VRML file
        @param show_as_face If 1 closed polygons are written as an
                            IndexedFaceSet, otherwise as IndexedLineSet
    */
    void VRML_Write(FILE *fd, int show_as_face) const;

    /// Determine the index of the line with the specified line number
    /** @param number Line number for which the index should be retrieved
        @return Index of sought line, or -1 if the line was not found
    */
    int FindLine(const LineNumber &number) const;
    
    /// Determine the index of the line with the specified tag value
    /**  @param tag LinetopologyTag for which the index should be retrieved
        @param value Tag value for which the index should be retrieved
        @return Index of sought line, or -1 if the line was not found
    */
    int FindLineByTagValue(const LineTopologyTag tag, const int value) const;

    /// Merge polygons with the same label
    /** Closed polygons that have a common edge and the same label are merged
        to single larger polygons. The original polygons are deleted.
    */
    void MergePolygons();

    /// Renumber polygons and 3D object points
    /** Object points are renumbered starting at a specified point number.
        The node numbers of the polygons are updated correspondingly.
        @param points 3D object points to be renumbered
        @param first_point_number New number for the first point (default 0)
        @param first_line_number New number for the first line (default 0)
    */
    void ReNumber(ObjectPoints &points, int first_point_number=0,
                  int first_line_number=0);

    /// Renumber polygons and 2D object points
    /** Object points are renumbered starting at a specified point number.
        The node numbers of the polygons are updated correspondingly.
        @param points 2D object points to be renumbered
        @param first_point_number New number for the first point (default 0)
        @param first_line_number New number for the first line (default 0)
    */
    void ReNumber(ObjectPoints2D &points, int first_point_number=0,
                  int first_line_number=0);

    /// Insert nodes for points on segments of the polygons
    /** If points are within a distance of 0.01 of the polygon edges, their
        numbers are inserted in the point number list of the polygon.
        @param points Points of the polygon corners and other points that
                      will be inserted if they are near the polygon edges.
    */
    void InsertNodes(const ObjectPoints &points);

    /// Remove collinear nodes
    /** If three consecutive points of a polygon are collinear, the middle
        point is removed from the polygon. 
    */
    void RemoveCollinearNodes(const ObjectPoints &);

    /// Construct by converting TIN meshes into triangular polygons
    /** @param tin A Delaunay triangulation
        @param first_line_number The first line number (default 0)
        @param label The label of the lines (default 0)
    */
    LineTopologies(const TIN &tin, int first_line_number=0, int label=0);

    /// Add walls between roof and floor points
    /** This function specific to building reconstruction. It is assumed that
        the roof outlines will be used later to construct the floor outline of
        a building (see DuplicateWithOffset). This function constructs the
        topologies of vertical faces in between the edges of the roof outlines
        and floor outlines. The point numbers of the not yet existing floor
        points are defined by adding the specified offset to the roof point
        numbers.
        @param roof_outlines The outlines of one or more roofs or roof faces
        @param number_offset Offset to construct point numbers of the floor
                             outline. This number should also be used in
                             the call to DuplicateWithOffset
    */
    void AddWalls(const LineTopologies &roof_outlines, int number_offset);

    /// Add TIN walls between roof and floor points
    /** Similar as previous, but now the rectangle will be divided in 2 triangles
        in this funtion.
    */
    void AddTINWalls(const LineTopologies &roof_outlines, int number_offset);

    /// Duplicate polygons with a fixed point number offset
    /** Add a copy of all polygons to the current set of polygons and increase
        the point numbers of the copied polygons by a specified number offset.
        This function is used for building reconstruction. The original points
        are points of a roof outline. After copying these points (by
        ObjectPoints::DuplicateWithFixedZ) and changing the height of the
        duplicated points, these points can be used to construct the floor
        of a building. Also see AddWalls.
    */

    void DuplicateWithOffset(int number_offset);

    /// Densify polygons by inserting new nodes on long edges
    /** New points are created and inserted into the polygon edges such that
        the edges are split in half until the maximum distance between two
        consecutive polygon points is below a threshold.
        @param points Points of the polygons. New points are added to this
                      point vector.
        @param max_dist Maximum distance between two consecutive polygon
                        points.
    */
    void Densify(ObjectPoints &points, double max_dist);

    /// Determine the bounds of a set of polygons
    /** @param points Points of the polygons.
        @return Data bounds of the polygons.
    */
    DataBounds3D &Bounds(const ObjectPoints &points) const;

    /// Test membership of a point number
    /** @return 1 if the point number is found in one of the polygons,
                0 otherwise.
    */
    int Contains(const PointNumber &) const;

    /// Test membership of a polygon
    bool Contains(const LineTopology &top) const;

    /// Test if the points of all polygons are present in the point number list
    /** @return 1 if all point numbers are found in the list, 0 otherwise.
    */
    int IsPartOf(const PointNumberList &list) const;

    /// Test if some points of the polygons are present in a point number list
    /** @return 1 if some point numbers are found in the list, 0 otherwise.
    */
    int OverlapsWith(const PointNumberList &list) const;

    /// Delete all lines
    void Erase();

    /// Make polygons counter clock wise when seen from the top
    void MakeCounterClockWise(const ObjectPoints &points);

    /// Make polygons clock wise when seen from the top
	void MakeClockWise(const ObjectPoints &points);
    
    /// Sort the lines after the line numbers
    void Sort();

  /// Duplicate polygons with a fixed point number offset and construct floor
    /** Add a copy of all polygons to the current set of polygons and increase
        the point numbers of the copied polygons by a specified number offset.
        This function is used for building reconstruction. The original points
        are points of a roof outline. After copying these points (by
        ObjectPoints::DuplicateWithFixedZ or 
        ObjectPoints::DuplicateWithFixedOffset) and changing the height of the
        duplicated points, these points can be used to construct the floor
        of a building. 
    */
    void AddFloor(const LineTopologies &roof_outlines, int number_offset);
  
    /// Transfrom topologies to adjacency matrix, represented as a n x n vector
    std::vector<int> TransformToGraph(ObjectPoints points, bool label = true, bool number=false);
    
    /// Transfrom topologies to adjacency matrix, represented as FRS matrix
    FRSmatrix<int> TransformToGraphMatrix(ObjectPoints points, bool label = true, bool number=false);

    ///Return pointnumbers of points that only have one reference in the linetopologies
    ObjectPoints ReturnEndPointsOfTopologies(ObjectPoints points);
    
    LineTopologies ReturnLinesWithPoint(const PointNumber &number) const;
    
    LineTopologies ReturnLinesWithPoint1ButWithoutPoint2(const PointNumber &number, const PointNumber &number2) const;
    
    int CostBetweenTwoNodes(ObjectPoints points, const PointNumber &number1, const PointNumber &number2);
    
    LineTopology ReturnClosedPolygon(ObjectPoints points);
};
#endif /* _LineTopologies_h_ */   /* Do NOT add anything after this line */
