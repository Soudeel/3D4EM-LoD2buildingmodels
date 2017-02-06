
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
 * \brief Interface to Class LineTopology - Topology of lines        
 *
 */
/*!
 * \class LineTopology
 * \ingroup Photogrammetry
 * \brief Interface to Class LineTopology - Topology of lines        
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg
 * \date		24-11-1998 (Created)
 * \date		10-03-1999 (Update #1 - \G_Vosselman)
 *
 * \remark \li Update #1 - Added a label to the topology class
 *
 * \remark \li Topology of a line
 * The line topology is specified as a vector of point numbers. These numbers
 *   can refer to any kind of point (i.e. image points, object points, etc.)
 *   Each line has a line number and a line label.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _LineTopology_h_
#define _LineTopology_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineTopology                    - Topology of lines        

  Initial creation
  Author  : Ildiko Suveg
  Date    : 24-11-1998

  Update #1
  Author  : George Vosselman
  Date    : 10-03-1999
  Changes : Added a label to the topology class
  
  Update #2
  Author : Shi Pu
  Date   : 11-04-2006
  Change:  Added Tags, and relevant changes to several functions
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include "PointNumberList.h"
#include "LineNumber.h"
#include "Database4Cpp.h"
#include "Position2D.h"

class LineTopologies;
class ObjectPoints;
class ObjectPoints2D;
class DataBounds2D;
class DataBounds3D;
class Image;
class ImageGrid;
class Line2D;
class LineSegment2D;
class LineSegments2D;
class TINMesh;
class Vector3D;

// Definition of TOP10 classes. Should be exported to separate file when extended.
enum TOP10Class {TOP10_Unknown, TOP10_Road, TOP10_Meadow, TOP10_Water,
                 TOP10_Building};
                 
///Types of Tags
enum LineTopologyTag {LineLabelTag, ClassTag, GeometryTag, TextureTag,
                      HoleMasterTag, HoleTag,
                      BuildingNumberTag, BuildingPartNumberTag,
					  GradientTag,
                      StripNumberTag,
                      TargetNumberTag, TargetType, NodeNumberTag, MatchResultTag,
                      HypoNumberTag, CorrespondingTargetLineNumber, LineNumberTag, 
                      SegmentLabel, CoverageTag, SecondSegmentLabel, PredictedHeight,
                      IDNumberTag, ToBeRemovedTag,
                      UndefinedLineTag=255};

typedef enum LineTopologyTag LineTopologyTag;

/// Types of classes
enum ClassTagValue {RoofClass, MapClass, MapPartitionClass, WallClass,
                    WindowClass, DoorClass, ExtrusionClass, StreetClass, 
                    UnknownClass=255};

/// Types of geometries
enum GeometryTagValue {LineStringGeometry, PolygonGeometry, BSplineGeometry,
                       UnknownGeometry=255};

/// Types of matchresults
enum TargetTypeValue {ShapeTarget, ConnectionTarget, DetailTarget, UnknownTargetType=255};

/// Types of matchresults
enum MatchResultTagValue {FullMatch, NotMatched, PartlyMatched, PoorlyMatched, UnknownMatch=255};
         
//------------------------------------------------------------------------------
// Topology of a line
/* The line topology is specified as a vector of point numbers. These numbers
    can refer to any kind of point (i.e. image points, object points, etc.)
    Each line has a line number and a line label.
*/
//------------------------------------------------------------------------------

class LineTopology : public PointNumberList, public LineNumber
{
  protected:

    /// Number of attributes
    int num_attributes;

    /// Vector of attribute tags
    unsigned char *tags;

    /// Vector of attribute values
    int *attributes;

private:
    /// Retrieve index of an existing attribute tag
    unsigned char ExistingAttributeIndex(const LineTopologyTag tag) const;

    /// Retrieve index of an attribute tag, create one if it does not yet exist
    unsigned char AttributeIndex(const LineTopologyTag tag);

  public:

    /// Default constructor
    
    
    LineTopology(int n=0, int l=0 ) : PointNumberList(), LineNumber()
    {
      num = n;   
      Initialise();
      SetAttribute(LineLabelTag, l);
    }

    /// Construct by converting a C object
    LineTopology(LineTop *linept) : PointNumberList(), LineNumber()
      { Initialise(); 
        C2Cpp(linept);}

    /// Construct from two point numbers
    /** @param n  Line number
        @param l  Line label
        @param p1 First point number
        @param p2 Second point number
    */
    LineTopology(int n, int l, const PointNumber &p1, const PointNumber &p2)
       : PointNumberList(), LineNumber()
      { num = n;  
        push_back(p1); push_back(p2); 
        Initialise();
        SetAttribute(LineLabelTag, l);
      }

    /// Construct a line segment from two point numbers
    /** @param n  Line number
        @param p1 First point number
        @param p2 Second point number
    */
    LineTopology(int n, int p1, int p2)
       : PointNumberList(), LineNumber()
      { num = n;  
        push_back(PointNumber(p1)); push_back(PointNumber(p2));
        Initialise();
      }

    /// Construct a closed polygon from four point numbers
    /** @param n  Line number
        @param p1 First point number
        @param p2 Second point number
        @param p3 Third point number
        @param p4 Fourth point number
    */
    LineTopology(int n, int p1, int p2, int p3, int p4)
       : PointNumberList(), LineNumber()
      { num = n;  
        push_back(PointNumber(p1)); push_back(PointNumber(p2));
        push_back(PointNumber(p3)); push_back(PointNumber(p4));
        push_back(PointNumber(p1));
        Initialise();
      }

    /// Copy constructor
    LineTopology(const LineTopology &line) : PointNumberList(), LineNumber()
       { Initialise();
         *this = line;
         } 
         
    // Initialisation of line topology
    void Initialise()
    {
      num_attributes = 0;
      tags = NULL;
      attributes = NULL;  
    }

    /// Default destructor
   ~LineTopology() { /*if (num_attributes > 0) { 
                   printf("destruct1\n");
                     free(tags);
                      printf("destruct2\n");
                    free(attributes); 
                    printf("destruct3\n");}*/
                    };


    /// Copy assignment
    LineTopology & operator = (const LineTopology &);

    /// Return the readable reference
    const LineTopology & LineTopologyReference() const
      {return (*this);}

    /// Return the writable reference
    LineTopology & LineTopologyReference()
      {return (*this);}

     /// Retrieve readable attribute value
    int Attribute(const LineTopologyTag tag) const;

    /// Retrieve readable attribute value
    int Attribute(const int tag) const
      { return Attribute((const LineTopologyTag) tag); }

    /// Retrieve writable attribute value
    int & Attribute(const LineTopologyTag tag);

    /// Retrieve writable attribute value
    int & Attribute(const int tag)
      { return Attribute((const LineTopologyTag) tag); }
      
      /// Check if an attribute value is present
    bool HasAttribute(const LineTopologyTag tag) const;

    /// Check if an attribute value is present
    bool HasAttribute(const int tag) const
      { return HasAttribute((const LineTopologyTag) tag); }

    /// Return the readable line label
    int Label() const
      { return Attribute(LineLabelTag); }

    /// Return the writable line label
    int &Label()
      { return Attribute(LineLabelTag); }
    
    /// Set an attribute value
    void SetAttribute(const LineTopologyTag tag, const int value);

    /// Set an attribute value
    void SetAttribute(const int tag, const int value)
      { SetAttribute((const LineTopologyTag) tag, value); }

    /// Set a float attribute value
    void SetAttribute(const LineTopologyTag tag, const float value);

    /// Set a float attribute value
    void SetAttribute(const int tag, const float value)
      { SetAttribute((const LineTopologyTag) tag, value); }

    /// Remove an attribute
    bool RemoveAttribute(const LineTopologyTag tag);

    /// Remove all attributes
    bool RemoveAttributes();

    /// Rename an attribute
    bool RenameAttribute(const LineTopologyTag oldtag,
                         const LineTopologyTag newtag);

    /// Return the number of attributes
    unsigned char NumAttributes() const { return num_attributes; }

    // Return attribute tag vector based on index
    const unsigned char * AttributeTags() const { return tags; }

    // Return attribute value based on index
    const int * AttributeValues() const { return attributes; }

    /// Conversion from C to C++ object
    void C2Cpp(LineTop *);

    /// Conversion from C++ to C object
    void Cpp2C(LineTop **) const;

    /// Delete a point number from the topology list
    void DeletePoint(const PointNumber *number);

    /// Check adjacency of two polygons
    /** Two polygons are considered adjacent when they share a minimum
        number of nodes (i.e. point numbers).
        @param polygon The possibly adjacent polygon
        @param min_num_nodes Minimum number of common nodes in order to declare
                             the polygons as adjacent.
        @return 1 if the two polygons are adjacent, 0 otherwise.
    */
    bool Adjacent(const LineTopology &polygon, int min_num_nodes) const;

    /// Check if two polygons have an edge in common
    /** Two polygons share an edge if the have two successive common node
        numbers.
        @param polygon The other polygon.
        @return 1 if the two polygons share an edge, 0 otherwise.
    */
    bool CommonEdge(const LineTopology &polygon) const;

    /// Return the polygon part that is shared with another polygon
    /** If two polygons share two or more successive nodes, these nodes are
        returned as a polygon. Otherwise a polygon with no nodes is returned.
    */
    LineTopology & CommonPart(const LineTopology &polygon) const;
    
/// Check the orientation of two polygons with a common edge
    /** @param polygon The other polygon.
        @return 1 if the two polygons have the same orientation, 0 otherwise.
    */
    
    int SameOrientation(const LineTopology &polygon) const;
    
     /// Return all polygon parts that are shared with another polygon
    /** If two polygons share two or more successive nodes, these nodes are
        returned as a polygon. Otherwise a polygon with no nodes is returned.
    */
    LineTopologies AllCommonParts(const LineTopology &polygon) const;
    
   
    /// Return the pointer to the next node of a polygon
    /** The polygon is assumed to be closed. If the current node is the last
        polygon node, the next node is obtained from the polygon start.
    */
    const PointNumber * NextNode(const PointNumber *current_node) const;

    /// Return the pointer to the next node of a polygon
    /** The polygon is assumed to be closed. If the current node is the last
        polygon node, the next node is obtained from the polygon start.
    */
    PointNumber * NextNode(PointNumber *current_node);

    /// Return the pointer to the previous node of a polygon
    /** The polygon is assumed to be closed. If the current node is the first
        polygon node, the previous node is obtained from the polygon end.
    */
    const PointNumber * PreviousNode(const PointNumber *current_node) const;

    /// Return the pointer to the previous node of a polygon
    /** The polygon is assumed to be closed. If the current node is the first
        polygon node, the previous node is obtained from the polygon end.
    */
    PointNumber * PreviousNode(PointNumber *current_node);

    /// Return the pointer to the node with a specific number
    PointNumber * NodePointer(int node_index);

    /// Return the const pointer to the node with a specific number
    const PointNumber * NodePointer(int node_index) const;

    /// Return the iterator to the next node of a polygon
    /** The polygon is assumed to be closed. If the current node is the last
        polygon node, the next node is obtained from the polygon start.
    */
    PointNumberList::const_iterator
      NextNode(PointNumberList::const_iterator current_node) const;

    /// Return the iterator to the next node of a polygon
    /** The polygon is assumed to be closed. If the current node is the last
        polygon node, the next node is obtained from the polygon start.
    */
    PointNumberList::iterator NextNode(PointNumberList::iterator current_node);

    /// Return the iterator to the previous node of a polygon
    /** The polygon is assumed to be closed. If the current node is the first
        polygon node, the previous node is obtained from the polygon end.
    */
    PointNumberList::const_iterator
      PreviousNode(PointNumberList::const_iterator current_node) const;

    /// Return the iterator to the previous node of a polygon
    /** The polygon is assumed to be closed. If the current node is the first
        polygon node, the previous node is obtained from the polygon end.
    */
    PointNumberList::iterator
      PreviousNode(PointNumberList::iterator current_node);

    /// Return the const_iterator to the node with a specific number
    PointNumberList::const_iterator NodeIterator(int node_index) const;

    /// Return the iterator to the node with a specific number
    PointNumberList::iterator NodeIterator(int node_index);

    /// Merge two polygons
    /** If two polygons share an edge a new polygon is constructed from the
        outer contour of the two adjacent polygons.
        @param second_polygon The other polygon
        @param merged_polygon The merged polygon
        @return 1 if the two polygons could be merged, 0 otherwise.
    */
    bool Merge(const LineTopology &second_polygon,
               LineTopology &merged_polygon) const;

    /// Split a polygon at a specified node number
    /** A close or open polygon is split into two parts. Both parts include the
        node with the specified node number.
        @param number Node number at which the polygon is to be split
        @param new_polygons The two created polygon parts
        @return 1 if the split was successful, 0 otherwise
    */
    bool Split(const PointNumber &number, LineTopologies &new_polygons) const;

    /// Split a closed polygon in two closed polygons at specified nodes
    bool Split(const PointNumber &number1, const PointNumber &number2,
               LineTopologies &new_polygons) const;

    /// Insert nodes for points on edges of the polygon
    /** If points are within a distance of the polygon edges, their
        numbers are inserted in the point number list of the polygon.
        @param points Points of the polygon corners and other points that
                      will be inserted if they are near the polygon edges.
    */
    void InsertNodes(const ObjectPoints &points, double distance=0.01);

    /// Insert nodes of a polygon located on the edges of this polygon
    /** If points of polygon pol are within a distance of the edges of
        this polygon, the node numbers are inserted in the point number
        list of this polygon.
        @param points List of points containing coordinates of all nodes
                      of both polygons.
        @param pol    Polygon with nodes that may be inserted into this
        @param dist   Distance threshold to decide whether a point is one
                      a polygon edge.
   */
   void InsertNodes(const ObjectPoints &points, const LineTopology &pol,
                    double distance=0.01);

   /// Remove collinear nodes
    /** If three consecutive points of a polygon are collinear, the middle
        point is removed from the polygon.
    */
    void RemoveCollinearNodes(const ObjectPoints &points, bool on3D=true, double err=0.005);

    /// Revert the node order
    void RevertNodeOrder();

    /// Test if polygon is closed
    int IsClosed() const
      {if (size() < 2) return(0);
       return(begin()->Number() == (end()-1)->Number());}

    /// Determine the bounds of a polygon of 2D points
    DataBounds2D & Bounds(const ObjectPoints2D &points) const;

    /// Determine the bounds of a polygon of 3D points
    DataBounds3D & Bounds(const ObjectPoints &points) const;

    /// Add a closed polygon to a bitmap image
    /** The grey values of pixels inside the closed polygon are set to 1.
        @param points Corner points of the polygon
        @param image  The bitmap image
        @param grid   The image grid specification for the conversion of
                      object coordinates to pixel coordinates
    */
    void AddToBitmap(const ObjectPoints &points, Image &image,
                     const ImageGrid &grid) const;

    /// Intersect a closed polygon (this) by a line.
    /** A closed polygon defined by the line topology (this) and a set of object
        points is intersected in the XY-plane by a line. The intersection
        results into no, one or multiple line segments. Within this function,
        the edges of the polygon are converted to a set of line segments. These
        segments are kept in a static variable. In case of multiple calls to
        this one can opt to reuse these line segments to speed up computations.
        @param points             The points of the polygon
        @param line               The line to intersect with the polygon
        @param intersections      The computed intersecting line segments. This
                                  vector is initialised if the last parameter
                                  is 0.
        @param err_dist           The distance tolerance to decide about
                                  intersections.
        @param min_length         The minimal length of intersecting line
                                  segments to be returned.
        @param reuse_pol_segments Flag to reuse the polygon line segments of
                                  a previous call. See explanation above.
        @return The number of intersecting line segments
    */
    int IntersectPolygonByLine(const ObjectPoints &points, const Line2D &line,
                               LineSegments2D &intersections,
                               double err_dist, double min_length,
                               int reuse_pol_segments=0) const;

    /// Split a polygon by an intersecting line
    /**
    */
    bool SplitPolygonByLine(ObjectPoints &points, const Line2D &line,
                            double snap_dist, double min_length,
                            LineTopologies &new_polygons) const;

    /// Split a polygon by an intersecting line
    /**
    */
    bool SplitPolygonByLineSegment(ObjectPoints &points,
                                   const LineSegment2D &segment,
                                   double snap_dist,
                                   LineTopologies &new_polygons) const;

/// Split a polygon by the nearest intersecting line
    /**
    */
    bool SplitPolygonByLineSegment_nearest(ObjectPoints &points,
                                   const LineSegment2D &segment,
                                   double snap_dist, double min_length,
                                   LineTopologies &new_polygons) const;

    /// Construct by converting a TIN mesh into a triangular polygon
    /** @param mesh TIN mesh which includes the three node numbers
        @param line_number Optional line number
        @param line_label Optional line label
    */
    LineTopology(const TINMesh &mesh, int line_number=0, int line_label=0);

    /// Calculate a surface normal
    /** Surface normal calculation with two methods.
        Method 0: Surface normal based on the first three points of the polygon
                  (fast but inaccurate if points are nearby)
        Method 1: Surface normal based on most distant points.
                  (slow for large polygons, but accurate)
        @param points Points of the polygon
        @param method Method for normal calculation, see above.
        @return Surface normal vector
    */
    Vector3D Normal(const ObjectPoints &points, int method=0) const;

    /// Densify polygon by inserting new nodes on long edges
    /** New points are created and inserted into the polygon edges such that
        the edges are split in half until the maximum distance between two
        consecutive polygon points is below a threshold.
        @param points Points of the polygon. New points are added to this
                      point vector.
        @param max_dist Maximum distance between two consecutive polygon
                        points.
    */
    void Densify(ObjectPoints &points, double max_dist);

    /// Select another polygon node as start node of a closed polygon
    bool NewStartNode(const PointNumber &new_start_node);

    /// Check if a polygon order is clockwise when seen from the top
    bool IsClockWise(const ObjectPoints &points) const;

    /// Make a polygon counter clockwise when seen from the top
    void MakeCounterClockWise(const ObjectPoints &points);
    /// Make a polygon clockwise when seen from the top
    void MakeClockWise(const ObjectPoints &points);
    /// Print the line
    void Print() const;

    /// Return the major TOP10 class code
    int TOP10MajorClass() const;
    
    /// Check if a TOP10 polygon is invisible
    bool TOP10Invisible() const;
    
    /// Check if two polygons are the same
    friend bool operator == (const LineTopology &pol1,
                             const LineTopology &pol2);
   
    int Smooth_Outline_3D(ObjectPoints &objpts, double angle_thres=15);
    
    /// Check if a closed polygon overlaps with a 2D rectangle
    /** @param points Points of the closed polygon
        @param rectangle Bounds of the rectangle
        @param bounds_check_done Should be true if it has already been checked
                                 that the bounding box of the polygon overlaps
                                 with the rectangle
        @return true if polygon overlaps with bounds. False in case of no
                overlap or in case the polygon is not closed
    */
    bool Overlap(const ObjectPoints &points, const DataBounds2D &rectangle,
                 bool bounds_check_done) const;
	//check if the input point is bounded by the closed polygon on xy plane
	bool BoundPoint(const ObjectPoints &polyPnts, const Vector3D& inPnt);

    double CalculateArea (const ObjectPoints &points);
    
    Position2D CalculateCentroid(const ObjectPoints &points);

 	int IsValid() const;
	void RemoveDoubleNodes();
};
#endif /* _LineTopology_h_ */   /* Do NOT add anything after this line */
