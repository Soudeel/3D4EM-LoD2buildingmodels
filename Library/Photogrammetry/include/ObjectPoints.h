
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
 * \brief Interface to Class ObjectPoints - A set of stochastic three-dimensional object points
 *
 */
/*!
 * \class ObjectPoints
 * \ingroup Photogrammetry
 * \brief Interface to Class ObjectPoints - A set of stochastic three-dimensional object points
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

#ifndef _ObjectPoints_h_
#define _ObjectPoints_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ObjectPoints  - A set of stochastic three-dimensional object points

--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "ObjectPoint.h"
#include "Database4Cpp.h"
#include "LineTopologies.h"
#include "DataBounds3D.h"
#include "Positions3D.h"

class ObjectPoints2D;
class CameraPoints;
class ModelPoints;
class InteriorOrientation;
class ExteriorOrientation;
class AbsoluteOrientation;
class TIN;

//------------------------------------------------------------------------------
/// A vector of object points
//------------------------------------------------------------------------------

class ObjectPoints : public VectorPoint<ObjectPoint>
{
  public:

    /// Default constructor
    ObjectPoints() : VectorPoint<ObjectPoint>() {}

    /// Construct by reading object points from a file
    ObjectPoints(const char *filename) : VectorPoint<ObjectPoint>()
    	{ Read(filename); }	     	

    /// Construct by converting a C object
    ObjectPoints(ObjPts *objpts) : VectorPoint<ObjectPoint>()
        { C2Cpp(objpts); }
    
    /// Construct from a vector of 2D object points
    /** @param points2D Vector of 2D object points
        @param z        Constant height for all constructed object points
    */
    ObjectPoints(const ObjectPoints2D *points2D, double z);
    
    /// Construct by forward intersection of camera points
    /** @param campts_left  Camera points of the left image
        @param campts_right Camera points of the right image
        @param intor_left   Interior orientation of the left image
        @param intor_right  Interior orientation of the right image
        @param extor_left   Exterior orientation of the left image
        @param extor_right  Exterior orientation of the right image
    */
    ObjectPoints(const CameraPoints *campts_left,
                 const CameraPoints *campts_right,
    		 const InteriorOrientation *intor_left,
                 const InteriorOrientation *intor_right, 
    	 	 const ExteriorOrientation *extor_left,
                 const ExteriorOrientation *extor_right);

    /// Construct by transforming model points to the object system
    /** The transformation between the model coordinate system and the object
        coordinate system is the 3D similarity transformation as defined by the
        absolute orientation parameters.
        @param modelpts Model points
        @param absor    Absolute orietation parameters
    */
    ObjectPoints(const ModelPoints *modelpts, const AbsoluteOrientation *absor);

    /// Construct from 3D positions and a point number
    /** @param positions 3D positions to be converted to object points
        @param number Point number of the first object point to be created
    */
    ObjectPoints(const Positions3D &positions, const PointNumber &number);
    
    /// Destructor
    ~ObjectPoints() {};

    /// Copy assignment
    ObjectPoints & operator = (const ObjectPoints & pts)
      { if (!empty()) erase(begin(), end());
        insert(begin(), pts.begin(), pts.end()); 
        return *this;
      }
  
    /// Return the reference
    ObjectPoints &ObjectPointsRef()
      { return *this; }

    /// Return the const reference
    const ObjectPoints &ObjectPointsRef() const
      { return *this; }

    /// Conversion from C to C++ object
    void C2Cpp(ObjPts *);

    /// Conversion from C++ to C object
    void Cpp2C(ObjPts **) const;

    /// Read object points from a file
    /** @param filename Name of a file with object points
        @return 0 - failure, 1 - success
    */
    int ReadC(const char *filename);
    int Read(const char *filename);

    /// Write object points to a file
    /** @param filename Name of the file for the object points
        @return 0 - failure, 1 - success
    */
    int WriteC(const char *filename) const;
    int Write(const char *filename);

    /// Write object points without covariance to a file
    /** @param filename Name of the file for the object points
        @return 0 - failure, 1 - success
    */
    int Write2(char *filename) const;
 
    /// Print the object points to stdout
    void Print() const;

    /// Read object points and topology from a DXF file
    /** Read an DXF file and extract 3D object points and their topology. If
        the object points are two-dimensional, the Z-coordinate is set to 0.
        @param filename      Name of the DXF file
        @param polygons      Topologies of read polygons
        @param lab_loc       Optionally, the colour value or layer number of a
                             DXF object is stored as the line label. Possible
                             values are 1 - Store no label, 2 - Store the colour
                             value as label, 3 - Store the layer number as
                             label.
        @param sel_on_colour If true, only lines with the specified colour
                             (next argument) are selected.
        @param colour        Only select lines of this colour. This parameter is
                             ignored if sel_on_colour is not set.
        @param sel_on_layer  If true, only lines with the specified layer
                             (next argument) are selected.
        @param layer         Only select lines of this layer. This parameter is
                             ignored if sel_on_layer is not set.
        @param loose_points  If true, points not connected to lines are also
                             stored
        @param ignore_zeros  Ignore points (0.0, 0.0, 0.0)
        @param force_polygon_closure Close all polylines
    */
    int ReadDXF(char *filename, LineTopologies &polygons, int lab_loc,
                int sel_on_colour, int colour, int sel_on_layer, int layer,
                bool loose_points=false, bool ignore_zeros=false,
				bool force_polygon_closure=false);

    /// Write polygons to a DXF file
    void WriteDXF(FILE *dxffile, const LineTopologies &polygons, bool use_label_as_layer) const;
    
    /// Write polygons to a DXF file
    void WriteDXF(FILE *dxffile, const LineTopologies &polygons) const
    {
         WriteDXF(dxffile, polygons, false);
    }
    
    /// Write the points to a VRML file
    void VRML_Write(FILE *fd) const;

    /// Write the points as crosses to a VRML file
    /** @param fd   File descriptor of an open VRML file
        @param size Size of the edges of the cross
    */
    void VRML_Write_Crosses(FILE *fd, double size) const;

    /// Write lines as cylinders to a VRML file
    /** @param fd     File descriptor of an open VRML file
        @param lines  Topologies of the lines
        @param radius Radius of the cylinders
    */
    void VRML_Write_Cylinders(FILE *fd, const LineTopologies &lines, 
                              double radius) const;

    /// Remove points with nearly the same coordinates
    /** If two or more points are within the specified distance, only the first
        point is kept. The topology of lines referring to the second or further
        points is adapted.
        @param lines Line topologies to be updated after point removal.
        @param dist  Range within which points are merged.
    */
    void RemoveDoublePoints(LineTopologies &lines, double dist, bool b2D=false);
	
	/// Remove unreferenced points. The point will be sorted according to the numbers
	void RemoveUnusedPnts(LineTopologies &lines);

   /// Derive a TIN for the object points
   TIN & Triangulate() const
     { return(Triangulate(LineTopologies(), ObjectPoints())); }

   /// Derive a TIN for object points, constrained by enclosing polygons
   /** @param bounds Topology of the bounding polygon
       @return The constrained Delaunay triangulation
   */
   TIN & Triangulate(const LineTopologies &bounds) const
     { return(Triangulate(bounds, ObjectPoints())); }

   /// Derive a TIN for object points, constrained by enclosing polygons and polygons of holes
   /** @param bounds Topology of the bounding polygons and the hole polygons
       @param holes Vector of object points with one point within each hole
       @return The constrained Delaunay triangulation
   */
   TIN & Triangulate(const LineTopologies &bounds,
                     const ObjectPoints &holes) const;

   /// Duplicate points with another Z-coordinate and point number
   /** All points are copied. The height of the new points is set to the
       specified value. The point numbers of the new points are derived by
       added the number offset to the point numbers of the original points.
   */
   void DuplicateWithFixedZ(double z, int number_offset);

   /// Duplicate points with an offset to Z-coordinate and point number
   /** All points are copied. The height of the new points are derived by adding
       the z-offset to the height of the copied points.
       The point numbers of the new points are derived by
       added the number offset to the point numbers of the original points.
   */
   void DuplicateWithFixedOffset(double z_offset, int number_offset);

   /// Write object faces in VRML 2.0 format
   void VRML2_Write(FILE *fd, const LineTopologies &faces,
                    bool same_colour, double R = 0.8, double G = 0.8, 
                    double B = 0.8) const; 
   /// Sort on coordinates
   void SortOnCoordinates();
   
   /// Derive the bounds of the point set
   DataBounds3D & Bounds() const;

   
   /// Return point by point number
   ObjectPoint PointByNumber(int number);
   
   /// Return point index by point number
   int PointIndexByNumber(int number);

   
   /// Average heights of points with nearly the same coordinates 
   /// (if height difference is below certain threshold "min_height_dist").
   void AverageHeightsOfDoublePoints(LineTopologies &polygons, double min_dist, double min_height_dist);
   
   /// Average coordinates of points with nearly the same coordinates 
   /// (if height difference is below certain threshold "min_height_dist" & planimetric distance is below min_dist).
   void AverageDoublePoints(LineTopologies &polygons, double min_dist, double min_height_dist);


   /// Write polygons to a simple DXF file, with filled polygons
   void WriteDXFMesh(FILE *dxffile, const LineTopologies &polygons,
		     int color = 7, bool line = true, bool header = true,
		     bool footer = true, bool use_label_as_layer = false) const;
		     
   
   int MaxPointNumber();
   
   /// check if an object point is visible
    /* 
        @param objpt      the point to check
        @param extor      exterior orientation
        @param top        a line topology for a line including the point
        
    */
    bool PointVisible(ObjectPoint *objpt,ExteriorOrientation* extor,LineTopology* top, double max_dist);

    // check if a mid point of two object point is visible
    /* 
    @param objpt1      the point one
    @param objpt2      the point two 
    @param extor      exterior orientation
    @param top        a line topology for a line including the point
    
    */
    bool MiddlePointVisible(ObjectPoint *objpt1,
                                   ObjectPoint *objpt2,
                                   ExteriorOrientation* extor,
                                   LineTopology* top, double max_dist);
};

#endif /* _ObjectPoints_h_ */   /* Do NOT add anything after this line */
