
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
 * \brief Interface to Class Plane
 *
 */
/*!
 * \class Plane
 * \ingroup Photogrammetry
 * \brief Interface to Class Plane
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

#ifndef _Plane_h_
#define _Plane_h_

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///                            A plane
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "Position3D.h"
#include "FeatureNumber.h"
#include "Line3D.h"
#include "AttributeTypes.h"

///Types of Tags
enum PlaneTag {PT_Label, PT_Fixed, PT_FirstScanLine, PT_LastScanLine,
               PT_DirectionStdDev, PT_NumberOfPoints,
               PT_Lambda0, PT_Lambda1, PT_Lambda2,
               PT_Reference,
               PT_Undefined=255};

typedef GenericAttributeType PlaneAttributeType;

extern PlaneAttributeType AttributeType(const PlaneTag attribute);

enum PlaneParameterFixedTag {PT_NothingFixed, PT_OrientationFixed,
                             PT_DistanceFixed, PT_AllFixed};
                             
enum PlaneReferenceTag {PT_NoReference, PT_OrientationReference,
                        PT_DistanceReference, PT_FullReference};

class Plane : public FeatureNumber
{
  protected:

    /// Normal vector
  	Vector3D normal;

    /// Distance of plane to the origin
  	double distance;

    /// Number of points used for sum and moment values
    int num_pts;

    /// Sum of the coordinates of the points in the plane
    Vector3D *coord_sum;

    /// Square sums of the coordinates of the points in the plane
    double *moments;
 	
    /// Offset for moment computations
    Vector3D *offset;
    
    /// Number of attributes
    int num_attributes;

    /// Vector of attribute tags
    unsigned char *tags;

    /// Vector of attribute values
    int *attributes;

  public:
    /// Default constructor
    Plane() : FeatureNumber()
      {InitialisePointers(); Initialise();}

    /// Construct plane from point and normal vector
    Plane(const Vector3D& point, const Vector3D& normalVec);

    /// Construct plane from three positions
    Plane(const Position3D &p1, const Position3D &p2, const Position3D &p3);

	/// Copy constructor
    Plane(const Plane &pl) : FeatureNumber()
	  { InitialisePointers(); num_attributes = 0; *this = pl; } 
	  		
    /// Default destructor
  	~Plane() {Erase();}

  private:
    /// Initialise the pointers (used in constructors only)
    void InitialisePointers();
    
    /// Retrieve index of an existing attribute tag
    unsigned char ExistingAttributeIndex(const PlaneTag tag) const;

    /// Retrieve index of an attribute tag, create one if it does not yet exist
    unsigned char AttributeIndex(const PlaneTag tag);

  public:
    /// Initialisation
  	void Initialise();

    /// Copy assignament
    Plane& operator=(const Plane& pl); 
  	
    /// Return the readable reference
    const Plane &PlaneReference() const
      {return(*this);}

    /// Return the writable reference
    Plane &PlaneReference()
      {return(*this);}

    /// Print the plane to stdout
  	void Print() const;

    /// Set the normal vector of the plane as the cross product of two vectors
	void SetNormal(const Vector3D &vec1, const Vector3D &vec2);	

    /// Use the specified vector as the normal vector of this plane
	void SetNormal(const Vector3D &vec)
      {normal = vec;}
		
    /// Set the distance of the plane to the origin
    void SetDistance(double dist)
      {distance = dist;}
    
    /// Set the distance of a non-vertical plane
    /** This function sets the distance to c - XGrad() * vec.X() -
        YGrad() * vec.Y(). The usage of this function is discouraged
    */
	void SetDistance(double c, const Vector3D &vec);	

    // Not documented. Function used for interpolation in planes.
	double Interp(double, double) const;
	
    /// Gradient in X-direction. Do not use this for vertical planes!
    /** The function returns the negative X-component of the normal vector
        divided by the Z-component of the normal vector. It does NOT check
        whether the Z-component is zero, and causes then a division by zero!
    */
	double XGrad() const
	  {return (-normal.X()/normal.Z());}

    /// Gradient in Y-direction. Do not use this for vertical planes!
    /** The function returns the negative Y-component of the normal vector
        divided by the Z-component of the normal vector. It does NOT check
        whether the Z-component is zero, and causes then a division by zero!
    */
	double YGrad() const
	  {return (-normal.Y()/normal.Z());}

    /// Return the readable label
    int Label() const
      { return Attribute(PT_Label); }

    /// Return the writable label
    int &Label()
      { return Attribute(PT_Label); }

    /// Return the readable distance
    double Distance() const
      { return distance; }

    /// Return the writable distance
    double &Distance()
      { return distance; }

    /// Return the readable normal vector
    const Vector3D& Normal() const
      { return normal; }

    /// Return the writable normal vector
    Vector3D &Normal()
      { return normal; }
      
    /// Return the offset pointer
    Vector3D * Offset() {return offset;}

	/// Return an eigenvalue (0 = largest, 1 = middle, 2 = smallest)
	/** @param index Index of eigenvalue
	    @return Eigenvalue, or -1 if no eigenvalues have been computed or
	            -2 if the index is < 0 or > 2.
	*/ 
    double Eigenvalue(int index) const;
    
    /// Return the readable smallest eigenvalue
    double SmallestEigenvalue() const
      { return Eigenvalue(2); }
    
    /// Determine the distance of a point to the plane
    double Distance(const Position3D &) const;

    /// Calculate Z-coordinate at the specified XY-location
    /** @param X X-coordinate
        @param Y Y-coordinate
        @param success 0 - failure because the plane is vertical. 1 - success.
        @return Z-coordinate at the specified XY-location
    */
    double Z_At(double X, double Y, int *success) const;

    /// Project a 3D position onto the plane
    Position3D Project(const Position3D &pos) const;

    /// Determine the intersection point of a line and a plane
    /** @param line Line to be intersected with the plane
        @param plane Plane to be intersected with the line
        @param pos Calculated intersection point
        @return 0 - failure (line parallel to plane), 1 - success.
    */
    friend bool IntersectLine3DPlane(const Line3D &line, const Plane &plane,
                                     Position3D &pos);
    /// Determine the intersection line of two planes
    /** @param plane1 First plane
        @param plane2 Second plane
        @param line   Calculated intersection line
        @return 0 - failure (parallel planes), 1 - success.
    */
    friend bool Intersect2Planes(const Plane &plane1, const Plane &plane2,
                                 Line3D &line);

    /// Check if plane is horizontal
    /** A plane is classified horizontal if the angle of the normal vector
        with the positive or negative Z-axis is less than or equal to the
        specified tolerance.
        @param angle_tolerance Maximum allowed angle in radians
        @return 0 - Plane is not horizontal. 1- Plane is horizontal.
    */
    bool IsHorizontal(double angle_tolerance) const;
    
    
    /// Check if plane is vertical
    /** A plane is classified vertical if the inner product of the normal vector
        with the positive or negative Z-axis is less than or equal to the
        specified tolerance.
        @param angle_tolerance Maximum allowed angle in radians
        @return 0 - Plane is not vertical. 1- Plane is vertical.
    */
    bool IsVertical(double angle_tolerance) const;
    

    /// Add a point to the plane and (optionally) recalculate the parameters
    bool AddPoint(const Position3D &pos, bool recalculate=true);

    /// Remove a point from the plane and (optionally) recalculate 
    bool RemovePoint(const Position3D &pos, bool recalculate=true);

    /// Add or remove a point from the plane and (optionally) recalculate 
    bool ChangePoint(const Position3D &pos, bool add,
                     bool recalculate=true);

    /// Recalculate plane parameters from moments
    bool Recalculate();
    
    /// Return the number of points in a plane
    int NumberOfPoints() const
      {return num_pts;}
      
    /// Return the centre of gravity of points added to the plane
    /** If no points have been added using AddPoint, the position returned
        contains coordinates INF
    */
    Position3D CentreOfGravity() const;
    
    /// Erase plane data
    void Erase();
    
    /// Swap the normal vector direction
    void SwapNormal();
    
     /// Retrieve readable attribute value
    int Attribute(const PlaneTag tag) const;

    /// Retrieve readable attribute value
    int Attribute(const int tag) const
      { return Attribute((const PlaneTag) tag); }

    /// Retrieve writable attribute value
    int & Attribute(const PlaneTag tag);

    /// Retrieve writable attribute value
    int & Attribute(const int tag)
      { return Attribute((const PlaneTag) tag); }
      
    /// Retrieve readable float attribute value
    float FloatAttribute(const PlaneTag tag) const;

    /// Retrieve readable float attribute value
    float FloatAttribute(const int tag) const
      { return FloatAttribute((const PlaneTag) tag); }

    /// Retrieve writable float attribute value
    float & FloatAttribute(const PlaneTag tag);

    /// Retrieve writable float attribute value
    float & FloatAttribute(const int tag)
      { return FloatAttribute((const PlaneTag) tag); }

    /// Check if an attribute value is present
    bool HasAttribute(const PlaneTag tag) const;

    /// Check if an attribute value is present
    bool HasAttribute(const int tag) const
      { return HasAttribute((const PlaneTag) tag); }

    /// Set an attribute value
    void SetAttribute(const PlaneTag tag, const int value);

    /// Set an attribute value
    void SetAttribute(const int tag, const int value)
      { SetAttribute((const PlaneTag) tag, value); }

    /// Set a float attribute value
    void SetAttribute(const PlaneTag tag, const float value);

    /// Set a float attribute value
    void SetAttribute(const int tag, const float value)
      { SetAttribute((const PlaneTag) tag, value); }

    /// Remove an attribute
    bool RemoveAttribute(const PlaneTag tag);

    /// Remove all attributes
    bool RemoveAttributes();

    /// Rename an attribute
    bool RenameAttribute(const PlaneTag oldtag,
                         const PlaneTag newtag);

    /// Return the number of attributes
    unsigned char NumAttributes() const { return num_attributes; }

    // Return attribute tag vector based on index
    const unsigned char * AttributeTags() const { return tags; }

    // Return attribute value based on index
    const int * AttributeValues() const { return attributes; }
};
#endif /* _Plane_h_ */  /* Don't add after this point */
