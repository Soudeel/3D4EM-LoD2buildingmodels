
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class Plane
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "Plane.h"
#include "stdmath.h"
#include <iostream>

#define pi 3.1415
/*
--------------------------------------------------------------------------------
                          Declarations of C routines
--------------------------------------------------------------------------------
*/

extern "C" void Solve_Normal_Eq(double *, double *, int);

extern "C" void rg_(int *, int *, double *, double *, double *, int *,
                    double *, double *, double *, int *);

/*
--------------------------------------------------------------------------------
		              Determine plane attribute type
--------------------------------------------------------------------------------
*/
PlaneAttributeType AttributeType(const PlaneTag attribute)
{
  switch ((int) attribute) {
  	case PT_DirectionStdDev:
  	case PT_Lambda0:
  	case PT_Lambda1:
  	case PT_Lambda2:
	  return FloatAttributeType;
  	default: return IntegerAttributeType;
  }
  return IntegerAttributeType;
}

/*
--------------------------------------------------------------------------------
                          Construction with point and normal vector
--------------------------------------------------------------------------------
*/

Plane::Plane(const Vector3D& point, const Vector3D& normalVec)
  : FeatureNumber()
{
  double len=normalVec.SqLength();
  if( len==0.0 )
  {
    normal=Vector3D( 0.0, 0.0, 1.0 );
    distance = point.Z();
  }
  normal=normalVec/sqrt(len);
  distance = normal.DotProduct( point );
  num_pts = 0;
  num_attributes = 0;
  InitialisePointers();
}


/*
--------------------------------------------------------------------------------
                     Construction from three positions
--------------------------------------------------------------------------------
*/

Plane::Plane(const Position3D &p1, const Position3D &p2, const Position3D &p3)
{
  normal = (p2 - p1).VectorProduct(p3 - p1);
  if (normal.Length() > 0.0) {
    normal = normal.Normalize();
    distance = 0.0;
    distance = Distance(p1);
  }
  num_pts = 0;
  num_attributes = 0;
  InitialisePointers();
}

/*
--------------------------------------------------------------------------------
		                Initialisation 
--------------------------------------------------------------------------------
*/
 
void Plane::InitialisePointers()
{
  coord_sum = offset = NULL;
  moments = NULL;
  tags = NULL;
  attributes = NULL;
}

void Plane::Initialise()
{
  normal = Vector3D();
  distance = 0.0;
  num_pts = 0;
  if (coord_sum != NULL) {
    delete coord_sum;
    delete offset;
    delete moments;
  }
  if (attributes != NULL) {
  	delete attributes;
  	delete tags;
  }
  num_attributes = 0;
  InitialisePointers();
}

/*
--------------------------------------------------------------------------------
		                Copy assignment
--------------------------------------------------------------------------------
*/
 
Plane & Plane::operator=(const Plane &pl)
{
  // Check for self assignment
  if (this == &pl) return *this;
  
  // Clear old data
  if (coord_sum) { delete coord_sum; coord_sum   = NULL;}
  if (offset)    { delete offset;    offset      = NULL;}
  if (moments)   { delete moments;   moments     = NULL;}
  // Copy new data
  num = pl.num;
  normal = pl.normal;
  distance = pl.distance;
  num_pts = pl.num_pts;
  if (pl.coord_sum != NULL) {
    coord_sum = new Vector3D(*(pl.coord_sum));
    offset    = new Vector3D(*(pl.offset));
    moments   = new double[9];
    memcpy((void *) moments, (const void *) pl.moments, 9 * sizeof(double));
  }
  // Copy plane attributes
  if (pl.num_attributes > 0) {
    if (pl.num_attributes > num_attributes) {
      if (num_attributes > 0) {
        free(tags);
        free(attributes);
      }
      tags = (unsigned char *) malloc(pl.num_attributes * 
                                      sizeof(unsigned char));
      attributes = (int *) malloc(pl.num_attributes * sizeof(int));
    }
    memcpy((void *) tags, (const void *) pl.tags,
           pl.num_attributes * sizeof(unsigned char));
    memcpy((void *) attributes, (const void *) pl.attributes,
           pl.num_attributes * sizeof(int));
  }
  else {
    if (num_attributes > 0) {
      free(tags);
      free(attributes);
    }
    tags       = NULL;
    attributes = NULL;
  }
  num_attributes = pl.num_attributes;   

  return *this;
}

/*
--------------------------------------------------------------------------------
		Print values of Plane parameters to screen
--------------------------------------------------------------------------------
*/
 
void Plane::Print() const
{
  printf("Normal vector has parameters: X %f, Y %f, Z %f\n", 
         normal.X(), normal.Y(), normal.Z());
  printf("Distance to the origin is %f\n", distance);	
  printf("Gradients are: X %f, Y %f\n", XGrad(), YGrad());
}

/*
--------------------------------------------------------------------------------
			  Set normal vector 
--------------------------------------------------------------------------------
*/
 
void Plane::SetNormal(const Vector3D &v1, const Vector3D &v2)
{
  normal = v1.VectorProduct(v2);
}

/*
--------------------------------------------------------------------------------
			  Set distance value
--------------------------------------------------------------------------------
*/
 
void Plane::SetDistance(double depend, const Vector3D &vec)
{
  distance = depend - (XGrad()*vec.X()) - (YGrad()*vec.Y());
}	

/*
--------------------------------------------------------------------------------
            	        Interpolate dependent value
--------------------------------------------------------------------------------
*/

double Plane::Interp(double X, double Y) const
{
  double depend = distance + XGrad()*X + YGrad()*Y;
  return depend;
}

/*
--------------------------------------------------------------------------------
                       Distance of a point to the plane
--------------------------------------------------------------------------------
*/

double Plane::Distance(const Position3D &point) const
{
  return(point.X() * normal.X() + point.Y() * normal.Y() +
         point.Z() * normal.Z() - distance);
}

/*
--------------------------------------------------------------------------------
                       Calculate Z at a given X and Y location
--------------------------------------------------------------------------------
*/

double Plane::Z_At(double X, double Y, int *success) const
{
  if (normal.Z() == 0) {
    *success = 0;
    return(0.0);
  }
  *success = 1;
  return((-X * normal.X() - Y * normal.Y() + distance) / normal.Z());
}

/*
--------------------------------------------------------------------------------
                            Project a point onto the plane
--------------------------------------------------------------------------------
*/

Position3D Plane::Project(const Position3D &point) const
{
  Position3D projection;
  double     dist_point;

  dist_point = Distance(point);
  projection.X() = point.X() - dist_point * normal.X();
  projection.Y() = point.Y() - dist_point * normal.Y();
  projection.Z() = point.Z() - dist_point * normal.Z();
  return(projection);
}

/*
--------------------------------------------------------------------------------
                            Intersect two planes
--------------------------------------------------------------------------------
*/

bool Intersect2Planes(const Plane &plane1, const Plane &plane2,
                           Line3D &line)
{
  Vector3D normal1, normal2, direction;
  double   a[9], b[3];

  normal1 = plane1.Normal();
  normal2 = plane2.Normal();

/* Check angle between planes */

  if (Angle(normal1, normal2) * 45 / atan(1.0) < 0.01) return(0);

/* Direction of the 3D line */

  direction = normal1.VectorProduct(normal2).Normalize();

/* Set up the equations system of the two planes and one additional plane
 * going through the origin and perpendicular to the line direction.
 *
 * Note that matrix A is transposed, since Solve_Normal_Eq passes A to a
 * FORTRAN routine. Solve_Normal_Eq was meant for symmetrical matrices and
 * therefore does not transpose A as required for FORTRAN matrices.
 */

  a[0] = normal1.X();        a[3] = normal1.Y();        a[6] = normal1.Z();
  a[1] = normal2.X();        a[4] = normal2.Y();        a[7] = normal2.Z();
  a[2] = direction.X();      a[5] = direction.Y();      a[8] = direction.Z();
  b[0] = plane1.Distance();  b[1] = plane2.Distance();  b[2] = 0.0;
  Solve_Normal_Eq(a, b, 3);

/* Set the line parameters */

  line = Line3D(b[0], b[1], b[2], direction.X(), direction.Y(), direction.Z());
  return(1);
}

/*
--------------------------------------------------------------------------------
                            Intersect line with plane
--------------------------------------------------------------------------------
*/

bool IntersectLine3DPlane(const Line3D &line, const Plane &plane,
                          Position3D &point)
{
  Vector3D normal, direction;
  double   denom, scalar;

/* Retrieve directions */

  normal = plane.Normal();
  direction = line.Direction();

/* Check intersection angle */

  denom = normal.DotProduct(direction);
  if (denom == 0.0) return(0);

/* Calculate intersection point */

  scalar = (plane.Distance() - line.FootPoint().DotProduct(normal)) / denom;
  point = line.Position(scalar);

  return(1);
}

/*
--------------------------------------------------------------------------------
                            Check if plane is horizontal
--------------------------------------------------------------------------------
*/

bool Plane::IsHorizontal(double err_angle) const
{
  if (Angle(Normal(), Vector3D(0, 0, 1)) <= err_angle) return(1);
  if (Angle(Normal(), Vector3D(0, 0, -1)) <= err_angle) return(1);
  return(0);
}

/*
--------------------------------------------------------------------------------
                            Check if plane is Vertical
--------------------------------------------------------------------------------
*/

bool Plane::IsVertical(double err_angle) const
{

//completely vertical make product=0;
 double product=normal.DotProduct(Vector3D(0,0,1));
 if((product>0)&&product<err_angle)
  return 1;
 else if((product<0)&&product>-err_angle)
  return 1;
  else return 0;  

}
/*
--------------------------------------------------------------------------------
                            Add or remove a point to the plane
--------------------------------------------------------------------------------
*/

bool Plane::AddPoint(const Position3D &pos, bool recalculate)
{
  return ChangePoint(pos, true, recalculate);
}

bool Plane::RemovePoint(const Position3D &pos, bool recalculate)
{
  return ChangePoint(pos, false, recalculate);
}

bool Plane::ChangePoint(const Position3D &pos, bool add, bool recalculate)
{
  Vector3D diff;

  // Check if moment vectors exist
  if (coord_sum == NULL) {
    coord_sum   = new Vector3D();
    offset      = new Vector3D();
    moments     = new double[9];
    memset((void *) moments, 0, 9 * sizeof(double));
  }

  if (num_pts == 0) *offset = pos;

  diff = pos - *offset;
  if (add) {
    num_pts++;              // Update point count
    *coord_sum += diff;     // Update coordinate sum
    for (int i=0; i<3; i++) // Update moment matrix
      for (int j=0; j<=i; j++)
        moments[i*3+j] += diff.X(i) * diff.X(j);
  }
  else {
    num_pts--;              // Update point count
    *coord_sum -= diff;     // Update coordinate sum
    for (int i=0; i<3; i++) // Update moment matrix
      for (int j=0; j<=i; j++)
        moments[i*3+j] -= diff.X(i) * diff.X(j);
  }

  if (recalculate) return Recalculate();
  return false;
}

/*
--------------------------------------------------------------------------------
                     Recalculate plane parameters from moments
--------------------------------------------------------------------------------
*/

bool Plane::Recalculate()
{
  Vector3D centre;
  double   tmp1[3], tmp2[3], eigen_real[3], eigen_imag[3], eigen_vec[3][3],
           central_moments[9];
  int      dim=3, matz=1, error, smallest, largest, middle;

  // Check number of points
  if (num_pts < 3) return false;

  // Centre of gravity of all points
  centre = *coord_sum / (double) num_pts;

  // Derive centralised moments
  for (int i=0; i<3; i++)
    for (int j=0; j<=i; j++)
      central_moments[i*3+j] = moments[i*3+j] -
                               num_pts * centre.X(i) * centre.X(j);

  // Mirror moments array to make it full
  central_moments[1] = central_moments[3];
  central_moments[2] = central_moments[6];
  central_moments[5] = central_moments[7]; 

  // Determine the eigen values and eigen vectors with Eispack
  rg_(&dim, &dim, (double *) central_moments, eigen_real, eigen_imag, &matz,
      (double *) eigen_vec, tmp1, tmp2, &error);

  // Select the smallest, middle and largest eigen value
  if (eigen_real[0] < eigen_real[1]) {
  	if (eigen_real[0] < eigen_real[2]) {
  	  smallest = 0;
  	  if (eigen_real[1] < eigen_real[2]) { middle = 1; largest = 2; }
  	  else { middle = 2; largest = 1; }
  	}
  	else { smallest = 2; middle = 0; largest = 1; }
  }
  else { // 1 < 0
    if (eigen_real[1] < eigen_real[2]) {
      smallest = 1;
      if (eigen_real[0] < eigen_real[2]) { middle = 0; largest = 2; }
      else { middle = 2; largest = 0; }
    }
    else { smallest = 2; middle = 1; largest = 0; }
  }
  SetAttribute(PT_Lambda0, (float) eigen_real[largest]);
  SetAttribute(PT_Lambda1, (float) eigen_real[middle]);
  SetAttribute(PT_Lambda2, (float) eigen_real[smallest]);

  // Take the eigen vector of the smallest eigen value as normal vector
  normal.X() = eigen_vec[smallest][0];
  normal.Y() = eigen_vec[smallest][1];
  normal.Z() = eigen_vec[smallest][2];
  normal = normal.Normalize();

  // Derive the distance to the origin (and correct for offset)
  distance = normal.DotProduct(centre + *offset);

  return true;
}

/*
--------------------------------------------------------------------------------
                     Derive the centre of gravity
--------------------------------------------------------------------------------
*/
Position3D Plane::CentreOfGravity() const
{
  Position3D centre;

  // Check if there are points
  if (num_pts == 0) return Position3D(1.0e30, 1.0e30, 1.0e30);

  // Centre of gravity of all points
  centre.vect() = (*coord_sum / (double) num_pts) + *offset;
  return centre;
}

/*
--------------------------------------------------------------------------------
                     Erase plane data
--------------------------------------------------------------------------------
*/
void Plane::Erase()
{
  if (coord_sum) {free(coord_sum); coord_sum = NULL;}
  if (offset) {free(offset); offset = NULL;}
  if (moments) {free(moments); moments = NULL;}
  num_pts = 0;
}

/*
--------------------------------------------------------------------------------
                     Swap the normal vector direction
--------------------------------------------------------------------------------
*/
void Plane::SwapNormal()
{
  normal *= -1.0;
  distance = -distance;
}

/*
--------------------------------------------------------------------------------
                     Attribute related functions
--------------------------------------------------------------------------------
*/
unsigned char Plane::ExistingAttributeIndex(const PlaneTag requested_tag) const
{
  const unsigned char *tag;
  unsigned char       index;
   
  for (index=0, tag=tags; index<num_attributes; index++, tag++)
    if (*tag == requested_tag) {return(index);}
  
  return(PT_Undefined);
}

unsigned char Plane::AttributeIndex(const PlaneTag tag)
{

  unsigned char index=ExistingAttributeIndex(tag);
 
  if (index == PT_Undefined) {
    num_attributes++;
    index = num_attributes - 1; // This will be the last attribute
    if (tags == NULL)
      tags = (unsigned char *) malloc(sizeof(unsigned char *));
    else
      tags = (unsigned char *) realloc(tags, num_attributes *
                                       sizeof(unsigned char *)); 
    tags[index] = (unsigned char) tag; // Store the tag;
    if (attributes == NULL) 
	  attributes = new int[1];
    else
      attributes = (int *) realloc(attributes, num_attributes * sizeof(int));
  }
  return(index);
}

int Plane::Attribute(const PlaneTag tag) const
{
  unsigned char index=ExistingAttributeIndex(tag);
  if (index == PT_Undefined) return(INT_MIN);
  return(attributes[index]);
}

int & Plane::Attribute(const PlaneTag tag)
{
  unsigned char index = AttributeIndex(tag);
  return attributes[index];
}

float Plane::FloatAttribute(const PlaneTag tag) const
{
  float *address;

  unsigned char index=ExistingAttributeIndex(tag);
  if (index == PT_Undefined) return(0.0);
  address = (float *) (attributes + index);
  return(*address);
}

float & Plane::FloatAttribute(const PlaneTag tag)
{
  float *address;

  unsigned char index = AttributeIndex(tag);
  address = (float *) (attributes + index);
  return (float &) *address;
}

bool Plane::HasAttribute(const PlaneTag tag) const
{
  unsigned char index=ExistingAttributeIndex(tag);
  return index != PT_Undefined;
}

void Plane::SetAttribute(const PlaneTag tag, const int value)
{
  Attribute(tag) = value;
}

void Plane::SetAttribute(const PlaneTag tag, const float value)
{
  FloatAttribute(tag) = value;
}

bool Plane::RemoveAttribute(const PlaneTag tag)
{
  unsigned char index=ExistingAttributeIndex(tag), i;
  if (index == PT_Undefined) return false;
  for (i=index; i<num_attributes-1; i++) {
    tags[i] = tags[i+1];
    attributes[i] = attributes[i+1];
  }
  num_attributes--;
  return true;
}

bool Plane::RemoveAttributes()
{
  if (num_attributes == 0) return false;
  num_attributes = 0;
  free(attributes);
  free(tags);
  return true;
}

bool Plane::RenameAttribute(const PlaneTag oldtag,
                                 const PlaneTag newtag)
{
  unsigned char index=ExistingAttributeIndex(oldtag);
  if (index == PT_Undefined) return false;
  tags[index] = newtag;
  return true;
}

// Return an eigenvalue
double Plane::Eigenvalue(int index) const
{
  PlaneTag tag;
  switch (index) {
  	case 0: tag = PT_Lambda0; break;
  	case 1: tag = PT_Lambda1; break;
  	case 2: tag = PT_Lambda2; break;
  	default: return -2.0;
  }
  if (!HasAttribute(tag)) return -1.0;
  return (double) FloatAttribute(tag);  
}


