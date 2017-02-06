
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
 Collection of functions for class Line3D

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>

#include "Line3D.h"
#include "Plane.h"
// #include "Segment3D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                          Declarations of C routine
--------------------------------------------------------------------------------
*/

extern "C" void rg_(int *, int *, double *, double *, double *, int *,
                    double *, double *, double *, int *);

/*
--------------------------------------------------------------------------------
		                Initialisation 
--------------------------------------------------------------------------------
*/
 
void Line3D::InitialisePointers()
{
  coord_sum = offset = NULL;
  moments = NULL;
}

void Line3D::Initialise()
{
  num_pts = 0;
  if (coord_sum != NULL) {
    delete coord_sum;
    delete offset;
    delete moments;
  }
  InitialisePointers();
  x[0] = x[1] = y[0] = y[1] = z[0] = z[1] = 0.0;
}

/*
--------------------------------------------------------------------------------
                      Constructor: Just initialisation
--------------------------------------------------------------------------------
*/

Line3D::Line3D()
{
  InitialisePointers();
  Initialise();
}

/*
--------------------------------------------------------------------------------
                      Constructor: A line determined by 2 points
--------------------------------------------------------------------------------
*/

Line3D::Line3D(const Position3D &pt1, const Position3D &pt2)
{
  InitialisePointers(); Initialise();
  x[0] = pt1.X();
  y[0] = pt1.Y();
  z[0] = pt1.Z();

  x[1] = pt2.X() - pt1.X();
  y[1] = pt2.Y() - pt1.Y();
  z[1] = pt2.Z() - pt1.Z();   

  Direction(Direction().Normalize()); // Normalisation of direction
}

/*
--------------------------------------------------------------------------------
            Constructor: A line determined by a point and a direction
--------------------------------------------------------------------------------
*/

Line3D::Line3D(const Position3D &pt, const Vector3D &dir)
{	
  InitialisePointers(); Initialise();

  x[0] = pt.X();
  y[0] = pt.Y();
  z[0] = pt.Z();

  x[1] = dir.X();
  y[1] = dir.Y();
  z[1] = dir.Z();   

  Direction(Direction().Normalize()); // Normalisation of direction
}

/*
--------------------------------------------------------------------------------
                                Destructor
--------------------------------------------------------------------------------
*/
Line3D::~Line3D()
{
  Erase();  // GV Debug
}


/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

Line3D& Line3D::operator=(const Line3D& lin)
{
  for(int i = 0; i < 2; i++) {
    x[i] = lin.x[i];
    y[i] = lin.y[i];
    z[i] = lin.z[i];
 }   
  num_pts = lin.num_pts;
  if (lin.coord_sum != NULL) {
    coord_sum = new Vector3D(*(lin.coord_sum));
    offset    = new Vector3D(*(lin.offset));
    moments   = new double[9];
    memcpy((void *) moments, (const void *) lin.moments, 9 * sizeof(double));
  }
  else {
    InitialisePointers();
  }
  return *this;
}


double Line3D::Y(double xx) const
{
  if (x[1] == 0)
  {
    printf("Line || Ox\n");
    return 0;
  }

  double t = (xx - x[0]) / x[1];
  return (y[0] + t * y[1]);
}     


double Line3D::Z(double xx) const
{
  if (x[1] == 0)
  {
    printf("Line || Ox\n");
    return 0;
  }
  
  double t = (xx - x[0]) / x[1];
  return (z[0] + t * z[1]);
}     


Position3D Line3D::DetPositionX(double xx) const
{
 if (x[1] == 0)
  {
    printf("Line || Ox\n");
    return Position3D((double)-1, (double)-1, (double)-1);
  }

  double t = (xx - x[0]) / x[1];
  return Position3D(xx, y[0] + t * y[1], z[0] + t * z[1]);
}   


Position3D Line3D::DetPositionY(double yy) const
{
 if (y[1] == 0)
  {
    printf("Line || Oy\n");
    return Position3D((double)-1, (double)-1, (double)-1);
  }

  double t = (yy - y[0]) / y[1];
  return Position3D(x[0] + t * x[1], yy, z[0] + t * z[1]);
}   


Position3D Line3D::DetPositionZ(double zz) const
{
 if (z[1] == 0)
  {
    printf("Line || Oz\n");
    return Position3D((double)-1, (double)-1, (double)-1);
  }

  double t = (zz - z[0]) / z[1];
  return Position3D(x[0] + t * x[1], y[0] + t * y[1], zz);
}   


double Line3D::DistanceToPoint(const Position3D &pt) const
{
   Vector3D a(x[0] - pt.X(), y[0] - pt.Y(), z[0] - pt.Z());
   Vector3D b(x[1], y[1], z[1]);
   b = b / b.Length();
   Vector3D d = a.VectorProduct(b);
   return d.Length();
}   
   

bool Line3D::PointOnLine(const Position3D &pt, double error) const
{
   return (DistanceToPoint(pt) < error) ;
}    	


bool Line3D::FindIntersection(const Line2D &lin, Position3D &pos) const
{
   Position2D pos1(x[0], y[0]);
   Position2D pos2(x[0] + x[1], y[0] + y[1]);
   Line2D lin2D(pos1, pos2);
   Position2D pos2D;
   if (!Intersection2Lines(lin2D, lin, pos2D))
      return false;
   pos = DetPositionX(pos2D.X());
   Position3D pos3 = DetPositionY(pos2D.Y());
   return true;   
}         

Line2D Line3D::ProjectOntoXOYPlane() const
{   
  Position2D pos1(x[0], y[0]);
  Position2D pos2(x[0] + x[1], y[0] + y[1]);
  return Line2D(pos1, pos2);
}

/*
--------------------------------------------------------------------------------
                                Intersection of two lines
--------------------------------------------------------------------------------
*/

bool Intersection2Lines(const Line3D &lin1, const Line3D &lin2, Position3D &pos)
{
  double t1 = (lin2.x[0] - lin1.x[0]) * lin2.y[1];
  t1 = t1 - (lin2.y[0] - lin1.y[0]) * lin2.x[1];
  double d = lin1.x[1] * lin2.y[1] - lin1.y[1] * lin2.x[1];
  if (d == 0)
    return 0;
  t1 = t1 / d;
  pos.X() = lin1.x[0] + lin1.x[1] * t1;
  pos.Y() = lin1.y[0] + lin1.y[1] * t1;
  pos.Z() = lin1.z[0] + lin1.z[1] * t1;
  return true;
}


double Angle2Lines(const Line3D &lin1, const Line3D &lin2)
{
   Vector3D a1(lin1.x[1], lin1.y[1], lin1.z[1]);
   Vector3D a2(lin2.x[1], lin2.y[1], lin2.z[1]);
   return acos(a1.DotProduct(a2)/ (a1.Length() * a2.Length()));
}
   

double Distance2Lines(const Line3D &lin1, const Line3D &lin2)
{
   Vector3D a1(lin1.x[1], lin1.y[1], lin1.z[1]);
   Vector3D a2(lin2.x[1], lin2.y[1], lin2.z[1]);
   Vector3D a = a1.VectorProduct(a2);
   Vector3D b(lin1.x[0] - lin2.x[0], lin1.y[0] - lin2.y[0], lin1.z[0] - lin2.z[0]);
   double d = b.DotProduct(a);
   if (d < 0) d = -d;
   d = d / a.Length();
   return d; 
}        


bool operator==(const Line3D &lin1, const Line3D &lin2)
{
   return (lin1.x[0] == lin2.x[0] && lin1.x[1] == lin2.x[1] &&
   		lin1.y[0] == lin2.y[0] && lin1.y[1] == lin2.y[1] &&
   		lin1.z[0] == lin2.z[0] && lin1.z[1] == lin2.z[1]);
}   		

/*
void Line3D::Rotate(Rotation3D r)
{ 
  Vector3D v1(x[0], y[0], z[0]);
  Vector3D v2(x[0] + x[1], y[0] + y[1], z[0] + z[1]);
  v1 = r * v1;
  v2 = r * v2;
  Line3D lin(v1, v2);
  *this = lin; 
}

void Line3D::Translate(Vector3D t)
{
  x[0] += t.X();
  y[0] += t.Y();
  z[0] += t.Z(); 
}
*/

void Line3D::Print() const
{
   printf("x0 = %12.3f, x1 = %6.3f\n", x[0], x[1]);
   printf("y0 = %12.3f, y1 = %6.3f\n", y[0], y[1]);
   printf("z0 = %12.3f, z1 = %6.3f\n", z[0], z[1]);
}   

void Line3D::PrintInt() const
{
   printf("x0 = %ld, x1 = %ld\n", (long)(1000*x[0]), (long)(1000*x[1]));
   printf("y0 = %ld, y1 = %ld\n", (long)(1000*y[0]), (long)(1000*y[1]));
   printf("z0 = %ld, z1 = %ld\n", (long)(1000*z[0]), (long)(1000*z[1]));
}   

/*
--------------------------------------------------------------------------------
                    Project a point onto the line
--------------------------------------------------------------------------------
*/

Position3D Line3D::Project(const Position3D &point) const
{
  Plane      plane;
  Vector3D   direction;
  Position3D projection;

/* Construct a plane through the point and perpendicular to the line */

  direction = Direction();
  plane.Normal() = direction.Normalize();
  plane.Distance() = point.DotProduct(plane.Normal());

/* Intersect the line with this plane */

  IntersectLine3DPlane(*this, plane, projection);
  return(projection);
}

/*
--------------------------------------------------------------------------------
                    Determine scalar value of point's projection
--------------------------------------------------------------------------------
*/

double Line3D::Scalar(const Position3D &point) const
{
  Position3D projection;
  Vector3D   dir;

  projection = Project(point);
  dir        = Direction();
  if (ABS(dir.X()) > ABS(dir.Y()) &&         // largest component in X direction
      ABS(dir.X()) > ABS(dir.Z()))
    return((projection.X() - FootPoint().X()) / dir.X());
  else if (ABS(dir.Y()) > ABS(dir.Z()))      // largest component in Y direction
    return((projection.Y() - FootPoint().Y()) / dir.Y());
  else                                       // largest component in Z direction
    return((projection.Z() - FootPoint().Z()) / dir.Z());
}
/*
--------------------------------------------------------------------------------
                            Add or remove a point to the line
--------------------------------------------------------------------------------
*/

bool Line3D::AddPoint(const Position3D &pos, bool recalculate)
{
  return ChangePoint(pos, true, recalculate);
}

bool Line3D::RemovePoint(const Position3D &pos, bool recalculate)
{
  return ChangePoint(pos, false, recalculate);
}

bool Line3D::ChangePoint(const Position3D &pos, bool add, bool recalculate)
{
  Vector3D diff;

  // Check if moment vectors exist
  if (coord_sum == NULL) {
    coord_sum = new Vector3D();
    offset    = new Vector3D();
    moments   = new double[9];
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
                     Recalculate line parameters from moments
--------------------------------------------------------------------------------
*/

bool Line3D::Recalculate()
{
  Vector3D centre, direction;
  double   tmp1[3], tmp2[3], eigen_real[3], eigen_imag[3], eigen_vec[3][3],
           central_moments[9];
  int      dim=3, matz=1, error, largest;

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

  // Select the largest eigen value
  largest = (eigen_real[0] > eigen_real[1]) ? 0 : 1;
  largest = (eigen_real[largest] > eigen_real[2]) ? largest : 2;

  // Take line through point of gravity with eigen vector as direction vector
  x[0] = centre.X() + offset->X();
  y[0] = centre.Y() + offset->Y();
  z[0] = centre.Z() + offset->Z();
  direction = Vector3D(eigen_vec[largest][0], eigen_vec[largest][1],
                       eigen_vec[largest][2]);
  if (direction.Length() > 0.0) direction = direction.Normalize();
  x[1] = direction.X();
  y[1] = direction.Y();
  z[1] = direction.Z();

  return true;
}

/*
--------------------------------------------------------------------------------
           Estimate the noise standard deviation of the line fitting
--------------------------------------------------------------------------------
*/ 
double Line3D::SigmaLineFitting() const
{
  double var;
  Vector3D d(x[1], y[1], z[1]); // Line direction
  // Line foot point corrected for offset
  Vector3D f(x[0] - offset->X(), y[0] - offset->Y(), z[0] - offset->Z()); 
  Vector3D n;
  
  d = d / d.Length(); // Make sure the line direction has unit length
  n = f.VectorProduct(d);

  // Mirror moments array to make it full and avoid mistakes below
  moments[1] = moments[3];
  moments[2] = moments[6];
  moments[5] = moments[7]; 

  var = num_pts * n.X() * n.X() +
        d.Z() * d.Z() * moments[1*3+1] +       // sum y^2
        d.Y() * d.Y() * moments[2*3+2] -       // sum z^2
        2.0 * n.X() * d.Z() * coord_sum->Y() + // sum y
        2.0 * n.X() * d.Y() * coord_sum->Z() - // sum z
        2.0 * d.Z() * d.Y() * moments[1*3+2] + // sum y z
        
        num_pts * n.Y() * n.Y() +
        d.X() * d.X() * moments[2*3+2] +       // sum z^2
        d.Z() * d.Z() * moments[0*3+0] -       // sum x^2
        2.0 * n.Y() * d.X() * coord_sum->Z() + // sum z
        2.0 * n.Y() * d.Z() * coord_sum->X() - // sum x
        2.0 * d.X() * d.Z() * moments[2*3+0] + // sum z x
       
        num_pts * n.Z() * n.Z() +
        d.Y() * d.Y() * moments[0*3+0] +       // sum x^2
        d.X() * d.X() * moments[1*3+1] -       // sum y^2
        2.0 * n.Z() * d.Y() * coord_sum->X() + // sum x
        2.0 * n.Z() * d.X() * coord_sum->Y() - // sum y
        2.0 * d.Y() * d.X() * moments[0*3+1];  // sum x y
  if (var <= 0.0) return 0.0;
  return sqrt(var/num_pts);
}

/*
--------------------------------------------------------------------------------
                     Erase line data
--------------------------------------------------------------------------------
*/
void Line3D::Erase()
{
  if (coord_sum) {free(coord_sum); coord_sum = NULL;}
  if (offset) {free(offset); offset = NULL;}
  if (moments) {free(moments); moments = NULL;}
  num_pts = 0;
}

/*
--------------------------------------------------------------------------------
                     Derive the centre of gravity
--------------------------------------------------------------------------------
*/
Position3D Line3D::CentreOfGravity() const
{
  Position3D centre;

  // Check if there are points
  if (num_pts == 0) return Position3D(1.0e30, 1.0e30, 1.0e30);

  // Centre of gravity of all points
  centre.vect() = (*coord_sum / (double) num_pts) + *offset;
  return centre;
}
