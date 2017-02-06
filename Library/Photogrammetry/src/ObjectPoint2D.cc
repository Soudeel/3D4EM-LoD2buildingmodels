
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
 Collection of functions for class ObjectPoint2D

 ObjectPoint2D::ObjectPoint2D(ImagePoint *, Construct from image point
                              ImageGrid *)
 ObjectPoint2D::ObjectPoint2D(ObjectPoint &) Make a 2D version of a 3D point
 ObjectPoint2D& ObjectPoint2D::operator=    Copy assignment
   (const ObjectPoint2D&)
 void ObjectPoint2D::Cpp2C(ObjPt2D **)      Conversion of C++ to C 
 void ObjectPoint2D::C2Cpp(ObjPt2D *)       Conversion of C to C++ 
 void ObjectPoint2D::Write(FILE *)          Write point to a record
 int ObjectPoint2D::IntersectLines          Intersect two line segments
   (ObjectPoints2D &, LineTopology &,
    LineTopology &)
 int ObjectPoint2D::InsidePolygon           Point inside polygon test
   (ObjectPoints2D &, LineTopology &)
 int ObjectPoint2D::InsidePolygon           Point inside polygon test
   (ObjectPoints &, LineTopology &)

 Initial creation
 Author : Ildi Suveg
 Date   : 23-11-1998

 Update #1
 Author : George Vosselman
 Date   : 30-11-1999
 Changes: Added Write(FILE *)

 Update #2
 Author : George Vosselman
 Date   : 25-08-2000
 Changes: Added IntersectLines and InsidePolygon

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "ObjectPoints2D.h"
#include "ObjectPoints.h"
#include "ImagePoint.h"
#include "ImageGrid.h"
#include "LineTopology.h"

/*
--------------------------------------------------------------------------------
                     Declarations of old DigPhot functions in C
--------------------------------------------------------------------------------
*/

extern "C" void Image_To_Object_2D(Grid *, ImgPt *, ObjPt2D *);


/*
--------------------------------------------------------------------------------
                  Construct from an image point using a grid 
--------------------------------------------------------------------------------
*/

ObjectPoint2D::ObjectPoint2D(const ImagePoint *imgpoint, const ImageGrid *imggrid)
  : Point2D()
{
  ObjPt2D *objpt;
  objpt = (ObjPt2D*) malloc(sizeof(ObjPt2D));
  
  ImgPt *imgpt;
  imgpt = NULL;
  imgpoint->Cpp2C(&imgpt);
  
  Grid *grid;
  grid = NULL;
  imggrid->Cpp2C(&grid);
  
  Image_To_Object_2D(grid, imgpt, objpt);
  
  C2Cpp(objpt);
  
  free(imgpt);
  free(objpt);
  free(grid);   
}

/*
--------------------------------------------------------------------------------
                 Make a 2D object point from a 3D object point
--------------------------------------------------------------------------------
*/

ObjectPoint2D::ObjectPoint2D(const ObjectPoint& objpt) : Point2D()
{
  x[0]   = objpt.X();
  x[1]   = objpt.Y();
  num    = objpt.Number();
  var[0] = objpt.Covar(0);
  var[1] = objpt.Covar(1);
  var[2] = objpt.Covar(3);
}

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

ObjectPoint2D& ObjectPoint2D::operator=(const ObjectPoint2D& objpt)
{
  // Check for self assignment
  if (this == &objpt) return *this;
  x[0]   = objpt.x[0];
  x[1]   = objpt.x[1];
  num    = objpt.num;
  var[0] = objpt.var[0];
  var[1] = objpt.var[1];
  var[2] = objpt.var[2];
    	
  return *this;
}



/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ObjectPoint2D::Cpp2C(ObjPt2D **objptptr) const
{
  ObjPt2D *objpt;

/* Allocate space if this has not been done yet */

  objpt = *objptptr;
  if (objpt == NULL) {
    objpt = (ObjPt2D *) malloc(sizeof(ObjPt2D));
    *objptptr = objpt;
  }

/* Copy the data from the C++ to the C object */

  objpt->x     = x[0];
  objpt->y     = x[1];
  objpt->num   = num;
  objpt->v_x   = var[0];
  objpt->v_y   = var[1];
  objpt->cv_xy = var[2];

}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ObjectPoint2D::C2Cpp(ObjPt2D *objpt)
{
  x[0]   = objpt->x;
  x[1]   = objpt->y;
  num    = objpt->num;
  var[0] = objpt->v_x;
  var[1] = objpt->v_y;
  var[2] = objpt->cv_xy;
}

/*
--------------------------------------------------------------------------------
                     Write point to a record
--------------------------------------------------------------------------------
*/

void ObjectPoint2D::Write(FILE *fileptr) const
{
  fprintf(fileptr, " %9d%16.5lf%15.5lf%18.5lg%16.5lg%16.5lg\n",
          num, x[0], x[1], var[0], var[1], var[2]);
}

/*
--------------------------------------------------------------------------------
                           Intersect lines
--------------------------------------------------------------------------------
*/

int ObjectPoint2D::IntersectLines(const ObjectPoints2D &points,
                                  const LineTopology &line1,
                                  const LineTopology &line2)
{
  ObjectPoint2D *point1a, *point1b, *point2a, *point2b;
  double        det1, det2;

/* Get all points */

  point1a = points.GetPoint(line1[0]);
  point1b = points.GetPoint(line1[1]);
  point2a = points.GetPoint(line2[0]);
  point2b = points.GetPoint(line2[1]);
  if (point1a == NULL || point1b == NULL ||
      point2a == NULL || point2b == NULL) {
    fprintf(stderr, "Warning: Not all points in database. No intersection computed.\n");
    return(0);
  }

/* Check for parallel lines */

  det1 = (point2a->X() - point2b->X()) * (point1b->Y() - point1a->Y()) -
         (point1b->X() - point1a->X()) * (point2a->Y() - point2b->Y());
  if (det1 == 0.0) return(0);

/* Intersect the lines */

  det2 = (point2a->X() - point1a->X()) * (point1b->Y() - point1a->Y()) -
         (point1b->X() - point1a->X()) * (point2a->Y() - point1a->Y());
  x[0] = point2a->X() + (det2 / det1) * (point2b->X() - point2a->X());
  x[1] = point2a->Y() + (det2 / det1) * (point2b->Y() - point2a->Y());
  return(1);
}

/*
--------------------------------------------------------------------------------
                           Inside polygon test
--------------------------------------------------------------------------------
*/

int ObjectPoint2D::InsidePolygon(const ObjectPoints2D &pts,
                                 const LineTopology &top) const
{
  const ObjectPoint2D          *pt0, *pt1;
  LineTopology::const_iterator node;
  int                          num_intersections;
  double                       Y_test;

/* Check if the polygon is closed and has at least three points */

  if (top.begin()->Number() != (top.end()-1)->Number()) return(0);
  if (top.size() < 4) return(0); /* 4 since begin and end point are the same */

/* Get the first polygon point */

  pt0 = pts.GetPoint(*(top.begin()));

/* Count the number of intersections of polygon edges with the line from the
 * point (X, Y) to (X, inf).
 */

  num_intersections = 0;
  for (node=top.begin()+1; node!=top.end(); node++) {

/* Get the next polygon point */

    pt1 = pts.GetPoint(*node);

/* Check whether the lines intersect */

    if ((pt0->X() - X()) * (pt1->X() - X()) <= 0 && pt1->X() != X()) {
      Y_test = ((pt1->X() - X()) * pt0->Y() +
                (X() - pt0->X()) * pt1->Y()) / (pt1->X() - pt0->X());
      if (Y_test > Y()) num_intersections++;
    }

/* Get ready for the next edge */

    pt0 = pt1;
  }

/* Return 1 for an odd number of intersections */

  if ((num_intersections/2)*2 == num_intersections) return(0);
  else return(1);
}

int ObjectPoint2D::InsidePolygon(const ObjectPoints &pts,
                                 const LineTopology &top) const
{
  const ObjectPoint            *pt0, *pt1;
  LineTopology::const_iterator node;
  int                          num_intersections;
  double                       Y_test;

/* Check if the polygon is closed and has at least three points */

  if (top.begin()->Number() != (top.end()-1)->Number()) return(0);
  if (top.size() < 4) return(0); /* 4 since begin and end point are the same */

/* Get the first polygon point */

  pt0 = pts.GetPoint(*(top.begin()));

/* Count the number of intersections of polygon edges with the line from the
 * point (X, Y) to (X, inf).
 */

  num_intersections = 0;
  for (node=top.begin()+1; node!=top.end(); node++) {

/* Get the next polygon point */

    pt1 = pts.GetPoint(*node);

/* Check whether the lines intersect */

    if ((pt0->X() - X()) * (pt1->X() - X()) <= 0 && pt1->X() != X()) {
      Y_test = ((pt1->X() - X()) * pt0->Y() +
                (X() - pt0->X()) * pt1->Y()) / (pt1->X() - pt0->X());
      if (Y_test > Y()) num_intersections++;
    }

/* Get ready for the next edge */

    pt0 = pt1;
  }

/* Return 1 for an odd number of intersections */

  if ((num_intersections/2)*2 == num_intersections) return(0);
  else return(1);
}
