
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
 Collection of functions for class ObjectPoint

 ObjectPoint& operator=(const ObjectPoint& point)     Copy assignemnt    
 void ObjectPoint::Cpp2C(ObjPt **)      Conversion of C++ class to C structure
 void ObjectPoint::C2Cpp(ObjPt *)       Conversion of C structure to C++ class

 Initial creation
 Author : Ildi Suveg
 Date   : 23-11-1998

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "ObjectPoint.h"
#include "ObjectPoint2D.h"
#include "ControlPoint.h"
#include "ImagePoint.h"
#include "ImageGrid.h"
#include "DataGrid.h"

/*
--------------------------------------------------------------------------------
                           Declarations of C functions                 
--------------------------------------------------------------------------------
*/

extern "C" void Image_To_Object(Grid *, ImgPt *, ObjPt *, double);
  

/*
--------------------------------------------------------------------------------
                     Construct from a 2D object point
--------------------------------------------------------------------------------
*/

ObjectPoint::ObjectPoint(const ObjectPoint2D *point2D, double z)
  : Point3D()
{
  x[0]   = point2D->x[0];
  x[1]   = point2D->x[1];
  x[2]   = z;
  num    = point2D->num;
  var[0] = point2D->var[0];
  var[1] = point2D->var[1];
  var[2] = 0;
  var[3] = point2D->var[2];
  var[4] = 0;
  var[5] = 0;	
}

/*
--------------------------------------------------------------------------------
                     Construct from a control point
--------------------------------------------------------------------------------
*/

ObjectPoint::ObjectPoint(const ControlPoint *ctrlpt) : Point3D()
{
  Vector3D v;
  v = ctrlpt->vect();
  Covariance3D cv; 
  PointNumber n;
  n = ctrlpt->Number();
  ObjectPoint(v, n, cv);
}

/*
--------------------------------------------------------------------------------
          Construct from an image point using an image grid and a height
--------------------------------------------------------------------------------
*/

ObjectPoint::ObjectPoint(const ImagePoint *imgpoint, const ImageGrid *imggrid,
                         double h) : Point3D()
{
  ObjPt *objpt;
  
  objpt = (ObjPt*) malloc(sizeof(ObjPt));
  
  ImgPt *imgpt;
  imgpt = NULL;
  imgpoint->Cpp2C(&imgpt);
  
  Grid *grid;
  grid = NULL;
  imggrid->Cpp2C(&grid);
  
  Image_To_Object(grid, imgpt, objpt, h);
  
  C2Cpp(objpt);
  
  free(imgpt);
  free(objpt);
  free(grid);   
}

/*
--------------------------------------------------------------------------------
        Construct from an image point using a data grid and a grey value
--------------------------------------------------------------------------------
*/

ObjectPoint::ObjectPoint(const ImagePoint *imgpoint, const DataGrid *grid,
                         double greyvalue) : Point3D()
{
  x[0] = grid->XOffset() + (imgpoint->Column() + 0.5) * grid->Pixelsize();
  x[1] = grid->YOffset() - (imgpoint->Row()    + 0.5) * grid->Pixelsize();
  x[2] = grid->DataOffset() + greyvalue * grid->DataScale();
}

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

ObjectPoint& ObjectPoint::operator=(const ObjectPoint& point)
{
  // Check for self assignment
  if (this == &point) return *this;
  x[0]   = point.x[0];
  x[1]   = point.x[1];
  x[2]   = point.x[2];
  num    = point.num;
  var[0] = point.var[0];
  var[1] = point.var[1];
  var[2] = point.var[2];
  var[3] = point.var[3];
  var[4] = point.var[4];
  var[5] = point.var[5];
  return *this;
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ObjectPoint::Cpp2C(ObjPt **objptptr) const
{
  ObjPt *objpt;

/* Allocate space if this has not been done yet */

  objpt = *objptptr;
  if (objpt == NULL) {
    objpt = (ObjPt*) malloc(sizeof(ObjPt));
    *objptptr = objpt;
  }

/* Copy the data from the C++ to the C object */

  objpt->x     = x[0];
  objpt->y     = x[1];
  objpt->z     = x[2];
  objpt->num   = num;
  objpt->v_x   = var[0];
  objpt->v_y   = var[1];
  objpt->v_z   = var[2];
  objpt->cv_xy = var[3];
  objpt->cv_xz = var[4];
  objpt->cv_yz = var[5];

}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ObjectPoint::C2Cpp(ObjPt *objpt)
{
  x[0]   = objpt->x;
  x[1]   = objpt->y;
  x[2]   = objpt->z;
  num    = objpt->num;
  var[0] = objpt->v_x;
  var[1] = objpt->v_y;
  var[2] = objpt->v_z;
  var[3] = objpt->cv_xy;
  var[4] = objpt->cv_xz;
  var[5] = objpt->cv_yz;
}

