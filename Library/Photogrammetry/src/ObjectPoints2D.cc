
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
 Collection of functions for class ObjectPoints2D          

 Initial creation
 Author : Ildi Suveg
 Date   : 23-11-1998

 Update #1
 Author : George Vosselman
 Date   : 27-12-2000
 Changes: Added copy assignment

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "ObjectPoints2D.h"
#include "ImagePoints.h"
#include "ImageGrid.h"

/*
--------------------------------------------------------------------------------
                     Construction from image points
--------------------------------------------------------------------------------
*/

ObjectPoints2D::ObjectPoints2D(const ImagePoints &imagepoints, const ImageGrid &grid)
  : VectorPoint<ObjectPoint2D>()
{
  ImagePoints::const_iterator imagepoint;

  for (imagepoint=imagepoints.begin();
       imagepoint!=imagepoints.end();
       imagepoint++) {
    push_back(ObjectPoint2D(&*imagepoint, &grid));
  }
}
/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ObjectPoints2D::Cpp2C(ObjPts2D **objptsptr) const
{
  ObjPts2D *objpts;

/* Allocate space if this has not been done yet */

  objpts = *objptsptr;
  if (objpts == NULL) {
    objpts = (ObjPts2D *) malloc(sizeof(ObjPts2D));
    objpts->pts = (ObjPt2D*) malloc(size() * sizeof(ObjPt2D));
    *objptsptr = objpts;
  }

/* Copy the data */

  objpts->num_pts = size();
  ObjectPoints2D::const_iterator i;
  ObjPt2D *pt;
  int l = 0;
  for(i = begin(); i != end(); i++, l++)
  {
	pt = &objpts->pts[l];
        i->Cpp2C(&pt);
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ObjectPoints2D::C2Cpp(ObjPts2D *objpts)
{
  if (!empty()) erase(begin(), end());
  reserve(objpts->num_pts);
  	
  ObjectPoint2D point; 	
  for(int i = 0 ; i < objpts->num_pts; i++)
  {
  	point.C2Cpp(&objpts->pts[i]);
  	push_back(point);
  }
}

/*
--------------------------------------------------------------------------------
                       Read object points from a database
--------------------------------------------------------------------------------
*/
int ObjectPoints2D::Read(const char *filename)
{
  ObjPts2D *objpts;

  objpts = Get_ObjPts2D(filename);        /* Read the database into C structure */
  if (objpts == NULL) return(0);
  C2Cpp(objpts);                        /* Convert to C++ object */
  Free_ObjPts2D(objpts);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write object points to a database
--------------------------------------------------------------------------------
*/
int ObjectPoints2D::Write(const char *filename) const
{
  int    error;
  ObjPts2D *objpts;

  objpts = NULL;
  Cpp2C(&objpts);
  error = Put_ObjPts2D(objpts, filename);
  Free_ObjPts2D(objpts);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print object points to stdout
--------------------------------------------------------------------------------
*/
void ObjectPoints2D::Print() const
{
  ObjPts2D *objpts;

  objpts = NULL;
  Cpp2C(&objpts);
  Print_ObjPts2D(objpts);
  Free_ObjPts2D(objpts);
}


/*
--------------------------------------------------------------------------------
                  Write header of object points database
--------------------------------------------------------------------------------
*/
int ObjectPoints2D::WriteHeader(const char *filename) const
{
  FILE       *fd;
  
/* Open the file */

  if ((fd = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Could not open %s to write 2D object point database.\n", 
            filename);
    return(0);
  }

/* Write the header */

  fprintf(fd, "%d %c file ID\n", OBJECT_POINTS_2D, comment_char);
  fprintf(fd, "%c Number of 2D object points: %d\n", comment_char, size());
  fprintf(fd, "%c Pointnumber         x              y            var  x      var  y      covar  x , y \n", comment_char);
  fprintf(fd, "%c------------- -------------- -------------- --------------- --------------- ---------------\n",
	  comment_char);
  return(1);
}


/*
--------------------------------------------------------------------------------
                                 Copy assignment
--------------------------------------------------------------------------------
*/

ObjectPoints2D &ObjectPoints2D::operator = (const ObjectPoints2D & points)
{
  if (!empty()) erase(begin(), end());
  if (!points.empty()) insert(begin(), points.begin(), points.end());
  return(*this);
}
