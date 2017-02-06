
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
 Collection of functions for class Position3D

 Position3D::Position3D(char *)            Construct by read from string
 void Position3D::Read(char *)             Read position from a string
 void Position3D::Write(FILE *)            Write position to a file
 void Position3D::VRML_Write(FILE *) const Write a point to a VRML file
 void Position3D::VRML_Write_Cross         Write a cross to a VRML file
   (FILE *, double) const
 void Position3D::VRML_Write_Sphere        Write a sphere to a VRML file
   (FILE *, double) const
 double Position3D::Distance               Distance between two positions
   (const Position3D &) const
 double Position3D::Distance               Distance of a position to a plane
   (const Plane &) const

 Initial creation
 Author : George Vosselman
 Date   : 19-03-1999

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

#include <stdio.h>
#include "Position3D.h"
#include "VRML_io.h"
#include "Plane.h"

/*
--------------------------------------------------------------------------------
              Construct by reading a position from a character string
--------------------------------------------------------------------------------
*/

Position3D::Position3D(char *line) : Vector3D()
{
  Read(line);
}


/*
--------------------------------------------------------------------------------
                    Read a position from a character string
--------------------------------------------------------------------------------
*/

void Position3D::Read(char *line)
{
  sscanf(line, "%lf %lf %lf", &(x[0]), &(x[1]), &(x[2]));
}


/*
--------------------------------------------------------------------------------
           Write a position to a line of the 3D position database
--------------------------------------------------------------------------------
*/

void Position3D::Write(FILE *fd) const
{
  fprintf(fd, "  %15.6f  %15.6f  %15.6f\n", x[0], x[1], x[2]);
}

/*
--------------------------------------------------------------------------------
               Write a point or cross to a VRML file
--------------------------------------------------------------------------------
*/

void Position3D::VRML_Write(FILE *fd) const
{
  fprintf(fd, "  %15.6f  %15.6f  %15.6f,\n", x[0], x[1], x[2]);
}

void Position3D::VRML_Write_Cross(FILE *fd, double size) const
{
  Position3D point;

  VRML_Write_Begin_Of_PointList(fd);
  point = *this; 
  point.X() -= size;    point.VRML_Write(fd);
  point.X() += 2*size;  point.VRML_Write(fd);  point.X() -= size;
  point.Y() -= size;    point.VRML_Write(fd);
  point.Y() += 2*size;  point.VRML_Write(fd);  point.Y() -= size;
  point.Z() -= size;    point.VRML_Write(fd);
  point.Z() += 2*size;  point.VRML_Write(fd);
  VRML_Write_End_Of_PointList(fd);
  fprintf(fd, "IndexedLineSet {coordIndex[0, 1, -1]}\n");
  fprintf(fd, "IndexedLineSet {coordIndex[2, 3, -1]}\n");
  fprintf(fd, "IndexedLineSet {coordIndex[4, 5, -1]}\n");
}

void Position3D::VRML_Write_Sphere(FILE *fd, double radius) const
{
  fprintf(fd, "Separator {\n");
  fprintf(fd, "Translation { translation %15.6f %15.6f %15.6f }\n",
          x[0], x[1], x[2]);
  fprintf(fd, "Sphere { radius %15.6f }\n", radius);
  fprintf(fd, "}\n");
}


double Position3D::Distance(const Position3D &pos2) const
{
   Vector3D v = *this - pos2;
   return v.Length();
}

double Position3D::Distance2D(const Position3D &pos2) const
{
   Vector3D v = *this - pos2;
   return v.Length2D();
}

double Position3D::Distance(const Plane &plane) const
{
   return(DotProduct(plane.Normal()) - plane.Distance());
}

///Calculate the image position based on Exterior Orientation
Position2D Position3D::ToPixel(ExteriorOrientation ext, double focal) const
{
   Position2D p2d;
    double ximt,  yimt;
    double zim, scale;
   
    ximt = ext.R(0,0) * (x[0] - ext.X()) +
	           ext.R(1,0) * (x[1] - ext.Y()) +
	           ext.R(2,0) * (x[2] - ext.Z());
	 
	 yimt = ext.R(0,1) * (x[0] - ext.X()) +
	           ext.R(1,1) * (x[1] - ext.Y()) +
	           ext.R(2,1) * (x[2]- ext.Z());
	 
	 zim     = ext.R(0,2) * (x[0] - ext.X()) +
	           ext.R(1,2) * (x[1] - ext.Y()) +
	           ext.R(2,2) * (x[2] - ext.Z());
	 
	 scale   = -focal / zim;
	 ximt*= scale;
	 yimt *= scale;    
     
     p2d.X()=ximt;
     p2d.Y()=yimt;    
           
     return p2d;      
}

/// Check if the location is the same
bool Position3D::SameLocation(const Position3D &pos) const
{
  return (*this == pos);
}
