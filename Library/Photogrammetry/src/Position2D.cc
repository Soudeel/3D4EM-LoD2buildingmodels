
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
 Collection of functions for class Position2D

 Position2D::Position2D(char *)
 void Position2D::Read(char *)
 void Position2D::Write(FILE *)
 Position2D::Position2D(PixelPosition *, InteriorOrientation *)

 Initial creation
 Author : Ildiko Suveg
 Date   : 03-03-1999

 Update #1
 Author : George Vosselman
 Date   : 19-03-1999
 Changes: Added I/O functionality

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "Position2D.h"

#include "PixelPosition.h"
#include "InteriorOrientation.h"
#include "ObjectPoint.h"
#include "ObjectPoints.h"
#include "Positions3D.h"
#include "Position3D.h"
#include "Positions2D.h"
#include "Plane.h"
#include "LineTopology.h"
#include "Line2D.h"
#include "LineSegment2D.h"

extern "C" int Pix_To_Metric(Interior *, double, double, double *, double *);


/*
--------------------------------------------------------------------------------
              Construct by reading a position from a character string
--------------------------------------------------------------------------------
*/

Position2D::Position2D(char *line) : Vector2D()
{
  Read(line);
}


/*
--------------------------------------------------------------------------------
                    Read a position from a character string
--------------------------------------------------------------------------------
*/

void Position2D::Read(char *line)
{
  sscanf(line, "%lf %lf", &(x[0]), &(x[1]));
}


/*
--------------------------------------------------------------------------------
           Write a position to a line of the 2D position database
--------------------------------------------------------------------------------
*/

void Position2D::Write(FILE *fd) const
{
  fprintf(fd, "  %15.6f  %15.6f\n", x[0], x[1]);
}


void Position2D::Print() const
{
  printf("%15.6f  %15.6f\n", x[0], x[1]);
}

/*
--------------------------------------------------------------------------------
          Transformation of pixel coordinates to camera coordinates
--------------------------------------------------------------------------------
*/



Position2D::Position2D(const PixelPosition *pos, const InteriorOrientation *intor)
  : Vector2D()
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  Pix_To_Metric(into, pos->x[0], pos->x[1], &x[0], &x[1]); 
  
  free(into);
}   


double Position2D::Distance(const Position2D &pos) const
{
   Vector2D v = *this - pos;
   return(v.Length());
}


Position3D Position2D::To3D(Plane myplane, ExteriorOrientation ext, double focal, int image_w, int image_h)  
{    
     /*
     double a1, a2, b1, b2, c1, c2, d1, d2, distance1, distance2,;
     double x2d, y2d;
     Plane p1, p2;
     
     x2d=x[0]-(image_w-1)/2; y2d=(image_h-1)/2-x[1];
     
     a1=focal*ext.R(0,0)+x2d*ext.R(0,2);
     b1=focal*ext.R(1,0)+x2d*ext.R(1,2);
     c1=focal*ext.R(2,0)+x2d*ext.R(2,2);
     d1=(ext.R(0,0)*ext.X()+ext.R(1,0)*ext.Y()+ext.R(2,0)*ext.Z())*focal+ext.R(0,2)*ext.X()*x2d+ext.R(1,2)*ext.Y()*x2d+ext.R(2,2)*ext.Z()*x2d;
     d1=-d1;     
     distance1=d1/sqrt(a1*a1+b1*b1+c1*c1); 
     p1.SetNormal((Vector3D(a1,b1,c1)).Normalize()); p1.SetDistance(distance1);
     
     a2=focal*ext.R(0,1)+y2d*ext.R(0,2);
     b2=focal*ext.R(1,1)+y2d*ext.R(1,2);
     c2=focal*ext.R(2,1)+y2d*ext.R(2,2);
     d2=(ext.R(0,1)*ext.X()+ext.R(1,1)*ext.Y()+ext.R(2,1)*ext.Z())*focal+ext.R(0,2)*ext.X()*y2d+ext.R(1,2)*ext.Y()*y2d+ext.R(2,2)*ext.Z()*y2d;  
     d2=-d2;
     distance2=d2/sqrt(a2*a2+b2*b2+c2*c2);
     p2.SetNormal((Vector3D(a2,b2,c2)).Normalize()); p2.SetDistance(distance2);
     
     Line3D line;
     
     Intersect2Planes(p1,p2,line);
     
     Position3D point;
     
     IntersectLine3DPlane(line, myplane, point);
     
     return point;      
     */
     double x2d, y2d;
     
     x2d=x[0]-(image_w-1)/2; y2d=(image_h-1)/2-x[1];
     
     Vector3D vc(x2d,y2d,-focal);
     
     Vector3D vt; vt=ext.rotation()*vc;
     
     Position3D camera_pos, temp;
     
     camera_pos=Position3D(ext.X(),ext.Y(),ext.Z());
     
     temp=Position3D(camera_pos+vt);
     
     Line3D line(camera_pos,temp);
     
     Position3D point;
     
     IntersectLine3DPlane(line, myplane, point);
     
     return point;      
}
