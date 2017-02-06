
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
 Generation and optional output of contours of rough planar point clouds

 Initial creation:
 Author : George Vosselman
 Date   : 29-03-2001

 Update #1
 Author : George Vosselman
 Date   : 10-11-2002
 Purpose: Added generated contours as variable in function call
 

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "ObjectPoints.h"
#include "LaserPoint.h"
#include "LaserPoints.h"
#include "BNF_io.h"
#include "VRML_io.h"
#include "Planes.h"
#include "PointNumberLists.h"
#include "LineTopologies.h"

void LaserPoints::Rough_Contours(PointNumberLists &faces, Planes &planes,
                                 TINEdges &edges, char *top_file,
                                 char *vrml_file, LineTopologies &face_contours)
{
  LineTopologies::iterator face_contour;
  PointNumberLists::iterator face;
  Planes::iterator           plane;
//ObjectPoints    centroids;
  FILE            *vrml;
  float           col_seed, red, green, blue;

// Clear face contours

  if (!face_contours.empty())
    face_contours.erase(face_contours.begin(), face_contours.end());

/* Determine the face contours */

  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end(); face++, plane++) {
    face_contours.push_back(DeriveContour(plane->Number(),
                                          face->PointNumberListReference(),
                                          edges));
//  centroids.push_back(Centroid(face->PointNumberListReference(),
//                               plane->Number()));
  }
//centroids.Write("centroids.objpts");

/* Close the contours */

  for (face_contour=face_contours.begin();
       face_contour!=face_contours.end();
       face_contour++)
    face_contour->push_back(*(face_contour->begin()));

/* Output of faces to VRML */

  if (vrml_file) {
    vrml = VRML_Open(vrml_file);
    VRML_Write(vrml);
    col_seed = 1.0;
    for (face_contour=face_contours.begin();
         face_contour!=face_contours.end();
         face_contour++) {
      VRML_Scalar_To_Colour(col_seed, &red, &green, &blue);
      col_seed += 0.7;  if (col_seed > 1.0) col_seed -= 1.0;
      VRML_Set_Colour(vrml, red, green, blue);
      face_contour->VRML_Write(vrml, 1);
    }
    VRML_Close(vrml);
  }

/* Output of face topologies */

  if (top_file) face_contours.Write(top_file);
}
