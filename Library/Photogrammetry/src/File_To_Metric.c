
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



/*------ File_To_Metric: Inverse interior orientation of camera to image system ------*/

/*  Version: 1.0
       Date: 21-nov-94
     Author: Frank A. van den Heuvel
      Input: Interior *par - parameters interior orientation (Database.h)
             ImgPts *img   - image co-ordinate File
     Output: CamPts *cam   - camera co-ordinate File
*/

#include <stdio.h>
#include "Database.h"

void File_To_Metric(Interior *par, ImgPts *img, CamPts *cam)
{  
  int i;
  
  void Record_To_Metric(Interior *, ImgPt *, CamPt *);

/*------ Initialize transformation ------*/
  cam->num_pts = img->num_pts;

/*------ Transformation of record ------*/
  for (i=0;i<img->num_pts;i++)
    Record_To_Metric(par, &(img->pts[i]), &(cam->pts[i]));
}
