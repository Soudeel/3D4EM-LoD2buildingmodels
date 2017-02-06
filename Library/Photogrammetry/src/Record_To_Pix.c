
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



/*------ Record_To_Pix: Inverse interior orientation of camera to image system ------*/

/*  Version: 1.0
       Date: 21-nov-94
     Author: Frank A. van den Heuvel
      Input: Interior *par - parameters interior orientation (Database.h)
             CamPt *cam_pt - camera co-ordinate record (mm)
     Output: ImgPt *img_pt - image co-ordinate record (pixel)
*/

#include <stdio.h>
#include "Database.h"

void Record_To_Pix(Interior *par, CamPt *cam_pt, ImgPt *img_pt)
{  
  void Metric_To_Pix(Interior *, double, double, double *, double *);
  
/*------ Transformation of x,y to r,c ------*/
  Metric_To_Pix(par, cam_pt->x, cam_pt->y, &(img_pt->r), &(img_pt->c));

/*------ Copy name ------*/
  img_pt->num = cam_pt->num;

/*------ Variance-Covariance transformation ------*/
  img_pt->v_r   =  cam_pt->v_y   / (par->spacing_r * par->spacing_r);
  img_pt->v_c   =  cam_pt->v_x   / (par->spacing_c * par->spacing_c);
  img_pt->cv_rc = -cam_pt->cv_xy / (par->spacing_r * par->spacing_c);

}
