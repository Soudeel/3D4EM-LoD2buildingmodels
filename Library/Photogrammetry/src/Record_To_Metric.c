
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



/*------ Record_To_Metric: Inverse interior orientation of camera to image system ------*/

/*  Version: 1.0
       Date: 21-nov-94
     Author: Frank A. van den Heuvel
      Input: Interior *par - parameters interior orientation (Database.h)
             ImgPt *img_pt - image co-ordinate record (pixel)
     Output: CamPt *cam_pt - camera co-ordinate record (mm)
*/

#include <stdio.h>
#include "Database.h"

void Record_To_Metric(Interior *par, ImgPt *img_pt, CamPt *cam_pt)
{  
  void Pix_To_Metric(Interior *, double, double, double *, double *);

/*------ Transformation of r,c to x,y ------*/
  Pix_To_Metric(par, img_pt->r, img_pt->c, &(cam_pt->x), &(cam_pt->y));

/*------ Copy name ------*/
  cam_pt->num = img_pt->num;

/*------ Variance-Covariance transformation ------*/
  cam_pt->v_x   =  img_pt->v_c   * (par->spacing_c * par->spacing_c);
  cam_pt->v_y   =  img_pt->v_r   * (par->spacing_r * par->spacing_r);
  cam_pt->cv_xy = -img_pt->cv_rc * (par->spacing_r * par->spacing_c);

}
