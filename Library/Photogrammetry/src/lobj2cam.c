
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
  * $Log$
  * Revision 1.3  2010/07/09 08:40:05  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:41:22  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:18  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:00  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:29  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:58  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.1.1.1  2003/04/09 11:07:37  rabbani
  * basic photogrammetric classes and functions (orientation, in/output, matrices, ...
  *
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:52  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for obj2cam
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lobj2cam
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "Database.h"
/* -library_includes_end */

/****************************************************************
* 
*  Routine Name: lobj2cam - *
* 
*       Purpose: Library Routine for obj2cam
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: R.Th. Ursem
*          Date: Jul 28, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lobj2cam(ObjPt *objpt, Interior *interior, Exterior *exterior, CamPt *campt)
/* -library_def_end */

/* -library_code */
{
   double cc, xdiff, ydiff, zdiff, denominator;

   xdiff = (objpt->x - exterior->x);
   ydiff = (objpt->y - exterior->y);
   zdiff = (objpt->z - exterior->z);
   denominator = exterior->rot[0][2] * xdiff + 
                 exterior->rot[1][2] * ydiff + 
                 exterior->rot[2][2] * zdiff;
   
   if (!denominator)
   {
      printf("lobj2cam: devide by zero!\n");
      return (0);
   }
                        
   campt->num = objpt->num;
   
   cc = interior->cc;
   campt->x = -cc * (exterior->rot[0][0] * xdiff + 
                     exterior->rot[1][0] * ydiff +
                     exterior->rot[2][0] * zdiff) / denominator;
   campt->y = -cc * (exterior->rot[0][1] * xdiff + 
                     exterior->rot[1][1] * ydiff +
                     exterior->rot[2][1] * zdiff) / denominator;
   return(1);
}
/* -library_code_end */
