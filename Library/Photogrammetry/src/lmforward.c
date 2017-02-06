
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
  * Revision 1.1  2005/07/04 13:43:57  WINDOWSNT\heuel
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
   >>>> 	Library Routine for mforward
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lmforward
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "Database.h"

struct ImgData { ImgPts   *imgpts;
                 char     *imgname;
                 char     *basename;
                 Exterior *extor;
                 Interior *intor; };
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lmforward - Forward intersection of points measured in multiple images and grouping of image point data with the corresponding interior and exterior orientation data in one structure.
* 
*       Purpose: Library Routine for mforward
*         Input: .IP "imagepoint_filter" 15
File filter for selecting the image point databases.
.IP "exterior_filter" 15
File filter for selecting the exterior orientation files.
.IP "interior_filter" 15
File filter for selecting the interior orientation files.
*        Output: .IP "imgdataptr" 15
Array of structures containing the collected information
(points, interior and exterior orientation) per image.
.IP "objptsptr" 15
The calculated object points.
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: George Vosselman
*          Date: Jul 28, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lmforward(char *imagepoint_filter, char *exterior_filter,
              char *interior_filter, ObjPts **objptsptr)
/* -library_def_end */

/* -library_code */
{

/* Variables */

  struct ImgData *imgdata;
  int            num_imgs;

/* Functions */

  void Collect_Data(char *, char *, char *, struct ImgData **, int *);
  void Multiple_Intersection(ObjPts **, struct ImgData *, int);

/*------------------------------------------------------------------------------
  Input of the image point data and the corresponding orientation data
------------------------------------------------------------------------------*/

  Collect_Data(imagepoint_filter, exterior_filter, interior_filter,
               &imgdata, &num_imgs);

/* Check if there are enough images to continue */

  if (num_imgs < 2) {
    if (num_imgs == 0)
      fprintf(stderr, "Error: There are no images with orientation data.\n");
    if (num_imgs == 1) {
      fprintf(stderr, "Error: There is only one image with orientation data.\n");
      fprintf(stderr, "       This is not sufficient to calculate approximate object coordinates.\n");
    }
    return(0);
  }
  printf("There are %d images with orientation data.\n", num_imgs);

/*------------------------------------------------------------------------------
  Calculate the approximate object coordinates
------------------------------------------------------------------------------*/

  Multiple_Intersection(objptsptr, imgdata, num_imgs);

  return(1);
}
/* -library_code_end */
