
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
  * Revision 1.1  2006/04/06 14:10:17  WINDOWSNT\vosselman
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
  * Revision 1.1.1.1  2003/03/07 12:59:10  pfeifer
  * Standard library: points, matrices, triangulation, ...
  *
  * Revision 1.1.1.1  2003/03/03 14:03:27  rabbani
  * Standard library for FRS
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for lesspts
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	llesspts
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "Database.h"

/* -library_includes_end */

/****************************************************************
* 
*  Routine Name: llesspts -


*       Purpose:


*         Input:


*        Output:


*       Returns:


*  Restrictions:


*    Written By:

George Vosselman

*          Date:
 May 04, 1999
*      Verified:


 
*  Side Effects:



 
* Modifications:



 
****************************************************************/
/* -library_def */
void llesspts(ImgPts *imgpts, int pd, int ncol, ImgPts *selpts)
/* -library_def_end */

/* -library_code */
{
  int   divr, divc;
  int   irow, icol, i;
  ImgPt *imgpt, *selpt;

  for (i=0, imgpt=imgpts->pts, selpt=selpts->pts;
       i<imgpts->num_pts;
       i++, imgpt++) {
    irow = (imgpt->num - 1) / ncol + 1;
    icol = (imgpt->num - (irow - 1) * ncol);
    divr = irow - (irow / pd) * pd;
    divc = icol - (icol / pd) * pd;
    if (divr == 1 && divc == 1) {
      *selpt = *imgpt;
      selpt++;
      selpts->num_pts++;
    }
  }
}

/* -library_code_end */
