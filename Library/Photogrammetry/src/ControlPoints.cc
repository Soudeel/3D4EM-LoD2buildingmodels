
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
 Collection of functions for class ControlPoints          

 void ControlPoints::Cpp2C(CtrlPts **     Conversion of C++ to C object
 void ControlPoints::C2Cpp(CtrlPts *)     Conversion of C to C++ object
 int  ControlPoints::Read(char *)         Read Object points from a database
 int  ControlPoints::Write(char *)        Write Object points to a database
 void ControlPoints::Print()              Print Object points to stdout
 PointNumber ControlPoints::AddPoint(ControlPoint *)  Add a point to the list

 Initial creation
 Author : Ildi Suveg
 Date   : 24-11-1998

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

#include "ControlPoints.h"

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ControlPoints::Cpp2C(CtrlPts **ctrlptsptr) const
{
  CtrlPts *ctrlpts;

/* Allocate space if this has not been done yet */

  ctrlpts = *ctrlptsptr;
  if (ctrlpts == NULL) {
    ctrlpts = (CtrlPts *) malloc(sizeof(CtrlPts));
    ctrlpts->pts = (CtrlPt*) malloc(size() * sizeof(CtrlPt));
    *ctrlptsptr = ctrlpts;
  }

/* Copy the data */

  ctrlpts->num_pts = size();
  ControlPoints::const_iterator i;
  CtrlPt *pt; 
  int l = 0;
  for(i = begin(); i != end(); i++, l++)
  {
	pt = &ctrlpts->pts[l];
        i->Cpp2C(&pt);
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ControlPoints::C2Cpp(CtrlPts *ctrlpts)
{
  if (!empty()) erase(begin(), end());
  reserve(ctrlpts->num_pts);
  	
  for(int i = 0 ; i < ctrlpts->num_pts; i++)
  {
  	ControlPoint point(&ctrlpts->pts[i]);
  	push_back(point);
  }
}

/*
--------------------------------------------------------------------------------
                       Read control points from a database
--------------------------------------------------------------------------------
*/
int ControlPoints::Read(char *filename)
{
  CtrlPts *ctrlpts;

  ctrlpts = Get_CtrlPts(filename);    /* Read the database into C structure */
  if (ctrlpts == NULL) return(0);
  C2Cpp(ctrlpts);                      /* Convert to C++ object */
  Free_CtrlPts(ctrlpts);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write control points to a database
--------------------------------------------------------------------------------
*/
int ControlPoints::Write(char *filename) const
{
  int    error;
  CtrlPts *ctrlpts;

  ctrlpts = NULL;
  Cpp2C(&ctrlpts);
  error = Put_CtrlPts(ctrlpts, filename);
  Free_CtrlPts(ctrlpts);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print control points to stdout
--------------------------------------------------------------------------------
*/
void ControlPoints::Print() const
{
  CtrlPts *ctrlpts;

  ctrlpts = NULL;
  Cpp2C(&ctrlpts);
  Print_CtrlPts(ctrlpts);
  Free_CtrlPts(ctrlpts);
}
    			

