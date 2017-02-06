
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
 Collection of functions for class ModelPoints          

 void ModelPoints::Cpp2C(ModelPts **     Conversion of C++ to C object
 void ModelPoints::C2Cpp(ModelPts *)     Conversion of C to C++ object
 int  ModelPoints::Read(char *)          Read Object points from a database
 int  ModelPoints::Write(char *)         Write Object points to a database
 void ModelPoints::Print()               Print Object points to stdout

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

#include "ModelPoints.h"

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ModelPoints::Cpp2C(ModelPts **modelptsptr) const
{
  ModelPts *modelpts;

// Allocate space if this has not been done yet 

  modelpts = *modelptsptr;
  if (modelpts == NULL) {
    modelpts = (ModelPts *) malloc(sizeof(ModelPts));
    modelpts->pts = (ModelPt*) malloc(size() * sizeof(ModelPt));
    *modelptsptr = modelpts;
  }

// Copy the data 

  modelpts->num_pts = size();
  ModelPoints::const_iterator i;
  ModelPoint point;
  ModelPt *pt; 
  int l;
  for(i = begin(), l = 0; i != end(); i++, l++)
  {
	point = *i;
	pt = &modelpts->pts[l];
        point.Cpp2C(&pt);
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ModelPoints::C2Cpp(ModelPts *modelpts)
{
  if (!empty()) erase(begin(), end());
  //reserve(modelpts->num_pts);

  for(int i = 0 ; i < modelpts->num_pts; i++)
  {
  	ModelPoint point(&modelpts->pts[i]);
  	push_back(point);
  }
}

/*
--------------------------------------------------------------------------------
                       Read Model points from a database
--------------------------------------------------------------------------------
*/
int ModelPoints::Read(char *filename)
{
  ModelPts *modelpts;

  modelpts = Get_ModelPts(filename);    /* Read the database into C structure */
  if (modelpts == NULL) return(0);
  C2Cpp(modelpts);                      /* Convert to C++ object */
  Free_ModelPts(modelpts);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write Model points to a database
--------------------------------------------------------------------------------
*/
int ModelPoints::Write(char *filename) const
{
  int    error;
  ModelPts *modelpts;

  modelpts = NULL;
  Cpp2C(&modelpts);
  error = Put_ModelPts(modelpts, filename);
  Free_ModelPts(modelpts);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print Model points to stdout
--------------------------------------------------------------------------------
*/
void ModelPoints::Print() const
{
  ModelPts *modelpts;

  modelpts = NULL;
  Cpp2C(&modelpts);
  Print_ModelPts(modelpts);
  Free_ModelPts(modelpts);
}

