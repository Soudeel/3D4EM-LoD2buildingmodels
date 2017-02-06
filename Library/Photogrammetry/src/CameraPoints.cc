
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
 Collection of functions for class CameraPoints          

 void CameraPoints::Cpp2C(CamPts **)      Conversion of C++ to C object
 void CameraPoints::C2Cpp(CamPts *)       Conversion of C to C++ object
 int  CameraPoints::Read(char *)          Read camera points from a database
 int  CameraPoints::Write(char *)         Write camera points to a database
 void CameraPoints::Print()               Print camera points to stdout

 Initial creation
 Author : George Vosselman
 Date   : 21-07-1998

 Update #1
 Author : Ildiko Suveg
 Date   : 25-11-1998
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <string.h>
#include "CameraPoints.h"
#include "InteriorOrientation.h"
#include "ExteriorOrientation.h"
#include "ObjectPoints.h"
#include "ControlPoints.h"
#include "ImagePoints.h"

/*
--------------------------------------------------------------------------------
                     Declarations of old DigPhot functions in C
--------------------------------------------------------------------------------
*/
/*
extern "C" CamPts *Get_CamPts(char *);
extern "C" int    Put_CamPts(CamPts *, char *);
extern "C" void   Print_CamPts(CamPts *);
extern "C" void   Free_CamPts(CamPts *);
extern "C" void   Free_ImgPts(ImgPts *);
*/
extern "C" void File_To_Metric(Interior *, ImgPts *, CamPts *);
extern "C" void File_To_Pix(Interior *, CamPts *, ImgPts *);

/*
--------------------------------------------------------------------------------
                 Construct camera points by transforming object points 
--------------------------------------------------------------------------------
*/


CameraPoints::CameraPoints(const ObjectPoints *objpts, const InteriorOrientation *intor, 
    			const ExteriorOrientation *extor)
 : VectorPoint<CameraPoint>()
{
   Object2CameraPoints(objpts, intor, extor);  
}  	


void CameraPoints::Object2CameraPoints(const ObjectPoints *objpts, 
	const InteriorOrientation *intor, const ExteriorOrientation *extor)
{
  erase(begin(), end());
  
  ObjectPoints::const_iterator i;
  for(i = objpts->begin(); i != objpts->end(); i++)
  {
	CameraPoint campt(&*i, intor, extor);
	push_back(campt);
  }  
}  	
   	
/*
--------------------------------------------------------------------------------
                 Construct camera points by transforming control points 
--------------------------------------------------------------------------------
*/


CameraPoints::CameraPoints(const ControlPoints *ctrlpts, const InteriorOrientation *intor, 
    			const ExteriorOrientation *extor)
 : VectorPoint<CameraPoint>()
{
    Control2CameraPoints(ctrlpts, intor, extor);
}    	

void CameraPoints::Control2CameraPoints(const ControlPoints *ctrlpts, 
	const InteriorOrientation *intor, const ExteriorOrientation *extor)
{
  erase(begin(), end());
 
  ControlPoints::const_iterator i;
  for(i = ctrlpts->begin(); i != ctrlpts->end(); i++)
  {
	CameraPoint campt(&*i, intor, extor);
	push_back(campt);
  }
}  	


/*
--------------------------------------------------------------------------------
                 Construct camera points by transforming image points 
--------------------------------------------------------------------------------
*/

CameraPoints::CameraPoints(const ImagePoints *imgpts, const InteriorOrientation *intor)
 : VectorPoint<CameraPoint>()
{
   Image2CameraPoints(imgpts, intor);
} 


void CameraPoints::Image2CameraPoints(const ImagePoints *imgpoints, 
	const InteriorOrientation *intor)
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  ImgPts *imgpts;	
  imgpts = NULL;
  imgpoints->Cpp2C(&imgpts);
  
  CamPts *campts = (CamPts *)malloc(sizeof(CamPts));
  campts->pts = (CamPt *)calloc(imgpoints->size(), sizeof(CamPt));

  // call library routine
  File_To_Metric(into, imgpts, campts);

  C2Cpp(campts);

  free(into);
  Free_CamPts(campts);
  Free_ImgPts(imgpts);  
}  	


/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void CameraPoints::Cpp2C(CamPts **camptsptr) const
{
  CamPts *campts;

// Allocate space if this has not been done yet 

  campts = *camptsptr;
  if (campts == NULL) {
    campts = (CamPts *) malloc(sizeof(CamPts));
    campts->pts = (CamPt*) malloc(size() * sizeof(CamPt));
    *camptsptr = campts;
  }

// Copy the data 

  campts->num_pts = size();
  CameraPoints::const_iterator i;
  CamPt *pt;
  int l = 0;
  for(i = begin(); i != end(); i++, l++)
  {
	pt = &campts->pts[l];
	i->Cpp2C(&pt);
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void CameraPoints::C2Cpp(CamPts *campts)
{
  if (!empty()) erase(begin(), end());
  reserve(campts->num_pts);
  	
  CameraPoint point; 	
  for(int i = 0; i < campts->num_pts; i++)
  {
  	point.C2Cpp(&campts->pts[i]);
  	push_back(point);
  }
}

/*
--------------------------------------------------------------------------------
                       Read camera points from a database
--------------------------------------------------------------------------------
*/
int CameraPoints::Read(char *filename)
{
  CamPts *campts;

  campts = Get_CamPts(filename);        // Read the database into C structure 
  if (campts == NULL) return(0);
  C2Cpp(campts);                        // Convert to C++ object 
  Free_CamPts(campts); 
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write camera points to a database
--------------------------------------------------------------------------------
*/
int CameraPoints::Write(char *filename) const
{
  int    error;
  CamPts *campts;

  campts = NULL;
  Cpp2C(&campts);
  error = Put_CamPts(campts, filename);
  Free_CamPts(campts); 
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print camera points to stdout
--------------------------------------------------------------------------------
*/
void CameraPoints::Print() const
{
  CamPts *campts;

  campts = NULL;
  Cpp2C(&campts);
  Print_CamPts(campts);
  Free_CamPts(campts); 
}


/*
--------------------------------------------------------------------------------
                        Transform camera points to image points
--------------------------------------------------------------------------------
*/

ImagePoints *CameraPoints::Camera2ImagePoints(const InteriorOrientation *intor) const
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  CamPts *campts;	
  campts = NULL;
  Cpp2C(&campts);
  
  ImgPts *imgpts = (ImgPts *)malloc(sizeof(ImgPts));
  imgpts->pts = (ImgPt *)malloc(size() * sizeof(ImgPt));
  strcpy(imgpts->img_name, "");

  // call library routine
  File_To_Pix(into, campts, imgpts);

  ImagePoints *imgpoints = new ImagePoints();
//  name = NULL;  
  imgpoints->C2Cpp(imgpts);

  free(into);
  Free_CamPts(campts);
  Free_ImgPts(imgpts);  
 
  return imgpoints;
}


