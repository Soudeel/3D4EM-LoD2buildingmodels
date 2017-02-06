
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
 Collection of functions for class ImagePoints          

 void     ImagePoints::Cpp2C(ImgPts **)      Conversion of C++ to C object
 void     ImagePoints::C2Cpp(ImgPts *)       Conversion of C to C++ object
 int      ImagePoints::Read(char *)          Read image points from a database
 int      ImagePoints::Write(char *)         Write image points to a database
 void     ImagePoints::Print()               Print image points to stdout
 
 Initial creation
 Author : George Vosselman
 Date   : 21-07-1998

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

#include <string.h>
#include "ImagePoints.h"
#include "CameraPoints.h"
#include "InteriorOrientation.h"

/*
--------------------------------------------------------------------------------
                     Declarations of old DigPhot functions in C
--------------------------------------------------------------------------------
*/

extern "C" void File_To_Pix(Interior *, CamPts *, ImgPts *);
extern "C" void File_To_Metric(Interior *, ImgPts *, CamPts *);

/*
--------------------------------------------------------------------------------
                     Constructors 
--------------------------------------------------------------------------------
*/

void ImagePoints::Camera2ImagePoints(const CameraPoints *campoints, 
	const InteriorOrientation *intor)
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  CamPts *campts;	
  campts = NULL;
  campoints->Cpp2C(&campts);
  
  ImgPts *imgpts = (ImgPts *)malloc(sizeof(ImgPts));
  imgpts->pts = (ImgPt *)malloc(campoints->size() * sizeof(ImgPt));
  strcpy(imgpts->img_name, "");

  // call library routine
  File_To_Pix(into, campts, imgpts);

  name = NULL;  
  C2Cpp(imgpts);

  free(into);
  Free_CamPts(campts);
  Free_ImgPts(imgpts);  
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ImagePoints::Cpp2C(ImgPts **imgptsptr) const
{
  ImgPts *imgpts;   
  imgpts = *imgptsptr;	

// Allocate space 
 
 if (imgpts == NULL)
 {
 	imgpts = (ImgPts*) malloc(sizeof(ImgPts));
 	if (imgpts == NULL)
 		printf("Error malloc\n");
 	imgpts->pts = (ImgPt*) malloc(size()*sizeof(ImgPt));
 	if (imgpts->pts == NULL)
 		printf("Error malloc 2\n");
 	*imgptsptr = imgpts;
 }
 

// Copy the data 

  if (name) strcpy(imgpts->img_name, name);
  else imgpts->img_name[0] = 0;
  imgpts->num_pts = size();

  ImagePoints::const_iterator i;
  ImgPt *pt;
  int l;
  for(i = begin(), l = 0; i != end(); i++, l++)
  {
 	pt = &imgpts->pts[l];
 	i->Cpp2C(&pt); 	
  } 
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ImagePoints::C2Cpp(ImgPts *imgpts)
{
  if (!empty()) erase(begin(), end());
  reserve(imgpts->num_pts);
  //if (name) delete [] name;	

  name = new char[strlen(imgpts->img_name) + 1];
  strcpy(name, imgpts->img_name);
 
  ImagePoint point; 	
  for(int i = 0 ; i < imgpts->num_pts; i++)
  {
  	point.C2Cpp(&imgpts->pts[i]);
  	push_back(point);
  } 
}

/*
--------------------------------------------------------------------------------
                       Read image points from a database
--------------------------------------------------------------------------------
*/

int ImagePoints::Read(char *filename)
{
  ImgPts *imgpts;

  imgpts = Get_ImgPts(filename);        // Read the database into C structure 
  if (imgpts == NULL) 
  	return(0);  	
  	
  C2Cpp(imgpts);                        // Convert to C++ object 
  Free_ImgPts(imgpts);  
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write image points to a database
--------------------------------------------------------------------------------
*/

int ImagePoints::Write(char *filename) const
{
  int    error;
  ImgPts *imgpts; 
  
  imgpts = NULL;  
  Cpp2C(&imgpts);  
  error = Put_ImgPts(imgpts, filename);  
  Free_ImgPts(imgpts);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print image points to stdout
--------------------------------------------------------------------------------
*/

void ImagePoints::Print() const
{ 
  ImgPts *imgpts;
  
  imgpts = NULL;
  Cpp2C(&imgpts);
  Print_ImgPts(imgpts);
  Free_ImgPts(imgpts);	
}

/*
--------------------------------------------------------------------------------
                        Transform image points to camera points
--------------------------------------------------------------------------------
*/


CameraPoints *ImagePoints::Image2CameraPoints(const InteriorOrientation *intor) const
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  ImgPts *imgpts;	
  imgpts = NULL;
  Cpp2C(&imgpts);
  
  CamPts *campts = (CamPts *)malloc(sizeof(CamPts));
  campts->pts = (CamPt *)calloc(size(), sizeof(CamPt));

  // call library routine
  File_To_Metric(into, imgpts, campts);

  CameraPoints *campoints = new CameraPoints();
  campoints->C2Cpp(campts);

  free(into);
  free(campts);
  Free_ImgPts(imgpts);  
  
  return campoints;
}  	

