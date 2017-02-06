
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
 Collection of functions for class ImagePoint
 
 ImagePoint& operator=(const ImagePoint& point)     Copy assignemnt    
 void ImagePoint::Cpp2C(ImgPt **)        Conversion of C++ class to C structure
 void ImagePoint::C2Cpp(ImgPt *)         Conversion of C structure to C++ class

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

#include <stdlib.h>
#include "ImagePoint.h"
#include "CameraPoint.h"
#include "InteriorOrientation.h"

extern "C" void Record_To_Pix(Interior *, CamPt *, ImgPt *);
extern "C" void Record_To_Metric(Interior *, ImgPt *, CamPt *);

/*
--------------------------------------------------------------------------------
                     		Constructor
--------------------------------------------------------------------------------
*/

ImagePoint::ImagePoint(const CameraPoint *campoint, const InteriorOrientation *intor)
 : ImagePointPlain(), Covariance2D()
{
   Camera2ImagePoint(campoint, intor);
}   
   
void ImagePoint::Camera2ImagePoint(const CameraPoint *campoint, 
	const InteriorOrientation *intor)
{
  Interior *into = (Interior*) malloc(sizeof(Interior));
  
  intor->Cpp2C(&into);	

  CamPt *campt;	
  campt = NULL;
  campoint->Cpp2C(&campt);

  ImgPt *imgpt = (ImgPt *)malloc(sizeof(ImgPt));
  
  // call library routine
  Record_To_Pix(into, campt, imgpt);

  C2Cpp(imgpt);  
  
  free(into);
  free(campt);  
  free(imgpt);
}


/*
--------------------------------------------------------------------------------
                     		Copy assignment
--------------------------------------------------------------------------------
*/

ImagePoint& ImagePoint::operator=(const ImagePoint& point)
{
  // Check for self assignment
  if (this == &point) return *this;
  x[0]   = point.x[0];
  x[1]   = point.x[1];
  num    = point.num;
  var[0] = point.var[0];
  var[1] = point.var[1];
  var[2] = point.var[2];
  return *this;
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ImagePoint::Cpp2C(ImgPt **imgptptr) const
{
  ImgPt *imgpt;

/* Allocate space if this has not been done yet */

  imgpt = *imgptptr;
  if (imgpt == NULL) {
    imgpt = (ImgPt *) malloc(sizeof(ImgPt));
    *imgptptr = imgpt;
  }

/* Copy the data from the C++ to the C object */

  imgpt->r     = x[0];
  imgpt->c     = x[1];
  imgpt->num   = num;
  imgpt->v_r   = var[0];
  imgpt->v_c   = var[1];
  imgpt->cv_rc = var[2];
}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ImagePoint::C2Cpp(ImgPt *imgpt)
{
  x[0]   = imgpt->r;
  x[1]   = imgpt->c;
  num    = imgpt->num;
  var[0] = imgpt->v_r;
  var[1] = imgpt->v_c;
  var[2] = imgpt->cv_rc;
}


/*
--------------------------------------------------------------------------------
                     Transform an image point to a camera point
--------------------------------------------------------------------------------
*/

CameraPoint *ImagePoint::Image2CameraPoint(const InteriorOrientation *intor) const
{
  Interior *into = (Interior*) malloc(sizeof(Interior));
   
  intor->Cpp2C(&into);	

  ImgPt *imgpt;	
  imgpt = NULL;
  Cpp2C(&imgpt);

  CamPt *campt = (CamPt *)malloc(sizeof(CamPt));
  
  // call library routine
  Record_To_Metric(into, imgpt, campt);

  CameraPoint *campoint = new CameraPoint();
  campoint->C2Cpp(campt);  
  
  free(into);
  free(campt);  
  free(imgpt);
  
  return campoint;
}



    
