
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
 Collection of functions for class CameraPoint

 CameraPoint& operator=(const CameraPoint& point)     Copy assignemnt    
 void CameraPoint::Cpp2C(CamPt **)      Conversion of C++ class to C structure
 void CameraPoint::C2Cpp(CamPt *)       Conversion of C structure to C++ class

 Initial creation
 Author : George Vosselman
 Date   : 21-07-1998

 Update #1
 Author :Adam Patrick Nyaruhuma
 Date   :25-03-2009
 Changes: - C code changed to C++ for constructor object to camera point,
          - added method Camera2ObjectPoint - given object height
          - added method CameraPoint2ObjectLine - given orientation parameters
          - added method Camera2ObjectPoint - given plane containing object point

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "CameraPoint.h"
#include "InteriorOrientation.h"
#include "ExteriorOrientation.h"
#include "ObjectPoint.h"
#include "ControlPoint.h"
#include "ImagePoint.h"
#include "Plane.h"


extern "C" int lobj2cam(ObjPt *, Interior *, Exterior *, CamPt *);
extern "C" void Record_To_Metric(Interior *, ImgPt *, CamPt *);
			
/*
--------------------------------------------------------------------------------
                 Construct camera point by transforming an object point 
--------------------------------------------------------------------------------
*/


CameraPoint::CameraPoint(const ObjectPoint *objpoint, const InteriorOrientation *intor, 
    			const ExteriorOrientation *extor) :  Point2D()
{       
   double cc, xdiff, ydiff, zdiff, denominator;

   xdiff = objpoint->X() - extor->X();
   ydiff = objpoint->Y() - extor->Y();
   zdiff = objpoint->Z() - extor->Z();
   denominator = extor->R(0,2) * xdiff + 
                 extor->R(1,2) * ydiff + 
                 extor->R(2,2) * zdiff;
   
   if (!denominator)
   {
      printf("obj2cam: devide by zero!\n");
      exit (0);
   }
                        
   Number() = objpoint->Number();
   
   cc = intor->CameraConstant();
   X() = -cc * (extor->R(0,0) * xdiff + 
                     extor->R(1,0) * ydiff +
                     extor->R(2,0) * zdiff) / denominator;
   Y() = -cc * (extor->R(0,1) * xdiff + 
                     extor->R(1,1) * ydiff +
                     extor->R(2,1) * zdiff) / denominator;
}   

    	
/*
--------------------------------------------------------------------------------
                 Construct camera point by transforming a control point 
--------------------------------------------------------------------------------
*/


CameraPoint::CameraPoint(const ControlPoint *ctrlpoint, const InteriorOrientation *intor, 
    			const ExteriorOrientation *extor)
{
   Control2CameraPoint(ctrlpoint, intor, extor);
}   	


void CameraPoint::Control2CameraPoint(const ControlPoint *ctrlpoint, 
	const InteriorOrientation *intor, const ExteriorOrientation *extor)
{
  Exterior *exto = (Exterior*) malloc(sizeof(Exterior));
  Interior *into = (Interior*) malloc(sizeof(Interior));

  extor->Cpp2C(&exto);	
  intor->Cpp2C(&into);	

  CamPt *campt;
  campt = (CamPt*) malloc(sizeof(CamPt));
  
  ObjectPoint objpoint(ctrlpoint);
  ObjPt *objpt;
  objpt = NULL;
  objpoint.Cpp2C(&objpt);
  
  lobj2cam(objpt, into, exto, campt);
  C2Cpp(campt); 
  
  free(exto);
  free(into);
  free(campt);
  free(objpt);
}  


/*
--------------------------------------------------------------------------------
                 Construct camera point by transforming an image point 
--------------------------------------------------------------------------------
*/


CameraPoint::CameraPoint(const ImagePoint *imgpoint, const InteriorOrientation *intor)
{
   Image2CameraPoint(imgpoint, intor);
}   


void CameraPoint::Image2CameraPoint(const ImagePoint *imgpoint, 
	const InteriorOrientation *intor)
{
  Interior *into = (Interior*) malloc(sizeof(Interior));
  
  intor->Cpp2C(&into);	

  CamPt *campt;
  campt = (CamPt*) calloc(1, sizeof(CamPt));
  ImgPt *imgpt;
  imgpt = NULL;
  imgpoint->Cpp2C(&imgpt);
  
  Record_To_Metric(into, imgpt, campt);
  
  C2Cpp(campt); 
  
  free(into);
  free(campt);
  free(imgpt);
}  
    			


/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

CameraPoint& CameraPoint::operator=(const CameraPoint& point)
{
  // Check for self assignment
  if (this == &point) return *this;
  x[0]   = point.x[0];
  x[1]   = point.x[1];
  num    = point.num;
  var[0] = point.var[0];
  var[1] = point.var[1];
  var[2] = point.var[2];
  return(*this);
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void CameraPoint::Cpp2C(CamPt **camptptr) const
{
  CamPt *campt;

/* Allocate space if this has not been done yet */

  campt = *camptptr;
  if (campt == NULL) {
    campt = (CamPt *) malloc(sizeof(CamPt));
    *camptptr = campt;
  }

/* Copy the data from the C++ to the C object */

  campt->x     = x[0];
  campt->y     = x[1];
  campt->num   = num;
  campt->v_x   = var[0];
  campt->v_y   = var[1];
  campt->cv_xy = var[2];
}


/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void CameraPoint::C2Cpp(CamPt *campt)
{
  x[0]   = campt->x;
  x[1]   = campt->y;
  num    = campt->num;
  var[0] = campt->v_x;
  var[1] = campt->v_y;
  var[2] = campt->cv_xy;
}

/*
--------------------------------------------------------------------------------
                     Object point from camera point with given height
--------------------------------------------------------------------------------
*/


ObjectPoint CameraPoint::Camera2ObjectPoint(const InteriorOrientation *intor,
                          const ExteriorOrientation *extor,const double height)
    {
    double
    cc = intor->CameraConstant(),
    X0 = extor->X(),
    Y0 = extor->Y(),
    Z0 = extor->Z(),
                               
    denominator = extor->R(2,0) * X() +
                extor->R(2,1) * Y() - 
                extor->R(2,2) * cc;
    
    if (!denominator)
    {
     printf("camera_to_obj: devide by zero!\n");
     exit (0);
    } 
    ObjectPoint* objpt=new ObjectPoint();
    objpt->Number() = Number();
    
    objpt->Z() =height;
    objpt->X() = X0 +(objpt->Z()-Z0)* (extor->R(0,0) * X() + 
                    extor->R(0,1)* Y() -
                    extor->R(0,2) * cc) / denominator;
                  
    objpt->Y() = Y0 +(objpt->Z()-Z0)* (extor->R(1,0) *X() + 
                    extor->R(1,1) * Y() -
                    extor->R(1,2) * cc) / denominator;
    return *objpt;
    }

/*
--------------------------------------------------------------------------------
                     3D line of in object space of a camera point
--------------------------------------------------------------------------------
*/


Line3D CameraPoint::CameraPoint2ObjectLine(const InteriorOrientation *into,
                                   const ExteriorOrientation *extor)
    {
    double height=0;
    //obtain a 3d point
    ObjectPoint temp=Camera2ObjectPoint(into,extor,height);
    // Construct line from two positions temp and projection centre
	Position3D projection_centre(extor->X(),extor->Y(),extor->Z());
    Line3D line(projection_centre,temp);
    return line;
    }
 
 /*
--------------------------------------------------------------------------------
                     Object point from camera point, given object plane
--------------------------------------------------------------------------------
*/
ObjectPoint* CameraPoint::Camera2ObjectPoint(const InteriorOrientation *into,
                                   const ExteriorOrientation *extor, const Plane *plane)                                   
    {
    Line3D line=CameraPoint2ObjectLine(into,extor);
    ObjectPoint *objpt=new ObjectPoint();
    if (!IntersectLine3DPlane (line, *plane, *objpt)){
         printf("Camera2ObjectPoint: No plane line intersection!\n");
         exit (0);
         }
    objpt->Number()=Number();
    return objpt;
    }

