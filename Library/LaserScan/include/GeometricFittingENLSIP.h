
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Fitting of geometric objects using ENLSIP
*
*--------------------------------------------------------------------*/
#ifndef _GEOMETRIC_FITTING_ENLSIP_H_
#define _GEOMETRIC_FITTING_ENLSIP_H_

#include "ENLSIPFitting.h"
class LaserPoints;

#define SHOW_DEBUG 0
class ENLSIPCylinder: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPCylinder(const LaserPoints* pPts = NULL);
		
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();

protected: 
	const LaserPoints *pLaserPoints;
};



//Class for torus fitting.
class ENLSIPTorus: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPTorus(const LaserPoints* pPts = NULL);
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();

protected: 
	const LaserPoints *pLaserPoints;
};


//Class for torus fitting.
class ENLSIPPlane: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPPlane(const LaserPoints* pPts = NULL);
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();

protected: 
	const LaserPoints *pLaserPoints;
};


//Class for torus fitting.
class ENLSIPSphere: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPSphere(const LaserPoints* pPts = NULL);
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();
	
protected: 
	const LaserPoints *pLaserPoints;
};



//Class for Fixed Point Correspondence based registration.
//Each point in scan1 is supposed to be the corresponding point for scan2.
class ENLSIPRegistrationFixed: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPRegistrationFixed(const LaserPoints* _pPtsFixed = NULL, const LaserPoints* _pPtsMoving = NULL);
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();
	
protected: 
	const LaserPoints *pPtsFixed;
	const LaserPoints *pPtsMoving;
};



//Class for ICP based registration
class ENLSIPRegistrationICP: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPRegistrationICP(const LaserPoints* _pPtsFixed = NULL, const LaserPoints* _pPtsMoving = NULL,
					double _percentageThreshold=100,double _distanceThreshold=-1,bool _useNormls=false);
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	///A static function for executing the test.
	static void ExecuteTest();
	
protected: 
	const LaserPoints *pPtsFixed;
	const LaserPoints *pPtsMoving;
	double percentageThreshold;
	double distanceThreshold;
	bool useNormals;
};




#endif //_GEOMETRIC_FITTING_ENLSIP_H_
