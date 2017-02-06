
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
*   File made : July 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Provides functions for fitting different geometric entities to LaserPoints.
*
*--------------------------------------------------------------------*/
#ifndef _LASERPOINTS_FITTING_H_
#define _LASERPOINTS_FITTING_H_
#include "LaserPoints.h"
#include "LaserPointsUtility.h"
#include "LaserPointsProcessing.h"
#include "LaserObjects.h"

/** Approximate cylinder using plane fitting on Gaussian sphere.
	The steps of algorithm are as follows:
	-Estimate normals using kNN
	-Fit plane to normals plotted in 3D (Find normal direction of great circle on gaussian sphere)
	-normal of plane approximates cylinder axis.
	-point closest to origin is approximated by laser point mean
	-radius is given a constant value (its linear if other six parameter are approximately right)
*/	
LaserCylinder 
ApproximateCylinder(
			LaserPoints& laserPoints, int kNN=20 );



///Fit cylinder using least squares.
LaserCylinder FitCylinder(LaserPoints& laserPoints, const LaserCylinder& approximate);

/** Fits Plane to given laser points using PCA of covariance matrix.
	
*/	
LaserPlane FitPlane(LaserPoints& laserPoints);

///Calculates the convex hull of the given set of points.
int
GetConvexHull(const LaserPoints& laserPoints,
			Vector3D& normal,
			Positions3D& positions);

/** Approximate a sphere to given laser points.
	Position is approximated by taking mean.
*/	
LaserSphere ApproximateSphere(const LaserPoints& laserPoints);

/** Fit sphere to laser points */
LaserSphere FitSphere(LaserPoints& laserPoints, const LaserSphere& approximate);

/** Fits torus to given laser points.
	Currently no approximation method is available, we make a number of iterations
	and increase the number of free parameters incrementally. 
	TO DO: Add a method for torus approximation.
*/	
	
LaserTorus ApproximateTorus(LaserPoints& laserPoints);

/** Fit a torus starting from the given approximation */
LaserTorus FitTorus(LaserPoints& laserPoints, const LaserTorus& approximate);

///Find R and T so that:
///Y = R*X + T
void FindTransformation(const LaserPoints& X, const LaserPoints& Y,	Rotation3D& rot, Vector3D& trans);

///Y is fixed and X is being transformed
void RegisterUsingICP(const LaserPoints& X, const LaserPoints& Y,
			Rotation3D& rot, Vector3D& trans,int maxIter,
			double percentageThreshold,bool useNormals,int kNN);
///Y is fixed and X is being transformed
void RegisterUsingICP(const LaserPoints& X, const LaserPoints& Y,
			Orientation3D *orient,int maxIter,double percentageThreshold,bool useNormals,int kNN);

///Find transformation between two point clouds.
void FindTransformation(const LaserPoints& X, const LaserPoints& Y,
			Orientation3D* orient);

///Fit plane using eigen analysis of precomputed summation values.
Vector3D FindPlane(const double x2,const double y2,const double z2,
				const double xy, const double xz, const double yz,
				const double x, const double y, const double z,
				const int n, double &rho,double* pResidual = 0);
				
///Fit cylinder using RANSAC.
int FitCylinderRANSAC(
			LaserPoints& laserPointsIn,
			IndicesVector& selectedIndices,
			Vector3D& in_point1,
			Vector3D& in_point2,
			double& in_dRadius,
			Vector3D& in_axis,
			Vector3D& in_pointOnAxis,
			double &in_dChiSquare, 
			bool doApproximation=true,
			int kNN=10,
			int nMaxIterations=10,
			double percentagePoints=10,
			double ransacIterations=10);				

#endif //_LASERPOINTS_FITTING_H_


