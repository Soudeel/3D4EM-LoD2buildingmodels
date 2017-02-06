
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



//---------------------------------------------------------------------------
//GeometricFittingLMQ.h
//
//Contains geometric fitting routines using Levenberg Marquardt method. The
//fitting engine is called from Numerical Recipes. As Numerical recipes version
//provides only for single argument passing scheme, the indices are passed to main
//routine, while data is accessed through a global pointer that is set at the start
//of particular function.
//
//Most of the fitting routines use orthogonal distance from the measured points
//to estimated geometry as a figure of merit. To stabilize the solution in most
//functions the direction-vector is normalized, in each call. The other implementation
//details are explained in individual function headers.
//
//The partial derivatives are calculated in most cases using MatLab's ccode() function.
//
//
// Author: Tahir Rabbani Shah
// Date: August 2002 (Delft)
//
//---------------------------------------------------------------------------
#ifndef __GEOMETRIC_FITTING_LMQ_H__
#define __GEOMETRIC_FITTING_LMQ_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "NRUtilities.h"
#define NRANSI
#include "Vector3D.h"

//Typedefinition of Levenberg-Marquardt callback.
typedef void (*LMQCallback)(double dX, double pParameterVector [], double* pFunctionValue, double pDerivativeVector [], int nVectorSize);

//Main Levenberg-Marquardt fitting routine.
// ppData: 				Matrix containing data. It is assumed the LMQ callback knows what to do with this
//						data once it knows the index of row to access.
// nRows:				Number of rows of ppData
// nCols:				Number of columns of ppData.
// pStdVector:			Standard deviation vector. size = nRows
// pConstraintVector:	If 1 parameter is fitted, for 0 it is fixed. size = nParameterVectorSize
// pChiSquare			Contains chisquare error on return.
// pMaxIterations:		Passes maximum iterations on entry, has actual iterations on exit.
// nNumberOfParams		Number of parameters in function to be fitted.
// pFittingCallback		Fitting callback function. Must update function value and its partial derivatives
//						for a given index in dX.
int FitLMQ(double** ppData,
				int nRows,
               	int nCols,
              	double* pStdVector,
             	int* pConstraintVector,
                double* pParameterVector,
                double* pChiSquare,
                int* pMaxIterations,
				int nNumberOfParams,
				LMQCallback pFittingCallback,
				const bool showDebugMessages = false);


//Fits a Torus to give XYZ coordinates in ppData.
//The number of parmeters is 8
// (P1,P2,P3) define a unit vector giving the axial direction of torus.
// (P4,P5,P6) give the centre point of torus.
// (P7) is the major radius.
// (P8) is the minor radius.
//
// ppData:				input X,Y,Z matrix.
// nRows:				Number of rows
// nCols: 				Number of columns, must be atleast 3.
// pStdVector:			Standard deviation of each measurement.
// pConstraintVector:   Must be at least 8x1. If 0 keep that param fixed.
// pParameterVector:	Seed values, must be 8x1.
// pChiSquare:			Has final chisquare on return.
// pMaxIteration:		Has allowed max iterations on entry, and has final iteration count
//						on return.
int FitTorusLMQ(double** ppData,
					int nRows,
                    int nCols,
                    double* pStdVector,
                    int* pConstraintVector,
                    double* pParameterVector,
                    double* pChiSquare,
                    int* pMaxIterations);


//Fit Cylinder using Levenberg-Marquardt method.
//The number of parameters is 7.
// (P1,P2,P3) define the unit vector along axis of cylinder.
// (P4,P5,P6) give the point of axis closest to origin.
// (P7) gives the radius of cylinder.
//
// ppData:				input X,Y,Z matrix.
// nRows:				Number of rows
// nCols: 				Number of columns, must be atleast 3.
// pStdVector:			Standard deviation of each measurement.
// pConstraintVector:   Must be at least 7x1. If 0 keep that param fixed.
// pParameterVector:	Seed values, must be 7x1.
// pChiSquare:			Has final chisquare on return.
// pMaxIteration:		Has allowed max iterations on entry, and has final iteration count
//
int FitCylinderLMQ(double** ppData,
					int nRows,
                    int nCols,
                    double* pStdVector,
                    int* pConstraintVector,
                    double* pParameterVector,
                    double* pChiSquare,
                    int* pMaxIterations);
//Fits a Sphere to give XYZ coordinates in ppData.
//The number of parmeters is 4
// (P1,P2,P3) define the centre of sphere.
// (P4) is the radius.
//
// ppData:				input X,Y,Z matrix.
// nRows:				Number of rows
// nCols: 				Number of columns, must be atleast 3.
// pStdVector:			Standard deviation of each measurement.
// pConstraintVector:   Must be at least 4x1. If 0 keep that param fixed.
// pParameterVector:	Seed values, must be 4x1.
// pChiSquare:			Has final chisquare on return.
// pMaxIteration:		Has allowed max iterations on entry, and has final iteration count
//	
int FitSphereLMQ(double** ppData,
					int nRows,
                    int nCols,
                    double* pStdVector,
                    int* pConstraintVector,
                    double* pParameterVector,
                    double* pChiSquare,
                    int* pMaxIterations);
		    


//Fit Line3D using Levenberg-Marquardt method.
//The number of parameters is 6.
// (P1,P2,P3) define the unit vector along line.
// (P4,P5,P6) give the point on axis closest to origin.
//
// ppData:				input X,Y,Z matrix.
// nRows:				Number of rows
// nCols: 				Number of columns, must be atleast 3.
// pStdVector:			Standard deviation of each measurement.
// pConstraintVector:   Must be at least 6x1. If 0 keep that param fixed.
// pParameterVector:	Seed values, must be 6x1.
// pChiSquare:			Has final chisquare on return.
// pMaxIteration:		Has allowed max iterations on entry, and has final iteration count
//
int FitLine3DLMQ(double** ppData,
					int nRows,
                    int nCols,
                    double* pStdVector,
                    int* pConstraintVector,
                    double* pParameterVector,
                    double* pChiSquare,
                    int* pMaxIterations);



//Fits an implicit function using least squares.
//The solution is calculated using eigen-vector fit method.
//ppData:				input X,Y,Z matrix.
//nRows:				number of rows.
//nCols:				number of columns.
//pAns:					Vector filled with answer on success. The ans corresponds to
//						minimum eigen-value.
//pChiSquare			If pAnsMax is NULL it can be address of single double
//						If pAnsMax is not NULL must be a vector of size 2.
//pAnMax:				Vector to be filled with eigen-vector corresponding to max eigen value.		    		    
//
int FitEig(double** ppData,
			int nRows,
			int nCols,
			double* pAns,
			double* pChiSquare = NULL,
			double* pAnsMax = NULL);
			
//Tests various eigen fitting routines for synthetic data.			
void TestEigFitting();

//Fits a biquadratic function to given data. The form of function is
// a1*x^2 + a2*y^2 + a3*x*y + a4*x + a5*y + a6*z + a7*d = 0
// Number of unknown parameters is 7.
// The function inputs are similar to FitEig function.
int FitBiquadraticEig(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);
//Fit using least squares. use explict form z=f(x,y)					
int FitBiquadraticLsq(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);					
					
//Fits a truncated biquadratic function to given data. The form of function is
// a1*x^2 + a2*y^2 + a3*x*y + a4*z + a5*d = 0
// Number of unknown parameters is 5.
// The function inputs are similar to FitEig function.
int FitTruncatedBiquadraticEig(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);
					
//Fit using least squares. use explict form z=f(x,y)	
//skip x, and y terms.				
int FitTruncatedBiquadraticLsq(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);										
					


//Fits a bicubic function to the given data. The form of the function is
// a1*x^3 + a2*y^3 + a3*x^2*y + a4*x*y^2 + a5*x^2 + a6*y^2 + a7*x*y + a8*x + a9*y + a10*z + a11 = 0;
// Number of unknown parameters is 11.
//The inputs are similar to FitEig function 					
int FitBicubicEig(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);
					
//Fits a plane to given data. The function is of the form:
// a1*x + a2*y + a3*z + a4 = 0
// Number of unknown parameters are 4.
// First three correspond to the normal to plane.					
int FitPlaneEig(double** ppData,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);
//This function is similar to FitPlaneEig, but also returns the other two axis along with 
int FitPlaneEig3(double** ppData,int nRows,int nCols,Vector3D& n1, Vector3D& n2, Vector3D& n3,
				double* e1=NULL, double* e2=NULL, double* e3=NULL);
					
					
//Fits a 2D line to the give XY data. Form of the function is:
// a1*x + a2*y + a3 = 0;
int FitLineEig(double** ppXY,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare=NULL);
					
//Fits a polynomial of given degree to x-y data.
//nDegree specifies the maximum power of fitted polynomial.
//If degree is n the fitted parameters are n+1
// a1*x^n + a2*x^(n-1) + ... + an*x + an+1 = 0
int FitPolyEig(double** ppXY,
					int nRows,
					int nCols,
					int nDegree,
					double* pAns,
					double* pChiSquare);
					
//Fits a quadratic surface to given XYZ points.
//Number of params is 10.
// a1*x^2 + a2*y^2 + a3*z^2 + a4*x*y + a5*x*z + a6*y*z + a7*x + a8*y + a9*z + a10 = 0											
int FitQuadraticSurfaceEig(double** ppXYZ,
					int nRows,
					int nCols,
					double* pAns,
					double* pChiSquare);
					
//Fits a Plane to the given n-D data and returns the n-Orthogonal Vectors.
//The solution is calculated using eigen-vector fit method.
//ppData:				input X,Y,Z matrix.
//nRows:				number of rows.
//nCols:				number of columns.
//ppEigenVectors		An nxn Matrix, to be filled with calculated orthogonal vectors.
//						The vectors are returned in the descending order of eigenValues.
//						In case of Plane, the first vector corresponds to plane normal (Direction of min Variance)
//pEigenValues			Corresponding eigenValues.
//
int 
FindOrthogonalVectors(double** ppData,
					int nRows,
					int nCols,
					double** ppEigenVectors,
					double* pEigenValues);
					
//Performs Principal component analysis of given nD data. The caculated components are returned in descending order.
//The solution is calculated using eigen-vector fit method.
//ppData:				input X,Y,Z matrix.
//nRows:				number of rows.
//nCols:				number of columns.
//ppProjectedData		ppData projected to calculated orthogonal axes. The maximum component is in first column.
int 
PerformPCA(double** ppData,
					int nRows,
					int nCols,
					double** ppProjectedData);
/*Calculates the rank of the given NR matrix using svd. If tol is
zero it is taken as max(rows*cols)*max_svd*eps. Rank equals the number
of svd's higher than this tol.
*/
int GetRank(double** a,int rows,int cols,double tol=0);					
					
/* Calculate determinant of fixed size matrices
*/
double Determinant2(double** m);
double Determinant3(double** m);
double Determinant4(double** m);

/* Find the roots of poly-nomial.
	polynomial[1] = dc-value;
	polynomial[2] = coeff of x
	polynomial[3] = coeff of x^2 
	.... and so on ...
*/
int FindPolyRoots(double* polynomial,int degree, double* realRoot,double* complexRoot);

/* Finds the eigen values and eigen vectors of a given matrix 
	Must be square paramsXparams
	Note that the eigen vectors are in the columns. 
	To access eigen vector1 iterate ppEigenVectors[1:params][1] and so on ...
*/
int FindEig(double** ppData,int params,double** ppEigVectors, double* pEigenValues);

		    
#undef NRANSI

#endif //__GEOMETRIC_FITTING_LMQ_H__
