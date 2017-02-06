#include "ENLSIPFitting.h"
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <include.h> // 	to access just the compiler options
#include <newmat.h> // 	to access just the main matrix library (includes include.h)
#include <newmatap.h> // 	to access the advanced matrix routines such as Cholesky decomposition, QR triangularisation etc (includes newmat.h)
#include <newmatio.h> // 	to access the output routines (includes newmat.h) You can use this only with compilers that support the standard input/output routines including manipulators
#include "ShMacro.h"



#include "f2c.h"

#define SHOW_DEBUG 0 //Define it 1 for seeing detailed debug messages
#define MAX(a,b) (((a)>(b))?(a):(b))
using namespace std;
using namespace NEWMAT;	

extern "C"  
{
	#include "f2c.h"
	int enlsip_(doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, U_fp, U_fp, integer *,
	     doublereal *, integer *, doublereal *, doublereal *, integer *);
};
extern double _enlsip_finite_diff_step_size;


//The static pointer has to be defined outside class declaration.
ENLSIPFitting* ENLSIPFitting::pActiveObject = NULL;

//Constructor.
ENLSIPFitting::ENLSIPFitting(int _paramCount,int _obsCount,int _eqCount,int _ineqCount,
	int _maxIter, bool _internalScaling)
	:paramCount(_paramCount), obsCount(_obsCount), eqCount(_eqCount),ineqCount(_ineqCount),
	maxIter(_maxIter), internalScaling(_internalScaling)
{
	finiteDiffStepSize = 1e-6;
}


///Optimize and return the final parameters.
	///Fill the other variables if necessary.
vector<double> ENLSIPFitting::Optimize(vector<double>& initialValues,
						int& iterationsUsed,
						int& functionEvaluationsUsed,
						int& jacobiansUsed,
						double * chiSquare)	
{						
	SetActive();
	functionCounter = constraintCounter = 0;
	_enlsip_finite_diff_step_size = finiteDiffStepSize;

	/* System generated locals */
	int exitCode, *wint;
	double *f, *h__;
	int i, l, m, n, p;
	double *x, *wreal;
	int *active;
	static int mdi, mdr, bnd;

	if(SHOW_DEBUG)
		cerr<<"Starting ENLSIP::Optimize....\n";

	//MakeData();

	n = paramCount;
	m = obsCount;
	p = eqCount;
	l = 2*ineqCount+eqCount;
	bnd = l+n;
	
	if(SHOW_DEBUG)
	{
		cerr<<"paramCount: "<<paramCount<<endl;
		cerr<<"eqCount: "<<eqCount<<endl;
		cerr<<"ineqCount: "<<ineqCount<<endl;
		cerr<<"bnd: "<<bnd<<endl;
	}
	mdi = (2*(l+bnd)+n+13);
	mdr = MAX((n*(m+2*n+l+6)+3*(m+2*l)+4*bnd+7),mdr = (n*(m+n+l+6)+3*(m+2*l)+4*bnd+7));

	wint = (int*)malloc(sizeof(int)*mdi);
	wreal = (double*)malloc(sizeof(double)*mdr);
	x = (double*)malloc(sizeof(double)*n);
	f = (double*)malloc(sizeof(double)*(m+l));
	h__ = (double*)malloc(sizeof(double)*l);
	active = (int*)malloc(sizeof(int)*(bnd+l));

	/*     SET THE FIRST ELEMENTS OF THE ARRAYS WINT AND WREAL TO */
	/*     SOMETHING NEGATIVE TO INDICATE THAT DEFAULT VALUES SHOULD */
	/*     BE USED */

	for (i = 1; i <= 6; ++i) 
	{
		wint[i - 1] = -1;
	}

	//number of iterations.
	if(maxIter>1)wint[2] = maxIter;

	//For internal data scaling
	wint[4] = internalScaling;
	
	//Do we want to allow 2nd derivative.
	wint[5] = 0;

	for (i = 1; i <= (l+7); ++i) 
	{
		wreal[i - 1] = -1.f;
	}
	double eps = 1e-6;
	
	for(int i=0;i<5;i++)
		wreal[i] = eps;

	for(i=0;i<paramCount;i++)
		x[i] = initialValues[i];


	/*     INVOKE ENLSIP TO MINIMIZE */
	if(SHOW_DEBUG)
	{
		for(i=0;i<n;i++)
			printf("Initial x[%d]: %f\n",i,x[i]);
	}
	
	enlsip_((doublereal*)x, (integer*)&n, (integer*)&m, (integer*)&p, (integer*)&l, (integer*)&mdi, (integer*)&mdr, 
	//(U_fp)fexTest,
	(U_fp)(ENLSIPFitting::FunctionCallback),
	(U_fp)(ENLSIPFitting::ConstraintCallback), 
	(integer*)wint, (doublereal*)wreal, (integer*)&exitCode, (doublereal*)f, (doublereal*)h__, (integer*)active);

	if(SHOW_DEBUG)
	{
		for(i=0;i<n;i++)
			printf("Final x[%d]: %f\n",i,x[i]);
	}
	
	
	cerr<<"NUMBER OF ITERATIONS UNTIL TERMINATION: "<<wint[6]<<endl;
	
	if(SHOW_DEBUG)
	{
		cerr<<"TOTAL NUMBER OF FUNCTION EVALUATIONS: "<<wint[7]<<endl;
		cerr<<"NUMBER OF JACOBIAN EVALUATIONS: "<<wint[8]<<endl;
		cerr<<"NO. OF FUNCTION EVALUATIONS CAUSED BY THE 2ND DERIVATIVE: "<<wint[9]<<endl;
		cerr<<"NO. OF FUNCTION EVALUATIONS DONE BY THE LINESEARCH ALGORITHM: "<<wint[10]<<endl;
		cerr<<"THE ESTIMATED PSEUDO RANK OF THE CONSTRAINT MATRIX AT THE TERMINATION POINT: "<<wint[11]<<endl;
		cerr<<"ESTIMATED PSEUDO RANK OF COMPOUND MATRIX (A) AT THE TERMINATION POINT: "<<wint[12]<<endl; 
		cerr<<"VALUE OF THE OBJECTIVE FUNCTION AT THE TERMINATION POINT: "<<wreal[5]<<endl;
		cerr<<"AN ESTIMATE OF THE CONVERGENCE FACTOR (SHOULD BE <=1): "<<wreal[6]<<endl;
		
		for(int i=7;i<(l+7);i++) 
		{
			cerr<<"    Constraint "<<(i-6)<<" weight: "<<wreal[i]<<endl;
		}
		cerr<<endl;
	}
	
	iterationsUsed = wint[6];
	functionEvaluationsUsed = wint[7];
	jacobiansUsed = wint[8];
	
	if(chiSquare)
		*chiSquare = 2*wreal[5];

	//copy the final results.
	vector<double> results(x,x+paramCount);
	
	//Show covariance.
	if(1)
	{
		NEWMAT::Matrix covariance, covObs, covConstraints;
		this->Covariance(results, 1, &covariance, &covObs, &covConstraints); 
		
		for(int i=0;i<results.size();i++)
		{
			cerr<<"Std[" <<i<<"]: "<<setw(10)<<sqrt(fabs(covariance(i+1,i+1)))<<"only obs: "<<setw(10)<< sqrt(fabs(covObs(i+1,i+1)))<<endl;		
		}
	}

	//free allocated resources.
	free(wint);
	free(wreal);
	free(x);
	free(f);
	free(h__);
	free(active);

	if(SHOW_DEBUG)
	{
		cerr<<"Finished ENLSIP::Optimize!!!\n";
	}
	return results;
}

///Optimize and return the final parameters.
///Fill the other variables if necessary.
vector<double> ENLSIPFitting::Optimize(vector<double>& initialValues,
					double * chiSquare,
					string* errorMessage,
					int* errorCode)
{
	int iter, funx, jacob;
	return 	ENLSIPFitting::Optimize(initialValues, iter,funx,jacob,chiSquare);
} 


///This function calculates the function values for all points.
bool ENLSIPFitting::EvaluateFunction(double *x,double* f,int count)
{
	cerr<<"ENLSIPFitting::EvaluateFunction doesn't do anything\n";
	return 0;
}

///This function differentiates the function values for all points.
///return zero to switch to internal finite-differences.
bool ENLSIPFitting::DifferentiateFunction(double *x, double* f,int count)
{
	//cerr<<"ENLSIPFitting::DifferentiateFunction doesn't do anything\n";
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPFitting::EvaluateConstraint(double *x, double* f,int count)
{
	cerr<<"ENLSIPFitting::EvaluateConstraint doesn't do anything\n";
	//Note how to specify inequality constraints.
	//Each ineq has two cosecutive locations.
	//For example to say that x>20 and x<100, we must say
	//f[i] = x-20;
	//f[i+1] = -x+100;
	//
	return 0;
}

///This function differentiates the constraint function.
///return zero to switch to internal finite-differences.
bool ENLSIPFitting::DifferentiateConstraint(double *x, double* f,int count)
{
	//cerr<<"ENLSIPFitting::DifferentiateConstraint doesn't do anything\n";
	return 0;
}

ENLSIPFitting::~ENLSIPFitting()
{

}

///static function working as function callback.
int ENLSIPFitting::FunctionCallback(double *x, int *n, double *f, int 
*m, int *ctrl, double *c__, int *mdc)
{

	ENLSIPFitting* active = ENLSIPFitting::ActiveObject();
	if (abs(*ctrl) != 1) 
	{
		if(!(active->DifferentiateFunction(x,f,*m)))
		{
			*ctrl = 0;
			if(SHOW_DEBUG)
				cerr<<"Turning finite difference on for FunctionCallback\n";
		}
		return 0;
	}
	active->EvaluateFunction(x,f,*m);
	active->IncrementFunctionCounter();;
	return 0;
}

///static function working as constraint callback.
int ENLSIPFitting::ConstraintCallback(double *x, int *n, double *h, 
int *l, int *ctrl, double *a, int *mda)
{
	ENLSIPFitting* active = ENLSIPFitting::ActiveObject();
	if (abs(*ctrl) != 1) 
	{
		if(!(active->DifferentiateConstraint(x,h,*l)))
		{
			*ctrl = 0;
			if(SHOW_DEBUG)
				cerr<<"Turning finite difference on for Constraint Callback\n";
		}
		return 0;
	}
	else
	{
		active->EvaluateConstraint(x,h,*l);
		active->IncrementConstraintCounter();
		return 0;
	}
}

///read function for param count.
int ENLSIPFitting::ParamCount() const
{
	return paramCount;
}

///increment function counter
void ENLSIPFitting::IncrementFunctionCounter() 
{
	functionCounter++;
}

//increment constraint counter
void ENLSIPFitting::IncrementConstraintCounter() 
{
	constraintCounter++;
}

///Read the counters.
int ENLSIPFitting::ConstraintCounter() const
{
	return constraintCounter;
}

///Read the counters.
int ENLSIPFitting::FunctionCounter() const
{
	return functionCounter;
}
///read function for obsCount.
int ENLSIPFitting::ObsCount() const
{
	return obsCount;
}

///read Function for eqCount.
int ENLSIPFitting::EqCount() const
{
	return eqCount;
}

///read function for ineqCount.
int ENLSIPFitting::IneqCount() const
{
	return ineqCount;
}

///read function for maxIter.
int ENLSIPFitting::MaxIter() const
{
	return maxIter;
}

///read function for internal scaling.
bool ENLSIPFitting::InternalScaling() const
{
	return internalScaling;
}

///read function for static current object.
ENLSIPFitting* ENLSIPFitting::ActiveObject() 
{
	return pActiveObject;
}

///Set finite diff step size.
void ENLSIPFitting::SetFiniteDiffStepSize(double d)
{
	finiteDiffStepSize = fabs(d);
}
	
///Set finite diff step size.
double ENLSIPFitting::GetFiniteDiffStepSize()const
{
	return finiteDiffStepSize;
}	
	
///Make this object active.
void ENLSIPFitting::SetActive()
{
	pActiveObject = this;
}


/*
int main (void)
{

	ENLSIPTest::ExecuteTest();
}
*/
	
///For returning the evaluated function in a vector.
std::vector<double> ENLSIPFitting::EvaluateFunction(const std::vector<double>& x)
{
	double *pX = new double[x.size()];
	double *pValue = new double[obsCount];
	
	std::copy(x.begin(),x.end(),pX);
	
	this->EvaluateFunction(pX,pValue,obsCount);
	
	vector<double> values(obsCount);
	std::copy(pValue,pValue+obsCount,values.begin());

	//Free the resources before returning.	
	delete[] pX;
	delete[] pValue;
	
	//Now return with the vector.
	return values;
}

///For returning the evaluated constraint in a vector.
std::vector<double> ENLSIPFitting::EvaluateConstraint(const std::vector<double>& x)
{
	//total constraints.
	int count = eqCount+2*ineqCount;
	double *pX = new double[paramCount];
	double *pValue = new double[count];
	
	std::copy(x.begin(),x.end(),pX);
	
	this->EvaluateConstraint(pX,pValue,obsCount);
	
	vector<double> values(count);
	std::copy(pValue,pValue+count,values.begin());

	//Free the resources before returning.	
	delete[] pX;
	delete[] pValue;
	
	//Now return with the vector.
	return values;
}

///For returning the evaluated equality constraint in a vector.
std::vector<double> ENLSIPFitting::EvaluateEqConstraint(const std::vector<double>& x)
{
	std::vector<double> values = EvaluateConstraint(x);
	values.resize(eqCount);
	return values;
}

///For returning the evaluated equality constraint in a vector.
std::vector<double> ENLSIPFitting::EvaluateIneqConstraint(const std::vector<double>& x)
{
	std::vector<double> all = EvaluateConstraint(x);
	
	//Select ineqs.
	std::vector<double> values(2*ineqCount);
	copy(all.begin()+eqCount,all.end(),values.begin());
	
	return values;
}
	
static NEWMAT::Matrix SvdInv(const NEWMAT::Matrix& A,double threshold=1e-16)
{
	// SVD gives A  = U * D * V.t()
	int m = A.Nrows();                     // number of rows
    int n = A.Ncols();                     // number of columns
	NEWMAT::Matrix U, V;
	NEWMAT::DiagonalMatrix D;
	
	SVD(A, D, U, V); 
	
	double mx = D(1);
	for(int i=1;i<=m;i++)
	{
		if(fabs(D(i)/mx) > threshold)
			D(i) = 1.00/D(i);
		else
			D(i) = 0;
	}
	
	return V*D*U.t();
}


///Functions for the calculation of the covariance matrices.
void ENLSIPFitting::Covariance(vector<double> x, double sigmaObs, 
	NEWMAT::Matrix* covariance, NEWMAT::Matrix* covObs, NEWMAT::Matrix* covConstraints)
{
	const double stepSize = finiteDiffStepSize;
	const double svdThreshold = 1e-12;
	//For observations we have.
	//B * x = (function)
	
	//For Covariance we have.
	//A * x = (function)
	
	//Fill in the jacobian matrix for the observations.
	//The partial derivatives will be evaluated numerically.
	
	NEWMAT::Matrix B(obsCount,paramCount);
		
	vector<double> vOrig = EvaluateFunction(x);
	
	for(int i=0;i<x.size();i++)
	{
		vector<double> temp = x;
		temp[i] = x[i] + MAX(fabs(x[i])*stepSize,stepSize);
		
		vector<double> vTemp = EvaluateFunction(temp);

		double diff = temp[i] - x[i];
		//cerr<<"Step size of "<<i<<"-th parameter is "<<diff<<endl;
				
		for(int j=0;j<obsCount;j++)
		{
			//cerr<<"B has "<<B.Nrows()<<"  "<<B.Ncols()<<flush<<endl;
			B(j+1,i+1) = (vTemp[j] - vOrig[j])/diff;
		}
	}
	
	///The covariance calculation.
	NEWMAT::Matrix BtBInv = SvdInv(B.t() * B,svdThreshold);
	NEWMAT::Matrix CovO = BtBInv * pow(sigmaObs,2);
	
	if(!eqCount)
	{
		if(covariance)
			*covariance = CovO;

		if(covObs)
			*covObs = CovO;
		
		//Return with whatever we have, nothing to do for constraints.	
		return;
	}
	
	//Now for constraints if we have some.
	NEWMAT::Matrix C(eqCount,paramCount);
	vector<double> cOrig = EvaluateEqConstraint(x);
	
	for(int i=0;i<x.size();i++)
	{
		vector<double> temp = x;
		temp[i] = x[i] + MAX(fabs(x[i])*stepSize,stepSize);
				
		vector<double> cTemp = EvaluateEqConstraint(temp);

		double diff = temp[i] - x[i];
				
		for(int j=0;j<eqCount;j++)
			C(j+1,i+1) = (cTemp[j] - cOrig[j])/diff;
	}
	
	//cerr<<"C Matrix: \n"<<C<<endl;
	///The covariance calculation.
	NEWMAT::Matrix CovC = C.t()*SvdInv(C*BtBInv*C.t(),svdThreshold)*C*BtBInv;
	
	NEWMAT::Matrix CovFull = CovO*(IdentityMatrix(CovC.Nrows())-CovC); 
	
	if(covariance)
		*covariance = CovFull;
		
	if(covObs)
		*covObs = CovO;
		
	if(covConstraints)
		*covConstraints = IdentityMatrix(CovC.Nrows())-CovC;
}
	
