/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to encapsulate ENLSIP, Constrained non-linear least squares
*				base on Guass-Newton method
*
*--------------------------------------------------------------------*/
#ifndef _ENLSIP_FITTING_H___
#define _ENLSIP_FITTING_H___

#include <math.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
#include <include.h> // 	to access just the compiler options
#include <newmat.h> // 	to access just the main matrix library (includes include.h)
#include <newmatio.h>



using std::vector;
using std::cerr;
using std::endl;
using std::string;
class NEWMAT::Matrix;

//This is the base class from which all ENLSIP fitting functions should derieve.
//Contains the interface and basic data elements.
class ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPFitting(int _paramCount,int _obsCount,int _eqCount=0,int _ineqCount=0,
		int _maxIter = -1, bool _internalScaling=1);
	
	///Optimize and return the final parameters.
	///Fill the other variables if necessary.
	virtual vector<double> Optimize(vector<double>& initialValues,
						double * chiSquare = NULL,
						string* errorMessage = NULL,
						int* errorCode=NULL);
						
	///const version of the above function
	virtual vector<double> Optimize(const vector<double>& initialValues_in,
						double * chiSquare = NULL,
						string* errorMessage = NULL,
						int* errorCode=NULL)
	{
		return Optimize(const_cast<vector<double>& > (initialValues_in),
					chiSquare, errorMessage, errorCode);
	}
						
	///Optimize and return the final parameters.
	///Fill the other variables if necessary.
	vector<double> Optimize(vector<double>& initialValues,
						int& iterationsUsed,
						int& functionEvaluationsUsed,
						int& jacobiansUsed,
						double * chiSquare = NULL);						
						
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double *x,double* f,int count);
	
	///For returning the evaluated function in a vector.
	std::vector<double> EvaluateFunction(const std::vector<double>& x);
	
	///For returning the evaluated constraint in a vector.
	std::vector<double> EvaluateConstraint(const std::vector<double>& x);
	
	///For returning the evaluated equality constraint in a vector.
	std::vector<double> EvaluateEqConstraint(const std::vector<double>& x);
	
	///For returning the evaluated equality constraint in a vector.
	std::vector<double> EvaluateIneqConstraint(const std::vector<double>& x);
	
	///This function differentiates the function values for all points.
	///return zero to switch to internal finite-differences.
	virtual bool DifferentiateFunction(double *x, double* f,int count);
		
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double *x, double* f,int count);
		
	///This function differentiates the constraint function.
	///return zero to switch to internal finite-differences.
	virtual bool DifferentiateConstraint(double *x, double* f,int count);
	
	virtual ~ENLSIPFitting();
	
	///Functions for the calculation of the covariance matrices.
	void Covariance(vector<double> x, double sigmaObs,
				NEWMAT::Matrix* covariance,  NEWMAT::Matrix* covObs, NEWMAT::Matrix* covConstraints);
	
	
	///static function working as function callback.
	static int FunctionCallback(double *x, int *n, double *f, int 
	*m, int *ctrl, double *c__, int *mdc);
	///static function working as constraint callback.
	static int ConstraintCallback(double *x, int *n, double *h, 
	int *l, int *ctrl, double *a, int *mda);
		
	///read function for param count.
	int ParamCount() const;
		
	///read function for obsCount.
	int ObsCount() const;
	
	///Set obsCount.
	void SetObsCount(int newCount)
	{
		obsCount = newCount;
	}
	
	///Set iterationCount.
	void SetMaxIter(int newCount)
	{
		maxIter = newCount;
	}
	
	///read Function for eqCount.
	int EqCount() const;
	
	///Set eq count.
	void SetEqCount(int n)
	{
		eqCount = n;
	}
		
	///read function for ineqCount.
	int IneqCount() const;
	
	///write function for ineqCount.
	void SetIneqCount(int n) 
	{
		ineqCount = n;
	}
	
	///Set the number of parameters.
	void SetParamCount(int n)
	{
		paramCount = n;
	}
	
		
	///read function for maxIter.
	int MaxIter() const;
	
	///read function for internal scaling.
	bool InternalScaling() const;
	
	///write function for internal scaling.
	void SetInternalScaling(bool n) 
	{
		internalScaling = n;
	}
		
	///read function for static current object.
	static ENLSIPFitting* ActiveObject();
	
	///increment function counter
	void IncrementFunctionCounter() ;
	
	//increment constraint counter
	void IncrementConstraintCounter();

	///Read the counters.
	int ConstraintCounter() const;
	
	///Read the counters.
	int FunctionCounter() const;
	
	///Set finite diff step size.
	void SetFiniteDiffStepSize(double d);
	
	///Set finite diff step size.
	double GetFiniteDiffStepSize()const;
	
	///Make this object active.
	void SetActive();
	
	
protected:
	///Number of parameters.
	int paramCount;
	
	///Number of observations.
	int obsCount;
	
	///Number of equality Constraints.
	int eqCount;
	
	///Number of in-equality constraints.
	int ineqCount;
	
	///Number of max iterations allowed.
	///If -1 just use the default value.
	int maxIter;
	
	///If 1 use internal scaling, if 0 don't use internal scaling.
	bool internalScaling;
	
	///internal counters for keeping track of function calls.
	int functionCounter;
	
	///internal counter for keeping track of constraint calls.
	int constraintCounter;
	
	///step size for finite differencing.
	double finiteDiffStepSize;
	
	///static pointer to keep "this" while we work with optimization routines.
	///its necessary to have it here if we don't want to change every occurance 
	///of function in the code
	static ENLSIPFitting* pActiveObject;
};


//Just a test class. Demonstrates how to derieve for custom fitting.
class ENLSIPTest: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPTest():ENLSIPFitting(3,20,1,1)
	{
	
	}
	
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count)
	{
		cerr<<"Calling ENLSIPTest::EvaluateFunction \n";
		
		double sumSq = 0;
		for(int i=0;i<count;i++)
		{
			f[i] = x[0]*pow((double)i,2) + x[1]*pow((double)i,3) + x[2]*pow(i,2.5) - 
				(2*pow((double)i,2) + 30*pow((double)i,3) + 6*pow(i,2.5));
			sumSq += pow(f[i],2);
			//printf("%d: %f\n",i,f[i]);
		}
		// printf("Sumsq: %f \n",sumSq);		
		return 0;
	}
	
	///This function differentiates the function values for all points.
	///return zero to switch to internal finite-differences.
	virtual bool DifferentiateFunction(double* x, double* f,int count)
	{
		cerr<<"Calling ENLSIPTest::DifferentiateFunction \n";
		return 0;
	}
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count)
	{
		cerr<<"Calling ENLSIPTest::EvaluateConstraint with count: "<< count <<"\n";
		f[0] = x[2] - 6;
		f[1] = x[1] + 2000;
		f[2] = -x[1] + 2000;
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
		
		return 0;
	}
	
	///This function differentiates the constraint function.
	///return zero to switch to internal finite-differences.
	virtual bool DifferentiateConstraint(double* x,double* f,int count)
	{
		cerr<<"Calling ENLSIPTest::DifferentiateConstraint \n";
		return 0;
	}
	
	virtual ~ENLSIPTest()
	{
		cerr<<"~ENLSIPTest()\n";
	}
	
	///A static function for executing the test.
	static void ExecuteTest()
	{
		cerr<<"Calling the ENLSIPTest::ExecuteTest()\n";
		
		//make a temporary optimizer object
		ENLSIPTest test;
		
		//Initialize.
		vector<double> initial(3,1);
		initial[0] = 2;
		initial[1] = 2;
		initial[2] = 4;
		
		//Opitmize.
		vector<double> final = test.Optimize(initial);
		
		//Print final result.
		for(int i=0;i<final.size();i++)
			cerr<<"Final: "<<i<<": "<<final[i]<<endl;
	}
};

#endif //_ENLSIP_FITTING_H___		 
	

