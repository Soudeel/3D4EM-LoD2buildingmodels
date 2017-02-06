
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



/*!
 * \file
 * \brief Interface to Class LSAdjust - Least Squares Adjustment
 *
 */
/*!
 * \class LSAdjust
 * \ingroup Photogrammetry
 * \brief Interface to Class LSAdjust - Least Squares Adjustment
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \S_Filin
 * \date		---- (Created)
 *
 * \remark \li LSAdjust
 *   is a class for least squares adjustment. The residuals and the length of the unknown vector are minimized.
 *   It has the design matrix (aka observation equation matrix) A and the vector of (reduced) observations F.
 *   Weights are considered in the adjustment, correlation between the observations is not supported. The weights are stored in a vector W.
 *   Clients of this class also have access to the normal equation matrix, its inverse, the unknowns, their correlations, the transformed observations, and the residual vector
 *   ATA = A(t)*W*A
 *   N1 = ( A(t)*W*A )^-1
 *   SolnX = ( A(t)*W*A )^-1 * A(t)*w*F
 *   Corr = ( A(t)*W*A )^-1 devided appropriately through the precision of the unknowns
 *   ATF = A(t)*W*F
 *   Resd = A * SolnX - F
 *   furthermore, the client has access to the median of the absolute residuals
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

/** \example LSAdjustExample.cc
 * This is an example of how to use the LSAdjust class.
 * More details about this example.
 */
 
#ifndef _LSADJUST_
#define _LSADJUST_

#include <math.h>
#include "vMatrix.h"


/*
    LSAdjust
    is a class for least squares adjustment.
    The residuals and the length of the unknown vector are minimized.
    It has the design matrix (aka observation equation matrix) A
    and the vector of (reduced) observations F.
    Weights are considered in the adjustment, correlation between the
    observations is not supported. The weights are stored in a vector W.
    Clients of this class also have access to the normal equation matrix,
    its inverse, the unknowns, their correlations, the transformed observations,
    and the residual vector
    ATA = A(t)*W*A
    N1 = ( A(t)*W*A )^-1
    SolnX = ( A(t)*W*A )^-1 * A(t)*w*F
    Corr = ( A(t)*W*A )^-1 devided appropriately through the precision of the unknowns
    ATF = A(t)*W*F
    Resd = A * SolnX - F
    furthermore, the client has access to
    the median of the absolute residuals
*/

class LSAdjust
{
public:
    /// Constructor
    LSAdjust();

    /// Destructor
    virtual ~LSAdjust();

    /// should be protected
    /// Generate the normal equations from the internal matrices m_A, m_F, m_W
    bool Generate_Normal_Equations();

    /// update system of normal equations with one observation equation
    /// ( recA * unknonws = obs ; with weight w )
    bool Update_Normal_Equations(vRecord &recA, double obs, double w);

    /// Number of unknowns required for a Least squares solution
    int NumOfUnknown()const;

    /// Add a record to a matrix
    void AddRecord(vRecord &A_record, double y, double w, bool _UpdateNormEq = false);

    /// Prepare for first iteration.
    /// - Incremental Solution
    void FullReset();

    /// Prepare for subsequent iteration
    void Reset();

    /// Deleting after adjustment\analysis information like A, F, ATA ATF etc.
    /// and leaving only statistical info - N1, correlation, std., and median
    void CleanAfter();

    /// Export the NormalMatrix
    vector < vector <double> > exportATA();

    /// Adjust using least squares
    bool Adjust(bool AnalyzeResults = false, vector < vector <double> > *Ninv =NULL);

    /// add another adjustment (with the same unknowns) to *this and
    /// return it as a new adjustment
    /// should be const !!!!!!!!111
    LSAdjust MergeLSsystems(LSAdjust &) ;

	/// adds the adjustmentsystem v to the current one
    bool AddLSsystem(LSAdjust &v);

    /// add one equation to the adjustment system
    /// add one observation equation to the system and solve it
    /// a = partial derivatives, y = observation, w = weight
    bool SequentialLS_add(vRecord &a, double y, double w=1.);

    /// remove one equation from the adjustment system
    /// see also the comment on SequantielLS_add
    bool SequentialLS_remove(vRecord &a, double y, double w=1.);

    
    /// compute the standard deviation from
    /// sigma^2 = ( obsNorm - SolnX(t) A(t) P F ) / redundancy
    double ComputeSTD();

    /// compute the correlation matrix from the inverted normal equation matrix
    /// return false, if the computation failed due to numerical problems (div by zero)
    bool ComputeCorrelation();

    /// return the A matrix, the client may change the values
    /// of the partial derivatives (i.e. the elements in A).
    /// However, if the size is changed an inconsitency betweem
    /// W and F is introduces
    vMatrix & A() {return m_A;}

    /// return the observation vector F.
    /// set the comment on returning A
    vRecord & Observation() {return m_F;}

    /// return the weight "matrix" W (i.e. the vector of diagonal elements)
    vRecord & Weight() {return m_W;}

    /// return the unknowns, assert for no empty vector
    const vRecord & Solution() const {assert(!m_SolnX.empty()); return m_SolnX;}
    vRecord & Solution() {assert(!m_SolnX.empty()); return m_SolnX;}
	
	/// Return true is there is a solution false if not
	bool isSolution() {return !(m_SolnX.empty());}

    /// return the residual vector
    const vRecord & Residuals() const {return m_Resd;}
    
    /// returns the residual for a given observation 
    /// e = y - a*Solution()
    double Residual(const vRecord &a, double y);

    /// return the normal equation matrix
    const vMatrix & ATA() const {return m_ATA;}
    vMatrix & ATA() {return m_ATA;}
   
    /// return the normal observations
    const vRecord & ATF() const {return m_ATF;}
    vRecord & ATF() {return m_ATF;}

    /// return the inverse of the normal equation
    const vMatrix & N1() const {return m_N1;}

    /// return the correlation matrix for the unknowns
    const vMatrix & Correlation() const { return m_Corr; }

    /// return the residual cofactor matrix
    vMatrix ResidualCofactors()const; 

    // return the normalized residuals (residual/(sigma0a-post*root(residual cofactor)))
    vRecord NormalizedResiduals()const; 

    /// return the obsNorm ( = Ft W F )
    double obsNorm() const {return m_obsNorm;}

    /// return the resNorm ( = Rt W R, R for residual vector )
    double resNorm() const; 

    /// Returning the Standard deviation function
    double Sigma() const { return m_std;}

    /// return the number of observations
    int nObs() const {return m_nObs;}

    /// return the redundancy of the adjustment ( degrees of freedom )
    int DegreesOfFreedom() const {return m_nObs - NumOfUnknown();}
 
    int DOF()const {return DegreesOfFreedom();}    

    /// return the median of the absolute values of the residuals
    double MedianRes() const { return m_medianRes; }
    double MedianDev() const { return MedianRes(); }

    double & obsNorm() {return m_obsNorm;}
    int & nObs() {return m_nObs;}
    
    /// set a threshold for considering the normal matrix singular.
    /// If the smallest eigenvalue is smaller than EPS, the system is
    /// considered singular. 
    void SetEPS(double _eps){ m_EPS = _eps;}

    /// return the threshold on the smallest eigenvalue of the normal
    /// equation matrix that triggers singularity.
    double EPS(){return m_EPS;}
    
    /// print the design matrix, the observation vector and the weights
    void PrintInput( ostream& OUTPUT )const; 

protected:
    /// Flag - indicates whether or not it is the first iteration
    bool m_first_iteration;

    /// Flag - indicates whether or not it is the first record
    bool m_first_record;

    /// The index of the current iteration
    int m_iteration;

    /// First Design matrix
    vMatrix m_A;

    /// Vector of observations
    vRecord m_F;

    /// Vector of weights
    vRecord m_W;

    /// ATA matrix
    vMatrix m_ATA;

    /// ATF vector
    vRecord m_ATF;

    /// Cofactor matrix
    vMatrix m_N1;

    /// Correlation matrix     
    vMatrix m_Corr;

    /// Solution of Least Squares adjustment after adjustment
    vRecord m_SolnX;

    /// Residuals vector
    vRecord m_Resd;

    /// Median of the residuals
    double m_medianRes;

    /// Std of a unit weight
    double m_std;

    /// FtPF used for computing the std without generating residual vector
    double m_obsNorm;

    /// FtPF used for computing the std without generating residual vector
    int m_nObs;
    
    /// Defining the sensitivity of the solution
    double m_EPS;
    
    /// status used for sequential adjustment
    bool m_isSynchronized;
 

private:
    vMatrix tmpMat; // This matrix can save redfinition of matrices
};

/*******************************************************************************
/ 
/    input struct   A = |m_ATA   m_Q|
/                       |m_Q^T   m_S|  --> S is a sparse matrix
/ 
/   output struct A1  = |m_N1      m_Qtild|
/                       |m_Q^Ttild m_Stild|  --> S is a sparse matrix
/ 
/  Implementation from numerical recipes
/ 
/
*******************************************************************************/
class SparseLS :public LSAdjust
{

public:

    SparseLS():isStild(false){}

    bool InvertSparseSystem();
    bool UpdateNormalEquations(const vRecord &, const vRecord &, int, double, double = 1.);
    bool SolveSparseSystem();
    void set_isStild() {isStild = true;}
    bool isStild_set() {return isStild;}
    bool isSet() {return (m_ATA.empty())? true : false;}
    void Print();
    int  nSparseBlocks() {return S.size();}
    void Reset();

    bool SetSparseSystem(int nPrms, int nObjects, int nItemPerObject);
    template <class Type> bool SetSparseSystem(int nPrms, Type & ObjList)
    {
//      Type::iterator obj;
        int total_nPrms = nPrms;
        int nObj = ObjList.size();
        int i;
        S.reserve(nObj);

        m_ATA.Define(nPrms, nPrms);
//      vMatrix tmp;
        for (i = 0; i < ObjList.size(); i++){
            S.push_back(vMatrix());
            S[i].Define(ObjList[i].nParams(), ObjList[i].nParams());
            int t = S[i].size();
            S[i].Print();
            total_nPrms += ObjList[i].nParams();
        }
        Q.Define(nPrms, total_nPrms - nPrms); //#block&strip Prms, #surf prms
        m_ATF = vRecord (total_nPrms);
        return true;
    }
protected:

    bool isStild;
    vMatrix Q;
    std::vector<vMatrix> S;
    vMatrix Qtild;
    vMatrix Stild;
};


#endif
