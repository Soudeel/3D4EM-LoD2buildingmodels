
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



// LSAdjust.cpp: implementation of the LSAdjust class.
//
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include "LSAdjust.h"

const bool  AnalyzeResults = true;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

LSAdjust::LSAdjust()
{
    m_EPS = 1.e-6;
    FullReset(); // Initialise
}

LSAdjust::~LSAdjust(){}

//----------------------------------------------------------------------------
bool LSAdjust::Generate_Normal_Equations()
{
    int i, k;
    
    // Generate upper triangular elements Process ATA matrix
    if(m_A.empty() || m_F.empty()) 
        return false;

    int n_rows = m_A.size();
    if (n_rows == 0 || n_rows != m_F.size())
        return false;
    int n_cols = m_A.begin()->size();
    if (n_cols == 0)
        return false;

    
    m_ATA = m_A.MultTranspose(&m_W);
    
    
    m_ATF.reserve(n_cols);
    double sum = 0.0;
    for(i = 0; i < n_cols; i++){
        for(k = 0; k < n_rows; k++)
            sum += m_A[k][i] * m_F[k] * m_W[k];
        m_ATF.push_back(sum);
        sum = 0.0;
    }

    m_nObs = m_A.size();
    for (vRecord::iterator wPtr = m_W.begin(); wPtr != m_W.end(); wPtr++)
        if (*wPtr == 0)
            m_nObs--;
    m_obsNorm = m_F.NormSquare(&m_W);
    return true;
}

//----------------------------------------------------------------------------
bool LSAdjust::Update_Normal_Equations(vRecord &recA, double obs, double w)
{
    // ATA() - is a square symmetric matrix
    // addRecA() - the additional record of the matrix A
    // obs - is the observation corresponding to equation addRecA
    // w - is the weight of obs
    if (recA.empty())
        return false;

    if (m_ATF.empty()){
        m_ATF = vRecord(recA.size(), 0.);
        m_nObs = 0;
        m_obsNorm = 0.;
    }

    if (m_ATA.empty())
        m_ATA.Define(recA.size(), recA.size());
    else if (m_ATA.size() != recA.size() || 
        m_ATA.begin()->size() != m_ATA.size() || m_ATA.size() != m_ATF.size())
        return false;

    int i, j;

    // Process ATA matrix and ATF vector
    for(i = 0; i < m_ATA.size(); i++){
        m_ATF[i] += recA[i] * obs * w;
        for(j = 0; j < m_ATA.size(); j++)
            m_ATA[i][j] += recA[i] * recA[j] * w;
    }
    m_nObs++;
    m_obsNorm += obs * obs * w;

    m_isSynchronized = false;

    return true;
}

//----------------------------------------------------------------------------
void LSAdjust::AddRecord(vRecord &Arecord, double obs, double w, bool _UpdateNormEq)
{
    // Add record to m_matrix
    m_A.push_back(Arecord);
    m_F.push_back(obs);
    m_W.push_back(w);

    m_isSynchronized = false;
    
    if (_UpdateNormEq)
        Update_Normal_Equations(Arecord, obs, w);
    else
	m_nObs++;
}

//----------------------------------------------------------------------------
void LSAdjust::Reset()
{
    m_first_record = true;
    m_ATA = 0.;//.Erase();
    m_ATF = 0.; //erase(m_ATF.begin(), m_ATF.end());
    m_nObs = 0;
    m_obsNorm = 0.;
    m_N1 = 0;
	m_std = -1;
	
    m_isSynchronized = true;

}

//----------------------------------------------------------------------------
void LSAdjust::FullReset()
{
    m_first_iteration = true;
    m_iteration = 1; // Current iteration
    m_first_record = true;
    m_nObs = 0;

    m_obsNorm = 0.;
    m_std = m_medianRes = -1.;
    
    m_F.Erase();
    m_ATF.Erase();
    m_W.Erase();
    m_SolnX.Erase();
    m_Resd.Erase();

    m_A.Erase();
    m_ATA.Erase();
    m_N1.Erase();
    m_Corr.Erase();
    tmpMat.Erase();

    m_isSynchronized = true;

}

//----------------------------------------------------------------------------
void LSAdjust::CleanAfter()
{
    m_F.Erase();
    m_W.Erase();
    m_Resd.Erase();
    m_A.Erase();
    m_ATA.Erase();
    m_Corr.Erase();
    tmpMat.Erase();
}
//----------------------------------------
vector < vector <double> > LSAdjust::exportATA()
{
printf("exportATA: normal equations are explicitly computed here! So only use AddRecord to fill A Matrix!\n");

Generate_Normal_Equations();

vector < vector <double> > ret;
ret.resize(m_ATA.size());

for (int r=0; r<m_ATA.size();r++)
{
	ret[r].resize(m_ATA.size());
	for (int c=0;c<m_ATA.size();c++)
		ret[r][c]=m_ATA[r][c];
}

return ret;
}
//----------------------------------------------------------------------------
bool LSAdjust::Adjust(bool AnalyzeResults, vector < vector <double> > *Ninv)
{
    //Computing the solution after forming the normal equations
    int i ;
    int rankDef; // needed for call to InvertBySVD

    if ((m_ATA.empty() || m_ATF.empty()) && (m_A.empty() || m_F.empty()))
        return false;
    
    if (m_ATA.empty() || m_ATF.empty())
        Generate_Normal_Equations();

    int n_prms  = m_ATA.size();

    //printf("DEBUG: Amatrix:\n");
    //m_A.Print(" %.3f ");


    //printf("DEBUG: ATA matrix:\n");
    //m_ATA.Print(" %.3f ");

if (Ninv==NULL)
{
    if (n_prms == 3){
        if (!m_ATA.Invert_3x3(m_N1, m_EPS)){
			m_std = -1;
	    	printf("Adjustment failed\n");
            return false;
        }
    }
    else if (!m_ATA.InvertBySVD(m_N1, rankDef, m_EPS)){
//      else if (!m_ATA.Invert(m_N1,STANDARD_INVERSION, m_EPS)){
		m_std = -1;
		printf("Adjustment failed\n");
        return false;
    }
}

else 
{
printf("Using Ninv from external source!\n");
  m_N1=vMatrix(m_ATA.size(),m_ATA.size());
	for (int r=0; r<m_ATA.size();r++)
	{
		for (int c=0;c<m_ATA.size();c++)
			{
			  m_N1[r][c]=(*Ninv)[r][c];
			  //printf(" %.4f",m_N1[r][c]);
			}
	//printf("\n");
	}

}

   printf("size of m_N1=%d\n",m_N1.size());
   printf("size of m_ATF=%d\n",m_ATF.size());

    m_SolnX = m_N1 * m_ATF;

    assert(m_SolnX.size() == m_N1.size());

    m_first_iteration = false; m_iteration++; 

    m_std = ComputeSTD();

    m_isSynchronized = true;

    if (!AnalyzeResults || Sigma() < 0.)
        return true;

    //Computing the residuals vector
    
    assert (!m_A.empty() && !m_F.empty() && m_A.size() == m_F.size() && 
                        m_A.begin()->size() == m_SolnX.size());

    long n_obs = m_A.size();

    m_Resd = m_A * m_SolnX - m_F;
    for(i = 0; i < n_obs; i++)
        if (m_W[i] == 0) m_Resd[i] = 0.;
    
    m_medianRes = m_Resd.Median();

    // Computing the correlation matrix
    if (!ComputeCorrelation())
        return false;

    return true;
}

double LSAdjust::resNorm() const
{
  return m_obsNorm - m_ATF.DotProduct(m_SolnX); 
}

// return the residual cofactor matrix
vMatrix LSAdjust::ResidualCofactors()const
{
  vMatrix Qvv; 
  vMatrix tmp; 
  tmp.Transpose( m_A ); 
  tmp *= -1.0;
  Qvv = m_A*m_N1; 
  Qvv = Qvv*tmp; 
  for( int i=0; i<m_W.size(); i++ )
    Qvv[i][i] += 1.0/m_W[i];
  return Qvv; 
}

// return the normalized residuals (residual/(sigma0a-post*root(residual cofactor)))
vRecord LSAdjust::NormalizedResiduals()const
{
  vMatrix Qvv = ResidualCofactors(); 
  vRecord normResid( m_Resd ); 
  if( m_std > 0.0 )
    for( int i=0; i<m_Resd.size(); i++ )
      normResid[i] /= (m_std*sqrt( Qvv[i][i] )); 
  return normResid; 
}

// print the A matrix, the observations and the weights
void LSAdjust::PrintInput( ostream& OUTPUT )const
{
  OUTPUT << "design matrix" << std::endl
         << m_A
         << "observations" << std::endl; 
  m_F.WriteToStream( OUTPUT ); 
  OUTPUT << "weights (obsevation cofactor inverse)" << std::endl; 
  m_W.WriteToStream( OUTPUT ); 
}

bool LSAdjust::AddLSsystem(LSAdjust &v)
{
    assert (!m_ATA.empty() || !m_A.empty());
    assert (!v.ATA().empty() || !v.A().empty());

    if (m_ATA.empty() || v.ATA().empty()){
        for (int i = 0; i < v.A().size(); i++)
            AddRecord(v.A()[i], v.Observation()[i], v.Weight()[i]);
        Adjust(AnalyzeResults);
    }
    else{
        assert(m_ATA.size() == v.ATA().size());
        ATA() += v.ATA();
        ATF() += v.ATF();
        obsNorm() += v.obsNorm();
        nObs() += v.nObs();
        Adjust();
        
    }
    return true;
}

LSAdjust LSAdjust::MergeLSsystems(LSAdjust &v)
{
    if ((m_ATA.empty() && m_A.empty()) || (v.ATA().empty() && v.A().empty()) 
            || m_ATA.size() != v.ATA().size())
        return LSAdjust();

    LSAdjust mergedLS(*this);
    if (m_ATA.empty() || v.ATA().empty()){
        for (int i = 0; i < v.A().size(); i++)
            mergedLS.AddRecord(v.A()[i], v.Observation()[i], v.Weight()[i]);
        mergedLS.Adjust(AnalyzeResults);
        return mergedLS;
    }
    else{       
        mergedLS.ATA() += v.ATA();
        mergedLS.ATF() += v.ATF();
        mergedLS.obsNorm() += v.obsNorm();
        mergedLS.nObs() += v.nObs();
        mergedLS.Adjust();
        
        return mergedLS;
    }
}

bool LSAdjust::SequentialLS_add(vRecord &a, double y, double w)
{
    assert (!m_N1.empty() && !a.empty() && w == 1);

    m_ATF += a * y;

    vRecord _a(a);
    vRecord v(a * m_N1);
//  m_N1.Print();
//  a.Print("a");
//  v.Print("v");

    double value = 1./(1. + a.DotProduct(v));
    v *= -value;

    tmpMat.CreateFromOuterProduct(&a, &v);
    tmpMat.IncrementDiagonal(1.);

    _a *= value * (y - a.DotProduct(m_SolnX));
    m_SolnX += m_N1 * _a;
    m_N1 *= tmpMat;

    m_obsNorm += y * y * w;
    m_nObs++;

    ComputeSTD();

    m_isSynchronized = true;

    return true;
}

bool LSAdjust::SequentialLS_remove(vRecord &a, double y, double w)
{
    assert(!m_N1.empty() && !a.empty() && w == 1);

    m_ATF -= a * y;

    vRecord _a(a);
    vRecord v(a * m_N1);

    double value = 1./(1. - a.DotProduct(v));
    v *= value;

    tmpMat.CreateFromOuterProduct(&a, &v);
    tmpMat.IncrementDiagonal(1.);

    _a *= value * (y - a.DotProduct(m_SolnX)); //constant
    m_SolnX -= m_N1 * _a;
    m_N1 *= tmpMat;

    m_obsNorm -= y * y * w;
    m_nObs--;

    ComputeSTD();

    m_isSynchronized = true;

    return true;
}

int LSAdjust::NumOfUnknown() const
{
    if (!m_ATA.empty())
        return m_ATA.size();
        
    if(!m_A.empty())
        return m_A.begin()->size();
       
    return -1;
}

double LSAdjust::Residual(const vRecord &a, double y)
{
    assert(m_isSynchronized);
    return y - m_SolnX.DotProduct(a);
} 
   
double LSAdjust::ComputeSTD()
{
    double eTe = m_obsNorm - m_ATF.DotProduct(m_SolnX);
    double redundancy = double(m_nObs - m_ATF.size());
    
    if (redundancy == 0){
        m_std = 0;
        return m_std;
    }
//  assert (eTe > -1.e-7);
    if (eTe < -1.e-5){      
        return -1;  
    }

    if (fabs(eTe) < 1.e-7)
        m_std = 0.;
    else if (eTe < 0 || redundancy < 0)
        m_std = -1;
    else if (redundancy == 0)
        m_std = 0.;
    else
        m_std = sqrt(eTe/redundancy);
    return m_std;
}

bool LSAdjust::ComputeCorrelation()
{
    m_Corr = m_N1; 
    int n_prms = m_N1.size();
    for (int i = 0; i < n_prms; i++)
        for (int j = 0; j < n_prms; j++){
            if (m_N1[i][i] * m_N1[j][j] > 0)
                m_Corr[i][j] /= sqrt(m_N1[i][i] * m_N1[j][j]);
            else
                return false;
        }
    return true;
}
