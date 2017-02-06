
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

#include "LSAdjust.h"

bool SparseLS::SetSparseSystem(int nPrms, int nObjects, int nItemPerObject)
{
	int total_nPrms = nPrms;
	S.reserve(nObjects);
	
	m_ATA.Define(nPrms, nPrms);
	for (int i = 0; i < nObjects; i++){
		S.push_back(vMatrix());
		S[i].Define(nItemPerObject, nItemPerObject);
		total_nPrms += nItemPerObject;
	}
	Q.Define(nPrms, total_nPrms - nPrms); //#block&strip Prms, #surf prms
	m_ATF.Define(total_nPrms);
	return true;
}

void SparseLS::Reset()
{
	m_ATA = 0.;
	Q = 0.;
	m_ATF = 0.;
	std::vector<vMatrix>::iterator mtrx;
	for (mtrx = S.begin(); mtrx < S.end(); mtrx++){
		*mtrx = 0.;
	}
//	m_N1.Erase();
//	Qtild.Erase();
//	Stild.Erase();

}
bool SparseLS::InvertSparseSystem()
{
	vMatrix QS1;
	vMatrix tPtild = m_ATA;
	vMatrix tmp;
	std::vector<vMatrix> Sinv(S.size());
	std::vector<vMatrix>::iterator matrix;
	std::vector<vMatrix>::iterator InvMatrix;
	int deficitR;

	if (S.empty()){
		m_ATA.InvertBySVD(m_N1, deficitR);
		return true;
	}
	
	for (matrix = S.begin(), InvMatrix = Sinv.begin(); 
			matrix != S.end(); matrix++, InvMatrix++){
		matrix->Invert(*InvMatrix, SVD_INVERSION);
	}

	if (!QS1.SparseMultiplication(Q, Sinv))
		return false;
	tmp = QS1.MultByTranspose(NULL, &Q); //SHOULD BE TRANSPOSED
	tPtild -= tmp;
	tPtild.InvertBySVD(m_N1, deficitR);
	Qtild.Multiply(m_N1, QS1);
	if (isStild){
		vMatrix tmp(QS1.Transpose_MultBy(NULL, &Qtild));
		Stild.SparseAddition(tmp, Sinv);
	}
	Qtild *= -1.;

	Sinv.erase(Sinv.begin(), Sinv.end());
	return true;
}

bool SparseLS::UpdateNormalEquations(const vRecord &p, const vRecord &s, 
									 int itemID, double y, double w)
{
	
	int i;
	if (p.size() != m_ATA.size() || itemID > S.size()) //also checkes initialization
		return false;

	int index = 0;
	for (i = 0; i < itemID; i++) index += S[i].size();

	if (!m_ATA.AddOuterProduct(&p) ||
		!S[itemID].AddOuterProduct(&s) ||
		!Q.AddOuterProduct(&p, 0, index, &s))
		return false;

//	Updating m_ATF
	for (i = 0; i < p.size(); i++)
		m_ATF[i] += y * p[i] * w;
	for (i = 0; i < s.size(); i++)
		m_ATF[p.size() + index + i] += y * s[i] * w;

//	m_ATA.Print();
//	Q.Print();

	return true;


}
bool SparseLS::SolveSparseSystem()
{
	int i, j;
	if (!InvertSparseSystem())
		return false;
	//Computing the solution [N1 Qtild] | m_ATF1|
	//                                  | m_ATF2|

	vRecord m_ATF1;
	vRecord m_ATF2;
	m_ATF1.insert(m_ATF1.begin(), m_ATF.begin(), m_ATF.begin() + m_N1.size());
	m_ATF2.insert(m_ATF2.begin(), m_ATF.begin() + m_N1.size(), m_ATF.end());

	m_SolnX = m_N1 * m_ATF1;
	if (!Qtild.empty()){
		vRecord soln_p2 = Qtild * m_ATF2;
		for (i = 0; i < m_SolnX.size(); i++)
			m_SolnX[i] += soln_p2[i];
	}
	if (isStild && !Stild.empty()){
		int     n_rows2 = Qtild.begin()->size();
		vRecord solVec2 = Stild * m_ATF2;
		for (i = 0; i < n_rows2; i++)
			for (j = 0; j < Qtild.size(); j++)
				solVec2[i] += Qtild[j][i] * m_ATF1[j];
		m_SolnX.insert(m_SolnX.end(), solVec2.begin(), solVec2.end());
	}
//	printf("\n -- X vector   --------\n");
//	for (i = 0; i < m_SolnX.size(); i++)
//		printf("%.4f ", m_SolnX[i]);
//	printf("\n ----------------------\n");
	return true;
}

void SparseLS::Print()
{
	std::vector<vMatrix>::iterator mtrx;
	m_ATA.Print();
	Q.Print();
	for (mtrx = S.begin(); mtrx !=S.end(); mtrx++)
		mtrx->Print();

	for (int i = 0; i < m_ATF.size(); i++)	printf ("%.2f ", m_ATF[i]);
	printf("\n ------------- \n");
}
