
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



#include <math.h>
#include <stdlib.h>
#include "digphot_arch.h"
#include "vMatrix.h"
#include "Rotation2D.h"
#include "Rotation3D.h"


extern "C" double **matrix(long nrl, long nrh, long ncl, long nch);
//extern "C" void svdcmp(double **a, int m, int n, double w[], double **v);
extern "C" double *NRvector(long nl, long nh);
extern "C" void free_vector(double *v, long nl, long nh);
extern "C" void free_matrix(double **m, long nrl, long nrh, long ncl, long nch);

#if defined linux
  extern "C" void dgeco_(double *, int *, int *, int *, double *, double *);
  extern "C" void dgedi_(double *, int *, int *, int *, double *, double *,
                         int *);
#else
#  if defined hpux
  extern "C" void dgeco(double *, int *, int *, int *, double *, double *);
  extern "C" void dgedi(double *, int *, int *, int *, double *, double *,
                        int *);
#  else /* irix */
  extern "C" void _dgeco(double *, int *, int *, int *, double *, double *);
  extern "C" void _dgedi(double *, int *, int *, int *, double *, double *,
                         int *);
#  endif 
#endif

//----------------------------------------------------------------------------
template <> bool vMatrix::Define(int m_rows, int m_cols, double val)
{
	if(!empty())
		Erase();
	if (m_rows == 0 || m_cols == 0)
		return false;

	vRecord r(m_cols, val); //row of mvarRows Set to 0 is default but just in case
	for(int col=0; col < m_rows; col++)
//		push_back(r);
		((std::vector<vRecord>*)this)->push_back( r );
		// for efficiency, the rows have all the same length, there is no point in checking it

	return true;
}

//----------------------------------------------------------------------------
/* This function is redundant. It's already specified in the template class */
/* Removed by George Vosselman 
void vMatrix::Erase()
{
	vMatrix::iterator record;

	// Clean up
//	printf("In Erase \n");
	if(!empty()){
		for(record = begin(); record != end(); record++)
			record->erase(record->begin(),record->end());
		erase(begin(),end());
	}
}
*/
//----------------------------------------------------------------------------
/*
vMatrix::vMatrix(const vMatrix& rhs)
{
	if (!empty()) Erase();
	for (vMatrix::const_iterator row = rhs.begin(); row != rhs.end(); row++)
		push_back(*row);
}
*/
//----------------------------------------------------------------------------
/*
vMatrix::vMatrix(const Rotation2D &rot)
{
  Define(2, 2);
  for (int ir=0; ir<2; ir++)
    for (int ic=0; ic<2; ic++)
      Val(ir, ic) = rot.R(ir, ic);
}
*/
//----------------------------------------------------------------------------
/*
vMatrix::vMatrix(const Rotation3D &rot)
{
  Define(3, 3);
  for (int ir=0; ir<3; ir++)
    for (int ic=0; ic<3; ic++)
      Val(ir, ic) = rot.R(ir, ic);
}
*/
//----------------------------------------------------------------------------
template <> void vMatrix::Identity(int m_rows, int m_cols)
{
	if (m_rows != m_cols)
	  printf("Unit of a non-square matrix\n");

	Define(m_rows, m_cols);
	int bound=(m_rows < m_cols)?m_rows:m_cols;
	for(int i =0; i < bound; i++)
		(*this)[i][i] = 1.;
}


//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::operator = (const vMatrix &rhs)
{
	if (this == &rhs)
		return *this;
	if (!empty()) Erase();
	for (vMatrix::const_iterator row = rhs.begin(); row != rhs.end(); row++)
		push_back(*row);
	return(*this);
}

//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::operator = (double rhs)
{
	if (!empty())
		for (int i = 0; i < size(); i++)
			for (int j = 0; j < begin()->size(); j++)
				(*this)[i][j] = rhs;
	return *this;
}

//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::Transpose (const vMatrix &b)
{
	assert( this != &b );
	Define( b.NumColumns(), b.NumRows() );
	for( int i = 0; i<b.NumRows(); i++ )
		for( int j=0; j<b.NumColumns(); j++ )
			(*this)[j][i] = b[i][j];
	return *this;
}

//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::operator += (const vMatrix &rhs)
{
	assert( !empty() );
	assert( !rhs.empty() );
	assert( rhs.size() == size() );
	assert( rhs.begin()->size() == begin()->size() );

	for (int i = 0; i < size(); i++)
		for (int j = 0; j < begin()->size(); j++)
			(*this)[i][j] += rhs.Val(i, j);
	return *this;
}

//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::operator -= (const vMatrix &rhs)
{
	assert( !empty() );
	assert( !rhs.empty() );
	assert( rhs.size() == size() );
	assert( rhs.begin()->size() == begin()->size() );

	for (int i = 0; i < size(); i++)
		for (int j = 0; j < begin()->size(); j++)
			(*this)[i][j] -= rhs.Val(i, j);
	return *this;
}

//---------------------------------------------------------------------------- 
template <> vMatrix & vMatrix::operator *= (const vMatrix &m)
{
       vMatrix res;
       res.Multiply(*this, m);
       *this = res;
       return *this;
 }

//----------------------------------------------------------------------------
template <> vMatrix & vMatrix::operator *= (double val)
{
	assert( !empty() );
	assert( begin()->size() );

	for (int i = 0; i < size(); i++)
		for (int j = 0; j < begin()->size(); j++)
			(*this)[i][j] *= val;
	return *this;
}

//----------------------------------------------------------------------------
// Multiply all matrix elements by a constant
template <> vMatrix & vMatrix::operator * (double c)
{
	if (!empty() && begin()->size())
		for (int i = 0; i < size(); i++)
			for (int j = 0; j < begin()->size(); j++)
				(*this)[i][j] *= c;
	return *this;
}

//----------------------------------------------------------------------------
/// Add a constant to all matrix elements
template <> vMatrix & vMatrix::operator + (double c)
{
	if (!empty() && begin()->size())
		for (int i = 0; i < size(); i++)
			for (int j = 0; j < begin()->size(); j++)
				(*this)[i][j] += c;
	return *this;
}

//----------------------------------------------------------------------------
/// Subtract a constant from all matrix elements
template <> vMatrix & vMatrix::operator - (double c)
{
	if (!empty() && begin()->size())
		for (int i = 0; i < size(); i++)
			for (int j = 0; j < begin()->size(); j++)
				(*this)[i][j] -= c;
	return *this;
}

//----------------------------------------------------------------------------
template <> bool vMatrix::IncrementDiagonal(const vRecord & vec)
{
      if(empty() || size() != begin()->size() || vec.size() || size())
              return false;

      for (int i = 0; i < size(); i++)
                      (*this)[i][i] += vec[i];
      return true;

}

//----------------------------------------------------------------------------
template <> bool vMatrix::IncrementDiagonal(double val)
{
      if(empty() || size() != begin()->size())
              return false;

      for (int i = 0; i < size(); i++)
                      (*this)[i][i] += val;
      return true;
}

/*
//----------------------------------------------------------------------------
void vMatrix::Multiply(vMatrix &A, vMatrix &B, vMatrix &X, int mvarARows, int mvarAColumns, int mvarBColumns)
{
	int i, j, k;
	double sum;
    
	//Multiply A with B and put the result in X
	for(i = 0; i < mvarARows; i++){
		for(j = 0; j < mvarBColumns; j++){
			sum = 0.0;
			for(k = 0; k < mvarAColumns; k++)
                sum = sum + A[i][k] * B[k][j];
            X[i][j] = sum;
		}
	}
}
*/

//----------------------------------------------------------------------------
/*
void vMatrix::Multiply(const vMatrix &A, const vMatrix &B)
{
	int n_rows = A.size();
	int n_cols = A.begin()->size();

	int m_rows = B.size();
	int m_cols = B.begin()->size();

	assert( n_cols == m_rows );
		
	Define(n_rows, m_cols);

	//Multiply A with B
	for(int i = 0; i < n_rows; i++)
		for(int j = 0; j < m_cols; j++)
			for(int k = 0; k < n_cols; k++)
                (*this)[i][j] += A[i][k] * B[k][j];
}
*/
//----------------------------------------------------------------------------
template <> vRecord vMatrix::operator * (const vRecord &m) const
{
	vRecord	res;

	int n_rows = size();
	int n_cols = begin()->size();

	int m_rows = m.size();

	assert( n_cols == m_rows );

	res.reserve(n_rows);
	double sum;
	//Multiply A with B and put the result in res
	for(int i = 0; i < n_rows; i++){
		sum = 0.;
		for(int k = 0; k < n_cols; k++)
			sum += (*this)[i][k] * m[k];
		res.push_back(sum);
	}
	return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::operator * (const vMatrix &m) const
{
	vMatrix	res;

	int n_rows = size();
	int n_cols = begin()->size();

	int m_rows = m.size();
	int m_cols = m.begin()->size();

/*	if (n_cols != m_rows)
		return res; */
	assert( n_cols == m_rows );

	res.Define(n_rows, m_cols);
    
	//Multiply A with B and put the result in res
	for(int i = 0; i < n_rows; i++)
		for(int j = 0; j < m_cols; j++)
			for(int k = 0; k < n_cols; k++)
                res[i][j] += (*this)[i][k] * m[k][j];
	return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::operator + (const vMatrix &rhs) const
{
	vMatrix	res;

	int n_rows = size(); 
	int n_cols = begin()->size();

	int m_rows = rhs.size();
	int m_cols = rhs.begin()->size();

	assert( n_rows == m_rows && n_cols == m_cols );

	res.Define(n_rows, n_cols);
    
	//Multiply A with B and put the result in res
	for(int i = 0; i < n_rows; i++){
		for(int j = 0; j < n_cols; j++){
                res[i][j] = (*this)[i][j] + rhs[i][j];
		}
	}
	return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::operator - (const vMatrix &rhs) const
{
	vMatrix	res;

	int n_rows = size(); 
	int n_cols = begin()->size();

	int m_rows = rhs.size();
	int m_cols = rhs.begin()->size();

	assert( n_rows == m_rows && n_cols == m_cols );

	res.Define(n_rows, n_cols);
    
	//Multiply A with B and put the result in res
	for(int i = 0; i < n_rows; i++){
		for(int j = 0; j < n_cols; j++){
                res[i][j] = (*this)[i][j] - rhs[i][j];
		}
	}
	return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::MultTranspose (const vRecord *W) const
{
	int i, j, k;
	double sum;
	int n_rows = size();
	int n_cols = begin()->size();

	assert( !W || W->size() == n_rows );

	vMatrix res;
	res.Define(n_cols, n_cols);

	if (W == NULL){
		for(i = 0; i < n_cols; i++){
			for(j = i; j < n_cols; j++){
				sum = 0.0;
				for(k = 0; k < n_rows; k++)
      				sum += (*this)[k][i] * (*this)[k][j];
				res[i][j] = sum;
			}
		}
	}
	else{
		for(i = 0; i < n_cols; i++){
			for(j = i; j < n_cols; j++){

				sum = 0.0;
				for(k = 0; k < n_rows; k++)
      				sum += (*this)[k][i] * (*this)[k][j] * (*W)[k];
				res[i][j] = sum;
			}
		}
	}

	// Copy to lower triangular elements
	for(i = 0; i < n_cols; i++)
	 	for(j = 0; j < i; j++)
			res[i][j] = res[j][i];

		return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::Transpose_MultBy (vRecord *_W, vMatrix *B) const
{
	int i, j, k;
	int n_rows = size();
	int n_cols = begin()->size();
	int m_rows, m_cols;
	vMatrix res;
	const vMatrix *D = NULL;
	vRecord *W = NULL;

	assert( !_W || _W->size() == n_rows );
	assert( !B || !B->empty() && B->NumRows() == n_rows );

	if (B){
		m_rows = B->size();
		m_cols = B->begin()->size();
		D = B;
	}
	else{
		m_rows = n_rows;
		m_cols = n_cols;
		D = this;
	}

	if (_W){
		W = _W;
	}
	else
		W = new vRecord(n_cols, 1.);

	res.Define(n_cols, m_cols);
	for(i = 0; i < n_cols; i++)
		for(j = 0; j < m_cols; j++)
			for(k = 0; k < n_rows; k++)
				res[i][j] += (*this)[k][i] * (*D)[k][j] * (*W)[k];
    
	if (!_W) W->erase(W->begin(), W->end());
	return res;
}

//----------------------------------------------------------------------------
template <> vMatrix vMatrix::MultByTranspose (vRecord *_W, vMatrix *B) const
{
	int i, j, k;
	int n_rows = size();
	int n_cols = begin()->size();
	int m_rows, m_cols;
	vMatrix res;
	const vMatrix *D = NULL;
	vRecord *W = NULL;

	assert( !B || B->NumColumns() == n_cols );
	assert( !_W || _W->size() == n_cols );

	if (B){
		m_rows = B->size();
		m_cols = B->begin()->size();
		D = B;
	}
	else{
		m_rows = n_rows;
		m_cols = n_cols;
		D = this;
	}

	if (_W){
		W = _W;
	}
	else
		W = new vRecord(n_cols, 1.);

	res.Define(n_rows, m_rows);
	for(i = 0; i < n_rows; i++)
		for(j = 0; j < m_rows; j++)
			for(k = 0; k < n_cols; k++)
				res[i][j] += (*this)[i][k] * (*D)[j][k] * (*W)[k];

	if (!_W) W->erase(W->begin(), W->end());
	return res;
}

//----------------------------------------------------------------------------
template <> bool vMatrix::CreateFromOuterProduct(vRecord *v, vRecord *w)
{

	int	m_rows = v->size();
	assert (!m_rows) ;

	int m_cols = (w == NULL)? m_rows : w->size();

	if(empty())
		Define(m_rows, m_cols);

	int n_rows = size(); 
	int n_cols = begin()->size();
	
	if (m_rows != n_rows && m_cols != n_cols)
		Define(m_rows, m_cols);

	if (w == NULL){
		for(int i = 0; i < n_rows; i++)
			for(int j = 0; j < n_cols; j++)
				(*this)[i][j] = (*v)[i]*(*v)[j];
	}
	else{
		for(int i = 0; i < n_rows; i++)
			for(int j = 0; j < n_cols; j++)
				(*this)[i][j] = (*v)[i]*(*w)[j];
	}
	return true;
}

//----------------------------------------------------------------------------
template <> bool vMatrix::AddOuterProduct(const vRecord *v, int row0, int col0, const vRecord *w)
{

	assert( v && !( v->empty() ) );

	int n_rows = size();
	int m_rows = v->size();
	assert( n_rows && m_rows );

	int n_cols = begin()->size();
	int m_cols = (w == NULL)? m_rows : w->size();

	assert( row0 + m_rows <= n_rows && col0 + m_cols <= n_cols );

	if (w == NULL){
		for(int i = 0; i < n_rows; i++)
			for(int j = 0; j < n_cols; j++)
				(*this)[i][j] += (*v)[i]*(*v)[j];
	}
	else{
		for(int i = 0; i < m_rows; i++)
			for(int j = 0; j < m_cols; j++){
				double tmp = (*this)[row0 + i][col0 + j], a = (*v)[i], b = (*w)[j];
				/// ???NP what is the above line good for?
				(*this)[row0 + i][col0 + j] += (*v)[i]*(*w)[j];
			}
	}
	return true;
}

//----------------------------------------------------------------------------

template <> bool vMatrix::AddPartial(const vMatrix &v, int r0, int c0)
{
	assert( r0 >= 0 && c0 >= 0 );
	assert( v.size()+r0 <= size() && v.begin()->size() + c0 <= begin()->size() );

	for (int i = 0; i < v.size(); i++)
		for (int j = 0; j < v.begin()->size(); j++)
			(*this)[r0 + i][c0 + j] += v[i][j];
	return true;
}

//----------------------------------------------------------------------------
template <> bool vMatrix::ReplacePartial(const vMatrix &v, int r0, int c0)
{
	assert( r0 >= 0 && c0 >= 0 );
	assert( v.size()+r0 <= size() && v.begin()->size() + c0 <= begin()->size() );

	for (int i = 0; i < v.size(); i++)
		for (int j = 0; j < v.begin()->size(); j++)
			(*this)[r0 + i][c0 + j] = v[i][j];
	return true;
}

//----------------------------------------------------------------------------

template <> vMatrix & vMatrix::Insert(const vMatrix &m, int first_row, int first_col,
                        int num_rows, int num_cols,
                        int row_offset, int col_offset)
{
	assert(first_row >=0 && first_col >= 0);
	assert(first_row+num_rows <= m.NumRows() && first_col+num_cols <= m.NumColumns());
	assert(row_offset >=0 && col_offset >= 0);
	assert(row_offset+num_rows <= NumRows() && col_offset+num_cols <= NumColumns());

	for (int ir=0; ir<num_rows; ir++)
		for (int ic=0; ic<num_cols; ic++)
			Val(ir + row_offset, ic + col_offset)  = m.Val(ir + first_row, ic + first_col);

	return *this;
}

//----------------------------------------------------------------------------
template <> void vMatrix::Convert2NR(double *** m)const
{
	long n_rows = size();
	long n_cols = begin()->size();
	
	*m = (double **) matrix(1L, n_cols, 1L, n_rows);
	if( !*m )
		std::cerr << "allocation failure in vMatrix::Convert2NR\n";

	for (int i = 1; i <= n_rows; i++)
		for(int j = 1; j <= n_cols; j++)
			(*m)[j][i] = (*this)[i-1][j-1];
}

//----------------------------------------------------------------------------

template <> void vMatrix::Convert2LP( double **a ) const
{
	long row = NumRows();
	long col = NumColumns();

	*a = (double*) calloc( (row*col), sizeof(double) );
	if( !*a )
		std::cerr << "allocation failure in vMatrix::Convert2LP\n";
	else
	{
		for( int i=0; i<row; i++ )
			for( int j=0; j<row; j++ )
				(*a)[ i*row+j ] = (*this)[i][j];
	}

	return;
}

//----------------------------------------------------------------------------

template <> void vMatrix::ConvertFromLP( const double *a, long row, long col )
{
	assert( row>=0 );
	assert( col>=0 );
	assert( a );

	Define( row, col );
	for( int i=0; i<row; i++ )
		for( int j=0; j<col; j++ )
			(*this)[i][j] = a[ i*row+j ];

	return;
}

//----------------------------------------------------------------------------
template <> void vMatrix::ConvertFromNR(vMatrix *A, double ** m, long n_rows, long n_cols)
{
	assert( 0 /* code missing */ );
}

//----------------------------------------------------------------------------
#ifndef EPS 
#define EPS 1.E-6
#endif
/*
bool vMatrix::Invert_3x3(vMatrix &Ainv) const
{

	assert( size() == 3 && begin()->size() == 3 );

	if (Ainv.size() != 3 || Ainv.begin()->size() != 3)
		Ainv.Define(3, 3);

	double a = (*this)[0][0], b = (*this)[0][1], c = (*this)[0][2];
	double                    e = (*this)[1][1], f = (*this)[1][2];
	double                                       i = (*this)[2][2];

	double t2 = f*f;
	double t4 = a*e;
	double t7 = b*b;
	double t9 = c*b;
	double t12 = c*c;

	double t15 = -t4*i+a*t2+t7*i-2.0*t9*f+t12*e;

	if(fabs(t15) <= EPS){
		return RankDef;
	}
	t15 = 1.0/t15;

	double t20 = (-b*i+c*f)*t15;
	double t24 = (b*f-c*e)*t15;
	double t30 = (a*f-t9)*t15;

	Ainv[0][0] = (-e*i+t2)*t15; Ainv[0][1] = -t20;           Ainv[0][2] = -t24;
	Ainv[1][0] = -t20;          Ainv[1][1] = -(a*i-t12)*t15; Ainv[1][2] = t30;
	Ainv[2][0] = -t24;          Ainv[2][1] = t30;            Ainv[2][2] = -(t4-t7)*t15;

	return FullRank;
}
*/
//----------------------------------------------------------------------------
/*
bool vMatrix::InvertBySVD(vMatrix & m_N1) const
{
	double	**nrU;
	double	**v;
	double	*w, *w_inv;
	bool	matrixType = FullRank;
	int i;

	Convert2NR(&nrU);
	long n_rows = size();
	v = (double **) matrix(1, n_rows, 1, n_rows);
	w = (double *) NRvector(1,n_rows);
	w = (double *) NRvector(1,n_rows);
	w_inv = (double *) NRvector(1,n_rows);

//	svdcmp(nrU, n_rows, n_rows, w, v);
	for (i = 1; i <= n_rows; i++)
		if (fabs(w[i]) > EPS)
			w_inv[i] = 1./w[i];
		else{
			w_inv[i] = 0.;
			matrixType = RankDef;
		}

	if (m_N1.empty() || m_N1.size() != n_rows || m_N1.begin()->size() != n_rows)
		m_N1.Define(n_rows, n_rows);
	else
		m_N1 = 0.;
	// Creating the g-inverse
	for (i = 1; i <= n_rows; i++)
		for (int j = 1; j <= n_rows; j++){
			double sum = 0.;
			for (int k = 1; k <= n_rows; k++)
				sum += nrU[i][k] * w_inv[k] * v[j][k];
			m_N1[i-1][j-1] = sum;
		}
	free_vector(w, 1, n_rows);
	free_vector(w_inv, 1, n_rows);
	free_matrix(v, 1, n_rows, 1, n_rows);
	free_matrix(nrU, 1, n_rows, 1, n_rows);
	return matrixType;
}
*/

//----------------------------------------------------------------------------
/*
bool vMatrix::Invert(vMatrix & A, int inversionType) 
{
	switch (inversionType){
	case STANDARD_INVERSION:
		if (size() == 3){
			if (!Invert_3x3(A)) 
				return false;
		}
		else if (!Inverse(A)) 
			return false;
		break;
	case SVD_INVERSION:
		if (!InvertBySVD(A))
			return false;
		break;
	default:
		return false;
		break;
	}
	return true;
}
*/
//----------------------------------------------------------------------------
/*
bool vMatrix::Inverse(vMatrix &Ainv) const
{
	int TempInt, i, j, p; 
	int LastCol; // Last column of the augmented matrix
	double Multiplier;

	int mvarColumns = size();
	int mvarRows = begin()->size();

	assert( mvarColumns == mvarRows );

	if (Ainv.size() != mvarRows || Ainv.begin()->size() != mvarRows)
		Ainv.Define(mvarRows, mvarRows);

	vMatrix M; // row, colums
	vVector PVT;

	// Initialise matrix M - Augmented matrix
	M.Define(mvarRows, 2 * mvarRows);

	// Initialise vector PVT
	int val=0;
	for(int row=0; row < mvarRows; row++)
		PVT.push_back(val);

	LastCol = 2 * mvarRows; // Last column of the augmented matrix

	// Create the Augmented matrix
	for(i = 0; i < mvarRows; i++){	// Copy matrix A to augmented matrix
		for(j = 0; j < mvarRows; j++)
			M[i][j] = (*this)[i][j];
		// Initialise the Identity matrix within the augmented matrix
		M[i][mvarRows + i] = 1.0; // Main diagonal=1
	}

	// Initialise the pivot vector
	for(i = 0; i < mvarRows; i++)
		PVT[i] = i;

	// Invert: Create an identity matrix on the left side of the augmented matrix
	for(p = 0; p < mvarRows; p++){
		// Pivot strategy: look for the element with the largest absolute value
		for(i = p + 1; i < mvarRows; i++){
			if(fabs(M[PVT[i]][p]) > fabs(M[PVT[p]][p])){
				TempInt = PVT[p];
				PVT[p] = PVT[i];
				PVT[i] = TempInt;
			}
		}
		if(fabs(M[PVT[p]][p]) <= EPS)//print "The matrix is singular"
			return false;
		for(i = p - 1; i >= 0; i--){ // Reduce upper values to zero
			Multiplier = M[PVT[i]][p] / M[PVT[p]][p];

			for(j = p + 1; j < LastCol; j++)
				M[PVT[i]][j] = M[PVT[i]][j] - Multiplier * M[PVT[p]][j];
		}

		for(i = p + 1; i < mvarRows; i++){ //Reduce lower values to zero
			Multiplier = M[PVT[i]][p] / M[PVT[p]][p];

			for(j = p + 1; j < LastCol; j++)
				M[PVT[i]][j] = M[PVT[i]][j] - Multiplier * M[PVT[p]][j];
		}
	}

	// Test for singularity
	if(M[PVT[mvarRows - 1]][mvarRows - 1] == 0)
		//MsgBox "The matrix is singular"
		return false;

	// Divide by diagonal of augmented matrix
	for(p = 0; p < mvarRows; p ++)
		for(j = mvarRows; j < 2 * mvarRows; j++)
			M[PVT[p]][j] = M[PVT[p]][j] / M[PVT[p]][p];

	// Copy inverted matrix to A matrix
	for(j = 0; j < mvarRows; j++){
		i = mvarRows + j;
		for(p = 0; p < mvarRows; p++)
			Ainv[p][j] = M[PVT[p]][i];
	}
	return FullRank;
}
*/
#undef EPS

//----------------------------------------------------------------------------
/*
vMatrix & vMatrix::Inverse( const vMatrix &matrix, double &cond )
{
  int    job, *pivot;
  double *z, det[2];

  int row = matrix.NumRows();
  int col = matrix.NumColumns();

  assert( row == col ); // Check the matrix is square
  *this = matrix;              // Copy the matrix to be inverted

  pivot = (int *) malloc(row * sizeof(int));
  z     = (double *) malloc(row * sizeof(double));

  double* a;
  Convert2LP( &a );

#if defined linux
  dgeco_(a, &row, &col, pivot, &cond, z);
#else
#if defined hpux
  dgeco(a, &row, &col, pivot, &cond, z);
#else
  _dgeco(a, &row, &col, pivot, &cond, z);
#endif
#endif

  if (1.0 + cond == 1.0)
    fprintf(stderr, "Warning: bad condition of matrix (%5.2e)\n", cond);

  job = 1;
#if defined linux
  dgedi_(a, &row, &col, pivot, det, z, &job);
#else
#if defined hpux
  dgedi(a, &row, &col, pivot, det, z, &job);
#else
  _dgedi(a, &row, &col, pivot, det, z, &job);
#endif
#endif

  ConvertFromLP( a, row, col );

  free(pivot);  free(z); free(a);
  return(*this);
}
*/
//----------------------------------------------------------------------------

template <> bool vMatrix::SparseMultiplication(vMatrix & A, std::vector<vMatrix> &obj)
{
	std::vector<vMatrix>::iterator matrx;
	int prm_counter = 0;

	assert( !A.empty() );
	// Size of the sparse matrix (sum of the sub matrices)
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		assert( !matrx->empty() && matrx->size() == matrx->begin()->size() );
		prm_counter += matrx->size();
	}

	assert( prm_counter == A.begin()->size() );

	Define(A.size(), prm_counter);
	int prev_pos = 0;
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		for (int i = 0; i < A.size(); i++)
			for(int j = 0; j < matrx->size(); j++)
				for(int k = 0; k < matrx->size(); k++)
					(*this)[i][prev_pos + j] += A[i][prev_pos + k]*(*matrx)[k][j];
				prev_pos += matrx->size();
	}
	return true;
}

//----------------------------------------------------------------------------

template <> bool vMatrix::SparseAddition(vMatrix & A, std::vector<vMatrix> &obj)
{
	std::vector<vMatrix>::iterator matrx;
	int prm_counter = 0;

	assert( !A.empty() );
 // Size of the sparse matrix (sum of the sub matrices)
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		assert( !matrx->empty() && matrx->size() == matrx->begin()->size() );
		prm_counter += matrx->size();
	}

	assert( prm_counter == A.begin()->size() );

	(*this) = A;
	int prev_pos = 0;
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		for (int i = 0; i < matrx->size(); i++)
			for(int j = 0; j < matrx->size(); j++)
					(*this)[prev_pos + i][prev_pos + j] += (*matrx)[i][j];
				prev_pos += matrx->size();
	}
	return true;
}

//----------------------------------------------------------------------------

template <> bool vMatrix::SparseSubtraction(vMatrix & A, std::vector<vMatrix> &obj)
{
	std::vector<vMatrix>::iterator matrx;
	int prm_counter = 0;

	assert( !A.empty() );
	// Size of the sparse matrix (sum of the syb matrices)
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		assert( !matrx->empty() && matrx->size() == matrx->begin()->size() );
		if (matrx->empty() || matrx->size() != matrx->begin()->size())
			return false;
		prm_counter += matrx->size();
	}

	assert( prm_counter == A.begin()->size() );

	(*this) = A;
	int prev_pos = 0;
	for (matrx = obj.begin(); matrx != obj.end(); matrx++){
		for (int i = 0; i < matrx->size(); i++)
			for(int j = 0; j < matrx->size(); j++)
					(*this)[prev_pos + i][prev_pos + j] -= (*matrx)[i][j];
				prev_pos += matrx->size();
	}
	return true;
}

//----------------------------------------------------------------------------
template <> double vMatrix::Det() const
{
   int col = NumColumns();
   int row = NumRows();
   double piv, det = 1;

   if (row != col)
      printf( "Determinant a non-square matrix!\n");
   
   vMatrix temp(*this);

   for (int k = 0; k < row; k++)
   {
      int indx = temp.Pivot(k);
      if (indx == -1)
	 return 0;
      if (indx != 0)
	 det = - det;
      det = det * temp.Val(k, k);
      for (int i = k + 1; i < row; i++)
      {
	 double piv = temp.Val(i, k) / temp.Val(k, k);
	 for (int j = k + 1; j < row; j++)
	    temp.Val(i, j) -= piv * temp.Val(k, j);
      }
   }
   return det;
}

//----------------------------------------------------------------------------
/*
int vMatrix::Pivot(int r)
{
  int row = NumRows();
  int col = NumColumns();
  int k = r;
  double amax = -1, temp;

  for (int i = r; i < row; i++)
    if ((temp = fabs(Val(i, r))) > amax && temp != 0.0)
     {
       amax = temp;
       k = i;
     }
     
  if (Val(k, r) == 0)
     return -1;
     
  if (k != r)
  {
     // swap k and r rows
     for (int j = 0; j < col; j++)
     {
       double q = Val(k, j);
       Val(k , j) = Val(r, j);
       Val(r, j) = q;
     }  
     return k;
  }
  return 0;
}
*/
//----------------------------------------------------------------------------
template <> void vMatrix::Print(char *format)const
{
        vMatrix* locRef = (vMatrix*)this;
	// ugly trick, in the above line the const-ness is casted away
	if( empty() )
		return;

	vMatrix::iterator	row;
	vRecord::iterator	col;
	int n_rows = size();
	int n_cols = begin()->size();

	for (row = locRef->begin(); row != locRef->end(); row++){
		if (row->size() > 1) {
			for (col = row->begin(); col != row->end(); col++)
				printf(format, *col);
			printf ("\n");
		}
		else
		{
			printf(format, *(row->begin()));
			printf("\n");
		}
	}

	printf("***************\n");
}

//----------------------------------------------------------------------------
template <> void vMatrix::Print()const
{
	Print( (char *) " %8.2f" );
}

//----------------------------------------------------------------------------
std::ostream& operator << (std::ostream& s, const vMatrix& mat)
{
	s << mat.NumRows() << "  " << mat.NumColumns() << "\n";
	for( int r=0; r<mat.NumRows(); r++ )
	{
		for( int c=0; c<mat.NumColumns(); c++ )
			s << mat[r][c] << ' ';
		s << "\n";
	}
	return s;
}

double MultRowColumn(const vMatrix &b1, int i, const vMatrix &b2, int j)
{
   double sum=0.0;

   assert(b1.NumColumns() == b2.NumRows());
   assert(i >= 0 && i < b1.NumRows());
   assert(j >= 0 && j < b2.NumColumns());

   for(int k = 0; k < b1.NumColumns(); k++)
     sum += b1[i][k] * b2[k][ j];

   return(sum);
}


