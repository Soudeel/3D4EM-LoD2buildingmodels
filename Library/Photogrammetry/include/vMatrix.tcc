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


#ifndef __VMATRIX__CC
#define __VMATRIX__CC

#include <math.h>
#include <stdlib.h>
#include "digphot_arch.h"
//#include "vMatrix.h"
#include "Rotation2D.h"
#include "Rotation3D.h"
// NR_REMOVED #include "nr.h" 
#include "vMatrix.h" // NR_ADDED
using namespace std; // NR_ADDED

//extern "C" double **matrix(long nrl, long nrh, long ncl, long nch);
//extern "C" void svdcmp(double **a, int m, int n, double w[], double **v);
//extern "C" double *NRvector(long nl, long nh);
//extern "C" void free_vector(double *v, long nl, long nh);
//extern "C" void free_matrix(double **m, long nrl, long nrh, long ncl, long nch);


// LINPACK functions for matrix inversion
extern "C" void dgeco_(double *, int *, int *, int *, double *, double *);
extern "C" void dgedi_(double *, int *, int *, int *, double *, double *, int *);

// eigen values and vectors from EISPACK
extern "C" void rg_(int *, int *, double *, double *, double *, int *,
                    double *, double *, double *, int *);

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::Define(int m_rows, int m_cols, T val)
{
	if(!this->empty())
		Erase();
	if (m_rows == 0 || m_cols == 0)
		return false;

	FRSvector<T> r(m_cols, val); // row of mvarRows Set to 0 is default 
                                     // but just in case
	for(int col=0; col < m_rows; col++)
//		push_back(r);
		((std::vector< FRSvector<T> >*)this)->push_back( r );
		// for efficiency, the rows have all the same length, 
                // there is no point in checking it

	return true;
}

//----------------------------------------------------------------------------
template<class T>
void FRSmatrix<T>::Erase()
{
	typename FRSmatrix<T>::iterator record;

	// Clean up
//	printf("In Erase \n");
	if(!this->empty()){
		for(record = this->begin(); record != this->end(); record++)
			record->erase(record->begin(),record->end());
		this->erase(this->begin(), this->end());
	}
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T>::FRSmatrix(const FRSmatrix<T>& rhs) : std::vector <FRSvector<T> >()
{
	if (!this->empty()) Erase();
	for (typename FRSmatrix<T>::const_iterator row = rhs.begin(); row != rhs.end(); row++)
		push_back(*row);
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T>::FRSmatrix(const Rotation2D &rot)
{
  Define(2, 2);
  for (int ir=0; ir<2; ir++)
    for (int ic=0; ic<2; ic++)
      Val(ir, ic) = rot.R(ir, ic);
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T>::FRSmatrix(const Rotation3D &rot)
{
  Define(3, 3);
  for (int ir=0; ir<3; ir++)
    for (int ic=0; ic<3; ic++)
      Val(ir, ic) = rot.R(ir, ic);
}

//----------------------------------------------------------------------------
template<class T>
void FRSmatrix<T>::Identity(int m_rows, int m_cols)
{
	if (m_rows != m_cols)
	  printf("Unit of a non-square matrix\n");

	Define(m_rows, m_cols);
	int bound=(m_rows < m_cols)?m_rows:m_cols;
	for(int i =0; i < bound; i++)
		(*this)[i][i] = 1.;
}


//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator = (const FRSmatrix<T> &rhs)
{
	if (this == &rhs)
		return *this;
	if (!this->empty()) Erase();
	for (typename FRSmatrix<T>::const_iterator row = rhs.begin(); row != rhs.end(); row++)
		push_back(*row);
	return(*this);
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator = (T rhs)
{
	if (!this->empty())
		for (int i = 0; i < this->size(); i++)
			for (int j = 0; j < this->begin()->size(); j++)
				(*this)[i][j] = rhs;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::Transpose (const FRSmatrix<T> &b)
{
	assert( this != &b );
	Define( b.NumColumns(), b.NumRows() );
	for( int i = 0; i<b.NumRows(); i++ )
		for( int j=0; j<b.NumColumns(); j++ )
			(*this)[j][i] = b[i][j];
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator += (const FRSmatrix<T> &rhs)
{
	assert( !this->empty() );
	assert( !rhs.empty() );
	assert( rhs.size() == this->size() );
	assert( rhs.begin()->size() == this->begin()->size() );

	for (int i = 0; i < this->size(); i++)
		for (int j = 0; j < this->begin()->size(); j++)
			(*this)[i][j] += rhs.Val(i, j);
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator -= (const FRSmatrix<T> &rhs)
{
	assert( !this->empty() );
	assert( !rhs.empty() );
	assert( rhs.size() == this->size() );
	assert( rhs.begin()->size() == this->begin()->size() );

	for (int i = 0; i < this->size(); i++)
		for (int j = 0; j < this->begin()->size(); j++)
			(*this)[i][j] -= rhs.Val(i, j);
	return *this;
}

//---------------------------------------------------------------------------- 
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator *= (const FRSmatrix<T> &m)
{
       FRSmatrix<T> res;
       res.Multiply(*this, m);
       *this = res;
       return *this;
 }

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator *= (T val)
{
	assert( !this->empty() );
	assert( this->begin()->size() );

	for (int i = 0; i < this->size(); i++)
		for (int j = 0; j < this->begin()->size(); j++)
			(*this)[i][j] *= val;
	return *this;
}

//----------------------------------------------------------------------------
// Multiply all matrix elements by a constant
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator * (T c)
{
	if (!this->empty() && this->begin()->size())
		for (int i = 0; i < this->size(); i++)
			for (int j = 0; j < this->begin()->size(); j++)
				(*this)[i][j] *= c;
	return *this;
}

//----------------------------------------------------------------------------
/// Add a constant to all matrix elements
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator + (T c)
{
	if (!this->empty() && this->begin()->size())
		for (int i = 0; i < this->size(); i++)
			for (int j = 0; j < this->begin()->size(); j++)
				(*this)[i][j] += c;
	return *this;
}

//----------------------------------------------------------------------------
/// Subtract a constant from all matrix elements
template<class T>
FRSmatrix<T> & FRSmatrix<T>::operator - (T c)
{
	if (!this->empty() && this->begin()->size())
		for (int i = 0; i < this->size(); i++)
			for (int j = 0; j < this->begin()->size(); j++)
				(*this)[i][j] -= c;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::IncrementDiagonal(const FRSvector<T> & vec)
{
      if(this->empty() || this->size() != this->begin()->size() ||
         vec.size() != this->size())
              return false;

      for (int i = 0; i < this->size(); i++)
                      (*this)[i][i] += vec[i];
      return true;

}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::IncrementDiagonal(T val)
{
      if(this->empty() || this->size() != this->begin()->size())
              return false;

      for (int i = 0; i < this->size(); i++)
                      (*this)[i][i] += val;
      return true;
}

/*
//----------------------------------------------------------------------------
template<class T>
void FRSmatrix<T>::Multiply(vMatrix &A, vMatrix &B, vMatrix &X, int mvarARows, int mvarAColumns, int mvarBColumns)
{
	int i, j, k;
	T sum;
    
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
template<class T>
void FRSmatrix<T>::Multiply(const FRSmatrix<T> &A, const FRSmatrix<T> &B)
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

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSmatrix<T>::operator * (const FRSvector<T> &m) const
{
	assert( !this->empty() && !m.empty() );

	int n_rows = this->size();
	int n_cols = this->begin()->size();
	int m_rows = m.size();

	assert( n_cols == m_rows );

	FRSvector<T> res( n_rows );

	T sum;
	//Multiply *this with m and put the result in res
	for(int i = 0; i < n_rows; i++){
		sum = 0.;
		for(int k = 0; k < n_cols; k++)
			sum += (*this)[i][k] * m[k];
		res[i] = sum;
	}
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> FRSmatrix<T>::operator * (const FRSmatrix<T> &m) const
{
	assert(!this->empty() && !m.empty());

	int n_rows = this->size();
	int n_cols = this->begin()->size();
	int m_rows = m.size();
	int m_cols = m.begin()->size();

	assert( n_cols == m_rows );

	FRSmatrix<T> res( n_rows, m_cols, 0 );
    
	//Multiply *this with m and put the result in res
	for(int i = 0; i < n_rows; i++)
		for(int j = 0; j < m_cols; j++)
			for(int k = 0; k < n_cols; k++)
                res[i][j] += (*this)[i][k] * m[k][j];
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> FRSmatrix<T>::operator + (const FRSmatrix<T> &rhs) const
{
	FRSmatrix<T>	res;

	int n_rows = this->size(); 
	int n_cols = this->begin()->size();

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
template<class T>
FRSmatrix<T> FRSmatrix<T>::operator - (const FRSmatrix<T> &rhs) const
{
	FRSmatrix<T>	res;

	int n_rows = this->size(); 
	int n_cols = this->begin()->size();

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
template<class T>
FRSmatrix<T> FRSmatrix<T>::MultTranspose (const FRSvector<T> *W) const
{
	int i, j, k;
	T sum;
	int n_rows = this->size();
	int n_cols = this->begin()->size();

	assert( !W || W->size() == n_rows );

	FRSmatrix<T> res;
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
template<class T>
FRSmatrix<T> FRSmatrix<T>::Transpose_MultBy (FRSvector<T> *_W, 
                                             FRSmatrix<T> *B) const
{
	int i, j, k;
	int n_rows = this->size();
	int n_cols = this->begin()->size();
	int m_rows, m_cols;
	FRSmatrix<T> res;
	const FRSmatrix<T> *D = NULL;
	FRSvector<T> *W = NULL;

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
		W = new FRSvector<T> (n_cols, 1.);

	res.Define(n_cols, m_cols);
	for(i = 0; i < n_cols; i++)
		for(j = 0; j < m_cols; j++)
			for(k = 0; k < n_rows; k++)
				res[i][j] += (*this)[k][i] * (*D)[k][j] * (*W)[k];
    
	if (!_W) W->erase(W->begin(), W->end());
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSmatrix<T> FRSmatrix<T>::MultByTranspose (FRSvector<T> *_W, 
                                            FRSmatrix<T> *B) const
{
	int i, j, k;
	int n_rows = this->size();
	int n_cols = this->begin()->size();
	int m_rows, m_cols;
	FRSmatrix<T> res;
	const FRSmatrix<T> *D = NULL;
	FRSvector<T> *W = NULL;

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
		W = new FRSvector<T>(n_cols, 1.);

	res.Define(n_rows, m_rows);
	for(i = 0; i < n_rows; i++)
		for(j = 0; j < m_rows; j++)
			for(k = 0; k < n_cols; k++)
				res[i][j] += (*this)[i][k] * (*D)[j][k] * (*W)[k];

	if (!_W) W->erase(W->begin(), W->end());
	return res;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::CreateFromOuterProduct(FRSvector<T> *v, FRSvector<T> *w)
{

	int	m_rows = v->size();
	assert (m_rows) ;

	int m_cols = (w == NULL)? m_rows : w->size();

	if(this->empty())
		Define(m_rows, m_cols);

	int n_rows = this->size(); 
	int n_cols = this->begin()->size();
	
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
template<class T>
bool FRSmatrix<T>::CreateProjectionMatrix(FRSvector<T> *v)
{
	FRSvector<T> loc( *v );
	T norm = loc.Norm();
	if( !norm )
		return false;
	loc /= norm;
	Identity( v->size() );
    for( int i=0; i<v->size(); i++ )
	{
        (*this)[i][i] -= (*v)[i]*(*v)[i];
		for( int j=i+1; j< v->size(); j++ )
			(*this)[j][i] = (*this)[i][j] = -(*v)[i]*(*v)[j];
	}
	return true;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::CreateMirrorMatrix(FRSvector<T> *v)
{
    FRSvector<T> loc( *v );
    T norm = loc.Norm();
    if( !norm )
        return false;
    loc /= norm;
    Identity( v->size() );
    for( int i=0; i<v->size(); i++ )
    {
        (*this)[i][i] -= (T)2*(*v)[i]*(*v)[i];
        for( int j = i + 1; j < v->size(); j++ )
            (*this)[j][i] = (*this)[i][j] = -(T)2*(*v)[i]*(*v)[j];
    }
    return true;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::AddOuterProduct(const FRSvector<T> *v, 
                                   int row0, int col0, 
                                   const FRSvector<T> *w)
{

	assert( v && !( v->empty() ) );

	int n_rows = this->size();
	int m_rows = v->size();
	assert( n_rows && m_rows );

	int n_cols = this->begin()->size();
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
				T tmp = (*this)[row0 + i][col0 + j], a = (*v)[i], b = (*w)[j];
				/// ???NP what is the above line good for?
				(*this)[row0 + i][col0 + j] += (*v)[i]*(*w)[j];
			}
	}
	return true;
}

//----------------------------------------------------------------------------

template<class T>
bool FRSmatrix<T>::AddPartial(const FRSmatrix<T> &v, int r0, int c0)
{
	assert( r0 >= 0 && c0 >= 0 );
	assert( v.size()+r0 <= this->size() && 
                v.begin()->size() + c0 <= this->begin()->size() );

	for (int i = 0; i < v.size(); i++)
		for (int j = 0; j < v.begin()->size(); j++)
			(*this)[r0 + i][c0 + j] += v[i][j];
	return true;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::ReplacePartial(const FRSmatrix<T> &v, int r0, int c0)
{
	assert( r0 >= 0 && c0 >= 0 );
	assert( v.size()+r0 <= this->size() && 
            v.begin()->size() + c0 <= this->begin()->size() );

	for (int i = 0; i < v.size(); i++)
		for (int j = 0; j < v.begin()->size(); j++)
			(*this)[r0 + i][c0 + j] = v[i][j];
	return true;
}

//----------------------------------------------------------------------------

template<class T>
FRSmatrix<T> & FRSmatrix<T>::Insert(const FRSmatrix<T> &m, 
                                    int first_row, int first_col,
                                    int num_rows, int num_cols,
                                    int row_offset, int col_offset)
{
	assert(first_row >=0 && first_col >= 0);
	assert(first_row+num_rows <= m.NumRows() && 
               first_col+num_cols <= m.NumColumns());
	assert(row_offset >=0 && col_offset >= 0);
	assert(row_offset+num_rows <= NumRows() && 
               col_offset+num_cols <= NumColumns());

	for (int ir=0; ir<num_rows; ir++)
		for (int ic=0; ic<num_cols; ic++)
			Val(ir + row_offset, ic + col_offset)  = m.Val(ir + first_row, ic + first_col);

	return *this;
}

//----------------------------------------------------------------------------
template<class T>
void FRSmatrix<T>::Convert2NR(T *** m)const
{
  printf("Function FRSmatrix<T>::Convert2NR(T *** m) const is obsolete\n");
  exit(0);
  
/* NR_REMOVED
	assert( sizeof(T) == sizeof(double) );
	// preliminary solution. Will fail on integers with 4 bytes, or on 
	// floats. Function matrix will have to be changed or replaced.

	long n_rows = this->size();
	long n_cols = this->begin()->size();
	
    *m = (T **) matrix(1L, n_cols, 1L, n_rows);
	if( !*m )
		std::cerr << "allocation failure in FRSmatrix<T>::Convert2NR\n";

	for (int i = 1; i <= n_rows; i++)
		for(int j = 1; j <= n_cols; j++)
			(*m)[j][i] = (*this)[i-1][j-1];
NR_REMOVED END */
}

//----------------------------------------------------------------------------

template<class T>
void FRSmatrix<T>::Convert2LP( T **a ) const
{
	long row = NumRows();
	long col = NumColumns();

	*a = (T*) calloc( (row*col), sizeof(T) );
	if( !*a )
		std::cerr << "allocation failure in FRSmatrix<T>::Convert2LP\n";
	else
	{
		for( int i=0; i<row; i++ )
			for( int j=0; j<row; j++ )
				(*a)[ i*row+j ] = (*this)[i][j];
	}

	return;
}

//----------------------------------------------------------------------------

template<class T>
void FRSmatrix<T>::ConvertFromLP( const T *a, long row, long col )
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
template<class T>
void FRSmatrix<T>::ConvertFromNR(FRSmatrix<T> *A, double ** m, 
                                 long n_rows, long n_cols)
{
	assert( 0 /* code missing */ );
}

//----------------------------------------------------------------------------

template<class T>
bool FRSmatrix<T>::Invert_3x3(FRSmatrix<T> &Ainv, T EPS) const
{

	assert( this->size() == 3 && this->begin()->size() == 3 );

	if (Ainv.size() != 3 || Ainv.begin()->size() != 3)
		Ainv.Define(3, 3);

	T a = (*this)[0][0], b = (*this)[0][1], c = (*this)[0][2];
	T                    e = (*this)[1][1], f = (*this)[1][2];
	T                                       i = (*this)[2][2];

	T t2 = f*f;
	T t4 = a*e;
	T t7 = b*b;
	T t9 = c*b;
	T t12 = c*c;

	T t15 = -t4*i+a*t2+t7*i-2.0*t9*f+t12*e;

	if(fabs(t15) <= EPS){
		return RankDef;
	}
	t15 = 1.0/t15;  // here is the division, if integer division is 
                        // possible without remainder, than also
                        // integer matrices are inverted correctly

	T t20 = (-b*i+c*f)*t15;
	T t24 = (b*f-c*e)*t15;
	T t30 = (a*f-t9)*t15;

	Ainv[0][0] = (-e*i+t2)*t15; Ainv[0][1] = -t20;           Ainv[0][2] = -t24;
	Ainv[1][0] = -t20;          Ainv[1][1] = -(a*i-t12)*t15; Ainv[1][2] = t30;
	Ainv[2][0] = -t24;          Ainv[2][1] = t30;            Ainv[2][2] = -(t4-t7)*t15;

	return FullRank;
}

//----------------------------------------------------------------------------

template<class T>
bool FRSmatrix<T>::InvertBySVD(FRSmatrix<T> & m_N1, int& rDef, T EPS) const
{
  // New code based on LINPACK

  int    job, *pivot;
  double *z, det[2], cond;

  int row = NumRows();
  int col = NumColumns();

  assert( row == col ); // Check the matrix is square

  pivot = (int *) malloc(row * sizeof(int));
  z     = (double *) malloc(row * sizeof(double));

  double* a;
  Convert2LP( &a );

  dgeco_(a, &row, &col, pivot, &cond, z);

  if (1.0 + cond == 1.0)
    fprintf(stderr, "Warning: bad condition of matrix (%5.2e)\n", cond);

  job = 1;
  dgedi_(a, &row, &col, pivot, det, z, &job);

  m_N1.ConvertFromLP( a, row, col );

  free(pivot);  free(z); free(a);
  return true;
  
// End of new code based on LINPACK

/* Inversion by Gauss removed 
  printf("Inversion by SVD removed from library, inversion by Gauss\n");
  rDef = 0;
  FRSmatrix<T> locCopy = *this;
  
  if (!InvertByGauss(m_N1, EPS)){
    	printf("Inversion failed, rDef has been set to 1\n");
    	rDef = 1;
		return false;
		}
  return true;

*/  
  
/* NR_REMOVED	
	bool matrixType = FullRank;
	int i;

	long n_rows = this->size();
	rDef = 0;
	FRSmatrix<T> locCopy = *this;
	NRMat<T> loc( locCopy );
	NRVec<T> eVal( n_rows );
	NRVec<T> eValInv( n_rows );
	NRMat<T> eVec( n_rows, n_rows );
	NR::svdcmp( loc, eVal, eVec );
	for (i = 0; i < n_rows; i++)
		if (fabs(eVal[i]) > EPS)
			eValInv[i] = 1./eVal[i];
		else{
			eValInv[i] = 0.;
			matrixType = RankDef;
			rDef++;
		}

	if (m_N1.empty() || m_N1.size() != n_rows || m_N1.begin()->size() != n_rows)
		m_N1.Define(n_rows, n_rows);
	else
		m_N1 = 0.;
	// Creating the g-inverse
	for (i = 0; i < n_rows; i++)
		for (int j = 0; j < n_rows; j++){
			double s;um = 0.;
			for (int k = 0; k < n_rows; k++)
				sum += eVec[i][k] * eValInv[k] * loc[j][k];
			m_N1[i][j] = sum;
		}
	return matrixType;
NR_REMOVED END */
}

//----------------------------------------------------------------------------
//NR_REMOVED
/*
// get the singular value decomposition
template<class T>
void FRSmatrix<T>::GetSingularValues(FRSvector<T>& eigVal, FRSmatrix<T>& eigVec)const 
{
  printf("Fuction FRSmatrix<T>::GetSingularValues(FRSvector<T>& eigVal, FRSmatrix<T>& eigVec)const\n");
  printf("has been removed from the mapping library.\n");
  
// NR_REMOVED
  FRSmatrix<T> locCopy = *this;
  NRMat<T> loc( locCopy );
  NRVec<T> eVal( eigVal );
  NRMat<T> eVec( eigVec );
  NR::svdcmp( loc, eVal, eVec );
//NR_REMOVED END 

  // Determine the eigen values and eigen vectors with Eispack
  
  int      matz=1, error;
  
  FRSmatrix<T> locCopy = *this;
  int n_rows = this->size();
  int n_cols = this->begin()->size();

  double   tmp1[n_cols], tmp2[n_cols], eigen_real[n_cols], eigen_imag[n_cols], eigen_vec[n_rows][n_cols],
           central_moments[n_rows*n_cols];
  
  rg_(n_rows, n_cols, (double *) real_matrix, eigen_real, eigen_imag, &matz,
      (double *) eigen_vec, tmp1, tmp2, &error);


}
*/
//END_NR_REMOVED

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::Invert(FRSmatrix<T> & A, int inversionType, T EPS) 
{
	switch (inversionType){
	case STANDARD_INVERSION:
		if (this->size() == 3){
			if (!Invert_3x3(A, EPS)){ 
				printf("Inversion failed\n");
				return false;
			}
		}
		else if (!InvertByGauss(A, EPS)){
			printf("Inversion failed\n");
			return false;
		}
		break;
	case SVD_INVERSION: {
		int rankDeficit;
		printf("Inversion by SVD removed from library, inversion by Gauss\n");
		if (!InvertByGauss(A, EPS)){
			printf("Inversion failed\n");
			return false;
		}
		}
		break;
	default:
		return false;
		break;
	}
	return true;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSmatrix<T>::InvertByGauss(FRSmatrix<T> &Ainv, T EPS) const
{
	int TempInt, i, j, p; 
	int LastCol; // Last column of the augmented matrix
	double Multiplier;

	int mvarColumns = this->size();
	int mvarRows = this->begin()->size();

	assert( mvarColumns == mvarRows );

	if (Ainv.size() != mvarRows || Ainv.begin()->size() != mvarRows)
		Ainv.Define(mvarRows, mvarRows);

	FRSmatrix<T> M; // row, colums
	intVector    PVT;

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
		if(fabs(M[PVT[p]][p]) <= EPS)//
			{printf("The matrix is singular, value = %f",M[PVT[p]][p]);
			return false;
			}

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
		printf("The matrix is singular\n");
		return false;

	// Divide by diagonal of augmented matrix
	for(p = 0; p < mvarRows; p ++)
		for(j = mvarRows; j < 2 * mvarRows; j++)
			M[(int)PVT[p]][j] = M[(int)PVT[p]][j] / M[(int)PVT[p]][p];

	// Copy inverted matrix to A matrix
	for(j = 0; j < mvarRows; j++){
		i = mvarRows + j;
		for(p = 0; p < mvarRows; p++)
			Ainv[p][j] = M[(int)PVT[p]][i];
	}

	return FullRank;
}

#undef EPS

//----------------------------------------------------------------------------

template<class T>
FRSmatrix<T> & FRSmatrix<T>::Inverse( const FRSmatrix<T> &matrix, double &cond )
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

  dgeco_(a, &row, &col, pivot, &cond, z);

  if (1.0 + cond == 1.0)
    fprintf(stderr, "Warning: bad condition of matrix (%5.2e)\n", cond);

  job = 1;
  dgedi_(a, &row, &col, pivot, det, z, &job);

  ConvertFromLP( a, row, col );

  free(pivot);  free(z); free(a);
  return(*this);
}

//----------------------------------------------------------------------------

template<class T>
bool FRSmatrix<T>::SparseMultiplication(FRSmatrix<T> & A, 
                                        std::vector< FRSmatrix<T> > &obj)
{
	typename std::vector< FRSmatrix<T> >::iterator matrx;
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

template<class T>
bool FRSmatrix<T>::SparseAddition(FRSmatrix<T> & A, 
                                  std::vector< FRSmatrix<T> > &obj)
{
	typename std::vector< FRSmatrix<T> >::iterator matrx;
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

template<class T>
bool FRSmatrix<T>::SparseSubtraction(FRSmatrix<T> & A, 
                                     std::vector< FRSmatrix<T> > &obj)
{
	typename std::vector< FRSmatrix<T> >::iterator matrx;
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
template<class T>
T FRSmatrix<T>::Det() const
{
   int col = NumColumns();
   int row = NumRows();
   double det = 1;

   if (row != col)
      printf( "Determinant a non-square matrix!\n");
   
   FRSmatrix<double> temp(*this);

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
   return (T)det;
}

//----------------------------------------------------------------------------
template<class T>
int FRSmatrix<T>::Pivot(int r)
{
  int row = NumRows();
  int col = NumColumns();
  int k = r;
  T amax = -1, temp;

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
       T q = Val(k, j);
       Val(k , j) = Val(r, j);
       Val(r, j) = q;
     }  
     return k;
  }
  return 0;
}

//----------------------------------------------------------------------------
template<class T>
void FRSmatrix<T>::Print(char *format)const
{
        FRSmatrix<T>* locRef = (FRSmatrix<T>*)this;
	// ugly trick, in the above line the const-ness is casted away
	if( this->empty() )
		return;

	typename FRSmatrix<T>::iterator	row;
	typename FRSvector<T>::iterator	col;
	int n_rows = this->size();
	int n_cols = this->begin()->size();

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
template<class T>
void FRSmatrix<T>::Print()const
{
	Print( (char *) " %8.2f" );
}

//----------------------------------------------------------------------------
template<class T>
std::ostream& operator << (std::ostream& s, const FRSmatrix<T>& mat)
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

//----------------------------------------------------------------------------

template<class T>
T MultRowColumn(const FRSmatrix<T> &b1, int i, 
                const FRSmatrix<T> &b2, int j)
{
   T sum=(T)0.;

   assert(b1.NumColumns() == b2.NumRows());
   assert(i >= 0 && i < b1.NumRows());
   assert(j >= 0 && j < b2.NumColumns());

   for(int k = 0; k < b1.NumColumns(); k++)
     sum += b1[i][k] * b2[k][ j];

   return(sum);
}

#endif

