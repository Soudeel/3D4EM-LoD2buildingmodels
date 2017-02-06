
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



#ifndef _VMATRIX_
#define _VMATRIX_

#include "LSStructs.h"

// Definition of matrix type renamed from MatrixType to Matrix_Type.
// It was conflicting with a MatrixType class in the newmat library.
//   
// GV 01-03-2013

typedef enum Matrix_type
{
	RankDef = false, /// Incremental Least Squares solution
	FullRank = true /// One step Least Squares solution
} Matrix_Type;


typedef enum Inversion_type
{
	STANDARD_INVERSION = 0, SVD_INVERSION = 1
} InversionType;

class Rotation2D;
class Rotation3D;

// The declaration of friend functions is required before friend declaration
// in FRSmatrix class definition. This also requires a first short declaration
// of the FRSmatrix template class itself.
// George Vosselman, 20-Mar-2005
template <class T>
class FRSmatrix;

// The following two declarations were moved from the end of this file to here.
template<class T>
std::ostream& operator<< ( std::ostream& s, const FRSmatrix<T>& mat );

template<class T>
T MultRowColumn( const FRSmatrix<T>& matrix1, int row1, 
                 const FRSmatrix<T>& matrix2, int col2 );
// End of modifications (GV)


template <class T>
class FRSmatrix : public std::vector< FRSvector<T> >
{
public:
	//////////////////////////////////////////////
	/// definition, construction, destruction, copying
	/// construction of special matrices (e.g. unit matrix)

	/// constructor: m x n elements (rows x cols)
	/// this is also the default constructor
	FRSmatrix(int rows=0, int cols=0, T val=(T)0)
	  { Define( rows, cols, val ); }

	/// Construct from a 2D rotation matrix
	FRSmatrix(const Rotation2D &rot);

	/// Construct from a 3D rotation matrix
	FRSmatrix(const Rotation3D &rot);

	/// Copy constructor
	FRSmatrix(const FRSmatrix<T>& rhs);

	/// constructor with a linear array of values (row sorted)
	FRSmatrix(const T *aa, int num_rows, int num_columns)
	  { ConvertFromLP( aa, num_rows, num_columns ); }

	/// Copy assignment (aka assignment operator)
	FRSmatrix & operator = (const FRSmatrix<T> &rhs);

	/// Copy assignment, with one scalar
	FRSmatrix & operator = (T rhs);

	/// destructor, release memory
	~FRSmatrix() {Erase();}

	/// Define a matrix of m_rows by m_cols
	bool Define(int m_rows, int m_cols, T val=0);

	/// Erase matrix elements
	void Erase();

	/// Define a unit matrix of m_rows by m_cols
	void Identity(int m_rows, int m_cols);

	///Copy assignment with transposition
	FRSmatrix & Transpose( const FRSmatrix<T> &b );

	/// Set this matrix to unity
	void Unit()
	  { Identity( NumRows(), NumColumns() ); }

	/// Construct a quadratic identiy matrix with row rows
	void Identity( int row )
	  { Identity ( row, row ); }

	/// Set all elements to zero
	void Zero()
	  { Define( NumRows(), NumColumns() ); }

	/////////////////////////////////////////////////
	/// conversion to/from different formats
	/// see also constructors from other elements

	/// Conversion to NR format (array of arrays)
	void Convert2NR(T ***) const;

	/// Conversion to Linpack format (one linear array)
	/// converts *this into a linear array of T
	/// the client has the responsibility to delete *a, 
        /// if it is not used anymore
	/// alternatively the signature could be (T &* a)
	void Convert2LP( T **a ) const;

	/// Conversion from NR format
	/// function not implemented, signature of method unclear
	void ConvertFromNR(FRSmatrix<T> *, double **, long, long);

	/// Conversion from LP format
	/// copy values from a (row sorted, c columns, r rows) to *this
	void ConvertFromLP( const T *a, long r, long c );

	/////////////////////////////////////////////////
	///query size

	// Not documented. Usage of NumRows is preferred.
	int NoRows() const
	  { return NumRows(); }

	/// Number of rows
	int NumRows() const
	  { return (int) this->size();}

	// Not documented. Usage of NumColumns is preferred.
	int NoCol() const
	  { return NumColumns();}

	/// Number of columns
	int NumColumns() const
	  { assert( NumRows() ); return (int) this->operator[](0).size(); }


	///////////////////////////////////////////////
	/// element access

	/// return a reference to element i,j
	/// also the square bracket operators can be used
	T & R(int i, int j){ return (*this)[i][j];}

	/// Element access for reading by row column coordinates
	T Val(int r, int c) const
	  { return (*this)[r][c]; }

	/// Element access for writing by row column coordinates
	T & Val(int r, int c)
	  { return R(r, c); }

	/// Element access for reading by index (for vectors)
	T Val(int i) const
	{ assert( NumColumns() == 1 );
	  return (*this)[i][0]; }

	/// Element access for writing by index (for vectors)
	T & Val(int i)
	{ assert( NumColumns() == 1 );
	  return R(i, 0); }

	////////////////////////////////////////////////
	///matrix operations (+-*/) with other matrices, vectors and scalars
	/// also including multiplication with transposed arguments
	/// and determinant calculation

	/// multiply this matrix with another matrix, return the result
	FRSmatrix<T> operator * (const FRSmatrix<T> &m) const;

	/// multiply this matrix with a vector, return the result
	FRSvector<T> operator * (const FRSvector<T> &m) const;

	/// add another matrix to this, return the result
	FRSmatrix<T> operator + (const FRSmatrix<T> &rhs) const;

	/// subtract another matrix from this matrix, return the result
	FRSmatrix<T> operator - (const FRSmatrix<T> &rhs) const;

	/// add another matrix to *this
	FRSmatrix<T> & operator += (const FRSmatrix<T> &rhs);

	/// subtract another matrix from *this
	FRSmatrix<T> & operator -= (const FRSmatrix<T> &rhs);

	/// multiply another matrix with *this
	FRSmatrix<T> & operator *= (const FRSmatrix<T> &m);

	/// multiply *this with a scalar
	FRSmatrix<T> & operator *= (T val);

	/// Multiply all matrix elements by a scalar (same as *=(double))
	FRSmatrix<T> & operator * (T c);

	/// Add a scalar to all matrix elements
	FRSmatrix<T> & operator + (T c);

	/// Subtract a scalar from all matrix elements
	FRSmatrix<T> & operator - (T c);

	/// Increment Diagonal by a diagonal matrix (represented as a vector)
	bool IncrementDiagonal(const FRSvector<T> &);

	/// Increment Diagonal by a diagonal matrix (represented as a constant)
	bool IncrementDiagonal(T val);

/*
	function omitted, because it is not object oriented
	/// Multiply a matrix B by a matirx A and store result in X,
	/// restricted to the upper left sub matrices of A and B 
        /// as defined in mvarARows ...
	void Multiply(vMatrix &A, vMatrix &B, vMatrix &X, 
                      int mvarARows, int mvarAColumns, int mvarBColumns);
*/
	/// store A*B in *this
	void Multiply(const FRSmatrix<T> &A, const FRSmatrix<T> &B);

	/// Multiply *this with its transposed from the left. Let A := *this ->
	/// Returning an ATA where W is an optional weight matrix
	/// with a default NULL for identity
	FRSmatrix<T> MultTranspose (const FRSvector<T> * W = NULL) const;

	/// Multiply *this transposed with the argument matrix.
	/// if the argument matrix pointer = NULL, then take *this instead
	/// vRecord is a diagonal weight matrix
	/// if the current matrix is A, and vMatrix*=NULL, 
        /// then the result is ATA
	FRSmatrix<T> Transpose_MultBy (FRSvector<T> *, FRSmatrix<T> *) const;

	/// Multiply *this with the transposed argument matrix.
	/// if the argument matrix pointer = NULL, then take *this instead
	/// if the current matrix is A then the result is AAT
	/// again, the vRecord is a diagonal weight matrix 
        /// *this * vRecord * vMatrix^T
	/// if vRecord == NULL, it is considered as an identity matrix
	FRSmatrix<T> MultByTranspose (FRSvector<T> *, FRSmatrix<T> *) const;

	/// Add the outer product (v*wT). at default w = NULL. Then adding v*vT
	/// n.b. outer product is aka dyadic product
	/// The addition is performed to a submatrix of *this, beginning at 
        /// row0, col0,
	/// the size of the submatrix is v->size()
	bool AddOuterProduct(const FRSvector<T> *v, 
                             int row0 = 0, int col0 = 0, 
                             const FRSvector<T> *w = NULL);

	/// Create *this from the outer product (v*wT). at default w = NULL. 
        /// Then creating from v*vT
	bool CreateFromOuterProduct(FRSvector<T> *v, FRSvector<T> *w = NULL);

	/// Create a projection matrix from the vector v.
    /// This is the matrix I-v*vT, I standing for identity matrix.
    /// A vector x multiplied with this matrix will then lie in the 
    /// hyperplane perpendicular to v and is the projection of x into
    /// this hyperplane.
    /// returns false, if v has length 0. In this case the matrix is
    /// the zero matrix.
    bool CreateProjectionMatrix(FRSvector<T> *v);

    /// Create a mirror matrix from the vector v.
    /// This matrix will mirror vectors on the hyperplane
    /// which has as normal v. 
    /// In 3D this can be used to rotate vectors which lie in plane
    /// perpendicular to n1 (or points in a plane through the origin) 
    /// into the plane perpendicular to n2. v = n1+n2. 
    /// The rotation axis is then n1 x n2,
    /// and the rotation angle is the angle between n1 and n2.
    /// Of course, these planes include the origin, otherwise it holds only
    /// for vectors. 
	/// returns false, if v has length 0.
    bool CreateMirrorMatrix(FRSvector<T> *v);

	/// Return the determinant of a matrix
	T Det() const;

	/// friend function declaration for computing the row of matrix1 
        /// with the column of matrix2

   	friend T MultRowColumn<T>( const FRSmatrix<T>& matrix1, int row1, 
                                   const FRSmatrix<T>& matrix2, int col2 );

	////////////////////////////////////////////////////////////////////
	///sub matrices adding and replacing
	/// concatenation (appending a row)

	/// add to the elements of *this starting at r0 and c0 those of v
	/// return false on impossiblitiy
	bool AddPartial(const FRSmatrix<T> & v, int r0 = 0, int c0 = 0);

	/// replace the elements of *this starting at r0 and c0 with those of v
	/// return false on impossiblitiy
	bool ReplacePartial(const FRSmatrix<T> &v, int r0=0, int c0=0);

	/// like ReplacePartial, but with a different name. Return the updated
	/// this-matrix. Assert correctness of arguments in debug mode.
	FRSmatrix<T> & Insert(const FRSmatrix<T> &sub, 
                              int row_offset, int col_offset)
	  { bool success = ReplacePartial( sub, row_offset, col_offset );
	    assert( success );
	    return *this; }

	/// Insert a part of a matrix at the specified location
	/// row_offset and column_offser refer to *this,
	/// the other four integers specify the submatrix of matrix to be
	/// copied to *this
	FRSmatrix<T> &Insert(const FRSmatrix<T> &matrix, 
                         int first_row, int first_column,
	                     int number_of_rows, int number_of_columns,
	                     int row_offset, int column_offset);

	void push_back( const FRSvector<T>& row )
	  { assert( !NumRows() || (int) row.size() == NumColumns() );
	    ((std::vector< FRSvector<T> >*)this)->push_back( row ); }

	////////////////////////////////////////////////////////////////////
	/// inversion

	/// Inversion by Gauss Elimination
	/// Ainv = (*this)^(-1), applying pivoting
	/// singularity is checked against EPS
	bool InvertByGauss(FRSmatrix<T> &Ainv, T EPS=(T)1.e-6) const;

	/// Inversion of a 3 by 3 Matrix
	/// singularity is checked against EPS
	bool Invert_3x3(FRSmatrix<T> &, T EPS=(T)1.e-6) const;

	/// Inversion by SVD
	/// singularity is checked against EPS
	/// the rank deficit is returned as rDeficit
	bool InvertBySVD(FRSmatrix<T> &, int& rDeficit, T EPS=(T)1.e-6) const;
	
	/// Inversion with the Linpack routines dgeco and dgedi.
	/// (*this) := matrix^(-1)
	/// dgeco performs a factorisation by Gaussian elimination.
	/// dgedi invert the factorised matrix.
	/// @param matrix Matrix to be inverted
	/// @param cond   Condition number of factorisation
	/// @return The inverted matrix (= this)
	FRSmatrix<T> &Inverse( const FRSmatrix<T> &matrix, double &cond );

	/// General interface for inversion
	bool Invert(FRSmatrix<T> &, int inversionType = STANDARD_INVERSION, T EPS = (T)1.e-6);

	/// get the singular value decomposition
	/// any Matrix A can be decomposed into U*W*Vt, where the t stands
	/// for the transposed matrix, in an (almost) unique way,
	/// with Ut*U = Vt*V = unit matrix of dimension: number of columns in A,
	/// and W as diagonal matrix. U has the same dimension as A, V is
	/// as square matrix. 
	/// Here, A (=*this) remains unchanged, eigVal holds the values of 
	/// W and eigVec is V.
	/// For quadratic symmetric matrices eigVal and eigVec are really
	/// the eigen values and (right) eigen vectors of the matrix *this.
//NR_REMOVED	void GetSingularValues(FRSvector<T>& eigVal, FRSmatrix<T>& eigVec)const;

	////////////////////////////////////////////////////////////////////
	/// sparse matrices

	/// assing *this to the multiplication of the "ordinay"
	/// matrix (A) with a sparse diagonal matrix (given as a
	/// vector of matrices)
	/// Sparse multiplication
	bool SparseMultiplication(FRSmatrix<T> & A, 
                                  std::vector< FRSmatrix<T> > &obj);

	/// *this = A + obj, see comment on SpaseMultiplication
	bool SparseAddition(FRSmatrix<T> & A, 
                            std::vector< FRSmatrix<T> > &obj);

	/// *this = A - obj, see comment on SpaseMultiplication
	bool SparseSubtraction(FRSmatrix<T> & A, 
                               std::vector< FRSmatrix<T> > &obj);


	///////////////////////////////////////////////////////////
	/// input/output (input still missing)

	/// Print matrix
	void Print()const;

	/// Print matrix with the specified format
	/// function should be const, but iterators prohibit this
        /// thus a dirty trick has been used in ::Print(char*)
	void Print(char *format)const;

	/// output with the insertion operator
	/// write first the number of rows and columns, then the matrix elements
	friend std::ostream& operator<< <T> (std::ostream& s, 
                                             const FRSmatrix<T>& mat);

protected:
	// Partial pivoting, starting at the specified row (?)
	int Pivot(int row);

};


typedef FRSmatrix<double> vMatrix;

#include "vMatrix.tcc"

#endif

