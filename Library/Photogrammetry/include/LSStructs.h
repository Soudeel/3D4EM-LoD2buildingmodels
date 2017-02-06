
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
 * <table border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \date          July 2002 (created)
 * \date		  Oct 2002 (modified)
 *
 * \remark Vector and least squares structures
 *
 *
 * \todo None
 *
 * \bug None
 *
  * \warning None
 */

#ifndef _vRecord_H_
#define _vRecord_H_

#include <vector>
#include <assert.h>
#include "Vector3D.h"


template <class T> bool cmpGT(T, T);
template <class T> bool cmpLT(T, T);
template <class T> bool cmpGE(T, T);
template <class T> bool cmpLE(T, T);
template <class T> bool cmpEQ(T, T);
template <class T> bool cmpNE(T, T);

template<class T>
class FRSvector : public std::vector<T>
{
public:
	// constructor, specify number of elements and their value
	// default constructor allocates no space
	FRSvector(int i = 0, T val = (T)0) : std::vector<T>()
        { assert( i>= 0 ); this->reserve( i ); 
          for (int j = 0; j < i; j++) this->push_back(val); }

	// constructor, make a copy of a Vector3D
	FRSvector(const Vector3D & rhs) : std::vector<T>()
        { Erase();
	  this->reserve(3);
	  push_back((T) rhs.X()); push_back((T) rhs.Y()); push_back((T) rhs.Z()); }

	/// constructor with a C array to the elements
	FRSvector(int i, const T* vals);

	/// destructor
	~FRSvector(){ Erase(); }

	/// copy constructor and assignment operator are not implemented
	/// these are inhereted from the parent class

	/// Conversion to a Vector3D
	/// sloppy set to false checks if vector size is 3; true handles any 
	/// case starting with the first element of *this
	Vector3D AsVector3D(bool sloppy = false);

	/// what is this for ????
	const FRSvector<T> & Vector() const {return (*this);}

	/// set the size of a vector (define it)
	void Define(int nElements, const T& val=0){
		assert(nElements > 0);
		
		if (this->size() == nElements){
			*this = val;
			return;
		}
		else{
			Erase();
			this->reserve(nElements);
			for(int j = 0; j < nElements; j++) 
				this->push_back(val);
		}

	}

	/// delete elements
	void Erase() {if (!this->empty()) this->erase(this->begin(), this->end());}

	/// assign rhs to all elements of the vector
	FRSvector<T> & operator = (T rhs);

	/// add rds to *this
	FRSvector<T> & operator += (const FRSvector<T> &rhs);

	/// add rhs to *this and return the resulting vectors
	FRSvector<T> operator + (const FRSvector<T> &rhs)const;

	/// multiply *this transposed with a matrix (vector of FRSvector)
	/// return the result
	FRSvector<T> operator * (const std::vector< FRSvector<T> > &)const;

	/// subtract rhs from *this and return the resulting vector
	FRSvector<T> operator - (const FRSvector<T> &rhs)const;

	/// subtract rhs from *this
	FRSvector<T> & operator -= (const FRSvector<T> &rhs);

	/// return vector = *this * val
	FRSvector<T> operator * (T val) const;

	/// multiply all elements by val
	FRSvector<T> & operator *= (T val);

	/// divide all elements by val
	FRSvector<T> & operator /= (T val);

	/// multiply rhs with *this
	/// Multiplication itemwise NOT DOT PRODUCT
	FRSvector<T> & operator *= (const FRSvector<T> &rhs);

	/// compute the (euclidean) norm
    /// possibly augmented by weights, given in W
    /// return value is always of type double, irrespective of typename T
	double Norm(const FRSvector<T> *W = NULL) const;

	/// compute the (euclidean) square of the norm
    /// possibly augmented by weights, given in W
     /// return value is always of type double, irrespective of typename T
	double NormSquare(const FRSvector<T> *W = NULL) const;
	
	/// normalize vector (divide by its Norm())
	void Normalize();

	/// compute the dot product (inner product) of
	/// *this and rhs. Return 0 if lengths are incompatible.
	T DotProduct(const FRSvector<T> &rhs) const;

	/// compute the minimum and maximum value (considering signs) of *this
	void MinMaxValues(T &minVal, T &maxVal) const;

	/// computing the Median value of an array
	double Median(bool isAbsolute = false) const;

	/// Returing the sum of all elements
	T Sum() const;

	/// Returing the average, passing back the Std.
	/// for Std. divide by size()-1, if only one element is given std=0.0
	double Stat(double &std) const;

	/// Computes a histogram. The lowest Class starts at lowBound, the 
	/// highest finishes at lowBound+binWidth*numBins, 
    /// the number of classes is numClasses, their width is binWidth.  
	/// Works for floating point and integer types of T. For latter
	/// the integer division (X-lowBound)/binWidth is the problematic part.
	/// The histogram is stored in histo, the return value are the class
	/// boundaries.
	/// below and above give the freuqency of the values out of bounds.
	FRSvector<T> Histogram( FRSvector<int>& histo,
                            const T& lowBound, const T& binWidth, 
                            int numBins,
                            int& numBelow, int& numAbove )const;

	/// Computes a histogram. If numClasses is even, zero is on the
	/// boundary between two classes, if numClasses is odd, it is the
	/// center of a class. The total number of classes is numClasses+2, 
	/// providing classes that range to infinity on either end.
	/// Calls the above Histogram method.
    /// May give unexpected results for int-types.
	FRSvector<T> Histogram( FRSvector<int>& histo,
                            int numBins, const T& binWidth,
                            int& numBelow, int& numAbove )const;

	/// Computes a histogram with classes of width classWidth. Concerning
	/// the default arguments for lower and upper boundary the rules are:
	/// low < upp     ... use supplied values
	/// low = upp = 0 ... make histogram starting from zero so that each
	///                   element will fall in a proper class
	/// low > upp     ... make histogram starting at lowest value in the 
	///                   vector, ending at the highest value
	/// in any case two classes ranging to + and -infinity, resp. will be 
	/// included.
	FRSvector<T> Histogram( FRSvector<int>& histo,
                            const T& binWidth, const T& lowBound = (T)0,
                            const T& uppBound = (T)0 )const;

	/// Changing the vector elements to their absolute values 
	void Absolute();

	/// Generating a signs vector (-1, 0, 1), each element is 
	/// turned into -1, 0, 1 depending on its sign. Values within 
	/// +- EPS are turned to zero
	FRSvector<T> Signs(T EPS=(T)1.e-6) const;

	/// Sorting the vector and return an index list to the original vector
	FRSvector<T> Sort(std::vector<int> &indices) const;

	/// Returning differences between consecutive elements in the vector
	FRSvector<T> AdjacentDifference() const;

	/// Counting occurance of a phenomenon in the vector
	/// where PFi2D can receive the operator that check the phenomenon
	/// returns indices of occurences
	FRSvector<int> Count(T value, bool (*op)(T, T) = cmpEQ, int fromItem = 0);

    /// count how often the phenomenon T is in the vector, where equality
    /// is related to the function cmpEQ. If the second parameter is 
    /// specified, the return value contains the indices of the equal 
    /// vector elements. Optionally, comparison can start not from the 
    /// very first item, but from fromItem.
	int CountOccurrences (T value, FRSvector<int> * = NULL, 
	                           bool (*op)(T, T) = cmpEQ, int fromItem = 0);

    /// return a vector that only has the unique values which can be found
    /// in *this. The operator != is used to determine, if two elements 
    /// are different.
    FRSvector<T> UniqueValues();

	/// add the elements of v to *this, starting at position i0
	/// return false on impossiblity
	bool AddPartial(const FRSvector<T> & v, int i0=0);

	/// replace the elements of v to *this, starting at position i0
    /// return false on impossiblity
    bool ReplacePartial(const FRSvector<T> & v, int i0=0);

	/// write as a row vector, preceeded by description
	void Print(char * description = NULL, char *format = NULL) const;

	/// write to stream s, just in the same style as vMatrix
	std::ostream& WriteToStream(std::ostream& s)const;
};

typedef FRSvector<double> vRecord;
typedef FRSvector<int> vVector;
typedef FRSvector<int>  intVector;
typedef FRSvector<long> vecLong;

#include "vRecord.tcc"

#endif
