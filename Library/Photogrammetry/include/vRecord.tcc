#ifndef __VRECORD_CC
#define __VRECORD_CC

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <numeric>
//#include "LSStructs.h"
#include "sortStruct.h"

//----------------------------------------------------------------------------
template<class T>
Vector3D FRSvector<T>::AsVector3D(bool sloppy)
{
	if (!sloppy) 
		assert (this->size() == 3);
	if (this->size() >= 3)
		return Vector3D( (*this)[0], (*this)[1], (*this)[2]);
	else if (this->size() == 2) 
		return Vector3D((*this)[0], (*this)[1], 0);
	else if (this->size() == 1) 
		return Vector3D((*this)[0], 0, 0);
	else
		return Vector3D();
}
//----------------------------------------------------------------------------
template<class T>
FRSvector<T>::FRSvector(int i, const T* vals) : std::vector<T>()
{
  assert( i>0 ); assert( vals );
  this->reserve( i ); 
  for( int j=0; j<i; j++ )
    this->operator[](j) = vals[j];
}
//----------------------------------------------------------------------------
template<class T>
FRSvector<T>& FRSvector<T>::operator = (T rhs)
{
	if (!this->empty())
		for (int i = 0; i < this->size(); i++)
			(*this)[i] = rhs;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> & FRSvector<T>::operator += (const FRSvector<T> &rhs)
{
	assert(!this->empty() && !rhs.empty() && rhs.size() == this->size());

	typename FRSvector<T>::const_iterator item;
	int i;
	for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
		(*this)[i] += *item;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> & FRSvector<T>::operator -= (const FRSvector<T> &rhs)
{
	assert( !this->empty() && !rhs.empty() && rhs.size() == this->size() );

	typename FRSvector<T>::const_iterator item;
	int i;

	for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
		(*this)[i] -= *item;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> & FRSvector<T>::operator *= (const FRSvector<T> &rhs)
{
	assert( !this->empty() && !rhs.empty() && rhs.size() == this->size() );

	typename FRSvector<T>::const_iterator item;
	int i;
	
	for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
		(*this)[i] *= *item;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> & FRSvector<T>::operator *= (T val)
{
	assert( !this->empty() );

	for (int i = 0; i < this->size(); i++)
		(*this)[i] *= val;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> & FRSvector<T>::operator /= (T val)
{
	assert( !this->empty() && val != 0. );

	for (int i = 0; i < this->size(); i++)
		(*this)[i] /= val;
	return *this;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::operator * (T val) const
{
	assert( !this->empty() );

	FRSvector<T> res(this->size());
	for (int i = 0; i < this->size(); i++)
		res[i] = (*this)[i] * val;
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::operator * (const std::vector< FRSvector<T> > & m)const
{
	assert( this->size() == m.size() );

	int n_items = m.begin()->size();
	FRSvector<T> res(n_items);
	for (int i = 0; i < n_items; i++)
		for (int j = 0; j < this->size(); j++)
			res[i] += (*this)[j] * m[j][i];
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::operator + (const FRSvector<T> &rhs)const
{
	assert( !this->empty() && this->size() == rhs.size() );

	FRSvector<T>	res;
	res.reserve(this->size());
	for(int i = 0; i < this->size(); i++)
		res.push_back((*this)[i] + rhs[i]);
	return res;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::operator - (const FRSvector<T> &rhs)const
{
	assert( !this->empty() && this->size() == rhs.size() );

	FRSvector<T>	res;
	res.reserve(this->size());
	for(int i = 0; i < this->size(); i++)
		res.push_back((*this)[i] - rhs[i]);
	return res;
}

//----------------------------------------------------------------------------
template<class T>
double FRSvector<T>::NormSquare(const FRSvector<T> *W) const
{
	double norm = 0.;
	if (W != NULL && !W->empty() && W->size() == this->size())
		for (int i = 0; i < this->size(); i++)
			norm += (*this)[i] * (*this)[i] * (*W)[i];
	else
		for (int i = 0; i < this->size(); i++)
			norm += (*this)[i] * (*this)[i];
	return norm;
}
//----------------------------------------------------------------------------

template<class T>
double FRSvector<T>::Norm(const FRSvector<T> *W) const
{
	double norm = 0.;
	if (W != NULL && !W->empty() && W->size() == this->size())
		for (int i = 0; i < this->size(); i++)
			norm += (*this)[i] * (*this)[i] * (*W)[i];
	else
		for (int i = 0; i < this->size(); i++)
			norm += (*this)[i] * (*this)[i];
	return sqrt( norm );
}

template<class T>
void FRSvector<T>::Normalize()
{
	assert( !this->empty() );

	double length = Norm();
	assert( length ); // this should typically be a throw statement
                          // rather than an assertion
	*this /= length;
}
//----------------------------------------------------------------------------
template<class T>
T FRSvector<T>::DotProduct(const FRSvector<T> &rhs)const
{
	assert( !this->empty() && this->size() == rhs.size() );

	T norm = 0.;
	for (int i = 0; i < this->size(); i++)
		norm += (*this)[i] * rhs[i];
	return norm;
}

//----------------------------------------------------------------------------
template<class T>
void FRSvector<T>::MinMaxValues(T &minVal, T &maxVal)const
{
	typename FRSvector<T>::const_iterator index;

	assert(!this->empty());

	index = std::min_element(this->begin(), this->end());
	minVal = *index;
	index = std::max_element(this->begin(), this->end());
	maxVal = *index;
}

//----------------------------------------------------------------------------^M
template<class T>
double FRSvector<T>::Median(bool isAbsolute) const
{
        assert(!this->empty());

        FRSvector<T> tmp(*this);
        if (isAbsolute == true) tmp.Absolute();

        std::sort(tmp.begin(), tmp.end());

        long medVal = (long)ceil( tmp.size()/2.0 );

        if (tmp.size() % 2 == 1)
                return (double)(   *(tmp.begin() + medVal)     );
        else
                return (double)( ( *(tmp.begin() + medVal - 1) + 
                                   *(tmp.begin() + medVal    ) ) / 2. );
}

//----------------------------------------------------------------------------
template<class T>
void FRSvector<T>::Absolute()
{
        typename FRSvector<T>::iterator itr;
        for (itr =  this->begin(); itr != this->end(); itr++)
                *itr = (T)fabs(*itr);
}

//----------------------------------------------------------------------------
template<class T>
T FRSvector<T>::Sum() const
{
        T total = (T)0;
        for (typename FRSvector<T>::const_iterator itr =  this->begin();
             itr != this->end(); itr++)
                total += *itr;

        return total;
}

//----------------------------------------------------------------------------
template<class T>
double FRSvector<T>::Stat(double &std) const
{
        assert (!this->empty());

        double avg = Sum()/(double) this->size();

        // Unbiased std. estiamte
        std  = (this->size() == 1) 
               ? 0
               : sqrt( (this->NormSquare() - (double) this->size() * avg*avg ) /
                       ((double)(this->size() - 1)));

        return avg;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::Histogram( FRSvector<int>& histo,
                                      const T& lowBound, const T& binWidth,
                                      int numBins, 
                                      int& numBelow, int& numAbove )const
{
  histo.resize( numBins, 0 );
  FRSvector<T> bounds( numBins+1 );
  numBelow=numAbove=0;
  
  typename FRSvector<T>::const_iterator itr;
  for( itr=this->begin(); itr!=this->end(); itr++ )
  {
    int index = (int)floor( ((*itr)-lowBound) / binWidth );
    if( index<0 ) 
      numBelow++;
    else if( index>=numBins )
      numAbove++;
    else
      histo[index]++;
  }

  for( int i=0; i<=numBins; i++ )
    bounds[i] = i*binWidth + lowBound;

  return bounds;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::Histogram( FRSvector<int>& histo,
                                      int numBins,
                                      const T& binWidth,
                                      int& numBelow, int& numAbove )const
{
  T low( -binWidth*(numBins/(T)2) );
  return Histogram( histo, low, binWidth, numBins, numBelow, numAbove );
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::Histogram( FRSvector<int>& histo,
                                      const T& binWidth, const T& lowBound, 
                                      const T& uppBound )const
{
  T min, max;
  int below, above;
  int numCl;
  if( lowBound < uppBound ) 
  {
    min = lowBound;
    max = uppBound;
    numCl = (int)ceil( (max-min)/binWidth );
  }
  else
  {
    MinMaxValues( min, max );
    if( lowBound == (T)0 && uppBound == (T)0 )
      min = (T)0; // use only max of data
    numCl = (int)ceil( (max-min)/binWidth );
    if( max == min+numCl*binWidth )
      numCl += 1;
  }
  return Histogram( histo, min, binWidth, numCl, below, above );
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::Signs(T tEPS) const
{
        FRSvector<T> signs(*this);
		typename FRSvector<T>::iterator itr;
        for (itr =  signs.begin(); itr != signs.end(); itr++){
			if( (T)fabs(*itr) < tEPS ) *itr =  0.;
			else if (*itr > 0)        *itr =  1.;
			else if (*itr < 0)        *itr = -1.;
        }
        return signs;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::Sort(std::vector<int> &indices) const
{
        std::vector< sortStruct<T> > tmp;
        typename std::vector< sortStruct<T> >::iterator tmpItr;
        tmp.reserve(this->size());
        
        int i = 0;
		typename FRSvector<T>::const_iterator itr;
        for (itr = this->begin(); itr != this->end(); itr++, i++)
                tmp.push_back(sortStruct<T>(*itr, i));
        std::sort(tmp.begin(), tmp.end());
        
        FRSvector<T> result;
        result.reserve(this->size());
        if (!indices.empty()) 
                indices.erase(indices.begin(), indices.end());
        indices.reserve(this->size());

        for (tmpItr = tmp.begin(); tmpItr != tmp.end(); tmpItr++){
                result.push_back(tmpItr->Item());
                indices.push_back(tmpItr->Index());
        }
        return result;
}


//----------------------------------------------------------------------------
template<class T>
FRSvector<T> FRSvector<T>::AdjacentDifference() const
{
        assert( this->size() > 1 );

        FRSvector<T> result(this->size());
        std::adjacent_difference(this->begin(), this->end(), result.begin());

        return result;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSvector<T>::AddPartial(const FRSvector<T> & v, int i0)
{
	assert( this->size() > 1 );
	if( i0 + v.size() > this->size() )
		return false;
	for( int i=0; i<v.size(); i++ )
		(*this)[i+i0] += v[i];
	return true;
}

//----------------------------------------------------------------------------
template<class T>
bool FRSvector<T>::ReplacePartial(const FRSvector<T> & v, int i0)
{
    assert( this->size() > 1 );
    if( i0 + v.size() > this->size() )
        return false;
    for( int i=0; i<v.size(); i++ )
        (*this)[i+i0] = v[i];
    return true;
}

//----------------------------------------------------------------------------
template<class T>
void FRSvector<T>::Print(char * description, char *format) const
{
	if (this->empty())
		return;
	typename FRSvector<T>::iterator elem;

	char * dformat = "%.3f ";
	char * pformat = (format == NULL)? dformat : format;
	if (description != NULL)
		printf ("%s = ", description);
	for (elem = this->begin(); elem != this->end(); elem++)
			printf(pformat, *elem);
	printf("\n");
}

//----------------------------------------------------------------------------
template<class T>
std::ostream& FRSvector<T>::WriteToStream(std::ostream& s)const
{
 	s << this->size() << "\n";
	for( int r=0; r<this->size(); r++ )
	{
		s << (*this)[r] << "  ";
	}
	s << "\n";
	return s;
}

//----------------------------------------------------------------------------
template<class T>
FRSvector<int> FRSvector<T>::Count(T value, bool (*op)(T, T), int fromItem)
{
        FRSvector<int> indices;
        int i = 0;
        typename FRSvector<T>::iterator itr = this->begin();

        if (fromItem > 0 && fromItem < this->size())
          { i += fromItem; itr += fromItem;}

        for (; itr != this->end(); itr++, i++)
                if (op(*itr, value))
                        indices.push_back(i);

        return indices;
}

//----------------------------------------------------------------------------
template<class T>
int FRSvector<T>::CountOccurrences(T value, FRSvector<int> *indices, 
								   bool (*op)(T, T), int fromItem)
{
        int count = 0;
        int i = 0;
        typename FRSvector<T>::iterator itr = this->begin();

		if (indices != NULL && !indices->empty())
			indices->erase(indices->begin(), indices->end());

        if (fromItem > 0 && fromItem < this->size())
          { i += fromItem; itr += fromItem;}

        for (; itr != this->end(); itr++, i++)
			if (op(*itr, value)){
				if (indices != NULL) indices->push_back(i);
				count++;
			}

        return count;
}

//----------------------------------------------------------------------------

template<class T>
FRSvector<T> FRSvector<T>::UniqueValues()
{
    intVector indices;
    intVector::iterator itr;
    FRSvector<T> result;

    FRSvector<T> tmp = Sort(indices);

    result.push_back(*(this->begin() + indices[0]));
    for (itr = indices.begin() + 1; itr != indices.end(); itr++){
        if (*(this->begin() + *itr) != *(this->begin() + *(itr - 1)))
            result.push_back(*(this->begin() + *itr));
    }
    return result;
}


//----------------------------------------------------------------------------
template <class T>inline
bool cmpGT(T ls , T rs){return (ls > rs) ? true : false;}
//----------------------------------------------------------------------------
template <class T> inline
bool cmpLT(T ls , T rs){return (ls < rs) ? true : false;}
//----------------------------------------------------------------------------
template <class T> inline
bool cmpGE(T ls , T rs){return (ls >= rs) ? true : false;}
//----------------------------------------------------------------------------
template <class T> inline
bool cmpLE(T ls , T rs){return (ls <= rs) ? true : false;}
//----------------------------------------------------------------------------
template <class T> inline
bool cmpEQ(T ls , T rs){return (ls == rs) ? true : false;}
//----------------------------------------------------------------------------
template <class T> inline
bool cmpNE(T ls , T rs){return (ls != rs) ? true : false;}


//----------------------------------------------------------------------------
/*bool vRecord::Multiply(const std::vector<vRecord> & m, const vRecord &v)
{
	assert (!m.empty() && m.begin()->size() != 0 &&
			!v.empty() && v.size() == m.begin()->size());

	int n_rows = m.size();
	int n_cols = m.begin()->size();
	int m_rows = v.size();

	if (size() != n_rows)
		*this = vRecord(n_rows);

	for(int i = 0; i < n_rows; i++){
		(*this)[i] = 0.;
		for(int k = 0; k < n_cols; k++)
			(*this)[i] += m[i][k] * v[k];
	}
	return true;
}*/

#endif

