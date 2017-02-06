
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
#include <algorithm>
#include "LSStructs.h"

//----------------------------------------------------------------------------
template <> Vector3D vRecord::AsVector3D(bool sloppy)
{
	if (!sloppy) 
		assert (size() == 3);
	if (size() >= 3)
		return Vector3D( (*this)[0], (*this)[1], (*this)[2]);
	else if (size() == 2) 
		return Vector3D((*this)[0], (*this)[1], 0);
	else if (size() == 1) 
		return Vector3D((*this)[0], 0, 0);
	else
		return Vector3D();
}
//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator = (double rhs)
{
	if (!empty())
		for (int i = 0; i < size(); i++)
			(*this)[i] = rhs;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator += (const vRecord &rhs)
{
	vRecord::const_iterator item;
	int i;
	assert( !empty() && !rhs.empty() && rhs.size() == size() );
//	if (!empty() && !rhs.empty() && rhs.size() == size())
		for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
			(*this)[i] += *item;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator -= (const vRecord &rhs)
{
	vRecord::const_iterator item;
	int i;
	assert( !empty() && !rhs.empty() && rhs.size() == size() );
//	if (!empty() && !rhs.empty() && rhs.size() == size())
		for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
			(*this)[i] -= *item;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator *= (const vRecord &rhs)
{
	vRecord::const_iterator item;
	int i;
	if (!empty() && !rhs.empty() && rhs.size() == size())
		for (item = rhs.begin(), i = 0; item != rhs.end(); item++, i++)
			(*this)[i] *= *item;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator *= (double val)
{
	if (!empty())
		for (int i = 0; i < size(); i++)
			(*this)[i] *= val;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord & vRecord::operator /= (double val)
{
	if (!empty() && val != 0.)
		for (int i = 0; i < size(); i++)
			(*this)[i] /= val;
	return *this;
}

//----------------------------------------------------------------------------
template <> vRecord vRecord::operator * (double val) const
{
	vRecord res(size());
	if (!empty())
		for (int i = 0; i < size(); i++)
			res[i] = (*this)[i] * val;
	return res;
}

//----------------------------------------------------------------------------
template <> vRecord vRecord::operator * (const std::vector<vRecord> & m)const
{
	if (size() != m.size())
		return vRecord();

	int n_items = m.begin()->size();
	vRecord res(n_items);
	for (int i = 0; i < n_items; i++)
		for (int j = 0; j < size(); j++)
			res[i] += (*this)[j] * m[j][i];
	return res;
}


//----------------------------------------------------------------------------
template <> vRecord vRecord::operator + (const vRecord &rhs)const
{
	vRecord	res;

	if (empty() || size() != rhs.size())
		return res;

	res.reserve(size());
	for(int i = 0; i < size(); i++)
		res.push_back((*this)[i] + rhs[i]);

	return res;
}

//----------------------------------------------------------------------------
template <> vRecord vRecord::operator - (const vRecord &rhs)const
{
	vRecord	res;

	if (empty() || size() != rhs.size())
		return res;

	res.reserve(size());
	for(int i = 0; i < size(); i++)
		res.push_back((*this)[i] - rhs[i]);

	return res;
}

//----------------------------------------------------------------------------
template <> double vRecord::Norm(const vRecord *W) const
{
	double norm = 0.;
	if (W != NULL && !W->empty() && W->size() == size())
		for (int i = 0; i < size(); i++)
			norm += (*this)[i] * (*this)[i] * (*W)[i];
	else
		for (int i = 0; i < size(); i++)
			norm += (*this)[i] * (*this)[i];
	return sqrt( norm );
}

template <> void vRecord::Normalize()
{
	double length = Norm();
	assert( length );
	*this /= length;
}
//----------------------------------------------------------------------------
template <> double vRecord::DotProduct(const vRecord &rhs) const
{
	if (empty() || size() != rhs.size())
		return 0;

	double norm = 0.;
	for (int i = 0; i < size(); i++)
		norm += (*this)[i] * rhs[i];
	return norm;
}

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
template <> void vRecord::MinMaxValues(double &minVal, double &maxVal) const
{
	vRecord::const_iterator index;
	index = std::min_element(begin(), end());
	minVal = *index;
	index = std::max_element(begin(), end());
	maxVal = *index;
}

//----------------------------------------------------------------------------
template <> void vRecord::Print(char * description, char *format) const
{
	if (empty())
		return;
	vRecord::const_iterator elem;
	if (description != NULL)
		printf ("%s = ", description);
	for (elem = begin(); elem != end(); elem++)
		printf ("%.3f ", *elem);
	printf("\n");
}


template <> std::ostream& vRecord::WriteToStream(std::ostream& s)const
{
 	s << size() << "\n";
	for( int r=0; r<size(); r++ )
	{
		s << (*this)[r] << "  ";
	}
	s << "\n";
	return s;
}

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
