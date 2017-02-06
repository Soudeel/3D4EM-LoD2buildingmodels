
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
 * \brief Interface to Class VectorPoint - A template class for different classes of points
 *
 */
/*!
 * \class VectorPoint
 * \ingroup Photogrammetry
 * \brief Interface to Class VectorPoint - A template class for different classes of points
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg and \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Generic class for vectors of different kind of numbered points
 * This class is used for several operations that are based on point
 *   numbers.
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _VectorPoint_h_
#define _VectorPoint_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  VectorPoint  - A template class for different classes of points

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                  Include files
--------------------------------------------------------------------------------
*/
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "PointNumber.h"

/*
--------------------------------------------------------------------------------
              Declaration of global function used for sorting
--------------------------------------------------------------------------------
*/

int ComparePointNumbers(const void *point1, const void *point2);

/*
--------------------------------------------------------------------------------
                                  VectorPoint  - Declaration
--------------------------------------------------------------------------------
*/

template <class Type>

/// Generic class for vectors of different kind of numbered points
/** This class is used for several operations that are based on point
    numbers. */

class VectorPoint : public std::vector <Type>
{
public:

    /// Default constructor
    VectorPoint() : std::vector<Type>() {}

    /// Destructor
    ~VectorPoint() {};

    /// Return the index of the point with the specified point number in an unsorted point vector
    /**
        @param ptnr Point number of the point to be found
        @return Index of the point number in the vector. In case the point is
                not found, -1 is returned.
    */
    int FindUnOrderedPoint(const PointNumber &ptnr) const
    {
       int nr = ptnr.Number();
       typename VectorPoint<Type>::const_iterator point;
       for(point = this->begin(); point != this->end(); point++)
        if (point->Number() == nr) return point - this->begin();
       return -1;
    }

    /// Return the index of the point with the specified point number in a sorted point vector
    /**
        @param ptnr Point number of the point to be found
        @return Index of the point number in the vector. In case the point is
                not found, -1 is returned.
    */
    int FindOrderedPoint(const PointNumber &ptnr) const
    {
      int nr = ptnr.Number(), guess_nr, dif, index_first, index_last,
          index_guess;
      typename VectorPoint<Type>::const_iterator first, last, guess;

      if (!this->size()) return -1;

      // First guess, assume point numbers are subsequent and start at zero
      if (nr >= 0 && nr < (int) this->size()) {
        guess = this->begin() + nr;
        if (guess->Number() == nr) return nr;
      }

      // Search by iterative partitioning 
      index_first = 0;          first = this->begin();
      if (first->Number() == nr) return 0;
      index_last  = this->size()-1;   last  = this->begin() + index_last;
      if (first->Number() >= last->Number()) return -1; // Not ordered
      index_guess = (int) ((float) (nr  - first->Number()) *
                           (float) (index_last - index_first) /
                           (float) (last->Number() - first->Number()));
      if (index_guess < index_first) index_guess = index_first;
      if (index_guess > index_last) index_guess = index_last;
      guess = this->begin() + index_guess;
      int loop=0;
      while (index_first != index_last) {
        loop++;
        guess_nr = guess->Number();
        if (guess_nr == nr) return index_guess; // Found index 
        if (guess_nr < first->Number() ||
            guess_nr > last->Number()) return -1; // Not ordered
        dif = nr - guess_nr;
        if (dif < 0) {
          if (index_last == index_guess) index_last--;
          else index_last = index_guess;
          if (index_first < index_guess + dif) index_first = index_guess + dif;
        }
        else { // dif > 0
          if (index_first == index_guess) index_first++;
          else index_first = index_guess;
          if (index_last > index_guess + dif) index_last = index_guess + dif;
        }
        if (index_first < 0) index_first = 0;
        if (index_last >= (int) this->size()) index_last = this->size() - 1;
        first = this->begin() + index_first;
        last  = this->begin() + index_last;
        if (index_first != index_last) {
          if (first->Number() >= last->Number()) return -1; // Not ordered
          index_guess = index_first + (int) ((float) (nr  - first->Number()) *
                               (float) (index_last - index_first) /
                               (float) (last->Number() - first->Number()));
          if (index_guess < index_first) index_guess = index_first;
          if (index_guess > index_last) index_guess = index_last;
          guess = this->begin() + index_guess;
        }
      }
      guess = this->begin() + index_first;
      if (guess->Number() == nr) return index_first;
      return -1;
    }

    /// Return the index of the point with the specified point number in a point vector
    /** This function first assumes the point vector to be sorted. If the point
        is not found, the complete point vector is scanned in a sequential
        manner.
        @param ptnr Point number of the point to be found
        @return Index of the point number in the vector. In case the point is
                not found, -1 is returned.
    */
    int FindPoint(const PointNumber &ptnr) const
    {
       int index;
       index = FindOrderedPoint(ptnr);
       if (index == -1) index = FindUnOrderedPoint(ptnr);
       return index;
    }
    
    /// Check if the vector contains a specific point
    bool Contains(const PointNumber &ptnr) const
    {
        return (FindPoint(ptnr) != -1);
    }

    /// Return the pointer to the point with the specified point number
    /**
        @param ptnr Point number of the point to be found
        @return Pointer to the point with the specified point number.
                In case the point is not found, NULL is returned.
    */
    Type* GetPoint(const PointNumber& ptnr) const
    {
       int index = FindPoint(ptnr);
       if (index == -1) return NULL;
       return const_cast <Type *> (&(*(this->begin() + index)));
    }

    /// Return the const iterator of the point with the specified point number
    /**
        @param ptnr Point number of the point to be found
        @return const iterator of the point with the specified point number.
                In case the point is not found, end() is returned.
    */
    typename VectorPoint<Type>::const_iterator
      ConstPointIterator(const PointNumber& ptnr) const
    {
       int index = FindPoint(ptnr);
       if (index == -1) return this->end();
       return this->begin() + index;
    }

    /// Return the iterator of the point with the specified point number
    /**
        @param ptnr Point number of the point to be found
        @return iterator of the point with the specified point number.
                In case the point is not found, end() is returned.
    */
    typename VectorPoint<Type>::iterator PointIterator(const PointNumber& ptnr)
    {
       int index = FindPoint(ptnr);
       if (index == -1) return this->end();
       return this->begin() + index;
    }

    /// Return the point with the specified point number of a sorted vector
    /** This implementation assumes that the points are sorted with increasing
        point numbers and that all point numbers are non-negative and unique.
        @param ptnr Point number of the point to be found
        @return Pointer to the point with the specified point number.
                In case the point is not found, NULL is returned.
    */
    Type* GetOrderedPoint(const PointNumber &ptnr) const
    {
       int index = FindOrderedPoint(ptnr);
       if (index == -1) return NULL;
       return const_cast <Type *> (&(*(this->begin() + index)));
    }

    /// Return the const iterator of the point with the specified point number
    /** This implementation assumes that the points are sorted with increasing
        point numbers and that all point numbers are non-negative and unique.
        @param ptnr Point number of the point to be found
        @return const iterator of the point with the specified point number.
                In case the point is not found, end() is returned.
    */
    typename VectorPoint<Type>::const_iterator
      ConstOrderedPointIterator(const PointNumber& ptnr) const
    {
       int index = FindOrderedPoint(ptnr);
       if (index == -1) return this->end();
       return this->begin() + index;
    }

    /// Return the iterator of the point with the specified point number
    /** This implementation assumes that the points are sorted with increasing
        point numbers and that all point numbers are non-negative and unique.
        @param ptnr Point number of the point to be found
        @return iterator of the point with the specified point number.
                In case the point is not found, end() is returned.
    */
    typename VectorPoint<Type>::iterator OrderedPointIterator(const PointNumber& ptnr)
    {
       int index = FindOrderedPoint(ptnr);
       if (index == -1) return this->end();
       return this->begin() + index;
    }

    /// Add a point to the list
    PointNumber AddPoint(Type *pt)
    {
       // check if there is a point with the same point number 
       PointNumber ptnr = pt->NumberRef();
       if (FindPoint(ptnr) == -1)
       {
         push_back(*pt);
  	 return ptnr;
       }
  	
       // find a unused point number	
       for(int i = 0; ; i++)
       {
          ptnr = PointNumber(i);
          if (FindPoint(ptnr) == -1)
    	  break;
       }
  
       // create new point 
       Type pt1 = *pt;
       pt1.Number() = ptnr.Number();
  
       // add point  	     	
       push_back(pt1);
       return ptnr;
    } 
    
    /// Delete a point from the list
    int DeletePoint(PointNumber *ptnr)
    { 
      int nr = ptnr->Number();
      typename VectorPoint<Type>::iterator i;
      for(i = this->begin(); i != this->end(); i++)
      {
  	if (i->Number() == nr)
 	{ 
 	    this->erase(i);
 	    return 1;
 	}    
      }  
      return -1;
    }

    /// Sort the list after point number
    void Sort()
    {
      qsort((void *) &*(this->begin()), this->size(), sizeof(Type),
            ComparePointNumbers);
    }

    PointNumber HighestPointNumber() const
    {
      PointNumber maximum;
      typename VectorPoint<Type>::const_iterator point;

      if (this->empty()) return PointNumber(0);
      maximum = this->begin()->NumberRef();
      for (point=this->begin()+1; point!=this->end(); point++)
        if (point->NumberRef() > maximum) maximum = point->NumberRef();
      return maximum;
    }
    
    /// Erase all points
    void Erase()
    {
      this->erase(this->begin(), this->end());
    }
};
#endif /* _VectorPoint_h_ */   /* Do NOT add anything after this line */
