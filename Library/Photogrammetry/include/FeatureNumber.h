
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
 * \brief Interface to Class FeatureNumber - Number of a point, line or other feature
 *
 */
/*!
 * \class FeatureNumber
 * \ingroup Photogrammetry
 * \brief Interface to Class FeatureNumber - Number of a point, line or other feature
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Number of any kind of feature (point, line, triangle, etc.)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _FeatureNumber_h_
#define _FeatureNumber_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  FeatureNumber      - Number of a point, line or other feature

--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
/// Feature number
/** Number of any kind of feature (point, line, triangle, etc.) */
//------------------------------------------------------------------------------

class FeatureNumber {

  protected:
    /// Number of the feature
    int num;

  public:

    /// Default constructor with initialisation
    FeatureNumber(int num_src = 0)
      { num = num_src; }

    /// Copy constructor
    FeatureNumber(const FeatureNumber &no)
      { num = no.num; }

    /// Destructor
    ~FeatureNumber() {};

    /// Copy assignment
    FeatureNumber & operator = (const FeatureNumber &src)
      {num = src.num; return(*this);}

    /// Prefix increment operator
    FeatureNumber& operator++()
      { num++; return *this; }

    /// Prefix decrement operator
    FeatureNumber& operator--()
      { num--; return *this; }

    /// Postfix increment operator
    FeatureNumber& operator++(int)
      { num++; return *this; }

    /// Postfix decrement operator
    FeatureNumber& operator--(int)
      { num--; return *this; }

    /// Test if two feature numbers are equal
    friend bool operator == (const FeatureNumber &n1, const FeatureNumber &n2)
      {return(n1.num == n2.num);}

    /// Test if two feature numbers are not equal
    friend bool operator != (const FeatureNumber &n1, const FeatureNumber &n2)
      {return(n1.num != n2.num);}

    /// Test if the first feature number is higher than the second
    friend bool operator > (const FeatureNumber &n1, const FeatureNumber &n2)
      {return(n1.num > n2.num);}

    /// Test if the first feature number is lower than the second
    friend bool operator < (const FeatureNumber &n1, const FeatureNumber &n2)
      {return(n1.num < n2.num);}

    /// Return the readable number
    int Number() const
      { return num; }
      
    /// Return the writable number
    int &Number()
      { return num; } 	  
      
    /// Return the readable reference
    const FeatureNumber &NumberRef() const
      { return *this; }  
      
    /// Return the writable reference
    FeatureNumber &NumberRef()
      { return *this; }  
};
#endif /* _FeatureNumber_h_ */   /* Do NOT add anything after this line */
