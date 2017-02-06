
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
 * \brief Interface to Class PointNumber - Number of a point
 *
 */
/*!
 * \class PointNumber
 * \ingroup Photogrammetry
 * \brief Interface to Class PointNumber - Number of a point
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _PointNumber_h_
#define _PointNumber_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  PointNumber      - Number of a point

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "FeatureNumber.h"

//------------------------------------------------------------------------------
/// A number of a point
//------------------------------------------------------------------------------

class PointNumber : public FeatureNumber {

  public:

    /// Construct with optional initial value
    PointNumber(int num_src = 0)
      : FeatureNumber(num_src) {}

    /// Copy constructor
    PointNumber(const PointNumber &n)
      : FeatureNumber(n.num) {}

    /// Destructor
    ~PointNumber() {};

    /// Copy assignament
    PointNumber& operator=(const PointNumber &n)
       { num = n.num; return *this; }   
      
    /// Return the writable reference
    PointNumber &NumberRef()
      { return *this; }    
      
    /// Return the readable reference
    const PointNumber &NumberRef() const
      { return *this; }    
};
#endif /* _PointNumber_h_ */   /* Do NOT add anything after this line */
