
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
 * \brief Interface to Class PointNumberList - A list of point numbers (in a vector, though)
 *
 */
/*!
 * \class PointNumberList
 * \ingroup Photogrammetry
 * \brief Interface to Class PointNumberList - A list of point numbers (in a vector, though)
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A vector of point numbers This vector is used for polygons, but also to indicate all points in a
 *  connected component of a TIN.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _PointNumberList_h_
#define _PointNumberList_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  PointNumberList  - A list of point numbers (in a vector, though)

--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "VectorPoint.h"
#include "PointNumber.h"


//------------------------------------------------------------------------------
// A vector of point numbers
/* This vector is used for polygons, but also to indicate all points in a
    connected component of a TIN.
*/
//------------------------------------------------------------------------------

class PointNumberList : public VectorPoint<PointNumber>
{
  public:

    /// Default constructor
    PointNumberList() : VectorPoint<PointNumber>() {}

    /// Copy constructor
    PointNumberList(const PointNumberList &);

    /// Destructor
    ~PointNumberList() {if (size()) erase(begin(), end());}

    /// Copy assignment
    PointNumberList & operator = (const PointNumberList &);

    /// Return the writable reference
    PointNumberList &PointNumberListReference()
      {return(*this);}

    /// Return the readable reference
    const PointNumberList &PointNumberListReference() const
      {return(*this);}

    /// Write polygon or face to a VRML file
    /** @param fd File descriptor of an opened VRML file
        @param as_face if 0, points are written as an IndexedLineSet. Otherwise,
                       closed polygons are written as a IndexedFaceSet.
    */
    void VRML_Write(FILE *fd, int as_face) const;

    /// Test membership of point number
    /** @return 1 if point number is found in the point number list,
                0 otherwise.
    */
    int Contains(const PointNumber &) const;

    /// Test if all points are present in a point number list
    /** @return 1 if all point numbers are found in the list, 0 otherwise.
    */
    int IsPartOf(const PointNumberList &list) const;

    /// Test if some points are present in a point number list
    /** @return 1 if some point numbers are found in the list, 0 otherwise.
    */
    int OverlapsWith(const PointNumberList &list) const;

    /// Return the first point number
    const PointNumber &FirstPointNumber() const
      {return begin()->NumberRef();}

    /// Return the first point number
    const PointNumber &LastPointNumber() const
      {return (end()-1)->NumberRef();}

    /// Erase all points from the list
    void Erase()
      {PointNumberList dummy;
       if (size()) erase(begin(), end());
       swap(dummy);}

    /// Insert point numbers into the list if they're not yet member
    int Add(const PointNumberList &list);
    
    /// Remove a point number
    int Remove(const PointNumber &number);
};
#endif /* _PointNumberList_h_ */   /* Do NOT add anything after this line */
