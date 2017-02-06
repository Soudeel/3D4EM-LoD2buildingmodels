
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
 * \brief Interface to Class TINEdgeSet - A clock-wise listing of numbers of points connected to this point by an edge in the TIN
 *
 */
/*!
 * \class TINEdgeSet
 * \ingroup TIN
 * \brief Interface to Class TINEdgeSet - A clock-wise listing of numbers of points connected to this point by an edge in the TIN
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A clock-wise listing of neighbouring TIN nodes
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _TINEdgeSet_h_
#define _TINEdgeSet_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  TINEdgeSet              - A clock-wise listing of numbers of points
                            connected to this point by an edge in the TIN

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "PointNumberList.h"
#include "TIN.h"

//------------------------------------------------------------------------------
/// A clock-wise listing of neighbouring TIN nodes
//------------------------------------------------------------------------------

class TINEdgeSet : public PointNumberList {

  public:

    /// Default constructor
    TINEdgeSet() : PointNumberList() {}

    /// Construct from a point number list
    TINEdgeSet(const PointNumberList &list) : PointNumberList()
      {insert(begin(), list.begin(), list.end());}

    /// Default destructor
    ~TINEdgeSet()
      {if (!empty()) erase(begin(), end());}

    /// Return the reference
    TINEdgeSet & TINEdgeSetRef()
      {return *this;}

    /// Return the const reference
    const TINEdgeSet & TINEdgeSetRef() const
      {return *this;}

    /// Copy assignment
    TINEdgeSet & operator = (const TINEdgeSet &);

    /// Derive the edge set from a TIN
    /** @param tin      The TIN from with an edge set should be extracted
        @param point_nr Point for which the edge set should be extracted
        @param mesh_nr  A TIN mesh that contains the point of interest
    */
    void Derive(const TIN &tin, const PointNumber &point_nr,
                const MeshNumber &mesh_nr);

    /// Add edges of another set to this set
    /** The extra edges are appended to this set. If already present, edges
        are not duplicated.
        @param extra_edges Edges that are to be added to this.
    */
    void Add(const TINEdgeSet &extra_edges);
};
#endif /* _TINEdgeSet_h_ */   /* Do NOT add anything after this line */
