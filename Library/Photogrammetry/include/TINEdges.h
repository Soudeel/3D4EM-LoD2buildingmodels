
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
 * \brief Interface to Class TINEdges - An array of TINEdgeSets which, for each point,
 *                          gives a clock-wise listing of numbers of points
 *                         connected to that point by an edge in the TIN
 *
 */
/*!
 * \class TINEdges
 * \ingroup TIN
 * \brief Interface to Class TINEdges - An array of TINEdgeSets which, for each point,
 *                          gives a clock-wise listing of numbers of points
 *                         connected to that point by an edge in the TIN
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A vector of clock-wise listings of neighbouring TIN nodes
 * For each node of a TIN, this edge set lists the point numbers of all
 *   nodes that are connected to this node by one TIN edge. The nodes are
 *   listed in clock-wise order.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _TINEdges_h_
#define _TINEdges_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  TINEdges              - An array of TINEdgeSets which, for each point,
                          gives a clock-wise listing of numbers of points
                          connected to that point by an edge in the TIN

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "TINEdgeSet.h"
#include "TIN.h"

class LineTopologies;

//------------------------------------------------------------------------------
/// A vector of clock-wise listings of neighbouring TIN nodes
/** For each node of a TIN, this edge set lists the point numbers of all
    nodes that are connected to this node by one TIN edge. The nodes are
    listed in clock-wise order.
*/
//------------------------------------------------------------------------------

class TINEdges : public std::vector<TINEdgeSet> {

  public:

    /// Default constructor
    TINEdges() : std::vector<TINEdgeSet>() {}

    /// Copy constructor
    TINEdges(const TINEdges &edges) : std::vector<TINEdgeSet>()
      {*this = edges;}

    /// Construct the edges from a TIN
    /** @param tin TIN for which the edge set should be constructed */
    TINEdges(const TIN &tin) : std::vector<TINEdgeSet>()
      {Derive(tin);}

    /// Default destructor
    ~TINEdges()
      {if (!empty()) erase(begin(), end());}

    /// Return the reference
    TINEdges & TINEdgesRef()
      {return *this;}

    /// Return the const reference
    const TINEdges & TINEdgesRef() const
      {return *this;}

    /// Copy assignment
    TINEdges & operator = (const TINEdges &);

    /// Derive the edges from a TIN
    /** @param tin TIN for which the edge set should be constructed */
    void Derive(const TIN &tin);

    /// Write the edges as IndexedLineSets to a VRML file
    /** Note that the coordinates of the edge nodes should have been written
        prior to the call to this function.
        @param vrml File pointer to an open VRML file. 
    */
    void VRML_Write(FILE *vrml) const;

    /// Integrate other TIN edges in this set of TIN edges
    /** All edges that are not yet part of a TIN edge set are appended.
        Note that the size of this and the added set needs to be the same.
        @param extra_edges Edges to be added to this edge sets
    */
    void Add(const TINEdges &extra_edges);

    /// Erase all edges
    void Erase();

    /// Convert the edges to a regular topology file
    LineTopologies * EdgeTopologies() const;
};
#endif /* _TINEdges_h_ */   /* Do NOT add anything after this line */
