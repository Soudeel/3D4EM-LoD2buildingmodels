
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
 * \brief Class LineTopsIterVector - A vector of LineTopologies iterators
 *
 */
/*!
 * \class LineTopsIterVector
 * \ingroup Photogrammetry
 * \brief Class LineTopsIterVector - A vector of LineTopologies iterators
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          ---- (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef LINETOPSITERVECTOR_H
#define LINETOPSITERVECTOR_H

#include <vector>
#include "LineTopologies.h"

class LineTopsIterVector: public std::vector <LineTopologies::iterator>
{
  public:
   /// Default constructor
   LineTopsIterVector() : std::vector <LineTopologies::iterator>() {}

   /// Default destructor
   ~LineTopsIterVector() {};

   /// Delete all entries
   void Clear()
     {if (!empty()) erase(begin(), end());}

   /// Check if a topology is in the vector
   bool Contains(const LineTopology &top) const;

   /// Check if a topologies iterator is part of the vector
   bool Contains(LineTopologies::const_iterator top) const;

   /// Remove a topologies iterator
   bool Remove(LineTopologies::iterator top);

   /// Return the reference
   LineTopsIterVector &LineTopsIterVectorRef()
     { return *this; }

   /// Return the const reference
   const LineTopsIterVector &LineTopsIterVectorRef() const
     { return *this; }
};
#endif // LINETOPSITERVECTOR_H
