
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
 * \brief Class LineTopPtrVector - A vector of pointers to LineTopology
 *
 */
/*!
 * \class LineTopPtrVector
 * \ingroup Photogrammetry
 * \brief Class LineTopPtrVector - A vector of pointers to LineTopology
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

#ifndef LINETOPPTRVECTOR_H
#define LINETOPPTRVECTOR_H

#include <vector>
#include "LineTopologies.h"

class LineTopPtrVector : public std::vector <LineTopology *>
{
  public:
   LineTopPtrVector() : std::vector <LineTopology *>() {}

   ~LineTopPtrVector() {};

   void Clear()
     {if (!empty()) erase(begin(), end());}
};

#endif // LINETOPPTRVECTOR_H
