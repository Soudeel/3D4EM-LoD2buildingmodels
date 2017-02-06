
/*
                  Copyright 2010 University of Twente
 
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
 * \brief Class Roads - A vector of roads
 *
 */
/*!
 * \class Roads
 * \ingroup Photogrammetry
 * \brief Class Roads - A vector of roads
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          15-12-2009 (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef ROADS_H
#define ROADS_H

#include "Road.h"

class Roads : public std::vector <Road>
{
  public:
    /// Default constructor
    Roads() {};

    /// Default destructor
    ~Roads() {};

    /// Return pointer to a road
    Road * RoadPtr(const char *name);

    /// Return iterator to a road
    Roads::iterator RoadIterator(const char *name);
    
// Further useful functions may be copied from Buildings.h
};
#endif // ROADS_H
