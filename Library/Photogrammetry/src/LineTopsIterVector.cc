
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



#include "LineTopsIterVector.h"

bool LineTopsIterVector::Contains(const LineTopology &requested_top) const
{
  for (LineTopsIterVector::const_iterator top=begin(); top!=end(); top++)
    if (**top == requested_top) return true;
  return false;
}

bool LineTopsIterVector::Contains(LineTopologies::const_iterator requested_top)
  const
{
  for (LineTopsIterVector::const_iterator top=begin(); top!=end(); top++)
    if (*top == requested_top) return true;
  return false;
}

bool LineTopsIterVector::Remove(LineTopologies::iterator top_to_be_removed)
{
  for (LineTopsIterVector::iterator top=begin(); top!=end(); top++)
    if (*top == top_to_be_removed) {
      erase(top);
      return true;
    }
  return false;
}
