
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class PointNumberList          

 Initial creation
 Author : George Vosselman
 Date   : 19-05-1999

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "PointNumberList.h"


PointNumberList::PointNumberList(const PointNumberList &list)
  : VectorPoint<PointNumber>()
{
  if (!list.empty()) insert(begin(), list.begin(), list.end());
}


/*
--------------------------------------------------------------------------------
                             Copy assignment
--------------------------------------------------------------------------------
*/

PointNumberList & PointNumberList::operator = (const PointNumberList &list)
{
  // Check for self assignment
  if (this == &list) return *this;

  if (!empty()) erase(begin(), end());
  if (!list.empty()) insert(begin(), list.begin(), list.end());
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Write polygon to VRML file
--------------------------------------------------------------------------------
*/

void PointNumberList::VRML_Write(FILE *fd, int show_as_face_if_closed) const
{
  PointNumberList::const_iterator point;

  if (size() < 2) return;
  if (*begin() == *(end()-1) && show_as_face_if_closed)
    fprintf(fd, "IndexedFaceSet { coordIndex [");
  else
    fprintf(fd, "IndexedLineSet { coordIndex [");
  for (point=begin(); point!=end()-1; point++)
    fprintf(fd, "%i,", point->Number());
  if (*begin() == *(end()-1)) fprintf(fd, "-1]}\n");
  else fprintf(fd, "%i,-1]}\n", (end()-1)->Number());
}

/*
--------------------------------------------------------------------------------
                         Test membership of point number
--------------------------------------------------------------------------------
*/

int PointNumberList::Contains(const PointNumber &number) const
{
  PointNumberList::const_iterator item;

  for (item=begin(); item!=end(); item++) if (*item == number) return(1);
  return(0);
}

/*
--------------------------------------------------------------------------------
                    Test if all points are part of another list
--------------------------------------------------------------------------------
*/

int PointNumberList::IsPartOf(const PointNumberList &list) const
{
  PointNumberList::const_iterator item;

  for (item=begin(); item!=end(); item++)
    if (!list.Contains(item->NumberRef())) return 0;
  return 1;
}

/*
--------------------------------------------------------------------------------
                    Test if some points are part of another list
--------------------------------------------------------------------------------
*/

int PointNumberList::OverlapsWith(const PointNumberList &list) const
{
  PointNumberList::const_iterator item;

  for (item=begin(); item!=end(); item++)
    if (list.Contains(item->NumberRef())) return 1;
  return 0;
}

/*
--------------------------------------------------------------------------------
             Add point numbers to the list that are not yet member
--------------------------------------------------------------------------------
*/

int PointNumberList::Add(const PointNumberList &list)
{
  PointNumberList::const_iterator item;
  int                             num_added=0;

  for (item=list.begin(); item!=list.end(); item++)
    if (!Contains(*item)) {
      push_back(*item);
      num_added++;
    }
  return num_added;
}

/*
--------------------------------------------------------------------------------
                   Remove a point number from the list
--------------------------------------------------------------------------------
*/

int PointNumberList::Remove(const PointNumber &number)
{
  PointNumberList::iterator item;

  for (item=begin(); item!=end(); item++)
    if (*item == number) {
      erase(item);
      return 1;
    }
  return 0;
}
