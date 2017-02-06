
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



#include "Roads.h"

Road * Roads::RoadPtr(const char *road_name)
{
  Roads::iterator road;
  for (road=begin(); road!=end(); road++)
    if (strcmp(road->Name(), road_name) == 0) return road->RoadPtr();
  return NULL;
}

Roads::iterator Roads::RoadIterator(const char *road_name)
{
  Roads::iterator road;
  for (road=begin(); road!=end(); road++)
    if (strcmp(road->Name(), road_name) == 0) return road;
  return end();
}
