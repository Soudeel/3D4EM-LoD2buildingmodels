
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



#include "Timer.h"

double Timer::Minutes() const
{
  return Seconds() / 60.0;
}

double Timer::Seconds() const
{
  clock_t new_time;

  new_time = clock();
  return (double) (new_time - start_time) / CLOCKS_PER_SEC;
}

char * Timer::TimeString() const
{
  char   *string, *ch;
  double seconds;
  int    hours, minutes, i;

  string = new char[12];
  seconds = Seconds();
  hours = (int) seconds / 3600;
  seconds -= hours * 3600;
  minutes = (int) seconds / 60;
  seconds -= minutes * 60;
  sprintf(string, "%2d:%2d:%5.2f", hours, minutes, seconds);
  for (i=1, ch=string+1; i<11; i++, ch++)
    if (*ch == ' ') *ch = '0';
  return string;
}
