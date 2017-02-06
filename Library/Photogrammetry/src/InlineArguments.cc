
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



#include <stdio.h>
#include <string.h>
#include "InlineArguments.h"

int InlineArguments::Index(const char *wanted_arg) const
{
  int index;
  
  for (index=0; index<argc; index++)
    if (strcmp(wanted_arg, argv[index]) == 0) return index;
  return 0;
}

bool InlineArguments::Contains(const char *wanted_arg) const
{
  return Index(wanted_arg) > 0;
}

int InlineArguments::Integer(const char *wanted_arg, int default_value) const
{
  int index=Index(wanted_arg), value;
  
  if (index == 0) return default_value;
  index++;
  if (index >= argc) return default_value;
  sscanf(argv[index], "%d", &value);
  return value;
}

double InlineArguments::Double(const char *wanted_arg,
                               double default_value) const
{
  int    index=Index(wanted_arg);
  double value;
  
  if (index == 0) return default_value;
  index++;
  if (index >= argc) return default_value;
  sscanf(argv[index], "%lf", &value);
  return value;
}

float InlineArguments::Float(const char *wanted_arg,
                             float default_value) const
{
  int   index=Index(wanted_arg);
  float value;
  
  if (index == 0) return default_value;
  index++;
  if (index >= argc) return default_value;
  sscanf(argv[index], "%f", &value);
  return value;
}

const char * InlineArguments::String(const char *wanted_arg, 
                                     const char *default_value) const
{
  int index=Index(wanted_arg);
  
  if (index == 0) return default_value;
  index++;
  if (index >= argc) return default_value;
  return (const char *) argv[index];
}

char * InlineArguments::String(const char *wanted_arg) const
{
  int index=Index(wanted_arg);
  
  if (index == 0) return NULL;
  index++;
  if (index >= argc) return NULL;
  return argv[index];
}
