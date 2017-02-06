
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



#include <math.h>
#include "FilteringParameters.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "Database4Cpp.h"

FilteringParameters::FilteringParameters()
{
// Parameters controlling the point wise filtering

    /// Allowed height difference at same location
    height_dif_at_same_location = 0.1;

    /// Filter radius
    filter_radius = 10.0;

    /// Allowed height difference at filter radius
    height_dif_at_filter_radius = 3.0;
}

bool FilteringParameters::Write(const char *filename) const
{
  FILE *fd;
  
  fd = fopen(filename, "w");
  if (fd == NULL) return false;
  Write(fd, 0);
  fclose(fd);
  return true;
}

void FilteringParameters::Write(FILE *fd, int indent) const
{
  BNF_Write_String(fd, "filteringparameters", indent, NULL);
  
  // Parameters controlling the point wise filtering
  BNF_Write_Double(fd, "heightdifsamelocation", indent+2, height_dif_at_same_location, "%.2f");
  BNF_Write_Double(fd, "filterradius", indent+2, filter_radius, "%.2f");
  BNF_Write_Double(fd, "heightdiffilterradius", indent+2, height_dif_at_filter_radius, "%.2f");
  // Parameters controlling the points to be returned
  BNF_Write_Integer(fd, "removenongroundpoints", indent+2, (int) remove_non_ground_points, "%d");
  BNF_Write_String(fd, "endfilteringparameters", indent, NULL);
}

bool FilteringParameters::Read(const char *filename)
{
  char   *buffer, *line, *keyword;
  int    keyword_length;
  FILE   *fd;
  
  fd = fopen(filename, "r");
  if (fd == NULL) return false;
  
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        // Look for the start of the filtering parameter block
        if (!strncmp(keyword, "filteringparameters", 
                     MAX(keyword_length, 19))) {
          Read(fd);
          fclose(fd);
          return true;
        }
      }
    }
  }
  printf("Error: Keyword filteringparameters not found in file %s\n",
         filename);
  return false;
}

void FilteringParameters::Read(FILE *fd)
{
  char   *buffer, *line, *keyword;
  int    keyword_length;
  double degree = 45 / atan(1.0);

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        // Parameters controlling the point wise filtering
        if (!strncmp(keyword, "heightdifsamelocation", MAX(keyword_length, 21)))
          height_dif_at_same_location = BNF_Double(line);
          
        else if (!strncmp(keyword, "filterradius", MAX(keyword_length, 12)))
          filter_radius = BNF_Double(line);
          
        else if (!strncmp(keyword, "heightdiffilterradius", MAX(keyword_length, 21)))
          height_dif_at_filter_radius = BNF_Double(line);
          
        // Parameters controlling the points to be returned
        else if (!strncmp(keyword, "removenongroundpoints", MAX(keyword_length, 21)))
          remove_non_ground_points = (bool) BNF_Integer(line);
          
        // end keyword
        else if (!strncmp(keyword, "endfilteringparameters", MAX(keyword_length, 22))) {
          free(buffer);
          return;
        }

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
}
