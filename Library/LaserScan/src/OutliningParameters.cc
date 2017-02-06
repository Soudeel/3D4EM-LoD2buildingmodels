
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
#include "OutliningParameters.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "Database4Cpp.h"

OutliningParameters::OutliningParameters()
{
  double degree = atan(1.0) / 45.0; 

  // Maximum dist between a point and an intersection line to determine segment ends
  max_dist_point_intersection = 0.8;

  // Minimum angle between preferred directions
  min_angle_directions = 10.0 * degree;
    
  // Hough space bin size for distance
  bin_size_dist = 0.5;
    
  // Hough space bin size for direction
  bin_size_dir = 3.0 * degree;
    
  // Maximum distance between a point and an outline to count as outline point
  max_dist_point_outline = 0.5;
  
  // Maximum gap size in outline segment in number of points
  max_num_pts_gap = 2;
    
  // Maximum gap size in outline segment in meters
  max_size_gap = 1.0;
    
  // Minumum number of points in an outline segment
  min_num_pts_segment = 9;
}

bool OutliningParameters::Write(const char *filename) const
{
  FILE *fd;
  
  fd = fopen(filename, "w");
  if (fd == NULL) return false;
  Write(fd, 0);
  fclose(fd);
  return true;
}

void OutliningParameters::Write(FILE *fd, int indent) const
{
  double degree = 45.0 / atan(1.0);
  
  BNF_Write_String(fd, "outliningparameters", indent, NULL);
 
  BNF_Write_Double(fd, "maxdistpointintersection", indent+2,
                   max_dist_point_intersection, "%.2f");
  BNF_Write_Double(fd, "minangledirections", indent+2,
                   min_angle_directions * degree, "%.2f");
  BNF_Write_Double(fd, "binsizedistance", indent+2, bin_size_dist, "%.2f");
  BNF_Write_Double(fd, "binsizedirection", indent+2, bin_size_dir * degree,
                   "%.2f");
  BNF_Write_Double(fd, "maxdistpointoutline", indent+2,
                   max_dist_point_outline, "%.2f");
  BNF_Write_Integer(fd, "maxnumptsgap", indent+2, max_num_pts_gap, "%d");
  BNF_Write_Double(fd, "maxsizegap", indent+2, max_size_gap, "%.2f");
  BNF_Write_Integer(fd, "minnumptssegment", indent+2, min_num_pts_segment, "%d");

  BNF_Write_String(fd, "endoutliningparameters", indent, NULL);
}

bool OutliningParameters::Read(const char *filename)
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
        // Look for the start of the outlining parameter block
        if (!strncmp(keyword, "outliningparameters", 
                     MAX(keyword_length, 19))) {
          Read(fd);
          fclose(fd);
          return true;
        }
      }
    }
  }
  printf("Error: Keyword outlining parameters not found in file %s\n",
         filename);
  return false;
}

void OutliningParameters::Read(FILE *fd)
{
  char   *buffer, *line, *keyword;
  int    keyword_length;
  double degree = 45 / atan(1.0);

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "maxdistpointintersection", MAX(keyword_length, 24)))
          max_dist_point_intersection = BNF_Double(line);
          
        else if (!strncmp(keyword, "minangledirections", MAX(keyword_length, 18)))
          min_angle_directions = BNF_Double(line) / degree;
          
        else if (!strncmp(keyword, "binsizedistance", MAX(keyword_length, 15)))
          bin_size_dist = BNF_Double(line);
          
        else if (!strncmp(keyword, "binsizedirection", MAX(keyword_length, 16)))
          bin_size_dir = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "maxdistpointoutline", MAX(keyword_length, 19)))
          max_dist_point_outline = BNF_Double(line);
          
        else if (!strncmp(keyword, "maxnumptsgap", MAX(keyword_length, 12)))
          max_num_pts_gap = BNF_Integer(line);

        else if (!strncmp(keyword, "maxsizegap", MAX(keyword_length, 10)))
          max_size_gap = BNF_Double(line);
          
        else if (!strncmp(keyword, "minnumptssegment", MAX(keyword_length, 16)))
          min_num_pts_segment = BNF_Integer(line);

        // end keyword
        else if (!strncmp(keyword, "endoutliningparameters",
                          MAX(keyword_length, 22))) {
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
