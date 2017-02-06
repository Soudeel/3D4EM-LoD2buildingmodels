
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
#include "FittingParameters.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "Database4Cpp.h"

FittingParameters::FittingParameters()
{
  double degree = atan(1.0) / 45.0;

  /// Use current segmentation and do not resegment with global Hough transform
  use_current_segmentation = true;
  
  /// Hough space model (0 = slope percentages, 1 = slope angles)
  hough_model = 0;

  /// Maximum slope (in ratio or angle)
  slope_max = 2.0;

  /// Bin size of the slope parameters in the Hough space
  slope_bin = 0.05;

  /// Bin size of the distance parameter in the Hough space
  dist_bin = 0.2;

  /// Minimum height of roof points
  min_height = 2.0;

  /// Minimum number of points in a plane
  min_numpts = 15;

  /// Size of local maximum window for maximum selection in the Hough space
  local_max_window = 3;

  /// Maximum distance of a point to a plane
  max_dist_point_to_plane = 0.3;

  /// Minimum distance between two planes
  min_dist_planes = 0.2;

  /// Minimum angle between two planes
  min_angle_planes = 10.0 * degree;

  /// Maximum angle of a plane that should be considered horizontal
  max_angle_horizontal = 5.0 * degree;

  /// Minimum length of an intersection line inside a partition
  /// but also, the minimum distance of an intersection line to the
  /// edge of a partition
  min_dist_intersection_partition = 0.7;

  /// Minimum angle between an intersection line and the partition edges
  /// used in the collinearity test
  min_angle_intersection_partition = 10.0 * degree;

  /// Maximum distance between an intersection line and a height jump edge
  max_dist_point_to_intersection = 2.0;

  /// Minimum length of a height jump edge inside a partition
  /// but also, the minimum distance of a height jump edge to the
  /// edge of a partition
  min_dist_height_jump_partition = 1.0;

  /// Minimum angle between a height jump edge and the partition edges
  /// (or intersection lines) used in the collinearity test
  min_angle_height_jump_partition = 90.0 * degree; // DISABLES HEIGHT JUMPS!

  /// Maximum distance between a supporting point and a height jump edge
  max_dist_point_to_height_jump = 1.4;

  /// Minimum height of a height jump edge
  min_jump_height = 100.0; // DISABLES HEIGHT JUMPS!

  /// Use initial partitioning of the map outline
  use_initial_partitioning = true;
  
  /// Minimum partition size
  min_partition_size = 0.5;

  /// Minimum partition percentage;
  min_partition_percentage = 5.0;
}

void FittingParameters::Write(FILE *fd, int indent) const
{
  double degree = 45 / atan(1.0);
  
  BNF_Write_String(fd, "fittingparameters", indent, NULL);

  BNF_Write_Integer(fd, "usecurrentsegmentation", indent+2, use_current_segmentation, "%d");
  BNF_Write_Integer(fd, "houghmodel", indent+2, hough_model, "%d");
  BNF_Write_Double(fd, "slopemax", indent+2, slope_max, "%.2f");
  BNF_Write_Double(fd, "slopebin", indent+2, slope_bin, "%.2f");
  BNF_Write_Double(fd, "distbin", indent+2, dist_bin, "%.2f");
  BNF_Write_Double(fd, "minheight", indent+2, min_height, "%.2f");
  BNF_Write_Integer(fd, "minnumpts", indent+2, min_numpts, "%d");
  BNF_Write_Integer(fd, "localmaxwindow", indent+2, local_max_window, "%d");
  BNF_Write_Double(fd, "maxdistpointtoplane", indent+2, max_dist_point_to_plane, "%.2f");
  BNF_Write_Double(fd, "mindistplanes", indent+2, min_dist_planes, "%.2f");
  BNF_Write_Double(fd, "minangleplanes", indent+2, min_angle_planes * degree, "%.2f");
  BNF_Write_Double(fd, "maxanglehorizontal", indent+2, max_angle_horizontal * degree, "%.2f");
  BNF_Write_Double(fd, "mindistinterpartition", indent+2, min_dist_intersection_partition, "%.2f");
  BNF_Write_Double(fd, "minangleinterpartition", indent+2, min_angle_intersection_partition * degree, "%.2f");
  BNF_Write_Double(fd, "maxdistpointtointer", indent+2, max_dist_point_to_intersection, "%.2f");
  BNF_Write_Double(fd, "mindistjumppartition", indent+2, min_dist_height_jump_partition, "%.2f");
  BNF_Write_Double(fd, "minanglejumppartition", indent+2, min_angle_height_jump_partition * degree, "%.2f");
  BNF_Write_Double(fd, "maxdistpointtojump", indent+2, max_dist_point_to_height_jump, "%.2f");
  BNF_Write_Double(fd, "minjumpheight", indent+2, min_jump_height, "%.2f");
  BNF_Write_Integer(fd, "useinitialpartitioning", indent+2, use_initial_partitioning, "%d");
  BNF_Write_Double(fd, "minpartitionsize", indent+2, min_partition_size, "%.2f");
  BNF_Write_Double(fd, "minpartitionperc", indent+2, min_partition_percentage, "%.2f");

  BNF_Write_String(fd, "endfittingparameters", indent, NULL);
}

void FittingParameters::Read(FILE *fd)
{
  char   *buffer, *line, *keyword;
  int    keyword_length;
  double degree = 45 / atan(1.0);

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        // Parameters controlling the data organisation
        if (!strncmp(keyword, "usecurrentsegmentation", MAX(keyword_length, 22)))
          use_current_segmentation = (BNF_Integer(line) != 0);
          
        else if (!strncmp(keyword, "houghmodel", MAX(keyword_length, 10)))
          hough_model = BNF_Integer(line);
          
        else if (!strncmp(keyword, "slopemax", MAX(keyword_length, 8)))
          slope_max = BNF_Double(line);
          
        else if (!strncmp(keyword, "slopebin", MAX(keyword_length, 8)))
          slope_bin = BNF_Double(line);

        else if (!strncmp(keyword, "distbin", MAX(keyword_length, 7)))
          dist_bin = BNF_Double(line);

        else if (!strncmp(keyword, "minheight", MAX(keyword_length, 9)))
          min_height = BNF_Double(line);

        else if (!strncmp(keyword, "minnumpts", MAX(keyword_length, 9)))
          min_numpts = BNF_Integer(line);
          
        else if (!strncmp(keyword, "localmaxwindow", MAX(keyword_length, 14)))
          local_max_window = BNF_Integer(line);
          
        else if (!strncmp(keyword, "maxdistpointtoplane", MAX(keyword_length, 19)))
          max_dist_point_to_plane = BNF_Double(line);

        else if (!strncmp(keyword, "mindistplanes", MAX(keyword_length, 13)))
          min_dist_planes = BNF_Double(line);

        else if (!strncmp(keyword, "minangleplanes", MAX(keyword_length, 14)))
          min_angle_planes = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "maxanglehorizontal", MAX(keyword_length, 18)))
          max_angle_horizontal = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "mindistinterpartition", MAX(keyword_length, 21)))
          min_dist_intersection_partition = BNF_Double(line);

        else if (!strncmp(keyword, "minangleinterpartition", MAX(keyword_length, 22)))
          min_angle_intersection_partition = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "maxdistpointtointer", MAX(keyword_length, 19)))
          max_dist_point_to_intersection = BNF_Double(line);

        else if (!strncmp(keyword, "mindistjumppartition", MAX(keyword_length, 20)))
          min_dist_height_jump_partition = BNF_Double(line);

        else if (!strncmp(keyword, "minanglejumppartition", MAX(keyword_length, 21)))
          min_angle_height_jump_partition = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "maxdistpointtojump", MAX(keyword_length, 18)))
          max_dist_point_to_height_jump = BNF_Double(line);

        else if (!strncmp(keyword, "minjumpheight", MAX(keyword_length, 13)))
          min_jump_height = BNF_Double(line);

        else if (!strncmp(keyword, "useinitialpartitioning", MAX(keyword_length, 22)))
          use_initial_partitioning = (BNF_Integer(line) != 0);

        else if (!strncmp(keyword, "minpartitionsize", MAX(keyword_length, 16)))
          min_partition_size = BNF_Double(line);

        else if (!strncmp(keyword, "minpartitionperc", MAX(keyword_length, 16)))
          min_partition_percentage = BNF_Double(line);

        // end keyword
        else if (!strncmp(keyword, "endfittingparameters", MAX(keyword_length, 20))) {
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
