
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



#ifndef FITTINGPARAMETERS_H
#define FITTINGPARAMETERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class FittingParameters
{
  protected:

// Parameters controlling the plane extraction

    /// Use current segmentation and do not resegment with global Hough transform
    bool use_current_segmentation;
  
    /// Hough space model (0 = slope percentages, 1 = slope angles)
    int hough_model;

    /// Maximum slope (in ratio or angle)
    double slope_max;

    /// Bin size of the slope parameters in the Hough space
    double slope_bin;

    /// Bin size of the distance parameter in the Hough space
    double dist_bin;

    /// Minimum height of roof points
    double min_height;

    /// Minimum number of points in a plane
    int min_numpts;

    /// Size of local maximum window for maximum selection in the Hough space
    int local_max_window;

    /// Maximum distance of a point to a plane
    double max_dist_point_to_plane;

    /// Minimum distance between two planes
    double min_dist_planes;

    /// Minimum angle between two planes
    double min_angle_planes;

    /// Maximum angle of a plane that should be considered horizontal
    double max_angle_horizontal;

// Parameters controlling the partitioning with intersection lines

    /// Minimum length of an intersection line inside a partition
    /// but also, the minimum distance of an intersection line to the
    /// edge of a partition
    double min_dist_intersection_partition;

    /// Minimum angle between an intersection line and the partition edges
    /// used in the collinearity test
    double min_angle_intersection_partition;

    /// Maximum distance between an intersection line and a height jump edge
    double max_dist_point_to_intersection;

// Parameters controlling the partitioning with height jump edges

    /// Minimum length of a height jump edge inside a partition
    /// but also, the minimum distance of a height jump edge to the
    /// edge of a partition
    double min_dist_height_jump_partition;

    /// Minimum angle between a height jump edge and the partition edges
    /// (or intersection lines) used in the collinearity test
    double min_angle_height_jump_partition;

    /// Maximum distance between a supporting point and a height jump edge
    double max_dist_point_to_height_jump;

    /// Minimum height of a height jump edge
    double min_jump_height;

// General partition parameters

    /// Use initial partitioning of the map outline
    bool use_initial_partitioning;
  
    /// Minimum partition size
    double min_partition_size;

    /// Minimum partition percentage;
    double min_partition_percentage;

  public:
    /// Default constructor
    FittingParameters();

    /// Return the const reference
    const FittingParameters & FittingParametersRef() const {return *this;}

    /// Return the reference
    FittingParameters & FittingParametersRef() {return *this;}

    /// Return the use current segmentation switch
    bool UseCurrentSegmentation() const {return use_current_segmentation;}
    
    /// Return the Hough space model (0 = slope percentages, 1 = slope angles)
    int HoughSpaceModel() const {return hough_model;}

    /// Return the maximum slope (in ratio or angle)
    double MaximumSlope() const {return slope_max;}

    /// Return the bin size of the slope parameters in the Hough space
    double SlopeBinSize() const {return slope_bin;}

    /// Return the bin size of the distance parameter in the Hough space
    double DistanceBinSize() const {return dist_bin;}

    /// Return the minimum height of roof points
    double MinimumRoofPointHeight() const {return min_height;}

    /// Return the minimum number of points in a plane
    int MinimumNumberOfPlanePoints() const {return min_numpts;}

    /// Return the size of local maximum window
    int LocalMaximumWindowSize() const {return local_max_window;}

    /// Return the maximum distance of a point to a plane
    double MaximumDistancePointToPlane() const {return max_dist_point_to_plane;}

    /// Return the minimum distance between two planes
    double MinimumDistanceTwoPlanes() const {return min_dist_planes;}

    /// Return the minimum angle between two planes
    double MinimumAngleTwoPlanes() const {return min_angle_planes;}

    /// Return the maximum angle of a plane that should be considered horizontal
    double MaximumAngleHorizontal() const {return max_angle_horizontal;}

    /// Return the minimum length of an intersection line inside a partition
    double MinimumDistanceIntersection() const
      {return min_dist_intersection_partition;}

    /// Return the minimum angle between an intersection line and the
    /// partition edges used in the collinearity test
    double MinimumAngleIntersection() const
      {return min_angle_intersection_partition;}

    /// Return the maximum distance between an intersection line and
    /// a height jump edge
    double MaximumDistancePointToIntersection() const
      {return max_dist_point_to_intersection;}

    /// Return the minimum length of a height jump edge inside a partition
    /// but also, the minimum distance of a height jump edge to the
    /// edge of a partition
    double MinimumDistanceHeightJump() const
      {return min_dist_height_jump_partition;}

    /// Return the minimum angle between a height jump edge and the partition
    /// edges (or intersection lines) used in the collinearity test
    double MinimumAngleHeightJump() const
      {return min_angle_height_jump_partition;}

    /// Return the maximum distance between a supporting point and
    /// a height jump edge
    double MaximumDistancePointToHeightJump() const
      {return max_dist_point_to_height_jump;}

    /// Return the minimum height of a height jump edge
    double MinimumJumpHeight() const {return min_jump_height;}

    /// Return the use initial partitioning switch
    bool UseInitialPartitioning() const {return use_initial_partitioning;}
    
    /// Return the minimum partition size
    double MinimumPartitionSize() const {return min_partition_size;}

    /// Return the minimum partition percentage;
    double MinimumPartitionPercentage() const {return min_partition_percentage;}
    
    /// Write to a BNF style file
    void Write(FILE *fd, int indent) const;
    
    /// Read from a BNF style file
    void Read(FILE *fd);
};

#endif // FITTINGPARAMETERS_H
