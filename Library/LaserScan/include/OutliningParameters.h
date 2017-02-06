
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



#ifndef OUTLININGPARAMETERS_H
#define OUTLININGPARAMETERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class OutliningParameters
{
  protected:

    // Maximum dist between a point and an intersection line to determine segment ends
    double max_dist_point_intersection;

    /// Minimum angle between preferred directions
    double min_angle_directions;
    
    /// Hough space bin size for distance
    double bin_size_dist;
    
    /// Hough space bin size for direction
    double bin_size_dir;
    
    /// Maximum distance between a point and an outline segment to count as outline point
    double max_dist_point_outline;
    
    /// Maximum gap size in outline segment in number of points
    int max_num_pts_gap;
    
    /// Maximum gap size in outline segment in meters
    double max_size_gap;
    
    /// Minumum number of points in an outline segment
    int min_num_pts_segment;
    
  public:
    /// Default constructor
    OutliningParameters();

    /// Return the const reference
    const OutliningParameters & OutliningParametersRef() const
      {return *this;}

    /// Return the reference
    OutliningParameters & OutliningParametersRef() {return *this;}

    /// Return the maximum distance between a point and the intersection line (readable)
    double MaximumDistancePointIntersectionLine() const
      {return max_dist_point_intersection;}
    
    /// Return the minimum angle between preferred directions (readable)
    double MinimumAnglePreferredDirections() const
      {return min_angle_directions;}
      
    /// Return the Hough bin size for the distance (readable)
    double HoughBinSizeDistance() const {return bin_size_dist;}  
      
    /// Return the Hough bin size for the direction (readable)
    double HoughBinSizeDirection() const {return bin_size_dir;}  

    /// Return the maximum distance between a point and an outline (readable)
    double MaximumDistancePointOutline() const
      {return max_dist_point_outline;}
    
    /// Return the maximum gap size in outline segment in number of points (readable)
    int MaximumPointGapInOutlineSegment() const {return max_num_pts_gap;}
    
    /// Return the maximum gap size in outline segment in meters (readable)
    double MaximumGapSizeInOutlineSegment() const {return max_size_gap;}
    
    /// Return the minumum number of points in an outline segment (readable)
    int MinimumNumberOfPointsInOutlineSegment() const {return min_num_pts_segment;}

    /// Return the maximum distance between a point and the intersection line (writable)
    double & MaximumDistancePointIntersectionLine()
      {return max_dist_point_intersection;}
    
    /// Return the minimum angle between preferred directions (writable)
    double & MinimumAnglePreferredDirections() {return min_angle_directions;}
      
    /// Return the Hough bin size for the distance (writable)
    double & HoughBinSizeDistance() {return bin_size_dist;}  
      
    /// Return the Hough bin size for the direction (writable)
    double & HoughBinSizeDirection() {return bin_size_dir;}  

    /// Return the maximum distance between a point and an outline (writable)
    double & MaximumDistancePointOutline() {return max_dist_point_outline;}
    
    /// Return the maximum gap size in outline segment in number of points (writable)
    int & MaximumPointGapInOutlineSegment() {return max_num_pts_gap;}
    
    /// Return the maximum gap size in outline segment in meters (writable)
    double & MaximumGapSizeInOutlineSegment() {return max_size_gap;}
    
    /// Return the minumum number of points in an outline segment (writable)
    int & MinimumNumberOfPointsInOutlineSegment() {return min_num_pts_segment;}

    /// Write parameter values in BNF style
    bool Write(const char *filename) const;
    
    /// Write parameter values in BNF style
    void Write(FILE *fd, int indent) const;
    
    /// Read parameter values from a BNF file
    bool Read(const char *filename);
    
    /// Read parameter values from a BNF file
    void Read(FILE *fd);
};

#endif // OUTLININGPARAMETERS_H
