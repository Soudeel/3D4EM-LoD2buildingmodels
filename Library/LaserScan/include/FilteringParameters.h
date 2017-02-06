
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



#ifndef FILTERINGPARAMETERS_H
#define FILTERINGPARAMETERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LaserPoint.h"

class FilteringParameters
{
  protected:

// Parameters controlling the point wise filtering

    /// Allowed height difference at same location
    double height_dif_at_same_location;

    /// Filter radius
    double filter_radius;

    /// Allowed height difference at filter radius
    double height_dif_at_filter_radius;

    /// Remove non-ground points after filtering
    bool remove_non_ground_points;
    
  public:
    /// Default constructor
    FilteringParameters();
    
    /// Return the const reference
    const FilteringParameters & FilteringParametersRef() const {return *this;}

    /// Return the reference
    FilteringParameters & FilteringParametersRef() {return *this;}

    /// Return the allowed height difference at the same location (readable)
    double HeightDifAtSameLocation() const {return height_dif_at_same_location;}
    
    /// Return the filter radius (readable)
    double FilterRadius() const {return filter_radius;}
    
    /// Return the allowed height difference at the filter radius (readable)
    double HeightDifAtFilterRadius() const {return height_dif_at_filter_radius;}
    
    /// Return the remove non-ground points switch (readable)
    bool RemoveNonGroundPoints() const {return remove_non_ground_points;}
    
    /// Return the allowed height difference at the same location (writable)
    double & HeightDifAtSameLocation() {return height_dif_at_same_location;}
    
    /// Return the filter radius (writable)
    double & FilterRadius() {return filter_radius;}
    
    /// Return the allowed height difference at the filter radius (writable)
    double & HeightDifAtFilterRadius() {return height_dif_at_filter_radius;}
    
    /// Return the remove non-ground points switch (writable)
    bool & RemoveNonGroundPoints() {return remove_non_ground_points;}
    
    /// Write parameter values in BNF style
    bool Write(const char *filename) const;
    
    /// Write parameter values in BNF style
    void Write(FILE *fd, int indent) const;
    
    /// Read parameter values from a BNF file
    bool Read(const char *filename);
    
    /// Read parameter values from a BNF file
    void Read(FILE *fd);
};

#endif // FILTERINGPARAMETERS_H
