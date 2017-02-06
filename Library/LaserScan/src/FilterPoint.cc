
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
 Additional collection of functions for class LaserPoint
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "LaserPoint.h"
#include "Plane.h"

/*
--------------------------------------------------------------------------------
                      Retrieval of specific attributes
--------------------------------------------------------------------------------
*/

bool LaserPoint::Filtered() const { return Attribute(IsFilteredTag) == 1; }
void LaserPoint::SetFiltered() { Attribute(IsFilteredTag) = 1; }
void LaserPoint::SetUnFiltered() { Attribute(IsFilteredTag) = 0; }
bool LaserPoint::Processed() const { return Attribute(IsProcessedTag) == 1; }
void LaserPoint::SetProcessed() { Attribute(IsProcessedTag) = 1; }
void LaserPoint::SetUnProcessed() { Attribute(IsProcessedTag) = 0; }
int LaserPoint::Label() const { return Attribute(LabelTag); }
void LaserPoint::Label(int label) { Attribute(LabelTag) = label; }

/*
--------------------------------------------------------------------------------
                            Slope based filtering
--------------------------------------------------------------------------------
*/

int LaserPoint::FilterOnSlope(LaserPoint *nb_point, double range,
                              double slope, double stdev)
{
  double distance2D, height_difference;

/* Determine distance and height difference */

  distance2D = (*this - *nb_point).Length2D();
  height_difference = x[2] - nb_point->x[2];

/* Try to filter both points */

  if (height_difference - 2 * sqrt(2.0) * stdev > slope * distance2D)
    SetFiltered();
  else if (height_difference + 2 * sqrt(2.0) * stdev < -slope * distance2D)
    nb_point->SetFiltered();

/* Return 1 in case point is within range and 0 otherwise */

  if (distance2D <= range) return(1);
  return(0);

/* Old filter which was restricted to points within range */
/*
  distance2D = (*this - *nb_point).Length2D();
  if (distance2D <= range) {
    height_difference = x[2] - nb_point->x[2];
    if (height_difference - 2 * sqrt(2.0) * stdev > slope * distance2D)
      SetFiltered();
    else if (height_difference + 2 * sqrt(2.0) * stdev < -slope * distance2D)
      nb_point->SetFiltered();
    return(1);
  }
  else return(0);
*/
}

int LaserPoint::WouldBeFilteredOnSlope(const LaserPoint *nb_point,
				       double slope, double stdev) const
{
  double distance2D, height_difference;

  distance2D = (*this - *nb_point).Length2D();
  height_difference = x[2] - nb_point->x[2];
  return(height_difference - 2 * sqrt(2.0) * stdev > slope * distance2D);
}

/*
--------------------------------------------------------------------------------
                            Morphological filtering
--------------------------------------------------------------------------------
*/

int LaserPoint::FilterOnMorphology(LaserPoint *nb_point, const Image & kernel,
                                   double use_range, double pixelsize_kernel,
                                   int use_stdev, double stdev,
                                   double tolerance)
{
  double      distance2D, height_difference, margin;
  float       kernel_stdev;
  const float *kernel_pixel;
  int         idist;

/* Check if the point is within the filter kernel */

  distance2D = (*this - *nb_point).Length2D();
  if (distance2D <= use_range) {
    height_difference = x[2] - nb_point->x[2];
    idist = (int) (distance2D / pixelsize_kernel);
    if (idist >= kernel.NumColumns()) idist = kernel.NumColumns() - 1;

/* Determine the allowed margin */

    margin = tolerance;
    if (use_stdev) {
      kernel_stdev = *(kernel.FloatPixel(1, idist));
      margin += 3.0 * sqrt(kernel_stdev * kernel_stdev + stdev * stdev);
    }

/* Test height difference against those allowed in the kernel. Only return
 * TRUE if neighbour point is filtered.
 */

    kernel_pixel = kernel.float_begin() + idist;
    if (height_difference > *kernel_pixel + margin) {
      SetFiltered();
      return(0);
    }
    else if (-height_difference > *kernel_pixel + margin) {
      nb_point->SetFiltered();
      return(1);
    }
    else return(0);
  }
  else return(0);
}

int LaserPoint::NeighbourWouldBeFiltered(const LaserPoint *nb_point,
                                         const Image & kernel,
                                         double use_range,
                                         double pixelsize_kernel, int use_stdev,
                                         double stdev, double tolerance) const
{
  double      distance2D, height_difference, margin;
  const float *kernel_pixel;
  float       kernel_stdev;
  int         idist;

/* Check if the point is within the filter kernel */

  distance2D = (*this - *nb_point).Length2D();
  if (distance2D <= use_range) {
    height_difference = x[2] - nb_point->x[2];
    idist = (int) (distance2D / pixelsize_kernel);
    if (idist >= kernel.NumColumns()) idist = kernel.NumColumns() - 1;

/* Determine the allowed margin */

    margin = tolerance;
    if (use_stdev) {
      kernel_stdev = *(kernel.FloatPixel(1, idist));
      margin += 3.0 * sqrt(kernel_stdev * kernel_stdev + stdev * stdev);
    }

/* Test height difference against those allowed in the kernel. Only return
 * TRUE if neighbour point would be filtered by the current point.
 */

    kernel_pixel = kernel.float_begin() + idist;
    if (-height_difference > *kernel_pixel + margin) return(1);
    else return(0);
  }
  else return(0);
}
