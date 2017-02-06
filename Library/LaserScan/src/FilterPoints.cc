
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
 Additional collection of functions for class LaserPoints

 void LaserPoints::SetUnFiltered()             Select all points
 void LaserPoints::SetFiltered()               Select no points
 void LaserPoints::Label(int)                  Set all point labels
 void LaserPoints::ReLabel(int, int)           Change point labels
 void LaserPoints::FilterOnSlope               Slope based filtering
   (double, double, double, TINEdges &)
 void LaserPoints::RemoveFilteredPoints()      Delete filtered points
 void LaserPoints::RemoveGroundPoints()        Delete ground points
 void LaserPoints::SelectFilteredPoints        Select the filtered points
   (LaserPoints & const)
 void LaserPoints::FilterOnMorphology          Filter on morphology
   (Image &, double, double, TINEdges &)
 void LaserPoints::FilterLowPoints             Filter extremely low points
   (Image &, double, double, TINEdges &, int)

 Initial creation
 Author : George Vosselman
 Date   : 19-05-1999

 Update #1
 Author : 
 Date   : 
 Changes: 

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
#include "LaserDataTypes.h"
#include "LaserPoints.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                  Select all points and set all labels
--------------------------------------------------------------------------------
*/

void LaserPoints::SetUnFiltered()
{
  LaserPoints::iterator point;
  for (point=begin(); point!=end(); point++) point->SetUnFiltered();
}

void LaserPoints::SetFiltered()
{
  LaserPoints::iterator point;
  for (point=begin(); point!=end(); point++) point->SetFiltered();
}

void LaserPoints::SetUnProcessed()
{
  LaserPoints::iterator point;
  for (point=begin(); point!=end(); point++) point->SetUnProcessed();
}

void LaserPoints::SetProcessed()
{
  LaserPoints::iterator point;
  for (point=begin(); point!=end(); point++) point->SetProcessed();
}

void LaserPoints::Label(int label)
{
  LaserPoints::iterator point;
  for (point=begin(); point!=end(); point++) point->Label(label);
}

int LaserPoints::ReTag(int old_value, int new_value, const LaserPointTag tag)
{
  LaserPoints::iterator point;
  int                   numpts=0;

  for (point=begin(); point!=end(); point++) {
    if (point->Attribute(tag) == old_value) {
      point->Attribute(tag) = new_value;
      numpts++;
    }
  }
  return(numpts);
}

int LaserPoints::ConditionalReTag(int old_value, int new_value,
                                  const LaserPointTag tag,
                                  int test_value, const LaserPointTag test_tag)
{
  LaserPoints::iterator point;
  int                   numpts=0;

  for (point=begin(); point!=end(); point++) {
    if (point->Attribute(tag) == old_value &&
        point->Attribute(test_tag) == test_value) {
      point->Attribute(tag) = new_value;
      numpts++;
    }
  }
  return(numpts);
}

int LaserPoints::MultiConditionalReTag(int new_value, const LaserPointTag new_tag, 
                                       int value1, const LaserPointTag tag1,
                                       int value2, const LaserPointTag tag2)
{
  LaserPoints::iterator point;
  int                   numpts=0;

  for (point=begin(); point!=end(); point++) 
  {
    if (point->Attribute(tag1) == value1 && point->Attribute(tag2) == value2) 
    {
      point->Attribute(new_tag) = new_value;
      numpts++;
    }
  }
  return(numpts);
}

/*
--------------------------------------------------------------------------------
                                 Add points 
--------------------------------------------------------------------------------
*/

int LaserPoints::AddPoints(const LaserPoints &additional_points)
{
  // Just copy the points
  insert(end(), additional_points.begin(), additional_points.end());
  // Recompute bounds, if bounds were there
  if (bounds.HasSomeData()) DeriveDataBounds(0);
  return (int) additional_points.size();
}

/*
--------------------------------------------------------------------------------
                   Add, remove or crop to points with a specific label
--------------------------------------------------------------------------------
*/

int LaserPoints::AddTaggedPoints(const LaserPoints &tagged_points, int value,
                                 const LaserPointTag tag)
{
  LaserPoints::const_iterator point;
  int                         num_added=0;

  for (point=tagged_points.begin(); point!=tagged_points.end(); point++) {
    if (point->HasAttributeValue(tag, value)) {
      push_back(*point);
      num_added++;
    }
  }
  return num_added;
}

int LaserPoints::AddTaggedPoints(LaserPoints &tagged_points, int value,
                                 const LaserPointTag tag,
                                 const LaserPointTag selecttag)
{
  LaserPoints::iterator point;
  int                   num_added=0;

  for (point=tagged_points.begin(); point!=tagged_points.end(); point++) {

    if (point->HasAttributeValue(tag, value)) {
      point->SetAttribute(selecttag, 1);
      push_back(*point);
      num_added++;

    }
  }
    
  return num_added;
}

int LaserPoints::AddTaggedPoints(LaserPoints &tagged_points,
                                 int value1, const LaserPointTag tag1,
                                 int value2, const LaserPointTag tag2,
                                 const LaserPointTag selecttag)
{
  LaserPoints::iterator point;
  int                   num_added=0;

  for (point=tagged_points.begin(); point!=tagged_points.end(); point++) {
    if (point->HasAttributeValue(tag1, value1) &&
	    point->HasAttributeValue(tag2, value2)) {
      point->SetAttribute(selecttag, 1);
      push_back(*point);
      num_added++;
    }
  }
    
  return num_added;
}

int LaserPoints::RemoveTaggedPoints(const LaserPointTag tag)
{
  LaserPoints::iterator point, goodpoint;
  int                   num_kept=0, num_removed;

  for (point=begin(), goodpoint=begin(); point!=end(); point++) {
    if (!point->HasAttribute(tag)) {
      *goodpoint = *point;
      goodpoint++;
      num_kept++;
    }
  }
  num_removed = size() - num_kept;
  erase(goodpoint, end());
  return num_removed;
}

int LaserPoints::CropTaggedPoints(const LaserPointTag tag)
{
  LaserPoints::iterator point, goodpoint;
  int                   num_kept=0, num_removed;

  for (point=begin(), goodpoint=begin(); point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      *goodpoint = *point;
      goodpoint++;
      num_kept++;
    }
  }
  num_removed = size() - num_kept;
  erase(goodpoint, end());
  return num_removed;
}

int LaserPoints::RemoveTaggedPoints(int value, const LaserPointTag tag)
{
  LaserPoints::iterator point, goodpoint;
  int                   num_kept=0, num_removed;

  for (point=begin(), goodpoint=begin(); point!=end(); point++) {
    if (!point->HasAttributeValue(tag, value)) {
      *goodpoint = *point;
      goodpoint++;
      num_kept++;
    }
  }
  num_removed = size() - num_kept;
  erase(goodpoint, end());
  return num_removed;
}

int LaserPoints::RemoveTaggedPoints(int value1, const LaserPointTag tag1,
                                    int value2, const LaserPointTag tag2)
{
  LaserPoints::iterator point, goodpoint;
  int                   num_kept=0, num_removed;

  for (point=begin(), goodpoint=begin(); point!=end(); point++) {
    if (!point->HasAttributeValue(tag1, value1) ||
	    !point->HasAttributeValue(tag2, value2)) {
      *goodpoint = *point;
      goodpoint++;
      num_kept++;
    }
  }
  num_removed = size() - num_kept;
  erase(goodpoint, end());
  return num_removed;
}

int LaserPoints::CropTaggedPoints(int value, const LaserPointTag tag)
{
  LaserPoints::iterator point, goodpoint;
  int                   num_kept=0, num_removed;

  for (point=begin(), goodpoint=begin(); point!=end(); point++) {
    if (point->HasAttributeValue(tag, value)) {
      *goodpoint = *point;
      goodpoint++;
      num_kept++;
    }
  }
  num_removed = size() - num_kept;
  erase(goodpoint, end());
  return num_removed;
}

/*
--------------------------------------------------------------------------------
                            Slope based filtering
--------------------------------------------------------------------------------
*/

void LaserPoints::FilterOnSlope(double maximum_range, double slope, 
			                    double stdev, const TINEdges &edges)
{
  LaserPoints::iterator     point, nb_point;
  PointNumberList           neighbourhood, outside;
  PointNumberList::iterator nb, node;
  int                       index, node_index;
  TINEdgeSet                neighbours;
  double                    range;

  int                   no_filtering=0, smaller_range=0, nb_size=0, num_nb=0;

/* Select all points */

  SetUnFiltered();
  Label(0);

/* Loop over all points */

  printf("Started filtering data set with %d points. This may take a while.\n",
	 size());
  neighbourhood.reserve(1000);
  outside.reserve(1000);
  for (index=0, point=begin(); point!=end(); index++, point++) {

/* Filter points without neighbours. These are points that have been
 * discarded in the triangulation, since there are other points with the same
 * X and Y coordinates.
 */

    if (edges[index].size() == 0) point->SetFiltered();

/* Calculate the maximum range for this point */

    range = (point->Z() - bounds.Minimum().Z() - 2 * stdev) / slope;
    if (range <= 0.0) no_filtering++;
    else if (range < maximum_range) smaller_range++;
    if (range > maximum_range) range = maximum_range;
    if (range > 0) {

/* Initialise the neighbourhood with the direct neighbours */

      neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
      outside.erase(outside.begin(), outside.end());
      neighbours = edges[index];
      for (nb=neighbours.begin(); /* Loop over direct neighbours */
	   (nb!=neighbours.end() && !point->Filtered());/* Stop when filtered */
	   nb++) {
        nb_point = begin() + nb->Number();
        if (point->FilterOnSlope(&*nb_point, range, slope, stdev)) {/*In range*/
          neighbourhood.push_back(*nb);         /* Add point to neighbourhood */
        }
        else {
          outside.push_back(*nb);
        }
        nb_point->Label(1);                     /* Label checked point        */
      }

/* Check the point versus the neighbours of the neighbours */

      for (node=neighbourhood.begin(), node_index=0;
           (node!=neighbourhood.end() && !point->Filtered());
           node++, node_index++) {
        neighbours = edges[node->Number()];
        for (nb=neighbours.begin();
  	     (nb!=neighbours.end() && !point->Filtered()); /*Stop if filtered */
	     nb++) {
          if (nb->Number() != index) {       /* Node is not the current point */
  	    nb_point = begin() + nb->Number();
  	    if (!nb_point->Label()) {        /* Point has not been checked yet*/
              if (point->FilterOnSlope(&*nb_point, range, slope, stdev)) {
                neighbourhood.push_back(*nb);
	        node = neighbourhood.begin() + node_index;
              }
              else {
                outside.push_back(*nb);
              }
              nb_point->Label(1);            /* Label checked point           */
            }
          }
        }
      }
      num_nb++;
      nb_size += neighbourhood.size();

/* Reset all labels */

      for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) {
        nb_point = begin() + node->Number();
        nb_point->Label(0);
      }
      for (node=outside.begin(); node!=outside.end(); node++) {
        nb_point = begin() + node->Number();
        nb_point->Label(0);
      }
    }

/* Output of counter */

    if (((index+1)/1000)*1000 == index+1 && index) {
      printf("%7d\r", index+1); fflush(stdout);
    }
  }
  printf("%6d  Done.\n", size());

/* Some statistics */

  printf("%d points required no filtering\n", no_filtering);
  printf("%d points had a smaller range\n", smaller_range);
  printf("Average neighbourhood size: %6.1f\n", (double) nb_size/num_nb);
}

/*
--------------------------------------------------------------------------------
                      Remove or select the filtered points
--------------------------------------------------------------------------------
*/

void LaserPoints::RemoveFilteredPoints()
{
  LaserPoints::iterator point, goodpoint;

  for (point=begin(), goodpoint=begin();
       point!=end();
       point++) {
    if (!point->Filtered()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
}

void LaserPoints::RemoveGroundPoints()
{
  LaserPoints::iterator point, goodpoint;

  for (point=begin(), goodpoint=begin();
       point!=end();
       point++) {
    if (point->Filtered()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
}

void LaserPoints::SelectFilteredPoints(const LaserPoints &allpoints)
{
  LaserPoints::const_iterator point;

  if (!empty()) erase(begin(), end());
  for (point=allpoints.begin(); point!=allpoints.end(); point++) {
    if (point->Filtered()) push_back(*point);
  }
}

/*
--------------------------------------------------------------------------------
                          Morphological filtering
--------------------------------------------------------------------------------
*/

void LaserPoints::FilterOnMorphology(const Image & kernel, double use_range,
				                     double stdev, double tolerance,
                                     const TINEdges &edges)
{
  LaserPoints::iterator     point, nb_point;
  PointNumberList           neighbourhood, savedpoints;
  PointNumberList::iterator nb, node;
  int                       index, node_index, use_stdev;
  TINEdgeSet                neighbours;
  double                    min_range, max_range, pixelsize_kernel, min2, max2;

  int                       nb_size=0;

/* Retrieve kernel properties */

  if (kernel.NumRows() == 1) { /* Only maximum height differences in kernel */
    kernel.Get1DLocationData(&min_range, &max_range);
    use_stdev = 0;
  }
  else { /* Standard deviations of maxima are also available */
    kernel.Get2DLocationData(&min_range, &max_range, &min2, &max2);
    use_stdev = 1;
  }
  pixelsize_kernel = max_range / kernel.NumColumns();

/* Select all points */

  SetUnFiltered();
  SetUnProcessed();

/* Loop over all points */

  printf("Started filtering data set with %d points. This may take a while.\n",
	 size());
  neighbourhood.reserve(1000);
  savedpoints.reserve(1000);
  for (index=0, point=begin(); point!=end(); index++, point++) {

/* Filter points without neighbours. These are points that have been
 * discarded in the triangulation, since there are other points with the same
 * X and Y coordinates.
 */

    if (edges[index].size() == 0) point->SetFiltered();

/* Initialise the neighbourhood with the direct neighbours */

    neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
    savedpoints.erase(savedpoints.begin(), savedpoints.end());
    neighbours = edges[index];
    for (nb=neighbours.begin(); /* Loop over direct neighbours */
         (nb!=neighbours.end());
	 nb++) {
      nb_point = begin() + nb->Number();
      if (point->FilterOnMorphology(&*nb_point, kernel, use_range,
			            pixelsize_kernel, use_stdev, stdev,
                                    tolerance)) {
        neighbourhood.push_back(*nb);  /* Add point to filtered neighbourhood */
      }
      else {
        savedpoints.push_back(*nb);
      }
      nb_point->SetProcessed();
    }

/* Check the point versus the neighbours of the neighbours */

    for (node=neighbourhood.begin(), node_index=0;
         node!=neighbourhood.end();
         node++, node_index++) {
      neighbours = edges[node->Number()];
      for (nb=neighbours.begin();
           (nb!=neighbours.end()); /*Stop if filtered */
	   nb++) {
        if (nb->Number() != index) {         /* Node is not the current point */
          nb_point = begin() + nb->Number();
          if (!nb_point->Processed()) {      /* Point has not been checked before */
            if (point->FilterOnMorphology(&*nb_point, kernel, use_range,
					  pixelsize_kernel, use_stdev,
                                          stdev, tolerance)) {
              neighbourhood.push_back(*nb);  /* Add to filtered neighbourhood */
	      node = neighbourhood.begin() + node_index;
            }
            else {
              savedpoints.push_back(*nb);
            }
            nb_point->SetProcessed();            /* Label checked point             */
          }
        }
      }
    }
    nb_size += neighbourhood.size();

/* Reset all processed status flags */

    for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) {
      nb_point = begin() + node->Number();
      nb_point->SetUnProcessed();
    }
    for (node=savedpoints.begin(); node!=savedpoints.end(); node++) {
      nb_point = begin() + node->Number();
      nb_point->SetUnProcessed();
    }

/* Output of counter */

    if (((index+1)/1000)*1000 == index+1 && index) {
      printf("%7d\r", index+1); fflush(stdout);
    }
  }
  printf("%7d  Done.\n", size());

/* Some statistics */

  printf("Average neighbourhood size: %6.1f\n", (double) nb_size/size());
  
  // Remove all processed status flags
  RemoveAttribute(IsProcessedTag);
}


/*
--------------------------------------------------------------------------------
                          Filtering extremely low points
--------------------------------------------------------------------------------
*/

void LaserPoints::FilterLowPoints(const Image & kernel, double use_range,
				  double stdev, double tolerance,
                                  const TINEdges &edges, int max_unfil_nbs)
{
  LaserPoints::iterator     point, nb_point;
  PointNumberList           neighbourhood;
  PointNumberList::iterator nb, node;
  int                       index, node_index, use_stdev, num_unfil_nbs;
  TINEdgeSet                neighbours;
  double                    min_range, max_range, pixelsize_kernel, min2, max2,
                            distance2D;
  int                       nb_size_sum=0, nb_max_size=0, num_fil=0,
                            num_double=0;

/* Retrieve kernel properties */

  if (kernel.NumRows() == 1) { /* Only maximum height differences in kernel */
    kernel.Get1DLocationData(&min_range, &max_range);
    use_stdev = 0;
  }
  else { /* Standard deviations of maxima are also available */
    kernel.Get2DLocationData(&min_range, &max_range, &min2, &max2);
    use_stdev = 1;
  }
  pixelsize_kernel = max_range / kernel.NumColumns();

/* Select all points */

  SetUnFiltered();
  Label(0);

/* Loop over all points */

  printf("Started filtering data set with %d points. This may take a while.\n",
	 size());
  neighbourhood.reserve(1000);
  for (index=0, point=begin(); point!=end(); index++, point++) {

/* Filter points without neighbours. These are points that have been
 * discarded in the triangulation, since there are other points with the same
 * X and Y coordinates.
 */

    if (edges[index].size() == 0) {
      point->SetFiltered();
      num_double++;
    }

/* Initialise the neighbourhood with the direct neighbours */

    neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
    neighbours = edges[index];
    num_unfil_nbs = 0;
    for (nb=neighbours.begin(); /* Loop over direct neighbours */
         nb!=neighbours.end() && num_unfil_nbs <= max_unfil_nbs;
	 nb++) {
      nb_point = begin() + nb->Number();
      distance2D = (*point - *nb_point).Length2D();
      if (distance2D <= use_range) {
        neighbourhood.push_back(*nb);           /* Add point to neighbourhood */
        nb_point->Label(1);
        if (!point->NeighbourWouldBeFiltered(&*nb_point, kernel, use_range,
			                     pixelsize_kernel, use_stdev, stdev,
                                             tolerance))
          num_unfil_nbs++;
      }
    }

/* Check the point versus the neighbours of the neighbours */

    for (node=neighbourhood.begin(), node_index=0;
         node!=neighbourhood.end() && num_unfil_nbs <= max_unfil_nbs;
         node++, node_index++) {
      neighbours = edges[node->Number()];
      for (nb=neighbours.begin();
           nb!=neighbours.end() && num_unfil_nbs <= max_unfil_nbs;
	   nb++) {
        if (nb->Number() != index) {         /* Node is not the current point */
          nb_point = begin() + nb->Number();
          if (!nb_point->Label()) {      /* Point has not been checked before */
            distance2D = (*point - *nb_point).Length2D();
            if (distance2D <= use_range) {
              neighbourhood.push_back(*nb);           /* Add to neighbourhood */
	      node = neighbourhood.begin() + node_index;
              nb_point->Label(1);
              if (!point->NeighbourWouldBeFiltered(&*nb_point, kernel,use_range,
					           pixelsize_kernel, use_stdev,
                                                   stdev, tolerance))
                num_unfil_nbs++;
            }
          }
        }
      }
    }
    nb_size_sum += neighbourhood.size();
    nb_max_size = MAX(nb_max_size, neighbourhood.size());

/* Filter the point if there are no more than max_unfil_nbs neighbouring points
 * that would not have been filtered by the current point.
 */

    if (num_unfil_nbs <= max_unfil_nbs) {
      point->SetFiltered();
      num_fil++;
    }

/* Reset all labels */

    for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) {
      nb_point = begin() + node->Number();
      nb_point->Label(0);
    }

/* Output of counter */

    if (((index+1)/1000)*1000 == index+1 && index) {
      printf("%7d\r", index+1); fflush(stdout);
    }
  }
  printf("%7d  Done.\n", size());

/* Some statistics */

  printf("Average neighbourhood size: %6.1f\n", (double) nb_size_sum/size());
  printf("Maximum neighbourhood size: %6.1f\n", (double) nb_max_size);
  if (num_fil) printf("Number of filtered points : %d\n", num_fil);
  if (num_double) printf("Number of double points   : %d\n", num_double);
}

/*
--------------------------------------------------------------------------------
             Set the last pulse flags based on the pulse count values
--------------------------------------------------------------------------------
*/

void LaserPoints::SetLastPulseFlags()
{
  LaserPoints::iterator point, next_point;
  
  if (empty()) return;
  for (point=begin(), next_point=point+1; next_point!=end();
       point++, next_point++)
    point->SetLastPulseFlag(point->PulseCount() >= next_point->PulseCount());
  point->SetLastPulseFlag(true);
}
