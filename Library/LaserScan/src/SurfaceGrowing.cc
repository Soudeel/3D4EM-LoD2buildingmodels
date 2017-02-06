
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
 Surface growing segmentation of a laser point cloud
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
/*
--------------------------------------------------------------------------------
                Segmentation of a point cloud by surface growing
--------------------------------------------------------------------------------
*/

#define NoSegmentNumber INT_MAX
#define UNLABELED       INT_MAX

bool LaserPoints::SurfaceGrowing(const SegmentationParameters &parameters,
                                 bool delete_edges, bool verbose,
                                 Planes *segment_planes)
{
  TINEdges                  *edges;
  TINEdges::iterator        set;
  TINEdgeSet                edgeset;
  Plane                     plane, new_plane, local_plane;
  Planes                    planes;
  Planes::iterator          current_plane, old_plane;
  int                       start_index, num_pts, num_ignored_planes,
                            num_surfaces, node_index, fit_interval,
                            surface_size, num_fits, num_hough, num_nbh_pts,
                            segment_number, old_segment_number, reflectance,
							num_old_surfaces;
  double                    dist, angle_difference, sq_error_old, sq_error_new,
                            plane_reflectance, reflectance_sum, reflectance_dif;
  PointNumberList           seed_nodes, surface_nodes, nbh_nodes,
                            competing_nodes, recalc_nodes;
  PointNumberList::iterator seed_node, surface_node, nbh_node, nbh_node2,
                            competing_node;
  LaserPoints               seed_points;
  LaserPoints::iterator     start_point, seed_point, surface_point, nbh_point,
                            nbh_point2, competing_point;
  HoughSpace                hough_space;
  bool                      found_plane, found_good_plane, add_point,
                            use_reflectance=false;
  LaserPointTag             reflectance_tag, segment_tag;

/* TESTING  
  PointNumberList           nbh_nbh_nodes;
  PointNumberList::iterator nbh_nbh_node;
  LaserPoints::iterator     nbh_nbh_point;
  int                       num_near_nbh_points;
// END TESTING */
  
  // Local variables
  double                    degree = atan(1.0) * 45.0;
  double                    max_angle_difference = 5 * degree;
  int                       min_num_pts_recalc = 4;
  
  // First derive the edges with the appropriate method
  edges = VerifyEdges(parameters);

  // Remove old segment numbers and processed flags
  segment_tag = parameters.SegmentAttribute();
  if (parameters.EraseOldLabels()) {
    SetAttribute(segment_tag, NoSegmentNumber);
    num_old_surfaces = 0;
  }
  else {
  	if (AttributeRange(segment_tag, segment_number, num_old_surfaces)) {
      for (start_point=begin(); start_point!=end(); start_point++)
  	    if (!start_point->HasAttribute(segment_tag))
  	      start_point->Attribute(segment_tag) = NoSegmentNumber;
  	}
  	else num_old_surfaces = 0;
  }
  SetAttribute(IsProcessedTag, 0);

  // Initialise planes for smooth face detection
  if (parameters.SurfaceModel() == 1)
    for (start_point=begin(); start_point!=end(); start_point++)
      planes.push_back(Plane());

  // Initialise counters
  if (verbose) {
    printf("Total number of points %d\n", size());
    printf("  Start  #Surf #Hough   #Fit   Size\n");
  }
  num_surfaces = num_hough = num_fits = surface_size = 0;
  
  // Check if reflectance strength should be used
  if (parameters.MaxReflectanceDifferenceSeed() >= 0 &&
      parameters.MaxReflectanceDifferenceGrowing() >= 0) {
    if (HasAttribute(ReflectanceTag)) {
      reflectance_tag = ReflectanceTag;
      use_reflectance = true;
    }
    else if (HasAttribute(IntensityTag)) {
      reflectance_tag = IntensityTag;
      use_reflectance = true;
    }
  }
         
  // Loop over all points as potential start points for seed surfaces
  for (start_point=begin(), start_index=0; start_point!=end();
       start_point++, start_index++) {
    // Start with a point without a segment number
    if (start_point->Attribute(segment_tag) == NoSegmentNumber &&
        start_point->Attribute(IsProcessedTag) == 0) {
      // Select the points around this point
      seed_nodes = TaggedNeighbourhood(PointNumber(start_index),
                                NoSegmentNumber,
                                parameters.SeedNeighbourhoodRadius(),
                                edges->TINEdgesRef(), segment_tag,
                                parameters.DistanceMetricDimension() == 2,
                                parameters.SeedNeighbourhoodDefinition() == 0);
      if (verbose)
        printf("%7d %6d %6d %6d %6d\r", start_index, num_surfaces,
               num_hough, num_fits, surface_size);
      // Skip this point if there are no sufficient neighbours
      if (seed_nodes.size() < parameters.MinNumberOfPointsSeed()) continue;

      // Put the points in a different laser point set
      // and check the reflectance difference
      seed_points.ErasePoints();
      if (use_reflectance) {
        plane_reflectance = start_point->Attribute(reflectance_tag);
        reflectance_sum = plane_reflectance;
      }
      for (seed_node=seed_nodes.begin(); seed_node!=seed_nodes.end();
           seed_node++) {
        if (use_reflectance) {
          seed_point = begin() + seed_node->Number();
          if (fabs(plane_reflectance - seed_point->Attribute(reflectance_tag)) <=
              parameters.MaxReflectanceDifferenceSeed())
            seed_points.push_back(*seed_point);
        }
        else
          seed_points.push_back((*this)[seed_node->Number()]);
      }

      // Skip this point if there are no sufficient neighbours with a similar
      // reflectance
      if (seed_points.size() < parameters.MinNumberOfPointsSeed()) continue;
      
      // Perform the Hough transformation
      seed_points.InitialiseHoughSpace(hough_space, -1,
                                       parameters.MaxSlopeAngle(),
                                       parameters.BinSizeSlopeAngle(),
                                       parameters.BinSizeDistance(), 1);
      seed_points.IncrementHoughSpace(hough_space);
      num_hough++;
      // Extract a plane from the Hough space
      num_ignored_planes = 0;
      found_good_plane = false;
      seed_points.Label(UNLABELED);
      do {
        // Select the best plane
        plane = hough_space.BestPlane(&num_pts, num_ignored_planes, 3);
        found_plane = (num_pts >= parameters.MinNumberOfPointsSeed());
        if (found_plane) {
          // Label and count the points near the plane
          num_pts = seed_points.Label(plane, parameters.MaxDistanceSeedPlane(),
                                      UNLABELED, UNLABELED, 1, 0);
          found_plane = (num_pts >= parameters.MinNumberOfPointsSeed());
          if (found_plane) {
            // Improve the plane fit
            plane = seed_points.FitPlane(1, 1);
            // Relabel and count the points near the plane
            num_pts = seed_points.Label(plane, parameters.MaxDistanceSeedPlane(),
                                        0, 1, 1, 0);
            // Fit again
            plane.Initialise();
            for (seed_point=seed_points.begin(), num_pts=0;
                 seed_point!=seed_points.end(); seed_point++) {
              if (seed_point->Label() == 1) {
                num_pts++;
                plane.AddPoint(seed_point->Position3DRef(), false);
              }
            }
            plane.Recalculate();
            num_pts = seed_points.Label(plane, parameters.MaxDistanceSeedPlane(),
                                        0, 1, 1, 0);
            found_plane = (num_pts >= parameters.MinNumberOfPointsSeed());
            // A good plane is found if the current point is near the plane
            found_good_plane =
              (fabs(plane.Distance(start_point->Position3DRef())) <
               parameters.MaxDistanceSeedPlane());
               
            // Otherwise try to find a next best plane
            if (!found_good_plane) num_ignored_planes++;
          }
        }
      } while (!found_good_plane && found_plane);
      hough_space.Erase();

      if (found_good_plane) {
        // Store the plane
        num_surfaces++;
        plane.Number() = plane.Label() = num_surfaces+num_old_surfaces-1;
        switch (parameters.SurfaceModel()) {
          default:
          case 0 : // Improve plane estimate with additional points later
            planes.push_back(plane);
            current_plane = planes.end() - 1;
            break;
          case 1 : // Local plane estimates
            current_plane = planes.begin() + start_index;
            *current_plane = plane;
            break;
        }
        // Set the segment numbers for the points of the detected plane and
        // transfer seed nodes to the surface node list
        // also calculate average reflectance, if used
        if (!surface_nodes.empty()) surface_nodes.Erase();
        if (use_reflectance) reflectance_sum = 0.0;
        for (seed_point=seed_points.begin(), seed_node=seed_nodes.begin();
             seed_point!=seed_points.end(); seed_point++, seed_node++) {
          if (seed_point->Label() == 1) {
            (*this)[seed_node->Number()].SetAttribute(segment_tag,
                                               num_surfaces+num_old_surfaces-1);
            surface_nodes.push_back(*seed_node);
            if (parameters.SurfaceModel() == 1)
              planes[seed_node->Number()] = plane;
            if (use_reflectance)
              reflectance_sum += seed_point->Attribute(reflectance_tag);
          };
        }
        if (use_reflectance)
          plane_reflectance = reflectance_sum / surface_nodes.size();
                    
        node_index = 0;
        fit_interval = 10;
        num_fits = 0;
        // Try to expand from each surface node
        for (surface_node=surface_nodes.begin() + node_index;
             surface_node!=surface_nodes.end(); surface_node++, node_index++) {
          // Determine the neighbourhood of the surface node
          nbh_nodes = Neighbourhood(*surface_node,
                                    parameters.GrowingRadius(),
                                    edges->TINEdgesRef(),
                                    parameters.DistanceMetricDimension() == 2,
                              parameters.GrowingNeighbourhoodDefinition() == 0);
          // Check if there are points without segment number or with other
          // segment numbers in the neighbourhood of the point
          for (nbh_node=nbh_nodes.begin(); nbh_node!=nbh_nodes.end();
               nbh_node++) {
            nbh_point = begin() + nbh_node->Number();
            if (parameters.SurfaceModel() == 1)
              current_plane = planes.begin() + surface_node->Number();
            if (nbh_point->Attribute(segment_tag) == NoSegmentNumber ||
                (parameters.SurfacesCompete() &&   // Competing surfaces
                 parameters.SurfaceModel() == 0 && // for planes only
                 nbh_point->Attribute(segment_tag) != num_surfaces+num_old_surfaces-1)) {
              dist = fabs(current_plane->Distance(nbh_point->Position3DRef()));
              if (use_reflectance) {
            ;    reflectance = nbh_point->Attribute(reflectance_tag);
                reflectance_dif = fabs(plane_reflectance - reflectance);
              }               
              if (dist < parameters.MaxDistanceSurface() &&
                  (!use_reflectance ||
                   reflectance_dif <= parameters.MaxReflectanceDifferenceGrowing())) {
                old_segment_number = nbh_point->Attribute(segment_tag);
                add_point = true;
                // Evaluate competing surfaces before assigning segment number
                if (old_segment_number != NoSegmentNumber) {
                  competing_nodes = Neighbourhood(*nbh_node,
                              parameters.GrowingRadius(), edges->TINEdgesRef(),
                              parameters.DistanceMetricDimension() == 2,
                              parameters.GrowingNeighbourhoodDefinition() == 0);
                  old_plane = planes.begin() + old_segment_number;
                  // Check RMS error of the local set of points
                  sq_error_old = sq_error_new = 0.0;
                  for (competing_node=competing_nodes.begin();
                       competing_node!=competing_nodes.end();
                       competing_node++) {
                    competing_point = begin() + competing_node->Number();
                    segment_number = competing_point->Attribute(segment_tag);
                    if (segment_number == old_segment_number ||
                        segment_number == num_surfaces + num_old_surfaces - 1) {
                      dist = old_plane->Distance(competing_point->Position3DRef());
                      sq_error_old += dist * dist;
                      dist = current_plane->Distance(competing_point->Position3DRef());
                      sq_error_new += dist * dist;
                    }
                  }
                  if (sq_error_old < sq_error_new) add_point = false;
                  // Update the old plane if required
                  if (add_point)
                    old_plane->RemovePoint(nbh_point->Position3DRef(), true);

                  /* Alternative method for competing surfaces:
                     Check normal vector direction. This method worked less well.

                  // Estimate local plane with points of both competing planes
                  local_plane.Initialise();
                  for (competing_node=competing_nodes.begin();
                       competing_node!=competing_nodes.end();
                       competing_node++) {
                    competing_point = begin() + competing_node->Number();
                    segment_number = competing_point->Attribute(segment_tag);
                    if (segment_number == old_segment_number ||
                        segment_number == num_surfaces + num_old_surfaces - 1)
                      local_plane.AddPoint(competing_point->Position3DRef(), false);
                  }
                  add_point = (local_plane.NumberOfPoints() > 3);
                  if (add_point) {
                    local_plane.Recalculate();
                    // Check normal vector angles
                    if (Angle(local_plane.Normal(), current_plane->Normal()) <
                        Angle(local_plane.Normal(), old_plane->Normal()))
                      old_plane->RemovePoint(nbh_point->Position3DRef(), true);
                    else
                      add_point = false;
                  }
                  */
                }
                // add_point = false; //test by soe to see what is influence of added to segm results...
               
               
/* TESTING 
                // Additional check if the neighbouring point is in a neighbourhood with
                // most points on the plane. This avoids growing planes in trees

                if (add_point) {
                  nbh_nbh_nodes = Neighbourhood(*nbh_node,
                              parameters.GrowingRadius(), edges->TINEdgesRef(),
                              parameters.DistanceMetricDimension() == 2,
                              parameters.GrowingNeighbourhoodDefinition() == 0);
                  num_near_nbh_points = 0;
                  for (nbh_nbh_node=nbh_nbh_nodes.begin();
                       nbh_nbh_node!=nbh_nbh_nodes.end(); nbh_nbh_node++) {
                    nbh_nbh_point = begin() + nbh_nbh_node->Number();
                    dist = fabs(current_plane->Distance(nbh_nbh_point->Position3DRef()));
                    if (dist < parameters.MaxDistanceSurface())
                      num_near_nbh_points++;
                  }
                  // 80% of neighbourhood points should be near plane
                  if ((float) num_near_nbh_points / (float) nbh_nbh_nodes.size() < 0.8)
//                  if (num_near_nbh_points != (int) nbh_nbh_nodes.size())
				    add_point = false;  
                }

// END TESTING */
                if (add_point) {    
                  nbh_point->SetAttribute(segment_tag, num_surfaces+num_old_surfaces-1);
                  surface_nodes.push_back(*nbh_node);
                  surface_node = surface_nodes.begin() + node_index;
                  surface_size = surface_nodes.size();
                  // Update plane reflectance
                  if (use_reflectance) {
                    reflectance_sum += reflectance;
                    plane_reflectance = reflectance_sum / surface_size;
                  }
                  // Update plane estimation
                  switch (parameters.SurfaceModel()) {
                    default:
                    case 0 : // Improve on plane estimate with additional point 
                      current_plane->AddPoint(nbh_point->Position3DRef(),
                          surface_size == (surface_size/fit_interval)*fit_interval);
                      if (surface_size == (surface_size/fit_interval)*fit_interval)
                        num_fits++;
                      if (surface_size > 10 * fit_interval) fit_interval *= 10;
                      if (verbose)
                        printf("%7d %6d %6d %6d %6d\r", start_index, num_surfaces,
                               num_hough, num_fits, surface_size);
                      break;
                    case 1: // Local estimate
                      if (dist > parameters.MinDistanceRecompute()) {
                        // Collect all nodes of the current surface within the seed radius
                        recalc_nodes = TaggedNeighbourhood(*nbh_node,
                                num_surfaces + num_old_surfaces - 1,
                                parameters.SeedNeighbourhoodRadius(),
                                edges->TINEdgesRef(), segment_tag,
                                parameters.DistanceMetricDimension() == 2,
                                parameters.SeedNeighbourhoodDefinition() == 0);
                        num_nbh_pts = (int) recalc_nodes.size();
                      }
                      else num_nbh_pts = 0;
                      if (num_nbh_pts >= min_num_pts_recalc) {
                        num_fits++;
                        new_plane.Initialise();
                        for (nbh_node2=recalc_nodes.begin();
                             nbh_node2!=recalc_nodes.end(); nbh_node2++) {
                          nbh_point2 = begin() + nbh_node2->Number();
                          new_plane.AddPoint(nbh_point2->Position3DRef(), false);
                        }
                        new_plane.Recalculate();
                        angle_difference = 
                          Angle(current_plane->Normal(), new_plane.Normal());
                        if (angle_difference > 90.0 * degree)
                          angle_difference = 180.0 * degree - angle_difference;
                        // Use local plane estimate if normal difference is small
                        if (angle_difference < max_angle_difference)
                          planes[nbh_node->Number()] = new_plane;
                        else { // Otherwise use plane of neighbouring point
                          current_plane->AddPoint(nbh_point->Position3DRef());
                          planes[nbh_node->Number()] = *current_plane;
                        }
                      }
                      else // Otherwise use plane of neighbouring point
                        planes[nbh_node->Number()] = *current_plane;
                      if (verbose)
                        printf("%7d %6d %6d %6d %6d\r", start_index, num_surfaces,
                               num_hough, num_fits, surface_size);
                      break;
                  }
                }
              }                                
            }
          }
        }
        // Remove the classification if the surface is too small
        if (surface_nodes.size() < parameters.MinNumberOfPointsComponent()) {
          for (surface_node=surface_nodes.begin();
               surface_node!=surface_nodes.end(); surface_node++) {
            surface_point = begin() + surface_node->Number();
            surface_point->SetAttribute(segment_tag, NoSegmentNumber);
          }
          num_surfaces--;
          if (parameters.SurfaceModel() == 0) {
            (planes.end()-1)->Erase();
            planes.erase(planes.end()-1); 
          }
        }
      }
      // If no good seed plane is found, disable all other points as potential
      // start points. This strategy reduces the number of required Hough 
      // transforms, but may cause missing small planes
      else {
        for (seed_node=seed_nodes.begin();
             seed_node!=seed_nodes.end(); seed_node++) {
    //      (*this)[seed_node->Number()].SetAttribute(IsProcessedTag, 1); //test by soe what is influence of this on segmentation
        }
      }
    }
  }
  if (verbose) printf("\n");
  // Copy planes if required
  if (segment_planes != NULL) segment_planes->swap(planes);
  // Remove segment numbers "NoSegmentNumber"
  for (surface_point=begin(); surface_point!=end(); surface_point++)
    if (surface_point->Attribute(segment_tag) == NoSegmentNumber)
      surface_point->RemoveAttribute(segment_tag);
  // Remove IsProcessed tags
  RemoveAttribute(IsProcessedTag);
  // Delete the edges, planes, and point number lists
  seed_points.ErasePoints();
  if (delete_edges) EraseNeighbourhoodEdges();
  planes.Erase();
  seed_nodes.Erase();
  surface_nodes.Erase();
  nbh_nodes.Erase();
  competing_nodes.Erase();
  
  return true;
}



bool LaserPoints::SegmentGrowing(const SegmentationParameters &parameters,
                                 int first_segment_number,
                                 bool delete_edges, bool verbose,
								 int tile_number)
{
  TINEdges                  *edges;
  TINEdges::iterator        set;
  TINEdgeSet                edgeset;
  int                       start_index, num_pts, num_ignored_planes,
                            num_segments, node_index, segment_size, num_nbh_pts,
                            segment_number, old_segment_number, reflectance;
  PointNumberList           seed_nodes, segment_nodes, nbh_nodes;                            
  PointNumberList::iterator seed_node, segment_node, nbh_node, nbh_node2,
                            competing_node;
  LaserPoint                tolerances = parameters.GrowingTolerances();
  LaserPoints               averages, sums;
  LaserPoints::iterator     start_point, seed_point, segment_point, nbh_point,
                            nbh_point2, competing_point;
  bool                      good_point;
  const unsigned char       *tolerance_tags, *tag;
  int                       num_tolerances, itag;
  LaserPoint                average, sum;
  LaserPointTag             segment_tag;
  double                    pi = 4.0 * atan(1.0);
 
  // First derive the edges with the appropriate method
  edges = VerifyEdges(parameters);

  // Remove old segment numbers and processed flags
  segment_tag = parameters.SegmentAttribute();
  segment_number = first_segment_number - 1;
  if (parameters.EraseOldLabels()) {
    SetAttribute(segment_tag, NoSegmentNumber);
  }
  else {
  	if (AttributeRange(segment_tag, itag, segment_number))
  	  if (first_segment_number > segment_number)
  	    segment_number = first_segment_number - 1;
  	for (start_point=begin(); start_point!=end(); start_point++)
  	  if (!start_point->HasAttribute(segment_tag))
  	    start_point->Attribute(segment_tag) = NoSegmentNumber;
  }
  SetAttribute(IsProcessedTag, 0);

  // Initialise counters
  if (verbose) {
    printf("Total number of points %d\n", size());
    printf("  Start  #Segm  Size\n");
  }
  num_segments = 0;
  
  // Tolerance information
  tolerance_tags = tolerances.AttributeTags();
  num_tolerances = tolerances.NumAttributes();
  
  // Loop over all points as potential start points for seed surfaces
  for (start_point=begin(), start_index=0; start_point!=end();
       start_point++, start_index++) {
       	
    // Start with a point without a segment number
    if (start_point->Attribute(segment_tag) == NoSegmentNumber &&
        start_point->Attribute(IsProcessedTag) == 0) {
      // Check if this point has all attributes to be checked
      for (itag=0, tag=tolerance_tags, good_point=true;
	       itag < num_tolerances && good_point; itag++, tag++) {
		if (*tag == (int) DoubleTag) continue;
		if (!start_point->HasAttribute((LaserPointTag) *tag))
		  good_point = false;
	  }
	  if (!good_point) continue;
	  
      // Select the points around this point
      seed_nodes = TaggedNeighbourhood(PointNumber(start_index),
                                NoSegmentNumber,
                                parameters.SeedNeighbourhoodRadius(),
                                edges->TINEdgesRef(), segment_tag,
                                parameters.DistanceMetricDimension() == 2,
                                parameters.SeedNeighbourhoodDefinition() == 0);
      
	  if (verbose)
        printf("%7d %6d %6d\r", start_index, num_segments, segment_size);
      // Skip this point if there are no sufficient neighbours
      if (seed_nodes.size() < parameters.MinNumberOfPointsSeed()) continue;

      // Check if suffient points are within the tolerances
      if (!segment_nodes.empty()) segment_nodes.Erase();
      for (seed_node=seed_nodes.begin();
	       seed_node!=seed_nodes.end(); seed_node++) {
	    seed_point = begin() + seed_node->Number();
        for (itag=0, tag=tolerance_tags, good_point=true;
		     itag < num_tolerances && good_point; itag++, tag++) {
		  if (*tag == (int) DoubleTag) continue;
		  if (!seed_point->HasAttribute((LaserPointTag) *tag)) {
		  	good_point = false;
		  }
		  else {
		  	switch (AttributeType((LaserPointTag) *tag)) {
		  	  default:
		  	  case IntegerAttributeType:
		  	  	good_point =
			      (fabs(start_point->Attribute((LaserPointTag) *tag) -
					    seed_point->Attribute((LaserPointTag) *tag)) <=
				  tolerances.Attribute((LaserPointTag) *tag));
				break;
				
		  	  case FloatAttributeType:
		  	  	// Possibly flip (scaled) normal
		  	  	if (*tag >= NormalXTag && *tag <= NormalZTag) {
		  	  	  if (start_point->Normal().Length() > 0.0 &&
					  seed_point->Normal().Length() > 0.0) {
					if (Angle(start_point->Normal(), seed_point->Normal()) >
					    pi / 2.0) seed_point->FlipNormal();
				  }
		  	  	}
		  	  	else if (*tag >= ScaledNormalXTag && *tag <= ScaledNormalZTag) {
		  	  	  if (start_point->ScaledNormal().Length() > 0.0 &&
					  seed_point->ScaledNormal().Length() > 0.0) {
					if (Angle(start_point->ScaledNormal(), seed_point->ScaledNormal()) >
					    pi / 2.0) seed_point->FlipScaledNormal();
				  }
		  	  	}
		  	  	good_point =
			      (fabs(start_point->FloatAttribute((LaserPointTag) *tag) -
					    seed_point->FloatAttribute((LaserPointTag) *tag)) <=
				  tolerances.FloatAttribute((LaserPointTag) *tag));
				break;
				
		  	  case DoubleAttributeType:
		  	  	good_point =
			      (fabs(start_point->DoubleAttribute((LaserPointTag) *tag) -
					    seed_point->DoubleAttribute((LaserPointTag) *tag)) <=
				  tolerances.DoubleAttribute((LaserPointTag) *tag));
				break;
		  	}
		  }
        }
        if (good_point) segment_nodes.push_back(*seed_node);
      }
      seed_nodes.Erase();

      // Skip this point if there are no sufficient neighbours with a similar
      // attribute values
      if (segment_nodes.size() < parameters.MinNumberOfPointsSeed()) continue;

      // Compute attribute value sums and averages
      segment_size = segment_nodes.size() + 1;
	  sum = *start_point;
      for (segment_node=segment_nodes.begin();
	       segment_node!=segment_nodes.end(); segment_node++) {
	    segment_point = begin() + segment_node->Number();
        for (itag=0, tag=tolerance_tags; itag < num_tolerances; itag++, tag++) {
		  switch (AttributeType((LaserPointTag) *tag)) {
		    default:
		    case IntegerAttributeType:
	          sum.Attribute((LaserPointTag) *tag) +=
	            segment_point->Attribute((LaserPointTag) *tag);
	          break;
		    case FloatAttributeType:
	          sum.FloatAttribute((LaserPointTag) *tag) +=
	            segment_point->FloatAttribute((LaserPointTag) *tag);
	          break;
		    case DoubleAttributeType:
	          sum.DoubleAttribute((LaserPointTag) *tag) +=
	            segment_point->DoubleAttribute((LaserPointTag) *tag);
	          break;
	      }
	    }
	  }
      for (itag=0, tag=tolerance_tags; itag < num_tolerances; itag++, tag++) {
		switch (AttributeType((LaserPointTag) *tag)) {
		  default:
		  case IntegerAttributeType:
	        average.Attribute((LaserPointTag) *tag) =
	          sum.Attribute((LaserPointTag) *tag) / segment_size;
	        break;
		  case FloatAttributeType:
	        average.FloatAttribute((LaserPointTag) *tag) =
	          sum.FloatAttribute((LaserPointTag) *tag) / segment_size;
	        break;
		  case DoubleAttributeType:
	        average.DoubleAttribute((LaserPointTag) *tag) =
	          sum.DoubleAttribute((LaserPointTag) *tag) / segment_size;
	        break;
	    }
	  }

      num_segments++;
      segment_number++;
      average.Attribute(PointNumberTag) = segment_number;
      
      // Set the segment numbers for the points of the detected segment 
      for (segment_node=segment_nodes.begin();
	       segment_node!=segment_nodes.end(); segment_node++) {
	    segment_point = begin() + segment_node->Number();
	    segment_point->Attribute(segment_tag) = segment_number;
      }

      node_index = 0;
      // Try to expand from each segment node
      for (segment_node=segment_nodes.begin() + node_index;
           segment_node!=segment_nodes.end(); segment_node++, node_index++) {
        // Determine the neighbourhood of the segment node
        nbh_nodes = Neighbourhood(*segment_node,
                                  parameters.GrowingRadius(),
                                  edges->TINEdgesRef(),
                                  parameters.DistanceMetricDimension() == 2,
                              parameters.GrowingNeighbourhoodDefinition() == 0);
        
		// Check if there are points without segment number or with other
        // segment numbers in the neighbourhood of the point
        for (nbh_node=nbh_nodes.begin(); nbh_node!=nbh_nodes.end();
             nbh_node++) {
          nbh_point = begin() + nbh_node->Number();
          if (nbh_point->Attribute(segment_tag) == NoSegmentNumber) {
                   // No competing segments yet
//                (parameters.SurfacesCompete() &&   // Competing surfaces
//                 parameters.SurfaceModel() == 0 && // for planes only
//                 nbh_point->Attribute(segment_tag) != num_surfaces-1)) {
	        // Check if this point is close to the current average
	        good_point = true;
            for (itag=0, tag=tolerance_tags, good_point=true;
		         itag < num_tolerances && good_point; itag++, tag++) {
		      if (*tag == (int) DoubleTag) continue;
		      if (!nbh_point->HasAttribute((LaserPointTag) *tag)) {
		  	    good_point = false;
		      }
		      else {
		  	    switch (AttributeType((LaserPointTag) *tag)) {
		  	      default:
		  	      case IntegerAttributeType:
		  	  	    good_point =
			          (fabs(average.Attribute((LaserPointTag) *tag) -
					        nbh_point->Attribute((LaserPointTag) *tag)) <=
				      tolerances.Attribute((LaserPointTag) *tag));
				    break;
				
		  	      case FloatAttributeType:
		  	      	// Possibly flip (scaled) normal
		  	  	    if (*tag >= NormalXTag && *tag <= NormalZTag) {
		  	  	      if (average.Normal().Length() > 0.0 && 
						  nbh_point->Normal().Length()> 0.0) {
					    if (Angle(average.Normal(), nbh_point->Normal()) > pi / 2.0)
						  nbh_point->FlipNormal();
				      }
		  	  	    }
		  	  	    else if (*tag >= ScaledNormalXTag && *tag <= ScaledNormalZTag) {
		  	  	      if (average.ScaledNormal().Length() > 0.0 && 
						  nbh_point->ScaledNormal().Length()> 0.0) {
					    if (Angle(average.ScaledNormal(), nbh_point->ScaledNormal()) > pi / 2.0)
						  nbh_point->FlipScaledNormal();
				      }
		  	  	    }

		  	  	    good_point =
			          (fabs(average.FloatAttribute((LaserPointTag) *tag) -
					        nbh_point->FloatAttribute((LaserPointTag) *tag)) <=
				      tolerances.FloatAttribute((LaserPointTag) *tag));

				    break;
				
		  	      case DoubleAttributeType:
		  	  	    good_point =
			          (fabs(average.DoubleAttribute((LaserPointTag) *tag) -
					        nbh_point->DoubleAttribute((LaserPointTag) *tag)) <=
				      tolerances.DoubleAttribute((LaserPointTag) *tag));
				    break;
		  	    }
		      }
            }

            if (good_point) {    
              nbh_point->SetAttribute(segment_tag, segment_number);
              segment_nodes.push_back(*nbh_node);
              segment_node = segment_nodes.begin() + node_index;
              segment_size = segment_nodes.size();

              // Update segment attributes
              for (itag=0, tag=tolerance_tags; itag < num_tolerances; itag++, tag++) {
		        switch (AttributeType((LaserPointTag) *tag)) {
		          default:
		          case IntegerAttributeType:
	                sum.Attribute((LaserPointTag) *tag) +=
	                  nbh_point->Attribute((LaserPointTag) *tag);
	                average.Attribute((LaserPointTag) *tag) =
	                  sum.Attribute((LaserPointTag) *tag) / segment_size;
	                break;
		          case FloatAttributeType:
	                sum.FloatAttribute((LaserPointTag) *tag) +=
	                  nbh_point->FloatAttribute((LaserPointTag) *tag);
	                average.FloatAttribute((LaserPointTag) *tag) =
	                  sum.FloatAttribute((LaserPointTag) *tag) / segment_size;
	                break;
		          case DoubleAttributeType:
	                sum.DoubleAttribute((LaserPointTag) *tag) +=
	                  nbh_point->DoubleAttribute((LaserPointTag) *tag);
	                average.DoubleAttribute((LaserPointTag) *tag) =
	                  sum.DoubleAttribute((LaserPointTag) *tag) / segment_size;
	                break;
	            }
	          }
              printf("%7d %6d %6d\r", start_index, num_segments, segment_size);
            }
          }
        }
        nbh_nodes.Erase();
      }
        
      // Remove the classification if the surface is too small
      if (segment_nodes.size() < parameters.MinNumberOfPointsComponent()) {
        for (segment_node=segment_nodes.begin();
             segment_node!=segment_nodes.end(); segment_node++) {
          segment_point = begin() + segment_node->Number();
          segment_point->SetAttribute(segment_tag, NoSegmentNumber);
        }
        num_segments--;
        segment_number--;
        // Also disable the other points in the initial seed segment as
        // starting points for a new search. This strategy reduces the number
		// of required seed tests, but may cause missing small segments.
        for (seed_node=seed_nodes.begin();
             seed_node!=seed_nodes.end(); seed_node++) {
          (*this)[seed_node->Number()].SetAttribute(IsProcessedTag, 1);
        }
      }
      
      segment_nodes.Erase();
    }
  }
  if (verbose) printf("\n");
  // Remove segment numbers "NoSegmentNumber" and add tile number as
  // SegmentStartTileNumberTag for new segments if requested
  for (segment_point=begin(); segment_point!=end(); segment_point++) {
    if (segment_point->Attribute(segment_tag) == NoSegmentNumber)
      segment_point->RemoveAttribute(segment_tag);
    else if (tile_number >= 0) {
	  if (!segment_point->HasAttribute(SegmentStartTileNumberTag)) {
	  	segment_point->Attribute(SegmentStartTileNumberTag) = tile_number;
      }
    }
  }
	
  // Remove IsProcessed tags
  RemoveAttribute(IsProcessedTag);
  // Delete the edges and point number lists
  if (delete_edges) EraseNeighbourhoodEdges();
  seed_nodes.Erase();
  segment_nodes.Erase();
  nbh_nodes.Erase();
  
  return true;
}

void LaserPoints::MajorityFilter(const SegmentationParameters &parameters,
                                 const TINEdges &edges)
{
  LaserPoints::iterator point;
  PointNumberList nbh_nodes;
  int             point_number, majority_value, count, minimum_value;
  LaserPointTag   tag = parameters.MajorityAttribute();
  bool            unlabeled_only = parameters.MajorityNoAttributeOnly(),
                  long_attribute, debug=false;
  long long int   long_majority_value;
  
  // Check attribute type
  switch (AttributeType(tag)) {
    case IntegerAttributeType: long_attribute = false; break;
    default:
    case FloatAttributeType:
    case DoubleAttributeType:
      printf("Error: Majority filtering is only available for integer attributes\n");
  	  return;
  	case LongAttributeType: long_attribute = true; break;
  }
  
  if (parameters.MajorityNoSurfaces()) {
  	if (tag == LongSegmentNumberTag) {
  	  printf("Error: This version of the majority filter has no knowledge\n");
  	  printf("       of the number of surfaces in various tiles\n");
  	  printf("       Please use the programme majorityfiltering instead.\n");
  	  return;
  	}
  	minimum_value = HighestSurfaceNumber() + 1;
  }
  else
    minimum_value = 0;
  
  if (debug && tag == LongSegmentNumberTag)
    printf("Long segment number tag\n");
  else if (debug)
    printf("Tag: %d\n", tag);
    
  for (point=begin(), point_number=0; point!=end(); point++, point_number++) {
  	// Skip points that already have this attribute
  	if (unlabeled_only && point->HasAttribute(tag)) continue;
  	
  	if (debug) printf("Point %.2f %.2f %.2f\n", point->X(),
	                  point->Y(), point->Z());
  	// Get neighbourhood
  	nbh_nodes = Neighbourhood(PointNumber(point_number),
                              parameters.MajorityNeighbourhoodRadius(), edges,
                              parameters.DistanceMetricDimension() == 2,
                              parameters.MajorityNeighbourhoodDefinition() == 0);

    if (!long_attribute) {
      majority_value = MostFrequentAttributeValue(tag, nbh_nodes, count,
	                                              minimum_value);
      if (count > 0) {
        point->Attribute(tag) = majority_value;
      }
    }
    else {
      long_majority_value = MostFrequentLongAttributeValue(tag, nbh_nodes,
	                                                       count, 0);
      if (count > 0) point->SetLongAttribute(tag, long_majority_value);
      if (debug) printf("nbh size %d, maj val %lld, count %d\n",
	                    nbh_nodes.size(), long_majority_value, count);
    }
  }
}

void LaserPoints::MergeSurfaces(const SegmentationParameters &parameters)
{
  std::vector<long long int>           segment_numbers, new_segment_numbers,
                                       tested_segments;
  std::vector<long long int>::iterator segment_number;
  std::vector<bool>                    validities;
  TINEdges                             *edges;
  LaserPoints::iterator                point1, point2, point3;
  long long int                        segment_number1, segment_number2,
                                       segment_number3,
                                       new_segment_number1, new_segment_number2;
  int 					               index, point_index, num_merged=0,
                                       num_pts1, num_pts2;
  Planes                               planes;
  Planes::iterator                     plane1, plane2;
  bool                                 merge_segments, valid, has_tile_numbers,
                                       debug=false;
  PointNumberList                      nbh;
  PointNumberList::iterator            nb, nb3;
  double                               pi = 4.0 * atan(1.0), angle;
  LaserPointTag                        segment_tag;
  
  segment_tag = SegmentNumberTag; // No option for component merging because of long segment numbers
  has_tile_numbers = HasAttribute(SegmentStartTileNumberTag);

  // Create a sorted vector of all different segment numbers
  LongSegmentNumbers(segment_numbers);
  std::sort(segment_numbers.begin(), segment_numbers.end());
  printf("Started merging %d segments with %d points\n",
         segment_numbers.size(), size());
         
  // Copy the vector
  new_segment_numbers.insert(new_segment_numbers.begin(),
                             segment_numbers.begin(), segment_numbers.end());

  // Fit all planes
  for (segment_number=segment_numbers.begin();
       segment_number!=segment_numbers.end(); segment_number++) {
    planes.push_back(FitPlane(*segment_number));
	validities.push_back((planes.end()-1)->NumberOfPoints() > 2);
  }

  // Verify edges
  edges = VerifyEdges(parameters.SegmentationParametersRef());

  // Process all point neighbourhoods
  printf("Maximum angle between planes %.2f\n",
         parameters.MaxAnglePlanes() * 45.0 / atan(1.0));
  printf("Maximum distance %.2f\n",
         parameters.MaxDistPointToOtherSegment());
  printf("Point    # Merged\n");
  for (point1=begin(), point_index=0; point1!=end(); point1++, point_index++) {
    printf("%7d %5d\r", point_index, num_merged);
    if (point1->HasAttribute(segment_tag)) {
    	
      // Get segment number
      segment_number1 = point1->LongSegmentNumber();
      
      // Get the corresponding new segment number (which initially is the
      // original number
      segment_number = std::find(segment_numbers.begin(), segment_numbers.end(),
	                             segment_number1);
	  index = std::distance(segment_numbers.begin(), segment_number);
	  new_segment_number1 = *(new_segment_numbers.begin() + index);
	  
      // Get plane parameters through corresponding location in planes vector
      plane1 = planes.begin() + index;
      
      // Check if the plane is valid
      valid = *(validities.begin() + index);
	  if (!valid) continue;

      // Test if the slope angle is acceptable
      angle = Angle(plane1->Normal(), Vector3D(0.0, 0.0, 1.0));
      if (angle > pi / 2.0) angle = pi - angle;
      if (angle > parameters.MergingMaxSlopeAngle()) {
        if (debug)
          printf("Segment %lld has a large slope %.2f\n",
		         new_segment_number1,
			     Angle(plane1->Normal(), Vector3D(0.0, 0.0, 1.0)) * 180 / pi);
        continue;
      }
		   
      // Get neighbouring nodes
      nbh = Neighbourhood(PointNumber(point_index),
                          parameters.MergingNeighbourhoodRadius(),
						  edges->TINEdgesRef(),
                          parameters.DistanceMetricDimension() == 2,
                          parameters.MergingNeighbourhoodDefinition() == 0);

	  // Loop over all neighbouring points
      for (nb=nbh.begin(); nb!=nbh.end(); nb++) {
      	point2 = begin() + nb->Number();
      	if (point2->HasAttribute(segment_tag)) {
      	  // Get the new segment number of this point
      	  segment_number2= point2->LongSegmentNumber();
          segment_number = std::find(segment_numbers.begin(), segment_numbers.end(),
	                                 segment_number2);
	      index = std::distance(segment_numbers.begin(), segment_number);
	      new_segment_number2 = *(new_segment_numbers.begin() + index);
      	  if (new_segment_number1 != new_segment_number2) {

      	  	// Only test adjacent segment once
            if(std::find(tested_segments.begin(), tested_segments.end(),
			             segment_number2) == tested_segments.end()) {
			  merge_segments = true;

			  // Get the plane parameters
              plane2 = planes.begin() + index;

              // Check if the plane has sufficient points
              merge_segments = *(validities.begin() + index);

              // Test if the slope angle is acceptable
              if (merge_segments) {
                angle = Angle(plane2->Normal(), Vector3D(0.0, 0.0, 1.0));
                if (angle > pi / 2.0) angle = pi - angle;
                if (angle > parameters.MergingMaxSlopeAngle())
                  merge_segments = false;
                if (debug && !merge_segments)
                  printf("Segment %lld has a large slope %.2f\n",
			             new_segment_number2,
						 Angle(plane2->Normal(), Vector3D(0.0, 0.0, 1.0)) * 180 / pi);
              }
                
              // Test angle between normal vectors
              if (merge_segments) {
                angle = Angle(plane1->Normal(), plane2->Normal());
                if (angle > pi/2.0) angle -= pi;
                if (fabs(angle) > parameters.MaxAnglePlanes()) merge_segments = false;
                if (debug && !merge_segments)
                  printf("Segments %lld and %lld make large angle %.2f\n",
			             new_segment_number1, new_segment_number2,
						 Angle(plane1->Normal(), plane2->Normal()) * 180 / pi);
              }

              // Test point to plane distances in neighbourhood
              if (merge_segments) {
              	num_pts1 = num_pts2 = 0;
                for (nb3=nbh.begin(); nb3!=nbh.end() && merge_segments; nb3++) {
      	          point3 = begin() + nb3->Number();
      	          if (point3->HasAttribute(segment_tag)) {
      	          	segment_number3 = point3->LongSegmentNumber();
      	          	if (segment_number3 == segment_number1) {
      	          	  if (fabs(plane2->Distance(point3->Position3DRef())) >
						  parameters.MaxDistPointToOtherSegment()) {
						merge_segments = false;
					  }
					  else num_pts2++;
      	          	}
      	          	else if (segment_number3 == segment_number2) {
      	          	  if (fabs(plane1->Distance(point3->Position3DRef())) >
						  parameters.MaxDistPointToOtherSegment()) {
						merge_segments = false;
					  }
					  else num_pts1++;
      	          	}
      	          }
      	        }
                if (debug && !merge_segments)
                  printf("Segments %lld or %lld are not close to plane\n",
			             new_segment_number1, new_segment_number2);
      	      }
      	      
      	      // Check if we've tested sufficient points for both planes
      	      if (merge_segments) {
      	      	if (num_pts1 < parameters.MinNumberPointsBothPlanes() ||
					num_pts2 < parameters.MinNumberPointsBothPlanes())
				  merge_segments = false;
                if (debug && !merge_segments)
                  printf("Segments %lld or %lld don't have enough points in neighbourhood (%d, %d)\n",
			             new_segment_number1, new_segment_number2,
						 num_pts1, num_pts2);
      	      }
      	      
      	      // Merge the segments
      	      if (merge_segments) {
      	      	num_merged++;
			    // Replace all occurrences of the new segment number 2 by
			    // the new segment number 1
			    for (segment_number=new_segment_numbers.begin();
				     segment_number!=new_segment_numbers.end(); segment_number++)
				  if (*segment_number == new_segment_number2)
				    *segment_number = new_segment_number1;
      	      }
      	      // Avoid testing this segment again for this point
      	      tested_segments.push_back(segment_number2);
            }
      	  }
      	}
      }
      // Erase list of compared neighbouring segments
      tested_segments.erase(tested_segments.begin(), tested_segments.end());
    }
  }
  
  // Set the new segment numbers  
  for (point1=begin(); point1!=end(); point1++) {
  	if (point1->HasAttribute(segment_tag)) {
  	  segment_number1 = point1->LongSegmentNumber();
      segment_number = std::find(segment_numbers.begin(), segment_numbers.end(),
	                             segment_number1);
	  index = std::distance(segment_numbers.begin(), segment_number);
	  new_segment_number1 = *(new_segment_numbers.begin() + index);
  	  if (new_segment_number1 != segment_number1) {
  	  	if (has_tile_numbers)
  	  	  point1->SetLongSegmentNumber(new_segment_number1);
  	  	else
  	      point1->Attribute(segment_tag) = (int) new_segment_number1;
  	  }
  	}
  }
  
  // Clean up
  segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
  new_segment_numbers.erase(new_segment_numbers.begin(), new_segment_numbers.end());
  tested_segments.erase(tested_segments.begin(), tested_segments.end());
  validities.erase(validities.begin(), validities.end());

  // Note: Edges are not deleted! They may be re-used in other functions.
}



void LaserPoints::MeanShift(const SegmentationParameters &parameters,
			                int next_segment_number, bool verbose)
{
  LaserPoints::iterator           lpt1, lpt2, corresponding_centre1,
                                  corresponding_centre2;
  LaserPoints                     corresponding_centres;
  LaserPoint                      band_widths;
  bool                            done, complete, within_window, converged;
  int                             iteration, itag, num_tags, i, nump, numiter,
                                  max_num_iterations;
  vector<LaserPointTag>           all_tags;
  vector<LaserPointTag>::iterator tag;
  double                          *value1, *value2, *bandwidths, *bandwidth,
                                  x, weight, weight_sum,
								  *values1, *values2, 
								  *weighted_shift_sums, *weighted_shift_sum,
								  shift;
  const unsigned char             *attribute_tags; 
  LaserPointTag                   segment_tag;
  
  // Transfer parameters to local variables
  band_widths        = parameters.AttributeBandWidths();
  max_num_iterations = parameters.MaxNumIterationsModeSeeking();
  segment_tag        = parameters.SegmentAttribute();
  
  // Put all attribute tags and coordinates in one vector for more compact code later
  num_tags = band_widths.NumAttributes();
  attribute_tags = band_widths.AttributeTags();
  for (i=0; i<num_tags; i++, attribute_tags++) {
  	if (*attribute_tags < NoTag) all_tags.push_back((LaserPointTag) *attribute_tags);
  }
  all_tags.push_back(XCoordinateTag);
  all_tags.push_back(YCoordinateTag);
  all_tags.push_back(ZCoordinateTag);
  
  // Store all band widths in a local vector
  num_tags = all_tags.size();
  bandwidths = (double *) malloc(num_tags * sizeof(double));
  for (tag=all_tags.begin(), bandwidth=bandwidths;
       tag!=all_tags.end(); tag++, bandwidth++) {
 	if (!band_widths.HasAttribute(*tag)) {
  	  printf("Error: No band width specified for attribute %s\n",
		     AttributeName(*tag, false));
	  return;
  	}
    switch (AttributeType(*tag)) {
      default:
      case IntegerAttributeType:
  	    printf("Error: Integer tags cannot be used in MeanShift\n");
  	    return;
	  
	  case FloatAttributeType:
	    *bandwidth = (double) band_widths.FloatAttribute(*tag); break;
	  
	  case DoubleAttributeType:
	    *bandwidth = band_widths.DoubleAttribute(*tag); break;
	}
	if (*bandwidth <= 0.0) {
	  printf("Error: Non-positive bandwidth (%.2f) for attribute %s\n",
	         *bandwidth, AttributeName(*tag, false));
	  return;
	}
  }

  // Create a vector to store all corresponding centres
  for (lpt1=begin(); lpt1!=end(); lpt1++)
    corresponding_centres.push_back(LaserPoint());
    
  // Erase old segment numbers if required
  if (parameters.EraseOldLabels()) RemoveAttribute(segment_tag);
  
  // Start with the mode search for every point
  if (verbose)
    printf("Mode search with %d attributes and 3 coordinates\n", num_tags-3);
  values1             = (double *) malloc(num_tags * sizeof(double));
  values2             = (double *) malloc(num_tags * sizeof(double));
  weighted_shift_sums = (double *) malloc(num_tags * sizeof(double));
  nump = numiter = 0;
  for (lpt1=begin(), i=0, corresponding_centre1=corresponding_centres.begin();
       lpt1!=end(); lpt1++, corresponding_centre1++, i++) {
    if (verbose) printf("%5d\r", i);
    
    // Skip this point if it already has a segment number
    if (lpt1->HasAttribute(segment_tag)) continue;
    
  	// Check if all attributes are there and put them in a local vector
  	// This is the starting position for the mode search
  	complete = true;
  	for (tag=all_tags.begin(), value1=values1; 
	     tag!=all_tags.end() && complete; tag++, value1++) {
  	  if (!lpt1->HasAttribute(*tag)) complete = false;
  	  else {
  	  	switch (AttributeType(*tag)) {
  	  	  default:
  	  	  case FloatAttributeType:
  	  	  	*value1 = (double) lpt1->FloatAttribute(*tag); break;
  	  	  case DoubleAttributeType:
  	  	  	*value1 = lpt1->DoubleAttribute(*tag); break;
  	  	}
  	  }
    }
  	
  	// Start path search
    if (complete) {
      nump++;
      iteration = 0;
      done = false;
      do { 
        // Initialise weighted shift sum and weight sum
        memset(weighted_shift_sums, 0, num_tags * sizeof(double));
        weight_sum = 0.0;
        // Collect weighted shifts and weights from all points
        for (lpt2=begin(); lpt2!=end(); lpt2++) { 
          // Put all attributes in a local vector
		  for (tag=all_tags.begin(), value2=values2, complete=true; 
		       tag!=all_tags.end() && complete; tag++, value2++) {
  	        if (!lpt2->HasAttribute(*tag)) complete = false;
  	        else {
  	  	      switch (AttributeType(*tag)) {
  	  	        default:
  	  	        case FloatAttributeType:
  	  	  	      *value2 = (double) lpt2->FloatAttribute(*tag); break;
  	  	        case DoubleAttributeType:
  	  	  	      *value2 = lpt2->DoubleAttribute(*tag); break;
  	  	      }
  	        }
  	      }
          if (complete) {
          	// Compute weight of this point
          	weight = 0.0;
          	for (itag=0, value1=values1, value2=values2, bandwidth=bandwidths;
			     itag<num_tags; itag++, value1++, value2++, bandwidth++) {
			  x = ((*value2) - (*value1)) / (*bandwidth);
			  weight += x * x;
		    }
		    weight = exp(-weight / 2.0);
		    // Update sums
          	for (itag=0, value1=values1, value2=values2,
			     weighted_shift_sum=weighted_shift_sums;
			     itag<num_tags;
				 itag++, value1++, value2++, weighted_shift_sum++) {
		      *weighted_shift_sum += ((*value2) - (*value1)) * weight;
		    }
		    weight_sum += weight;
          } 
        }
   
        // Apply Mean shift vector and check for convergence
        converged = true;
        for (itag=0, value1=values1, weighted_shift_sum=weighted_shift_sums,
             bandwidth=bandwidths; itag!=num_tags;
			 itag++, value1++, weighted_shift_sum++, bandwidth++) {
	      // Estimate shift
          shift = (*weighted_shift_sum) / weight_sum;

          // Check for convergence
          if (shift > (*bandwidth) / 100.0) converged = false;

          // Update mode
          *value1 += shift;
	    }
        iteration++;
       
        // Record all the converged values
        if (converged || iteration == max_num_iterations) {
		  for (tag=all_tags.begin(), value1=values1; 
		       tag!=all_tags.end(); tag++, value1++) {
        	switch (AttributeType(*tag)) {
        	  default:
        	  case FloatAttributeType:
        	  	corresponding_centre1->FloatAttribute(*tag) = (float) (*value1);
        	  	break;
        	  case DoubleAttributeType:
        	  	corresponding_centre1->DoubleAttribute(*tag) = *value1;
        	  	break;
        	}
          }
          done = true;
        }
      } while( done == false );
      numiter += iteration;
    }     
  }
  if (verbose) {
    if (nump) {
      printf("Average number of iterations per point: %.1f\n",
             (float) numiter / (float) nump);
    }
    else {
      printf("All points already have a segment number!\n");
    }
  }

  // Grouping points that are within the window size
  int seg_size;
  for (lpt1=begin(), corresponding_centre1=corresponding_centres.begin();
       lpt1!=end(); lpt1++, corresponding_centre1++) {
    if(!lpt1->HasAttribute(segment_tag) &&
	   (corresponding_centre1->NumAttributes() > 0 ||
	    band_widths.NumAttributes() == 0)) {  
	  seg_size = 1;
      lpt1->SetAttribute(segment_tag, next_segment_number);
      for(lpt2=begin(), corresponding_centre2=corresponding_centres.begin();
	      lpt2!=end(); lpt2++, corresponding_centre2++) {
        if(!lpt2->HasAttribute(segment_tag) &&
	       (corresponding_centre2->NumAttributes() > 0 ||
	        band_widths.NumAttributes() == 0) &&
		   lpt1 != lpt2) {  
          within_window = true;
          for (tag=all_tags.begin(), bandwidth=bandwidths;
		       tag!=all_tags.end() && within_window; tag++, bandwidth++) {
            switch (AttributeType(*tag)) {
	  	      default:
		      case FloatAttributeType:
		        if (fabs(corresponding_centre1->FloatAttribute(*tag) -
		                 corresponding_centre2->FloatAttribute(*tag)) >
		            (*bandwidth) / 4.0)
		          within_window = false;
		          break;
		      
		      case DoubleAttributeType:
		        if (fabs(corresponding_centre1->DoubleAttribute(*tag) -
		                 corresponding_centre2->DoubleAttribute(*tag)) >
		            (*bandwidth) / 4.0)
		          within_window = false;
		          break;
		    }
          }
          if(within_window)
            lpt2->SetAttribute(segment_tag, next_segment_number);
          if (within_window) seg_size++;
	    }
      }
      next_segment_number++;
    }     
  }

  corresponding_centres.ErasePoints();
  free(values1);
  free(values2);
  free(weighted_shift_sums);
  free(bandwidths);
  all_tags.erase(all_tags.begin(), all_tags.end());
}
