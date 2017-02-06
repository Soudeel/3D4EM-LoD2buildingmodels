
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
 Reconstruction of a building model from laser data and a ground plan

 Initial creation:
 Author : George Vosselman
 Date   : 05-07-2000

 Update #1
 Author : George Vosselman
 Date   : 10-11-2002
 Purpose: Split rooffaces_cpp into two parts: an interface and a
          function inside class LaserPoints
*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "Positions2D.h"
#include "ObjectPoints2D.h"
#include "ObjectPoints.h"
#include "LaserPoint.h"
#include "LaserPoints.h"
#include "BNF_io.h"
#include "VRML_io.h"
#include "Planes.h"
#include "HoughSpace.h"
#include "PointNumberLists.h"
#include "LineTopologies.h"

int LaserPoints::ReconstructBuilding(

  // Intermediate and final results
  TINEdges         &laser_edges,
  Planes           &initial_planes,
  PointNumberLists &initial_face_points,
  LineTopologies   &initial_face_contours,
  Planes           &planes,
  PointNumberLists &face_points,
  LineTopologies   &face_contours,
  ObjectPoints2D   &segm_map_points,
  LineTopologies   &segm_map_lines,
  ObjectPoints2D   &refined_map_points,
  LineTopologies   &refined_map_lines,
  ObjectPoints2D   &intersection_points,
  LineTopologies   &intersection_lines,
  ObjectPoints2D   &jump_edge_points,
  LineTopologies   &jump_edges,
  ObjectPoints     &building_corners,
  LineTopologies   &building_faces,
  
  // Control parameters
  bool   use_current_segmentation,
  int    equation_type,
  double slope_max,
  double slope_bin,
  double dist_bin,
  double min_height,
  int min_numpts,
  int local_max_window,
  double max_dist_point_to_plane, 
  double min_dist_planes,
  double min_angle_planes,
  double max_angle_horizontal,
  double min_dist_intersection_partition,
  double min_angle_intersection_partition,
  double max_dist_point_to_intersection,
  double min_dist_height_jump_partition,
  double min_angle_height_jump_partition,
  double max_dist_point_to_height_jump,
  double min_jump_height,
  double min_partition_size,
  double min_partition_percentage,

  // Ground height
  double ground_height,
  
  // Output files
  char *model_pts_file,
  char *model_top_file,
  char *model_vrml_file,
  char *init_top_file,
  char *init_vrml_file,
  char *grown_top_file,
  char *grown_vrml_file
)
{
  int                        face_number, success, min_size, debug=0,
                             max_size, wall_label, wall_colour_set;
  double                     av_size;
  Plane                      new_plane;
  Planes::iterator           plane;
  HoughSpace                 houghspace;
  PointNumberLists::iterator face;
  PointNumberList            new_face;
  LineTopologies::iterator   partition, face_bound;
  LaserPoints::iterator      point;
  FILE                       *vrml;
  vector <int>               segment_numbers;
  vector <int>::iterator     segment_number;

// Output of parameters

  if (debug) {
    printf("use_current_segmentation %d\n", use_current_segmentation);
    printf("equation_type %d\n", equation_type);
    printf("slope_max %6.3f\n", slope_max);
    printf("slope_bin %6.3f\n", slope_bin);
    printf("dist_bin %6.3f\n", dist_bin);
    printf("min_height %6.3f\n", min_height);
    printf("min_numpts %6d\n", min_numpts);
    printf("local_max_window %6d\n", local_max_window);
    printf("max_dist_point_to_plane %6.3f\n", max_dist_point_to_plane); 
    printf("min_dist_planes %6.3f\n", min_dist_planes);
    printf("min_angle_planes %6.3f\n", min_angle_planes);
    printf("max_angle_horizontal %6.3f\n", max_angle_horizontal);
    printf("min_dist_intersection_partition %6.3f\n", min_dist_intersection_partition);
    printf("min_angle_intersection_partition %6.3f\n", min_angle_intersection_partition);
    printf("max_dist_point_to_intersection %6.3f\n", max_dist_point_to_intersection);
    printf("min_dist_height_jump_partition %6.3f\n", min_dist_height_jump_partition);
    printf("min_angle_height_jump_partition %6.3f\n", min_angle_height_jump_partition);
    printf("max_dist_point_to_height_jump %6.3f\n", max_dist_point_to_height_jump);
    printf("min_jump_height %6.3f\n", min_jump_height);
    printf("min_partition_size %6.3f\n", min_partition_size);
    printf("min_partition_percentage %6.3f\n", min_partition_percentage);
    printf("min_partition_size %6.3f\n", min_partition_size);
  }

  
  // Use current segmentation if available
  if (HasAttribute(SegmentNumberTag) && use_current_segmentation) {
    // Derive planes from segments and define labels to be used later
    segment_numbers = AttributeValues(SegmentNumberTag);
    Label(65535);
    for (segment_number=segment_numbers.begin(), face_number=0; 
         segment_number!=segment_numbers.end();
         segment_number++, face_number++) {
      // Get all points of the segment
      new_face = TaggedPointNumberList(SegmentNumberTag, *segment_number);
      face_points.push_back(new_face);
      // Relabel the segment points
      Label(new_face, face_number);
      // Fit a plane
      new_plane = FitPlane(new_face, face_number);
      planes.push_back(new_plane);
    }
    refined_map_points = segm_map_points;
    refined_map_lines  = segm_map_lines;
  }

  // Old segmentation with global Hough transform per partition
  else {  
    // Loop over all partitions of the ground plan
    face_number = 0;
    printf("Extracted planes: number, size, (av., min. and max. size)\n");
    printf("%3d: %5d (%5d %5d %5d)\r", -1, 0, 0, 0, 0);  fflush(stdout);
    min_size = 1000000;
    max_size = 0;
    av_size = 0.0;
    for (partition=segm_map_lines.begin(); partition!=segm_map_lines.end();
         partition++) {

      // Label all points inside the partition
      Label(0);
      LabelInside(segm_map_points, partition->PointNumberListReference(), 1);

      // Initialise the Hough space for these points
      InitialiseHoughSpace(houghspace, 1, slope_max, slope_bin, dist_bin,
                           equation_type);

      // Add the points with label 1 to the Hough space
      IncrementHoughSpace(houghspace, 1);
  
      // Determine the planes in this partition
      do {
        // Extract the next plane from the Hough space
        success = ExtractFace(houghspace, local_max_window,
                              max_dist_point_to_plane, min_numpts,
                              laser_edges, face_number, new_plane, new_face);

        // Store all face information
        if (success) {
          face_number++;
          initial_face_points.push_back(new_face);
          initial_planes.push_back(new_plane);
          if (new_face.size() < min_size) min_size = new_face.size();
          if (new_face.size() > max_size) max_size = new_face.size();
          av_size = (av_size * (initial_face_points.size() - 1) + new_face.size())
                    / initial_face_points.size();
          printf("%3d: %5d (%5d %5d %5d)\r", new_plane.Number(), new_face.size(),
                 (int) av_size, min_size, max_size);  fflush(stdout);
        }
      } while (success);
    }
    printf("\n");
    if (initial_planes.empty()) return(0);

    // Label all points in the faces
    Label(65535); // Used to indicate point have no label
    for (face=initial_face_points.begin(), plane=initial_planes.begin();
         face!=initial_face_points.end(); face++, plane++)
      Label(face->PointNumberListReference(), plane->Number());

    // Determine initial face contours, optional output of topology and VRML
    Rough_Contours(initial_face_points, initial_planes,
                   laser_edges, init_top_file, init_vrml_file,
                   initial_face_contours);

    // Copy some data in order to preserve initial results for analysis
    planes             = initial_planes;
    face_points        = initial_face_points;
    refined_map_points = segm_map_points;
    refined_map_lines  = segm_map_lines;

    // Use the planes found in the difference partitions as seed regions for
    // face growing and merging
    FaceGrowing(face_points, planes, max_dist_point_to_plane,
                min_numpts, laser_edges);
    if (planes.empty()) return(0);
  }

/* Adjust parallel and horizontal planes */

  ParallelFaces(face_points, planes, min_dist_planes,
                min_angle_planes, max_angle_horizontal);

// Determine grown face contours, optional output of topology and VRML

  Rough_Contours(face_points, planes, laser_edges,
                 grown_top_file, grown_vrml_file, face_contours);

/* Merge the partitioning information with the detected faces and
 * construct the roof.
 */

  AlignFacesToPartitions(face_points, planes,
                         refined_map_points, refined_map_lines,
                         building_corners, building_faces, laser_edges,
                         min_dist_intersection_partition,
                         min_angle_intersection_partition,
                         max_dist_point_to_intersection,
                         min_dist_height_jump_partition,
                         min_angle_height_jump_partition,
                         max_dist_point_to_height_jump,
                         min_jump_height,
                         min_partition_size, min_partition_percentage,
                         max_angle_horizontal,
                         intersection_points, intersection_lines,
                         jump_edge_points, jump_edges);

/* Construct the walls */

  plane = planes.end() - 1;
  wall_label = plane->Number() + 1;
  ConstructWalls(building_corners, building_faces, wall_label, ground_height);

/* Output of the building model */

  if (model_pts_file) building_corners.Write(model_pts_file);
  if (model_top_file) building_faces.Write(model_top_file);
  if (model_vrml_file) {
    vrml = VRML_Open(model_vrml_file);
    building_corners.VRML_Write(vrml);
    VRML_Set_Colour(vrml, 1.0, 0.2, 0.2);    // Roof colour
    wall_colour_set = 0;
    for (face_bound=building_faces.begin();
         face_bound!=building_faces.end();
         face_bound++) {
      if (face_bound->Label() == wall_label && !wall_colour_set) {
        VRML_Set_Colour(vrml, 1.0, 1.0, 0.6);
        wall_colour_set = 1;
      }
      face_bound->VRML_Write(vrml, 1);
    }
    VRML_Close(vrml);
  }

// Remove labels of points that do not belong to a plane

  for (point=begin(); point!=end(); point++)
    if (point->Label() == 65535) point->RemoveAttribute(LabelTag);

  return(1);
}
