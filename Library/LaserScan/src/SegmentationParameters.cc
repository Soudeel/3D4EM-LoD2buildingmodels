
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
#include "SegmentationParameters.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "Database4Cpp.h"

SegmentationParameters::SegmentationParameters()
{
  double degree = atan(1.0) / 45.0; 

// Parameters controlling the data organisation

  // Neighbourhood storage model (0 = TIN, 1 = Octree, 2 = kd-tree)
  nbh_model = 2;

  // Octree bin size
  octree_bin_max_numpts = 100;

  // Octree bin overlap
  octree_bin_overlap = 1.0;

  // Distance metric dimension (2 = 2D, 3 = 3D)
  dist_dimension = 3;
  
  // Number of neighbours for kd-tree
  num_nbs = 20;

// Parameters controlling definition of connected components

  // Maximum distance between points in a component
  max_dist_connected = 1.0;
    
  // Minimum number of points in a component
  min_numpts_component = 10;
  
  // Attribute used for storing connected component labelling results
  component_attribute = SegmentNumberTag;
  
  // rase all old component labels before relabelling
  erase_old_labels = true;

// Parameters controlling the seed selection

  // Seed point neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
  seed_neighbourhood = 0;
 
  // Seed neighbourhood radius
  seed_radius = 1.0;

  // Maximum slope angle of Hough space
  max_slope_angle = 90.0 * degree;

  // Bin size of slope angle in Hough space
  bin_size_slope_angle = 3.0 * degree;

  // Bin size of distance in Hough space
  bin_size_distance = 0.2;

  // Minimum number of points in the seed plane
  min_numpts_seed = 10;

  // Maximum distance of points to seed plane
  max_dist_seed_plane = 0.2;
  
  // Maximum reflectance difference in seed plane
  max_refl_dif_seed = -1;

// Parameters controlling the surface growing

  /// Surface model (0=plane, 1=smooth surface)
  surface_model = 0;

  // Growing neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
  growing_neighbourhood = 0;
 
  // Neighbourhood radius for surface growing
  growing_radius = 1.0;

  // Maximum distance of points to surface
  max_dist_surface = 0.3;

  /// Minimum distance for recomputing local plane
  min_dist_recompute = 0.15;
    
  // Maximum reflectance difference in growing phase
  max_refl_dif_growing = -1;

  // Switch for letting surfaces compete for points
  competing_surfaces = true;
  
// Parameters controlling segment growing based on attributes
// Neighbourhoods, distances and minimum seed size are as for surface growing

  // Tolerances for attributes 
  growing_tolerances = LaserPoint();

// Parameters controlling mean shift segmentation

  // Attributes band widths for mean shift
  attribute_band_widths = LaserPoint(10.0, 10.0, 10.0);
    
  // Maximum number of iterations for mean shift mode seeking
  max_num_iter_mode_seeking = 50;

// Parameters controlling merging of segments of a completed segmentation

  /// Merging neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
  merging_neighbourhood = 1;
    
  /// Neighbourhood radius for merging
  merging_radius = 1.0;

  // Maximum angle between planes
  max_angle_between_planes = 5.0 * degree;
    
  // Maximum slope angle of planes to be merged
  merging_max_slope_angle = 2.0 * degree;
  
  // Maximum distance between point and plane of other segment
  max_dist_point_to_other_segment = 0.2;
  
  // Minimum number of points in both planes in neighbourhood (readable)
  min_num_pts_both_planes = 5;
    
// Parameters controlling majority filtering

  /// Majority filtering neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
  majority_neighbourhood = 0;
    
  /// Neighbourhood radius for majority filtering
  majority_radius = 1.0;

  /// Attribute to be filtered
  majority_attribute = SegmentNumberTag;
    
  /// Only apply majority filtering to points without the attribute
  majority_no_attribute_only = false;
  
  /// Do not apply majority filtering to surfaces (readable)
  majority_no_surfaces = true;
}

bool SegmentationParameters::Write(const char *filename) const
{
  FILE *fd;
  
  fd = fopen(filename, "w");
  if (fd == NULL) return false;
  Write(fd, 0);
  fclose(fd);
  return true;
}

void SegmentationParameters::Write(FILE *fd, int indent) const
{
  double              degree = 45 / atan(1.0);
  const unsigned char *tag;
  int                 itag, num_tags;

  BNF_Write_String(fd, "segmentationparameters", indent, NULL);
  
  // Parameters controlling the data organisation
  BNF_Write_Integer(fd, "nbhmodel", indent+2, nbh_model, "%d");
  BNF_Write_Integer(fd, "octreebinmaxnumpts", indent+2, octree_bin_max_numpts, "%d");
  BNF_Write_Double(fd, "octreebinoverlap", indent+2, octree_bin_overlap, "%.2f");
  BNF_Write_Integer(fd, "distdimension", indent+2, dist_dimension, "%d");
  BNF_Write_Integer(fd, "numnbs", indent+2, num_nbs, "%d");

  // Parameters controlling definition of connected components
  BNF_Write_Double(fd, "maxdistconnected", indent+2, max_dist_connected, "%.2f");
  BNF_Write_Integer(fd, "minnumptscomponent", indent+2, min_numpts_component, "%d");
  BNF_Write_Integer(fd, "componentattribute", indent+2, component_attribute, "%d");
  BNF_Write_Integer(fd, "eraseoldlabels", indent+2, (int) erase_old_labels, "%d");

  // Parameters controlling the seed selection
  BNF_Write_Integer(fd, "seednbhdef", indent+2, seed_neighbourhood, "%d");
  BNF_Write_Double(fd, "seedradius", indent+2, seed_radius, "%.2f");
  BNF_Write_Double(fd, "maxslopeangle", indent+2, max_slope_angle * degree, "%.2f");
  BNF_Write_Double(fd, "binsizeslopeangle", indent+2, bin_size_slope_angle * degree, "%.2f");
  BNF_Write_Double(fd, "binsizedistance", indent+2, bin_size_distance, "%.2f");
  BNF_Write_Integer(fd, "minnumptsseed", indent+2, min_numpts_seed, "%d");
  BNF_Write_Double(fd, "maxdistseedplane", indent+2, max_dist_seed_plane, "%.2f");
  BNF_Write_Double(fd, "maxrefldifseed", indent+2, max_refl_dif_seed, "%.2f");
  
  // Parameters controlling the surface growing
  BNF_Write_Integer(fd, "surfacemodel", indent+2, surface_model, "%d");
  BNF_Write_Integer(fd, "growingnbhdef", indent+2, growing_neighbourhood, "%d");
  BNF_Write_Double(fd, "growingradius", indent+2, growing_radius, "%.2f");
  BNF_Write_Double(fd, "maxdistsurface", indent+2, max_dist_surface, "%.2f");
  BNF_Write_Double(fd, "mindistrecompute", indent+2, min_dist_recompute, "%.2f");
  BNF_Write_Double(fd, "maxrefldifgrowing", indent+2, max_refl_dif_growing, "%.2f");
  BNF_Write_Integer(fd, "competingsurfaces", indent+2, (int) competing_surfaces, "%d");

  // Parameters controlling segment growing based on attributes
  // Neighbourhoods, distances and minimum seed size are as for surface growing
  
  // Loop over all attributes for growing
  for (itag=0, tag=growing_tolerances.AttributeTags();
       itag<growing_tolerances.NumAttributes(); itag++, tag++) {
    if ((LaserPointTag) *tag >= NoTag) continue;
    BNF_Write_Integer(fd, "growingattribute", indent+2, *tag, "%d");
    switch (AttributeType((LaserPointTag) *tag)) {
      default:
      case IntegerAttributeType:
        BNF_Write_Integer(fd, "growingtolerance", indent+2,
		                  growing_tolerances.Attribute(*tag), "%d"); break;
      case FloatAttributeType:
        BNF_Write_Double(fd, "growingtolerance", indent+2,
		                 (double) growing_tolerances.FloatAttribute(*tag), "%.2f"); break;
      case DoubleAttributeType:
        BNF_Write_Double(fd, "growingtolerance", indent+2,
		                 growing_tolerances.DoubleAttribute(*tag), "%.2f"); break;
    }
  }

// Parameters controlling mean shift segmentation
  BNF_Write_Integer(fd, "msattribute", indent+2, XCoordinateTag, "%d");
  BNF_Write_Double(fd, "msbandwidth", indent+2, attribute_band_widths.X(), "%.2f");
  BNF_Write_Integer(fd, "msattribute", indent+2, YCoordinateTag, "%d");
  BNF_Write_Double(fd, "msbandwidth", indent+2, attribute_band_widths.Y(), "%.2f");
  BNF_Write_Integer(fd, "msattribute", indent+2, ZCoordinateTag, "%d");
  BNF_Write_Double(fd, "msbandwidth", indent+2, attribute_band_widths.Z(), "%.2f");

  // Coordinate bandwidths
  // Loop over all attribute band widths
  for (itag=0, tag=attribute_band_widths.AttributeTags();
       itag<attribute_band_widths.NumAttributes(); itag++, tag++) {
    if ((LaserPointTag) *tag >= NoTag) continue;
    BNF_Write_Integer(fd, "msattribute", indent+2, *tag, "%d");
    switch (AttributeType((LaserPointTag) *tag)) {
      default:
      case IntegerAttributeType:
        BNF_Write_Integer(fd, "msbandwidth", indent+2,
		                  attribute_band_widths.Attribute(*tag), "%d"); break;
      case FloatAttributeType:
        BNF_Write_Double(fd, "msbandwidth", indent+2,
		                 (double) attribute_band_widths.FloatAttribute(*tag), "%.2f"); break;
      case DoubleAttributeType:
        BNF_Write_Double(fd, "msbandwidth", indent+2,
		                 attribute_band_widths.DoubleAttribute(*tag), "%.2f"); break;
    }
  }
  BNF_Write_Integer(fd, "msmaxnumitermodeseeking", indent+2, max_num_iter_mode_seeking, "%d");

  // Parameters controlling merging of segments of a completed segmentation
  BNF_Write_Integer(fd, "mergingnbhdef", indent+2, merging_neighbourhood, "%d");
  BNF_Write_Double(fd, "mergingradius", indent+2, merging_radius, "%.2f");
  BNF_Write_Double(fd, "maxangleplanes", indent+2, max_angle_between_planes * degree, "%.2f");
  BNF_Write_Double(fd, "mergingmaxslopeangle", indent+2, merging_max_slope_angle * degree, "%.2f");
  BNF_Write_Double(fd, "maxdistpointtoothersegment", indent+2, max_dist_point_to_other_segment, "%.2f");
  BNF_Write_Integer(fd, "minnumptsbothplanes", indent+2, min_num_pts_both_planes, "%d");

  // Parameters controlling majority filtering
  BNF_Write_Integer(fd, "majoritynbhdef", indent+2, majority_neighbourhood, "%d");
  BNF_Write_Double(fd, "majorityradius", indent+2, majority_radius, "%.2f");
  BNF_Write_Integer(fd, "majorityattribute", indent+2, majority_attribute, "%d");
  BNF_Write_Integer(fd, "majoritynoattributeonly", indent+2, (int) majority_no_attribute_only, "%d");
  BNF_Write_Integer(fd, "majoritynosurfaces", indent+2, (int) majority_no_surfaces, "%d");

  BNF_Write_String(fd, "endsegmentationparameters", indent, NULL);
}

bool SegmentationParameters::Read(const char *filename)
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
        // Look for the start of the segmentation parameter block
        if (!strncmp(keyword, "segmentationparameters", 
                     MAX(keyword_length, 22))) {
          Read(fd);
          fclose(fd);
          free(buffer);
          return true;
        }
      }
    }
  }
  printf("Error: Keyword segmentationparameters not found in file %s\n",
         filename);
  free(buffer);
  return false;
}

void SegmentationParameters::Read(FILE *fd)
{
  char          *buffer, *line, *keyword;
  int           keyword_length;
  double        degree = 45 / atan(1.0);
  LaserPointTag last_growing_attribute = NoTag,
                last_ms_attribute = NoTag;

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        // Parameters controlling the data organisation
        if (!strncmp(keyword, "nbhmodel", MAX(keyword_length, 8)))
          nbh_model = BNF_Integer(line);
          
        else if (!strncmp(keyword, "octreebinmaxnumpts", MAX(keyword_length, 18)))
          octree_bin_max_numpts = BNF_Integer(line);
          
        else if (!strncmp(keyword, "octreebinoverlap", MAX(keyword_length, 16)))
          octree_bin_overlap = BNF_Double(line);
          
        else if (!strncmp(keyword, "distdimension", MAX(keyword_length, 13)))
          dist_dimension = BNF_Integer(line);

        else if (!strncmp(keyword, "numnbs", MAX(keyword_length, 6)))
          num_nbs = BNF_Integer(line);

        // Parameters controlling definition of connected components
        else if (!strncmp(keyword, "maxdistconnected", MAX(keyword_length, 16)))
          max_dist_connected = BNF_Double(line);
          
        else if (!strncmp(keyword, "minnumptscomponent", MAX(keyword_length, 18)))
          min_numpts_component = BNF_Integer(line);

        else if (!strncmp(keyword, "componentattribute", MAX(keyword_length, 18)))
          component_attribute = (LaserPointTag) BNF_Integer(line);

        else if (!strncmp(keyword, "eraseoldlabels", MAX(keyword_length, 14)))
          erase_old_labels = ( BNF_Integer(line) != 0 );

        // Parameters controlling the seed selection
        else if (!strncmp(keyword, "seednbhdef", MAX(keyword_length, 10)))
          seed_neighbourhood = BNF_Integer(line);
          
        else if (!strncmp(keyword, "seedradius", MAX(keyword_length, 10)))
          seed_radius = BNF_Double(line);
          
        else if (!strncmp(keyword, "maxslopeangle", MAX(keyword_length, 13)))
          max_slope_angle = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "binsizeslopeangle", MAX(keyword_length, 17)))
          bin_size_slope_angle = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "binsizedistance", MAX(keyword_length, 15)))
          bin_size_distance = BNF_Double(line);

        else if (!strncmp(keyword, "minnumptsseed", MAX(keyword_length, 13)))
          min_numpts_seed = BNF_Integer(line);
          
        else if (!strncmp(keyword, "maxdistseedplane", MAX(keyword_length, 16)))
          max_dist_seed_plane = BNF_Double(line);
          
        else if (!strncmp(keyword, "maxrefldifseed", MAX(keyword_length, 14)))
          max_refl_dif_seed = BNF_Double(line);

        // Parameters controlling the surface growing
        else if (!strncmp(keyword, "surfacemodel", MAX(keyword_length, 12)))
          surface_model = BNF_Integer(line);
          
        else if (!strncmp(keyword, "growingnbhdef", MAX(keyword_length, 13)))
          growing_neighbourhood = BNF_Integer(line);
          
        else if (!strncmp(keyword, "growingradius", MAX(keyword_length, 13)))
          growing_radius = BNF_Double(line);
          
        else if (!strncmp(keyword, "maxdistsurface", MAX(keyword_length, 14)))
          max_dist_surface = BNF_Double(line);
          
        else if (!strncmp(keyword, "mindistrecompute", MAX(keyword_length, 16)))
          min_dist_recompute = BNF_Double(line);

        else if (!strncmp(keyword, "maxrefldifgrowing", MAX(keyword_length, 17)))
          max_refl_dif_growing = BNF_Double(line);

        else if (!strncmp(keyword, "competingsurfaces", MAX(keyword_length, 17)))
          competing_surfaces = ( BNF_Integer(line) != 0 );

        // Parameters controlling segment growing based on attributes
        // Neighbourhoods, distances and minimum seed size are as for surface growing
        else if (!strncmp(keyword, "growingattribute", MAX(keyword_length, 16)))
          last_growing_attribute = (LaserPointTag) BNF_Integer(line);
        
        else if (!strncmp(keyword, "growingtolerance", MAX(keyword_length, 16))) {
          if (last_growing_attribute == NoTag) {
          	printf("Error: No attribute tag specified for growing tolerance\n");
          }
          else {
          	switch (AttributeType(last_growing_attribute)) {
          	  default:
          	  case IntegerAttributeType:
          	  	growing_tolerances.Attribute(last_growing_attribute) =
          	  	  BNF_Integer(line); break;
          	  case FloatAttributeType:
          	  	growing_tolerances.FloatAttribute(last_growing_attribute) =
          	  	  (float) BNF_Double(line); break;
          	  case DoubleAttributeType:
          	  	growing_tolerances.DoubleAttribute(last_growing_attribute) =
          	  	  BNF_Double(line); break;
          	}
          }
        }

        // Parameters controlling mean shift segmentation
        else if (!strncmp(keyword, "msattribute", MAX(keyword_length, 11)))
          last_ms_attribute = (LaserPointTag) BNF_Integer(line);
        
        else if (!strncmp(keyword, "msbandwidth", MAX(keyword_length, 11))) {
          if (last_ms_attribute == NoTag) {
          	printf("Error: No attribute tag specified for growing tolerance\n");
          }
          else {
          	switch (AttributeType(last_ms_attribute)) {
          	  default:
          	  case IntegerAttributeType:
          	  	attribute_band_widths.Attribute(last_ms_attribute) =
          	  	  BNF_Integer(line); break;
          	  case FloatAttributeType:
          	  	attribute_band_widths.FloatAttribute(last_ms_attribute) =
          	  	  (float) BNF_Double(line); break;
          	  case DoubleAttributeType:
          	  	attribute_band_widths.DoubleAttribute(last_ms_attribute) =
          	  	  BNF_Double(line); break;
          	}
          }
        }

        else if (!strncmp(keyword, "msmaxnumitermodeseeking", MAX(keyword_length, 23)))
          max_num_iter_mode_seeking = BNF_Integer(line);

        // Parameters controlling merging of segments of a completed segmentation
        else if (!strncmp(keyword, "mergingnbhdef", MAX(keyword_length, 13)))
          merging_neighbourhood = BNF_Integer(line);
          
        else if (!strncmp(keyword, "mergingradius", MAX(keyword_length, 13)))
          merging_radius = BNF_Double(line);

        else if (!strncmp(keyword, "maxangleplanes", MAX(keyword_length, 14)))
          max_angle_between_planes = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "mergingmaxslopeangle", MAX(keyword_length, 20)))
          merging_max_slope_angle = BNF_Double(line) / degree;

        else if (!strncmp(keyword, "maxdistpointtoothersegment", MAX(keyword_length, 26)))
          max_dist_point_to_other_segment = BNF_Double(line);

        else if (!strncmp(keyword, "minnumptsbothplanes", MAX(keyword_length, 19)))
          min_num_pts_both_planes = BNF_Integer(line);

        // Parameters controlling majority filtering
        else if (!strncmp(keyword, "majoritynbhdef", MAX(keyword_length, 14)))
          majority_neighbourhood = BNF_Integer(line);
          
        else if (!strncmp(keyword, "majorityradius", MAX(keyword_length, 14)))
          majority_radius = BNF_Double(line);

        else if (!strncmp(keyword, "majorityattribute", MAX(keyword_length, 17)))
          majority_attribute = BNF_Integer(line);

        else if (!strncmp(keyword, "majoritynoattributeonly", MAX(keyword_length, 23)))
          majority_no_attribute_only = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "majoritynosurfaces", MAX(keyword_length, 18)))
          majority_no_surfaces = ( BNF_Integer(line) != 0 );

        // end keyword
        else if (!strncmp(keyword, "endsegmentationparameters", MAX(keyword_length, 25))) {
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

SegmentationParameters::SegmentationParameters(int mode)
{
  SegmentationParameters();                                                   
                                                   
  switch (mode) {
    case 1: //terrestrial lase scan default parameters
            num_nbs = 30;
            bin_size_slope_angle =5;
            seed_radius = 0.5;
            bin_size_distance = 0.1;
            max_dist_seed_plane = 0.1;
            growing_radius = 0.3;
            max_dist_surface = 0.2;
            break;
  }
}
