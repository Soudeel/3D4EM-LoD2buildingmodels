
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



#ifndef SEGMENTATIONPARAMETERS_H
#define SEGMENTATIONPARAMETERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LaserPoint.h"

class SegmentationParameters
{
  protected:

// Parameters controlling the data organisation

    /// Neighbourhood storage model (0 = TIN, 1 = Octree, 2 = kd-tree)
    int nbh_model;

    /// Octree bin maximum number of points
    int octree_bin_max_numpts;

    /// Octree bin overlap
    double octree_bin_overlap;

    /// Distance metric dimension (2 = 2D, 3 = 3D)
    int dist_dimension;
    
    /// Number of neighbours in kd-tree
    int num_nbs;

// General segmentation parameters

    /// Minimum number of points in a component
    int min_numpts_component;
    
    /// Attribute for storing result of the segmentation
    LaserPointTag component_attribute;
    
    /// Erase all old component labels before relabelling
    bool erase_old_labels;

// Parameters controlling definition of connected components

    /// Maximum distance between points in a component
    double max_dist_connected;
    
// Parameters controlling the seed selection

    /// Seed point neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
    int seed_neighbourhood;
    
    /// Seed neighbourhood radius
    double seed_radius;

    /// Maximum slope angle of Hough space
    double max_slope_angle;

    /// Bin size of slope angle in Hough space
    double bin_size_slope_angle;

    /// Bin size of distance in Hough space
    double bin_size_distance;

    /// Minimum number of points in seed plane
    int min_numpts_seed;

    /// Maximum distance of points to seed plane
    double max_dist_seed_plane;
    
    /// Maximum reflectance difference in seed plane
    double max_refl_dif_seed;
    
// Parameters controlling the surface growing

    /// Surface model (0=plane, 1=smooth surface)
    int surface_model;

    /// Growing neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
    int growing_neighbourhood;
    
    /// Neighbourhood radius for surface growing
    double growing_radius;

    /// Maximum distance of points to surface
    double max_dist_surface;

    /// Minimum distance for recomputing local plane
    double min_dist_recompute;
    
    /// Maximum reflectance difference in seed plane
    double max_refl_dif_growing;
    
    /// Switch for letting surfaces compete for points
    bool competing_surfaces;
    
// Parameters controlling segment growing based on attributes
// Neighbourhoods, distances and minimum seed size are as for surface growing

    /// Attributes tolerances for segment growing
    LaserPoint growing_tolerances;
        
// Parameters controlling mean shift segmentation

    /// Attributes band widths for mean shift
    LaserPoint attribute_band_widths;
    
    /// Maximum number of iterations for mean shift mode seeking
    int max_num_iter_mode_seeking;

// Parameters controlling merging of segments of a completed segmentation

    /// Merging neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
    int merging_neighbourhood;
    
    /// Neighbourhood radius for merging
    double merging_radius;

    /// Maximum angle between planes
    double max_angle_between_planes;
    
    /// Maximum slope angle
    double merging_max_slope_angle;
    
    /// Maximum distance between point and plane of other segment
    double max_dist_point_to_other_segment;
    
    /// Minimum number of points in both planes in neighbourhood
    int min_num_pts_both_planes;
    
// Parameters controlling majority filtering

    /// Majority filtering neighbourhood definition (0 = TIN/Tree edges, 1 = radius)
    int majority_neighbourhood;
    
    /// Neighbourhood radius for majority filtering
    double majority_radius;

    /// Attribute to be filtered
    LaserPointTag majority_attribute;
    
    /// Only apply majority filtering to points without the attribute
    bool majority_no_attribute_only;

    /// Do not apply majority filtering to surfaces
    bool majority_no_surfaces;
    
  public:
    /// Default constructor
    SegmentationParameters();
;    
    /// Constructor, mode indicate some pre-defined parameter values
    SegmentationParameters(int mode);

    /// Return the const reference
    const SegmentationParameters & SegmentationParametersRef() const {return *this;}

    /// Return the reference
    SegmentationParameters & SegmentationParametersRef() {return *this;}

// Readable parameters

    /// Return the neighbourhood storage model (readable)
    int NeighbourhoodStorageModel() const {return nbh_model;}
    
    /// Return the octree bin size (readable)
    int OctreeBinMaxNumberOfPoints() const {return octree_bin_max_numpts;}
    
    /// Return the octree bin overlap (readable)
    double OctreeBinOverlap() const {return octree_bin_overlap;}
    
    /// Return the distance metric dimension (readable)
    int DistanceMetricDimension() const {return dist_dimension;}
    
    /// Return the number of neighbours for the kd-tree (readable)
    int NumberOfNeighbours() const {return num_nbs;}
    
    /// Return the maximum distance between points in a component (readable)
    double MaxDistanceInComponent() const {return max_dist_connected;}
    
    /// Return the minimum number of points in a component (readable)
    int MinNumberOfPointsComponent() const {return min_numpts_component;}

    /// Return the minimum number of points in a segment (readable)
    int MinNumberOfPointsSegment() const {return min_numpts_component;}

    /// Return the attribute used for storing results of connected component labelling (readable)
    LaserPointTag ComponentAttribute() const {return component_attribute;}
    
    /// Return the attribute used for storing results of the segmentation (readable)
    LaserPointTag SegmentAttribute() const {return component_attribute;}
    
    /// Return the switch for erasing old segment numbers (readable)
    bool EraseOldLabels() const {return erase_old_labels;}

    /// Return the switch for erasing old segment numbers (readable)
    bool EraseOldNumbers() const {return erase_old_labels;}

    /// Return the seed neighbourhood definition (readable)
    int SeedNeighbourhoodDefinition() const {return seed_neighbourhood;}
    
    /// Return the seed neighbourhood radius (readable)
    double SeedNeighbourhoodRadius() const {return seed_radius;}

    /// Return the maximum slope angle of the Hough space (readable)
    double MaxSlopeAngle() const {return max_slope_angle;}

    /// Return the bin size of the slope angle in the Hough space (readable)
    double BinSizeSlopeAngle() const {return bin_size_slope_angle;}

    /// Return the bin size of the distance in the Hough space (readable)
    double BinSizeDistance() const {return bin_size_distance;}

    /// Return the minimum number of points in a seed (readable)
    int MinNumberOfPointsSeed() const {return min_numpts_seed;}

    /// Return the maximum distance between a point and the seed plane (readable)
    double MaxDistanceSeedPlane() const {return max_dist_seed_plane;}
    
    /// Return the maximum reflectance difference in the seed (readable)
    double MaxReflectanceDifferenceSeed() const {return max_refl_dif_seed;}

    /// Return the surface model (readable)
    int SurfaceModel() const {return surface_model;}

    /// Return the growing neighbourhood definition (readable)
    int GrowingNeighbourhoodDefinition() const {return growing_neighbourhood;}
    
    /// Return the radius for surface growing (readable)
    double GrowingRadius() const {return growing_radius;}

    /// Return the maximum distance between a point and the surface (readable)
    double MaxDistanceSurface() const {return max_dist_surface;}

    /// Return the minimum distance for recomputing local plane (readable)
    double MinDistanceRecompute() const {return min_dist_recompute;}
    
    /// Return the maximum reflectance difference in the growing phase (readable)
    double MaxReflectanceDifferenceGrowing() const {return max_refl_dif_growing;}

    /// Return the switch for letting surfaces compete (readable)
    bool SurfacesCompete() const {return competing_surfaces;}

    /// Return attribute tolerances for segment growing (readable)
    LaserPoint GrowingTolerances() const {return growing_tolerances;}

    /// Return attribute band widths for mean shift (readable)
    LaserPoint AttributeBandWidths() const {return attribute_band_widths;}

;    /// Maximum number of iterations for mean shift mode seeking (readable)
    int MaxNumIterationsModeSeeking() const {return max_num_iter_mode_seeking;}

    /// Merging neighbourhood definition (0 = TIN/Tree edges, 1 = radius) (readable)
    int MergingNeighbourhoodDefinition() const {return merging_neighbourhood;}
    
    /// Neighbourhood radius for merging (readable)
    double MergingNeighbourhoodRadius() const {return merging_radius;}

    /// Return the maximum angle between planes (readable)
    double MaxAnglePlanes() const {return max_angle_between_planes;}
    
    /// Return the maximum slope angle of planes to be merged (readable)
    double MergingMaxSlopeAngle() const {return merging_max_slope_angle;}

    /// Return the maximum distance between point and plane of other segment (readable)
    double MaxDistPointToOtherSegment() const {return max_dist_point_to_other_segment;}
    
    /// Minimum number of points in both planes in neighbourhood (readable)
    int MinNumberPointsBothPlanes() const {return min_num_pts_both_planes;}
    
    /// Majority filtering neighbourhood definition (0 = TIN/Tree edges, 1 = radius) (readable)
    int MajorityNeighbourhoodDefinition() const {return majority_neighbourhood;}
    
    /// Neighbourhood radius for majority filtering (readable)
    double MajorityNeighbourhoodRadius() const {return majority_radius;}

    /// Attribute to be filtered (readable)
    LaserPointTag MajorityAttribute() const {return majority_attribute;}
    
    /// Only apply majority filtering to points without the attribute (readable)
    bool MajorityNoAttributeOnly() const {return majority_no_attribute_only;}
    
    /// Do not apply majority filtering to surfaces (readable)
    bool MajorityNoSurfaces() const {return majority_no_surfaces;}

// Writable parameters

    /// Return the neighbourhood storage model (writable)
    int & NeighbourhoodStorageModel() {return nbh_model;}
    
    /// Return the octree bin size (writable)
    int & OctreeBinMaxNumberOfPoints() {return octree_bin_max_numpts;}
    
    /// Return the octree bin overlap (writable)
    double & OctreeBinOverlap() {return octree_bin_overlap;}
    
    /// Return the distance metric dimension (writable)
    int & DistanceMetricDimension() {return dist_dimension;}
    
    /// Return the number of neighbours for the kd-tree (writable)
    int & NumberOfNeighbours() {return num_nbs;}
    
    /// Return the maximum distance between points in a component (writable)
    double & MaxDistanceInComponent() {return max_dist_connected;}
    
    /// Return the minimum number of points in a component (writable)
    int & MinNumberOfPointsComponent() {return min_numpts_component;}

    /// Return the minimum number of points in a segment (writable)
    int & MinNumberOfPointsSegment() {return min_numpts_component;}

    /// Return the attribute used for storing results of connected component labelling (writable)
    LaserPointTag & ComponentAttribute() {return component_attribute;}
    
    /// Return the attribute used for storing results of the segmentation (writable)
    LaserPointTag & SegmentAttribute() {return component_attribute;}
    
    /// Return the switch for erasing old segment numbers (writable)
    bool & EraseOldLabels() {return erase_old_labels;}
    
    /// Return the switch for erasing old segment numbers (writable)
    bool & EraseOldNumbers() {return erase_old_labels;}
    
    /// Return the seed neighbourhood definition (writable)
    int & SeedNeighbourhoodDefinition() {return seed_neighbourhood;}
    
    /// Return the seed neighbourhood radius (writable)
    double & SeedNeighbourhoodRadius() {return seed_radius;}

    /// Return the maximum slope angle of the Hough space (writable)
    double & MaxSlopeAngle() {return max_slope_angle;}

    /// Return the bin size of the slope angle in the Hough space (writable)
    double & BinSizeSlopeAngle() {return bin_size_slope_angle;}

    /// Return the bin size of the distance in the Hough space (writable)
    double & BinSizeDistance() {return bin_size_distance;}

    /// Return the minimum number of points in a seed (writable)
    int & MinNumberOfPointsSeed() {return min_numpts_seed;}

    /// Return the maximum distance between a point and the seed plane (writable)
    double & MaxDistanceSeedPlane() {return max_dist_seed_plane;}

    /// Return the maximum reflectance difference in the seed (writable)
    double & MaxReflectanceDifferenceSeed() {return max_refl_dif_seed;}

    /// Return the surface model (writable)
    int & SurfaceModel() {return surface_model;}

    /// Return the growing neighbourhood definition (writable)
    int & GrowingNeighbourhoodDefinition() {return growing_neighbourhood;}
    
    /// Return the radius for surface growing (writable)
    double & GrowingRadius() {return growing_radius;}

    /// Return the maximum distance between a point and the surface (writable)
    double & MaxDistanceSurface() {return max_dist_surface;}

    /// Return the minimum distance for recomputing local plane (writable)
    double & MinDistanceRecompute() {return min_dist_recompute;}
    
    /// Return the maximum reflectance difference in the growing phase (writable)
    double & MaxReflectanceDifferenceGrowing() {return max_refl_dif_growing;}

    /// Return the switch for letting surfaces compete (writable)
    bool & SurfacesCompete() {return competing_surfaces;}
    
    /// Return attribute tolerances for segment growing (writable)
    LaserPoint & GrowingTolerances() {return growing_tolerances;}

    /// Return attribute band widths for mean shift (writable)
    LaserPoint & AttributeBandWidths() {return attribute_band_widths;}

    /// Maximum number of iterations for mean shift mode seeking (writable)
    int & MaxNumIterationsModeSeeking() {return max_num_iter_mode_seeking;}

    /// Merging neighbourhood definition (0 = TIN/Tree edges, 1 = radius) (writable)
    int & MergingNeighbourhoodDefinition() {return merging_neighbourhood;}
    
    /// Neighbourhood radius for merging (writable)
    double & MergingNeighbourhoodRadius() {return merging_radius;}

    /// Return the maximum angle between planes (writable)
    double & MaxAnglePlanes() {return max_angle_between_planes;}
    
     /// Return the maximum slope angle of planes to be merged (writable)
    double & MergingMaxSlopeAngle() {return merging_max_slope_angle;}

    /// Return the maximum distance between point and plane of other segment (writable)
    double & MaxDistPointToOtherSegment() {return max_dist_point_to_other_segment;}

    /// Minimum number of points in both planes in neighbourhood (writable)
    int & MinNumberPointsBothPlanes() {return min_num_pts_both_planes;}
    
    /// Majority filtering neighbourhood definition (0 = TIN/Tree edges, 1 = radius) (writable)
    int & MajorityNeighbourhoodDefinition() {return majority_neighbourhood;}
    
    /// Neighbourhood radius for majority filtering (writable)
    double & MajorityNeighbourhoodRadius() {return majority_radius;}

    /// Attribute to be filtered (writable)
    LaserPointTag & MajorityAttribute() {return majority_attribute;}
    
    /// Only apply majority filtering to points without the attribute (writable)
    bool & MajorityNoAttributeOnly() {return majority_no_attribute_only;}

    /// Do not apply majority filtering to surfaces (readable)
    bool & MajorityNoSurfaces() {return majority_no_surfaces;}

// I/O functions

    /// Write parameter values in BNF style
    bool Write(const char *filename) const;
    
    /// Write parameter values in BNF style
    void Write(FILE *fd, int indent) const;
    
    /// Read parameter values from a BNF file
    bool Read(const char *filename);
    
    /// Read parameter values from a BNF file
    void Read(FILE *fd);
};

#endif // SEGMENTATIONPARAMETERS_H
