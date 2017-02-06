
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



/*!
 * \file
 * \brief Class BuildingPart - Outline and 3D model of a building part
 *
 */
/*!
 * \class BuildingPart
 * \ingroup Photogrammetry
 * \brief Class BuildingPart - Outline and 3D model of a building part
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date                ---- (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef BUILDINGPART_H
#define BUILDINGPART_H

#include <stdio.h>
#include <stdlib.h>
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "DataTypes.h"
#include "LineTopsIterVector.h"
#include "LSAdjust.h"
#include "LineSegments2D.h"
#include "DataBounds2D.h"
#include "Planes.h"
#include "Line3D.h"
#include "LaserPoints.h"
#include "FittingParameters.h"

enum RoofType { UnknownRoof, FlatRoof, GableRoof, GableRoofRot, HipRoof,
                HipRoofRot, GambrelRoof, GambrelRoofRot, HipRoof2, HipRoof2Rot,
                SphereRoof, ShedRoof, ConeRoof, CylinderRoof, CylinderRoofRot};
typedef enum RoofType RoofType;

class BuildingPart : public FeatureNumber
{
  protected:
    /// Map points
    ObjectPoints *map_points;

    /// Map partition(s)
    LineTopologies *map_part_top;

    /// Roof type
    RoofType roof_type;

    /// Roof parameters
    vMatrix roof_parameters;
    
    /// Model points
    ObjectPoints *model_points;

    /// Roof faces
    LineTopologies *roof;

    /// Walls
    LineTopologies *walls;
    
    /// Roof planes
    Planes *roof_planes;

  public:
    /// Default constructor
    BuildingPart() : FeatureNumber()
      {roof_type = UnknownRoof; map_points = NULL; model_points = NULL;
       map_part_top = NULL; roof = NULL; walls = NULL; roof_planes = NULL;}

    /// Construct from a number and the map points
    BuildingPart(int part_number, ObjectPoints *points)
      {roof_type = UnknownRoof; Number() = part_number; map_points = points;
       model_points = NULL;
       map_part_top = NULL; roof = NULL; walls = NULL; roof_planes = NULL;}

    /// Default destructor
    ~BuildingPart() {};

    /// Copy assignment
    BuildingPart & operator = (const BuildingPart &);

    /// Return the pointer
    BuildingPart * BuildingPartPtr() {return this;}

    /// Return the map points
    ObjectPoints * MapPoints() const {return map_points;}

    /// Return the map topology
    LineTopologies * MapPartitionTopology() const {return map_part_top;}

    /// Return the model points
    ObjectPoints * ModelPoints() const {return model_points;}

    /// Return the topology of the roof faces
    LineTopologies * RoofFaces() const {return roof;}

    /// Return the topology of the wall faces
    LineTopologies * WallFaces() const {return walls;}

    /// Check the presence of data of a specific type
    bool ContainsData(DataType type) const;

    /// Check the presence of data
    bool ContainsData() const;

    /// Select polygons with a specific point and data type
    void SelectPolygons(LineTopsIterVector &selection,
                        const PointNumber &point_number,
                        DataType type) const;

    /// Add map partition
    void AddMapPartitionData(const LineTopology &new_map_part_top,
                             bool clear_first=false);

    /// Add multiple map partitions
    void AddMapPartitionData(const LineTopologies &new_map_part_tops,
                             bool clear_first=false);

    /// Add model data
    void AddModelData(const LineTopologies &new_model_tops,
                      ObjectPoints *new_model_points=NULL);

    /// Add model data
    void AddModelData(const LineTopology &new_model_top,
                      ObjectPoints *new_model_points=NULL);
                      
    /// Add model points
    void AddModelData(ObjectPoints *new_model_points);

    /// Set line numbers based on building and building part numbers
    void SetLineNumbers(int building_number);

    /// Collect all map data
    void CollectMapData(LineTopologies &map_data) const;

    /// Collect all model data
    void CollectModelData(LineTopologies &model_data) const;

    /// Collect all model points
    void CollectModelPoints(ObjectPoints &points) const;
    
    /// Check if the building part contains a specific point number
    bool Contains(const PointNumber &number, const DataType type) const;

    /// Check if the building part contains a specific polygon
    bool Contains(const LineTopology &polygon,
                  const DataType type) const;

    /// Check if all building part points of a specific type are in a list
    bool IsPartOf(const PointNumberList &numbers, const DataType type) const;

    /// Delete polygons from the building part
    bool DeletePolygons(const LineTopologies &polygons, const DataType type);
    
    /// Crop to polygons of the specified set
    bool CropPolygons(const LineTopologies &polygons, const DataType type);

    /// Delete all data
    void DeleteData();

    /// Delete data of the specified type
    void DeleteData(const DataType type);

    /// Delete data of the specified type and collect deleted point numbers
    void DeleteData(const DataType type, PointNumberList &deleted_points);

    /// Roof height at a specified position
    /** This function only works for specified roof types. If the roof type
        is unknown, roof height 0 is returned.
    */
    double RoofHeight(const Position2D &pos) const;

    /// Calculate partial derivative of roof height with respect to a parameter
    /** This function only works for specified roof types. If the roof type
        is unknown, 0.0 is returned.
    */
    double PartialRoofDerivative(const Position2D &pos, int parameter) const;

    /// Return the number of roof parameters
    int NumberOfRoofParameters() const;

    /// Set the roof type
    void SetRoofType(RoofType type);

    /// Calculate approximate values of the roof parameters
    void ApproximateRoofValues(const LineSegments2D &wall_segments,
                               const LaserPoints &points);

    /// Create a hip roof
    bool CreateHipRoof(ObjectPoints &roof_points);

    /// Create a sphere roof
    void CreateSphereRoof(const LaserPoints &laser_points,
                          ObjectPoints &roof_points, double ground_height);

    /// Create a cone roof
    void CreateConeRoof(const LaserPoints &laser_points,
                        ObjectPoints &roof_points, double ground_height);

    /// Create a cylinder roof
    void CreateCylinderRoof(ObjectPoints &roof_points, double ground_height);

    /// Return the model part topologies
    std::vector <LineTopologies *> * ModelTopology() const;
    
    /// Estimate parameters of a roof
    bool FitRoofPrimitive(const LaserPoints &points,
                          const LineSegments2D &wall_segments, double &rms,
                          int max_iter);

    /// Automatically reconstruct a complex roof
    bool ReconstructComplexRoof(LaserPoints &points,
                                ObjectPoints &model_points,
                                const FittingParameters &parameters,
                                double local_ground_height);

    /// Fit a roof model
    bool ReconstructRoofPrimitive(RoofType type, const LaserPoints &points,
                                  ObjectPoints &model_points,
                                  double ground_height);
    
    // Derive the roof planes from model points and topology
    void DeriveRoofPlanes();
    
    /// Derive residuals of laser points to roof surface
    void DeriveResiduals(LaserPoints &point, bool remove_old_residuals);
                 
    /// Print polygons of a building part
    void Print() const;
    
  private:
    bool PrepareRoofReconstruction(RoofType type, const LaserPoints &points,
                                   ObjectPoints &model_points);
};
#endif // BUILDINGPART_H
