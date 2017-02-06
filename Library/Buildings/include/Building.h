
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
 * \brief Class Building - Outline and building parts
 *
 */
/*!
 * \class Building
 * \ingroup Photogrammetry
 * \brief Class Building - Outline and building parts
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          ----- (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef BUILDING_H
#define BUILDING_H

#include "BuildingPart.h"
#include "LineTopsIterVector.h"

class Building : public FeatureNumber, public vector <BuildingPart>
{
  protected:

    /// Map points
    ObjectPoints *map_points;
    /// Outline
    LineTopologies *map_top;
    /// Partitioning
    LineTopologies *map_part_top;
    
    /// Model points
    ObjectPoints *model_points;
    
  public:
    /// Default constructor
    Building() {map_points = NULL; map_top = NULL; map_part_top=NULL;}

    /// Construct from only a number
    Building(int building_number)
      {Number() = building_number; map_points = NULL;
       map_top = map_part_top = NULL;}

    /// Construct from a number and map points
    Building(int building_number, ObjectPoints *points);

    /// Construct from a map polygon
    Building(int building_number, ObjectPoints *points,
             const LineTopology &top);

    /// Default destructor
    ~Building() {};

    /// Return the reference
    Building & BuildingRef() {return *this;}

    /// Return the building pointer
    Building * BuildingPtr() {return this;}

    /// Return a specific building part
    BuildingPart * BuildingPartPtr(int number);
    
    /// Return a building part with a specific point number
    BuildingPart * BuildingPartPtr(const PointNumber &number, DataType type);

    /// Return a building part with a specific line
    BuildingPart * BuildingPartPtr(const LineTopology &polygon, DataType type);

    /// Return the const reference
    const Building & BuildingRef() const {return *this;}

    /// Return all model parts
    std::vector <LineTopologies *> * ModelTopology() const;

    /// Return all roof faces
    std::vector <LineTopologies *> * RoofFaces() const;

    /// Return all wall faces
    std::vector <LineTopologies *> * WallFaces() const;

    /// Return object points of a specific data type
    ObjectPoints * Points(DataType type) const;

    /// Return the object topology of a specific data type
    std::vector <LineTopologies *> * Topology(DataType type) const;

    /// Return the map line topology
    LineTopologies * MapDataPtr() {return map_top;}

    /// Return the map partition line topology
    LineTopologies * MapPartitionDataPtr() {return map_part_top;}

    /// Check for data presence of a specific type
    bool ContainsData(DataType type) const;

    /// Check for data presence
    bool ContainsData() const;

    /// Retrieve the next unused part number
    int NextPartNumber() const;

    /// Add map points
    void AddMapPoints(ObjectPoints *new_map_points);

    /// Add map topology
    void AddMapData(const LineTopology &new_map_top);

    /// Add map partition topology
    void AddMapPartitionData(const LineTopology &new_map_part_top,
                             bool clear_first=false);

    /// Add map partition topologies
    void AddMapPartitionData(const LineTopologies &new_map_part_tops,
                             bool clear_first=false);

    /// Add map partition topology to building part
    void AddMapPartitionData(int part_number,
                             const LineTopology &new_map_part_top);

    /// Add map partition topologies to building part
    void AddMapPartitionData(int part_number,
                             const LineTopologies &new_map_part_tops);

    /// Add model data
    void AddModelData(int part_number, ObjectPoints *new_model_points,
                      const LineTopology &new_model_top);

    /// Add model points
    void AddModelData(ObjectPoints *new_model_points);

    /// Set the line numbers based on the building and building part numbers
    void SetLineNumbers();

    /// Collect all map data
    void CollectMapData(LineTopologies &map_data, bool collect_map_data=true,
                        bool collect_map_partition_data_in_building=true,
                        bool collect_map_partition_data_in_part=true) const;

    /// Collect all model data
    void CollectModelData(LineTopologies &model_data) const;

    /// Select polygons with a specific point and data type
    void SelectPolygons(LineTopsIterVector &selection,
                        const PointNumber &point_number,
                        DataType type) const;

    /// Select polygons with one or more specified points and data type
    void SelectPolygons(LineTopsIterVector &selection,
                        const PointNumberList &point_numbers,
                        DataType type) const;

    /// Delete polygons from the building
    bool DeletePolygons(const LineTopologies &polygons, const DataType type);

    /// Crop polygons of a building to specified collection
    bool CropPolygons(const LineTopologies &polygons, const DataType type);
    
    /// Check if the building contains a point of a specific type
    bool Contains(const PointNumber &number, const DataType type) const;

    /// Check if all buildings points of a specific type are part of a list
    bool IsPartOf(const PointNumberList &numbers, const DataType type) const;

    /// Delete all building data
    void DeleteData();
    
    /// Delete building data of a specific type
    bool DeleteData(const DataType type);

    /// Delete building data of a specific type and collect deleted point no's
    bool DeleteData(const DataType type, PointNumberList &deleted_points);

    /// Split a map partition into two parts
    bool SplitMapPartition(LineTopologies::iterator old_partition,
                           const LineTopologies &new_partitions);

    /// Merge two partitions
    bool MergePartitions(LineTopologies::iterator partition1,
                         LineTopologies::iterator partition2);

    /// Partition a building (partition) contour
    int Partition(const DataType type,
                  LineTopsIterVector::iterator partition_line);

    /// Retrieve a building part based on a map partition outline
    Building::iterator RetrievePart(const LineTopology &part);

    /// Retrieve a building part based on a set of map partition outlines
    Building::iterator RetrievePart(const LineTopologies &part);
    
    /// Print all polygons of a building
    void Print() const;
    
    /// Print all polygons of a specific type
    void Print(const DataType type) const;
};

#endif // BUILDING_H
