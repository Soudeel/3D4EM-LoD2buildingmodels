
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
 * \brief Class Buildings - A vector of buildings
 *
 */
/*!
 * \class Buildings
 * \ingroup Photogrammetry
 * \brief Class Buildings - A vector of buildings
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          ---- (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef BUILDINGS_H
#define BUILDINGS_H

#include "Building.h"

class Buildings : public std::vector <Building>
{
  public:
    /// Default constructor
    Buildings() {};

    /// Construct from map data
    Buildings(const char *map_point_file, const char *map_top_file,
              ObjectPoints &map_points, bool &success)
      {success = ImportMapData(map_point_file, map_top_file, map_points);}

    /// Default destructor
    ~Buildings() {};

    /// Return pointer to a building
    Building * BuildingPtr(int number);

    /// Return iterator to a building
    Buildings::iterator BuildingIterator(int number);
    
    /// Return pointer to a building with a specific point
    Building * BuildingPtr(const PointNumber &number, DataType data_type);

    /// Return iterator to a building with a specific point
    Buildings::iterator BuildingIterator(const PointNumber &number,
                                         DataType data_type);

    /// Read new map data into building structures
    bool ImportMapData(const char *map_point_file, const char *map_top_file,
                       ObjectPoints &map_points);

    /// Import PCM map data into building structures
    bool ImportPCMMapData(const char *map_point_file, const char *map_top_file,
                          ObjectPoints &map_points, bool give_warnings=true);

    /// Import PCM model data into building structures
    bool ImportPCMModelData(const char *model_point_file, const char *model_top_file,
                            ObjectPoints &model_points);

   /// Check on the presence of data
    bool ContainsData(DataType type) const;

    /// Set the line numbers based on building and building part numbers
    void SetLineNumbers();

    /// Write all map (topology) data
    bool WriteMapData(const char *map_top_file);

    /// Write all model (topology) data
    bool WriteModelData(const char *model_top_file);

    /// Write all model data as VRML model
    bool WriteVRML(const char *vrml_file);

    /// Write all model data to a DXF file
    bool WriteDXF(const char *dxf_file);

    /// Collect all map data
    void CollectMapData(LineTopologies &map_data, bool collect_map_data=true,
                        bool collect_map_partition_data_in_building=true,
                        bool collect_map_partition_data_in_part=true) const;

    /// Collect all model data
    void CollectModelData(LineTopologies &model_data) const;

    /// Erase all buildings
    void Erase();
    
    /// Delete polygons from the building data
    bool DeletePolygons(const LineTopologies &polygons, const DataType type);

    /// Split a building using new map data
    void SplitBuilding(int building_number,
                       const LineTopologies &new_map_lines);

    /// Merge two buildings
    bool MergeBuildings(int building_number1, int building_number2);

    /// Check if a point is used
    bool Contains(const ObjectPoints *points, const PointNumber &number,
                  const DataType type) const;

    /// Determine the next unused building number
    int NextNumber() const;

    /// Return map or model points
    ObjectPoints * Points(const DataType type);

};
#endif // BUILDINGS_H
