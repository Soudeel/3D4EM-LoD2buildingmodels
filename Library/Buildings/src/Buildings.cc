
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



#include "Buildings.h"
#include "ObjectPoints2D.h"
#include "VRML_io.h"

Building * Buildings::BuildingPtr(int number)
{
  Buildings::iterator building;
  for (building=begin(); building!=end(); building++)
    if (building->Number() == number) return building->BuildingPtr();
  return NULL;
}

Buildings::iterator Buildings::BuildingIterator(int number)
{
  Buildings::iterator building;
  
  for (building=begin(); building!=end(); building++)
    if (building->Number() == number) return building;
  return end();
}

Building * Buildings::BuildingPtr(const PointNumber &number, DataType type)
{
  Buildings::iterator building;
  
  for (building=begin(); building!=end(); building++)
    if (building->Contains(number, type)) return building->BuildingPtr();
  return NULL;
}

Buildings::iterator Buildings::BuildingIterator(const PointNumber &number,
                                                DataType type)
{
  Buildings::iterator building;
  
  for (building=begin(); building!=end(); building++)
    if (building->Contains(number, type)) return building;
  return end();
}

bool Buildings::ImportMapData(const char *map_point_file, const char *map_top_file,
                              ObjectPoints &map_points)
{
  ObjectPoints   new_map_points;
  LineTopologies new_map_top;
  int            next_number=0;

  if (!new_map_points.Read(map_point_file)) return false;
  if (!new_map_top.Read(map_top_file, false)) return false;
  // Renumber the new map points, but not the lines
  if (map_points.size())
    new_map_top.ReNumber(new_map_points, (map_points.end()-1)->Number() + 1, -1);
  // Add the new points
  map_points.insert(map_points.end(), new_map_points.begin(),
                    new_map_points.end());
  // Create a building structure for each map polygon
  if (size()) next_number = (end()-1)->Number() + 1;
  for (LineTopologies::const_iterator polygon=new_map_top.begin();
       polygon != new_map_top.end(); polygon++, next_number++) {
    push_back(Building(polygon->Number(), &map_points,
                       polygon->LineTopologyReference()));
  }
  // Clear local data
  new_map_points.erase(new_map_points.begin(), new_map_points.end());
  new_map_top.erase(new_map_top.begin(), new_map_top.end());
  return true;
}

bool Buildings::ImportPCMMapData(const char *map_point_file,
                                 const char *map_top_file,
                                 ObjectPoints &map_points,
								 bool give_warnings)
{
  ObjectPoints   new_map_points;
  LineTopologies new_map_top;
  int            building_number, part_number;
  Building       *building;

  if (!new_map_points.Read(map_point_file)) return false;
  if (!new_map_top.Read(map_top_file, false)) return false;
  // Add the new points
  map_points.insert(map_points.end(), new_map_points.begin(),
                    new_map_points.end());
  // Insert all building data in either existing or new buildings
  for (LineTopologies::const_iterator polygon=new_map_top.begin();
       polygon != new_map_top.end(); polygon++) {
    // Extract building and building part number
    if (polygon->HasAttribute(BuildingNumberTag)) { // New style
      building_number = polygon->Attribute(BuildingNumberTag);
      if (polygon->HasAttribute(BuildingPartNumberTag))
        part_number = polygon->Attribute(BuildingPartNumberTag);
      else
        part_number = 0; // Map line of whole building
    }
    else { // Old style, building and part numbers in polygon number
      building_number = polygon->Number() / 1000;
      part_number = polygon->Number() - 1000 * building_number;
    }
    // Get the building or create a new one and add the map points
    building = BuildingPtr(building_number);
    if (building) 
      building->AddMapPoints(&map_points);
    else {
      push_back(Building(building_number, &map_points));
      building = (end()-1)->BuildingPtr();
    }
    // Insert building data
    if (part_number == 0) {
      switch (polygon->Label()) {
        case MapLabel:
          building->AddMapData(polygon->LineTopologyReference());
          break;
        case MapPartitionLabel:
          building->AddMapPartitionData(polygon->LineTopologyReference());
          break;
        default: // If no known label, just create a new building
          push_back(Building(polygon->Number(), &map_points,
                             polygon->LineTopologyReference()));
          break;
      }
    }
    // Insert building part data
    else {
      switch (polygon->Label()) {
        case MapPartitionLabel:
          building->AddMapPartitionData(part_number,
                                        polygon->LineTopologyReference());
          break;
        default:
          if (give_warnings)
            printf("Unknown label %d in ImportPCMMapData\n", polygon->Label());
      }
    }
  }
  // Set building and building part numbers (for old style data)
  SetLineNumbers();
  // Clear local data
  new_map_points.erase(new_map_points.begin(), new_map_points.end());
  new_map_top.erase(new_map_top.begin(), new_map_top.end());
    return true;
}

bool Buildings::ImportPCMModelData(const char *model_point_file,
                                   const char *model_top_file,
                                   ObjectPoints &model_points)
{
  ObjectPoints        new_model_points;
  LineTopologies      new_model_top;
  Buildings::iterator building;
  int                 building_number, building_part_number;
  
  if (!new_model_points.Read(model_point_file)) return false;
  if (!new_model_top.Read(model_top_file, false)) return false;
  // Ensure a counter clock wise order of all polygons
  new_model_top.MakeCounterClockWise(new_model_points);
  // Renumber the new model points, but not the line numbers
  if (model_points.size())
    new_model_top.ReNumber(new_model_points,
                           (model_points.end()-1)->Number() + 1, -1);
  // Add the new points
  model_points.insert(model_points.end(), new_model_points.begin(),
                      new_model_points.end());
  // Create a building structure for each set of roof and wall faces
  for (LineTopologies::const_iterator polygon=new_model_top.begin();
       polygon != new_model_top.end(); polygon++) {
    // Determine building and building part numbers
    if (polygon->HasAttribute(BuildingNumberTag)) { // New style
      building_number = polygon->Attribute(BuildingNumberTag);
      building_part_number = polygon->Attribute(BuildingPartNumberTag);
    }
    else { // Old style, building number included in polygon number
      building_number = polygon->Number() / 1000;
      building_part_number = polygon->Number() - 1000 * building_number;
    }
    // Locate building
    building = BuildingIterator(building_number);
    // Create a new one if not found
    if (building == end()) { // Create new building
      push_back(Building(building_number));
      building = end()-1;
    }
    // Add polygon to the building
    building->AddModelData(building_part_number,
                           &model_points, polygon->LineTopologyReference());
  }
  // Set building and building part numbers (for old style data)
  SetLineNumbers();
  // Clear local data
  new_model_points.erase(new_model_points.begin(), new_model_points.end());
  new_model_top.erase(new_model_top.begin(), new_model_top.end());
  return true;
}

bool Buildings::ContainsData(DataType type) const
{
  Buildings::const_iterator building;
  for (building=begin(); building!=end(); building++)
    if (building->ContainsData(type)) return true;
  return false;
}

void Buildings::SetLineNumbers()
{
  for (Buildings::iterator building=begin(); building!=end(); building++)
    building->SetLineNumbers();
}

bool Buildings::WriteMapData(const char *map_top_file)
{
  LineTopologies map_data;
  SetLineNumbers(); // Ensure consistent line numbering
  CollectMapData(map_data);
  int status = map_data.Write(map_top_file);
  map_data.erase(map_data.begin(), map_data.end());
  return (status == 1);
}

bool Buildings::WriteModelData(const char *model_top_file)
{
  LineTopologies model_data;
  SetLineNumbers(); // Ensure consistent line numbering
  CollectModelData(model_data);
  int status = model_data.Write(model_top_file);
  model_data.erase(model_data.begin(), model_data.end());
  return (status == 1);
}

ObjectPoints * Buildings::Points(const DataType type)
{
  Buildings::iterator building;
  ObjectPoints        *points;

  for (building=begin(); building!=end(); building++)
    if ((points = building->Points(type)) != NULL) return points;
  return NULL;
}

bool Buildings::WriteVRML(const char *vrml_file)
{
  LineTopologies model_data;
  ObjectPoints   *model_points, copy_of_model_points;
  FILE           *fd;

  // Check if there are buildings and model points
  if (empty()) return false;
  model_points = Points(ModelData);
  if (model_points == NULL) return false;
  if (model_points->empty()) return false;

  // Collect all topology data
  CollectModelData(model_data);
  if (model_data.empty()) return false;
  SetLineNumbers(); // Ensure consistent line numbering

  // Renumber the points in a copy
  copy_of_model_points.insert(copy_of_model_points.begin(),
                              model_points->begin(), model_points->end());
  model_data.ReNumber(copy_of_model_points);

  // Open the VRML file
  fd = VRML2_Open(vrml_file);
  if (fd == NULL) return false;
  copy_of_model_points.VRML2_Write(fd, model_data, 0);
  VRML2_Close(fd);
  
  // Clean and return success
  copy_of_model_points.erase(copy_of_model_points.begin(),
                             copy_of_model_points.end());
  model_data.erase(model_data.begin(), model_data.end());
  return true;
}

bool Buildings::WriteDXF(const char *dxf_file)
{
  LineTopologies model_data;
  ObjectPoints   *model_points;
  FILE           *fd;

  // Check if there are buildings and model points
  if (empty()) return false;
  model_points = Points(ModelData);
  if (model_points == NULL) return false;
  if (model_points->empty()) return false;

  // Collect all topology data
  CollectModelData(model_data);
  if (model_data.empty()) return false;

  // Open the DXF file
  fd = fopen(dxf_file, "w");
  if (fd == NULL) return false;
  model_points->WriteDXFMesh(fd, model_data, 0, true, true, true, false);
  fclose(fd);
  
  // Clean and return success
  model_data.erase(model_data.begin(), model_data.end());
  return true;
}

void Buildings::CollectMapData(LineTopologies &map_data,
                               bool collect_map_data,
                               bool collect_map_partition_data_in_building,
                               bool collect_map_partition_data_in_part) const
{
  for (Buildings::const_iterator building=begin(); building!=end(); building++)
    building->CollectMapData(map_data, collect_map_data,
                             collect_map_partition_data_in_building,
                             collect_map_partition_data_in_part);
}

void Buildings::CollectModelData(LineTopologies &model_data) const
{
  for (Buildings::const_iterator building=begin(); building!=end(); building++)
    building->CollectModelData(model_data);
}

void Buildings::Erase()
{
  Buildings::iterator building;
  
  // Delete data of all buildings
  for (building=begin(); building!=end(); building++)
    building->DeleteData();
  erase(begin(), end());
}

bool Buildings::DeletePolygons(const LineTopologies &polygons,
                               const DataType type)
{
  Buildings::iterator building;
  bool                deleted_polygon=false;

  for (building=begin(); building!=end(); building++) {
    if (building->DeletePolygons(polygons, type)) {
      deleted_polygon = true;
      if (!building->ContainsData()) { // Delete empty buildings
        erase(building);
        building--;
      }
    }
  }
  return deleted_polygon;
}

void Buildings::SplitBuilding(int building_number,
                              const LineTopologies &new_map_lines)
{
  Buildings::iterator building=BuildingIterator(building_number);
  ObjectPoints        *map_points;
  int                 next_number;

  if (building == end()) return;         // No such building
  if (new_map_lines.size() != 2) return; // Incorrect map data

  // Replace map lines of current building
  building->DeleteData(MapData);
  building->AddMapData(new_map_lines[0]);
  building->SetLineNumbers();
  map_points = building->Points(MapData);

  // Add a new building for the second map line
  building = end() - 1;
  next_number = building->Number() + 1;
  push_back(Building(next_number, map_points, new_map_lines[1]));
  building = end() - 1;
  building->SetLineNumbers();
}

bool Buildings::MergeBuildings(int number1, int number2)
{
  Buildings::iterator      building1, building2;
  LineTopologies::iterator line1, line2, partition2;
  LineTopology             merged_line;
  LineTopologies           *partitions2;
  PointNumber              common_point = PointNumber(-1);

  building1 = BuildingIterator(number1);
  building2 = BuildingIterator(number2);

  // Check if the buildings are their and share the map point set
  if (building1 == end() || building2 == end()) return false;
  if (building1->Points(MapData) != building2->Points(MapData)) return false;

  // Get the lines and do some checks
  if (building1->MapDataPtr() == NULL) return false;
  if (building1->MapDataPtr()->size() != 1) return false;
  line1 = building1->MapDataPtr()->begin();
  if (building2->MapDataPtr() == NULL) return false;
  if (building2->MapDataPtr()->size() != 1) return false;
  line2 = building2->MapDataPtr()->begin();

  // Merge of two closed lines
  if (line1->IsClosed() && line2->IsClosed()) {
    // Insert common nodes
    line1->InsertNodes(building2->Points(MapData)->ObjectPointsRef(),
                       line2->LineTopologyReference(), 0.01);
    line2->InsertNodes(building1->Points(MapData)->ObjectPointsRef(),
                       line1->LineTopologyReference(), 0.01);
    if (!line1->Merge(line2->LineTopologyReference(), merged_line))
      return false;
  }
  // Can not merge open with closed line
  else if (line1->IsClosed() || line2->IsClosed()) {
    printf("Error: Can not merge open with closed polygon\n");
    return false;
  }
  // Joining two open polygons
  else {
    if (*(line1->begin()) == *(line2->begin()) ||
        *(line1->begin()) == *(line2->end()-1)) {
      if (*(line1->begin()) == *(line2->end()-1)) line2->RevertNodeOrder();
      line1->RevertNodeOrder();
    }
    else if (*(line1->end()-1) == *(line2->begin()) ||
             *(line1->end()-1) == *(line2->end()-1)) {
      if (*(line1->end()-1) == *(line2->end()-1)) line2->RevertNodeOrder();
    }
    else {
      printf("Error joining polygons:");
      printf("Selected polygons do not chair end nodes.\n");
      return false;
    }
    merged_line.insert(merged_line.begin(), line1->begin(), line1->end());
    merged_line.insert(merged_line.end(), line2->begin()+1, line2->end());
  }

  // Replace the line of building 1
  building1->MapDataPtr()->Erase(); // Delete old line
  building1->MapDataPtr()->push_back(merged_line); // Add the new one

  // Insert the parts of building 2 into building 1
  if (building2->size())
    building1->insert(building1->end(), building2->begin(), building2->end());

  // Add the partitions of building 2 to those of building 1
  partitions2 = building2->MapPartitionDataPtr();
  if (partitions2 != NULL) {
    if (partitions2->size()) {
      for (partition2=partitions2->begin(); partition2!=partitions2->end();
           partition2++)
        building1->AddMapPartitionData(partition2->LineTopologyReference());
    }
  }

  // Reset the line numbers (to update those of former building 2 lines)
  building1->SetLineNumbers();

  // Delete building 2
  building2->DeleteData(MapData);
  building2->DeleteData(ModelData);
  building2->DeleteData(MapPartitionData);
  erase(building2);
  return true;
}

bool Buildings::Contains(const ObjectPoints *points, const PointNumber &number,
                         const DataType type) const
{
  Buildings::const_iterator building;

  for (building=begin(); building!=end(); building++)
    if (building->Points(type) == points)
      if (building->Contains(number, type)) return true;
  return false;
}

int Buildings::NextNumber() const
{
  Buildings::const_iterator building;
  int                       max_number = 0;

  for (building=begin(); building!=end(); building++)
    if (building->Number() > max_number) max_number = building->Number();
  return max_number + 1;
}
