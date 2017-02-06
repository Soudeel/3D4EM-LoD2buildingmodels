
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



#include "Building.h"

// Stuff needed for building partitioning

#include "ObjectPoints2D.h"
#include "House.h"
#include "PartitioningRules.h"
#include "DataBounds2D.h"

#define MAP_PTS "part.pts"
#define MAP_TOP "part.top"

#define OFFSET_X 200000
#define OFFSET_Y 480000


Building::Building(int building_number, ObjectPoints *new_map_points)
{
  Number()     = building_number;
  map_points   = new_map_points;
  map_top      = NULL;
  map_part_top = NULL;
  model_points = NULL;
}

Building::Building(int building_number, ObjectPoints *new_map_points,
                   const LineTopology &new_map_top)
{
  Number()   = building_number;
  map_points = new_map_points;
  map_top    = new LineTopologies();
  map_top->push_back(new_map_top);
  map_top->begin()->Attribute(BuildingNumberTag) = building_number;
  map_top->begin()->Label() = MapLabel;
  map_part_top = NULL;
  model_points = NULL;
}

void Building::AddMapPoints(ObjectPoints *new_map_points)
{
  map_points = new_map_points;
}

void Building::AddMapData(const LineTopology &new_map_top)
{
//  if (map_top) map_top->erase(map_top->begin(), map_top->end());
//  else map_top = new LineTopologies();
//  map_top->push_back(new_map_top);
	if (!map_top) map_top = new LineTopologies();
	map_top->push_back(new_map_top);
}

void Building::AddMapPartitionData(const LineTopology &new_map_part_top,
                                   bool clear_first)
{
  // Clear old map partition data (optional)
  if (map_part_top) {
    if (clear_first)
      map_part_top->erase(map_part_top->begin(), map_part_top->end());
  }
  // Make sure we have a vector
  else map_part_top = new LineTopologies();
  // Add new partition
  map_part_top->push_back(new_map_part_top);
  // Set the building number and label
  (map_part_top->end()-1)->Attribute(BuildingNumberTag) = Number();
  (map_part_top->end()-1)->Label()  = MapPartitionLabel;
}

void Building::AddMapPartitionData(const LineTopologies &new_map_part_tops,
                                   bool clear_first)
{
  LineTopologies::const_iterator new_map_part_top;
  
  for (new_map_part_top=new_map_part_tops.begin();
       new_map_part_top!=new_map_part_tops.end(); new_map_part_top++)
    AddMapPartitionData(new_map_part_top->LineTopologyReference(),
                        clear_first &&
                        new_map_part_top == new_map_part_tops.begin());
}

void Building::AddMapPartitionData(int part_number,
                                   const LineTopology &new_map_part_top)
{
  BuildingPart *part = BuildingPartPtr(part_number);
  if (!part) {
    push_back(BuildingPart(part_number, map_points));
    part = (end()-1)->BuildingPartPtr();
  }
  part->AddMapPartitionData(new_map_part_top, true);
}

void Building::AddMapPartitionData(int part_number,
                                   const LineTopologies &new_map_part_tops)
{
  BuildingPart *part = BuildingPartPtr(part_number);
  if (!part) {
    push_back(BuildingPart(part_number, map_points));
    part = (end()-1)->BuildingPartPtr();
  }
  part->AddMapPartitionData(new_map_part_tops, true);
}

void Building::AddModelData(int part_number, ObjectPoints *new_model_points,
                            const LineTopology &new_model_top)
{
  BuildingPart *part;

  model_points = new_model_points;
  part = BuildingPartPtr(part_number);
  if (!part) {
    push_back(BuildingPart(part_number, map_points));
    part = (end()-1)->BuildingPartPtr();
  }
  part->AddModelData(new_model_top, new_model_points);
}

void Building::AddModelData(ObjectPoints *new_model_points)
{
  model_points = new_model_points;
}

BuildingPart * Building::BuildingPartPtr(int number)
{
  Building::iterator part;
  for (part=begin(); part!=end(); part++)
    if (part->Number() == number) return part->BuildingPartPtr();
  return NULL;
}

BuildingPart * Building::BuildingPartPtr(const PointNumber &number,
                                         DataType type)
{
  Building::iterator part;
  for (part=begin(); part!=end(); part++)
    if (part->Contains(number, type)) return part->BuildingPartPtr();
  return NULL;
}

BuildingPart * Building::BuildingPartPtr(const LineTopology &polygon,
                                         DataType type)
{
  Building::iterator part;
  for (part=begin(); part!=end(); part++)
    if (part->Contains(polygon, type)) return part->BuildingPartPtr();
  return NULL;
}

std::vector <LineTopologies *> * Building::ModelTopology() const
{
  Building::const_iterator building_part;
  std::vector <LineTopologies *> *model_parts;

  model_parts = new std::vector <LineTopologies *>;
  for (building_part=begin(); building_part!=end(); building_part++) {
    if (building_part->RoofFaces())
      model_parts->push_back(building_part->RoofFaces());
    if (building_part->WallFaces())
      model_parts->push_back(building_part->WallFaces());
  }
  return model_parts;
}

std::vector <LineTopologies *> * Building::RoofFaces() const
{
  Building::const_iterator building_part;
  std::vector <LineTopologies *> *roof_faces;

  roof_faces = new std::vector <LineTopologies *>;
  for (building_part=begin(); building_part!=end(); building_part++)
    roof_faces->push_back(building_part->RoofFaces());
  return roof_faces;
}

std::vector <LineTopologies *> * Building::WallFaces() const
{
  Building::const_iterator building_part;
  std::vector <LineTopologies *> *wall_faces;

  wall_faces = new std::vector <LineTopologies *>;
  for (building_part=begin(); building_part!=end(); building_part++)
    wall_faces->push_back(building_part->WallFaces());
  return wall_faces;
}

ObjectPoints *Building::Points(DataType type) const
{
  switch (type) {
    case MapData:
    case MapPartitionData:
      return map_points;
    case ModelData:
    case LastModelData:
      return model_points;
    default:
      printf("Unrecognised data type %d in Building::Points(DataType)\n", type);
  }
  return NULL;  
}

std::vector <LineTopologies *> * Building::Topology(DataType type) const
{
  std::vector <LineTopologies *> *faces;
  switch (type) {
    case MapData: 
    case SelectedMapData:
      faces = new std::vector <LineTopologies *>;
      if (map_top) faces->push_back(map_top);
      break;
    case MapPartitionData:
    case SelectedMapPartitionData:
      faces = new std::vector <LineTopologies *>;
      if (map_part_top) faces->push_back(map_part_top);
      break;
    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
    case LastModelData:
    case SelectedModelFaceData:
      faces = ModelTopology();
      break;
    default:
      printf("Unrecognised data type %d in Building::Topology(DataType)\n",
             type);
      faces = new std::vector <LineTopologies *>; // Just an empty vector
      break;
  }
  return faces;
}

bool Building::ContainsData(DataType type) const
{
  Building::const_iterator building_part;

  switch (type) {
    case MapData:
      if (map_points && map_top)
        if (map_points->size() > 1 && map_top->size()) return true;
      break;
    case MapPartitionData:
      if (map_points && map_part_top)
        if (map_points->size() > 1 && map_part_top->size()) return true;
      break;
    case ModelData:
    case LastModelData:
      for (building_part=begin(); building_part!=end(); building_part++) {
        if (building_part->ContainsData(type)) return true;
      }
      break;
    case LaserData:
      return false;
    default:
      printf("Unrecognised data type %d in Building::ContainsData(DataType)\n",
             type);
      break;
  }
  return false;
}

bool Building::ContainsData() const
{
  if (ContainsData(MapData)) return true;
  if (ContainsData(MapPartitionData)) return true;
  if (ContainsData(ModelData)) return true;
  return false;
}

void Building::SetLineNumbers()
{
  LineTopologies::iterator polygon;
  if (map_top)
    for (polygon=map_top->begin(); polygon!=map_top->end(); polygon++)
      polygon->Attribute(BuildingNumberTag) = Number();
  if (map_part_top)
    for (polygon=map_part_top->begin(); polygon!=map_part_top->end(); polygon++) {
      polygon->Attribute(BuildingNumberTag) = Number();
    }
  for (Building::iterator part=begin(); part!=end(); part++)
    part->SetLineNumbers(Number());
}

void Building::CollectMapData(LineTopologies &map_data,
                              bool collect_map_data,
                              bool collect_map_partition_data_in_building,
                              bool collect_map_partition_data_in_part) const
{
  LineTopologies::const_iterator polygon;
  Building::const_iterator part;

  if (map_top && collect_map_data)
    for (polygon=map_top->begin(); polygon!=map_top->end(); polygon++)
      map_data.push_back(polygon->LineTopologyReference());
  if (map_part_top && collect_map_partition_data_in_building)
    for (polygon=map_part_top->begin(); polygon!=map_part_top->end(); polygon++)
      map_data.push_back(polygon->LineTopologyReference());
  if (collect_map_partition_data_in_part)
    for (part=begin(); part!=end(); part++) part->CollectMapData(map_data);
}

void Building::CollectModelData(LineTopologies &model_data) const
{
  for (Building::const_iterator part=begin(); part!=end(); part++)
    part->CollectModelData(model_data);
}

void Building::SelectPolygons(LineTopsIterVector &selection,
                              const PointNumber &number, DataType type) const
{
  std::vector <LineTopologies *>           *top_sets;
  std::vector <LineTopologies *>::iterator top_set;
  LineTopologies::iterator                 top;
  LineTopologies                           *model_part;

  switch (type) {
    // Select map (partition) line with the point number
    case MapData:
    case SelectedMapData:
    case MapPartitionData:
    case SelectedMapPartitionData:
      top_sets = Topology(type);
      for (top_set=top_sets->begin(); top_set!=top_sets->end(); top_set++)
        for (top=(*top_set)->begin(); top!=(*top_set)->end(); top++)
          if (top->Contains(number)) selection.push_back(top);
      break;

    // Select the whole model if the point number is contained
    case ModelData:
    case SelectedModelData:
      if (Contains(number, type)) {
        top_sets = Topology(type);
        for (top_set=top_sets->begin(); top_set!=top_sets->end(); top_set++)
          for (top=(*top_set)->begin(); top!=(*top_set)->end(); top++)
            selection.push_back(top);
      }
      break;

    // Select the model part or model face if the point number is contained
    case SelectedModelPartData:
    case SelectedModelFaceData:
      for (Building::const_iterator part=begin(); part!=end(); part++) {
        if (part->Contains(number, type)) {
          model_part = part->RoofFaces();
          for (top=model_part->begin(); top!=model_part->end(); top++)
            if (type == SelectedModelPartData || top->Contains(number))
              selection.push_back(top);
          model_part = part->WallFaces();
          for (top=model_part->begin(); top!=model_part->end(); top++)
            if (type == SelectedModelPartData || top->Contains(number))
              selection.push_back(top);
        }
      }
      break;

    default:
      break;
  }
}

void Building::SelectPolygons(LineTopsIterVector &selection,
                              const PointNumberList &numbers,
                              DataType type) const
{
  std::vector <LineTopologies *>           *top_sets;
  std::vector <LineTopologies *>::iterator top_set;
  LineTopologies::iterator                 top;
  LineTopologies                           *model_part;

  switch (type) {
    // Select map (partition) line with the point number
    case MapData:
    case SelectedMapData:
    case MapPartitionData:
    case SelectedMapPartitionData:
      top_sets = Topology(type);
      for (top_set=top_sets->begin(); top_set!=top_sets->end(); top_set++)
        for (top=(*top_set)->begin(); top!=(*top_set)->end(); top++)
          if (top->IsPartOf(numbers)) selection.push_back(top);
      break;

    // Select the whole model if all points are in the number list
    case ModelData:
    case SelectedModelData:
      if (IsPartOf(numbers, type)) {
        top_sets = Topology(type);
        for (top_set=top_sets->begin(); top_set!=top_sets->end(); top_set++)
          for (top=(*top_set)->begin(); top!=(*top_set)->end(); top++)
            selection.push_back(top);
      }
      break;

    // Select the model part if all points are in the number list
    case SelectedModelPartData:
      for (Building::const_iterator part=begin(); part!=end(); part++) {
        if (part->IsPartOf(numbers, type)) {
          model_part = part->RoofFaces();
          for (top=model_part->begin(); top!=model_part->end(); top++)
            selection.push_back(top);
          model_part = part->WallFaces();
          for (top=model_part->begin(); top!=model_part->end(); top++)
            selection.push_back(top);
        }
      }
      break;

    default:
      break;
  }
}

bool Building::DeletePolygons(const LineTopologies &polygons,
                              const DataType type)
{
  bool                     deleted_polygon = false;
  LineTopologies::iterator polygon;
  LineTopologies           *map_data;
  Building::iterator       part;

  switch (type) {
    case MapData:
    case MapPartitionData:
      if (type == MapData) map_data = map_top;
      else map_data = map_part_top;
      if (map_data) {
        for (polygon=map_data->begin(); polygon!=map_data->end(); polygon++) {
          if (polygons.Contains(*polygon)) {
            map_data->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
      }
      break;

    case ModelData:
      for (part=begin(); part!=end(); part++) {
        if (part->DeletePolygons(polygons, type)) {
          deleted_polygon = true;
          if (!part->ContainsData()) { // Delete empty building parts
            erase(part);
            part--;
          }
        }
      }
      break;

    default:
      break;
  }
  return deleted_polygon;
}

bool Building::CropPolygons(const LineTopologies &polygons,
                            const DataType type)
{
  bool                     deleted_polygon = false;
  LineTopologies::iterator polygon;
  LineTopologies           *map_data;
  Building::iterator       part;

  switch (type) {
    case MapData:
    case MapPartitionData:
      if (type == MapData) map_data = map_top;
      else map_data = map_part_top;
      if (map_data) {
        for (polygon=map_data->begin(); polygon!=map_data->end(); polygon++) {
          if (!polygons.Contains(*polygon)) {
            map_data->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
      }
      break;

    case ModelData:
      for (part=begin(); part!=end(); part++) {
        if (part->CropPolygons(polygons, type)) {
          deleted_polygon = true;
          if (!part->ContainsData()) { // Delete empty building parts
            erase(part);
            part--;
          }
        }
      }
      break;

    default:
      break;
  }
  return deleted_polygon;
}
                              
bool Building::Contains(const PointNumber &number, const DataType type) const
{
  LineTopologies::const_iterator polygon;
  Building::const_iterator       part;

  switch (type) {
    case MapData:
    case SelectedMapData:
      if (map_top)
        for (polygon=map_top->begin(); polygon!=map_top->end(); polygon++)
          if (polygon->Contains(number)) return true;
      break;

    case MapPartitionData:
    case SelectedMapPartitionData:
      if (map_part_top)
        for (polygon=map_part_top->begin();
             polygon!=map_part_top->end(); polygon++)
          if (polygon->Contains(number)) return true;
      break;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      for (part=begin(); part!=end(); part++)
        if (part->Contains(number, type)) return true;
      break;

    default:
      break;
  }
  return false;
}

bool Building::IsPartOf(const PointNumberList &numbers,
                        const DataType type) const
{
  LineTopologies::const_iterator polygon;
  Building::const_iterator       part;

  switch (type) {
    case MapData:
    case SelectedMapData:
      if (map_top)
        return map_top->IsPartOf(numbers);
      break;

    case MapPartitionData:
    case SelectedMapPartitionData:
      if (map_part_top)
        return map_part_top->IsPartOf(numbers);
      break;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      for (part=begin(); part!=end(); part++)
        if (!part->IsPartOf(numbers, type)) return false;
      return true;

    default:
      break;
  }
  return false;
}

void Building::DeleteData()
{
  Building::iterator part;
  
  // Delete al building parts
  for (part=begin(); part!=end(); part++)
    part->DeleteData();
  erase(begin(), end());
  
  // Delete topology data
  if (map_top)
    if (map_top->size()) map_top->Erase();
  if (map_part_top)
    if (map_part_top->size()) map_part_top->Erase(); 
}

bool Building::DeleteData(const DataType type)
{
  Building::iterator part;

  // Delete data of a specific type
  switch (type) {

    case MapData:
      if (map_top)
        if (map_top->size()) {
          map_top->Erase();
          return true;
        }
      break;

    case MapPartitionData:
      if (map_part_top)
        if (map_part_top->size()) {
          map_part_top->Erase();
          return true;
        }
      break;

    case ModelData:
      if (size()) {
        for (part=begin(); part!=end(); part++) part->DeleteData();
        erase(begin(), end());
        return true;
      }
      break;

    default:
      break;
  }
  return false;
}

bool Building::DeleteData(const DataType type, PointNumberList &deleted_points)
{
  Building::iterator             part;
  LineTopologies::const_iterator top;

  // Delete data of a specific type
  switch (type) {

    case MapData:
      if (map_top)
        if (map_top->size()) {
          for (top=map_top->begin(); top!=map_top->end(); top++)
            deleted_points.Add(top->PointNumberListReference());
          map_top->Erase();
          return true;
        }
      break;

    case MapPartitionData:
      if (map_part_top)
        if (map_part_top->size()) {
          for (top=map_part_top->begin(); top!=map_part_top->end(); top++)
            deleted_points.Add(top->PointNumberListReference());
          map_part_top->Erase();
          return true;
        }
      break;

    case ModelData:
      if (size()) {
        for (part=begin(); part!=end(); part++) {
          part->DeleteData(type, deleted_points);
          part->DeleteData(); // Also delete the rest (map partition)
        }
        erase(begin(), end());
        return true;
      }
      break;

    default:
      break;
  }
  return false;
}

bool Building::SplitMapPartition(LineTopologies::iterator old_partition,
                                 const LineTopologies &new_partitions)
{
  LineTopologies::iterator partition;
  bool                     deleted_partition;

  // Delete the old partition
  for (partition=map_part_top->begin(), deleted_partition=false;
       partition!=map_part_top->end() && !deleted_partition; partition++) {
    if (partition == old_partition) {
      map_part_top->erase(partition);
      deleted_partition = true;
    }
  }
  if (!deleted_partition) {
    printf("Error: partition to be split not found in building\n");
    return false;
  }

  // Add the new partitions
  AddMapPartitionData(new_partitions[0]);
  AddMapPartitionData(new_partitions[1]);
  return true;
}

bool Building::MergePartitions(LineTopologies::iterator partition1,
                               LineTopologies::iterator partition2)
{
  LineTopology merged_partition;

  // Check if both partitions are closed polygons
  if (!partition1->IsClosed() || !partition2->IsClosed()) {
    printf("Merging of open partitions is not defined.\n");
    return false;
  }
  // Insert common nodes
  partition1->InsertNodes(map_points->ObjectPointsRef(),
                          partition2->LineTopologyReference(), 0.01);
  partition2->InsertNodes(map_points->ObjectPointsRef(),
                          partition1->LineTopologyReference(), 0.01);
  // Merge the partitions
  if (!partition1->Merge(partition2->LineTopologyReference(), merged_partition))
    return false;
  // Replace the nodes of the first partition by those of the merged one.
  partition1->erase(partition1->begin(), partition1->end());
  partition1->insert(partition1->begin(), merged_partition.begin(),
                     merged_partition.end());
  // Delete the second partition
  map_part_top->erase(partition2);
  return true;
}

int Building::Partition(const DataType type,
                        LineTopsIterVector::iterator partition_line)
{
  LineTopologies::iterator                    outline;
  ObjectPoints2D                              map_points_2D;
  ObjectPoint                                 *map_point, new_point;
  LineTopology::iterator                      node;
  House                                       house;
  std::vector<ObjectPoints2D>                 rect_list;
  std::vector<ObjectPoints2D>::const_iterator rect;
  ObjectPoints2D::const_iterator              rect_point;
  PointNumber                                 next_point_number;
  DataBounds2D                                bounds;
  ObjectPoints                                new_points;
  Vector2D                                    shift;

  // Several data consistence checks
  if (type == SelectedMapData) {
    // Check if there is a building outline
    if (map_top == NULL) return 1;
    if (map_top->size() == 0) return 1;
    outline = map_top->begin();
  }
  else { // SelectedMapPartitionData
    if (map_part_top == NULL) return 1;
    if (map_part_top->size() == 0) return 1;
    outline = *partition_line;
    if (outline < map_part_top->begin() || outline >= map_part_top->end())
      return 1;
  }
  // Check if the outline is closed
  if (!outline->IsClosed()) return 2;
  // Check if the line has at least five corners
  if (outline->size() <= 6) return 3;

  // Make a list of 2D map points
  for (node=outline->begin(); node!=outline->end(); node++) {
    map_point = map_points->GetPoint(*node);
    if (map_point == NULL) return 4;
    map_points_2D.push_back(ObjectPoint2D(*map_point));
  }

  // Create initial ground plan
  house = House(outline->Number(), *outline, map_points_2D);
  bounds = outline->Bounds(map_points_2D); // Determine coordinate centre
  shift = (bounds.Minimum() + bounds.Maximum()) / 2.0;
  house.ShiftHouse(shift.X(), shift.Y());
  house.DeleteCollinearPoints();

  // Divide into rectangles
  next_point_number = PointNumber((map_points->end()-1)->Number()+1);
  house.DivideHousePlan(rect_list, next_point_number);

  // Check if there are at least two partitions
  if (rect_list.size() <= 2) return 5;

  // Clear old partition data if the whole building is partitioned
  if (map_part_top && type == SelectedMapData) {
    if (!map_part_top->empty())
      map_part_top->erase(map_part_top->begin(), map_part_top->end());
  }
  // Erase the selected partition that is now partitioned further
  else if (type == SelectedMapPartitionData) map_part_top->erase(outline);

  // Retrieve the original next number (now overwritten by DivideHousePlan)
  next_point_number = PointNumber((map_points->end()-1)->Number()+1);
  // Store object points and topology of partitioned building
  for (rect=rect_list.begin(); rect!=rect_list.end(); rect++) {
    LineTopology partition(rect - rect_list.begin());
    for (rect_point=rect->begin(); rect_point!=rect->end(); rect_point++) {
      // Add point number to new partition topology
      partition.push_back(rect_point->Number());
      // Check if the point needs to be added to the new point list
      if (rect_point->Number() >= next_point_number.Number())
        if (new_points.GetPoint(rect_point->NumberRef()) == NULL) {
          new_point = ObjectPoint(&*rect_point, 0.0);
          new_point.X() += shift.X();
          new_point.Y() += shift.Y();
          new_points.push_back(new_point);
        }
    }
    partition.push_back(*partition.begin()); // Close the partition
    partition.Attribute(BuildingNumberTag) = Number(); // Inherit building number
    // Add the partition to the building structure
    if (map_part_top == NULL) map_part_top = new LineTopologies();
    map_part_top->push_back(partition);
  }
  // Add the new points to the collection of map points
  if (!new_points.empty()) {
    new_points.Sort();
    map_points->insert(map_points->end(), new_points.begin(), new_points.end());
  }
  
  return 0;
}

int Building::NextPartNumber() const
{
  Building::const_iterator part;
  int                      max_number = 0;

  for (part=begin(); part!=end(); part++)
    if (part->Number() > max_number) max_number = part->Number();
  return max_number + 1;
}

Building::iterator Building::RetrievePart(const LineTopology &part_outline)
{
  Building::iterator  part;
  LineTopologies      *map_partitions;

  // Try to find a part with the same topology
  for (part=begin(); part!=end(); part++) {
    map_partitions = part->MapPartitionTopology();
    if (map_partitions)
      if (map_partitions->size() == 1 &&
          map_partitions->Contains(part_outline))
        return part;
  }

  // If there is no such part, create a new part for the given outline
  AddMapPartitionData(NextPartNumber(), part_outline);
  part = end() - 1;
  part->SetLineNumbers(Number());
  return part;
}

Building::iterator Building::RetrievePart(const LineTopologies &part_outlines)
{
  Building::iterator       part;
  LineTopologies           *map_partitions;
  LineTopologies::iterator map_partition;
  bool                     found;

  // Try to find a part with the same topologies
  for (part=begin(); part!=end(); part++) {
    map_partitions = part->MapPartitionTopology();
    if (map_partitions) {
      if (map_partitions->size() == part_outlines.size()) {
        for (map_partition=map_partitions->begin(), found=true;
             map_partition!=map_partitions->end() && found; map_partition++)
          if (!part_outlines.Contains(*map_partition)) found = false;
        if (found) return part;
      }
    }
  }

  // If there is no such part, create a new part for the given outline
  AddMapPartitionData(NextPartNumber(), part_outlines);
  part = end() - 1;
  part->SetLineNumbers(Number());
  return part;
}

// Print all polygons of a building
void Building::Print() const
{
  printf("Building number %d\n", num);
  Print(MapData);
  Print(MapPartitionData);
  Print(ModelData);
}

void Building::Print(const DataType type) const
{
  LineTopologies::const_iterator top;
  Building::const_iterator part;
  
  if (type == MapData) {
    if (map_top) {
      printf("Map data\n");
      for (top=map_top->begin(); top!=map_top->end(); top++)
        top->Print();
    }
    else printf("No map data\n");
  }
  if (type == MapPartitionData) {
    if (map_part_top) {
      printf("Map partition data\n");
      for (top=map_part_top->begin(); top!=map_part_top->end(); top++)
        top->Print();
    }
    else printf("No map partition data\n");
  }
  if (type == ModelData) {
    for (part=begin(); part!=end(); part++) part->Print();
    if (empty()) printf("No model parts\n");
  }
}
