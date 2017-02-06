
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



#include "BuildingPart.h"

BuildingPart & BuildingPart::operator = (const BuildingPart &part)
{
  if (this == &part) return *this;  // Check for self assignment
  num             = part.num;
  map_points      = part.map_points; // Only copy pointer!
  model_points    = part.model_points; // Only copy pointer!
  roof_type       = part.roof_type;
  roof_parameters = part.roof_parameters;
  if (part.map_part_top) map_part_top = new LineTopologies(*part.map_part_top);
  else map_part_top = NULL;
  if (part.roof) roof = new LineTopologies(*part.roof);
  else roof = NULL;
  if (part.walls) walls = new LineTopologies(*part.walls);
  else walls = NULL;
  return *this;
}

bool BuildingPart::ContainsData(DataType type) const
{
  switch (type) {
    case MapData:
    case SelectedMapData:
      return false;

    case MapPartitionData: 
    case SelectedMapPartitionData:
      if (map_part_top)
        if (map_part_top->size()) return true;
      return false;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
    case LastModelData:
    case SelectedModelFaceData:
      if (roof)
        if (roof->size()) return true;
      if (walls)
        if (walls->size()) return true;
      return false;

    case LaserData:
      return false;

    default:
      printf("Unknown data type %d in BuildingPart::ContainsData(DataType)\n",
             type);
  }
  return false;
}

bool BuildingPart::ContainsData() const
{
  if (ContainsData(ModelData)) return true;
  if (ContainsData(MapPartitionData)) return true;
  return false;
}

void BuildingPart::SelectPolygons(LineTopsIterVector &selection,
                                  const PointNumber &number, DataType type) const
{
  LineTopologies::iterator  top;

  if (type != LastModelData && type != SelectedModelPartData && 
      type != SelectedModelFaceData) return;
  if (Contains(number, type)) {
    if (roof)
      for (top=roof->begin(); top!=roof->end(); top++)
        if (type == SelectedModelPartData || top->Contains(number))
          selection.push_back(top);
    if (walls)
      for (top=walls->begin(); top!=walls->end(); top++)
        if (type == SelectedModelPartData || top->Contains(number))
          selection.push_back(top);
  }
}


void BuildingPart::AddMapPartitionData(const LineTopology &new_map_part_top,
                                       bool clear_first)
{
  if (clear_first) {
    if (map_part_top) delete map_part_top;
    map_part_top = NULL;
  }
  if (map_part_top == NULL) map_part_top = new LineTopologies();
  map_part_top->push_back(new_map_part_top);
  (map_part_top->end()-1)->Label() = MapPartitionLabel;
}

void BuildingPart::AddMapPartitionData(const LineTopologies &new_map_part_tops,
                                       bool clear_first)
{
  LineTopologies::iterator top;
  if (clear_first) {
    if (map_part_top) delete map_part_top;
    map_part_top = NULL;
  }
  if (map_part_top == NULL) map_part_top = new LineTopologies();
  map_part_top->insert(map_part_top->end(), new_map_part_tops.begin(),
                       new_map_part_tops.end());
  for (top=map_part_top->end()-new_map_part_tops.size();
       top!=map_part_top->end(); top++)
    top->Label() = MapPartitionLabel;
}

void BuildingPart::AddModelData(const LineTopologies &new_model_tops,
                                ObjectPoints *new_model_points)
{
  LineTopologies::const_iterator new_model_top;

  if (new_model_points) model_points = new_model_points;
  for (new_model_top=new_model_tops.begin();
       new_model_top!=new_model_tops.end(); new_model_top++)
    AddModelData(new_model_top->LineTopologyReference());
}

void BuildingPart::AddModelData(const LineTopology &new_model_top,
                                ObjectPoints *new_model_points)
{
  if (new_model_points) model_points = new_model_points;
  switch (new_model_top.Label()) {
    case RoofLabel:
      if (!roof) roof = new LineTopologies();
      roof->push_back(new_model_top);
      break;

    case WallLabel:
      if (!walls) walls = new LineTopologies();
      walls->push_back(new_model_top);
      break;

    default:
      printf("Unknown label %d in BuildingPart::AddModelData\n",
             new_model_top.Label());
  }
}

void BuildingPart::AddModelData(ObjectPoints *new_model_points)
{
  model_points = new_model_points;
}

void BuildingPart::SetLineNumbers(int building_number)
{
  LineTopologies::iterator polygon;
  
  if (map_part_top) 
    for (polygon=map_part_top->begin(); polygon!=map_part_top->end(); polygon++) {
      polygon->Attribute(BuildingNumberTag) = building_number;
      polygon->Attribute(BuildingPartNumberTag) = Number();
    }
  if (roof)
    for (polygon=roof->begin(); polygon!=roof->end(); polygon++) {
      polygon->Attribute(BuildingNumberTag) = building_number;
      polygon->Attribute(BuildingPartNumberTag) = Number();
    }
  if (walls)
    for (polygon=walls->begin(); polygon!=walls->end(); polygon++) {
      polygon->Attribute(BuildingNumberTag) = building_number;
      polygon->Attribute(BuildingPartNumberTag) = Number();
    }
}

void BuildingPart::CollectMapData(LineTopologies &map_data) const
{
  if (map_part_top) 
    map_data.insert(map_data.end(), map_part_top->begin(),
                    map_part_top->end());
}

void BuildingPart::CollectModelData(LineTopologies &model_data) const
{
  if (roof)
    model_data.insert(model_data.end(), roof->begin(), roof->end());
  if (walls)
    model_data.insert(model_data.end(), walls->begin(), walls->end());
}

void BuildingPart::CollectModelPoints(ObjectPoints &points) const
{
  LineTopologies::const_iterator polygon;
  LineTopology::const_iterator   node;
  ObjectPoints::const_iterator   point;

  if (roof) {
    for (polygon=roof->begin(); polygon!=roof->end(); polygon++) {
      for (node=polygon->begin(); node!=polygon->end(); node++) {
        if (points.GetPoint(*node) == NULL) { // We don't have this point yet
          point = model_points->ConstPointIterator(*node); // Get it
          points.push_back(*point); // Copy it
        }
      }
    }
  }
  if (walls) {
    for (polygon=walls->begin(); polygon!=walls->end(); polygon++) {
      for (node=polygon->begin(); node!=polygon->end(); node++) {
        if (points.GetPoint(*node) == NULL) { // We don't have this point yet
          point = model_points->ConstPointIterator(*node); // Get it
          points.push_back(*point); // Copy it
        }
      }
    }
  }
}

bool BuildingPart::Contains(const PointNumber &number,
                            const DataType type) const
{
  LineTopologies::const_iterator polygon;

  switch (type) {
    case MapPartitionData:
    case SelectedMapPartitionData:
      if (map_part_top)
        for (polygon=map_part_top->begin(); polygon!=map_part_top->end();
             polygon++)
          if (polygon->Contains(number)) return true;
      break;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
    case LastModelData:
    case SelectedModelFaceData:
      if (roof)
        for (polygon=roof->begin(); polygon!=roof->end(); polygon++)
          if (polygon->Contains(number)) return true;
      if (walls)
        for (polygon=walls->begin(); polygon!=walls->end(); polygon++)
          if (polygon->Contains(number)) return true;
      break;

    default:
      break;
  }
  return false;
}

bool BuildingPart::Contains(const LineTopology &ref_polygon,
                            const DataType type) const
{
  LineTopologies::const_iterator polygon;

  switch (type) {
    case MapPartitionData:
    case SelectedMapPartitionData:
      if (map_part_top)
        for (polygon=map_part_top->begin(); polygon!=map_part_top->end();
             polygon++)
          if (ref_polygon == *polygon) return true;
      break;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
    case SelectedModelFaceData:
      if (roof)
        for (polygon=roof->begin(); polygon!=roof->end(); polygon++)
          if (ref_polygon == *polygon) return true;
      if (walls)
        for (polygon=walls->begin(); polygon!=walls->end(); polygon++)
          if (ref_polygon == *polygon) return true;
      break;

    default:
      break;
  }
  return false;
}

bool BuildingPart::IsPartOf(const PointNumberList &numbers,
                            const DataType type) const
{
  switch (type) {
    case MapPartitionData:
    case SelectedMapPartitionData:
      if (map_part_top) return map_part_top->IsPartOf(numbers);
      else return false;

    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      if (roof)
        if (roof->IsPartOf(numbers)) return true;
      if (walls)
        if (walls->IsPartOf(numbers)) return true;
      return false;

    default:
      break;
  }
  return false;
}

void PrintRoof(const LineTopologies *roof, const LineTopologies *walls)
{
  LineTopologies::const_iterator face;
  LineTopology::const_iterator   node;

  if (roof) {
    for (face=roof->begin(); face!=roof->end(); face++) {
      printf("Num %5d Label %2d Pts: ", face->Number(), face->Label());
      for (node=face->begin(); node!=face->end(); node++)
        printf("%5d", node->Number());
      printf("\n");
    }
  }
  if (walls) {
    for (face=walls->begin(); face!=walls->end(); face++) {
      printf("Num %5d Label %2d Pts: ", face->Number(), face->Label());
      for (node=face->begin(); node!=face->end(); node++)
        printf("%5d", node->Number());
      printf("\n");
    }
  }
}

bool BuildingPart::DeletePolygons(const LineTopologies &polygons,
                                  const DataType type)
{
  bool                     deleted_polygon = false;
  LineTopologies::iterator polygon;

  switch (type) {
    case ModelData:
    case LastModelData:
      if (roof) {
        for (polygon=roof->begin(); polygon!=roof->end(); polygon++) {
          if (polygons.Contains(*polygon)) {
            roof->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (roof->empty()) roof = NULL;
      }
      if (walls) {
        for (polygon=walls->begin(); polygon!=walls->end(); polygon++) {
          if (polygons.Contains(*polygon)) {
            walls->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (walls->empty()) walls = NULL;
      }
      break;

    case MapPartitionData:
      if (map_part_top) {
        for (polygon=map_part_top->begin(); polygon!=map_part_top->end();
             polygon++) {
          if (polygons.Contains(*polygon)) {
            map_part_top->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (map_part_top->empty()) map_part_top = NULL;
      }
      break;

    default:
      break;
  }
  return deleted_polygon;
}

bool BuildingPart::CropPolygons(const LineTopologies &polygons,
                                const DataType type)
{
  bool                     deleted_polygon = false;
  LineTopologies::iterator polygon;

  switch (type) {
    case ModelData:
      if (roof) {
        for (polygon=roof->begin(); polygon!=roof->end(); polygon++) {
          if (!polygons.Contains(*polygon)) {
            roof->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (roof->empty()) roof = NULL;
      }
      if (walls) {
        for (polygon=walls->begin(); polygon!=walls->end(); polygon++) {
          if (!polygons.Contains(*polygon)) {
            walls->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (walls->empty()) walls = NULL;
      }
      break;

    case MapPartitionData:
      if (map_part_top) {
        for (polygon=map_part_top->begin(); polygon!=map_part_top->end();
             polygon++) {
          if (!polygons.Contains(*polygon)) {
            map_part_top->erase(polygon);
            polygon--;
            deleted_polygon = true;
          }
        }
        if (map_part_top->empty()) map_part_top = NULL;
      }
      break;

    default:
      break;
  }
  return deleted_polygon;
}

void BuildingPart::DeleteData()
{
  // Delete map partition line
  if (map_part_top) map_part_top->Erase();
  // Delete roof faces
  if (roof) roof->Erase();
  // Delete walls
  if (walls) walls->Erase();
}

void BuildingPart::DeleteData(const DataType type)
{
  switch (type) {
    case MapPartitionData:
      if (map_part_top) map_part_top->Erase();
      break;
    case ModelData:
      if (roof) roof->Erase();
      if (walls) walls->Erase();
      break;
    default:
      break;
  }
}

void BuildingPart::DeleteData(const DataType type,
                              PointNumberList &deleted_points)
{
  LineTopologies::const_iterator pol;

  switch (type) {
    case MapPartitionData:
      if (map_part_top) {
        for (pol=map_part_top->begin(); pol!=map_part_top->end(); pol++)
          deleted_points.Add(pol->PointNumberListReference());
        map_part_top->Erase();
      }
      break;
    case ModelData:
      if (roof) {
        for (pol=roof->begin(); pol!=roof->end(); pol++)
          deleted_points.Add(pol->PointNumberListReference());
        roof->Erase();
      }
      if (walls) {
        for (pol=walls->begin(); pol!=walls->end(); pol++)
          deleted_points.Add(pol->PointNumberListReference());
        walls->Erase();
      }
      break;
    default:
      break;
  }
}

void BuildingPart::DeriveRoofPlanes()
{
  LineTopologies::const_iterator face;
  Plane                          plane;
  LineTopology::const_iterator   node;
  ObjectPoints::const_iterator   point;

  // Checking data
  if (!roof) return;
  if (!model_points) {
    printf("Error: no model points to compute roof planes\n");
    return;
  }

  // Initialising roof planes
  if (!roof_planes) roof_planes = new Planes();
  if (!roof_planes->empty())
    roof_planes->erase(roof_planes->begin(), roof_planes->end());

  // Loop over all roof faces
  for (face=roof->begin(); face!=roof->end(); face++) {
    // Loop over all corners of a roof face
    for (node=face->begin(); node!=face->end()-1; node++) {
      point = model_points->ConstPointIterator(node->NumberRef());
      plane.AddPoint(point->Position3DRef(), false);
    }
    plane.Recalculate();
    roof_planes->push_back(plane);
    plane.Initialise();
  }
}
 
double BuildingPart::RoofHeight(const Position2D &pos) const
{
  double phi, d, u, rh, sl, v, smid, rl, rw, sl2, xc, yc, rad, r2, slx, sly;
  int                            success;
  LineTopologies::const_iterator roof_face;
  ObjectPoint2D                  test_point;
  LineTopology::const_iterator   node;
  Plane                          plane;
  ObjectPoints::const_iterator   roof_point;

// First deal with some specific roof types

  switch (roof_type) {
    case UnknownRoof:
      if (roof == NULL) return 0.0;
      // See if there is a roof face at this position
      for (roof_face=roof->begin(); roof_face!=roof->end(); roof_face++) {
        test_point.vect() = pos.vect();
        if (test_point.InsidePolygon(*model_points, *roof_face)) {
          // Fit plane to points of roof face
          for (node=roof_face->begin(); node!=roof_face->end(); node++) {
            roof_point = model_points->ConstPointIterator(*node);
            plane.AddPoint(roof_point->Position3DRef(), false);
          }
          plane.Recalculate();
          // Return plane height at requested position
          return plane.Z_At(pos.X(), pos.Y(), &success);
        }
      }
      // Point outside all roof faces
      return 0.0;
    case FlatRoof:
      return roof_parameters.Val(0);

    case ShedRoof:
      rh = roof_parameters.Val(0);
      slx = roof_parameters.Val(1);
      sly = roof_parameters.Val(2);
      return(rh + slx * pos.X() + sly * pos.Y());

    case SphereRoof:
      rh  = roof_parameters.Val(0);     // Top height
      xc  = roof_parameters.Val(1);     // X-coordinate of centre
      yc  = roof_parameters.Val(2);     // Y-coordinate of centre
      rad = roof_parameters.Val(3);     // Radius
      r2  = (pos.X() - xc) * (pos.X() - xc) + (pos.Y() - yc) * (pos.Y() - yc);
      r2  = rad * rad - r2;
      if (r2 > 0) return(rh - rad + sqrt(r2));
      else return(rh - rad);

    case ConeRoof:
      rh  = roof_parameters.Val(0);     // Top height
      xc  = roof_parameters.Val(1);     // X-coordinate of centre
      yc  = roof_parameters.Val(2);     // Y-coordinate of centre
      sl  = roof_parameters.Val(3);     // Slope
      r2  = (pos.X() - xc) * (pos.X() - xc) + (pos.Y() - yc) * (pos.Y() - yc);
      if (r2 > 0.0) return(rh - sl * sqrt(r2));  
      else return(rh);
        
    default:
      break;
  }

// Roof parameters common to other roof types

  phi = roof_parameters.Val(0);     // Ridge normal orientation
  d   = roof_parameters.Val(1);     // Ridge distance to origin
  rh  = roof_parameters.Val(2);     // Ridge height
  sl  = roof_parameters.Val(3);     // Slope

// Model dependent parameters

  switch (roof_type) {
    case HipRoof2:    // Hip roof with two different slopes
    case HipRoof2Rot: // The same hip roof rotated by 90 degrees
      sl2 = roof_parameters.Val(6);
    case HipRoof:    // Hip roof
    case HipRoofRot: // The same hip roof rotated by 90 degrees
      smid  = roof_parameters.Val(4);  // Scalar of ridge centre
      rl    = roof_parameters.Val(5);  // Ridge length
      break;
    case GambrelRoof:    // Gambrel roof
    case GambrelRoofRot: // Gambrel roof rotated by 90 degrees
      rw = roof_parameters.Val(4);  // Ridge width
      break;
    case CylinderRoof:  // Cylinder roof
    case CylinderRoofRot: // Cylinder roof rotated by 90 degrees
      rad = roof_parameters.Val(3);
      break;
    default:
      break;
  }

// Distance of point to roof ridge

  u   = pos.X() * cos(phi) + pos.Y() * sin(phi) - d;

  switch (roof_type) {
    case GableRoof: // Standard gable roof
    case GableRoofRot: // Standard gable roof 90 degree rotated
      return(rh - fabs(u) * sl);
    case HipRoof: // Hip roof
    case HipRoofRot: // Hip roof rotated by 90 degrees
      sl2 = sl;
    case HipRoof2: // Hip roof with two different slopes
    case HipRoof2Rot: // The same hip roof rotated by 90 degrees
      v = -pos.X() * sin(phi) + pos.Y() * cos(phi) - smid;
      if (fabs(v) >= rl / 2.0 + fabs(u))
        return(rh - (fabs(v) - rl / 2.0) * sl2);
      else return(rh - fabs(u) * sl);
    case GambrelRoof: // u roof
    case GambrelRoofRot: // u roof rotated by 90 degrees
      if (fabs(u) < rw / 2.0) return(rh);
      else return(rh - (fabs(u) - rw / 2.0) * sl);
    case CylinderRoof:  // Cylinder roof
    case CylinderRoofRot: // Cylinder roof rotated by 90 degrees
      if (fabs(u) > fabs(rad)) return(rh - rad); // Location outside cylinder radius
      return(rh - rad + sqrt(rad*rad - u*u));
    default:
      break;
  }

  return 0.0; // To avoid compiler warnings, we should not get here though
}

double BuildingPart::PartialRoofDerivative(const Position2D &pos,
                                           int parameter) const
{
  double phi, d, u, rh, sl, usign, vsign, v, rw, smid, rl, deriv, sl2,
         xc, yc, rad, r2, slx, sly, q;

  if (roof_type == UnknownRoof) return 0.0;

// Roof parameters common to most roof types

  if (roof_type != FlatRoof && roof_type != ShedRoof &&
      roof_type != SphereRoof && roof_type != ConeRoof) {
    phi = roof_parameters.Val(0);     // Ridge normal orientation
    d   = roof_parameters.Val(1);     // Ridge distance to origin
    u = pos.X() * cos(phi) + pos.Y() * sin(phi) - d; // Distance to roof ridge
    usign = (u > 0) ? 1 : -1;
  }

  switch (roof_type) {
    case GambrelRoof: // u roof
    case GambrelRoofRot: // u roof rotated by 90 degrees
      rw = roof_parameters.Val(4);
    case GableRoof: // Standard gable roof
    case GableRoofRot: // Standard gable roof 90 degree rotated
    case HipRoof: // Hip roof
    case HipRoofRot: // Hip roof rotated by 90 degrees
    case HipRoof2: // Hip roof with two different slopes
    case HipRoof2Rot: // The same hip roof rotated by 90 degrees
      rh = roof_parameters.Val(2);     // Ridge height
      sl = roof_parameters.Val(3);     // Slope
      break;
    case CylinderRoof:
    case CylinderRoofRot:
      rh = roof_parameters.Val(2);
      rad = roof_parameters.Val(3);
      break;
    default:
      break;
  }

// Model specific derivatives

  switch (roof_type) {
    case FlatRoof: // Flat roof
      switch (parameter) {
        case 0: // Roof height
          return(1.0);
      }
    case GableRoof: // Standard gable roof
    case GableRoofRot: // Standard gable roof 90 degree rotated
      switch (parameter) {
        case 0: // Ridge normal orientation
          if (u >= 0) return(sl * (pos.X() * sin(phi) - pos.Y() * cos(phi)));
          else return(sl * (-pos.X() * sin(phi) + pos.Y() * cos(phi)));
        case 1: // Ridge distance to origin
          return(usign * sl);
        case 2: // Ridge height
          return(1.0);
        case 3: // Slope
          return(-1.0 * fabs(u));
      }
      break;

    case HipRoof: // Hip roof
    case HipRoofRot: // Hip roof rotated by 90 degrees
    case HipRoof2: // Hip roof with two different slopes
    case HipRoof2Rot: // The same hip roof rotated by 90 degrees
      smid  = roof_parameters.Val(4);  // Scalar of ridge centre
      rl    = roof_parameters.Val(5);  // Ridge length
      if (roof_type == HipRoof2 || roof_type == HipRoof2Rot)
        sl2 = roof_parameters.Val(6);
      else sl2 = sl;
      v = -pos.X() * sin(phi) + pos.Y() * cos(phi) - smid;
      vsign = (v > 0) ? 1 : -1;
      switch (parameter) {
        case 0: // Ridge normal orientation
          if (fabs(v) < rl / 2.0 + fabs(u)) {
            deriv = sl * (pos.X() * sin(phi) - pos.Y() * cos(phi));
            return(usign * deriv);
          }
          else {
            deriv = sl2 * (pos.X() * cos(phi) + pos.Y() * sin(phi));
            return(vsign * deriv);
          }
        case 1: // Ridge distance to origin
          if (fabs(v) < rl / 2.0 + fabs(u)) return(usign * sl);
          else return(0.0);
        case 2: // Ridge height
          return(1.0);
        case 3: // Slope
          if (roof_type == HipRoof || roof_type == HipRoofRot) {
            if (fabs(v) < rl / 2.0 + fabs(u)) return(-1.0 * fabs(u));
            else return(-1.0 * (fabs(v) - rl / 2.0));
          }
          else { // The first of two different slopes
            if (fabs(v) < rl / 2.0 + fabs(u)) return(-1.0 * fabs(u));
            else return(0.0);
          }
        case 4: // Scalar of ridge centre
          if (fabs(v) < rl / 2.0 + fabs(u)) return(0.0);
          else return(vsign * sl2);
        case 5: // Ridge length
          if (fabs(v) < rl / 2.0 + fabs(u)) return(0.0);
          else return(sl2 / 2.0);
        case 6: // The second of two different slopes
          if (fabs(v) < rl / 2.0 + fabs(u)) return(0.0);
          else return(-1.0 * (fabs(v) - rl / 2.0));
      }
      break;

    case GambrelRoof: // u roof
    case GambrelRoofRot: // u roof rotated by 90 degrees
      switch (parameter) {
        case 0: // Ridge normal orientation
          if (u < rw / 2.0) return(0.0);
          else return(usign * sl * (pos.X() * sin(phi) - pos.Y() * cos(phi)));
        case 1: // Ridge distance to origin
          if (u < rw / 2.0) return(0.0);
          else return(usign * sl);
        case 2: // Ridge height
          return(1.0);
        case 3: // Slope
          if (u < rw / 2.0) return(0.0);
          else return(-1.0 * (fabs(u) - rw / 2.0));
        case 4: // Ridge width
          if (u < rw / 2.0) return(0.0);
          else return(sl / 2.0);
      }
      break;

    case SphereRoof: // Sphere
      rh  = roof_parameters.Val(0);     // Top height
      xc  = roof_parameters.Val(1);     // X-coordinate of centre
      yc  = roof_parameters.Val(2);     // Y-coordinate of centre
      rad = roof_parameters.Val(3);     // Radius
      r2  = (pos.X() - xc) * (pos.X() - xc) + (pos.Y() - yc) * (pos.Y() - yc);
      r2  = rad * rad - r2;
      if (r2 > 0) r2 = sqrt(r2); else return(0.0);
      switch (parameter) {
        case 0: // Top height
          return(1.0);
        case 1: // X-coordinate of centre
          return((pos.X() - xc) / r2);
        case 2: // Y-coordinate of centre
          return((pos.Y() - yc) / r2);
        case 3: // Radius
          return(-1.0 + rad / r2);
      }
      break;

    case ConeRoof: // Cone
      rh  = roof_parameters.Val(0);     // Top height
      xc  = roof_parameters.Val(1);     // X-coordinate of centre
      yc  = roof_parameters.Val(2);     // Y-coordinate of centre
      sl  = roof_parameters.Val(3);     // Slope
      r2  = (pos.X() - xc) * (pos.X() - xc) + (pos.Y() - yc) * (pos.Y() - yc);
      if (r2 > 0) r2 = sqrt(r2);
      switch (parameter) {
        case 0: // Top height
          return(1.0);
        case 1: // X-coordinate of centre
          return(sl * (pos.X() - xc) / r2);
        case 2: // Y-coordinate of centre
          return(sl * (pos.Y() - yc) / r2);
        case 3: // Radius
          return(-r2);
      }
      break;

    case ShedRoof: // Shed roof
      rh  = roof_parameters.Val(0);
      slx = roof_parameters.Val(1);
      sly = roof_parameters.Val(2);
      switch (parameter) {
        case 0: // Centre height
          return(1.0);
        case 1: // X-slope
          return(pos.X());
        case 2: // Y-slope
          return(pos.Y());
      }
      break;

    case CylinderRoof: // Standard cylinder roof
    case CylinderRoofRot: // Standard cylinder roof 90 degree rotated
      q = rad * rad - u * u;
      if (q > 0.0) q = sqrt(q);
      else return(0.0); // Point outside cylinder radius
      switch (parameter) {
        case 0: // Ridge normal orientation
          return(u * (pos.X() * sin(phi) - pos.Y() * cos(phi)) / q);
        case 1: // Ridge distance to origin
          return(u / q);
        case 2: // Ridge height
          return(1.0);
        case 3: // Radius
          return(-1 + rad / q);
      }
      break;

    default:
      break;
  }
  return 0.0; // To avoid compiler warnings, we do not get here though
}

int BuildingPart::NumberOfRoofParameters() const
{
  switch (roof_type) {
    case UnknownRoof:     return 0;
    case FlatRoof:        return 1;
    case GableRoof:
    case GableRoofRot:    return 4;
    case HipRoof:
    case HipRoofRot:      return 6;
    case GambrelRoof:
    case GambrelRoofRot:  return 5;
    case HipRoof2:
    case HipRoof2Rot:     return 7;
    case SphereRoof:
    case ConeRoof:        return 4;
    case ShedRoof:        return 3;
    case CylinderRoof:
    case CylinderRoofRot: return 4;
    default: printf("Invalid roof type %d\n", roof_type); exit(0);
  }
  return 0;
}

void BuildingPart::SetRoofType(RoofType type)
{
  roof_type = type;
  roof_parameters = vMatrix(NumberOfRoofParameters(), 1);
}

void BuildingPart::ApproximateRoofValues(const LineSegments2D &map_segments,
                                         const LaserPoints &original_points)
{
  LineSegments2D::const_iterator wall, longest_wall;
  double                         max_len, d_min, d_max, d, u, v, umax, vmax,
                                 phi, vmin, dist, max_dist;
  Line2D                         ridge;
  Position2D                     corner;
  DataBounds2D                   bounds;
  vector <double>                heights;
  LaserPoints::iterator          point, point0, point1, point2;
  LaserPoints::const_iterator    original_point;
  int                            index, index1, index2;
  SegmentationParameters         nbh_parameters;
  TINEdges                       *edges;
  Position3D                     centre;
  Line3D                         line1, line2;
  Vector3D                       normal, hor_tangent, steepest_descent;
  Plane                          plane1, plane2;
  LaserPoints                    points;

  int Compare_Double(const void *, const void *);

  // Sort the heights for most roof types
  for (original_point=original_points.begin();
       original_point!=original_points.end(); original_point++)
    heights.push_back(original_point->Z());
  qsort((void *) &*heights.begin(), heights.size(), sizeof(double),
        Compare_Double);

  switch (roof_type) {
    case ShedRoof: // Shed roof
      roof_parameters.Val(1) = 0.0; // No X-slope
      roof_parameters.Val(2) = 0.0; // No Y-slope
    case FlatRoof: // Flat roof
      // Roof height
      roof_parameters.Val(0) = heights[heights.size()/2]; // Median height
      break;

    case GableRoof:       // Standard gable roof
    case GableRoofRot:    // Standard gable roof 90 degree rotated
    case HipRoof:         // Hip roof
    case HipRoofRot:      // Hip roof rotated by 90 degrees
    case GambrelRoof:     // u roof
    case GambrelRoofRot:  // u roof rotated by 90 degree
    case HipRoof2:        // Hip roof with two different slopes
    case HipRoof2Rot:     // The same hip roof rotated by 90 degrees
    case CylinderRoof:    // Cylinder roof
    case CylinderRoofRot: // Cylinder roof rotated by 90 degrees
      // Get normal orientation of longest edge
      max_len = 0;
      for (wall=map_segments.begin(); wall!=map_segments.end(); wall++) {
        if (wall->Length() > max_len) {
          max_len = wall->Length();
          longest_wall = wall;
        }
      }
      // Use this wall for the initial ridge orientation
      ridge = longest_wall->Line2DReference();
      // For the rotated roof types, construct a perpendicular line
      if (roof_type == GableRoofRot   || roof_type == HipRoofRot ||
          roof_type == GambrelRoofRot || roof_type == HipRoof2Rot ||
          roof_type == CylinderRoofRot)
        ridge = longest_wall->PerpendicularLine(longest_wall->MiddlePoint());

      // Get distance of the ridge line by taking the middle of the distances
      // to the longest wall plus the distance of that wall to the origin.
      d_min = d_max = 0;
      for (wall=map_segments.begin(); wall!=map_segments.end(); wall++) {
        d = ridge.DistanceToPointSigned(wall->BeginPoint());
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;
        d = ridge.DistanceToPointSigned(wall->EndPoint());
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;
      }
      roof_parameters.Val(0) = ridge.NormalAngle();
      roof_parameters.Val(1) = ridge.GetDisto() + (d_min + d_max) / 2.0;
      // Ridge height
      roof_parameters.Val(2) = heights[(int) (heights.size() * 0.9)];
      switch (roof_type) {
        case HipRoof:     // Hip roof
        case HipRoofRot:  // Hip roof rotated by 90 degrees
        case HipRoof2:    // Hip roof with two different slopes
        case HipRoof2Rot: // The same hip roof rotated by 90 degrees
         umax = 0.0; vmin = 1e10; vmax = -1e10;
         phi = roof_parameters.Val(0);   d = roof_parameters.Val(1);
         for (wall=map_segments.begin(); wall!=map_segments.end(); wall++) {
           corner = wall->BeginPoint();
           u = corner.X() * cos(phi) + corner.Y() * sin(phi) - d;
           if (fabs(u) > umax) umax = fabs(u);
           v = -corner.X() * sin(phi) + corner.Y() * cos(phi);
           if (v < vmin) vmin = v;
           if (v > vmax) vmax = v;
           corner = wall->EndPoint();
           u = corner.X() * cos(phi) + corner.Y() * sin(phi) - d;
           if (fabs(u) > umax) umax = fabs(u);
           v = -corner.X() * sin(phi) + corner.Y() * cos(phi);
           if (v < vmin) vmin = v;
           if (v > vmax) vmax = v;
         }
         // Scalar of ridge centre
         roof_parameters.Val(4) = (vmin + vmax) / 2.0;
         roof_parameters.Val(5) = vmax - vmin - 2.0 * umax;
         if (roof_parameters.Val(5) < 0.0) roof_parameters.Val(5) = 0.0;
         // Continue with slope as in gable roofs
        case GableRoof: // Gable roofs
        case GableRoofRot:
          // Slope
          roof_parameters.Val(3) =
            (roof_parameters.Val(2) - heights[(int) (heights.size() * 0.1)]) /
            ((d_max - d_min) / 2.0);
          if (roof_type == HipRoof2 || roof_type == HipRoof2Rot)
            roof_parameters.Val(6) = roof_parameters.Val(3);
          break;
        case GambrelRoof:    // u roofs
        case GambrelRoofRot: // ad hoc values, assuming a 45 degree slope
          roof_parameters.Val(3) = 1.0; // Slope
          roof_parameters.Val(4) =
            (d_max - d_min) / 2.0 -
            (roof_parameters.Val(2) - heights[(int) (heights.size() * 0.1)]);
          break;
        case CylinderRoof:    // Cylinder roof
        case CylinderRoofRot: // Cylinder roof rotated by 90 degrees
          // Radius
          roof_parameters.Val(3) =
            roof_parameters.Val(2) - heights[(int) (heights.size() * 0.05)];
          break;
        default:
          break;
      }
      break;

    case SphereRoof: // Sphere
      // Assuming top in the middle of the point cloud
      bounds.Initialise();
      for (wall=map_segments.begin(); wall!=map_segments.end(); wall++)
        bounds.Update(wall->BeginPoint());
      roof_parameters.Val(0) = heights[(int) (heights.size() * 0.98)]; // Top 2%
      roof_parameters.Val(1) = bounds.Minimum().X() + bounds.XRange() / 2.0;
      roof_parameters.Val(2) = bounds.Minimum().Y() + bounds.YRange() / 2.0;
      roof_parameters.Val(3) = (bounds.XRange() + bounds.YRange()) / 4.0;
      break;

    case ConeRoof: // Cone
      if (original_points.size() < 20) {
        // Assuming top in the middle of the point cloud
        bounds.Initialise();
        for (wall=map_segments.begin(); wall!=map_segments.end(); wall++)
          bounds.Update(wall->BeginPoint());
        roof_parameters.Val(0) = heights[(int) (heights.size() * 0.98)]; // Top 2%
        roof_parameters.Val(1) = bounds.Minimum().X() + bounds.XRange() / 2.0;
        roof_parameters.Val(2) = bounds.Minimum().Y() + bounds.YRange() / 2.0;
        roof_parameters.Val(3) = (roof_parameters.Val(0) -
                                  heights[(int) (heights.size() * 0.02)]) /
                                 sqrt(bounds.XRange() * bounds.XRange() +
                                      bounds.YRange() + bounds.YRange());
      }
      else {
        // Make a local (non-const) copy of the points
        points.insert(points.begin(), original_points.begin(),
                      original_points.end());
        printf("# points %d %d\n", original_points.size(), points.size());
        // Get the point in a corner
        point0 = points.begin();
        max_dist = 0.0;
        for (point=point0+1, index=1; point!=points.end(); point++, index++) {
          dist = (point->vect() - point0->vect()).SqLength();
          if (dist > max_dist) {
            max_dist = dist;
            point1 = point;
            index1 = index;
          }
        }
        // Get another point furthest away from the first one
        max_dist = 0;
        for (point=points.begin(), index=0; point!=points.end(); point++, index++) {
          dist = (point->vect() - point1->vect()).SqLength();
          if (dist > max_dist) {
            max_dist = dist;
            point2 = point;
            index2 = index;
          }
        }
        printf("Indices %d %d\n", index1, index2);
        // Derive knn edges of the points
        nbh_parameters.NeighbourhoodStorageModel() = 2; // kd-tree
        nbh_parameters.NumberOfNeighbours() = 20;
        edges = points.DeriveEdges(nbh_parameters);
        printf("num edge sets %d edges size %d %d\n", edges->size(),
               (edges->begin()+index1)->size(),
               (edges->begin()+index2)->size());
        // Fit planes to neighbourhoods of the two selected points
        plane1 = points.FitPlane((edges->begin()+index1)->PointNumberListReference(), 1);
        plane1.Print();
        plane2 = points.FitPlane((edges->begin()+index2)->PointNumberListReference(), 2);
        plane2.Print();
        // Derive vectors of lines in plane with steepest descent through
        // the selected points
        normal = plane1.Normal();
        hor_tangent = normal.VectorProduct(Vector3D(0.0, 0.0, 1.0));
        printf("hor_tangent %.2f %.2f %.2f\n", hor_tangent.X(),
               hor_tangent.Y(), hor_tangent.Z());
        steepest_descent = normal.VectorProduct(hor_tangent);
        printf("steepest_descent %.2f %.2f %.2f\n", steepest_descent.X(),
               steepest_descent.Y(), steepest_descent.Z());
        line1 = Line3D(point1->Position3DRef(), steepest_descent);
        line1.Print();
        normal = plane2.Normal();
        hor_tangent = normal.VectorProduct(Vector3D(0.0, 0.0, 1.0));
        steepest_descent = normal.VectorProduct(hor_tangent);
        line2 = Line3D(point2->Position3DRef(), steepest_descent);
        // Intersect these lines
        Intersection2Lines(line1, line2, centre);
        // Centre point is the intersection point
        roof_parameters.Val(0) = centre.Z();
        roof_parameters.Val(1) = centre.X();
        roof_parameters.Val(2) = centre.Y();
        // Derive slope from centre point and point1
        roof_parameters.Val(3) = 
          (centre.Z() - point1->Z()) /
          (centre.vect2D() - point1->vect2D()).Length();
        printf("Top at %.2f %.2f %.2f, slope %.2f\n", centre.X(),
               centre.Y(), centre.Z(), roof_parameters.Val(3));
        edges->Erase();
        points.ErasePoints();
      }
      break;

    default: printf("Invalid roof type %d\n", roof_type); exit(0);
  }
}

bool BuildingPart::CreateHipRoof(ObjectPoints &roof_points)
{
  Line2D                       ridge;
  LineSegment2D                new_ridge;
  LineSegments2D               intersections, sloped_ridges, roof_edges;
  LineSegments2D::iterator     intersection, longest_segment, roof_edge,
                               sloped_ridge;
  double                       dist, longest_dist;
  ObjectPoints::const_iterator point, ridge_pointA, ridge_pointB;
  LineTopology::const_iterator node;
  LineTopology::iterator       first_node, last_node;
  int                          i, j, found, num_outline_points;
  ObjectPoint                  ridge_point;
  Position2D                   ridge2D, intersection_pos;
  Position3D                   pos2, pos3;
  Vector3D                     ridge_dir_3D;
  Vector2D                     ridge_perp_dir_2D;
  Rotation3D                   rot;
  LineTopology                 roof_top, new_outline;
  Planes                       planes;
  Planes::const_iterator       plane1, plane2;
  Line3D                       inter_line;

  if (roof_points.empty()) {
    printf("Error: expected relevant map_points to be copied to roof_points\n");
    return false;
  }

  // Intersect outline by ridge line
  ridge = Line2D(sin(roof_parameters.Val(0)), cos(roof_parameters.Val(0)),
                 roof_parameters.Val(1));
  if (!map_part_top->begin()->IntersectPolygonByLine(roof_points, ridge,
                                                     intersections,
                                                     0.01, 0.01)) return false;
  
  // Determine the longest intersecting segment
  longest_dist = 0;
  for (intersection=intersections.begin();
       intersection!=intersections.end(); intersection++) {
    if (intersection->Length() > longest_dist) {
      longest_dist = intersection->Length();
      longest_segment = intersection;
    }
  }

  // Calculate maximum building width using first (and only) map partition
  longest_dist = 0;
  for (node=map_part_top->begin()->begin();
       node!=map_part_top->begin()->end(); node++) {
    point = roof_points.ConstPointIterator(node->NumberRef());
    dist  = ridge.DistanceToPoint(Position2D(point->X(), point->Y()));
    if (dist > longest_dist) longest_dist = dist;
  }

  // Create the two ridge points
  ridge_point.Z() = roof_parameters.Val(2);
  ridge2D = ridge.Position(roof_parameters.Val(4) +
                           roof_parameters.Val(5) / 2.0);
  ridge_point.X() = ridge2D.X();
  ridge_point.Y() = ridge2D.Y();
  ridge_point.Number() = (roof_points.end()-1)->Number() + 1;
  roof_points.push_back(ridge_point);
  ridge2D = ridge.Position(roof_parameters.Val(4) - roof_parameters.Val(5) / 2.0);
  ridge_point.X() = ridge2D.X();
  ridge_point.Y() = ridge2D.Y();
  ridge_point.Number() = (roof_points.end()-1)->Number() + 2;
  roof_points.push_back(ridge_point);

  // Create the four new sloped ridge lines (segments starting at ridge points)
  num_outline_points = map_part_top->begin()->size() - 1;
  ridge_dir_3D = roof_points[num_outline_points].vect() -
                 roof_points[num_outline_points+1].vect();
  if (roof_type == HipRoof || roof_type == HipRoofRot) { 
    // Constant slope, sloped ridge lines are at 45 degrees from ridge
    rot = Rotation3D(0.0, 0.0, -atan(1.0)); // -45 degree rotation matrix
    ridge_dir_3D = rot * ridge_dir_3D; // direction of first sloped ridge
    ridge_dir_3D *= 10000.0; // make it a long vector
    rot = Rotation3D(0.0, 0.0, 2.0 * atan(1.0)); // 90 degree rotation matrix
    for (i=0; i<2; i++) {
      for (j=0; j<2; j++) {
        sloped_ridges.push_back(
           LineSegment2D(Position2D(roof_points[num_outline_points+i].vect2D()),
                         Position2D(roof_points[num_outline_points+i].vect2D() +
                                    ridge_dir_3D.vect2D())));
        ridge_dir_3D = rot * ridge_dir_3D;
      }
    }
  }
  else {
    ridge_perp_dir_2D.X() = -ridge_dir_3D.Y();
    ridge_perp_dir_2D.Y() = ridge_dir_3D.X();
    // Two different slopes, construct slope ridge lines by intersecting planes
    // Construct the planes
    for (i=0, j=1; i<2; i++, j-=2) { 
      pos2.X() = roof_points[num_outline_points+i].X() + j * ridge_dir_3D.X();
      pos2.Y() = roof_points[num_outline_points+i].Y() + j * ridge_dir_3D.Y();
      pos2.Z() = RoofHeight(Position2D(pos2.vect2D()));
      pos3.X() = pos2.X() + ridge_perp_dir_2D.X() / 10.0;
      pos3.Y() = pos2.Y() + ridge_perp_dir_2D.Y() / 10.0;
      pos3.Z() = RoofHeight(Position2D(pos3.vect2D()));
      planes.push_back(Plane(roof_points[num_outline_points+i].Position3DRef(),
                             pos2, pos3));

      pos2.X() = roof_points[num_outline_points+i].X() +
                 j * ridge_perp_dir_2D.X();
      pos2.Y() = roof_points[num_outline_points+i].Y() +
                 j * ridge_perp_dir_2D.Y();
      pos2.Z() = RoofHeight(Position2D(pos2.vect2D()));
      pos3.X() = pos2.X() + ridge_dir_3D.X() / 10.0;
      pos3.Y() = pos2.Y() + ridge_dir_3D.Y() / 10.0;
      pos3.Z() = RoofHeight(Position2D(pos3.vect2D()));
      planes.push_back(Plane(roof_points[num_outline_points+i].Position3DRef(),
                             pos2, pos3));
    }
    // Intersect the planes
    plane1 = planes.end() - 1;
    for (plane2=planes.begin(), i=0; plane2!=planes.end();
         plane1=plane2, plane2++, i++) {
      if (Intersect2Planes(*plane1, *plane2, inter_line)) {
        if (i == 0 || i == 1)
          pos2 = roof_points[num_outline_points].Position3DRef();
        else
          pos2 = roof_points[num_outline_points+1].Position3DRef();
        pos3 = inter_line.Position(inter_line.Scalar(pos2) + 100.0);
        if (pos3.Z() < pos2.Z())
          sloped_ridges.push_back(LineSegment2D(Position2D(pos2.vect2D()),
                                                Position2D(pos3.vect2D())));
        else {
          pos3 = inter_line.Position(inter_line.Scalar(pos2) - 100.0);
          sloped_ridges.push_back(LineSegment2D(Position2D(pos2.vect2D()),
                                                Position2D(pos3.vect2D())));
        }
      }
      else printf("Intersection failed\n");
    }
  }

  // Intersect the outline with the sloped ridge lines
  new_outline = *(map_part_top->begin());
  for (sloped_ridge = sloped_ridges.begin(), j=0;
       sloped_ridge != sloped_ridges.end(); sloped_ridge++, j++) {
    // Regenerate roof edges to incorporate new points
    roof_edges = LineSegments2D(roof_points, new_outline);
    for (roof_edge = roof_edges.begin(), i=0, found=0;
         roof_edge != roof_edges.end() && !found; roof_edge++, i++) {
      if (Intersect2LineSegments2D(sloped_ridge->LineSegment2DRef(),
                                   roof_edge->LineSegment2DRef(),
                                   intersection_pos, 0.01)) {
        found = 1;
        ridge_point.X() = intersection_pos.X();
        ridge_point.Y() = intersection_pos.Y();
        ridge_point.Number() = (roof_points.end()-1)->Number() + 1;

        // Put point in point list and new outline point number list
        roof_points.push_back(ridge_point);
        new_outline.insert(new_outline.begin()+i+1, ridge_point.NumberRef());
      }
    }
  }

  if (roof_points.size() != map_part_top->begin()->size() + 5) {
    printf("Error: missed some ridge points, no roof reconstructed.\n");
    return false;
  }

  // Create the topology of the four faces
  first_node = new_outline.begin();
  while (first_node->Number() < roof_points[num_outline_points+2].Number())
    first_node = new_outline.NextNode(first_node);
  if (!roof) roof = new LineTopologies();
  for (i=0; i<4; i++) {
    // Clear roof face topology
    if (!roof_top.empty()) roof_top.erase(roof_top.begin(), roof_top.end());
    // Collect the points on the outline between two new ridge points
    roof_top.push_back(first_node->NumberRef());
    last_node = new_outline.NextNode(first_node);
    roof_top.push_back(last_node->NumberRef());
    while (last_node->Number() < roof_points[num_outline_points+2].Number()) {
      last_node = new_outline.NextNode(last_node);
      roof_top.push_back(last_node->NumberRef());
    }
    // Add one or two of the top ridge points
    if (first_node->Number() == roof_points[num_outline_points+2].Number()) {
      if (last_node->Number() == roof_points[num_outline_points+3].Number()) {
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
      }
      else { // last point 5
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
      }
    }
    else if (first_node->Number() == roof_points[num_outline_points+3].Number()) {
      if (last_node->Number() == roof_points[num_outline_points+2].Number()) {
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
      }
      else { // last point 4
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
      }
    }
    else if (first_node->Number() == roof_points[num_outline_points+4].Number()) {
      if (last_node->Number() == roof_points[num_outline_points+5].Number()) {
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
      }
      else { // last point 3
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
      }
    }
    else { // first point 5
      if (last_node->Number() == roof_points[num_outline_points+4].Number()) {
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
      }
      else { // last point 2
        roof_top.push_back(roof_points[num_outline_points+0].NumberRef());
        roof_top.push_back(roof_points[num_outline_points+1].NumberRef());
      }
    }
    // Close and store the polygon
    roof_top.push_back(roof_top.begin()->NumberRef());
    roof->push_back(roof_top);
    // Set first node for next roof face
    first_node = last_node;
  }

  return true;
}

void BuildingPart::CreateSphereRoof(const LaserPoints &laser_points,
                                    ObjectPoints &roof_points,
                                    double ground_height)
{
  double                      max_rad, rad, pi=4.0*atan(1.0), alpha, alpha_inc,
                              beta, beta_inc, height, rh, radius;
  Position2D                  centre;
  LaserPoints::const_iterator laserpoint;
  int                         num_nodes, i, num, num_nodes1, num_roof_faces;
  LineTopology                circle;
  LineTopologies              one_circle;
  TIN                         tin;
  ObjectPoints::iterator      roof_point;

  double tolerance = 0.05;

  // Remove old roof points
  if (!roof_points.empty())
    roof_points.erase(roof_points.begin(), roof_points.end());

  // Extract parameters
  rh      = roof_parameters.Val(0);
  centre  = Position2D(roof_parameters.Val(1), roof_parameters.Val(2));
  radius  = roof_parameters.Val(3);

  // Determine maximum radius
  max_rad = 0.0;
  for (laserpoint=laser_points.begin(); laserpoint!=laser_points.end();
       laserpoint++) {
    rad = (centre - laserpoint->vect2D()).Length();
    if (rad > max_rad) max_rad = rad;
  }
  if (max_rad > radius) max_rad = radius;

// Generate the points on the sphere

  alpha_inc = 2.0 * acos((radius - tolerance) / radius);
  num = 0;
  for (alpha = asin(max_rad / radius); alpha > 0.0; alpha -= alpha_inc) {
    rad       = radius * sin(alpha);
    beta_inc  = 2.0 * acos((rad - tolerance) / rad);
    beta_inc /= 3.0;
    num_nodes = (int) (2.0 * pi / beta_inc) + 1;
    beta_inc  = 2.0 * pi / num_nodes;
    height    = rh - radius + radius * cos(alpha);
    if (num == 0) num_nodes1 = num_nodes;
    for (i=0, beta=0.0; i<num_nodes; i++, beta+=beta_inc, num++) {
      roof_points.push_back(ObjectPoint(centre.X() + rad * cos(beta),
                                        centre.Y() + rad * sin(beta), height,
                                        num, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      if (i == num) circle.push_back(PointNumber(num));
    }
  }
  // Add top point
  roof_points.push_back(ObjectPoint(centre.X(), centre.Y(), rh, num,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

// Triangulate the points

  tin = roof_points.Triangulate();
  roof = new LineTopologies(tin);
  roof->MakeCounterClockWise(roof_points);

// Add the walls of the cylinder

  circle.push_back(PointNumber(0));
  circle.MakeCounterClockWise(roof_points);
  one_circle.push_back(circle);
  roof_points.insert(roof_points.end(), roof_points.begin(),
                     roof_points.begin()+num_nodes1);
  for (roof_point=roof_points.end()-num_nodes1;
       roof_point!=roof_points.end(); roof_point++) {
    roof_point->Number() += num+1;
    roof_point->Z()       = ground_height;
  }
  num_roof_faces = roof->size();
  roof->AddWalls(one_circle, num+1);
  
// Transfer wall faces to the wall face set

  walls->insert(walls->end(), roof->begin()+num_roof_faces, roof->end());
  roof->erase(roof->begin()+num_roof_faces, roof->end());
}

void BuildingPart::CreateConeRoof(const LaserPoints &laser_points,
                                  ObjectPoints &roof_points,
                                  double ground_height)
{
  Position2D                  centre;
  ObjectPoint2D               centre_point;
  int                         number_offset;
  LaserPoints::const_iterator laserpoint;
  TIN                         tin;
  ObjectPoints::iterator      roof_point;
  bool                        added_centre=false;
  LineTopologies              *one_outside;
  
  // Copy the outline and renumber the points
  one_outside = new LineTopologies();
  one_outside->push_back(map_part_top->begin()->LineTopologyReference());
  one_outside->ReNumber(roof_points);
  
  // Add the cone centre if it is inside the map outline
  centre  = Position2D(roof_parameters.Val(1), roof_parameters.Val(2));
  centre_point = ObjectPoint2D(centre, PointNumber((int) roof_points.size()),
                               Covariance2D());
  if (centre_point.InsidePolygon(roof_points,
                               one_outside->begin()->LineTopologyReference())) {
    roof_points.push_back(ObjectPoint(&centre_point, 0.0));
    added_centre = true;
  }
  
  // Determine the heights of all roof points
  for (roof_point=roof_points.begin(); roof_point!=roof_points.end();
       roof_point++)
    roof_point->Z() = RoofHeight(Position2D(roof_point->vect2D()));

  // Triangulate the roof points
  tin = roof_points.Triangulate(*one_outside); // Constrained by outline
  roof = new LineTopologies(tin);
  roof->MakeCounterClockWise(roof_points);
  roof->SetAttribute(LineLabelTag, RoofLabel);

  // Construct and add the walls
  number_offset = roof_points.size();
  if (added_centre) // Temporarily remove centre
    roof_points.erase(roof_points.end()-1); 
  roof_points.DuplicateWithFixedZ(ground_height, number_offset);
  if (added_centre) // Put centre back again
    roof_points.push_back(ObjectPoint(&centre_point,
                                      RoofHeight(centre_point.Position2DRef())));
  walls = new LineTopologies();
  // Make walls
  walls->AddWalls(one_outside->LineTopologiesReference(), number_offset);
  walls->SetAttribute(LineLabelTag, WallLabel);
  delete one_outside;
}

void BuildingPart::CreateCylinderRoof(ObjectPoints &roof_points,
                                      double ground_height)
{
  double                      max_rad, rad, pi=4.0*atan(1.0), alpha, alpha_inc,
                              height, rh, radius, phi, d, alpha_max, shift;
  int                         number_offset;
  LineTopology                outline;
  LineTopology::iterator      node;
  TIN                         tin;
  ObjectPoints::iterator      roof_point;
  Line2D                      ridge, cylinder_line;
  LineSegments2D              segments;
  LineSegments2D::iterator    segment;
  LineTopologies              one_outline;
  ObjectPoint                 intersection_point;
  ObjectPoints                intersection_points;

  double tolerance = 0.05;

  // Remove old roof points
  if (!roof_points.empty())
    roof_points.erase(roof_points.begin(), roof_points.end());

  // Copy the outline and renumber the points
  outline = map_part_top->begin()->LineTopologyReference();
  for (node=outline.begin(); node!=outline.end(); node++)
    roof_points.push_back(*(map_points->PointIterator(*node)));
  one_outline.push_back(outline);
  one_outline.ReNumber(roof_points);
  outline = *(one_outline.begin());
 
  // Extract parameters
  phi    = roof_parameters.Val(0);     // Ridge normal orientation
  d      = roof_parameters.Val(1);     // Ridge distance to origin
  rh     = roof_parameters.Val(2);     // Ridge height
  radius = roof_parameters.Val(3);     // Cyclinder radius

  // Ridge line
  ridge = Line2D(sin(phi), cos(phi), d);

  // Determine maximum radius
  max_rad = 0.0;
  for (roof_point=roof_points.begin(); roof_point!=roof_points.end();
       roof_point++) {
    rad = ridge.DistanceToPoint(Position2D(roof_point->vect2D()));
    if (rad > max_rad) max_rad = rad;
  }
  if (max_rad > radius) max_rad = radius;
  alpha_max = asin(max_rad/radius);

  // Generate the points on the cylinder
  alpha_inc = 2.0 * acos((radius - tolerance) / radius);
  alpha_max = (int) (alpha_max/alpha_inc + 1.0) * alpha_inc;
  for (alpha = -alpha_max; alpha < alpha_max; alpha += alpha_inc) {
    // Construct line on cylinder at alpha radians from the top line
    shift = radius * sin(alpha);
    cylinder_line = Line2D(sin(phi), cos(phi),
                           ridge.DistanceToOrigin() + shift);

    // Collect intersection points of building outline with this line
    outline.IntersectPolygonByLine(roof_points, cylinder_line, segments,
                                   tolerance, tolerance, (alpha != -alpha_max));

    // Points should be inserted in outline for the construction of the walls
    // For now, just store them in a list
    for (segment=segments.begin(); segment!=segments.end(); segment++) {
      intersection_point.X() = segment->BeginPoint().X();
      intersection_point.Y() = segment->BeginPoint().Y();
      intersection_point.Number() = roof_points.size() +
                                    intersection_points.size();
      intersection_points.push_back(intersection_point);
      intersection_point.X() = segment->EndPoint().X();
      intersection_point.Y() = segment->EndPoint().Y();
      intersection_point.Number() = roof_points.size() +
                                    intersection_points.size();
      intersection_points.push_back(intersection_point);
    }
  }
  
  // Add the intersection points to the building outline
  roof_points.insert(roof_points.end(), intersection_points.begin(),
                     intersection_points.end());
  outline.InsertNodes(roof_points, tolerance);
  
  // Determine height of all roof points
  for (roof_point=roof_points.begin(); roof_point!=roof_points.end();
       roof_point++)
    roof_point->Z() = RoofHeight(Position2D(roof_point->vect2D()));
    
  // Triangulate the points
  tin = roof_points.Triangulate();
  roof = new LineTopologies(tin);
  roof->MakeCounterClockWise(roof_points);

  // Add the walls of the building
  number_offset = roof_points.size()+1;
  roof_points.DuplicateWithFixedZ(ground_height, number_offset);
  if (walls == NULL) walls = new LineTopologies();
  else walls->Erase();
  one_outline.push_back(outline);
  walls->AddWalls(one_outline, number_offset);
  walls->SetAttribute(LineLabelTag, WallLabel);
  
  one_outline.Erase();
}

std::vector <LineTopologies *> * BuildingPart::ModelTopology() const
{
  std::vector <LineTopologies *> *model_parts;

  model_parts = new std::vector <LineTopologies *>;
  if (roof)  model_parts->push_back(roof);
  if (walls) model_parts->push_back(walls);
  return model_parts;
}

bool BuildingPart::FitRoofPrimitive(const LaserPoints &points,
                                    const LineSegments2D &map_segments,
                                    double &rms, int max_iter)
{
  LaserPoints::const_iterator    point;
  Position2D                     pos, centre_pos;
  vector<double>                 heights;
  double                         pi, res, ressq, sigma0, sigma0prev, tol, obs;
  LSAdjust                       adjustment;
  int                            converged, iter, par, numpar, numres;
  LineSegments2D::const_iterator map_segment;
  DataBounds2D                   bounds;
  vRecord                        row_A, sol;

  pi = 4.0 * atan(1.0);

// Determine the centre point of the roof

  bounds.Initialise();
  for (map_segment=map_segments.begin();
       map_segment!=map_segments.end(); map_segment++)
    bounds.Update(map_segment->BeginPoint());
  centre_pos.X() = bounds.Minimum().X() + bounds.XRange() / 2.0;
  centre_pos.Y() = bounds.Minimum().Y() + bounds.YRange() / 2.0;

// Determine approximate values of roof parameters

  ApproximateRoofValues(map_segments, points);

// Apply shifts to the approximate parameters

  if (roof_type == SphereRoof || roof_type == ConeRoof) {
    roof_parameters.Val(1) -= centre_pos.X();
    roof_parameters.Val(2) -= centre_pos.Y();
  }
  else if (roof_type != FlatRoof && roof_type != ShedRoof)
    roof_parameters.Val(1) -= centre_pos.X() * cos(roof_parameters.Val(0)) +
                   centre_pos.Y() * sin(roof_parameters.Val(0));
  if (roof_type == HipRoof || roof_type == HipRoofRot ||
      roof_type == HipRoof2 || roof_type == HipRoof2Rot) {
    roof_parameters.Val(4) -= -centre_pos.X() * sin(roof_parameters.Val(0)) +
                   centre_pos.Y() * cos(roof_parameters.Val(0));
  }

// Iterative estimation

  iter = 0;
  converged = 0;
  numpar = roof_parameters.size();
  row_A = vRecord(numpar, 0.0);
  adjustment.SetEPS(1.0e-30);
  tol = 1.0;
  do {

// Set up and solve the equation system

    if (iter) {
      adjustment.FullReset();
      for (point=points.begin(); point!=points.end(); point++) {
        pos = Position2D(point->X()-centre_pos.X(), point->Y()-centre_pos.Y());
        // Row of design matrix
        for (par=0; par<numpar; par++)
          row_A[par] = PartialRoofDerivative(pos, par);
        // Observation
        obs = point->Z() - RoofHeight(pos);
        // Update design matrix
        if (fabs(obs) < tol) adjustment.Update_Normal_Equations(row_A, obs,1.0);
      }
      // Estimate increments
      if (!adjustment.Adjust()) {
        printf("Adjustment failed in iteration %d with %d observations\n",
               iter, adjustment.nObs());
        return false;
      }
      sol = adjustment.Solution();
      // Update paramters
      for (par=0; par<numpar; par++) roof_parameters.Val(par) += sol[par];
    }

// Estimate residuals

    for (point=points.begin(), ressq=0.0, numres=0;
         point!=points.end(); point++) {
      pos = Position2D(point->X()-centre_pos.X(), point->Y()-centre_pos.Y());
      res = point->Z() - RoofHeight(pos);
      if (fabs(res) < tol) {
        ressq += res * res;
        numres++;
      }
    }

    sigma0prev = sigma0;
    sigma0 = sqrt(ressq/(numres-numpar));
    if (iter) {
      converged = (sigma0 / sigma0prev > 0.99);
      for (par=0; par<numpar && converged; par++)
        if (fabs(sol[par]) > 0.01) converged = 0;
      tol = 0.5;
    }
    switch (roof_type) {
      case FlatRoof:
        printf("%2d rh %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0), sigma0, points.size()-numres,
               numres);
        break;
      case GableRoof:
      case GableRoofRot:
        printf("%2d phi %6.2f d %5.2f rh %5.2f sl %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0) * 180 / pi, roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3), sigma0,
               points.size()-numres, numres);
        break;
      case HipRoof:
      case HipRoofRot:
        printf("%2d phi %6.2f d %5.2f rh %5.2f sl %5.2f sm %5.2f rl %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0) * 180 / pi, roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3),
               roof_parameters.Val(4), roof_parameters.Val(5), sigma0, 
               points.size()-numres, numres);
        break;
      case GambrelRoof:
      case GambrelRoofRot:
        printf("%2d phi %6.2f d %5.2f rh %5.2f sl %5.2f rw %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0) * 180 / pi, roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3),
               roof_parameters.Val(4), sigma0, points.size()-numres, numres);
        break;
      case HipRoof2: // Hip roof with two different slopes
      case HipRoof2Rot:
        printf("%2d phi %6.2f d %5.2f rh %5.2f sl %5.2f sm %5.2f rl %5.2f sl2 %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0) * 180 / pi, roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3),
               roof_parameters.Val(4), roof_parameters.Val(5),
               roof_parameters.Val(6), sigma0, points.size()-numres, numres);
        break;
      case SphereRoof:
        printf("%2d rh %5.2f xc %5.2f yc %5.2f rad %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0), roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3), sigma0, 
               points.size()-numres, numres);
        break;
      case ConeRoof:
        printf("%2d rh %5.2f xc %5.2f yc %5.2f sl %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0), roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3), sigma0, 
               points.size()-numres, numres);
        break;
      case ShedRoof:
        printf("%2d rh %5.2f slx %5.2f sly %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0), roof_parameters.Val(1),
               roof_parameters.Val(2), sigma0, points.size()-numres, numres);
        break;
      case CylinderRoof:
      case CylinderRoofRot:
        printf("%2d phi %6.2f d %5.2f rh %5.2f rad %5.2f sig0 %5.3f out %3d in %3d",
               iter, roof_parameters.Val(0) * 180 / pi, roof_parameters.Val(1),
               roof_parameters.Val(2), roof_parameters.Val(3), sigma0,
               points.size()-numres, numres);
        break;
      default:
        break;
    }
    if (iter) printf(" rel %5.2f\n", sigma0/sigma0prev);
    else printf("\n");
    iter++;
  } while (!converged && iter<=max_iter && numres);

  if (numres == 0) {
    rms = 1e10;
    return false;
  }
  rms = sqrt(ressq/numres);
  if (!converged) rms = 1e10;

// Correct for the offset

  if (roof_type == SphereRoof || roof_type == ConeRoof) {
    roof_parameters.Val(1) += centre_pos.X();
    roof_parameters.Val(2) += centre_pos.Y();
  }
  else if (roof_type == ShedRoof) {
    roof_parameters.Val(0) -= roof_parameters.Val(1) * centre_pos.X() +
                              roof_parameters.Val(2) * centre_pos.Y();
  }
  else if (roof_type != FlatRoof)
    roof_parameters.Val(1) += centre_pos.X() * cos(roof_parameters.Val(0)) +
                              centre_pos.Y() * sin(roof_parameters.Val(0));
  if (roof_type == HipRoof || roof_type == HipRoofRot ||
      roof_type == HipRoof2 || roof_type == HipRoof2Rot) {
    roof_parameters.Val(4) += -centre_pos.X() * sin(roof_parameters.Val(0)) +
                              centre_pos.Y() * cos(roof_parameters.Val(0));
  }

  return true;
}

bool BuildingPart::ReconstructRoofPrimitive(RoofType type,
                                            const LaserPoints &laser_points,
                                            ObjectPoints &roof_points,
                                            double ground_height)
{
  double                   rms;
  LineTopologies           initial_roof, *one_outside;
  LineTopologies::iterator face;
  LineTopology::iterator   node;
  LineTopology             circle_polygon, *outside;
  Circle2D                 circle;
  Line2D                   ridge;
  ObjectPoint              new_point, *map_point;
  ObjectPoints::iterator   roof_point;
  LineSegments2D           map_segments;
  int                      number_offset, num_roof_faces;

  if (!PrepareRoofReconstruction(type, laser_points, roof_points))
    return false;
  if (map_part_top->size() != 1) return false;

  // Derive map segments
  map_segments = LineSegments2D(map_points->ObjectPointsRef(),
                                map_part_top->begin()->LineTopologyReference());

  // Fit the point cloud
  if (!FitRoofPrimitive(laser_points, map_segments, rms, 20)) return false;

// Generate building model

  // Copy the points of the first (and only) map partition
  for (node=map_part_top->begin()->begin(); 
       node!=map_part_top->begin()->end()-1; node++) {
    map_point = map_points->GetPoint(node->NumberRef());
    roof_points.push_back(*map_point);
  }
  roof_points.Sort();

  switch (roof_type) {
    case FlatRoof:
    case ShedRoof:
      roof->push_back(map_part_top->begin()->LineTopologyReference());
      break;

    case GableRoof:
    case GableRoofRot:
      ridge = Line2D(sin(roof_parameters.Val(0)), cos(roof_parameters.Val(0)),
                     roof_parameters.Val(1));
      if (!map_part_top->begin()->SplitPolygonByLine(roof_points, ridge,
                                                     0.01, 0.01,
                                             roof->LineTopologiesReference())) {
        printf("Error splitting polygon by estimated ridge line\n");
        return false;
      }
      break;

    case HipRoof:
    case HipRoofRot:
    case HipRoof2:
    case HipRoof2Rot:
      if (!CreateHipRoof(roof_points)) {
        printf("Error in constructing hip roof\n");
        return false;
      }
      // Temporarily set all point heights to zero (needed for InsertNodes)
      for (roof_point=roof_points.begin(); roof_point!=roof_points.end();
           roof_point++)
        roof_point->Z() = 0.0;
      break;

    case GambrelRoof:
    case GambrelRoofRot:
      ridge = Line2D(sin(roof_parameters.Val(0)), cos(roof_parameters.Val(0)),
                     roof_parameters.Val(1) + roof_parameters.Val(4) / 2.0);
      if (!map_part_top->begin()->SplitPolygonByLine(roof_points, ridge,
                                                     0.01, 0.01,initial_roof)) {
        printf("Error splitting polygon by estimated ridge line\n");
        return false;
      }
      ridge = Line2D(sin(roof_parameters.Val(0)), cos(roof_parameters.Val(0)),
                     roof_parameters.Val(1) - roof_parameters.Val(4) / 2.0);
      if (initial_roof[0].SplitPolygonByLine(roof_points, ridge, 0.01, 0.01,
                                             roof->LineTopologiesReference()))
        roof->push_back(initial_roof[1]);
      else if (initial_roof[1].SplitPolygonByLine(roof_points, ridge,
                                                  0.01, 0.01,
                                               roof->LineTopologiesReference()))
        roof->push_back(initial_roof[0]);
      else {
        printf("Error splitting polygon by estimated ridge line\n");
        return false;
      }
      break;

    case SphereRoof:
      CreateSphereRoof(laser_points, roof_points, ground_height);
      break;

    case ConeRoof:
      CreateConeRoof(laser_points, roof_points, ground_height);
      break;

    case CylinderRoof:
    case CylinderRoofRot:
      CreateCylinderRoof(roof_points, ground_height);
      break;

    default:
      printf("Reconstruction of roof type %d not yet implemented\n", roof_type);
      return false;
  }

  if (roof_type != SphereRoof && roof_type != ConeRoof &&
      roof_type != CylinderRoof && roof_type != CylinderRoofRot) {
    // Create a polygon with all edges of the buildings outside
    outside = new LineTopology(*(map_part_top->begin()));
    outside->InsertNodes(roof_points);
    outside->MakeCounterClockWise(roof_points);

    // Determine the heights of all roof points
    for (roof_point=roof_points.begin(); roof_point!=roof_points.end();
         roof_point++)
      roof_point->Z() = RoofHeight(Position2D(roof_point->vect2D()));

    // Make all roof polygons counter clockwise
    roof->MakeCounterClockWise(roof_points);

    // Construct the walls
    number_offset = roof_points.HighestPointNumber().Number() + 1;
    roof_points.DuplicateWithFixedZ(ground_height, number_offset);
    one_outside = new LineTopologies();
    one_outside->push_back(*outside);
    num_roof_faces = roof->size();
    roof->AddWalls(one_outside->LineTopologiesReference(), number_offset);
    roof->ReNumber(roof_points);
    walls->insert(walls->end(), roof->begin()+num_roof_faces, roof->end());
    roof->erase(roof->begin()+num_roof_faces, roof->end());
    delete outside;
    delete one_outside;
  }
  
  // Set all roof and wall labels
  for (face=roof->begin(); face!=roof->end(); face++)
    face->Label() = RoofLabel;
  for (face=walls->begin(); face!=walls->end(); face++)
    face->Label() = WallLabel;
  model_points = &roof_points;

  return true;
}

bool BuildingPart::PrepareRoofReconstruction(RoofType type,
                                             const LaserPoints &laser_points,
                                             ObjectPoints &roof_points)
{
  // Check if the required data is there
  if (laser_points.empty()) return false; 
  if (!map_part_top) return false;
  if (map_part_top->empty()) return false;

  // Empty model points and topology
  if (!roof_points.empty())
    roof_points.erase(roof_points.begin(), roof_points.end());
  if (roof) {
    if (!roof->empty()) roof->erase(roof->begin(), roof->end());
  }
  else {
    roof = new LineTopologies();
    if (!roof) return false;
  }
  if (walls) {
    if (!walls->empty()) walls->erase(walls->begin(), walls->end());
  }
  else {
    walls = new LineTopologies();
    if (!walls) return false;
  }

  // Set the roof type
  SetRoofType(type);

  return true;
}

bool BuildingPart::ReconstructComplexRoof(LaserPoints &laser_points,
                                          ObjectPoints &roof_points,
                                          const FittingParameters &parameters,
                                          double local_ground_height)
{
  TINEdges                     building_edges;
  Planes                       initial_planes, face_planes;
  PointNumberLists             initial_face_points, face_points;
  LineTopologies               initial_face_contours, face_contours,
                               segm_map_lines, refined_map_lines,
                               intersection_lines, jump_edges, building_faces;
  ObjectPoints2D               segm_map_points, refined_map_points,
                               intersection_points, jump_edge_points;
  ObjectPoint                  *objpt;
  ObjectPoint2D                segm_map_point;
  LineTopology::const_iterator node;
  ObjectPoints::iterator       corner;
  LineTopologies::iterator     face, segm_map_line;
  int                          wall_label;
  const FittingParameters      *p;

  // Check on input data
  if (!PrepareRoofReconstruction(UnknownRoof, laser_points, roof_points))
    return false;

// Construct input data

  // Construct TIN and TIN edges
  laser_points.DeriveTIN();
  building_edges.Derive(laser_points.TINReference());

  // Copy the map partition data
  segm_map_lines = *map_part_top;
  for (segm_map_line=segm_map_lines.begin();
       segm_map_line!=segm_map_lines.end(); segm_map_line++) {
    for (node=segm_map_line->begin(); node!=segm_map_line->end()-1; node++) {
      if (segm_map_points.FindPoint(*node) == -1) {
        objpt = map_points->GetPoint(node->Number());
        segm_map_point.Number() = objpt->Number();
        segm_map_point.vect()   = objpt->vect2D();
        segm_map_points.push_back(segm_map_point);
      }
    }
  }
  segm_map_lines.ReNumber(segm_map_points, 0, 0);

// Reconstruct building

  p = &parameters;
  laser_points.ReconstructBuilding(
     // Intermediate and final results
     building_edges,
     initial_planes, initial_face_points, initial_face_contours,
     face_planes, face_points, face_contours,
     segm_map_points, segm_map_lines, refined_map_points, refined_map_lines,
     intersection_points, intersection_lines, jump_edge_points, jump_edges,
     roof_points, building_faces,

     // Control parameters
     p->UseCurrentSegmentation(), p->HoughSpaceModel(),
     p->MaximumSlope(), p->SlopeBinSize(), p->DistanceBinSize(),
     p->MinimumRoofPointHeight(), p->MinimumNumberOfPlanePoints(),
     p->LocalMaximumWindowSize(), p->MaximumDistancePointToPlane(),

     p->MinimumDistanceTwoPlanes(), p->MinimumAngleTwoPlanes(),
     p->MaximumAngleHorizontal(),

     p->MinimumDistanceIntersection(), p->MinimumAngleIntersection(),
     p->MaximumDistancePointToIntersection(),

     p->MinimumDistanceHeightJump(), p->MinimumAngleHeightJump(),
     p->MaximumDistancePointToHeightJump(), p->MinimumJumpHeight(),

     p->MinimumPartitionSize(), p->MinimumPartitionPercentage(),

     // Ground height
     local_ground_height,
     
     // Output files
     NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  // Reset heights of all wall points
  for (corner=roof_points.begin(); corner!=roof_points.end();
       corner++)
    if (corner->Z() == laser_points.Bounds().Minimum().Z()) corner->Z()= -2.0;

  // Transfer topologies of all roof and wall faces
  model_points = &roof_points;
  wall_label = 0;
  for (face=building_faces.begin(); face!=building_faces.end(); face++)
    if (face->Label() > wall_label) wall_label = face->Label();
  for (face=building_faces.begin(); face!=building_faces.end(); face++)
    if (face->Label() == wall_label) {
      face->Label() = WallLabel;
      walls->push_back(*face);
    }
    else {
      face->Label() = RoofLabel;
      roof->push_back(*face);
    }
  return true;
}

void BuildingPart::DeriveResiduals(LaserPoints &points,
                                   bool remove_old_residuals)
{
  LaserPoints::iterator          point;
  LineTopologies::const_iterator face;
  Planes::const_iterator         plane;
  float                          residual, old_residual;
  int                            success;
  bool                           found;
  
  // Remove old residuals if requested
  if (remove_old_residuals) points.RemoveAttribute(ResidualTag);
  
  // Derive roof planes in case of an automatically reconstructed roof
  if (roof_type == UnknownRoof) DeriveRoofPlanes();
  
  // Derive the residuals
  for (point=points.begin(); point!=points.end(); point++) {
    // For automatically reconstructed roofs, look for the right roof face
    // and calculate the distance to the plane
    if (roof_type == UnknownRoof) {
      for (face=roof->begin(), plane=roof_planes->begin(), found=false;
           face!=roof->end() && !found; face++, plane++) {
        if (point->InsidePolygon(model_points->ObjectPointsRef(),
                                 face->LineTopologyReference())) {
          found = true;
          residual = point->Z() - plane->Z_At(point->X(), point->Y(), &success);
        }
      }
    }
    // For shape primitives, use the parametric roof description
    else
      residual = point->Z() - RoofHeight(Position2D(point->vect2D()));

    // Store the smallest residual if the point already has a residual
    if (point->HasAttribute(ResidualTag)) {
      old_residual = point->FloatAttribute(ResidualTag);
      if (fabs(residual) > fabs(old_residual))
        point->FloatAttribute(ResidualTag) = residual;
    }
    else
      point->FloatAttribute(ResidualTag) = residual;
  }
}

// Print all polygons of this building part
void BuildingPart::Print() const
{
  LineTopologies::const_iterator top;
  
  printf("Building part %d\n", num);
  if (map_part_top) {
    printf("Map partition data\n");
    for (top=map_part_top->begin(); top!=map_part_top->end(); top++)
      top->Print();
  }
  if (roof) {
    printf("Roof data\n");
    for (top=roof->begin(); top!=roof->end(); top++)
      top->Print();
  }
  if (walls) {
    printf("Wall data\n");
    for (top=walls->begin(); top!=walls->end(); top++)
      top->Print();
  }
  
}
