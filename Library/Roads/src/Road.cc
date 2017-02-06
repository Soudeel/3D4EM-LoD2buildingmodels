
/*
                  Copyright 2010 University of Twente
 
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



#include "Road.h"
#include "math.h"

RoadPart * Road::RoadPartPtr(int number)
{
  Road::iterator part;
  for (part=begin(); part!=end(); part++)
    if (part->Number() == number) return part->RoadPartPtr();
  return NULL;
}

char * Road::DeriveMetaDataFileName(const char *directory)
{
  if (meta_file) free(meta_file);
  meta_file = ComposeFileName(directory, name, ".road");
  return meta_file;
}

/*
--------------------------------------------------------------------------------
                       Read the road meta data
--------------------------------------------------------------------------------
*/
bool Road::ReadMetaData(const char *filename)
{
  FILE                  *fd;
  char                  *buffer, *line, *keyword;
  int                   keyword_length, success;

  // Open the file
  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening road database %s\n", filename);
    return false;
  }
  
  // Verify that the first keyword is "road"
  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading road database %s\n", filename);
    fclose(fd);
    return false;
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "road", max(keyword_length, 4))) {
    fprintf(stderr, "Error: File %s is not a road database.\n", filename);
    fprintf(stderr, "       First keyword is %s, not road.\n",
            keyword);
  }

  // Initialise all meta data and delete old data
  if (name) free(name);
  if (size()) erase(begin(), end());

  // Store meta data file name
  if (meta_file) free(meta_file);
  StringCopy(&meta_file, filename);
  
  // Process all lines
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "name", max(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "metadata", max(keyword_length, 8)))
          meta_file = BNF_String(line);

        else if (!strncmp(keyword, "roadpart", max(keyword_length, 8))) {
          push_back(RoadPart());
          (end() - 1)->ReadMetaData(BNF_String(line));
        }

        else if (!strncmp(keyword, "endroad", max(keyword_length,7))) {
          free(buffer);
          fclose(fd);
          return true;
        }

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endroad.\n");
  free(buffer);
  fclose(fd);
  return false;
}

/*
--------------------------------------------------------------------------------
                   Write the road meta data to file
--------------------------------------------------------------------------------
*/

bool Road::WriteMetaData(bool write_road_parts) const
{
  FILE                 *fd;
  int                  indent;
  Road::const_iterator road_part;

  // Open the file
  fd = fopen(meta_file, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening road database %s\n", meta_file);
    return false;
  }

  // Write the meta data
  indent = 0;
  BNF_Write_String(fd, "road", indent, NULL);  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (meta_file) BNF_Write_String(fd, "metadata", indent, meta_file);
  for (road_part=begin(); road_part!=end(); road_part++) {
    BNF_Write_String(fd, "roadpart", indent, road_part->MetaDataFile());
    if (write_road_parts) road_part->WriteMetaData();
  }
  indent -= 2;
  BNF_Write_String(fd, "endroad", 0, NULL);

  // Close the file and return success
  fclose(fd);
  return true;
}

/*
--------------------------------------------------------------------------------
   Generate road part outlines and centre lines based on a sensor trajectory
--------------------------------------------------------------------------------
*/

int Road::InitialiseRoadParts(const Positions3D &car_trajectory,
                              double track_part_length, double track_width,
                              double track_part_overlap,
                              double track_outline_step_size,
                              const char *directory)
{
  ObjectPoints                *road_part_points;
  LineTopologies              *road_part_tops;
  bool                        done=false;
  Positions3D                 track_steps;
  vector<double>              track_distances;
  vector<double>::iterator    track_distance;
  vector<Vector3D>            normals;
  Vector3D                    bound_normals[2], direction;
  Position3D                  step_start, step_end, bound_pos[2];
  Positions3D::const_iterator track_pos, prev_track_pos, next_track_pos;
  double                      fraction, dist, dist_sum, track_length,
                              dist_start, dist_overlap[2], index_real,
                              index_fraction, track_length2;
  int                         index_start, index_end, index_int, point_no,
                              num_pts, use_overlap;
  ObjectPoint                 road_part_point;
  LineTopology                road_part_top;
  Road::iterator              road_part;
  
  if (track_part_length < track_outline_step_size) return -1;
  if (car_trajectory.size() < 2) return -2;
  if (track_width <= 0.0) return -3;
  
  // Collect all track positions at distances track_outline_step_size
  track_pos = car_trajectory.begin();
  track_steps.push_back(*track_pos);
  track_distances.push_back(0.0);
  step_end = *track_pos;
  track_length = track_length2 = 0.0;
  do {
    step_start = step_end;
    dist = dist_sum = step_start.Distance(*track_pos);
    while (dist_sum < track_outline_step_size && track_pos != car_trajectory.end()) {
      prev_track_pos = track_pos;
      track_pos++;
      if (track_pos != car_trajectory.end()) {
        dist = track_pos->Distance(*prev_track_pos);
        track_length2 += dist;
        track_distances.push_back(track_length2);
        dist_sum += dist;
      }
    }
    if (dist_sum >= track_outline_step_size) {
      fraction = (track_outline_step_size - dist_sum + dist) / dist;
      if (dist == dist_sum) step_end = step_start;
      else step_end = *prev_track_pos;
      step_end.X() += fraction * (track_pos->X() - step_end.X());
      step_end.Y() += fraction * (track_pos->Y() - step_end.Y());
      track_steps.push_back(step_end);
      track_length += track_outline_step_size;
    }
    else {
      done = true;
      if (step_start.Distance(*(car_trajectory.end()-1)) > 0.0) {
        track_steps.push_back(*(car_trajectory.end()-1));
        track_length += step_start.Distance(*(car_trajectory.end()-1));
      }
    }  
  } while (!done);
  
  // Get normal vectors at all steps
  for (track_pos=track_steps.begin(), next_track_pos=track_pos+1;
       track_pos!=track_steps.end();
       track_pos++, prev_track_pos++, next_track_pos++) {
    if (track_pos == track_steps.begin()) prev_track_pos = track_pos;
    else if (next_track_pos == track_steps.end()) next_track_pos = track_pos;
    direction = (next_track_pos->vect() - prev_track_pos->vect()).Normalize();
    normals.push_back(Vector3D(direction.Y(), -direction.X(), 0.0));
    if (track_pos == track_steps.begin()) prev_track_pos--;
  }
  
  // Construct all road part outlines
  for (dist_start=0.0; dist_start<track_length; dist_start+=track_part_length) {
    road_part_points = new ObjectPoints();
    road_part_tops = new LineTopologies();
    point_no = 0;
    // First loop without overlap, second loop with overlap  
    for (use_overlap=0; use_overlap<2; use_overlap++) {
      if (use_overlap) {  
        dist_overlap[0] = dist_start - track_part_overlap / 2.0;
        dist_overlap[1] = dist_start + track_part_length + track_part_overlap / 2.0;
      }
      else {
        dist_overlap[0] = dist_start;
        dist_overlap[1] = dist_start + track_part_length;
      }
      if (dist_overlap[0] < 0.0) dist_overlap[0] = 0.0;
      if (dist_overlap[1] > track_length) dist_overlap[1] = track_length;

      // Get positions and normals at start and end of road part and 
      // determine indices of intermediate steps
      for (int i=0; i<2; i++) {
        if (dist_overlap[i] == 0.0) { // Start of track
          bound_pos[i] = track_steps[0];
          bound_normals[i] = normals[0];
          if (i == 0) index_start = 1;
          else return -4;
        }
        else if (dist_overlap[i] == track_length) { // End of track
          bound_pos[i] = *(track_steps.end()-1);
          bound_normals[i] = *(normals.end()-1);
          if (i == 1) index_end = track_steps.size() - 2;
          else return -5;
        }
        else { // Interpolate
          index_real = dist_overlap[i] / track_outline_step_size;
          index_int  = (int) index_real;
          index_fraction = index_real - (double) index_int;
          if (index_fraction < 0.001) {
            bound_pos[i] = track_steps[index_int];
            bound_normals[i] = normals[index_int];
            if (i == 0) index_start = index_int + 1;
            else index_end = index_int -1;
          }
          else if (index_fraction > 0.999) {
            bound_pos[i] = track_steps[index_int+1];
            bound_normals[i] = normals[index_int+1];
            if (i == 0) index_start = index_int + 2;
            else index_end = index_int;
          }
          else {
            bound_pos[i] = Position3D(track_steps[index_int] * (1.0 - index_fraction) +
                                      track_steps[index_int+1] * index_fraction);
            bound_normals[i] = normals[index_int] * (1.0 - index_fraction) +
                               normals[index_int+1] * index_fraction;
            if (i == 0) index_start = index_int + 1;
            else index_end = index_int;
          }
        }
      }
        
      // Create outline points
      num_pts = (2 + index_end - index_start + 1) * 2;
      road_part_point.NumberRef() = PointNumber(point_no); point_no++;
      road_part_point.vect() = bound_pos[0].vect() + 
                               bound_normals[0] * (track_width/2.0);
      road_part_points->push_back(road_part_point);
      for (index_int=index_start; index_int!= index_end+1; index_int++) {
        road_part_point.NumberRef() = PointNumber(point_no); point_no++;
        road_part_point.vect() = track_steps[index_int].vect() + 
                                 normals[index_int] * (track_width/2.0);
        road_part_points->push_back(road_part_point);
      }
      road_part_point.NumberRef() = PointNumber(point_no); point_no++;
      road_part_point.vect() = bound_pos[1].vect() + 
                               bound_normals[1] * (track_width/2.0);
      road_part_points->push_back(road_part_point);
      road_part_point.NumberRef() = PointNumber(point_no); point_no++;
      road_part_point.vect() = bound_pos[1].vect() - 
                               bound_normals[1] * (track_width/2.0);
      road_part_points->push_back(road_part_point);
      for (index_int=index_end; index_int!= index_start-1; index_int--) {
        road_part_point.NumberRef() = PointNumber(point_no); point_no++;
        road_part_point.vect() = track_steps[index_int].vect() - 
                                 normals[index_int] * (track_width/2.0);
        road_part_points->push_back(road_part_point);
      }
      road_part_point.NumberRef() = PointNumber(point_no); point_no++;
      road_part_point.vect() = bound_pos[0].vect() - 
                               bound_normals[0] * (track_width/2.0);
      road_part_points->push_back(road_part_point);
      // Create outline topology
      road_part_top.Erase();
      road_part_top.Number() = size() * 10 + use_overlap;
      point_no -= num_pts;
      for (int i=0; i<num_pts; i++, point_no++)
        road_part_top.push_back(PointNumber(point_no));
      road_part_top.push_back(*(road_part_top.begin())); // Close polygon
      road_part_tops->push_back(road_part_top);
      
      // Create centre line points
        // Intersection of track with road part start
      road_part_point.NumberRef() = PointNumber(point_no);
      point_no++; num_pts = 1;
      road_part_point.vect() = bound_pos[0].vect();
      road_part_points->push_back(road_part_point);
        // Search first track point inside road part
      track_pos = car_trajectory.begin(); 
      track_distance = track_distances.begin();
      while (*track_distance <= dist_overlap[0] &&
             track_pos != car_trajectory.end()) {
        track_distance++; track_pos++;
      }
      if (track_pos == car_trajectory.end()) return -6;
        // Store all track points inside road part
      while (*track_distance < dist_overlap[1] &&
             track_pos != car_trajectory.end()) {
        road_part_point.NumberRef() = PointNumber(point_no);
        point_no++; num_pts++;
        road_part_point.vect() = track_pos->vect();
        road_part_points->push_back(road_part_point);
        track_distance++; track_pos++;
      }
        // Intersection of track with road part end
      road_part_point.NumberRef() = PointNumber(point_no);
      point_no++; num_pts++;
      road_part_point.vect() = bound_pos[1].vect();
      road_part_points->push_back(road_part_point);
          
      // Create centre line topology
      road_part_top.Erase();
      road_part_top.Number() = size() * 10 + use_overlap + 2;
      point_no -= num_pts;
      for (int i=0; i<num_pts; i++, point_no++)
        road_part_top.push_back(PointNumber(point_no));
      road_part_tops->push_back(road_part_top);
    }
    // Remove double generated points
    road_part_points->RemoveDoublePoints(*road_part_tops, 0.001);
  
    // Create road part
    push_back(RoadPart(size(), road_part_points,
                       &*(road_part_tops->begin()),     // Outline without overlap
                       &*(road_part_tops->begin()+2),   // Outline with overlap
                       &*(road_part_tops->begin()+1),   // Cenrre line without overlap
                       &*(road_part_tops->begin()+3))); // Centre line with overlap
  }
  
  // Set meta data for all road parts
  for (road_part=begin(); road_part!=end(); road_part++) {
    road_part->DeriveName(name);
    road_part->DeriveMetaDataFileName(directory);
    road_part->DeriveOutlineFileNames(directory);  
  }
  
  return size();
}

/*
--------------------------------------------------------------------------------
                    Collect outlines of all road parts
--------------------------------------------------------------------------------
*/
void Road::CollectRoadPartOutlines(ObjectPoints &points, LineTopologies &tops,
                                   bool with_overlap) const
{
  Road::const_iterator         part;
  ObjectPoints                 part_points;
  ObjectPoints::const_iterator part_point;
  const LineTopology           *part_top;
  LineTopologies               part_tops;
  LineTopology::const_iterator node;
  
  // Delete old data
  if (points.size()) points.Erase();
  if (tops.size()) tops.Erase();
  
  for (part=begin(); part!=end(); part++) {
    if (with_overlap) part_top = part->OutlineOverlapTopology();
    else part_top = part->OutlineTopology();
    if (part_top && part->OutlinePoints()) { // Check if there is data
      // Copy relevant data from part
      for (node=part_top->begin(); node!=part_top->end(); node++) {
        part_point = part->OutlinePoints()->ConstPointIterator(*node);
        part_points.push_back(*part_point);
      }
      part_tops.push_back(*part_top);
      // Renumber copied data from part
      part_tops.ReNumber(part_points, points.size(), part_top->Number());
      // Add renumbered data
      for (part_point=part_points.begin(); part_point!=part_points.end(); part_point++)
        points.push_back(*part_point);
      tops.push_back(*(part_tops.begin()));
      // Clear local copies
      part_points.Erase();
      part_tops.Erase();
    }
  }
  // Remove double generated points
  points.RemoveDoublePoints(tops, 0.001);
}

/*
--------------------------------------------------------------------------------
               Reading and writing of outlines of all road parts
--------------------------------------------------------------------------------
*/

bool Road::ReadRoadPartOutlines()
{
  Road::iterator road_part;
  bool           ret_val=true;
  
  for (road_part=begin(); road_part!=end(); road_part++)
    if (!road_part->ReadOutlines()) ret_val = false;
    
  return ret_val;
}

bool Road::WriteRoadPartOutlines() const
{
  Road::const_iterator road_part;
  bool                 ret_val=true;
  
  for (road_part=begin(); road_part!=end(); road_part++)
    if (!road_part->WriteOutlines()) ret_val = false;
    
  return ret_val;
}

/*
--------------------------------------------------------------------------------
               Reading and writing of inventories of all road parts
--------------------------------------------------------------------------------
*/

bool Road::ReadRoadPartInventories()
{
  Road::iterator road_part;
  bool           ret_val=true;
  
  for (road_part=begin(); road_part!=end(); road_part++)
    if (!road_part->ReadInventory()) ret_val = false;
    
  return ret_val;
}

bool Road::WriteRoadPartInventories() const
{
  Road::const_iterator road_part;
  bool                 ret_val=true;
  
  for (road_part=begin(); road_part!=end(); road_part++)
    if (!road_part->WriteInventory()) ret_val = false;
    
  return ret_val;
}
