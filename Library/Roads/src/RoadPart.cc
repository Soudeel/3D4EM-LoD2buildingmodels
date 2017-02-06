
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



#include "RoadPart.h"
#include "LineSegment2D.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                                  Constructors
--------------------------------------------------------------------------------
*/

// Default constructor
RoadPart::RoadPart()
{
  num = -1;
  name = NULL;
  meta_file = NULL;
  outline_points_file = outline_topology_file = NULL;
  outline_points = NULL;
  outline_top = outline_overlap_top = NULL;
  centre_line_top = centre_line_overlap_top = NULL;
  inventory_points_file = inventory_topology_file = NULL;
  inventory_points = NULL;
  inventory_tops = NULL;
  laser_points_file = NULL;
}

// Construct from outline
RoadPart::RoadPart(int part_no, ObjectPoints *points,
                   LineTopology *top, LineTopology *overlap_top,
                   LineTopology *line_top, LineTopology *line_overlap_top)
{
  num                     = part_no;
  name                    = NULL;
  meta_file               = NULL;
  outline_points_file     = outline_topology_file = NULL;
  outline_points          = points;
  outline_top             = top;
  outline_overlap_top     = overlap_top;
  centre_line_top         = line_top;
  centre_line_overlap_top = line_overlap_top;
  inventory_points_file   = inventory_topology_file = NULL;
  inventory_points        = NULL;
  inventory_tops          = NULL;
  laser_points_file       = NULL;
}

// Re-initialisation
void RoadPart::ReInitialise()
{
  num = -1;
  name = NULL;
  meta_file = NULL;
  outline_points_file = outline_topology_file = NULL;
  outline_points = NULL; 
  outline_top = outline_overlap_top = NULL;
  centre_line_top = centre_line_overlap_top = NULL;
  inventory_points_file = inventory_topology_file = NULL;
  inventory_points = NULL; inventory_tops = NULL;
  if (tiles.size()) tiles.Erase();
  if (laser_points.size()) laser_points.ErasePoints();
  laser_points_file = NULL;
}

// Copy assignment
RoadPart & RoadPart::operator = (const RoadPart &part)
{
  if (this == &part) return *this;  // Check for self assignment
  num                     = part.num;
  StringCopy(&name, part.name);
  StringCopy(&meta_file, part.meta_file);
  StringCopy(&outline_points_file, part.outline_points_file);
  StringCopy(&outline_topology_file, part.outline_topology_file);
  outline_points          = part.outline_points;     // Only copy pointer!
  outline_top             = part.outline_top;        // Only copy pointer!
  outline_overlap_top     = part.outline_overlap_top;// Only copy pointer!
  centre_line_top         = part.centre_line_top;    // Only copy pointer!
  centre_line_overlap_top = part.centre_line_overlap_top; // Only copy pointer!
  StringCopy(&inventory_points_file, part.inventory_points_file);
  StringCopy(&inventory_topology_file, part.inventory_topology_file);
  inventory_points        = part.inventory_points; // Only copy pointer!
  inventory_tops          = part.inventory_tops;   // Only copy pointer!
  tiles                   = part.tiles;
  StringCopy(&laser_points_file, part.laser_points_file);
  laser_points            = part.laser_points;
  return *this;
}

/*
--------------------------------------------------------------------------------
                 Derive names of the road part and various files
--------------------------------------------------------------------------------
*/

char * RoadPart::DeriveName(const char *road_name)
{
  int  i;
  char *ch;

  if (name) free(name);
  name = (char *) malloc(strlen(road_name) + 7);
  sprintf(name, "%s_%5d", road_name, num);
  for (i=strlen(road_name), ch=name+i; i<strlen(name); i++, ch++)
   if (*ch == ' ') *ch = '0';
  return(name);
}

char * RoadPart::DeriveMetaDataFileName(const char *directory)
{
  if (meta_file) free(meta_file);
  meta_file = ComposeFileName(directory, name, ".roadpart");
  return meta_file;
}

void RoadPart::DeriveOutlineFileNames(const char *directory)
{
  char *outline_name;
  
  outline_name = (char *) malloc(strlen(name) + 9);
  sprintf(outline_name, "%s_outline", name);
  if (outline_points_file) free(outline_points_file);
  outline_points_file = ComposeFileName(directory, outline_name, ".objpts");
  if (outline_topology_file) free(outline_topology_file);
  outline_topology_file = ComposeFileName(directory, outline_name, ".top");
  free(outline_name);
}

void RoadPart::DeriveInventoryFileNames(const char *directory)
{
  char *inventory_name;
  
  inventory_name = (char *) malloc(strlen(name) + 11);
  sprintf(inventory_name, "%s_inventory", name);
  if (inventory_points_file) free(inventory_points_file);
  inventory_points_file = ComposeFileName(directory, inventory_name, ".objpts");
  if (inventory_topology_file) free(inventory_topology_file);
  inventory_topology_file = ComposeFileName(directory, inventory_name, ".top");
  free(inventory_name);  
}

char * RoadPart::DeriveLaserPointsFileName(const char *directory)
{
  if (laser_points_file) free(laser_points_file);
  laser_points_file = ComposeFileName(directory, name, ".laser");
  return laser_points_file;
}

/*
--------------------------------------------------------------------------------
                       Read the road part meta data
--------------------------------------------------------------------------------
*/
bool RoadPart::ReadMetaData(const char *filename)
{
  FILE                  *fd;
  char                  *buffer, *line, *keyword;
  int                   keyword_length, success, number;
  LaserSubUnit          *tile;

  // Open the file
  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening road part database %s\n", filename);
    return false;
  }
  
  // Verify that the first keyword is "roadpart"
  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading road part database %s\n", filename);
    fclose(fd);
    return false;
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "roadpart", max(keyword_length, 8))) {
    fprintf(stderr, "Error: File %s is not a road part database.\n", filename);
    fprintf(stderr, "       First keyword is %s, not roadpart.\n",
            keyword);
  }

  // Initialise all meta data and delete old data
  ReInitialise();

  // Store meta data file name
  if (meta_file) free(meta_file);
  StringCopy(&meta_file, filename);
  
  // Process all lines
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "number", max(keyword_length, 6)))
          num = BNF_Integer(line);

        else if (!strncmp(keyword, "name", max(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "metadata", max(keyword_length, 8)))
          meta_file = BNF_String(line);

        else if (!strncmp(keyword, "outline_points", max(keyword_length, 14)))
          outline_points_file = BNF_String(line);

        else if (!strncmp(keyword, "outline_topology", max(keyword_length, 16)))
          outline_topology_file = BNF_String(line);

        else if (!strncmp(keyword, "inventory_points", max(keyword_length, 16)))
          inventory_points_file = BNF_String(line);

        else if (!strncmp(keyword, "inventory_topology", max(keyword_length, 18)))
          inventory_topology_file = BNF_String(line);

        else if (!strncmp(keyword, "tile", max(keyword_length, 4))) {
          tile = new LaserSubUnit();
          tile->ReadMetaData(BNF_String(line));
          tiles.push_back(tile);
        }
        
        else if (!strncmp(keyword, "laser_points", max(keyword_length, 12)))
          laser_points_file = BNF_String(line);

        else if (!strncmp(keyword, "endroadpart", max(keyword_length,11))) {
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
  fprintf(stderr, "Error: Did not find keyword endroadpart.\n");
  free(buffer);
  fclose(fd);
  return false;
}

/*
--------------------------------------------------------------------------------
                   Write the road part meta data to file
--------------------------------------------------------------------------------
*/

bool RoadPart::WriteMetaData() const
{
  FILE       *fd;
  int        indent;
  LaserTilePtrVector::const_iterator tile;

  // Open the file
  fd = fopen(meta_file, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening road part database %s\n", meta_file);
    return false;
  }

  // Write the meta data
  indent = 0;
  BNF_Write_String(fd, "roadpart", indent, NULL);  indent += 2;
  if (num >= 0) BNF_Write_Integer(fd, "number", indent, num, "%d");
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (meta_file) BNF_Write_String(fd, "metadata", indent, meta_file);
  if (outline_points_file)
    BNF_Write_String(fd, "outline_points", indent, outline_points_file);
  if (outline_topology_file)
    BNF_Write_String(fd, "outline_topology", indent, outline_topology_file);
  if (inventory_points_file)
    BNF_Write_String(fd, "inventory_points", indent, inventory_points_file);
  if (inventory_topology_file)
    BNF_Write_String(fd, "inventory_topology", indent, inventory_topology_file);
  for (tile=tiles.begin(); tile!=tiles.end(); tile++)
    BNF_Write_String(fd, "tile", indent, (*tile)->MetaDataFile());
  if (laser_points_file)
    BNF_Write_String(fd, "laser_points", indent, laser_points_file);
  indent -= 2;
  BNF_Write_String(fd, "endroadpart", 0, NULL);

  // Close the file and return success
  fclose(fd);
  return true;
}

/*
--------------------------------------------------------------------------------
      Read and write the outline and centre line (with and without overlap)
--------------------------------------------------------------------------------
*/

bool RoadPart::ReadOutlines()
{
  LineTopologies           *tops;
  LineTopologies::iterator top;
  int                      part_number;
  
  // Check availability of file names
  if (!outline_points_file) {
    printf("Error: File name of outline points not defined by DeriveOutlineFileNames\n");
    return false;
  }
  if (!outline_topology_file) {
    printf("Error: File name of outline topology not defined by DeriveOutlineFileNames\n");
    return false;
  }

  // Create new points and topologies and delete old contents
  if (!outline_points) outline_points = new ObjectPoints();
  else if (outline_points->size()) outline_points->Erase();
  if (!outline_top) outline_top = new LineTopology();
  else if (outline_top->size()) outline_top->Erase();
  if (!outline_overlap_top) outline_overlap_top = new LineTopology();
  else if (outline_overlap_top->size()) outline_overlap_top->Erase();
  if (!centre_line_top) centre_line_top = new LineTopology();
  else if (centre_line_top->size()) centre_line_top->Erase();
  if (!centre_line_overlap_top) centre_line_overlap_top = new LineTopology();
  else if (centre_line_overlap_top->size()) centre_line_overlap_top->Erase();

  // Read the points
  if (!outline_points->Read(outline_points_file)) {
    printf("Error reading outline points from file %s\n", outline_points_file);
    return false;
  }
  if (outline_points->empty()) {
    printf("Error: No points in file %s\n", outline_points_file);
    return false;
  }
  
  // Read the topologies
  tops = new LineTopologies();
  if (!tops->Read(outline_topology_file)) {
    printf("Error reading outline topologies from file %s\n", outline_topology_file);
    return false;
  }
  if (tops->size() != 4) {
    printf("Warning: Expected 4 lines in file %s\n", outline_topology_file);
  }

  // Distribute the various topologies over the variables
  for (top=tops->begin(); top!=tops->end(); top++) {
    part_number = top->Number() / 10;
    if (part_number != num) {
      printf("Error: Outline number (%d) does not match with part number (%d)\n",
             part_number, num);
      return false;
    }
    switch (top->Number() - part_number * 10) {
      case 0 : *outline_top = *top; break;
      case 1 : *outline_overlap_top = *top; break;
      case 2 : *centre_line_top = *top; break;
      case 3 : *centre_line_overlap_top = *top; break;
      default: printf("Error: Undefined topology number %d\n", top->Number());
               return false;
    }
  }

  // Delete local topology container and return success
  tops->Erase();
  delete tops;
  return true;
}

bool RoadPart::WriteOutlines() const
{
  LineTopologies *tops;
  
  // Check availability of file names
  if (!outline_points_file) {
    printf("Error: File name of outline points not defined by DeriveOutlineFileNames\n");
    return false;
  }
  if (!outline_topology_file) {
    printf("Error: File name of outline topology not defined by DeriveOutlineFileNames\n");
    return false;
  }
  
  // Write the outline points
  if (!outline_points->Write(outline_points_file)) {
    printf("Error writing outline points to file %s\n", outline_points_file);
    return false;
  }
  
  // Collect the outline topologies
  tops = new LineTopologies();
  tops->push_back(*outline_top);
  tops->push_back(*outline_overlap_top);
  tops->push_back(*centre_line_top);
  tops->push_back(*centre_line_overlap_top);
  if (!tops->Write(outline_topology_file)) {
    printf("Error writing outline topologies to file %s\n", outline_topology_file);
    return false;
  }
  
  // Erase local container and return success
  tops->Erase();
  delete tops;
  return true;
}

/*
--------------------------------------------------------------------------------
      Read and write the road part inventory data
--------------------------------------------------------------------------------
*/

bool RoadPart::ReadInventory()
{
  // Check availability of file names
  if (!inventory_points_file) {
    printf("Error: File name of inventory points not defined by DeriveInventoryFileNames\n");
    return false;
  }
  if (!inventory_topology_file) {
    printf("Error: File name of inventory topology not defined by DeriveInventoryFileNames\n");
    return false;
  }

  // Create new points and topologies and delete old contents
  if (!inventory_points) inventory_points = new ObjectPoints();
  else if (inventory_points->size()) inventory_points->Erase();
  if (!inventory_tops) inventory_tops = new LineTopologies();  
  else if (inventory_tops->size()) inventory_tops->Erase();

  // Read the points
  if (!inventory_points->Read(inventory_points_file)) {
    printf("Error reading inventory points from file %s\n", inventory_points_file);
    return false;
  }
  
  // Read the topologies
  if (!inventory_tops->Read(inventory_topology_file)) {
    printf("Error reading inventory topologies from file %s\n", inventory_topology_file);
    return false;
  }
  return true;
}

bool RoadPart::WriteInventory() const
{
  // Check availability of file names
  if (!inventory_points_file) {
    printf("Error: File name of inventory points not defined by DeriveInventoryFileNames\n");
    return false;
  }
  if (!inventory_topology_file) {
    printf("Error: File name of inventory topology not defined by DeriveInventoryFileNames\n");
    return false;
  }
  
  // Write the inventory points
  if (!inventory_points->Write(inventory_points_file)) {
    printf("Error writing inventory points to file %s\n", inventory_points_file);
    return false;
  }

  // Write the inventory topology  
  if (!inventory_tops->Write(inventory_topology_file)) {
    printf("Error writing inventory topologies to file %s\n", inventory_topology_file);
    return false;
  }
  return true;
}


/*
--------------------------------------------------------------------------------
                   Collect all tiles of a road part
--------------------------------------------------------------------------------
*/

int RoadPart::CollectTiles(LaserBlock &block, bool include_overlap)
{
  LaserTilePtrVector::iterator tile;
  DataBounds2D        part_bounds;
  
  // Check required input data
  if (block.empty()) {
    printf("Error: Block %s has 0 tiles\n", block.Name());
    return -2;
  }
  if (block.begin()->empty()) {
    printf("Error: Block %s has 0 tiles\n", block.Name());
    return -2;
  }
  if (block.begin()->begin()->DataOrganisation() != TileWise) {
    printf("Error: Block %s does not contain tiles\n", block.Name());
    return -1;
  }
  if (outline_points == NULL || outline_top == NULL) {
    printf("Error: No road part outline defined\n");
    return -3;
  }
  if (!outline_top->IsClosed()) {
    printf("Error: Road part outline is not a closed polygon\n");
    return -4;
  }
  
  // Get rectangular bounds of the road part
  part_bounds = outline_top->Bounds(*outline_points).Bounds2D();
  
  // Get all tiles within this rectangle
  tiles = block.SelectTiles(part_bounds);
  if (tiles.empty()) return 0;
  
  // Verify if tiles are within or overlapping with the road part  
  for (tile=tiles.begin(); tile!=tiles.end(); tile++) {
    if (include_overlap) {
      if (!outline_overlap_top->Overlap(*outline_points, (*tile)->TileBounds().Bounds2D(), true)) {
        tiles.erase(tile);
        tile--;
      }
    }
    else {
      if (!outline_top->Overlap(*outline_points, (*tile)->TileBounds().Bounds2D(), true)) {
        tiles.erase(tile);
        tile--;
      }
    }
  }
  
  return (int) tiles.size();
}

/*
--------------------------------------------------------------------------------
                   Collect points within road part boundary
--------------------------------------------------------------------------------
*/

int RoadPart::CollectLaserPoints(bool include_overlap, bool clip_at_bounds)
{
  LaserTilePtrVector::iterator tile;
  LineTopologies               outline_tops;
  LineTopology                 *top;
  
  // Check if there are already tiles selected for this road part
  if (tiles.empty()) return -1;
  
  // Clear old data
  if (laser_points.size()) laser_points.ErasePoints();
  
  // Select the right outline
  if (include_overlap) {
    if (outline_overlap_top == NULL) return -2;
    top = outline_overlap_top;
  }
  else {
    if (outline_top == NULL) return -3;
    top = outline_top;
  }
  
  // Store the outline topology in the container required for the select function
  if (clip_at_bounds) outline_tops.push_back(*top);
  
  // Loop over all tiles
  for (tile=tiles.begin(); tile!=tiles.end(); tile++) {
    // If no points from the overlap are required, there may be tiles that
    // are completely outside the road part outline. These should not be read.
    if (!include_overlap) 
      if (!top->Overlap(*outline_points, (*tile)->TileBounds().Bounds2D(),
                        false)) continue;
    // Read tile points
    if (!(*tile)->Read((*tile)->PointFile(), false)) return -4;
    // Select all points of the tile or just those inside the outline
    if (clip_at_bounds)
      // Select data within polygon
      (*tile)->Select(laser_points, *outline_points, outline_tops);
    else
      // Select all tile points
      laser_points.insert(laser_points.end(), (*tile)->begin(), (*tile)->end());
    // Erase tile points
    (*tile)->ErasePoints();
  }
  
  // Erase local container
  if (clip_at_bounds) outline_tops.Erase();
  
  // Return number of collected points
  return (int) laser_points.size();
}
