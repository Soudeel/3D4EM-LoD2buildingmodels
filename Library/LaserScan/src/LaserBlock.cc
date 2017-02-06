
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class LaserBlock

 Initial creation
 Author : George Vosselman
 Date   : 30-03-1999

 Update #1
 Author : 
 Date   : 
 Changes: 

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "LaserBlock.h"
#include "ObjectPoints2D.h"
#include "LineTopologies.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                       Declarations of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char*);
extern "C" char *get_full_filename(const char *, const char *, int *);

/*
--------------------------------------------------------------------------------
           Construct a block out of a file filter of laser point files
--------------------------------------------------------------------------------
*/

LaserBlock::LaserBlock(const char *long_filter)
{
  char       *directory, *filter, *filename;
  int        icon;

  Initialise();
  filter = NULL;
  StringCopy(&filter, long_filter);
  directory = (char *) malloc(MAXCHARS);
  parse_filter(&filter, directory);
  icon = 0;
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    push_back(LaserUnit(filename));         /* Add strip or tile to the block */
  }
  free(directory);
  if (!empty()) scanner = begin()->Scanner(); // Copy unit scanner information
}

/*
--------------------------------------------------------------------------------
                             Copy assignment
--------------------------------------------------------------------------------
*/

LaserBlock & LaserBlock::operator = (const LaserBlock &block)
{
  // Check for self assignment
  if (this == &block) return *this;
  if (!empty()) erase(begin(), end());
  if (!block.empty()) insert(begin(), block.begin(), block.end());
  LaserPointsInfoReference() = block.LaserPointsInfoReference();
  LaserMetaFileReference()   = block.LaserMetaFileReference();
  StringCopy(&control_points_file, block.control_points_file);
  control_points = block.control_points;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise the meta data
--------------------------------------------------------------------------------
*/

void LaserBlock::Initialise()
{
  LaserPointsInfo::Initialise();
  meta_file = name = control_points_file = NULL;
}

void LaserBlock::ReInitialise()
{
  if (!empty()) erase(begin(), end());          /* Delete old strips or tiles */
  LaserPointsInfo::ReInitialise();
  if (meta_file) free(meta_file);         meta_file    = NULL;
  if (name) free(name);                   name         = NULL;
  if (control_points_file) free(control_points_file);
  control_points_file = NULL;
  if (!control_points.empty())
    control_points.erase(control_points.begin(), control_points.end());
}

/*
--------------------------------------------------------------------------------
         Create a block from an arbitrary laser meta file or point file
--------------------------------------------------------------------------------
*/

int LaserBlock::Create(const char *filename, int *fileclass,
                       bool read_units, bool read_subunits)
{
  int success;

  ReInitialise();
  *fileclass = BNF_LaserFileClass(filename);
  switch (*fileclass) {
    case LASER_BLOCK:
      success = ReadMetaData(filename, read_units, read_subunits);
      break;

    case LASER_STRIP:
      push_back(LaserUnit(filename, &success, read_subunits));
      break;

    case LASER_SUB_UNIT:
      push_back(LaserUnit(LaserSubUnit(filename, &success)));
      break;

    case LASER_POINT_SET:
      push_back(LaserUnit(LaserSubUnit(LaserPoints(filename, &success))));
      break;

    case LASER_RAW_DATA:
      push_back(LaserUnit(LaserSubUnit(LaserPoints(filename))));
      success = 1;
      break;

    default:
      fprintf(stderr, "Error: Invalid data class (%d) of file %s\n",
	          *fileclass, filename);
      exit(0);
  }
  return(success);
}

/*
--------------------------------------------------------------------------------
                       Read the block meta data
--------------------------------------------------------------------------------
*/


int LaserBlock::ReadMetaData(const char *filename, bool read_units,
                             bool read_subunits)
{
  FILE                  *fd, *fd_tiles;
  char                  *buffer, *line, *keyword, *file;
  int                   keyword_length, success, number;
  LaserDataOrganisation data_org;
  LaserBlock::iterator  tile_container, strip;

/* Open the file */

  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening block database %s\n", filename);
    return(0);
  }

/* Verify that the first keyword is "laserblock" */

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading block database %s\n", filename);
    fclose(fd);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "laserblock", MAX(keyword_length, 10))) {
    fprintf(stderr, "Error: File %s is not a block database.\n", filename);
    fprintf(stderr, "       First keyword is %s, not laserblock.\n",
	    keyword);
  }

/* Initialise all meta data */

  ReInitialise();

/* Process all lines */

  data_org = UnknownOrganisation;
  SetMetaDataFile(filename);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "info", MAX(keyword_length, 4)))
          LaserPointsInfo::ReadMetaData(fd);

        else if (!strncmp(keyword, "control_points", MAX(keyword_length, 14)))
          control_points_file = BNF_String(line);

        else if (!strncmp(keyword, "strip", MAX(keyword_length, 5))) {
          if (data_org == UnknownOrganisation) data_org = StripWise;
          else if (data_org != StripWise) {
            fprintf(stderr, "Error: A mix of strips and block tiles is not possible!\n");
            fprintf(stderr, "Error encountered in file %s\n", filename);
            exit(0);
          }
          file = BNF_String(line);
          if (file) {
            if (read_units) {
              push_back(LaserUnit(file, &success, read_subunits));
              if (!success) erase(end()-1);
            }
            else { // Only store file name in a further empty unit
              push_back(LaserUnit());
              (end()-1)->SetMetaDataFile(file);
            }
          }
        }

        else if (!strncmp(keyword, "tile", MAX(keyword_length, 4))) {
          if (data_org == UnknownOrganisation) data_org = TileWise;
          else if (data_org != TileWise) {
            fprintf(stderr, "Error: A mix of strips and block tiles is not possible!\n");
            fprintf(stderr, "Error encountered in file %s\n", filename);
            exit(0);
          }
          if (size() == 0) {
            push_back(LaserUnit());
            tile_container = begin();
            tile_container->DataOrganisation() = TileWise;
          }
          file = BNF_String(line);
          if (file) {
            tile_container->push_back(LaserSubUnit(file, &success));
            if (!success) tile_container->erase(tile_container->end()-1);
          }
        }

        // Meta data of multiple tiles in one file (useful for large blocks)
        else if (!strncmp(keyword, "tiles", MAX(keyword_length, 5))) {
          if (data_org == UnknownOrganisation) data_org = TileWise;
          else if (data_org != TileWise) {
            fprintf(stderr, "Error: A mix of strips and block tiles is not possible!\n");
            fprintf(stderr, "Error encountered in file %s\n", filename);
            exit(0);
          }
          if (size() == 0) {
            push_back(LaserUnit());
            tile_container = begin();
            tile_container->DataOrganisation() = TileWise;
          }
          file = BNF_String(line);
          if (file) {
            fd_tiles = fopen(file, "r");
            if (fd_tiles == NULL) {
              fprintf(stderr, "Error opening tiles file %s\n", file);
              exit(0);
            }
            while ((line = fgets(buffer, MAXCHARS, fd_tiles))) {
              if (!Is_Comment(line)) {
                keyword = BNF_KeyWord(line, &keyword_length);
                if (keyword) {
                  if (!strncmp(keyword, "lasertile", MAX(keyword_length, 9))) {
                    tile_container->push_back(LaserSubUnit(fd_tiles, TileWise,
                                                           &success));
                    if (!success) tile_container->erase(tile_container->end()-1);
                  }
                }
              }
            }
            fclose(fd_tiles);
          }
        }

        else if (!strncmp(keyword, "endlaserblock", MAX(keyword_length,13))) {
          if (data_org == StripWise) // Set the strip numbers
            for (strip=begin(), number=0; strip!=end(); strip++, number++)
              strip->StripNumber() = number;
          else if (data_org == UnknownOrganisation) 
            fprintf(stderr, "Warning: block meta data file %s contains no strips or tiles!\n", filename);
          free(buffer);
          fclose(fd);
          return(1);
        }

	else {
	  keyword[keyword_length] = 0;
	  fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
	}
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserblock.\n");
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the block meta data to file
--------------------------------------------------------------------------------
*/

int LaserBlock::WriteMetaData() const
{
  return(WriteMetaData(meta_file, 0, 0));
}

int LaserBlock::WriteMetaData(const char *filename) const
{
  return(WriteMetaData(filename, 0, 0));
}

int LaserBlock::WriteMetaData(int write_units, int write_sub_units) const
{
  return(WriteMetaData(meta_file, write_units, write_sub_units));
}

int LaserBlock::WriteMetaData(const char *filename, int write_units,
			      int write_sub_units) const
{
  FILE       *fd;
  int        indent;
  LaserBlock::const_iterator unit;
  LaserUnit::const_iterator tile;

/* Open the file */

  if (!filename) return(0); /* Silently return */
  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening block database %s\n", filename);
    return(0);
  }

/* Write the meta data */

  indent = 0;
  BNF_Write_String(fd, "laserblock", indent, NULL);  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (control_points_file)
    BNF_Write_String(fd, "control_points", indent, control_points_file);
  LaserPointsInfo::WriteMetaData(fd, indent);
  for (unit=begin();
       unit!=end();
       unit++) {
    if (unit->DataOrganisation() & StripWise)
      BNF_Write_String(fd, "strip", indent, unit->MetaDataFile());
    else /* TileWise */
      for (tile=unit->begin(); tile!=unit->end(); tile++) 
        BNF_Write_String(fd, "tile", indent, tile->MetaDataFile());
    if (write_units) (void) unit->WriteMetaData(write_sub_units);
  }
  indent -= 2;
  BNF_Write_String(fd, "endlaserblock", 0, NULL);

/* Close the file and return */

  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Derive meta data file name
--------------------------------------------------------------------------------
*/

char *LaserBlock::DeriveMetaDataFileName(const char *directory)
{
  return(LaserMetaFile::DeriveMetaDataFileName(directory, ".block"));
}

/*
--------------------------------------------------------------------------------
                    Add data of laser points to an image
--------------------------------------------------------------------------------
*/

void LaserBlock::ImageData(Image &image, Image &dist, const DataGrid &grid,
                           ImagedDataType datatype, int interpolation_method,
                           double max_mesh_size)
{
  LaserBlock::iterator unit;

/* Collect data in all units */

  for (unit=begin(); unit!=end(); unit++) {
    unit->ImageData(image, dist, grid, datatype, interpolation_method,
                    max_mesh_size);
  }
}

/*
--------------------------------------------------------------------------------
                   Derive the data bounds of the block
--------------------------------------------------------------------------------
*/

DataBoundsLaser LaserBlock::DeriveDataBounds(int use_known_bounds)
{
  LaserBlock::iterator unit;

/* Collect the bounds by a loop over the units */

  bounds.Initialise();
  for (unit=begin(); unit!=end(); unit++)
    bounds.Update(unit->DeriveDataBounds(use_known_bounds));

  return(bounds);
}

/*
--------------------------------------------------------------------------------
                   Write boundaries of tiles
--------------------------------------------------------------------------------
*/

int LaserBlock::WriteTileBoundaries(const char *corner_file,
                                    const char *topology_file) const
{
  ObjectPoints2D corners;
  LineTopologies topology;
  LaserBlock::const_iterator unit;
  LaserUnit::const_iterator subunit;

/* Extract the corners and the topology from the tiles */

  for (unit=begin(); unit!=end(); unit++) {
    for (subunit=unit->begin(); subunit!=unit->end(); subunit++) {
      if (!(subunit->DataOrganisation() & TileWise)) {
        fprintf(stderr,"Error: No tile boundaries if subunits are no tiles!\n");
        exit(0);
      }
      subunit->AddTileBoundary(corners, topology);
    }
  }
  
/* Write the corners and the topology */

  if (corner_file) {
    if (!corners.Write(corner_file)) {
      fprintf(stderr, "Error writing tile corners to file %s\n", corner_file);
      return(0);
    }
  }
  if (topology_file) {
    if (!topology.Write(topology_file, false)) {
      fprintf(stderr,"Error writing tile topology to file %s\n", topology_file);
      return(0);
    }
  }
  return(1);
}

// Collect tile boundaries
void LaserBlock::CollectTileBoundaries(ObjectPoints2D &corners,
                                       LineTopologies &topology,
                                       const LaserTilePtrVector &tiles) const
{
  LaserTilePtrVector::const_iterator tile;
  
  for (tile=tiles.begin(); tile!=tiles.end(); tile++)
    (*tile)->AddTileBoundary(corners, topology);
}

void LaserBlock::CollectTileBoundaries(ObjectPoints2D &corners,
                                       LineTopologies &topology,
                                       const DataBounds2D &area_bounds)
{
  LaserTilePtrVector tiles = SelectTiles(area_bounds);
  LaserTilePtrVector::iterator tile;
  
  for (tile=tiles.begin(); tile!=tiles.end(); tile++)
    (*tile)->AddTileBoundary(corners, topology);
  
  tiles.erase(tiles.begin(), tiles.end());
}

/*
--------------------------------------------------------------------------------
                           Erase all points
--------------------------------------------------------------------------------
*/

void LaserBlock::ErasePoints()
{
  LaserBlock::iterator unit;

  for (unit=begin(); unit!=end(); unit++) unit->ErasePoints();
}

/*
--------------------------------------------------------------------------------
                      Read all points up to a maximum
--------------------------------------------------------------------------------
*/

int LaserBlock::ReadPoints(int max_num_pts)
{
  LaserBlock::iterator unit;
  int                  num_read=0, num_read_unit;

  for (unit=begin(); unit!=end() && num_read <= max_num_pts; unit++) {
    num_read_unit = unit->ReadPoints(max_num_pts - num_read);
    if (num_read_unit == -1) num_read = max_num_pts + 1;
    else num_read += num_read_unit;
  }

  if (num_read > max_num_pts) {
    ErasePoints();
    return -1;
  }
  return num_read;
}

/*
--------------------------------------------------------------------------------
                Determine number of tile rows and columns
--------------------------------------------------------------------------------
*/

int LaserBlock::NumberOfTileRows() const
{
  if (empty()) return 0;
  return begin()->NumberOfTileRows();
}

int LaserBlock::NumberOfTileColumns() const
{
  if (empty()) return 0;
  return begin()->NumberOfTileColumns();
}

/*
--------------------------------------------------------------------------------
                Return a specific block tile
--------------------------------------------------------------------------------
*/

LaserSubUnit * LaserBlock::Tile(int tile_row, int tile_column)
{
  LaserUnit::iterator  tile, first_tile, last_tile;
  LaserBlock::iterator tiles;
  int                  tile_range;
  
  if (empty()) return NULL;
  tiles = begin();
  if (tiles->empty()) return NULL;
  if (tiles->DataOrganisation() != TileWise) {
    printf("Warning: block is not organised in tiles.\n");
    return NULL;
  }
  
  first_tile = tiles->begin();
  if (first_tile->CompareTileLocation(tile_row, tile_column) == 0)
    return &*first_tile;
  last_tile = tiles->end() - 1;
  if (last_tile->CompareTileLocation(tile_row, tile_column) == 0)
    return &*last_tile;
  tile_range = ((int) &*last_tile - (int) &*first_tile) / 
               sizeof(LaserSubUnit);
  while (tile_range > 1) {
    tile = first_tile + tile_range / 2;
    switch (tile->CompareTileLocation(tile_row, tile_column)) {
      case -1: // This tile comes earlier; look further on
        first_tile = tile; break;
      case 0:  // Found the tile
        return &*tile;
      case 1: // This tile comes later; look earlier
        last_tile = tile; break;
    }
    tile_range = ((int) &*last_tile - (int) &*first_tile) / 
                 sizeof(LaserSubUnit);
  }
  return NULL;
}

// Const version
const LaserSubUnit * LaserBlock::Tile(int tile_row, int tile_column) const
{
  LaserUnit::const_iterator  tile, first_tile, last_tile;
  LaserBlock::const_iterator tiles;
  int                        tile_range;
  
  if (empty()) return NULL;
  tiles = begin();
  if (tiles->empty()) return NULL;
  if (tiles->DataOrganisation() != TileWise) {
    printf("Warning: block is not organised in tiles.\n");
    return NULL;
  }
  
  first_tile = tiles->begin();
  if (first_tile->CompareTileLocation(tile_row, tile_column) == 0)
    return &*first_tile;
  last_tile = tiles->end() - 1;
  if (last_tile->CompareTileLocation(tile_row, tile_column) == 0)
    return &*last_tile;
  tile_range = ((int) &*last_tile - (int) &*first_tile) / 
               sizeof(LaserSubUnit);
  while (tile_range > 1) {
    tile = first_tile + tile_range / 2;
    switch (tile->CompareTileLocation(tile_row, tile_column)) {
      case -1: // This tile comes earlier; look further on
        first_tile = tile; break;
      case 0:  // Found the tile
        return &*tile;
      case 1: // This tile comes later; look earlier
        last_tile = tile; break;
    }
    tile_range = ((int) &*last_tile - (int) &*first_tile) / 
                 sizeof(LaserSubUnit);
  }
  return NULL;
}

LaserSubUnit * LaserBlock::Tile(const Position2D &pos)
{
  LaserBlock::iterator tiles;
  LaserUnit::iterator  tile;
  double               min_X, max_Y;
  int                  tile_row, tile_column;
  
  // Get a valid tile
  if (empty()) return NULL;
  tiles = begin();
  if (tiles->empty()) return NULL;
  tile = tiles->begin();
  // Retrieve the upper left coordinates of the block
  min_X = tile->TileBounds().Minimum().X() - 
          tile->TileColumn() * tile->TileBounds().XRange();
  max_Y = tile->TileBounds().Maximum().Y() +
          tile->TileRow() * tile->TileBounds().YRange();
  // Determine the tile index of the given point
  tile_row = (int) ((max_Y - pos.Y()) / tile->TileBounds().YRange());
  tile_column = (int) ((pos.X() - min_X) / tile->TileBounds().XRange());
  // Retrieve the tile by tile coordinates
  return Tile(tile_row, tile_column);
}

int LaserBlock::TileRow(double Y) const
{
  LaserBlock::const_iterator tiles;
  LaserUnit::const_iterator  tile;
  double                     max_Y;
  int                        tile_row;
  
  // Get a valid tile
  if (empty()) return -1;
  tiles = begin();
  if (tiles->empty()) return -1;
  tile = tiles->begin();
  // Retrieve the maximum Y coordinate of the block
  max_Y = tile->TileBounds().Maximum().Y() +
          tile->TileRow() * tile->TileBounds().YRange();
  // Determine the tile row of the given point
  tile_row = (int) ((max_Y - Y) / tile->TileBounds().YRange());
  return tile_row;
}

int LaserBlock::TileColumn(double X) const
{
  LaserBlock::const_iterator tiles;
  LaserUnit::const_iterator  tile;
  double                     min_X;
  int                        tile_column;
  
  // Get a valid tile
  if (empty()) return -1;
  tiles = begin();
  if (tiles->empty()) return -1;
  tile = tiles->begin();
  // Retrieve the maximum Y coordinate of the block
  min_X = tile->TileBounds().Minimum().X() - 
          tile->TileColumn() * tile->TileBounds().XRange();
  // Determine the tile column of the given point
  tile_column = (int) ((X - min_X) / tile->TileBounds().XRange());
  return tile_column;
}

/*
--------------------------------------------------------------------------------
                Select all tiles within a rectangular area
--------------------------------------------------------------------------------
*/

LaserTilePtrVector & LaserBlock::SelectTiles(const DataBounds2D &area_bounds)
{
  LaserTilePtrVector *tile_list;
  LaserSubUnit       *tile;
  int                row, col, row_min, row_max, col_min, col_max;
  
  tile_list = new LaserTilePtrVector();
  
  // Get the minimum and maximum tile indices
  row_min = MAX(0, TileRow(area_bounds.Maximum().Y()));
  row_max = MIN(NumberOfTileRows()-1, TileRow(area_bounds.Minimum().Y()));
  col_min = MAX(0, TileColumn(area_bounds.Minimum().X()));
  col_max = MIN(NumberOfTileColumns()-1, TileColumn(area_bounds.Maximum().X()));
  // Collect all tiles in this area
  for (row=row_min; row!=row_max+1; row++) {
    for (col=col_min; col!=col_max+1; col++) {
      tile = Tile(row, col);
      if (tile != NULL) tile_list->push_back(tile);
    }
  }
  
  return *tile_list;
}

/* 
--------------------------------------------------------------------------------
                Select all data within a rectangular area
--------------------------------------------------------------------------------
*/

void LaserBlock::SelectPoints(const DataBounds2D &area_bounds,
                              LaserPoints &selection)
{
  int             row, col, row_min, row_max, col_min, col_max;
  LaserSubUnit    *tile;
  DataBoundsLaser selection_bounds = DataBoundsLaser(area_bounds);

  // Get the minimum and maximum tile indices
  row_min = MAX(0, TileRow(area_bounds.Maximum().Y()));
  row_max = MIN(NumberOfTileRows()-1, TileRow(area_bounds.Minimum().Y()));
  col_min = MAX(0, TileColumn(area_bounds.Minimum().X()));
  col_max = MIN(NumberOfTileColumns()-1, TileColumn(area_bounds.Maximum().X()));
  
  // Collect points of all tiles in this area
  for (row=row_min; row!=row_max+1; row++) {
    for (col=col_min; col!=col_max+1; col++) {
      tile = Tile(row, col);
      if (tile != NULL) {
        tile->Read(tile->PointFile(), false);
        tile->Select(selection, selection_bounds);
        tile->ErasePoints();
      }
    }
  }
}

// TODO check if reading in the two SelectPoints functions
// can be faster with merged laser files

void LaserBlock::SelectPoints(LaserPoints &selection,
                              const Line2D &line, double max_dist,
                              double scalar_min, double scalar_max)
{
  DataBounds2D bounds;
  int          row, col, row_min, row_max, col_min, col_max;
  LaserSubUnit *tile;
  bool         verbose=false;
  
  // Determine the bounding box of the rotated rectangle
  bounds = line.BoundingBox(scalar_min, scalar_max, max_dist);
  
  // Get the minimum and maximum tile indices
  row_min = MAX(0, TileRow(bounds.Maximum().Y()));
  row_max = MIN(NumberOfTileRows()-1, TileRow(bounds.Minimum().Y()));
  col_min = MAX(0, TileColumn(bounds.Minimum().X()));
  col_max = MIN(NumberOfTileColumns()-1, TileColumn(bounds.Maximum().X()));
  if (verbose) 
    printf("Selecting points from tile rows %d-%d and tile columns %d-%d\n",
           row_min, row_max, col_min, col_max);
           
  // Collect points of all tiles in this area
  for (row=row_min; row!=row_max+1; row++) {
    for (col=col_min; col!=col_max+1; col++) {
      tile = Tile(row, col);
      if (tile != NULL) {
        tile->Read(tile->PointFile(), false);
        tile->LaserPoints::Select(selection, line, max_dist,
                                  scalar_min, scalar_max);
        tile->ErasePoints();
      }
    }
  }
}

long long int LaserBlock::NumberOfPoints() const
{
  LaserBlock                 *block;
  LaserBlock::const_iterator unit;
  long long int              num_pts = 0;
  
  // If needed read meta data in local block
  if (empty()) {
    block = new LaserBlock();
    if (MetaDataFile())
      if (!block->ReadMetaData(MetaDataFile()))
        printf("Error reading block meta data from file %s\n", MetaDataFile());
    for(unit=block->begin(); unit!=block->end(); unit++)
      num_pts += unit->NumberOfPoints();
    delete block;
  }
  else
    for(unit=begin(); unit!=end(); unit++)
      num_pts += unit->NumberOfPoints();
  
  return num_pts;
}
