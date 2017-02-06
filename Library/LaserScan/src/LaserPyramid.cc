
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
 Collection of functions for class LaserPyramid

 Initial creation
 Author : George Vosselman
 Date   : 16-04-2007

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
#include "LaserPyramid.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                             Copy assignment
--------------------------------------------------------------------------------
*/

LaserPyramid & LaserPyramid::operator = (const LaserPyramid &pyramid)
{
  if (this == &pyramid) return *this;  // Check for self assignment
  if (!empty()) erase(begin(), end());
  if (!pyramid.empty()) insert(begin(), pyramid.begin(), pyramid.end());
  LaserDataFilesReference() = pyramid.LaserDataFilesReference();
  original_point_spacing = pyramid.original_point_spacing;
  reduction_factor       = pyramid.reduction_factor;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise the meta data
--------------------------------------------------------------------------------
*/

void LaserPyramid::Initialise()
{
  LaserDataFiles::Initialise();
  reduction_factor = 2;
  original_point_spacing = 0.0;
  used_level = 0;
  point_fd = NULL;
}

void LaserPyramid::ReInitialise()
{
  LaserDataFiles::ReInitialise();
  if (point_fd != NULL) fclose(point_fd); // Close point file
  if (!empty()) erase(begin(), end());  // Delete old blocks
  Initialise();
}

/*
--------------------------------------------------------------------------------
                       Read the pyramid meta data
--------------------------------------------------------------------------------
*/


int LaserPyramid::ReadMetaData(const char *filename)
{
  FILE                    *fd;
  char                    *buffer, *line, *keyword, *file;
  int                     keyword_length, success, number;
  LaserPyramid::iterator  block;

  // Open the file
  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening pyramid database %s\n", filename);
    return(0);
  }

  // Verify that the first keyword is "laserpyramid"
  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading pyramid database %s\n", filename);
    fclose(fd);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "laserpyramid", MAX(keyword_length, 12))) {
    fprintf(stderr, "Error: File %s is not a pyramid database.\n", filename);
    fprintf(stderr, "       First keyword is %s, not laserpyramid.\n",
            keyword);
  }

  // Initialise all meta data
  ReInitialise();

  // Process all lines
  SetMetaDataFile(filename);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
	      name = BNF_String(line);

        else if (!strncmp(keyword, "points", MAX(keyword_length, 6)))
          point_file = BNF_String(line);

        else if (!strncmp(keyword, "block", MAX(keyword_length, 5))) {
          file = BNF_String(line);
          if (file) {
	        push_back(LaserBlock(file, &success));
	        if (!success) erase(end()-1); 
          }
        }

        else if (!strncmp(keyword, "endlaserpyramid", MAX(keyword_length,15))) {
          free(buffer);
          fclose(fd);
          return(1);
        }

        else if (!strncmp(keyword, "point_spacing", MAX(keyword_length, 13)))
          original_point_spacing = BNF_Double(line);

        else if (!strncmp(keyword, "reduction_factor", MAX(keyword_length, 16)))
          reduction_factor = BNF_Integer(line);

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserpyramid.\n");
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the pyramid meta data to file
--------------------------------------------------------------------------------
*/

int LaserPyramid::WriteMetaData() const
{
  return(WriteMetaData(meta_file, 0, 0, 0));
}

int LaserPyramid::WriteMetaData(const char *filename) const
{
  return(WriteMetaData(filename, 0, 0, 0));
}

int LaserPyramid::WriteMetaData(int write_blocks, int write_units,
                                int write_sub_units) const
{
  return(WriteMetaData(meta_file, write_blocks, write_units, write_sub_units));
}

int LaserPyramid::WriteMetaData(const char *filename, int write_blocks,
                                int write_units, int write_sub_units) const
{
  FILE       *fd;
  int        indent;
  LaserPyramid::const_iterator block;

  // Open the file
  if (!filename) return(0); // Silently return
  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening pyramid database %s\n", filename);
    return(0);
  }

  // Write the meta data
  indent = 0;
  BNF_Write_String(fd, "laserpyramid", indent, NULL);  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (point_file) BNF_Write_String(fd, "points", indent, point_file);
  BNF_Write_Double(fd, "point_spacing", indent, original_point_spacing, "%.2f");
  BNF_Write_Integer(fd, "reduction_factor", indent, reduction_factor, "%d");
  for (block=begin(); block!=end(); block++) {
    BNF_Write_String(fd, "block", indent, block->MetaDataFile());
    if (write_blocks) (void) block->WriteMetaData(write_units, write_sub_units);
  }
  indent -= 2;
  BNF_Write_String(fd, "endlaserpyramid", 0, NULL);

  // Close the file and return
  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Derive meta data file name
--------------------------------------------------------------------------------
*/

char *LaserPyramid::DeriveMetaDataFileName(const char *directory)
{
  return(LaserMetaFile::DeriveMetaDataFileName(directory, ".pyramid"));
}

/*
--------------------------------------------------------------------------------
                     Create a pyramid from a tiled block
--------------------------------------------------------------------------------
*/

int LaserPyramid::Create(const LaserBlock &block, int red_method,
                         int red_factor,
                         double specified_original_point_spacing,
                         bool overwrite)
{
  int                       level, child_row, parent_row, child_col,
                            parent_col, num_child_rows, num_child_columns;
  LaserPyramid::iterator    parent_block, child_block;
  LaserBlock::iterator      tile_container;
  LaserSubUnit              original_tile, parent_tile, *child_tile,
                            reduced_tile;
  double                    point_spacing, tile_size_X, tile_size_Y,
                            block_min_X, block_max_Y, tile_min_X, tile_max_Y;
  char                      *name, *directory, *slash, *ch;
  bool                      debug=false, six_digits;
  
  // Check if the block has a valid tile structure
  if (block.empty()) {
    printf("Error: no data in block %s.\n", block.Name());
    return 0;
  }
  if (block.begin()->DataOrganisation() != TileWise) {
    printf("Error: block %s is not organised tile wise.\n", block.Name());
    printf("       First use programme createtiles to reorganise the block.\n");
    return 0;
  }
  if (block.begin()->empty()) {
    printf("Error: first block unit does not contain data.\n");
    return 0;
  }
  original_tile = *(block.begin()->begin());
  if (original_tile.TileBorder() > 0.001) {
    printf("Warning: tile borders are larger than 0.\n");
    printf("         Tile border size will not scale with pyramid level.\n");
  }
  
  // Determine point spacing for data reduction
  if (!original_tile.Read(original_tile.PointFile(), false)) {
    printf("Error reading first tile of block %s\n", block.Name());
    return 0;
  }
  
  if (specified_original_point_spacing > 0.0)
    original_point_spacing = specified_original_point_spacing;
  else
    original_point_spacing =
      original_tile.MedianInterPointDistance(MIN(original_tile.size()-1, 1000));
  original_tile.ErasePoints();
  reduction_factor = red_factor;
  
  // Derive directory name from point file name of first tile
  directory = (char *) malloc(strlen(original_tile.PointFile()) + 1);
  strcpy(directory, original_tile.PointFile());
  slash = strrchr(directory, '/');
  if (slash == NULL) {free(directory); directory = NULL;}
  else *slash = 0;
  
  // Set pyramid meta data
  SetName(block.Name());
  DeriveMetaDataFileName(directory);
  
  // Copy block to level 0 of the pyramid
  level = 0;
  num_child_rows    = block.NumberOfTileRows();
  num_child_columns = block.NumberOfTileColumns();
  // Use six digits in tile names for blocks with more than 1000 tiles in a row or column
  six_digits = (num_child_rows > 1000 | num_child_columns > 1000);
  push_back(block);
  point_spacing = original_point_spacing;
  parent_tile.DataOrganisation() = TileWise;
  parent_tile.SetTileBorder(0.0);
  
  // Retrieve tile geometry information
  tile_size_X = original_tile.TileBounds().XRange();
  tile_size_Y = original_tile.TileBounds().YRange();
  block_min_X = original_tile.TileBounds().Minimum().X() -
                original_tile.TileColumn() * tile_size_X;
  block_max_Y = original_tile.TileBounds().Maximum().Y() +
                original_tile.TileRow() * tile_size_Y;

  // Iterate until no further reduction is required
  child_block = begin();
//  while (num_child_rows > 1 || num_child_columns > 1) {
  while (child_block->begin()->size() > 1) {
    level++;
    point_spacing *= (double) reduction_factor;
    tile_size_X   *= (double) reduction_factor;
    tile_size_Y   *= (double) reduction_factor;
    
    // Create a parent block
    push_back(LaserBlock());
    parent_block = end() - 1;
    name = (char *) malloc(strlen(block.Name()) + 5);
    sprintf(name, "%s_L%2d", block.Name(), level);
    if (name[strlen(name)-2] == ' ') name[strlen(name)-2] = '0';
    parent_block->SetName(name);
    free(name);
    parent_block->DeriveMetaDataFileName(directory);
    
    // Create a tile container
    parent_block->push_back(LaserUnit());
    tile_container = parent_block->begin();
    tile_container->DataOrganisation() = TileWise;
    
    child_block = parent_block - 1;
    // Loop over all tiles of the parent block
    for (parent_row=0, tile_max_Y=block_max_Y;
         parent_row * reduction_factor < num_child_rows;
         parent_row++, tile_max_Y-=tile_size_Y) {
      for (parent_col=0, tile_min_X=block_min_X;
           parent_col * reduction_factor < num_child_columns;
           parent_col++, tile_min_X+=tile_size_X) {

        // Compose the file names and check if they are already there
        if (six_digits) {
          name = (char *) malloc(strlen(block.Name()) + 19);
          sprintf(name, "%s_L%2d_%6d_%6d", block.Name(), level,
                  parent_row, parent_col);
        }
        else  {
          name = (char *) malloc(strlen(block.Name()) + 13);
          sprintf(name, "%s_L%2d_%3d_%3d", block.Name(), level,
                  parent_row, parent_col);
        }
        ch = name + strlen(block.Name());
        while (*ch != 0) {
          if (*ch == ' ') *ch = '0';
          ch++;
        }      
        parent_tile.SetName(name);
        free(name);
        parent_tile.DerivePointFileName(directory);
        parent_tile.DeriveMetaDataFileName(directory);
        if (overwrite || !BNF_FileExists(parent_tile.PointFile()) ||
            !BNF_FileExists(parent_tile.MetaDataFile())) {
          // Loop over all child tiles of this tile to collect the data
          for (child_row = parent_row * reduction_factor;
               child_row < (parent_row+1) * reduction_factor; child_row++) {
            for (child_col = parent_col * reduction_factor;
                 child_col < (parent_col+1) * reduction_factor; child_col++) {
              child_tile = child_block->Tile(child_row, child_col);
              if (child_tile != NULL) {  
                // Read tile data
                if (!child_tile->Read(child_tile->PointFile(), false)) {
                  printf("Error reading data of tile %s\n",
                         child_tile->PointFile());
                  return 0;
                }
                // Add it to the parent tile
                parent_tile.insert(parent_tile.end(), child_tile->begin(),
                                   child_tile->end());
                child_tile->ErasePoints();
              }
            }
          }
          if (parent_tile.size() > 0) {
            // Set tile bounds of parent tile
            parent_tile.TileBounds().SetMinimumX(tile_min_X);
            parent_tile.TileBounds().SetMaximumX(tile_min_X + tile_size_X);
            parent_tile.TileBounds().SetMinimumY(tile_max_Y - tile_size_Y);
            parent_tile.TileBounds().SetMaximumY(tile_max_Y);
            parent_tile.SetTileLocation(parent_row, parent_col);
            // Reduce the collected data
            switch (red_method) {
              case 1: // Reduction based on point spacing in 2D
                parent_tile.DeriveTIN();
                parent_tile.LaserPoints::ReduceData(point_spacing);
                break;
              case 2: // Reduction based on point spacing in 3D, use knn=10
                parent_tile.LaserPoints::ReduceData(point_spacing, 10);
                break;
              case 3: // Reduction by taking every n'th point
                parent_tile.LaserPoints::ReduceData(red_factor * red_factor, 0);
                break;
              case 4: // Reduction by randomly selecting from every set of n points
                parent_tile.LaserPoints::ReduceData(red_factor * red_factor, 1);
                break;
              case 5: // Reduction by taking every n'th point using knn
                parent_tile.LaserPoints::ReduceData(4, 10, red_factor * red_factor);
                break;
              default:
                printf("In function LaserPyramid::Create(...)\n");
                printf("Invalid reduction method %d, valid value range is 1-5.\n",
                       red_method);
                exit(0);
            }
            // Also make sure just to select points inside the tile
            // This line is required to eliminate points in the border or
            // points of multiple echoes outside the tile.
            parent_tile.LaserPoints::ReduceData(parent_tile.TileBounds());
            // Save and erase the data
            printf("Saving tile (%d, %d) at level %d with %d points\n",
                   parent_row, parent_col, level, parent_tile.size());
            parent_tile.Write(false);
            parent_tile.ErasePoints();
            if (red_method == 1) parent_tile.GetTIN()->Erase();
            parent_tile.WriteMetaData();
            // Add the tile to the parent block
            tile_container->push_back(parent_tile);
          }
        }
        else {
          // Reread the meta data from the existing tile and copy into container
          // Make a copy of the filename, because it is reinitialised in
          // ReadMetaData.
          name = (char *) malloc(strlen(parent_tile.MetaDataFile() + 1));
          strcpy(name, parent_tile.MetaDataFile());
          if (debug) printf("Parent point file %s\n", parent_tile.PointFile());
          if (debug) printf("Parent tile file %s\n", parent_tile.MetaDataFile());
          if (debug) printf("Read old tile %s\n", name);
          parent_tile.ReadMetaData(name);
          tile_container->push_back(parent_tile);
        }  
      }
    }
    num_child_rows    = parent_block->NumberOfTileRows();
    num_child_columns = parent_block->NumberOfTileColumns();
  }
  // Write all meta data
  WriteMetaData(true, true, true);
  return 1;
}

/*
--------------------------------------------------------------------------------
     Select tiles in the specified area with the specified point spacing
--------------------------------------------------------------------------------
*/
LaserTilePtrVector & LaserPyramid::SelectTiles(const DataBounds2D &bounds,
                                               double point_spacing)
{
  LaserPyramid::iterator block;
  double                 block_spacing;
  LaserTilePtrVector     *empty_list;
  bool                   verbose=false;
  
  if (verbose) {printf("Selecting tiles..."); fflush(stdout);}
  // Return empty tile vector if there are no pyramid levels
  if (empty()) {
    empty_list = new LaserTilePtrVector();
    return *empty_list;
  }
  
  // Find the pyramid level with the right spacing
  block_spacing = original_point_spacing;
  used_level = 0;
  block = begin();
  while (block_spacing * reduction_factor < point_spacing &&
         block+1 != end()) {
    block++;
    used_level++;
    block_spacing *= reduction_factor;
  }
  
  if (verbose) printf("done\n");
  // Select the tiles from the block
  return block->SelectTiles(bounds);
}

/*
--------------------------------------------------------------------------------
              Update a laser point selection
--------------------------------------------------------------------------------
*/
int LaserPyramid::UpdateSelection(const DataBounds2D &bounds,
                                  double point_spacing,
                                  LaserPoints &selection, int max_num_pts)
{
  LaserTilePtrVector           selected_tiles, new_tiles;
  LaserTilePtrVector::iterator tile;
  int                          i, new_tile_index, old_tile_index, tile_size;
  LaserPoints::iterator        new_point_index, old_point_index;
  bool                         changed, verbose=false;
  int                          tile_sum=0;

  // Select the tiles within the bounds
  selected_tiles = SelectTiles(bounds, point_spacing);
  changed = false;
  
  // First do a quick check
  for (tile=selected_tiles.begin(); tile!=selected_tiles.end() && !changed;
       tile++)
    if (!used_tiles.Contains(*tile)) changed = true;
  if (!changed) {
    if (verbose) printf("Same selected tiles\n");
    return 0;
  }
  
  changed = false;
  // Save points from the previous selection that can be re-used
  if (verbose) printf("Checking points to be re-used\n");
  new_point_index = selection.begin();
  new_tile_index  = -1;
  for (tile=used_tiles.begin(), old_tile_index=0;
       tile!=used_tiles.end(); tile++, old_tile_index++) {
    if (selected_tiles.Contains(*tile)) {
      new_tiles.push_back(*tile);
      new_tile_index++;
      if (old_tile_index == 0) {
        old_point_index = selection.begin();
        tile_size = end_of_tile[0] + 1;
      }
      else {
        old_point_index = selection.begin() + end_of_tile[old_tile_index-1] + 1;
        tile_size = end_of_tile[old_tile_index] - end_of_tile[old_tile_index-1];
      }
      tile_sum += tile_size;
      if (old_tile_index != new_tile_index) {
        if (verbose) printf("Copying data of tile %s\n", (*tile)->Name());
        // Copy points to new location
        for (i=0; i<tile_size; i++, old_point_index++)
          *new_point_index++ = *old_point_index;
        // Update end of tile information
        if (new_tile_index == 0) end_of_tile[0] = tile_size - 1;
        else 
          end_of_tile[new_tile_index] = end_of_tile[new_tile_index-1] +
                                        tile_size;
      }
      else new_point_index += tile_size;
    }
  }
  // Remove points that are no longer needed
  if (new_point_index != selection.end()) {
    if (verbose) printf("Removing old tiles\n");
    changed = true;
    if (!selection.empty()) selection.erase(new_point_index, selection.end());
    if (!end_of_tile.empty())
      end_of_tile.erase(end_of_tile.begin()+new_tile_index+1, end_of_tile.end());
  }
  // Add points of new tiles
  for (tile=selected_tiles.begin(); tile!=selected_tiles.end(); tile++) {
    if (!new_tiles.Contains(*tile)) {
      changed = true;
      // Read points of new tile
      if (verbose) printf("Reading new tile %s\n", (*tile)->Name());
//      if (!(*tile)->Read((*tile)->PointFile(), false)) {
      if (!ReadTile(**tile)) {
        printf("Error reading points of tile %s\n", (*tile)->Name());
        return -1;
      }
      // Check the selection size limit
      if (selection.size() + (*tile)->size() > max_num_pts) return 2;
      // Add the points and tile information
      selection.insert(selection.end(), (*tile)->begin(), (*tile)->end());
      new_tiles.push_back(*tile);
      new_tile_index++;
      tile_sum += (*tile)->size();
      if (new_tile_index == 0)
        end_of_tile.push_back((*tile)->size() - 1);
      else
        end_of_tile.push_back(end_of_tile[new_tile_index-1] + (*tile)->size());
      (*tile)->ErasePoints();
    }
  }
  // Save the tile pointer vector
  used_tiles.swap(new_tiles);
/*
  if (changed && verbose) {
    printf("Used:");
    for (tile=new_tiles.begin(); tile!=new_tiles.end(); tile++)
      printf(" %s", (*tile)->Name());
    printf("\n");
    printf("New:");
    for (tile=used_tiles.begin(); tile!=used_tiles.end(); tile++)
      printf(" %s", (*tile)->Name());
    printf("\n");
    tile = used_tiles.begin();
    printf("Selection area (%.1f, %.1f), tile size %.1f\n",
           bounds.XRange(), bounds.YRange(), (*tile)->TileBounds().XRange());
    printf("Sum of %d tiles: %d, point count: %d\n", used_tiles.size(), 
           tile_sum, selection.size());
  }
*/
  new_tiles.Erase();
  selected_tiles.Erase();
  if (verbose) printf("Tile update completed\n");
  if (changed) return 1;
  return 0;
}

/*
--------------------------------------------------------------------------------
                Remove information on previously used tiles
--------------------------------------------------------------------------------
*/

void LaserPyramid::InvalidateUsedTiles()
{
  if (!used_tiles.empty()) used_tiles.Erase();
  if (!end_of_tile.empty()) 
    end_of_tile.erase(end_of_tile.begin(), end_of_tile.end());
}

/*
--------------------------------------------------------------------------------
                Remove information on previously used tiles
--------------------------------------------------------------------------------
*/

void LaserPyramid::SelectedTileRange(int &min_row, int &max_row,
                                     int &min_column, int &max_column,
                                     int &current_level) const
{
  LaserTilePtrVector::const_iterator tile;
  
  min_row = min_column = INT_MAX;
  max_row = max_column = INT_MIN;
  for (tile=used_tiles.begin(); tile!=used_tiles.end(); tile++) {
    if ((*tile)->TileRow() < min_row) min_row = (*tile)->TileRow();
    if ((*tile)->TileRow() > max_row) max_row = (*tile)->TileRow();
    if ((*tile)->TileColumn() < min_column) min_column = (*tile)->TileColumn();
    if ((*tile)->TileColumn() > max_column) max_column = (*tile)->TileColumn();
  }
  current_level = used_level;
}

/*
--------------------------------------------------------------------------------
       Read tile (from merged pyramid point file or tile point file)
--------------------------------------------------------------------------------
*/

int LaserPyramid::ReadTile(LaserSubUnit &tile)
{
  if (PointFile() != NULL) { // There is a merged pyramid point file
    if (point_fd == NULL) { // File is not yet opened
      if (!OpenPointFile()) return 0;
    }
    // Read from merged pyramid point file
    return tile.Read(point_fd);
  }
  
  // Read from tile point file
  return tile.Read(tile.PointFile(), false);
}

int LaserPyramid::ClosePointFile()
{
  if (point_fd == NULL) return 0; // Nothing to close
  fclose(point_fd);
  point_fd = NULL;
  return 1;
}

int LaserPyramid::OpenPointFile(char *filename)
{
  if (filename != NULL) SetPointFile(filename);
  if (point_fd != NULL) ClosePointFile(); // Close old point file
  point_fd = fopen(PointFile(), "rb");
  if (point_fd == NULL) {
    printf("Error opening pyramid point file %s\n", PointFile());
    return 0;
  }
  return 1;
}

/*
--------------------------------------------------------------------------------
       Return pointer to block at specified pyramid level
--------------------------------------------------------------------------------
*/

const LaserBlock * LaserPyramid::BlockAtLevel(int level) const
{
  if (level < 0) return NULL;
  if (level > size() - 1) return NULL;
  return &*(begin() + level);
}

/*
--------------------------------------------------------------------------------
      Collect used tile boundaries
--------------------------------------------------------------------------------
*/
void LaserPyramid::CollectUsedTileBoundaries(ObjectPoints2D &corners,
                                             LineTopologies &topology) const
{
  LaserTilePtrVector::const_iterator tile;
  
  for (tile=used_tiles.begin(); tile!=used_tiles.end(); tile++)
    (*tile)->AddTileBoundary(corners, topology);
}

/*
--------------------------------------------------------------------------------
      Number of points
--------------------------------------------------------------------------------
*/
long long int LaserPyramid::NumberOfPoints() const
{
  LaserPyramid                 *pyramid;
  LaserPyramid::const_iterator block;
  long long int                num_pts = 0;
  
  // If needed read meta data in local pyramid
  if (empty()) {
    pyramid = new LaserPyramid();
    if (MetaDataFile())
      if (!pyramid->ReadMetaData(MetaDataFile()))
        printf("Error reading pyramid meta data from file %s\n", MetaDataFile());
    for(block=pyramid->begin(); block!=pyramid->end(); block++)
      num_pts += block->NumberOfPoints();
    delete pyramid;
  }
  else
    for(block=begin(); block!=end(); block++)
      num_pts += block->NumberOfPoints();
  
  return num_pts;
}
