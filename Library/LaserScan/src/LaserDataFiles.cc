
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
 Collection of functions for class LaserDataFiles

 Initial creation
 Author : George Vosselman
 Date   : 18-04-1999

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
#include "LaserDataFiles.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                            Copy assignment
--------------------------------------------------------------------------------
*/

LaserDataFiles LaserDataFiles::operator = (const LaserDataFiles &ldf)
{
  // Check for self assignment
  if (this == &ldf) return *this;
  LaserMetaFileReference() = ldf.LaserMetaFileReference();
  SetPointFile(ldf.point_file);
  SetTINFile(ldf.tin_file);
  return(*this);
}

/*
--------------------------------------------------------------------------------
                              Initialisation
--------------------------------------------------------------------------------
*/

void LaserDataFiles::Free()
{
  LaserMetaFile::Free();
  if (point_file) free(point_file);
  if (tin_file) free(tin_file);
}

void LaserDataFiles::Initialise()
{
  LaserMetaFile::Initialise();
  point_file = tin_file = NULL;
}

void LaserDataFiles::ReInitialise()
{
  Free();
  Initialise();
}

/*
--------------------------------------------------------------------------------
                         Set and derive point file name
--------------------------------------------------------------------------------
*/

void LaserDataFiles::SetPointFile(const char *filename)
{
  StringCopy(&point_file, filename);
}

char *LaserDataFiles::DerivePointFileName(const char *directory,
                                          const char *subdirectory)
{
  if (!name)
    if (!DeriveName()) return(NULL); /* We need a name */
  if (point_file) free(point_file);
  point_file = ComposeFileName(directory, name, ".laser", subdirectory);
  return(point_file);
}

/*
--------------------------------------------------------------------------------
                         Set and derive TIN file name
--------------------------------------------------------------------------------
*/

void LaserDataFiles::SetTINFile(const char *filename)
{
  StringCopy(&tin_file, filename);
}

char *LaserDataFiles::DeriveTINFileName(const char *directory)
{
  if (!name)
    if (!DeriveName()) return(NULL); /* We need a name */
  if (tin_file) free(tin_file);
  tin_file = ComposeFileName(directory, name, ".tin");
  return(tin_file);
}

/*
--------------------------------------------------------------------------------
                         Derive name of dataset
--------------------------------------------------------------------------------
*/

char *LaserDataFiles::DeriveName()
{
  char *file;

  if (point_file) file = point_file;
  else if (meta_file) file = meta_file;
  else if (tin_file) file = tin_file;
  else return(NULL); /* We need some file name */
  if (name) free(name);
  name = DeriveNameFromFile(file);
  return(name);
}

/*
--------------------------------------------------------------------------------
                      Derive name of the meta data file
--------------------------------------------------------------------------------
*/

char *LaserDataFiles::DeriveMetaDataFileName(const char *directory,
                                             const char *extension)
{
  if (!name)
    if (!DeriveName()) return(NULL); /* We need a name */
  if (meta_file) free(meta_file);
  meta_file = ComposeFileName(directory, name, extension);
  return(meta_file);
}
