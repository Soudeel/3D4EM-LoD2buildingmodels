
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
 Collection of functions for class LaserMetaFile

 LaserMetaFile LaserMetaFile::operator = Copy assignment
   (const LaserMetaFile &)
 void LaserMetaFile::Free()                          Release memory
 void LaserMetaFile::Initialise()                    Initialisation
 void LaserMetaFile::ReInitialise()                  Reinitialisation
 void LaserMetaFile::SetName(const char *)           Set the dataset name
 char *LaserMetaFile::DeriveName()                   Derive name of the dataset
 void LaserMetaFile::SetMetaDataFile(const char *)   Set the point file name
 char *LaserMetaFile::DeriveMetaDataFileName         Derive meta data file name
   (const char *, const char *)

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
#include "LaserMetaFile.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                            Copy assignment
--------------------------------------------------------------------------------
*/

LaserMetaFile LaserMetaFile::operator = (const LaserMetaFile &lmf)
{
  // Check for self assignment
  if (this == &lmf) return *this;
  SetName(lmf.name);
  SetMetaDataFile(lmf.meta_file);
  return(*this);
}

/*
--------------------------------------------------------------------------------
                              Initialisation
--------------------------------------------------------------------------------
*/

void LaserMetaFile::Free()
{
  if (name) free(name);
  if (meta_file) free(meta_file);
}

void LaserMetaFile::Initialise()
{
  name = meta_file = NULL;
}

void LaserMetaFile::ReInitialise()
{
  Free();
  Initialise();
}

/*
--------------------------------------------------------------------------------
                         Set and derive name of dataset
--------------------------------------------------------------------------------
*/

void LaserMetaFile::SetName(const char *new_name)
{
  StringCopy(&name, new_name);
}

char *LaserMetaFile::DeriveName()
{
  if (!meta_file) return(NULL);
  if (name) free(name);
  name = DeriveNameFromFile(meta_file);
  return(name);
}

/*
--------------------------------------------------------------------------------
                        Set and derive meta file name
--------------------------------------------------------------------------------
*/

void LaserMetaFile::SetMetaDataFile(const char *filename)
{
  StringCopy(&meta_file, filename);
}

char *LaserMetaFile::DeriveMetaDataFileName(const char *directory,
                                            const char *extension)
{
  if (!name) return(NULL);  /* We need a name */
  if (meta_file) free(meta_file);
  meta_file = ComposeFileName(directory, name, extension);
  return(meta_file);
}
