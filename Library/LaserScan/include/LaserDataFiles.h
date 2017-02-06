
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
 * \brief Interface to Class LaserDataFiles - Name of dataset and several files
 *
 */
/*!
 * \class LaserDataFiles
 * \ingroup LMetaData
 * \brief Interface to Class LaserDataFiles - Name of dataset and several files
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		18-04-1999 (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _LaserDataFiles_h_
#define _LaserDataFiles_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 LaserDataFiles                          - Name of dataset and several files

 Initial creation
 Author : George Vosselman
 Date   : 18-04-1999

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

#include "LaserMetaFile.h"

//------------------------------------------------------------------------------
///                 Names of laser data files
//------------------------------------------------------------------------------

class LaserDataFiles : public LaserMetaFile
{
  protected:

    /// Name of the file with the laser points
    char *point_file;

    /// Name of the file with the Delaunay triangulation
    char *tin_file;

  public:

    /// Default constructor
    LaserDataFiles()
      {Initialise();}

    /// Copy assignment
    LaserDataFiles operator = (const LaserDataFiles &);

    /// Return the reference
    LaserDataFiles &LaserDataFilesReference() 
      {return(*this);}

    /// Return the const reference
    const LaserDataFiles &LaserDataFilesReference() const
      {return(*this);}

    /// Release all memory
    void Free();

    /// Initialisation of new file names
    void Initialise();

    /// Reinitialisation of old file names
    void ReInitialise();

    /// Return the point file name
    char *PointFile() const
      {return(point_file);}

    /// Set the point file name
    void SetPointFile(const char *filename);

    /// Derive the point file name
    char *DerivePointFileName(const char *directory,
                              const char *subdirectory=NULL);

    /// Return the TIN file name
    char *TINFile() const
      {return(tin_file);}

    /// Set the TIN file name
    void SetTINFile(const char *filename);

    /// Derive the TIN file name
    char *DeriveTINFileName(const char *directory);

    /// Derive the data set name from "point_file" or "meta_file"
    char *DeriveName();

    /// Derive the meta data file name
    /** @param directory Directory for the meta data file
        @param extension File extension, e.g. pointset or strip
    */
    char *DeriveMetaDataFileName (const char *directory, const char *extension);
};

#endif /* _LaserDataFiles_h_ */  /* Don't add after this point */
