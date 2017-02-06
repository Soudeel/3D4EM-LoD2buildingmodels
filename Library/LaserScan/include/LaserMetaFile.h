
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
 * \brief Interface to Class LaserMetaFile - Name of dataset and meta file
 *
 */
/*!
 * \class LaserMetaFile
 * \ingroup LMetaData
 * \brief Interface to Class LaserMetaFile - Name of dataset and meta file
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		18-04-1999 (Created)
 *
 * \remark \li None
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _LaserMetaFile_h_
#define _LaserMetaFile_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 LaserMetaFile                          - Name of dataset and meta file

 Initial creation
 Author : George Vosselman
 Date   : 18-04-1999

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
///                 Names of dataset and meta file
//------------------------------------------------------------------------------

class LaserMetaFile
{
  protected:
    /// Name of the data set
    char *name;

    /// Name of the meta data file
    char *meta_file;


  public:

    /// Default constructor
    LaserMetaFile()
      {Initialise();}

    /// Copy assignment
    LaserMetaFile operator = (const LaserMetaFile &);

    /// Return the reference
    LaserMetaFile &LaserMetaFileReference() 
      {return(*this);}

    /// Return the const reference
    const LaserMetaFile &LaserMetaFileReference() const
      {return(*this);}

    /// Release all memory
    void Free();

    /// Initialisation of new names
    void Initialise();

    /// Reinitialisation of old names
    void ReInitialise();

    /// Return the data set name
    char *Name() const
      {return(name);};

    /// Set the data set name
    void SetName(const char *new_name);

    /// Derive the data set name from "meta_file"
    char *DeriveName();

    /// Return the meta data file name
    char *MetaDataFile() const
      {return(meta_file);}

    /// Set the meta data file name
    void SetMetaDataFile(const char *filename);

    /// Derive the meta data file name
    /** @param directory Directory for the meta data file
        @param extension File extension, e.g. pointset or strip
    */
    char *DeriveMetaDataFileName            /* Derive meta data file name     */
      (const char *directory, const char *extension);
};

#endif /* _LaserMetaFile_h_ */  /* Don't add after this point */
