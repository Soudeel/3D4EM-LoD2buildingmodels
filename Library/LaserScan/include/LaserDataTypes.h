
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
 * \brief Definition of file identifiers for the different kind of laser altimetry databases.
 *
 */
/*!
 * \ingroup LMetaData
 * \brief Definition of file identifiers for the different kind of laser altimetry databases.
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		29-Mar-1999 (Created)
 * \date        19-11-1999 (Update #1) - \G_Vosselman
 *
 * \remark \li Identifiers of the files with I/O routines in C++. There are no interfaces C.
 *
 * \remark \li File identifying constants. If a file doesn't start with a valid id, the contents is not recognized and an error messsage is generated
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _LaserDataTypes_h_
#define _LaserDataTypes_h_

/*                               _\\|//_ 
                                 ( .-. )
------------------------------o00--(_)--00o------------------------------

  Function      : Definition of file identifiers for the different kind
                  of laser altimetry databases.
  Author        : George Vosselman
  Creation date : 29 - Mar - 1999

  Modified by   : 
  Dated         : 
  Modification  : 
------------------------------------------------------------------------
*/


/*                               _\\|//_ 
                                 ( 0-0 )
------------------------------o00--(_)--00o------------------------------
  File identifying constants. If a file doesn't start with a valid id, 
  the contents is not recognized and an error messsage is generated
-------------------------------------------------------------------------
*/

/* Identifiers of the files with I/O routines in C++. There are no interfaces
 * to C.
 */

#define LASER_POINTS_V1     1001 // unsigned char reflectance and pulse count
#define LASER_SCANLINES     1002
#define LASER_POINTS_V2     1003 // int reflectance and pulse count
#define LASER_POINTS_V3     1004 // variable number of integer tagged attributes
#define LASER_LAS     1179861324 // corresponding to characters "LASF"



/* Enumeration of different file types used by programs that process 
 * blocks, strips, strip parts, tiles, strip tiles, point sets or raw laser
 * data.
 * One of these values is returned by BNF_LaserFileType an BNF_LaserFileClass.
 */

#define LASER_UNKNOWN           0
#define LASER_BLOCK		1
#define LASER_STRIP		2
#define LASER_STRIP_PART	3
#define LASER_POINT_SET		4	
#define LASER_RAW_DATA		5
#define LASER_TILE    		6
#define LASER_STRIP_TILE   	7
#define LASER_SUB_UNIT     	8
#define LASER_PYRAMID           9

#endif /* _LaserDataTypes_h */    /* Don't add after this point */
