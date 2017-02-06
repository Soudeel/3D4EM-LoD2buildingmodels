
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



/*                               _\\|//_ 
                                 ( .-. )
------------------------------o00--(_)--00o------------------------------

  Function      : Definition of file identifiers for the different kind
                  of databases.
  Author        : George Vosselman
  Creation date : 18 - Mar - 1999

  Modified by   : 
  Dated         : 
  Modification  : 
------------------------------------------------------------------------
*/

#ifndef _file_id_h_
#define _file_id_h_


/*                               _\\|//_ 
                                 ( 0-0 )
------------------------------o00--(_)--00o------------------------------
  File identifying constants. If a file doesn't start with a valid id, 
  the contents is not recognized and an error messsage is generated
-------------------------------------------------------------------------
*/

/* Identifiers of the files with I/O routines in C. All these I/O routines have
 * interfaces to C++
 */

#define IMAGE_POINTS              701
#define OBJECT_POINTS             702
#define CONTROL_POINTS            703
#define INTERNAL_ORIENTATION      704
#define EXTERNAL_ORIENTATION      705
#define RELATIVE_ORIENTATION      706
#define CAMERA_POINTS             707
#define ABSOLUTE_ORIENTATION      708
#define MODEL_POINTS              709
#define IMAGE_LINES               710
#define LINE_TOPOLOGY             711
#define GRID                      712
#define OBJECT_POINTS_2D          713
#define GRID_3D                   714
#define PROJECTIVE_TRANSFORMATION 715
#define LINE_TOPOLOGY_V1          716
#define TRIANGLES                 802  /* Used to be in altimetry toolbox */

/* Identifiers of the files with I/O routines in C++. There are no interfaces
 * to C.
 */

#define POSITIONS2D               901
#define POSITIONS3D               902
#define MAPPINGLIB_PLANES         903

/* A line in an input file will be ignored if it starts with this character */

#define comment_char '#'

#endif /* _file_id_h_ */
