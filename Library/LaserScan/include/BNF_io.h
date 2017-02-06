
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
 * \brief This file contains the declarations of the functions for I/O in BNF format.
 *
 */
/*!
 * \ingroup LIDAR
 * \brief This file contains the declarations of the functions for I/O in BNF format.
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
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

#ifndef _BNF_io_h_
#define _BNF_io_h_

/*
--------------------------------------------------------------------------------
This file contains the declarations of the functions for I/O in BNF format.

--------------------------------------------------------------------------------
*/

#define MAXCHARS 256

extern char   *BNF_KeyWord(char *, int *);
extern char   *BNF_String(char *);
extern int    BNF_Integer(const char *);
extern void   BNF_Two_Integers(const char *, int *, int *);
extern void   BNF_Three_Integers(const char *, int *, int *, int *);
extern long long BNF_LongInteger(const char *);
extern double BNF_Double(const char *);
extern void   BNF_Write_String(FILE *, const char *, int, const char *);
extern void   BNF_Write_Integer(FILE *, const char *, int, int, const char *);
extern void   BNF_Write_Two_Integers(FILE *, const char *, int, int, int,
                                     const char *);
extern void   BNF_Write_Three_Integers(FILE *, const char *, int, int, int, int,
                                       const char *);
extern void   BNF_Write_LongInteger(FILE *, const char *, int, long long, const char *);
extern void   BNF_Write_Double(FILE *, const char *, int, double, const char *);
extern void   BNF_Indent(FILE *, int);
extern int    BNF_LaserFileClass(const char *);
extern int    BNF_LaserFileType(const char *);
extern char   *StringCopy(char **, const char *);
extern char   *ComposeFileName(const char *, const char *, const char *,
                               const char *subdirectory=NULL);
extern char   *DeriveNameFromFile(const char *);
//WARING. Biao, Nov 2016
// old function FileExists is changed to be BNF_FileExists
// to solve conflicts with other librarires
extern int    BNF_FileExists(const char *);
extern int    DirectoryExists(const char *);

#endif /* _BNF_io_h_ */  /* Don't add after this point */
