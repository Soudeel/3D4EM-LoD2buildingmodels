
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

 Initial creation:
 Author  : George Vosselman
 Date    : 25-03-1999

 Update #1
 Author : George Vosselman
 Date   : 2-4-1999
 Changes: Added char *StringCopy(char *)

 Update #2
 Author : George Vosselman
 Date   : 10-12-1999
 Changes: Added char FileExists(char *)

 Update #3
 Author : George Vosselman
 Date   : 02-07-2004
 Changes: Changed "char" to "const char" whereever possible
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
#include <dirent.h>
#include "LaserDataTypes.h"
#include "stdmath.h"
#include "Database4Cpp.h"

#define MAXCHARS 256

/*
--------------------------------------------------------------------------------
                         Extract a keyword from a line
--------------------------------------------------------------------------------
*/

char *BNF_KeyWord(char *line, int *keyword_length)
{
  char *firstchar, *lastchar;
  
/* Find the first "non-blank" character */

  firstchar = line;
  while (*firstchar == ' ' && *firstchar != 0) firstchar++;
  if (*firstchar == 0) return(NULL);

/* Find the last character of the keyword */

  lastchar = firstchar;
  while (*lastchar != ' ' && *lastchar != ':' && *lastchar != 0) lastchar++;
  if (*lastchar == 0) return(NULL);

/* Return the keyword */

  *keyword_length = (int) lastchar - (int) firstchar;
  return(firstchar);
}

/*
--------------------------------------------------------------------------------
                   Read a string, integer or double value
--------------------------------------------------------------------------------
*/

char *BNF_String(char *line)
{
  char *firstchar, *lastchar, *string;

// Find the first double quote

  firstchar = strchr(line, '"');
  if (!firstchar) {
    fprintf(stderr, "Error: No double quote in line:\n%s\n", line);
    return(NULL);
  }
  firstchar++;

// Find the second double quote

  lastchar = strchr(firstchar, '"');
  if (!lastchar) {
    fprintf(stderr, "Error: Unterminated string in line:\n%s\n", line);
    return(NULL);
  }
  *lastchar = 0;
 
// Duplicate the string and return it

  string = (char *) malloc(strlen(firstchar) + 1);
  strcpy(string, firstchar);
  *lastchar = '"'; // Restore the line
  return(string);
}

int BNF_Integer(const char *line)
{
  char *colon;
  int  number;

  colon = strchr(line, ':');
  sscanf(colon+1, "%d", &number);
  return(number);
}

void BNF_Two_Integers(const char *line, int *number1, int *number2)
{
  char *colon;

  colon = strchr(line, ':');
  sscanf(colon+1, "%d%d", number1, number2);
}

void BNF_Three_Integers(const char *line, int *number1, int *number2,
                        int *number3)
{
  char *colon;

  colon = strchr(line, ':');
  sscanf(colon+1, "%d%d%d", number1, number2, number3);
}

long long BNF_LongInteger(const char *line)
{
  char       *colon;
  long long  number;

  colon = strchr(line, ':');
  sscanf(colon+1, "%I64d", &number);
  return(number);
}

double BNF_Double(const char *line)
{
  char    *colon;
  double  number;

  colon = strchr(line, ':');
  sscanf(colon+1, "%lf", &number);
  return(number);
}

/*
--------------------------------------------------------------------------------
                Write an indent, string, integer or double value
--------------------------------------------------------------------------------
*/

void BNF_Indent(FILE *fd, int indent)
{
  int i;
  for (i=0; i<indent; i++) fprintf(fd, " ");
}

void BNF_Write_String(FILE *fd, const char *keyword, int indent,
                      const char *string)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  if (string) fprintf(fd, "\"%s\"\n", string);
  else fprintf(fd, "\n");
}

void BNF_Write_Integer(FILE *fd, const char *keyword, int indent, int value,
		       const char *format)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  fprintf(fd, format, value);
  fprintf(fd, "\n");
}

void BNF_Write_Two_Integers(FILE *fd, const char *keyword, int indent,
                            int value1, int value2, const char *format)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  fprintf(fd, format, value1, value2);
  fprintf(fd, "\n");
}

void BNF_Write_Three_Integers(FILE *fd, const char *keyword, int indent,
                              int value1, int value2, int value3,
                              const char *format)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  fprintf(fd, format, value1, value2, value3);
  fprintf(fd, "\n");
}

void BNF_Write_LongInteger(FILE *fd, const char *keyword, int indent, 
                           long long value, const char *format)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  fprintf(fd, format, value);
  fprintf(fd, "\n");
}

void BNF_Write_Double(FILE *fd, const char *keyword, int indent, double value,
		      const char *format)
{
  BNF_Indent(fd, indent);
  fprintf(fd, "%s: ", keyword);
  fprintf(fd, format, value);
  fprintf(fd, "\n");
}

/*
--------------------------------------------------------------------------------
                     Determine type of laser data file
--------------------------------------------------------------------------------
*/

int BNF_LaserFileType(const char *filename)
{
  FILE *fd;
  char *line, *keyword;
  int  keyword_length;

/* Read the first line up to a maximum of MAXCHARS characters */

  if (!(fd = Open_Compressed_File(filename, "r"))){
    return(LASER_UNKNOWN);
  }
  line = (char *) calloc(MAXCHARS, sizeof(char));
  fgets(line, MAXCHARS, fd);
  fclose(fd);

/* Analyse the string for meta data keyword */

  keyword = BNF_KeyWord(line, &keyword_length);
  if (keyword) {
    if (!strncmp(line, "laserpyramid", MAX(keyword_length, 12)))
      return(LASER_PYRAMID);
    else if (!strncmp(line, "laserblock", MAX(keyword_length, 10)))
      return(LASER_BLOCK);
    else if (!strncmp(line, "laserstrip", MAX(keyword_length, 10)))
      return(LASER_STRIP);
    else if (!strncmp(line, "lasertile", MAX(keyword_length, 9)))
      return(LASER_TILE);
    else if (!strncmp(line, "laserstrippart", MAX(keyword_length, 14)))
      return(LASER_STRIP_PART);
    else if (!strncmp(line, "laserpointset", MAX(keyword_length, 13)))
      return(LASER_POINT_SET);
  }

/* Assuming the file contains raw laser data */

  return(LASER_RAW_DATA);
}

int BNF_LaserFileClass(const char *filename)
{
  int type;

  type = BNF_LaserFileType(filename);
  if (type == LASER_STRIP_PART || type == LASER_TILE ||
      type == LASER_STRIP_TILE) return(LASER_SUB_UNIT);
  return(type);
}

/*
--------------------------------------------------------------------------------
                           Make a copy of a string
--------------------------------------------------------------------------------
*/

char *StringCopy(char **s_dest_ptr, const char *s_src)
{
  if (*s_dest_ptr == s_src) return s_src; // No copying on itself needed
  if (s_src == NULL) {
    if (*s_dest_ptr) free(*s_dest_ptr);
    *s_dest_ptr = NULL;
  }
  else {
    if (*s_dest_ptr == NULL) *s_dest_ptr = (char *) malloc(strlen(s_src) + 1);
    else *s_dest_ptr = (char *) realloc(*s_dest_ptr, strlen(s_src) + 1);
    strcpy(*s_dest_ptr, s_src);
  }
  return(*s_dest_ptr);
}

/*
--------------------------------------------------------------------------------
                           Compose a file name
--------------------------------------------------------------------------------
*/

char *ComposeFileName(const char *directory, const char *name,
                      const char *extension, const char *subdirectory)
{
  int  len;
  char *filename;

  if (!name) return(NULL); /* We need a name */
  len = strlen(name) + 1;
  if (directory) len += strlen(directory) + 1;
  if (extension) len += strlen(extension) + 1;
  filename = (char *) malloc(len);
  filename[0] = 0;
  if (directory) {
    strcpy(filename, directory);
    if (directory[strlen(directory)-1] != '/') strcat(filename, "/");
  }
  if (subdirectory) {
    if (subdirectory[0] == '/') strcat(filename, subdirectory+1);
    else strcat(filename, subdirectory);
    if (subdirectory[strlen(subdirectory)-1] != '/') strcat(filename, "/");
  }
  strcat(filename, name);
  if (extension) {
    if (extension[0] != '.') strcat(filename, ".");
    strcat(filename, extension);
  }
  return(filename);
}

/*
--------------------------------------------------------------------------------
                           Derive a data set name from a file name
--------------------------------------------------------------------------------
*/

char *DeriveNameFromFile(const char *file)
{
  const char *firstchar, *lastchar;
  char       *name;
  int        len;

  if (!file) return(NULL);
  firstchar = strrchr(file, '/');
  if (!firstchar) firstchar = file;
  else firstchar++;
  lastchar = strchr(firstchar, '.');
  if (!lastchar) lastchar = firstchar + strlen(firstchar);
  len = (int) lastchar - (int) firstchar;
  name = (char *) malloc(len+1);
  strncpy(name, firstchar, len);
  name[len] = 0;
  return(name);
}

/*
--------------------------------------------------------------------------------
                           Check if file or directory exists
--------------------------------------------------------------------------------
*/

int BNF_FileExists(const char *file)
{
  FILE *fd;

  if ((fd = Open_Compressed_File(file, "r")) != NULL) {
    Close_Compressed_File(fd);
    return(1);
  }
  else return(0);
}

int DirectoryExists(const char *directory)
{
  DIR *dir;
  
  if ((dir = opendir(directory)) == NULL) return 0;
  closedir(dir);
  return 1;
}
