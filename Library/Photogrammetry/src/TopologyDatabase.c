
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
                                 _\\|//_ 
                                 ( .-. )
------------------------------o00--(_)--00o---------------------------------

  Function      : Input and output routines for the database containing
                  the topology of lines
  Author        : George Vosselman
  Creation date : 11 - Sep - 1995

  Modification  : 03 - Feb - 1999
  Author        : George Vosselman
  Purpose       : Add label to topology structure
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define Name_H   "%c Line topology database : %s\n"
#define Num_H    "%c Number of lines : %d\n"
#define Line_H   "%c Linenumber    # of points\n"
#define Point_H  "%c Pointnumbers\n"
#define Sep_H    "%c------------- -------------- -------------- -------------- -------------- --------------\n"
#define FormatL  " %9d%15d%15d\n"
#define FormatP1 " %9d"
#define FormatP2 "%15d"
#define FormatP6 "%15d\n"

/*
----------------------------[ Get_LinesTops ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read line topologies into a structure of type "LineTops" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Image Lines
----------------------------[ Get_LineTops ]--------------------------
*/
LineTops *Get_LineTops(const char *filename)
{
  FILE *fp;                   /* File Pointer                      */
  int i, ipt;                 /* General counters                  */
  LineTops *lines;            /* The database that is constructed  */
  int *pts;                   /* Array with the points of a line   */
  int num_pts;                /* Number of points in a line        */
  char string[MAXCHARS];      /* String read from intputfile       */
  int file_id;                /* Constant that identifies the file */
  int label;                  /* Label of the line                 */


  if ((fp = Open_Compressed_File(filename, "r")) == NULL) {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
  }
  
/*------ Allocate memory for Lines structure -----------------------------*/
  lines = (LineTops *) malloc( sizeof(LineTops) );

/*------ Initiate the LineTops structure -----------------------------------*/
  strcpy(lines->name, filename);
  lines->num_lines = 0;
  
/*------ Check out the file's id -----------------------------------------*/
  fgets(string, MAXCHARS, fp);
  sscanf(string, "%d", &file_id);
  if (file_id != LINE_TOPOLOGY) {
    printf("Given file (%s) does not contain line topology data!\n", filename);
    return( NULL );
  }
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp)) {
    fgets(string, MAXCHARS, fp);

    if (!feof(fp) && !Is_Comment(string)) {
/*------ Create (extra) space for the line that is about to be read -----*/
      if (!lines->num_lines) 
        lines->lines = (LineTop *) malloc( sizeof(LineTop ) );
      else
        lines->lines = (LineTop *) realloc( lines->lines, 
                           (lines->num_lines + 1) * sizeof(LineTop) );

/*------ Extract the line number and the number of points from the string --*/
      label = 0;
      sscanf(string, "%d %d %d",
             &(lines->lines[lines->num_lines].num), &num_pts, &label);
      lines->lines[lines->num_lines].num_pts = num_pts;
      lines->lines[lines->num_lines].label = label;

/*------ Allocate space for the points in multiples of 6 -------------------*/
      pts = (int *) malloc( sizeof(int) * 6 * ((num_pts-1) / 6 + 1) );
      lines->lines[lines->num_lines].pts = pts;

/*------ Read the point records --------------------------------------------*/

      for (ipt=0; ipt<num_pts; ipt+=6, pts+=6) {
        fgets(string, MAXCHARS, fp);
        sscanf(string, "%d %d %d %d %d %d",
               pts, pts+1, pts+2, pts+3, pts+4, pts+5);
      }
      lines->num_lines++; 
    }
  }  
/*    
  lines->num_lines--; 
*/
/*------ Print out a message that we are done reading --------------------*/
  if (lines->num_lines == 0) lines->lines = NULL;
  printf( "Topology of %d lines read from %s.\n", lines->num_lines, filename);
  
  fclose(fp);
  
  return (lines);
}
  
/*
---------------------------------------------------------------------------
  Returns the difference between line numbers.
  Function is used for sorting the lines.
---------------------------------------------------------------------------
*/
int Compare_LineTops(const void *a, const void *b)
{
  const LineTop *al, *bl;
  
  al = (const LineTop *) a;
  bl = (const LineTop *) b;
  return( (al->num - bl->num) );
}

/*
------------------------------[ Put_LineTops ]------------------------------
  This function writes the LineTops structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read LineTops
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_LineTops ]------------------------------
*/
int Put_LineTops(LineTops *lines, const char *filename)
{
  FILE *fp;                 /* File Pointer                             */
  int i,j;                  /* General counter                          */
  LineTop *l;               /* Pointer to line                          */
  int *p;                   /* Pointer to the point numbers             */
  char image_name[256];     /* Name of the image that will be preserved */
  char string[256];         /* Input string from existing file          */
  
/*------ Open the file for writing ---------------------------------------*/
  
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write line topology database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on linenumber ----------------------------*/
  l = lines->lines;
  qsort(l, lines->num_lines, sizeof(LineTop), Compare_LineTops);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file ID\n", LINE_TOPOLOGY);
  fprintf( fp, Name_H,   comment_char, lines->name);
  fprintf( fp, Num_H,    comment_char, lines->num_lines);
  fprintf( fp, Line_H,   comment_char);
  fprintf( fp, Point_H,  comment_char);
  fprintf( fp, Sep_H,    comment_char);
  
/*------ Print the image points into the file ----------------------------*/
  for (i = 0; i < lines->num_lines; i++) {
    fprintf(fp, FormatL, l[i].num, l[i].num_pts, l[i].label);
    p = l[i].pts;
    for (j = 0; j < l[i].num_pts; j++, p++) {
      switch (j - (j/6)*6) {
        case (0) : fprintf(fp, FormatP1, *p);
                   break;
        case (5) : fprintf(fp, FormatP6, *p);
                   break;
        default  : fprintf(fp, FormatP2, *p);
      }
    }
    if ((l[i].num_pts/6)*6 != l[i].num_pts) fprintf(fp, "\n");
  }
    
/*------ Print out a message that we are done writing --------------------*/
  printf("Topology of %d lines written to %s.\n", lines->num_lines, filename);

  fclose(fp);

  return (True);
}

/*
------------------------------[ Print_LineTops ]----------------------------
  This function prints the LineTops structure to stdout.

  IN : Structure containing read LineTops 
  RETURN: Nothing
------------------------------[ Print_LineTops ]------------------------------
*/
void Print_LineTops(const LineTops *lines)
{
  int i, j;
  int *p;
  
  printf( Name_H,   ' ', lines->name);
  printf( Num_H,    ' ', lines->num_lines);
  printf( Line_H,   ' ');
  printf( Point_H,  ' ');
  printf( Sep_H,    ' ');
  
  for (i = 0; i < lines->num_lines; i++) {
    printf(FormatL, (lines->lines[i]).num, (lines->lines[i]).num_pts,
		    (lines->lines[i]).label);
    p = (lines->lines[i]).pts;
    for (j = 0; j < (lines->lines[i]).num_pts; j++, p++) {
      switch (j - (j/6)*6) {
        case (0) : printf(FormatP1, *p);
                   break;
        case (5) : printf(FormatP6, *p);
                   break;
        default  : printf(FormatP2, *p);
      }
    }
    if (((lines->lines[i]).num_pts/6)*6 != (lines->lines[i]).num_pts)
      printf("\n");
  }
}


/*
------------------------------[ Free_LineTops ]--------------------------------
  This function deallocates the memory used by LineTops.

  IN : Structure containing read LineTops 
  RETURN: Nothing
------------------------------[ Free_LineTops ]--------------------------------
*/

void Free_LineTops(LineTops *lines)
{
   int i;
   LineTop *plin;
   
   if (lines == NULL) return;
   plin = lines->lines;
   if (plin != NULL) {
     for(i = 0; i < lines->num_lines; i++, plin++) free(plin->pts);
     free(lines->lines);
   }
   free(lines);
   lines = NULL;   
}	
