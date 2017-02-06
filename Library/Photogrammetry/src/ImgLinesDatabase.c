
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

  Function      : Input and output routines for the database of 
                  image lines
  Author        : M.G. Vosselman
  Creation date : 28 - Aug - 1995
  Modification  : ?? - ??? - 199?
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define Name_H   "%c Image lines in image  : %s\n"
#define Num_H    "%c Number of image lines : %d\n"
#define Line_H   "%c Linenumber    # of points        label\n"
#define Point_H  "%c Pointnumber        row            col           var row         var col     covar row,col\n"
#define Sep_H    "%c------------- -------------- -------------- --------------- --------------- ---------------\n"
#define FormatL  " %9d%13d%15d\n"
#define FormatP  " %9d%16.3lf%15.3lf%18.5lg%16.5lg%16.5lg\n"

/*
----------------------------[ Get_ImgLines ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read Line into a structure of type "ImgLine" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Image Lines
----------------------------[ Get_ImgLines ]--------------------------
*/
ImgLines *Get_ImgLines(const char *filename)
{
  FILE *fp;                   /* File Pointer                      */
  int i, ipt;                 /* General counters                  */
  ImgLines *img_lines;        /* The database that is constructed  */
  ImgPt *pts;                 /* Array with the points of a line   */
  int num_pts;                /* Number of points in a line        */
  char string[MAXCHARS];      /* String read from intputfile       */
  int file_id;                /* Constant that identifies the file */
  double def_vr, def_vc;      /* Default values for variances      */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL) {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
  }
  
/*------ Allocate memory for Lines structure -----------------------------*/
  img_lines = (ImgLines *) malloc( sizeof(ImgLines) );

/*------ Initiate the ImgLines structure -----------------------------------*/
  strcpy(img_lines->img_name, filename);
  img_lines->num_lines = 0;
  
  def_vr = def_vc = -1.0;
  
/*------ Check out the file's id -----------------------------------------*/
  fgets(string, MAXCHARS, fp);
  sscanf(string, "%d", &file_id);
  if (file_id != IMAGE_LINES) {
    printf("Given file (%s) does not contain image lines!\n", filename);
    return( NULL );
  }
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp)) {
    fgets(string, MAXCHARS, fp);

    if (!Is_Comment(string)) {
/*------ Create (extra) space for the line that is about to be read -----*/
      if (!img_lines->num_lines) 
        img_lines->lines = (ImgLine *) malloc( sizeof(ImgLine ) );
      else
        img_lines->lines = (ImgLine *) realloc( img_lines->lines, 
                           (img_lines->num_lines + 1) * sizeof(ImgLine) );

/*------ Extract the line number and the number of points from the string --*/
      sscanf(string, "%d %d %d",
             &(img_lines->lines[img_lines->num_lines].num),
             &num_pts,
             &(img_lines->lines[img_lines->num_lines].label));
      img_lines->lines[img_lines->num_lines].num_pts = num_pts;

/*------ Allocate space for the points -------------------------------------*/
      pts = (ImgPt *) malloc( sizeof(ImgPt) * num_pts );
      img_lines->lines[img_lines->num_lines].pts = pts;

/*------ Read the image point records --------------------------------------*/

      for (ipt=0; ipt<num_pts; ipt++, pts++) {
        fgets(string, MAXCHARS, fp);

/*------ Extract the measured image point from the read string -------------*/
        pts->v_r   = pts->v_c = -1.0;
        pts->cv_rc = 0.0;
        sscanf(string, "%d %lf %lf %lf %lf %lf",
               &(pts->num),
               &(pts->r),
               &(pts->c), 
               &(pts->v_r),
               &(pts->v_c), 
               &(pts->cv_rc) );
      
/*------ Ask for default variances if they are not in the file ------*/
        if (def_vr < 0 && pts->v_r < 0 &&
  	    def_vc < 0 && pts->v_c < 0)
  	  printf("No variance factors specified in %s!\n", filename);
      
        if (def_vr < 0 && pts->v_r < 0) {
  	  printf("Give the default variance in row direction : ");
	  scanf("%lf", &def_vr);
	  pts->v_r = def_vr;
        }
             
        if (def_vc < 0 && pts->v_c < 0) {
  	  printf("Give the default variance in column direction : ");
	  scanf("%lf", &def_vc);
	  pts->v_c = def_vc;
        }
      
/*------ Filling the missing variances with the supplied defaults ------*/
        if (pts->v_r < 0) pts->v_r = def_vr;
        if (pts->v_c < 0) pts->v_c = def_vc;

      }
      img_lines->num_lines++; 
    }
  }  
    
  img_lines->num_lines--; 

/*------ Print out a message that we are done reading --------------------*/
  printf( "%d image lines read from %s.\n", img_lines->num_lines, filename);
  
  fclose(fp);
  
  return (img_lines);
}
  
/*
---------------------------------------------------------------------------
  Returns the difference between line numbers.
  Function is used for sorting the lines.
---------------------------------------------------------------------------
*/
int Compare_ImgLines(const void *a, const void *b)
{ 
  const ImgLine *al, *bl;

  al = (const ImgLine *) a;
  bl = (const ImgLine *) b;
  return( (al->num - bl->num) );
}

/*
------------------------------[ Put_ImgLines ]------------------------------
  This function writes the ImgLines structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read ImgLines
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ImgLines ]------------------------------
*/
int Put_ImgLines(ImgLines *img_lines, const char *filename)
{
  FILE *fp;                 /* File Pointer                             */
  int i,j;                  /* General counter                          */
  ImgLine *l;               /* Pointer to image line                    */
  ImgPt *p;                 /* Pointer to the measured coordinates      */
  char image_name[256];     /* Name of the image that will be preserved */
  char string[256];         /* Input string from existing file          */
  
  FILE *Open_Compressed_File(const char *, const char *);
  int Is_Comment(const char *);

/*------ Open the file for writing ---------------------------------------*/
  strcpy(image_name, "");
  
  if ((fp = Open_Compressed_File(filename, "r")) != NULL)
    {
    while (!feof(fp))
      {
      fgets(string, MAXCHARS, fp);
      if (Is_Comment(string) && strstr(string, "lines in image") != 0)
        strcpy(image_name, string);
      }
    fclose(fp);
    }

  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write image lines database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  l = img_lines->lines;
  qsort(l, img_lines->num_lines, sizeof( ImgLine), Compare_ImgLines);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file ID\n", IMAGE_LINES);
  if (strlen(image_name) > 0) fprintf( fp, "%s", image_name);
  else fprintf( fp, Name_H,   comment_char, img_lines->img_name);
  fprintf( fp, Num_H,    comment_char, img_lines->num_lines);
  fprintf( fp, Line_H,   comment_char);
  fprintf( fp, Point_H,  comment_char);
  fprintf( fp, Sep_H,    comment_char);
  
/*------ Print the image points into the file ----------------------------*/
  for (i = 0; i < img_lines->num_lines; i++) {
    fprintf(fp, FormatL, l[i].num, l[i].num_pts, l[i].label);
    p = l[i].pts;
    for (j = 0; j < l[i].num_pts; j++, p++)
      fprintf( fp, FormatP, p->num, p->r, p->c, p->v_r, p->v_c, p->cv_rc);
  }
    
/*------ Print out a message that we are done writing --------------------*/
  printf("%d image lines written to %s.\n", img_lines->num_lines, filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_ImgLines ]----------------------------
  This function prints the ImgLines structure to stdout.

  IN : Structure containing read ImgLines 
  RETURN: Nothing
------------------------------[ Print_ImgLines ]------------------------------
*/
void Print_ImgLines(const ImgLines *img_lines)
{
  int i, j;
  ImgPt *p;
  
  printf( Name_H,   ' ', img_lines->img_name);
  printf( Num_H,    ' ', img_lines->num_lines);
  printf( Line_H,   ' ');
  printf( Point_H,  ' ');
  printf( Sep_H,    ' ');
  
  for (i = 0; i < img_lines->num_lines; i++) {
    printf(FormatL, (img_lines->lines[i]).num,
                    (img_lines->lines[i]).num_pts,
                    (img_lines->lines[i]).label);
    p = (img_lines->lines[i]).pts;
    for (j = 0; j < (img_lines->lines[i]).num_pts; j++, p++)
      printf(FormatP, p->num, 
                      p->r, 
                      p->c,
                      p->v_r, 
                      p->v_c,
                      p->cv_rc);
  }
}

/*
------------------------------[ Free_ImgLines ]--------------------------------
  This function deallocates the memory used by ImgLines.

  IN : Structure containing read ImgLines 
  RETURN: Nothing
------------------------------[ Free_ImgLines ]--------------------------------
*/

void Free_ImgLines(ImgLines *imglins)
{
   int i;
   ImgLine *plin;
   
   plin = imglins->lines;
   for(i = 0; i < imglins->num_lines; i++, plin++)
      free(plin->pts);
   free(imglins->lines);
   free(imglins);
   imglins = NULL;   
}	
