
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
                  2D object points
  Author        : George Vosselman
  Creation date : 26 - Sep - 1997
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define Num_H    "%c Number of 2D object points : %d\n"
#define Point_H  "%c Pointnumber          x               y            var  x          var  y      covar  x , y \n"
#define Line_H   "%c------------- --------------- --------------- --------------- --------------- ---------------\n"
#define Format   " %9d%17.5lf%17.5lf%18.5lg%16.5lg%16.5lg\n"

/*
----------------------------[ Get_ObjPts2D ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read point into a structure of type "ObjPts2D" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read 2D object Points
----------------------------[ Get_ObjPts2D ]----------------------------
*/
ObjPts2D *Get_ObjPts2D(const char *filename)
  {
  FILE *fp;                   /* File Pointer                      */
  int i;                      /* General counter                   */
  ObjPts2D *objpts2d;            /* The database that is constructed  */
  char line[MAXCHARS];        /* Line read from intputfile         */
  int file_id;                /* Constant that identifies the file */
  double def_vx, def_vy;      /* Default values for variances      */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for point structure -----------------------------*/
  objpts2d = (ObjPts2D *)malloc( sizeof(ObjPts2D) );

/*------ Initiate the ObjPts2D structure -----------------------------------*/
  objpts2d->num_pts = 0;
  
  def_vx = def_vy = -1.0;
  
/*------ Check out the file's id -----------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != OBJECT_POINTS_2D)
    {
    printf("Given file (%s) does not contain 2D object points!\n", filename);
    return( NULL );
    }
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the point that is about to be read -----*/
      if (!objpts2d->num_pts) 
        objpts2d->pts = (ObjPt2D *) malloc( sizeof(ObjPt2D ) );
      else
        objpts2d->pts = (ObjPt2D *) realloc( objpts2d->pts, 
                          (objpts2d->num_pts + 1) * sizeof(ObjPt2D) );

      objpts2d->pts[objpts2d->num_pts].v_x   =
      objpts2d->pts[objpts2d->num_pts].v_y   = -1.0;
      objpts2d->pts[objpts2d->num_pts].cv_xy = 0.0;
      
/*------ Extract the 2D object point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %lf %lf",
             &(objpts2d->pts[objpts2d->num_pts].num),
             &(objpts2d->pts[objpts2d->num_pts].x),
             &(objpts2d->pts[objpts2d->num_pts].y), 
             &(objpts2d->pts[objpts2d->num_pts].v_x),
             &(objpts2d->pts[objpts2d->num_pts].v_y), 
             &(objpts2d->pts[objpts2d->num_pts].cv_xy) );
      
/*------ Ask for default variances if they are not in the file ------*/
      if (def_vx < 0 && objpts2d->pts[objpts2d->num_pts].v_x < 0 &&
	  def_vy < 0 && objpts2d->pts[objpts2d->num_pts].v_y < 0)
	printf("No variance factors specified in %s!\n", filename);
      
      if (def_vx < 0 && objpts2d->pts[objpts2d->num_pts].v_x < 0)
      {
	printf("Give the default variance in x direction : ");
	scanf("%lf", &def_vx);
	objpts2d->pts[objpts2d->num_pts].v_x = def_vx;
      }
             
      if (def_vy < 0 && objpts2d->pts[objpts2d->num_pts].v_y < 0)
      {
	printf("Give the default variance in y direction : ");
	scanf("%lf", &def_vy);
	objpts2d->pts[objpts2d->num_pts].v_y = def_vy;
      }
      
/*------ Filling the missing variances with the supplied defaults ------*/
      if (objpts2d->pts[objpts2d->num_pts].v_x < 0)
	 objpts2d->pts[objpts2d->num_pts].v_x = def_vx;
      
      if (objpts2d->pts[objpts2d->num_pts].v_y < 0)
	 objpts2d->pts[objpts2d->num_pts].v_y = def_vy;

      objpts2d->num_pts++; 
      }
    }  
    
  objpts2d->num_pts--; 

  fclose(fp);
  
  return (objpts2d);
  }
  
/*
---------------------------------------------------------------------------
  Returns True if the string is a comment string (if first non space is
  equal to the character defined in comment_char) otherwise returns false
---------------------------------------------------------------------------
*/
int Compare_ObjPts2D(const void *a, const void *b)
{
  ObjPt2D *ap, *bp;

  ap = (ObjPt2D *) a;
  bp = (ObjPt2D *) b;
  return( (ap->num - bp->num) );
}

/*
------------------------------[ Put_ObjPts2D ]--------------------------------
  This function writes the ObjPts2D structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read ObjPts2D
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ObjPts2D ]--------------------------------
*/
int Put_ObjPts2D(ObjPts2D *objpts2d, const char *filename)
  {
  FILE *fp;                 /* File Pointer                             */
  int i;                    /* General counter                          */
  ObjPt2D *p;               /* Pointer to the measured coordinates      */
  char image_name[256];     /* Name of the image that will be preserved */
  char line[256];           /* Input line from existing file            */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write 2D point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = objpts2d->pts;
  qsort((void *) p, objpts2d->num_pts, sizeof( ObjPt2D), Compare_ObjPts2D);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file ID\n", OBJECT_POINTS_2D);
  fprintf( fp, Num_H,    comment_char, objpts2d->num_pts);
  fprintf( fp, Point_H,  comment_char);
  fprintf( fp, Line_H,   comment_char);
  
/*------ Print the 2D points into the file ----------------------------*/
  for (i = 0; i < objpts2d->num_pts; i++)
    if (p[i].x != -1.0 && p[i].y != -1.0)
      fprintf( fp, Format, p[i].num, p[i].x, p[i].y, p[i].v_x, p[i].v_y, p[i].cv_xy);
    
  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_ObjPts2D ]------------------------------
  This function prints the ObjPts2D structure to stdout.

  IN : Structure containing read ObjPts2D 
  RETURN: Nothing
------------------------------[ Print_ObjPts2D ]------------------------------
*/
void Print_ObjPts2D(const ObjPts2D *objpts2d)
  {
  int i;
  
  printf( Num_H,    ' ', objpts2d->num_pts);
  printf( Point_H,  ' ');
  printf( Line_H,   ' ');
  
  for (i = 0; i < objpts2d->num_pts; i++)
    printf(Format, (objpts2d->pts[i]).num, 
                   (objpts2d->pts[i]).x, 
                   (objpts2d->pts[i]).y,
                   (objpts2d->pts[i]).v_x, 
                   (objpts2d->pts[i]).v_y,
                   (objpts2d->pts[i]).cv_xy);
  }


/*
------------------------------[ Free_ObjPts2D ]------------------------------
  This function deallocates the memory used by ObjPts2D.

  IN : Structure containing read ObjPts2D 
  RETURN: Nothing
------------------------------[ Free_ObjPts2D ]------------------------------
*/

void    Free_ObjPts2D(ObjPts2D *objpts2d)
{
   free(objpts2d->pts);
   free(objpts2d);
   objpts2d = NULL;
}	
