
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
                  photocoordinates
  Author        : R.Th. Ursem
  Creation date : 23 - Nov - 1994
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define Num_H    "%c Number of camera points : %d\n"
#define Pixel_H  "%c Pointnumber         x              y            var  x          var  y      covar  x , y \n"
#define Line_H   "%c------------- -------------- -------------- --------------- --------------- ---------------\n"
#define Format   " %9d%16.5lf%15.5lf%18.5lg%16.5lg%16.5lg\n"

/*
----------------------------[ Get_CamPts ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read point into a structure of type "CamPts" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read camera Points
----------------------------[ Get_CamPts ]----------------------------
*/
CamPts *Get_CamPts(const char *filename)
  {
  FILE *fp;                   /* File Pointer                      */
  int i;                      /* General counter                   */
  CamPts *cam_pts;            /* The database that is constructed  */
  char line[MAXCHARS];        /* Line read from intputfile         */
  int file_id;                /* Constant that identifies the file */
  double def_vx, def_vy;      /* Default values for variances      */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for Pixel structure -----------------------------*/
  cam_pts = (CamPts *)malloc( sizeof(CamPts) );

/*------ Initiate the CamPts structure -----------------------------------*/
  cam_pts->num_pts = 0;
  
  def_vx = def_vy = -1.0;
  
/*------ Check out the file's id -----------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != CAMERA_POINTS)
    {
    printf("Given file (%s) does not contain camera points!\n", filename);
    return( NULL );
    }
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the Pixel that is about to be read -----*/
      if (!cam_pts->num_pts) 
        cam_pts->pts = (CamPt *) malloc( sizeof(CamPt ) );
      else
        cam_pts->pts = (CamPt *) realloc( cam_pts->pts, 
                          (cam_pts->num_pts + 1) * sizeof(CamPt) );

      cam_pts->pts[cam_pts->num_pts].v_x   =
      cam_pts->pts[cam_pts->num_pts].v_y   = -1.0;
      cam_pts->pts[cam_pts->num_pts].cv_xy = 0.0;
      
/*------ Extract the measured image point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %lf %lf",
             &(cam_pts->pts[cam_pts->num_pts].num),
             &(cam_pts->pts[cam_pts->num_pts].x),
             &(cam_pts->pts[cam_pts->num_pts].y), 
             &(cam_pts->pts[cam_pts->num_pts].v_x),
             &(cam_pts->pts[cam_pts->num_pts].v_y), 
             &(cam_pts->pts[cam_pts->num_pts].cv_xy) );
      
/*------ Ask for default variances if they are not in the file ------*/
      if (def_vx < 0 && cam_pts->pts[cam_pts->num_pts].v_x < 0 &&
	  def_vy < 0 && cam_pts->pts[cam_pts->num_pts].v_y < 0)
	printf("No variance factors specified in %s!\n", filename);
      
      if (def_vx < 0 && cam_pts->pts[cam_pts->num_pts].v_x < 0)
      {
	printf("Give the default variance in row direction : ");
	scanf("%lf", &def_vx);
	cam_pts->pts[cam_pts->num_pts].v_x = def_vx;
      }
             
      if (def_vy < 0 && cam_pts->pts[cam_pts->num_pts].v_y < 0)
      {
	printf("Give the default variance in column direction : ");
	scanf("%lf", &def_vy);
	cam_pts->pts[cam_pts->num_pts].v_y = def_vy;
      }
      
/*------ Filling the missing variances with the supplied defaults ------*/
      if (cam_pts->pts[cam_pts->num_pts].v_x < 0)
	 cam_pts->pts[cam_pts->num_pts].v_x = def_vx;
      
      if (cam_pts->pts[cam_pts->num_pts].v_y < 0)
	 cam_pts->pts[cam_pts->num_pts].v_y = def_vy;

      cam_pts->num_pts++; 
      }
    }  
    
  cam_pts->num_pts--; 

/*------ Print out a message that we are done reading --------------------*/
  printf( "%d camera points read from %s.\n", cam_pts->num_pts, filename);
  
  fclose(fp);
  
  return (cam_pts);
  }
  
/*
---------------------------------------------------------------------------
  Returns True if the string is a comment string (if first non space is
  equal to the character defined in comment_char) otherwise returns false
---------------------------------------------------------------------------
*/
int Compare_CamPts(const void *a, const void *b)
{
  CamPt *ap, *bp;

  ap = (CamPt *) a;
  bp = (CamPt *) b;
  return( (ap->num - bp->num) );
}

/*
------------------------------[ Put_CamPts ]--------------------------------
  This function writes the CamPts structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read CamPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_CamPts ]--------------------------------
*/
int Put_CamPts(CamPts *cam_pts, const char *filename)
  {
  FILE *fp;                 /* File Pointer                             */
  int i;                    /* General counter                          */
  CamPt *p;                 /* Pointer to the measured coordinates      */
  char image_name[256];     /* Name of the image that will be preserved */
  char line[256];           /* Input line from existing file            */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write image point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = cam_pts->pts;
  qsort((void *) p, cam_pts->num_pts, sizeof( CamPt), Compare_CamPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file ID\n", CAMERA_POINTS);
  fprintf( fp, Num_H,    comment_char, cam_pts->num_pts);
  fprintf( fp, Pixel_H,  comment_char);
  fprintf( fp, Line_H,   comment_char);
  
/*------ Print the image points into the file ----------------------------*/
  for (i = 0; i < cam_pts->num_pts; i++)
    if (p[i].x != -1.0 && p[i].y != -1.0)
      fprintf( fp, Format, p[i].num, p[i].x, p[i].y, p[i].v_x, p[i].v_y, p[i].cv_xy);
    
/*------ Print out a message that we are done writing --------------------*/
  printf("%d camera points written to %s.\n", cam_pts->num_pts, filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_CamPts ]------------------------------
  This function prints the CamPts structure to stdout.

  IN : Structure containing read CamPts 
  RETURN: Nothing
------------------------------[ Print_CamPts ]------------------------------
*/
void Print_CamPts(const CamPts *cam_pts)
  {
  int i;
  
  printf( Num_H,    ' ', cam_pts->num_pts);
  printf( Pixel_H,  ' ');
  printf( Line_H,   ' ');
  
  for (i = 0; i < cam_pts->num_pts; i++)
    printf(Format, (cam_pts->pts[i]).num, 
                   (cam_pts->pts[i]).x, 
                   (cam_pts->pts[i]).y,
                   (cam_pts->pts[i]).v_x, 
                   (cam_pts->pts[i]).v_y,
                   (cam_pts->pts[i]).cv_xy);
  }

/*
------------------------------[ Free_CamPts ]--------------------------------
  This function deallocates the memory used by CamPts.

  IN : Structure containing read CamPts 
  RETURN: Nothing
------------------------------[ Free_CamPts ]--------------------------------
*/

void Free_CamPts(CamPts *campts)
{
   free(campts->pts);
   free(campts);
   campts = NULL;
}	
