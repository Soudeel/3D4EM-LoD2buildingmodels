
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
                  measured pixelcoordinates
  Author        : R.Th. Ursem
  Creation date : 19 - July - 1994
  Modification  :  4 -  Aug - 1994
      - Sorted writing of the database
      - Checking of double entries
      - functions for searching points added
  Modification  :  4 -  Nov - 1994
      - Kept comments when writing to an existing file
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define Name_H   "%c Image points in image  : %s\n"
#define Num_H    "%c Number of image points : %d\n"
#define Pixel_H  "%c Pointnumber        row            col           var row         var col     covar row,col\n"
#define Line_H   "%c------------- -------------- -------------- --------------- --------------- ---------------\n"
#define Format   " %9d%16.3lf%15.3lf%18.5lg%16.5lg%16.5lg\n"

/*
---------------------------------------------------------------------------
  This function deletes an observation from the given database by moving
  the last observation to the given index, and then freeing the last 
  observation.
  
  IN : database to update
  IN : number of the point that is to be deleted
  
  RETURN : True if succesful, false if not. 
---------------------------------------------------------------------------
*/
int Delete_Pix(ImgPts *img_pts, int number)
  {
  ImgPt *p;
  int index, i;
  
  p = img_pts->pts;

/*------ Search for the index to go with this number ------*/
  for (index = 0; index < img_pts->num_pts; index++)
    if (p[index].num == number) break;
 
/*------ Decrease the number of measured points ------*/
  img_pts->num_pts--;
  
/*------ Copy the last entry to the given index ------*/
  p[index] = p[img_pts->num_pts];
  
/*------ Free the last entry ------*/
  img_pts->pts = (ImgPt *) realloc( img_pts->pts, 
                 (img_pts->num_pts + 1) * sizeof(ImgPt) );

  return( True );
  }

/*
---------------------------------------------------------------------------
  Searches the database for information. if img_pt->num == -1 then it is 
  looking for the point with the given coordinates. Otherwise it is looking
  for the point with the given point number.
  
  IN : query structure containing search information
  IN : database to search for the point
  
  RETURN : True if succesful, False if not
---------------------------------------------------------------------------
*/
ImgPt *Search_ImgPts(ImgPt *img_pt, ImgPts *img_pts)
{
  register int i;
  ImgPt *p;
  
  if (img_pts == NULL || img_pt == NULL) return( NULL );
  
  p = img_pts->pts;

/* Look for coordinates */

  if (img_pt->num == -1) {
    for (i= 0; i < img_pts->num_pts; i++, p++) {
      if (img_pt->r == p->r && img_pt->c == p->c) {
        img_pt->num = p->num;
        return( p );
      }
    }
  }

/* Look for point number */

  else {
    for (i= 0; i < img_pts->num_pts; i++, p++) {
      if (img_pt->num == p->num) {
        img_pt->r = p->r;
        img_pt->c = p->c;
        return( p );
      }
    }
  }

/* Point not found */

  return(NULL);
}
  
/*
---------------------------------------------------------------------------
  Returns a constant indicating if the given image point already exists 
  in the given database.
  
  IN : query structure containing the coordinates and the pointnumber
       of the point that is searched
  IN : database to search for the point
  
  RETURN : IMGPT_NOT_KNOWN, IMGPT_NUM_EXIST or IMGPT_COORD_EXIST 
---------------------------------------------------------------------------
*/
int ImgPts_Exist(ImgPt *img_pt, ImgPts *img_pts)
{
  register int i;
  ImgPt *p;
  
  if (img_pts == NULL || img_pt == NULL)
    return( IMGPT_NOT_KNOWN );
  
  p = img_pts->pts;
  for (i= 0; i < img_pts->num_pts; i++, p++)
  {
    if (p->num == img_pt->num) 
      return( IMGPT_NUM_EXIST );
    else if (p->r == img_pt->r && p->c == img_pt->c) 
      return( IMGPT_COORD_EXIST );
  }
  
  return( IMGPT_NOT_KNOWN );
}

/*
----------------------------[ Get_ImgPts ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read Pixel into a structure of type "ImgPts" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Image Points
----------------------------[ Get_ImgPts ]----------------------------
*/
ImgPts *Get_ImgPts(const char *filename)
{
  FILE *fp;                   /* File Pointer                      */
  int i;                      /* General counter                   */
  ImgPts *img_pts;            /* The database that is constructed  */
  char line[MAXCHARS];        /* Line read from intputfile         */
  int file_id;                /* Constant that identifies the file */
  double def_vr, def_vc;      /* Default values for variances      */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
    }

/*------ Allocate memory for Pixel structure -----------------------------*/
  img_pts = (ImgPts *) malloc( sizeof(ImgPts) );

/*------ Initiate the ImgPts structure -----------------------------------*/
  strcpy(img_pts->img_name, filename);
  img_pts->num_pts = 0;
  
  def_vr = def_vc = -1.0;
  
/*------ Check out the file's id -----------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != IMAGE_POINTS)
    {
    printf("Given file (%s) does not contain image points!\n", filename);
    return( NULL );
    }
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the Pixel that is about to be read -----*/
      if (!img_pts->num_pts) 
        img_pts->pts = (ImgPt *) malloc( sizeof(ImgPt ) );
      else
        img_pts->pts = (ImgPt *) realloc( img_pts->pts, 
                          (img_pts->num_pts + 1) * sizeof(ImgPt) );

      img_pts->pts[img_pts->num_pts].v_r =
      img_pts->pts[img_pts->num_pts].v_c = -1.0;
      img_pts->pts[img_pts->num_pts].cv_rc = 0.0;
      
/*------ Extract the measured image point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %lf %lf",
             &(img_pts->pts[img_pts->num_pts].num),
             &(img_pts->pts[img_pts->num_pts].r),
             &(img_pts->pts[img_pts->num_pts].c), 
             &(img_pts->pts[img_pts->num_pts].v_r),
             &(img_pts->pts[img_pts->num_pts].v_c), 
             &(img_pts->pts[img_pts->num_pts].cv_rc) );
      
/*------ Ask for default variances if they are not in the file ------*/
      if (def_vr < 0 && img_pts->pts[img_pts->num_pts].v_r < 0 &&
	  def_vc < 0 && img_pts->pts[img_pts->num_pts].v_c < 0)
	printf("No variance factors specified in %s!\n", filename);
      
      if (def_vr < 0 && img_pts->pts[img_pts->num_pts].v_r < 0)
      {
	printf("Give the default variance in row direction : ");
	scanf("%lf", &def_vr);
	img_pts->pts[img_pts->num_pts].v_r = def_vr;
      }
             
      if (def_vc < 0 && img_pts->pts[img_pts->num_pts].v_c < 0)
      {
	printf("Give the default variance in column direction : ");
	scanf("%lf", &def_vc);
	img_pts->pts[img_pts->num_pts].v_c = def_vc;
      }
      
/*------ Filling the missing variances with the supplied defaults ------*/
      if (img_pts->pts[img_pts->num_pts].v_r < 0)
	img_pts->pts[img_pts->num_pts].v_r = def_vr;
      
      if (img_pts->pts[img_pts->num_pts].v_c < 0)
	img_pts->pts[img_pts->num_pts].v_c = def_vc;

      img_pts->num_pts++; 
      }
    }  
    
  img_pts->num_pts--; 

/*------ Print out a message that we are done reading --------------------*/
  printf( "%d image points read from %s.\n", img_pts->num_pts, filename);
  
  fclose(fp);
  
  return (img_pts);
}
  
/*
---------------------------------------------------------------------------
  Returns True if the string is a comment string (if first non space is
  equal to the character defined in comment_char) otherwise returns false
---------------------------------------------------------------------------
*/
int Compare_ImgPts(const void *a, const void *b)
{
  const ImgPt *ap, *bp;

  ap = (const ImgPt *) a;
  bp = (const ImgPt *) b;
  return( (ap->num - bp->num) );
}

/*
------------------------------[ Put_ImgPts ]--------------------------------
  This function writes the ImgPts structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read ImgPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ImgPts ]--------------------------------
*/
int Put_ImgPts(ImgPts *img_pts, const char *filename)
  {
  FILE *fp;                 /* File Pointer                             */
  int i;                    /* General counter                          */
  ImgPt *p;                 /* Pointer to the measured coordinates      */
  char image_name[256];     /* Name of the image that will be preserved */
  char line[256];           /* Input line from existing file            */
  
  FILE *Open_Compressed_File(const char *, const char *);
  int Is_Comment(const char *);

/*------ Open the file for writing ---------------------------------------*/
  strcpy(image_name, "");
  
  if ((fp = Open_Compressed_File(filename, "r")) != NULL)
    {
    while (!feof(fp))
      {
      fgets(line, MAXCHARS, fp);
      if (Is_Comment(line) && strstr(line, "points in image") != 0)
        strcpy(image_name, line);
      }
    fclose(fp);
    }

  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write image point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = img_pts->pts;
  qsort((void *) p, img_pts->num_pts, sizeof( ImgPt), Compare_ImgPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file ID\n", IMAGE_POINTS);
  if (strlen(image_name) > 0) fprintf( fp, "%s", image_name);
  else fprintf( fp, Name_H,   comment_char, img_pts->img_name);
  fprintf( fp, Num_H,    comment_char, img_pts->num_pts);
  fprintf( fp, Pixel_H,  comment_char);
  fprintf( fp, Line_H,   comment_char);
  
/*------ Print the image points into the file ----------------------------*/
  for (i = 0; i < img_pts->num_pts; i++)
    if (p[i].r != -1.0 && p[i].c != -1.0)
      fprintf( fp, Format, p[i].num, p[i].r, p[i].c, p[i].v_r, p[i].v_c, p[i].cv_rc);
    
/*------ Print out a message that we are done writing --------------------*/
  printf("%d image points written to %s.\n", img_pts->num_pts, filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_ImgPts ]------------------------------
  This function prints the ImgPts structure to stdout.

  IN : Structure containing read ImgPts 
  RETURN: Nothing
------------------------------[ Print_ImgPts ]------------------------------
*/
void Print_ImgPts(const ImgPts *img_pts)
  {
  int i;
  
  printf( Name_H,   ' ', img_pts->img_name);
  printf( Num_H,    ' ', img_pts->num_pts);
  printf( Pixel_H,  ' ');
  printf( Line_H,   ' ');
  
  for (i = 0; i < img_pts->num_pts; i++)
    printf(Format, (img_pts->pts[i]).num, 
                   (img_pts->pts[i]).r, 
                   (img_pts->pts[i]).c,
                   (img_pts->pts[i]).v_r, 
                   (img_pts->pts[i]).v_c,
                   (img_pts->pts[i]).cv_rc);
  }

/*
------------------------------[ Free_ImgPts ]--------------------------------
  This function deallocates the memory used by ImgPts.

  IN : Structure containing read ImgPts 
  RETURN: Nothing
------------------------------[ Free_ImgPts ]--------------------------------
*/

void Free_ImgPts(ImgPts *imgpts)
{
   free(imgpts->pts);
   free(imgpts);
   imgpts = NULL;
}	
