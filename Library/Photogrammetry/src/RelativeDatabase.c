
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
                                 ( O-O )
------------------------------o00--(_)--00o---------------------------------

  Function      : Input and output routines for relative orientation data
  Author        : R.Th. Ursem
  Creation date : 04 - Nov - 1994
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c Relative orientation elements\n\n"
#define ROT "%c The rotation matrix from image space to model space\n\
%c ----------------------------------------------------------\n"
#define POS "\n%c The position of the projection centre in model space\n\
%c ----------------------------------------------------------\n"

/*
----------------------------[ Get_Relative ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Exterior" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_Relative ]--------------------------
*/
int Get_Relative(const char *filename, Exterior *cam_1, Exterior *cam_2)
{
  FILE *fp;                   /* File Pointer                     */
  int i, j;                   /* counters                         */
  Exterior *exterior;         /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* File identifier                  */
  int  error;                 /* Returned error status            */

  error = ALL_VALID;
  
  if ((fp = Open_Compressed_File(filename, "r")) == NULL) {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (0);
  }
  
/*------ Print out a message that we are busy reading --------------------*/
  printf( "Reading relative orientation elements from %s\n", filename);

/*------ Allocate memory for the structure -------------------------------*/
  if (cam_1 == NULL) cam_1 = (Exterior *)malloc( sizeof(Exterior) );
  if (cam_2 == NULL) cam_2 = (Exterior *)malloc( sizeof(Exterior) );

/*------ Read the file ---------------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != RELATIVE_ORIENTATION)
    {
    printf("   Error in file identifier\n");
    printf("   This file ( %s ) does not contain relative orientation data\n", filename);
    return( 0 );
    }

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
      if (i < 3)
        sscanf( line, "%lf %lf %lf", &(cam_1->rot[i][0]), 
                                     &(cam_1->rot[i][1]), 
                                     &(cam_1->rot[i][2]) );
      else if (i == 3)
        {
        sscanf( line, "%lf %lf %lf", &(cam_1->x), &(cam_1->y), &(cam_1->z));
	break;
	}
      i++;
      }
    }  
    
  i = 0;
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
      if (i < 3)
        sscanf( line, "%lf %lf %lf", &(cam_2->rot[i][0]), 
                                     &(cam_2->rot[i][1]), 
                                     &(cam_2->rot[i][2]) );
      else if (i == 3)
        sscanf( line, "%lf %lf %lf", &(cam_2->x), &(cam_2->y), &(cam_2->z));
      i++;
      }
    }  
    
  fclose(fp);

  printf("   Calculating exterior orientation elements of camera 1\n"); 
  if (!Angles_From_Rot(cam_1)) error |= INVALID_ANGLES;
  if (!Quat_From_Rot(cam_1))   error |= INVALID_QUATERNION;
   
  printf("   Calculating exterior orientation elements of camera 2\n"); 
  if (!Angles_From_Rot(cam_2)) error |= INVALID_ANGLES;
  if (!Quat_From_Rot(cam_2))   error |= INVALID_QUATERNION;
  
/*------ Print out a message that we are done reading --------------------*/
  printf( "Done reading relative orientation elements.\n");
  
  return (error);
  }
  
/*
------------------------------[ Put_Relative ]--------------------------------
  This function writes the Exterior structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read Exterior
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Relative ]--------------------------------
*/
int Put_Relative(Exterior *cam_1, Exterior *cam_2, const char *filename)
{
  FILE *fp;                        /* File Pointer                        */
  int i;

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write relative orientation elements.\n", 
        filename);
    return (False);
    }
    
/*------ Print out a message that we are busy writing --------------------*/
  fprintf( stdout, "Writing relative orientation elements to %s\n", filename);

/*------ Print the headers and the information in the database file ------*/
  fprintf( fp, "%d # file identifier\n", RELATIVE_ORIENTATION);
  fprintf( fp, HEADER, comment_char);
  
/*------ First camera ------*/
  fprintf( fp, "# First camera\n");
  fprintf( fp, ROT, comment_char, comment_char);
  for (i = 0; i < 3; i++)
    fprintf( fp, "%20.16lf%20.16lf%20.16lf\n", cam_1->rot[i][0], 
                                               cam_1->rot[i][1], 
                                               cam_1->rot[i][2]);

  fprintf( fp, POS, comment_char, comment_char);
  fprintf( fp, "%20.7lf%20.7lf%20.7lf\n", cam_1->x, cam_1->y, cam_1->z);
  
/*------ Second camera ------*/
  fprintf( fp, "\n# Second camera\n");
  fprintf( fp, ROT, comment_char, comment_char);
  for (i = 0; i < 3; i++)
    fprintf( fp, "%20.16lf%20.16lf%20.16lf\n", cam_2->rot[i][0], 
                                               cam_2->rot[i][1], 
                                               cam_2->rot[i][2]);

  fprintf( fp, POS, comment_char, comment_char);
  fprintf( fp, "%20.7lf%20.7lf%20.7lf\n", cam_2->x, cam_2->y, cam_2->z);
  
/*------ Print out a message that we are done writing --------------------*/
  fprintf( stdout, "Done writing relative orientation elements.\n");

  fclose(fp);

  return (True);
  }
