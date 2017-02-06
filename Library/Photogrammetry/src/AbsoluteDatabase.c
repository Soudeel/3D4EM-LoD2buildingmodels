
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

  Function      : Input and output routines for absolute orientation data
  Author        : R.Th. Ursem
  Creation date : 27 - Mrt - 1994
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c absolute orientation elements\n\n"
#define ROT "%c The rotation matrix from model space to object space\n\
%c ----------------------------------------------------------\n"
#define POS "\n%c Translation from model space to object space\n\
%c ----------------------------------------------------------\n"
#define LAMBDA "\n%c Scale factor from model space to object space\n\
%c ----------------------------------------------------------\n"
#define EPSILON 0.000000000001

/*
----------------------------[ Get_Absolute ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Absolute" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read parameters
----------------------------[ Get_Absolute ]--------------------------
*/
Absolute *Get_Absolute(const char *filename, int *error)
  {
  FILE *fp;                   /* File Pointer                     */
  int i, j;                   /* counters                         */
  Absolute *absolute;         /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* File identifier                  */

  *error = ALL_VALID;
  
  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for structure -----------------------------------*/
  absolute = (Absolute *)malloc( sizeof(Absolute) );

/*------ Read the file ---------------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != ABSOLUTE_ORIENTATION)
    {
    printf("Error in file identifier\n");
    printf("This file ( %s ) does not contain absolute orientation data\n", filename);
    return( NULL );
    }

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
	 switch (i) {
	    case 0:
	    case 1:
	    case 2:
	       sscanf( line, "%lf %lf %lf", &(absolute->rot[i][0]), 
		      &(absolute->rot[i][1]), 
		      &(absolute->rot[i][2]) );
	       break;
	    case 3:
	       sscanf( line, "%lf %lf %lf", &(absolute->x), &(absolute->y), &(absolute->z));
	       break;
	    case 4:
	       sscanf( line, "%lf", &(absolute->lambda) );
	       break;
	 }
      i++;
      }
    }  
    
  fclose(fp);

  if (!Angles_From_Rot((Exterior *) absolute)) *error |= INVALID_ANGLES;
  if (!Quat_From_Rot((Exterior *) absolute))   *error |= INVALID_QUATERNION;
  
/*------ Update the status byte in the structure ------*/
  absolute->status = *error;
  
/*------ Print out a message that we are done reading --------------------*/
  printf( "Absolute orientation elements read from %s.\n", filename);
  
  return (absolute);
  }
  
/*
------------------------------[ Put_Absolute ]--------------------------------
  This function writes the Absolute structure into a file.
  
  IN : Structure containing read Absolute
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Absolute ]--------------------------------
*/
int Put_Absolute(Absolute *absolute, const char *filename)
  {
  FILE *fp;                        /* File Pointer                        */
  int i;

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write absolute orientation elements database.\n", 
        filename);
    return (False);
    }
    
/*------ Print the headers and the information in the database file ------*/
  fprintf( fp, "%d # file identifier\n", ABSOLUTE_ORIENTATION);
  fprintf( fp, HEADER, comment_char);
  
  fprintf( fp, ROT, comment_char, comment_char);
  for (i = 0; i < 3; i++)
    fprintf( fp, "%20.16lf%20.16lf%20.16lf\n", absolute->rot[i][0], 
                                               absolute->rot[i][1], 
                                               absolute->rot[i][2]);

  fprintf( fp, POS, comment_char, comment_char);
  fprintf( fp, "%20.7lf%20.7lf%20.7lf\n", absolute->x, absolute->y, absolute->z);
  
  fprintf( fp, LAMBDA, comment_char, comment_char);
  fprintf( fp, "%20.7lf\n", absolute->lambda);
  
/*------ Print out a message that we are done writing --------------------*/
  fprintf( stdout, "Absolute orientation elements written to %s.\n", filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_Absolute ]------------------------------
  This function prints the Absolute structure to stdout.

  IN : Structure containing read Absolute 
  RETURN: Nothing
------------------------------[ Print_Absolute ]------------------------------
*/
void Print_Absolute(const Absolute *absolute)
  {
  int i,j;
  double factor = 180 / 3.141592653589;
  
  printf("\n\tTranslation parameters :\n");
  printf("\t------------------------------------------\n");
  printf("\t(x, y, z) = (%9.4lf,%9.4lf,%9.4lf)\n", absolute->x, 
                                                   absolute->y, 
                                                   absolute->z);

  printf("\n\trotation matrix :\n");
  printf("\t------------------------------------------\n");
  for (i = 0; i < 3; i++)
    {
    for (j = 0; j < 3; j++)
      printf("\t%10.7lf", absolute->rot[i][j]);
    printf("\n");
    }

  if (absolute->status & INVALID_QUATERNION)
    printf("\n\tThe quaternion elements are *NOT* valid\n");
  else
    {
    printf("\n\tquaternion elements :\n");
    printf("\t------------------------------------------\n");
    printf("\ta : %10.7lf\n", absolute->a);
    printf("\tb : %10.7lf\n", absolute->b);
    printf("\tc : %10.7lf\n", absolute->c);
    }

  if (absolute->status & INVALID_ANGLES)
    printf("\n\tThe rotation angles are *NOT* valid\n");
  else
    {
    printf("\n\trotation angles :\n");
    printf("\t------------------------------------------\n");
    printf("\tomega : %16.7lf degrees\n", factor * absolute->w[0]);
    printf("\tphi   : %16.7lf degrees\n", factor * absolute->f[0]);
    printf("\tkappa : %16.7lf degrees\n", factor * absolute->k[0]);
    }
  
  printf("\n\tscale factor:\n");
  printf("\t------------------------------------------\n");
  printf("\t%10.7lf\n", absolute->lambda);
  
  printf("\n");
  }
