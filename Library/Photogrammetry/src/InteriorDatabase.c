
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

  Function      : Input and output routines for interior orientation data
  Author        : R.Th. Ursem
  Creation date : 10 - Aug - 1994
  Modified by   : F.A. van den Heuvel
  Dated         : 27 - Sep - 1996
  Modification  : Image dimensions (in pixels) are added at the end of the file
  Modified by   : F.A. van den Heuvel
  Dated         : 24 - Nov - 1999
  Modification  : BINGO parameters are added at the end of the file
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c Interior orientation elements\n\
%c --------------------------------------------------------------\n"
#define CC_H      "%c Camera constant  (in mm)             "
#define PP_H      "%c Principal point  (in pix: row column)"
#define K_H       "%c Radial lens distortion               "
#define P_H       "%c Tangential lens distortion           "
#define SHEAR_H   "%c Shear factor                         "
#define ROT_H     "%c Rotation angle   (in degree)         "
#define SPACING_H "%c Pixel spacing    (in mm:  row column)"
#define DIM_H     "%c Image dimensions (in pix: row column)"
#define BINGOFF_H "%c BINGO Format factor                  "
#define BINGOCC_H "%c Correction to camera constant (in mm)"
#define BINGOPP_H "%c Principle point  (in mm:  x y)       "
#define BINGOAP_H "%c Additional parameter (id value)      "


/*
----------------------------[ Get_Interior ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Interior" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_Interior ]--------------------------
*/
Interior *Get_Interior(const char *filename)
  {
  FILE *fp;                   /* File Pointer                     */
  int i,itmp;                 /* i: line counter                  */
  double tmp;
  Interior *interior;         /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* File identification number       */
  double pi;

  pi = 4.0 * atan(1.0);
  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for Pixel structure -----------------------------*/
  interior = (Interior *)malloc( sizeof(Interior) );

/*------ Check the file ID -----------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != INTERNAL_ORIENTATION)
    {
    printf("Error in the file id\n");
    printf("This file does not contain internal orientation information\n");
    return( NULL );
    }

/*------ Initialize ------------------------------------------------------*/
  interior->bingo_cc = 0;
  interior->bingo_xh = 0;
  interior->bingo_yh = 0;
  for (i=0;i<MAXBINGO;i++) interior->bingo[i] = 0;

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!feof(fp) && !Is_Comment(line))
      {
      switch (i)
        {
        case 0 :  /*------ Camera Constant ------*/
          sscanf(line, "%lf", &(interior->cc));
          break;
        case 1 : /*------ Coordinates of the principal point ------*/
          sscanf(line, "%lf %lf", &(interior->rh), &(interior->ch));
          break;
        case 2 :  /*------ radial lens distortion ------*/
          sscanf(line, "%lf %lf %lf", &(interior->k1), &(interior->k2), &(interior->k3));
          break;
        case 3 :  /*------ tangential lens distortion ------*/
          sscanf(line, "%lf %lf", &(interior->p1), &(interior->p2));
          break;
        case 4 :  /*------ shear factor ------*/
          sscanf(line, "%lf", &(interior->shear));
          break;
        case 5 :  /*------ rotation angle ------*/
          sscanf(line, "%lf", &(interior->rot));
          interior->rot /=  180.0 / pi;
          break;
        case 6 :  /*------ pixelspacing ------*/
          sscanf(line, "%lf %lf", &(interior->spacing_r), &(interior->spacing_c));
          break;
        case 7 :  /*------ image dimensions ------*/
          sscanf(line, "%lf %lf", &(interior->dim_r), &(interior->dim_c));
          break;
        case 8 :  /*------ Bingo format factor ------*/
          sscanf(line, "%lf", &(interior->bingo[0]));
          break;
        case 9 :  /*------ Bingo camera constant ------*/
          sscanf(line, "%lf", &(interior->bingo_cc));
          break;
        case 10 :  /*------ Bingo principle point ------*/
          sscanf(line, "%lf %lf", &(interior->bingo_xh), &(interior->bingo_yh));
          break;
        }  /* end of case statement */
        if (i>10)
        {
          sscanf(line, "%i %lf", &itmp, &tmp);
          interior->bingo[itmp] = tmp;
        }
        i++;
      }
    }  
  /* cc correction from Bingo is added to cc !! */
  interior->cc += interior->bingo_cc;

  if (i<8) printf( "* Warning: %d lines missing \n", 8-i);
  if (i>8) printf( "\n* Warning: %d lines read for Bingo parameters \n", i-8);

/*------ Print out a message that we are done reading --------------------*/
  printf( "Interior orientation elements read from %s.\n", filename);
  
  fclose(fp);

  return (interior);
  }
  
/*
------------------------------[ Put_Interior ]--------------------------------
  This function writes the Interior structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read Interior
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Interior ]--------------------------------
*/
int Put_Interior(const Interior *interior, const char *filename)
  {
  int i;
  FILE *fp;                        /* File Pointer                        */

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write interior orientation elements database.\n", 
        filename);
    return (False);
    }
    
/*------ Print the headers and the information in the database file ------*/
  fprintf( fp, "%d # file identifier\n", INTERNAL_ORIENTATION);
  fprintf( fp, HEADER, comment_char, comment_char);
  
  fprintf( fp, "%7.4lf %23s", interior->cc, " ");                               
  fprintf( fp, CC_H, comment_char);    
  
  fprintf( fp, "\n%7.3lf %7.3lf %15s", interior->rh, interior->ch, " ");                
  fprintf( fp, PP_H, comment_char);    

  fprintf( fp, "\n%10.6lg %10.6lg %10.6lg  ", interior->k1, interior->k2, interior->k3); 
  fprintf( fp, K_H, comment_char);    

  fprintf( fp, "\n%10.6lg %10.6lg %12s", interior->p1, interior->p2, " ");                
  fprintf( fp, P_H, comment_char);    

  fprintf( fp, "\n%16.7lf %14s", interior->shear, " ");
  fprintf( fp, SHEAR_H, comment_char);    

  fprintf( fp, "\n%16.7lf %14s", interior->rot *  180.0 / 3.141592653589, " ");
  fprintf( fp, ROT_H, comment_char);    

  fprintf( fp, "\n%10.6lg %10.6lg %15s", interior->spacing_r, interior->spacing_c, " ");  
  fprintf( fp, SPACING_H, comment_char);   
   
  fprintf( fp, "\n%10.1lg %10.1lg %15s", interior->dim_r, interior->dim_c, " ");  
  fprintf( fp, DIM_H, comment_char);

  if (interior->bingo[0]>0)
    {
      fprintf( fp, "\n%10.6lg %15s", interior->bingo[0], " ");  
      fprintf( fp, BINGOFF_H, comment_char);
      fprintf( fp, "\n%10.6lg %15s", interior->bingo_cc, " ");  
      fprintf( fp, BINGOCC_H, comment_char);
      fprintf( fp, "\n%10.6lg %10.6lg %15s", interior->bingo_xh, interior->bingo_yh, " ");  
      fprintf( fp, BINGOPP_H, comment_char);
      for (i=1;i<MAXBINGO;i++)
	{
	  if (interior->bingo[i]!=0)
	    {
	      fprintf( fp, "\n%2d %10.6lg %15s", i, interior->bingo[i], " ");  
	      fprintf( fp, BINGOAP_H, comment_char);
	    }
	}
    }

   
  fprintf( fp, "\n");                            
  
/*------ Print out a message that we are done writing --------------------*/
  printf("Interior orientation elements written to %s.\n", filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_Interior ]------------------------------
  This function prints the Interior structure to stdout.

  IN : Structure containing read Interior 
  RETURN: Nothing
------------------------------[ Print_Interior ]------------------------------
*/
void Print_Interior(const Interior *interior)
  {
  char tab[80];
  int i;
  
  printf("\n");
  printf( HEADER, ' ', ' '); 
  
  printf( CC_H, ' ');      
  printf(": %lf\n", interior->cc);
  
  printf( PP_H, ' ');      
  printf(": (%1.3lf,%1.3lf)\n", interior->rh, interior->ch);
  
  strcpy( tab, K_H );
  i = 0;
  while (tab[i] != 0) tab[i++] = ' ';
  tab[strlen(tab) - 1] = 0;
  
  printf( K_H, ' ');       
  printf(  ": %10.6lg\t(k1)\n", interior->k1);
  printf("%s: %10.6lg\t(k2)\n", tab, interior->k2);
  printf("%s: %10.6lg\t(k3)\n", tab, interior->k3);
  
  strcpy( tab, P_H );
  i = 0;
  while (tab[i] != 0) tab[i++] = ' ';
  tab[strlen(tab) - 1] = 0;

  printf( P_H, ' ');       
  printf(  ": %10.6lg\t(p1)\n", interior->p1);
  printf("%s: %10.6lg\t(p2)\n", tab, interior->p2);
  
  printf( SHEAR_H, ' ');   
  printf(": %16.7lf\n", interior->shear);
  
  printf( ROT_H, ' ');   
  printf(": %16.7lf\n", interior->rot *  180.0 / 3.141592653589);
  
  printf( SPACING_H, ' '); 
  printf(": %10.6lf (row) %10.6lf (column)\n", interior->spacing_r, interior->spacing_c);

  printf( DIM_H, ' '); 
  printf(": %10.1lf (row) %10.1lf (column)\n", interior->dim_r, interior->dim_c);


  if (interior->bingo[0]>0)
    {
      printf( BINGOFF_H, ' ');
      printf( ": %10.6lg %15s\n", interior->bingo[0], " ");  
      printf( BINGOCC_H, ' ');
      printf( ": %10.6lg %15s\n", interior->bingo_cc, " ");  
      printf( BINGOPP_H, ' ');
      printf( ": %10.6lg %10.6lg %15s\n", interior->bingo_xh, interior->bingo_yh, " ");  
      for (i=1;i<MAXBINGO;i++)
	{
	  if (interior->bingo[i]!=0)
	    {
	      printf( BINGOAP_H, ' ');
	      printf( ": %2d %10.6lg %15s\n", i, interior->bingo[i], " ");  
	    }
	}
    }

  printf("\n");
  }
