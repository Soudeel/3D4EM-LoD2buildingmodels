
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

  Function      : Input and output routines for exterior orientation data
  Author        : R.Th. Ursem
  Creation date : 10 - Aug - 1994
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "digphot_arch.h"
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c Exterior orientation elements\n\n"
#define ROT "%c The rotation matrix from image space to object space\n\
%c ----------------------------------------------------------\n"
#define POS "\n%c The position of the projection centre in object space\n\
%c ----------------------------------------------------------\n"
#define EPSILON  0.000000000001
#define EPSILON2 0.00001

/*
----------------------------------------------------------------------
 This routine calculates the elements of the rotation matrix from the
 given rotation angles.
 
             matrix = R(w) . R(f) . R(k)
 
 Rotation from camera to object, thus order w, f, k when rotating from
 object to camera.

 IN : exterior orientation structure, containing rotation matrix
 RETURN : True if succes, otherwise False
----------------------------------------------------------------------
*/
int Calculate_Rotation_Matrix(double w, double f, double k, 
                              double matrix[3][3])
{
  matrix[0][0] =  cos(k) * cos(f);
  matrix[0][1] = -sin(k) * cos(f);
  matrix[0][2] =  sin(f);
  
  matrix[1][0] =  sin(w) * cos(k) * sin(f) + cos(w) * sin(k);
  matrix[1][1] =  cos(w) * cos(k) - sin(w) * sin(k) * sin(f);
  matrix[1][2] = -sin(w) * cos(f);
  
  matrix[2][0] =  sin(w) * sin(k) - cos(w) * cos(k) * sin(f);
  matrix[2][1] =  cos(w) * sin(k) * sin(f) + sin(w) * cos(k);
  matrix[2][2] =  cos(w) * cos(f);

  return(1);
  }

/*
----------------------------------------------------------------------
 This routine calculates sets of rotation angles from the given 
 rotation matrix;
 
 IN : exterior orientation structure, containing rotation matrix
 RETURN : True if succes, otherwise False
----------------------------------------------------------------------
*/
int Angles_From_Rot(Exterior *exterior)
{
  double omega[2];        /* Two possible omega angles                   */
  double phi[2];          /* Two possible phi angles                     */
  double kappa[2];        /* Two possible kappa angles                   */ 
  double ref_rot[3][3];   /* Rotation matrix of combination of w,f,k     */
  double solution[2][3];  /* Two sets of three angles that form the rot. */
  double difference;      /* Difference to the original rotation matrix  */
  int w,f,k;              /* Omega, phi and kappa possibility            */
  int num_f;              /* Number of phi possibilities                 */
  int i,j;                /* Loop counters                               */
  int solution_counter;   /* Solution counter                            */
  
  double pi = fabs(acos(-1.0)); /* 3.1415926535897931 etc. */

/*------ Initialize the solution array -----------------------------------*/
  solution_counter = 0;
  for (i = 0; i < 2; i++) {
    for (j = 0; j < 3; j++) solution[i][j] = 0;
    exterior->w[i] = exterior->f[i] = exterior->k[i] = 0;
  }
    
/*------ There are two possibilities for the value of phi ----------------*/ 
  phi[0]  = asin(exterior->rot[0][2]);
  phi[1]  = pi - phi[0];
  
/*------ Special case if phi equals 90 degrees ---------------------------*/
  if ( fabs(phi[0] - phi[1]) < EPSILON ) {
    num_f    = 1;
    kappa[0] = 0.0;
    omega[0] = atan2(exterior->rot[1][0], exterior->rot[1][1]);
    kappa[1] = pi/2.0;
    omega[1] = atan2(exterior->rot[2][0], exterior->rot[1][0]);
  }

/*------ Normal case, two solutions for both omega and kappa -------------*/

  else {
    num_f    = 2;
    omega[0] = atan2( -exterior->rot[1][2], exterior->rot[2][2]);
    omega[1] = omega[0] + pi;
    kappa[0] = atan2( -exterior->rot[0][1], exterior->rot[0][0]);
    kappa[1] = kappa[0] + pi;
  }

/*------ Now figure out which of the 8 combinations are the right ones ---*/
  for (w = 0; w < 2; w++) {
    for (f = 0; f < num_f; f++) {
      for (k = 0; k < 2; k++) {
/*----- Calculate the rotation matrix with this combination --------------*/
        Calculate_Rotation_Matrix(omega[w], phi[f], kappa[k], ref_rot);

/*----- Calculate the difference with the original rotation matrix -------*/
        difference = 0.0;
        for (i = 0; i < 3; i++)
          for (j = 0; j < 3; j++)
            difference += fabs( ref_rot[i][j] - exterior->rot[i][j] );
        
/*----- Check if difference is small enough and store combination --------*/
        if (difference < EPSILON2) {
          if (solution_counter == 2) {
            printf("More than 2 angle solutions found. Keeping the last one.\n");
            solution_counter--;
          }
          solution[solution_counter][0] = omega[w]; 
          solution[solution_counter][1] = phi[f]; 
          solution[solution_counter][2] = kappa[k]; 
          solution_counter++;
        }
      }
    }
  }

/*------ Warning if not two solutions have been found ------------------*/
  switch (solution_counter) {
    case 0 : fprintf(stderr, "ERROR: no angles could be calculated\n");
             return( False );
    case 1 : fprintf(stderr,
                     "WARNING: only one set of angles could be calculated\n");
  }

/*------ Copy the found solutions into the exterior structure ----------*/ 
  for (i = 0; i < solution_counter; i++) {
    exterior->w[i] = solution[i][0];
    exterior->f[i] = solution[i][1];
    exterior->k[i] = solution[i][2];
  }
    
  return( True );
}

/*
----------------------------------------------------------------------
 This routine calculates the quaternion elements from the given 
 rotation matrix;
 
 IN : exterior orientation structure, containing quaternion elemnts
 RETURN : True if succes, otherwise False
----------------------------------------------------------------------
*/
int Quat_From_Rot(Exterior *exterior)
  {
  int i,j;                /* loop counters                              */
  double wr[3], wi[3];    /* eigenvaluess, real and imaginary part      */
  double z[3][3];         /* eigenvectors, belonging to the eigenvalues */
  double iv1[3], fv1[3];  /* temporary matrices for rg() routine        */
  int error;              /* return value from rg() routine             */
  int dim = 3;            /* dimension of the rotation matrix           */
  int matz = 1;           /* if non-zero, eigenvectors are calculated   */
  double copy_rot[3][3];  /* copy of the rotation matrix for rg() func. */
  double h;               /* temporary value for the calculation of mu  */
  double mu;              /* multiplication factor                      */
  double a, b, c;         /* copy of quaternion elements, shorter names */
  double det;             /* 1 over the determinant of the rot. matrix  */
  double check;           /* element of the rot. matrix, calc. from quat*/
  double e, f, g;         /* multiplication factors, shorter names      */

#ifdef hpux
  void rg(int *, int *, double *, double *, double *, int *, double *, double *,
          double *, int *);
#else
  void rg_(int *, int *, double *, double *, double *, int *, double *,
           double *, double *, int *);
#endif
  
/*------ First copy the rotation matrix cause rg() modifies it ---------*/
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      copy_rot[i][j] = exterior->rot[i][j];
      
/*------ Calculate the eigenvalues and eigenvectors of the rot. matrix -*/
#ifdef hpux
  rg(&dim, &dim, (double *) copy_rot, wr,wi, &matz, (double *) z, iv1, fv1,
     &error);
#else
  rg_(&dim, &dim, (double *) copy_rot, wr,wi, &matz, (double *) z, iv1, fv1,
      &error);
#endif
  
/*------ Uncomment to print the eigenvalues and eigenvectors -----------
  for (i = 0; i < 3; i++)                                             
    printf("%16.13lf + i %16.13lf -> (%16.13lf, %16.13lf, %16.13lf)\n",       
           wr[i], wi[i], z[i][0], z[i][1], z[i][2]);                  
  ----------------------------------------------------------------------*/
  
/*------ Search the eigenvalues and use only the real ones -------------*/
  for (i = 0; i < 3; i++) 
    if (wi[i] == 0)
      {
      /*------ Copy the found eigenvector in the structure -------------*/
      a = z[i][0];
      b = z[i][1];
      c = z[i][2];

      /*------ Setup the coefficients of the multiplication ------------*/
      h = exterior->rot[i][i];
      if (i == 0)      { e = h - 1.0; f = g = h + 1.0; }
      else if (i == 1) { f = h - 1.0; e = g = h + 1.0; }
      else             { g = h - 1.0; e = f = h + 1.0; }
      
      /*------ Check if this eigenvector can be used -------------------*/
      if ( fabs( e * a*a + f * b*b + g * c*c ) > EPSILON )
        break; 
      }

/*------ Check for divide by zero --------------------------------------*/
  if (h != 1.0 && fabs( e * a*a + f * b*b + g * c*c ) < EPSILON)
    {
    fprintf( stderr, "ERROR: calculation of quaternion elements not possible\n");
    exterior->a = exterior->b = exterior->c = 0;
    return( False );
    }

/*------ Calculate the multiplication factor ---------------------------*/
  if (h != 1.0) mu = (1. - h) / ( e * a*a + f * b*b + g * c*c );
  else mu = 0.0;
  mu = sqrt(mu);
  
/*------ Now check if the found mu factor is the correct one -----------*/
  det = 1. + mu*mu * (a*a + b*b + c*c);
  check = (2. * a * b * mu*mu - 2. * c * mu) / det;
  if ( fabs(check - exterior->rot[0][1]) > EPSILON) mu *= -1.0; 

/*------ Update the exterior orientation structure ---------------------*/
  exterior->a = a * mu;
  exterior->b = b * mu;
  exterior->c = c * mu;
  
  return( True );
  }

/*
----------------------------------------------------------------------
 This routine calculates the elements of the rotation matrix from the
 given quaternion elements.
 
 IN : exterior orientation structure, containing quaternion elemnts
 RETURN : True if succes, otherwise False
----------------------------------------------------------------------
*/
int Rot_From_Quat(Exterior *exterior)
  {
  int i, j;
  
  double a = exterior->a;
  double b = exterior->b;
  double c = exterior->c;
  
  double det = 1.0 + a*a + b*b + c*c;
  
  exterior->rot[0][0] = 1 + a*a - b*b - c*c;
  exterior->rot[0][1] = 2 * a * b - 2 * c;
  exterior->rot[0][2] = 2 * a * c + 2 * b;
  
  exterior->rot[1][0] = 2 * a * b + 2 * c;
  exterior->rot[1][1] = 1 - a*a + b*b - c*c;
  exterior->rot[1][2] = 2 * b * c - 2 * a;
  
  exterior->rot[2][0] = 2 * a * c - 2 * b;
  exterior->rot[2][1] = 2 * b * c + 2 * a;
  exterior->rot[2][2] = 1 - a*a - b*b + c*c;
  
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      exterior->rot[i][j] /= det;

  return( True );
  }

/*
----------------------------------------------------------------------
 This routine calculates the elements of the rotation matrix from the
 given rotation angles.
 
             exterior->rot = R(w) . R(f) . R(k)
 
 IN : exterior orientation structure, containing rotation angles
 RETURN : True if succes, otherwise False
----------------------------------------------------------------------
*/
int Rot_From_Angles(Exterior *exterior)
  {

  Calculate_Rotation_Matrix(exterior->w[0], 
                            exterior->f[0], 
                            exterior->k[0],
                            exterior->rot);
  return( True );
  }

/*
----------------------------[ Get_Exterior ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Exterior" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_Exterior ]--------------------------
*/
Exterior *Get_Exterior(const char *filename, int *error)
  {
  FILE *fp;                   /* File Pointer                     */
  int i, j;                   /* counters                         */
  Exterior *exterior;         /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* File identifier                  */

  *error = ALL_VALID;
  
  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for Pixel structure -----------------------------*/
  exterior = (Exterior *)malloc( sizeof(Exterior) );

/*------ Read the file ---------------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != EXTERNAL_ORIENTATION)
    {
    printf("Error in file identifier\n");
    printf("This file ( %s ) does not contain external orientation data\n", filename);
    return( NULL );
    }

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp)) {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line)) {
      if (i < 3)
        sscanf( line, "%lf %lf %lf", &(exterior->rot[i][0]), 
                                     &(exterior->rot[i][1]), 
                                     &(exterior->rot[i][2]) );
      else if (i == 3)
        sscanf( line, "%lf %lf %lf", &(exterior->x), &(exterior->y), &(exterior->z));
      i++;
    }
  }
    
  fclose(fp);

  if (!Angles_From_Rot(exterior)) *error |= INVALID_ANGLES;
  if (!Quat_From_Rot(exterior))   *error |= INVALID_QUATERNION;
  
/*------ Update the status byte in the structure ------*/
  exterior->status = *error;
  
/*------ Print out a message that we are done reading --------------------*/
  printf( "Exterior orientation elements read from %s.\n", filename);
  
  return (exterior);
  }
  
/*
------------------------------[ Put_Exterior ]--------------------------------
  This function writes the Exterior structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read Exterior
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Exterior ]--------------------------------
*/
int Put_Exterior(const Exterior *exterior, const char *filename)
  {
  FILE *fp;                        /* File Pointer                        */
  int i;

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write Exterior orientation elements database.\n", 
        filename);
    return (False);
    }
    
/*------ Print the headers and the information in the database file ------*/
  fprintf( fp, "%d # file identifier\n", EXTERNAL_ORIENTATION);
  fprintf( fp, HEADER, comment_char);
  
  fprintf( fp, ROT, comment_char, comment_char);
  for (i = 0; i < 3; i++)
    fprintf( fp, "%20.16lf%20.16lf%20.16lf\n", exterior->rot[i][0], 
                                               exterior->rot[i][1], 
                                               exterior->rot[i][2]);

  fprintf( fp, POS, comment_char, comment_char);
  fprintf( fp, "%20.7lf%20.7lf%20.7lf\n", exterior->x, exterior->y, exterior->z);
  
/*------ Print out a message that we are done writing --------------------*/
  fprintf( stdout, "Exterior orientation elements written to %s.\n", filename);

  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_Exterior ]------------------------------
  This function prints the Exterior structure to stdout.

  IN : Structure containing read Exterior 
  RETURN: Nothing
------------------------------[ Print_Exterior ]------------------------------
*/
void Print_Exterior(const Exterior *exterior)
  {
  int i,j;
  double factor = 180 / 3.141592653589;
  
  printf("\n\tPosition of the camera :\n");
  printf("\t------------------------------------------\n");
  printf("\t(x, y, z) = (%9.4lf,%9.4lf,%9.4lf)\n", exterior->x, 
                                                   exterior->y, 
                                                   exterior->z);

  printf("\n\trotation matrix :\n");
  printf("\t------------------------------------------\n");
  for (i = 0; i < 3; i++)
    {
    for (j = 0; j < 3; j++)
      printf("\t%10.7lf", exterior->rot[i][j]);
    printf("\n");
    }

  if (exterior->status & INVALID_QUATERNION)
    printf("\n\tThe quaternion elements are *NOT* valid\n");
  else
    {
    printf("\n\tquaternion elements :\n");
    printf("\t------------------------------------------\n");
    printf("\ta : %10.7lf\n", exterior->a);
    printf("\tb : %10.7lf\n", exterior->b);
    printf("\tc : %10.7lf\n", exterior->c);
    }

  if (exterior->status & INVALID_ANGLES)
    printf("\n\tThe rotation angles are *NOT* valid\n");
  else
    {
    printf("\n\trotation angles :\n");
    printf("\t------------------------------------------\n");
    printf("\tomega : %16.7lf degrees\n", factor * exterior->w[0]);
    printf("\tphi   : %16.7lf degrees\n", factor * exterior->f[0]);
    printf("\tkappa : %16.7lf degrees\n", factor * exterior->k[0]);
    }
  
  printf("\n");
  }
