
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



#include <stdlib.h>
#include <string.h>
#include "Database.h"

struct ImgData { ImgPts   *imgpts;
                 char     *imgname;
                 char     *basename;
                 Exterior *extor;
                 Interior *intor; };

/*------------------------------------------------------------------------------
  Collect_Data receives the file filters of image point databases, interior
  orientation data and exterior orientation data and returns the image points
  together with the corresponding orientation data in a structure.
  If the specification of the filter for the interior orientation does not
  contain a wildcard, the filter is treated as the name of the interior
  orientation file that is common to all images.

  Author: M.G. Vosselman
  Date  : 25-10-95
  Modified by : F.A. van den Heuvel
  Date        : 27-09-96
  Modification: Allow exterior orientation files to be skipped 
                like the interior orientation files (filter is NULL).
------------------------------------------------------------------------------*/

int Collect_Data(char *imagepoint_filter, char *exterior_filter,
                 char *interior_filter, struct ImgData **imgdataptr, 
                 int *num_imgsptr)
/*
char           *imagepoint_filter,  Filter for image point databases
               *exterior_filter,    Filter for exterior orientation files
               *interior_filter;    Filter for interior orientation files
struct ImgData **imgdataptr;        Pointer to the image data structures
int            *num_imgsptr;        Number of images with orientation data
*/

{

/* Variables */

  struct ImgData *imgdata;
  char           directory[BUFSIZ], filtercopy[BUFSIZ], *filter, *filename;
  int            num_imgs, i, j, icon, lendir, wildcard;

/* Functions */

  char     *get_filename(const char *, const char *, int *);
  void     parse_filter(char **, char *);

/*------------------------------------------------------------------------------
  Input of files with image points
------------------------------------------------------------------------------*/

/* Separate the directory from the file filter of the image points */

  strcpy(filtercopy, imagepoint_filter);
  filter = &filtercopy[0];
  parse_filter(&filter, directory);
  lendir = strlen(directory);

/* First count the number of files with image points. Then, allocate
 * the image data structures for the images.
 */

  num_imgs = 0;
  icon     = 0; /* new context */
  while (get_filename(directory, filter, &icon) != NULL) num_imgs++;
  imgdata = (struct ImgData *) malloc(num_imgs * sizeof(struct ImgData));
  *imgdataptr = imgdata;

/* Read the image point files and determine the basenames of the images */

  i = icon = 0;
  while ((filename = get_filename(directory,filter,&icon)) != NULL) {
    strcat(directory, filename);
    imgdata[i].imgpts   = Get_ImgPts(directory);
    imgdata[i].imgname  = (char *) malloc(strlen(filename)+1);
    strcpy(imgdata[i].imgname, filename);
    imgdata[i].basename = NULL;
    imgdata[i].extor    = NULL;
    imgdata[i].intor    = NULL;
    for (j=0; j<strlen(filename)-1; j++) {
      if (filename[j] == '.') {
        imgdata[i].basename = (char *) malloc(j+2);
        strncpy(imgdata[i].basename, filename, j+1);
        imgdata[i].basename[j+1] = '\0';
        break;
      }
    }
    i++;
    directory[lendir] = '\0';
  }

/*------------------------------------------------------------------------------
  Input of files with exterior orientations 
------------------------------------------------------------------------------*/

/* Some applications of this function do not need the exterior orientation.
 * In these cases, the specified filter is NULL.
 */

  if (exterior_filter != NULL) {
  	
/* Separate the directory from the file filter of the exterior orientations */

	  strcpy(filtercopy, exterior_filter);
	  filter = &filtercopy[0];
	  parse_filter(&filter, directory);
	  lendir = strlen(directory);

/* Find the corresponding files with the exterior orientation */

	  icon = 0;
	  while ((filename = get_filename(directory,filter,&icon)) != NULL) {
	    for (i=0; i<num_imgs; i++) {
	      if (!strncmp(filename, imgdata[i].basename,
	                   strlen(imgdata[i].basename))) {
	        strcat(directory, filename);             /* Compose complete filename */
	        imgdata[i].extor = Get_Exterior(directory, &j);
	        directory[lendir] = '\0';    /* Remove filename from directory string */
	        break;
	      }
   	 }
	    if (i == num_imgs) { /* No corresponding image found */
	      fprintf(stderr, "Warning: No image point file for exterior orientation file %s\n",
	              filename);
	    }
	  }
  }

/*------------------------------------------------------------------------------
  Input of file(s) with interior orientation(s) 
------------------------------------------------------------------------------*/

/* Some applications of this function do not need the interior orientation.
 * In these cases, the specified filter is NULL.
 */

  if (interior_filter != NULL) {

/* Check if there is a wildcard in the filename */

    wildcard = 0;
    for (i=0; i<strlen(interior_filter); i++) {
      if (interior_filter[i] == '*' || interior_filter[i] == '?') {
        wildcard = 1;
        break;
      }
    }

/* If there is a wildcard in the specification of the interior orientation file
 * there should be a interior orientation for every image.
 */
    
    if (wildcard) {

/* Separate the directory from the file filter of the interior orientations
 */

      strcpy(filtercopy, interior_filter);
      filter = &filtercopy[0];
      parse_filter(&filter, directory);
      lendir = strlen(directory);

/* Find the corresponding files with the interior orientation */

      icon = 0;
      while ((filename = get_filename(directory,filter,&icon)) != NULL) {
        for (i=0; i<num_imgs; i++) {
          if (!strncmp(filename, imgdata[i].basename,
                       strlen(imgdata[i].basename))) {
            strcat(directory, filename);
            imgdata[i].intor = Get_Interior(directory);
            directory[lendir] = '\0';
            break;
          }
        }
        if (i == num_imgs) { /* No corresponding image found */
          fprintf(stderr, "Warning: No image point file for interior orientation file %s\n",
                  filename);
        }
      }
    }

/* If there is no wildcard in the specification of the interior orientation
 * file, the file is in common to all images.
 */

    else {
      imgdata[0].intor = Get_Interior(interior_filter);
      for (i=1; i<num_imgs; i++)
        imgdata[i].intor = imgdata[0].intor;
    }
  }
 
/*------------------------------------------------------------------------------
  Eliminate images without an interior or exterior orientation file 
------------------------------------------------------------------------------*/

  for (i=0; i<num_imgs; i++) {
    if ((imgdata[i].extor == NULL && exterior_filter != NULL) || 
        (imgdata[i].intor == NULL && interior_filter != NULL)) {
      if (imgdata[i].extor == NULL && exterior_filter != NULL)
        fprintf(stderr, "Warning: No exterior orientation file for image point file %s\n",
                imgdata[i].imgname);
      if (imgdata[i].intor == NULL && interior_filter != NULL)
        fprintf(stderr, "Warning: No interior orientation file for image point file %s\n",
                imgdata[i].imgname);
      for (j=i; j<num_imgs-1; j++)
        imgdata[j] = imgdata[j+1];
      i--; num_imgs--;
    }
  }
  *num_imgsptr = num_imgs;

  return(1);
}

/*------------------------------------------------------------------------------
  The initial file filter may contain a directory specification. The routine
  parse_filter, changes the pointer of the file filter to the first position
  after the directory specification and returns the directory in a separate
  string. 
------------------------------------------------------------------------------*/

void parse_filter(char **filter, char *directory)
{
  int i;

/* Default directory is the local directory */

  strcpy(directory, "./");

/* Find the last '/'  or '\' (=124)*/

  for (i=strlen(*filter)-1; i>=0; i--) {
    if ((*filter)[i] == '/' || (*filter)[i] == 124 ) {
      (*filter)[i] = '\0';
      if ((*filter)[0] == 39) // skip quote
        strcpy(directory, (*filter)+1);
      else
        strcpy(directory, *filter);
      strcat(directory, "/");
      *filter += i+1;
      // remove quote from filter if present
      if ((*filter)[strlen(*filter)-1] == 39)
        (*filter)[strlen(*filter)-1] = 0;
      break;
    }
  }
}
