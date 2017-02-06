
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
  * $Log$
  * Revision 1.3  2010/07/09 08:40:05  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:41:22  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:15  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:00  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:29  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:57  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.1.1.1  2003/04/09 11:07:37  rabbani
  * basic photogrammetric classes and functions (orientation, in/output, matrices, ...
  *
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:53  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for anaglyph
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lanaglyph
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "viff.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lanaglyph - *
* 
*       Purpose: Library Routine for anaglyph
*         Input: .IP
img1 - pointer to the first input image structure
.br
img2 - pointer to the second input image structure
.br
img3 - pointer to the blend image structure (24 Bit !)
.br
x    - optional blend ratio
*        Output: .IP
img3 - blended image
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: R.Th. Ursem
*          Date: Jul 27, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lanaglyph(xvimage *img1, xvimage *img2, xvimage *img3, int left_band,
              int right_band, int dr, int dc)
/* -library_def_end */

/* -library_code */
{
  void Copy_Image_In_Band();
  unsigned char *result;
  int size, w, h;
  int l_r, l_c, r_r, r_c;
  int red_band=1, green_band=2, blue_band=3;

  void Copy_Image_In_Band(unsigned char **, xvimage *, int, int, int, int);
  
  w = img3->row_size;
  h = img3->col_size;
  size = w * h;
  
  /*------ Process every pixel in the left and right images ------*/
  result = (unsigned char *)img3->imagedata;

  if (left_band == right_band)
    {
    fprintf( stderr, "Left and right image can not be in the same band!\n");
    return( 0 );
    }
  
  l_r = l_c = r_r = r_c = 0;
  
  if (dr < 0) 
     l_r = -dr;
  else 
     r_r =  dr;
  
  if (dc < 0) 
     l_c = -dc;
  else 
     r_c =  dc;

/*                           _\\|//_ 
                             ( .~. )
--------------------------o00--(_)--00o------------------------------
*/
  printf("  red\t-> ");
  if (left_band  == red_band) 
  {
     printf("left\n");
     Copy_Image_In_Band(&result, img1, l_r, l_c, w, h);
  }
  
  else if (right_band == red_band) 
  {
     printf("right\n");
     Copy_Image_In_Band(&result, img2, r_r, r_c, w, h);
  }
  
  else 
  {
     printf("none\n");
     Copy_Image_In_Band(&result, NULL, 0, 0, w, h);
  }
  
/*                           _\\|//_ 
                             ( .~. )
--------------------------o00--(_)--00o------------------------------
*/
  printf("  green\t-> ");
  if (left_band  == green_band) 
  {
     printf("left\n");
     Copy_Image_In_Band(&result, img1, l_r, l_c, w, h);
  }
  
  else if (right_band == green_band) 
  {
     printf("right\n");
     Copy_Image_In_Band(&result, img2, r_r, r_c, w, h);
  }
  
  else 
  {
     printf("none\n");
     Copy_Image_In_Band(&result, NULL, 0, 0, w, h);
  }

/*                           _\\|//_ 
                             ( .~. )
--------------------------o00--(_)--00o------------------------------
*/
  printf("  blue\t-> ");
  if (left_band  == blue_band) 
  {
     printf("left\n");
     Copy_Image_In_Band(&result, img1, l_r, l_c, w, h);
  }
  
  else if (right_band == blue_band) 
  {
     printf("right\n");
     Copy_Image_In_Band(&result, img2, r_r, r_c, w, h);
  }
  
  else 
  {
     printf("none\n");
     Copy_Image_In_Band(&result, NULL, 0, 0, w, h);
  }
  
  return( 1 );
}
  
void Copy_Image_In_Band(unsigned char **result, xvimage *img, int dr, int dc,
                        int w, int h)
{
   int row, col;
   
   if (img != NULL)
   {
      unsigned char *image = (unsigned char *)img->imagedata;
      
      for (row = 0; row < h; row++)
         for (col = 0; col < w; col++, (*result)++) 
         {
            if (col - dc >= 0 && row - dr >= 0 && 
                col - dc < img->row_size && row - dr < img->col_size) 
            {
               *(*result) = *image; 
               image++; 
            } 
            else
               *(*result) = (unsigned char) 0;
         }
   }
   
   else 
      for (row = 0; row < w*h; row++, (*result)++) 
         *(*result) = (unsigned char) 0;
   
}
/* -library_code_end */
