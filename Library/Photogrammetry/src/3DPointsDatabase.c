
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

  Function      : Input and output routines for the database of 
                  object- and passpoints
  Author        : R.Th. Ursem
  Creation date : 9 - Aug - 1994

  Update #1     : Added search function Search_ObjPts
  Author        : M.G. Vosselman
  Date          : 20 - Oct - 1995
  
  Update #2     : Added search function Search_CtrlPts
  Author        : M.G. Vosselman
  Date          : 30 - Oct - 1995
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdlib.h>
#include <stdio.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define NUMBER_H    "%c Number of points in this file : %d\n"

#define O_CONTENTS_H    "%c This file contains object points\n"
#define O_EXPL_H   "%c                       coordinates                                   variances                                    covariances\n\
%c point        x                y              z              x              y              z               xy             xz             yz\n"
#define O_LINE_H   "%c-----------------------------------------------------------------------------------------------------------------------------------------------\n"
#define O_FORMAT   " %4d%17.7lf%17.7lf%15.7lf%16.7le%15.7le%15.7le%16.7le%15.7le%15.7le\n"
#define O_FORMAT2   " %4d%16.7lf%16.7lf%15.7lf\n"

#define OP_EXPL_H   "%c                 coordinates                    variances                  covariances\n\
%cpoint    x           y           z        x        y        z        xy        xz        yz\n"
#define OP_LINE_H   "%c-----------------------------------------------------------------------------------------------\n"
#define OP_FORMAT  "%4d%11.5lf,%11.5lf,%11.5lf%9.2le%9.2le%9.2le%10.2le%10.2le%10.2le\n"

#define M_CONTENTS_H    "%c This file contains model points\n"
#define M_EXPL_H   "%c                       coordinates                                   variances                                    covariances\n\
%c point        x                y              z              x              y              z               xy             xz             yz\n"
#define M_LINE_H   "%c-----------------------------------------------------------------------------------------------------------------------------------------------\n"
#define M_FORMAT   " %4d%17.7lf%17.7lf%15.7lf%16.7le%15.7le%15.7le%16.7le%15.7le%15.7le\n"

#define MP_EXPL_H   "%c                 coordinates                    variances                  covariances\n\
%cpoint    x           y           z        x        y        z        xy        xz        yz\n"
#define MP_LINE_H   "%c-----------------------------------------------------------------------------------------------\n"
#define MP_FORMAT  "%4d%11.5lf,%11.5lf,%11.5lf%9.2le%9.2le%9.2le%10.2le%10.2le%10.2le\n"

#define C_CONTENTS_H   "%c This file contains control points\n"
#define C_STATUS_EXPL_H "%c The status field has the following meaning :\n\
%c\t0 -> Full control point (x, y and z known)\n\
%c\t1 -> Planimetric control point (x and y known, z is not to be used)\n\
%c\t2 -> Height control point (only z is known and can be used)\n\
%c\t3 -> No control point, all coordinates are unknown\n"
#define C_EXPL_H   "%c Pointnumber         x               y              z        status\n"
#define C_LINE_H   "%c------------- -------------- --------------- -------------- --------\n"
#define C_FORMAT  " %9d%16.3lf%16.3lf%15.3lf%9d\n"

int CheckID(FILE *fp, int wanted_id)
{
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* file identifier                  */

/*------ Check the file identifier ---------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);

  return (file_id == wanted_id);
}

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
                         O B J E C T P O I N T S
=========================================================================

----------------------------[ Get_ObjPts ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read points into a structure of type "ObjPts" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_ObjPts ]----------------------------
*/
ObjPts *Get_ObjPts(const char *filename)
  {
  FILE *fp;                   /* File Pointer                     */
  int i;                      /* General counter                  */
  ObjPts *obj_pts;            /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* file identifier                  */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Check the file identifier ---------------------------------------*/
  if (!CheckID(fp, OBJECT_POINTS))
    {
    printf("file identification error, this file ( %s ) does not contain \
object points\n", filename);
    return( NULL );
    }

/*------ Allocate memory for Pixel structure -----------------------------*/
  obj_pts = (ObjPts *) malloc( sizeof(ObjPts) );

/*------ Initiate the ObjPts structure -----------------------------------*/
  obj_pts->num_pts = 0;
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the Pixel that is about to be read -----*/
      if (!obj_pts->num_pts) 
        obj_pts->pts = (ObjPt *) malloc( sizeof(ObjPt ) );
      else
        obj_pts->pts = (ObjPt *) realloc( obj_pts->pts, 
                          (obj_pts->num_pts + 1) * sizeof(ObjPt) );

/*------ Extract the measured object point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &(obj_pts->pts[obj_pts->num_pts].num),
             &(obj_pts->pts[obj_pts->num_pts].x),
             &(obj_pts->pts[obj_pts->num_pts].y),
             &(obj_pts->pts[obj_pts->num_pts].z),
	     
             &(obj_pts->pts[obj_pts->num_pts].v_x),
             &(obj_pts->pts[obj_pts->num_pts].v_y),
             &(obj_pts->pts[obj_pts->num_pts].v_z),
	     
             &(obj_pts->pts[obj_pts->num_pts].cv_xy),
             &(obj_pts->pts[obj_pts->num_pts].cv_xz),
             &(obj_pts->pts[obj_pts->num_pts].cv_yz)
	     );
             
      obj_pts->num_pts++; 
      }
    }  
    
  if (obj_pts->num_pts > 0) obj_pts->num_pts--; 
  else obj_pts->pts = NULL;

  fclose(fp);

  return (obj_pts);
  }
  
/*
---------------------------------------------------------------------------
  This function is used to sort the array of points in ascending order
---------------------------------------------------------------------------
*/
int Compare_ObjPts(const void *a, const void *b)
{
  const ObjPt *ap, *bp;
  ap = (const ObjPt *) a;
  bp = (const ObjPt *) b;
  return(ap->num - bp->num);
  }

/*
------------------------------[ Put_ObjPts ]--------------------------------
  This function writes the ObjPts structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read ObjPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ObjPts ]--------------------------------
*/
int Put_ObjPts(ObjPts *obj_pts, const char *filename)
{
  FILE *fp;                 /* File Pointer                        */
  int i;                    /* General counter                     */
  ObjPt *p;                 /* Pointer to the coordinates          */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write object point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = obj_pts->pts;
  qsort(p, obj_pts->num_pts, sizeof(ObjPt), Compare_ObjPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file identifier\n", OBJECT_POINTS);
  fprintf( fp, O_CONTENTS_H, comment_char);
  fprintf( fp, NUMBER_H,     comment_char, obj_pts->num_pts);
  fprintf( fp, O_EXPL_H,     comment_char, comment_char);
  fprintf( fp, O_LINE_H,     comment_char);
  
/*------ Print the object points into the file ----------------------------*/
  for (i = 0; i < obj_pts->num_pts; i++)
    fprintf( fp, O_FORMAT, p[i].num, p[i].x, p[i].y, p[i].z,
	    p[i].v_x, p[i].v_y, p[i].v_z, 
	    p[i].cv_xy, p[i].cv_xz, p[i].cv_yz);
    
  fclose(fp);

  return (True);
}

/*
------------------------------[ Put_ObjPts2 ]--------------------------------
  This function writes the ObjPts structure without coviriance into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read ObjPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ObjPts ]--------------------------------
*/

int Put_ObjPts2(ObjPts *obj_pts, const char *filename)
{
  FILE *fp;                 /* File Pointer                        */
  int i;                    /* General counter                     */
  ObjPt *p;                 /* Pointer to the coordinates          */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write object point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = obj_pts->pts;
  qsort(p, obj_pts->num_pts, sizeof(ObjPt), Compare_ObjPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file identifier\n", OBJECT_POINTS);
  fprintf( fp, O_CONTENTS_H, comment_char);
  fprintf( fp, NUMBER_H,     comment_char, obj_pts->num_pts);
  fprintf( fp, O_EXPL_H,     comment_char, comment_char);
  fprintf( fp, O_LINE_H,     comment_char);
  
/*------ Print the object points into the file ----------------------------*/
  for (i = 0; i < obj_pts->num_pts; i++)
    fprintf( fp, O_FORMAT2, p[i].num, p[i].x, p[i].y, p[i].z);
    
  fclose(fp);

  return (True);
}




/*
------------------------------[ Print_ObjPts ]------------------------------
  This function prints the ObjPts structure to stdout.

  IN : Structure containing read ObjPts 
  RETURN: Nothing
------------------------------[ Print_ObjPts ]------------------------------
*/
void Print_ObjPts(const ObjPts *obj_pts)
{
  int i;
  
  printf( " Number of objects points %d\n", obj_pts->num_pts);
  printf( OP_EXPL_H,  ' ', ' ');
  printf( OP_LINE_H, ' ');
  
  for (i = 0; i < obj_pts->num_pts; i++)
    printf(OP_FORMAT, (obj_pts->pts[i]).num, 
                   (obj_pts->pts[i]).x, 
                   (obj_pts->pts[i]).y, 
                   (obj_pts->pts[i]).z,

                   (obj_pts->pts[i]).v_x, 
                   (obj_pts->pts[i]).v_y, 
                   (obj_pts->pts[i]).v_z,

                   (obj_pts->pts[i]).cv_xy, 
                   (obj_pts->pts[i]).cv_xz, 
                   (obj_pts->pts[i]).cv_yz);
}



/*
------------------------------[ Free_ObjPts ]--------------------------------
  This function deallocates the memory used by ObjPts.

  IN : Structure containing read ObjPts 
  RETURN: Nothing
------------------------------[ Free_ObjPts ]--------------------------------
*/

void Free_ObjPts(ObjPts *objpts)
{
   free(objpts->pts);
   free(objpts);
   objpts = NULL;
}	


/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
                         M O D E L P O I N T S
=========================================================================

----------------------------[ Get_ObjPts ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read points into a structure of type "ObjPts" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_ObjPts ]----------------------------
*/
ModelPts *Get_ModelPts(const char *filename)
{
  FILE *fp;                   /* File Pointer                     */
  int i;                      /* General counter                  */
  ModelPts *model_pts;        /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* file identifier                  */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Check the file identifier ---------------------------------------*/
  if (!CheckID(fp, MODEL_POINTS))
    {
    printf("file identification error,\nthis file ( %s ) does not contain \
model points\n", filename);
    return( NULL );
    }

/*------ Allocate memory for Pixel structure -----------------------------*/
  model_pts = (ModelPts *)malloc( sizeof(ModelPts) );

/*------ Initiate the ObjPts structure -----------------------------------*/
  model_pts->num_pts = 0;
  
/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the point that is about to be read -----*/
      if (!model_pts->num_pts) 
        model_pts->pts = (ModelPt *) malloc( sizeof(ModelPt) );
      else
        model_pts->pts = (ModelPt *) realloc( model_pts->pts, 
                          (model_pts->num_pts + 1) * sizeof(ModelPt) );

/*------ Extract the measured object point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &(model_pts->pts[model_pts->num_pts].num),
             &(model_pts->pts[model_pts->num_pts].x),
             &(model_pts->pts[model_pts->num_pts].y),
             &(model_pts->pts[model_pts->num_pts].z),
	     
             &(model_pts->pts[model_pts->num_pts].v_x),
             &(model_pts->pts[model_pts->num_pts].v_y),
             &(model_pts->pts[model_pts->num_pts].v_z),
	     
             &(model_pts->pts[model_pts->num_pts].cv_xy),
             &(model_pts->pts[model_pts->num_pts].cv_xz),
             &(model_pts->pts[model_pts->num_pts].cv_yz)
	     );
             
      model_pts->num_pts++; 
      }
    }  
    
  if (model_pts->num_pts > 0) model_pts->num_pts--; 
  else model_pts->pts = NULL;

  fclose(fp);

  return (model_pts);
}
  
/*
---------------------------------------------------------------------------
  This function is used to sort the array of points in ascending order
---------------------------------------------------------------------------
*/
int Compare_ModelPts(const void *a, const void *b)
{
  const ModelPt *ap, *bp;
  
  ap = (const ModelPt *) a;
  bp = (const ModelPt *) b;
  return(ap->num - bp->num);
}

/*
------------------------------[ Put_ModelPts ]--------------------------------
  This function writes the ModelPts structure into a file. 
  
  IN : Structure containing read ModelPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_ModelPts ]--------------------------------
*/
int Put_ModelPts(ModelPts *model_pts, const char *filename)
{
  FILE *fp;                 /* File Pointer                        */
  int i;                    /* General counter                     */
  ModelPt *p;               /* Pointer to the coordinates          */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write model point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = model_pts->pts;
  qsort(p, model_pts->num_pts, sizeof(ModelPt), Compare_ModelPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file identifier\n", MODEL_POINTS);
  fprintf( fp, M_CONTENTS_H, comment_char);
  fprintf( fp, NUMBER_H,     comment_char, model_pts->num_pts);
  fprintf( fp, M_EXPL_H,     comment_char, comment_char);
  fprintf( fp, M_LINE_H,     comment_char);
  
/*------ Print the object points into the file ----------------------------*/
  for (i = 0; i < model_pts->num_pts; i++)
    fprintf( fp, M_FORMAT, p[i].num, p[i].x, p[i].y, p[i].z,
	    p[i].v_x, p[i].v_y, p[i].v_z, 
	    p[i].cv_xy, p[i].cv_xz, p[i].cv_yz);
    
  fclose(fp);

  return (True);
}

/*
------------------------------[ Print_ModelPts ]------------------------------
  This function prints the ModelPts structure to stdout.

  IN : Structure containing read ModelPts 
  RETURN: Nothing
------------------------------[ Print_ModelPts ]------------------------------
*/
void Print_ModelPts(const ModelPts *model_pts)
{
  int i;
  
  printf( " Number of model points %d\n", model_pts->num_pts);
  printf( MP_EXPL_H,  ' ', ' ');
  printf( MP_LINE_H, ' ');
  
  for (i = 0; i < model_pts->num_pts; i++)
    printf(MP_FORMAT, (model_pts->pts[i]).num, 
                   (model_pts->pts[i]).x, 
                   (model_pts->pts[i]).y, 
                   (model_pts->pts[i]).z,

                   (model_pts->pts[i]).v_x, 
                   (model_pts->pts[i]).v_y, 
                   (model_pts->pts[i]).v_z,

                   (model_pts->pts[i]).cv_xy, 
                   (model_pts->pts[i]).cv_xz, 
                   (model_pts->pts[i]).cv_yz
		   );
}


/*
------------------------------[ Free_ModelPts ]--------------------------------
  This function deallocates the memory used by ModelPts.

  IN : Structure containing read ModelPts2D 
  RETURN: Nothing
------------------------------[ Free_ModelPts ]--------------------------------
*/

void Free_ModelPts(ModelPts *modelpts)
{
   free(modelpts->pts);
   free(modelpts);
   modelpts = NULL;
}	




/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
                        C O N T R O L P O I N T S
=========================================================================

----------------------------[ Get_CtrlPts ]----------------------------
  This function reads the file indicated by "filename", and puts the
  read points into a structure of type "CtrlPts" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read Object Points
----------------------------[ Get_CtrlPts ]----------------------------
*/
CtrlPts *Get_CtrlPts(const char *filename)
  {
  FILE *fp;                   /* File Pointer                     */
  int i;                      /* General counter                  */
  CtrlPts *ctrl_pts;          /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* file identifier                  */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for Pixel structure -----------------------------*/
  ctrl_pts = (CtrlPts *)malloc( sizeof(CtrlPts) );

/*------ Initiate the CtrlPts structure ----------------------------------*/
  ctrl_pts->num_pts = 0;
  
/*------ Check the file identifier ---------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != CONTROL_POINTS)
    {
    printf("file identification error, this file ( %s ) does not contain \
control points\n", filename);
    return( NULL );
    }

/*------ Read the file ---------------------------------------------------*/
  while (!feof(fp))
    {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line))
      {
/*------ Create (extra) space for the Pixel that is about to be read -----*/
      if (!ctrl_pts->num_pts) 
        ctrl_pts->pts = (CtrlPt *) malloc( sizeof(CtrlPt ) );
      else
        ctrl_pts->pts = (CtrlPt *) realloc( ctrl_pts->pts, 
                          (ctrl_pts->num_pts + 1) * sizeof(CtrlPt) );

/*------ Extract the measured object point from the read line -------------*/
      sscanf(line, "%d %lf %lf %lf %d",
             &(ctrl_pts->pts[ctrl_pts->num_pts].num),
             &(ctrl_pts->pts[ctrl_pts->num_pts].x),
             &(ctrl_pts->pts[ctrl_pts->num_pts].y),
             &(ctrl_pts->pts[ctrl_pts->num_pts].z),
             &(ctrl_pts->pts[ctrl_pts->num_pts].status) );
             
      ctrl_pts->num_pts++; 
      }
    }  
    
  if (ctrl_pts->num_pts > 0) ctrl_pts->num_pts--; 
  else ctrl_pts->pts = NULL;
  
  fclose(fp);

  return (ctrl_pts);
  }
  
/*
---------------------------------------------------------------------------
  This function is used to sort the array of points in ascending order
---------------------------------------------------------------------------
*/
int Compare_CtrlPts(const void *a, const void *b)
{
  const CtrlPt *ap, *bp;
  
  ap = (const CtrlPt *) a;
  bp = (const CtrlPt *) b;
  return(ap->num - bp->num);
  }

/*
------------------------------[ Put_CtrlPts ]--------------------------------
  This function writes the CtrlPts structure into a file. The name
  of the file is included in the structure.
  
  IN : Structure containing read CtrlPts
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_CtrlPts ]--------------------------------
*/
int Put_CtrlPts(CtrlPts *ctrl_pts, const char *filename)
  {
  FILE *fp;                 /* File Pointer                        */
  int i;                    /* General counter                     */
  CtrlPt *p;                /* Pointer to the coordinates          */
  
/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL)
    {
    fprintf( stderr, "Could not open %s to write contol point database.\n", 
        filename);
    return (False);
    }
    
/*------ Sort the measurements on pointnumber ----------------------------*/
  p = ctrl_pts->pts;
  qsort(p, ctrl_pts->num_pts, sizeof(CtrlPt), Compare_CtrlPts);

/*------ Print the headers in the database file --------------------------*/
  fprintf( fp, "%d # file identifier\n", CONTROL_POINTS);
  fprintf( fp, C_CONTENTS_H, comment_char);
  fprintf( fp, NUMBER_H, comment_char, ctrl_pts->num_pts);
  fprintf( fp, C_STATUS_EXPL_H, comment_char, comment_char, comment_char, 
                                comment_char, comment_char);
  fprintf( fp, C_EXPL_H, comment_char);
  fprintf( fp, C_LINE_H, comment_char);
  
/*------ Print the object points into the file ----------------------------*/
  for (i = 0; i < ctrl_pts->num_pts; i++)
    fprintf( fp, C_FORMAT, p[i].num, p[i].x, p[i].y, p[i].z, p[i].status);
    
  fclose(fp);

  return (True);
  }

/*
------------------------------[ Print_CtrlPts ]------------------------------
  This function prints the CtrlPts structure to stdout.

  IN : Structure containing read CtrlPts 
  RETURN: Nothing
------------------------------[ Print_CtrlPts ]------------------------------
*/
void Print_CtrlPts(const CtrlPts *ctrl_pts)
{
  int i;
  char *status_string[] = {"Full", "Planimetric", "Height", "None"};
  
  printf( NUMBER_H,   ' ', ctrl_pts->num_pts);
  printf( C_EXPL_H,  ' ');
  printf( C_LINE_H,  ' ');
  
  for (i = 0; i < ctrl_pts->num_pts; i++) {
    printf( "%9d%16.3lf%15.3lf%15.3lf", (ctrl_pts->pts[i]).num, 
                   (ctrl_pts->pts[i]).x, 
                   (ctrl_pts->pts[i]).y, 
                   (ctrl_pts->pts[i]).z);
    if (ctrl_pts->pts[i].status < 4 && ctrl_pts->pts[i].status >= 0)
      printf( "      %s\n", status_string[(ctrl_pts->pts[i]).status]);
    else
      printf( "      error in statusfield\n");
  }
}
  

/*
---------------------------------------------------------------------------
  Searches the database for information. if obj_pt->num == 0 then it is 
  looking for the point with the given coordinates. If the coordinates
  are (both) 0 it is looking for the coordinates of the given point.
  
  IN : query structure containing search information
  IN : database to search for the point
  
  RETURN : True if succesful, False if not
---------------------------------------------------------------------------
*/

ObjPt *Search_ObjPts(ObjPt *obj_pt, ObjPts *obj_pts, int *index)
{
  register int i;
  ObjPt *p;
  
  if (obj_pts == NULL || obj_pt == NULL) return( NULL );
  
  p = obj_pts->pts;
  if (obj_pt->num == -1) {
    for (i= 0; i < obj_pts->num_pts; i++, p++)
      if (obj_pt->x == p->x && obj_pt->y == p->y && obj_pt->z == p->z) {
        obj_pt->num = p->num;
        *index = i;
        return( p );
      }
        
  }
  else
    for (i= 0; i < obj_pts->num_pts; i++, p++)
      if (obj_pt->num == p->num) {
        obj_pt->x = p->x;
        obj_pt->y = p->y;
        obj_pt->z = p->z;
        *index = i;
        return( p );
      }

  return( NULL );
}

/*
---------------------------------------------------------------------------
  Searches the database for information. if ctrl_pt->num == 0 then it is 
  looking for the point with the given coordinates. If the coordinates
  are (both) 0 it is looking for the coordinates of the given control point.
  
  IN : query structure containing search information
  IN : database to search for the control point
  
  RETURN : True if succesful, False if not
---------------------------------------------------------------------------
*/

CtrlPt *Search_CtrlPts(CtrlPt *ctrl_pt, CtrlPts *ctrl_pts,
                       int *index)
{
  register int i;
  CtrlPt *p;
  
  if (ctrl_pts == NULL || ctrl_pt == NULL) return( NULL );
  
  p = ctrl_pts->pts;
  if (ctrl_pt->num == -1) {
    for (i= 0; i < ctrl_pts->num_pts; i++, p++)
      if (ctrl_pt->x == p->x && ctrl_pt->y == p->y && ctrl_pt->z == p->z) {
        ctrl_pt->num = p->num;
        *index = i;
        return( p );
      }
        
  }
  else
    for (i= 0; i < ctrl_pts->num_pts; i++, p++)
      if (ctrl_pt->num == p->num) {
        ctrl_pt->x = p->x;
        ctrl_pt->y = p->y;
        ctrl_pt->z = p->z;
        *index = i;
        return( p );
      }

  return( NULL );
}


/*
------------------------------[ Free_CtrlPts ]--------------------------------
  This function deallocates the memory used by CtrlPts.

  IN : Structure containing read CtrlPts 
  RETURN: Nothing
------------------------------[ Free_CtrlPts ]--------------------------------
*/

void Free_CtrlPts(CtrlPts *ctrlpts)
{
   free(ctrlpts->pts);
   free(ctrlpts);
   ctrlpts = NULL;
}	
