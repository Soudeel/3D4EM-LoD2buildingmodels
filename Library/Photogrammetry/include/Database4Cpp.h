
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



/*                               _\\|//_ 
                                 ( .-. )
------------------------------o00--(_)--00o------------------------------

  Function      : Header file for the libraries that contains in- and 
                  output routines for databases of any kind.
  Author        : R.Th. Ursem
  Creation date : 19 - Jul - 1994

  Modified by   : F.A. van den Heuvel
  Dated         : 27 - Sep - 1996
  Modification  : Image dimensions (in pixels) are added at the end of the
		      structure Interior

  Modified by   : M.G. Vosselman
  Dated         : 26 - Sep - 1997
  Modification  : Added structures for 2D grid specification and 2D
                  object points
  
  Modified by   : M.G. Vosselman
  Dated         : 02 - Feb - 1999
  Modification  : Added label to the line topology structure

  Modified by   : M.G. Vosselman
  Dated         : 12 - Mar - 1999
  Modification  : Added structure for a grid specification including
		      offset and scale of the imaged data
  
  Modified by   : M.G. Vosselman
  Dated         : 18 - Mar - 1999
  Modification  : Added structure for triangulations
  
  Modified by   : F.A. van den Heuvel
  Dated         : 22 - Nov - 1999
  Modification  : Structure for interior orientation extended with BINGO parameters

  Modified by   : George Vosselman
  Dated         : 26 - Mar - 2005
  Modification  : Created two versions, one for inclusion in C programmes (Database.h)
                  and one for inclusion in C++ programmes (this file)
  
  Naming Conventions:
    structures -> typedef structures! Words separated by Capitals
                  ->  "ImageCoordinates"
    variables  -> all lowercase, words separated by underscore
                  ==>  "image_point"
    functions  -> words separated by Capitals and underscores
                  ->  "Display_Image_Points" 

------------------------------------------------------------------------
*/

#ifndef _Database_h_
#define _Database_h_

#include <stdio.h>

#define MAXCHARS 256

#ifndef True
#define True 1
#endif

#ifndef False
#define False 0
#endif

/*                               _\\|//_ 
                                 ( 0-0 )
------------------------------o00--(_)--00o------------------------------
  File identifying constants. If a file doesn't start with a valid id, 
  the contents is not recognized and an error messsage is generated
-------------------------------------------------------------------------
*/

#include "file_id.h"

/*                               _\\|//_ 
                                 ( 0-0 )
------------------------------o00--(_)--00o------------------------------
  Functions used by C I/O routines
-------------------------------------------------------------------------
*/

extern "C" int Is_Comment(const char *);
extern "C" FILE *Open_Compressed_File(const char *, const char *);
extern "C" int Close_Compressed_File(FILE *);
extern "C" char *Name_Compressed_File(const char *);
extern "C" char *Name_Uncompressed_File(const char *);
extern "C" int Compress_File(const char *, const char *, int);


/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
                       I M A G E   P O I N T S
               
  Source  : $DATABASES/src/Lib/ImgPtsDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

/*- Three constants that can be returned from the ImgPts_Exist function -*/
#define IMGPT_NOT_KNOWN    0
#define IMGPT_NUM_EXIST    1
#define IMGPT_COORD_EXIST  2

/*------ Coordinates in pixels ------*/
typedef struct {
  double r, c;                /* measurement in an image (row,col)  */
  int    num;                 /* pointnumber of the measurement     */
  double v_r, cv_rc, v_c;     /* variance and covariance factors    */
} ImgPt;
  
typedef struct {
  char  img_name[MAXCHARS];    /* name of the database to read       */ 
  int   num_pts;               /* Number of points in the database   */
  ImgPt *pts;                 /* Array of measured points           */
} ImgPts;
  
/*-------------[ general functions for image points ]------------------*/
extern "C" ImgPts *Get_ImgPts(const char *filename);
extern "C" int     Put_ImgPts(ImgPts *img_pts, const char *filename);
extern "C" void    Print_ImgPts(const ImgPts *img_pts);
extern "C" void    Free_ImgPts(ImgPts *img_pts);	

/*----------------------[ special functions ]--------------------------*/
extern "C" int    ImgPts_Exist(ImgPt *img_pt, ImgPts *img_pts);
extern "C" ImgPt *Search_ImgPts(ImgPt *img_pt, ImgPts *img_pts);
extern "C" int    Delete_Pix(ImgPts *img_pts, int number);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
                       C A M E R A   P O I N T S
               
  Source  : $DATABASES/src/Lib/CamPtsDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

/*------ Photo Coordinates in mm ------*/
typedef struct {
  double x, y;                /* measurement in an image (x,y)      */
  int    num;                 /* pointnumber of the measurement     */
  double v_x, cv_xy, v_y;     /* variance and covariance factors    */
} CamPt;

typedef struct {
  int   num_pts;               /* Number of points in the database   */
  CamPt *pts;                 /* Array of measured points           */
} CamPts;
  
/*-------------[ general functions for photo points ]------------------*/
extern "C" CamPts *Get_CamPts(const char *filename);
extern "C" int     Put_CamPts(CamPts *cam_pts, const char *filename);
extern "C" void    Print_CamPts(const CamPts *cam_pts);
extern "C" void    Free_CamPts(CamPts *cam_pts);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

               O B J E C T -   A N D   P A S S P O I N T S

  Source  : $DATABASES/src/Lib/3DPointsDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/
typedef struct {
  double x, y, z;                          /* object point coordinates */
  int    num;                              /* point number             */
  double v_x, cv_xy, v_y,                  /* variance and             */
         cv_xz, cv_yz, v_z;                /* covariance factors       */
} ObjPt, ModelPt;

typedef struct {
  int   num_pts;                     /* number of object points in pts */
  ObjPt *pts;                        /* array of 3D points             */
} ObjPts, ModelPts;

/*----------------[ general functions for ObjPts ]---------------------*/
extern "C" ObjPts *Get_ObjPts(const char *filename);
extern "C" int     Put_ObjPts(ObjPts *obj_pts, const char *filename);
extern "C" int     Put_ObjPts2(ObjPts *obj_pts, const char *filename);
extern "C" void    Print_ObjPts(const ObjPts *obj_pts);
extern "C" void    Free_ObjPts(ObjPts *obj_pts);	

/*----------------[ general functions for ModelPts ]-------------------*/
extern "C" ModelPts *Get_ModelPts(const char *filename);
extern "C" int       Put_ModelPts(ModelPts *model_pts, const char *fname);
extern "C" void      Print_ModelPts(const ModelPts *model_pts);
extern "C" void      Free_ModelPts(ModelPts *model_pts);

#define NO_CTRL     3
#define HEIGHT_CTRL 2
#define PLANI_CTRL  1
#define FULL_CTRL   0

/* Definition moved to 3DPointsDatabase.c
static char *status_string[] = {"Full", "Planimetric", 
                                "Height", "None"};
*/

typedef struct {
  double x, y, z;                            /* Ctrl point coordinates */
  int    num;                                /* point number           */
  int    status;          /* height, planimetric or full control point */
} CtrlPt;

typedef struct {
  int num_pts;                        /* number of Ctrl points in pts  */
  CtrlPt *pts;                        /* array of Ctrl points          */
} CtrlPts;

/*----------------[ general functions for 3DPoints ]-------------------*/
extern "C" CtrlPts *Get_CtrlPts(const char *filename);
extern "C" int      Put_CtrlPts(CtrlPts *ctrl_pts, const char *filename);
extern "C" void     Print_CtrlPts(const CtrlPts *ctrl_pts);
extern "C" void     Free_CtrlPts(CtrlPts *ctrl_pts);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
                       I M A G E   L I N E S
               
  Source  : $DATABASES/src/Lib/ImgLinesDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

/*------ Coordinates in pixels ------*/
typedef struct {
  int    num_pts;             /* number of points on the line       */
  ImgPt  *pts;                /* array of image points              */
  int    num;                 /* line number of the measurement     */
  int    label;               /* label of the line                  */
} ImgLine;
  
typedef struct {
  char img_name[MAXCHARS];    /* name of the database to read       */ 
  int  num_lines;             /* number of lines in the database    */
  ImgLine *lines;             /* array of measured lines            */
} ImgLines;
  
/*-------------[ general functions for image lines ]------------------*/
extern "C" ImgLines *Get_ImgLines(const char *filename);
extern "C" int       Put_ImgLines(ImgLines *img_lines, const char *filename);
extern "C" void      Print_ImgLines(const ImgLines *img_lines);
extern "C" void      Free_ImgLines(ImgLines *img_lines); 

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
                       L I N E   T O P O L O G Y
               
  Source  : $DATABASES/src/Lib/TopologyDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

/*------ Coordinates in pixels ------*/
typedef struct {
  int    num;                 /* line number                        */
  int    num_pts;             /* number of points on the line       */
  int    *pts;                /* array of points numbers            */
  int    label;               /* line label                         */
} LineTop;
  
typedef struct {
  char name[MAXCHARS];        /* name of the database to read       */ 
  int  num_lines;             /* number of lines in the database    */
  LineTop *lines;             /* array of measured lines            */
} LineTops;
  
/*-------------[ general functions for image lines ]------------------*/
extern "C" LineTops *Get_LineTops(const char *filename);
extern "C" int       Put_LineTops(LineTops *lines, const char *filename);
extern "C" void      Print_LineTops(const LineTops *lines);
extern "C" void      Free_LineTops(LineTops *lines);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                  I N T E R I O R   O R I E N T A T I O N

  Source  : $DATABASES/src/Lib/InteriorDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/
#define MAXBINGO 31    /* 1+ maximum # of additional parameters of Bingo  */

typedef struct {
  double cc;           /* Camera Constant in mm                           */
  double rh, ch;       /* Image coordinates of the principal point (pix)  */
  double k1, k2, k3;   /* radial lens distortion parameters               */
  double p1, p2;       /* tangential lens distortion parameters           */
  double shear;        /* shear factor                                    */
  double rot;          /* rotation angle in degrees                       */
  double spacing_r;    /* Pixel spacing in row direction (mm)             */
  double spacing_c;    /* Pixel spacing in column direction (mm)          */
  double dim_r, dim_c; /* Image dimensions (in pix: row column)           */
  double bingo_cc;     /* correction to camera constant by Bingo          */
  double bingo_xh, bingo_yh; /* principle point by Bingo                  */
  double bingo[MAXBINGO];  /* format factor + additional parameters of Bingo */
} Interior;

/*----------------------[ general functions ]--------------------------*/
extern "C" Interior *Get_Interior(const char *filename);
extern "C" int       Put_Interior(const Interior *interior, const char *filename);
extern "C" void      Print_Interior(const Interior *interior);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                   E X T E R I O R   O R I E N T A T I O N

  Source  : $DATABASES/src/Lib/ExteriorDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
  General comments :
      Only the coordinates of the camera and the rotation matrix 
      are stored in the file. When the routine Get_Exterior is
      called, these are read from the file, and the other elements
      of the exterior structure are calculated automatically.
  
=========================================================================
*/

/*------ Error codes possible from Get_Exterior ------------------------*/
#define ALL_VALID          0
#define INVALID_ANGLES     1
#define INVALID_QUATERNION 2

typedef struct {
  double x, y, z;          /* Coordinates of the camera in object space */
  double w[2], f[2], k[2]; /* Omega, Phi and Kappa angles of the camera */
  double a, b, c;          /* Quaternion elements                       */
  double rot[3][3];        /* Rotation matrix                           */
  int    status;           /* Constant which holds validity of elements */
} Exterior;

/*----------------------[ general functions ]--------------------------*/
extern "C" Exterior *Get_Exterior(const char *, int *);
extern "C" int       Put_Exterior(const Exterior *, const char *);
extern "C" void      Print_Exterior(const Exterior *);

/*--------------------[ calculating functions ]------------------------*/
extern "C" int Angles_From_Rot(Exterior *exterior);
extern "C" int Rot_From_Angles(Exterior *exterior);

extern "C" int Quat_From_Rot(Exterior *exterior);
extern "C" int Rot_From_Quat(Exterior *exterior);

extern "C" int Calculate_Rotation_Matrix(double, double, double, double mat[3][3]);

extern "C" int Exterior_Direct(double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix, 
		            int a, int b, int c, int d, double disratio, double cc, 
                    Exterior *sol);
extern "C" int Exterior_Indirect(int ngevonden, int *lijstpnr,
                      double *lijstxpix, double *lijstypix, 
		              double *lijstxpixvar, double *lijstypixvar,
                      double *lijstxycovar, double *lijstxpas, 
                      double *lijstypas, double *lijstzpas, 
		              int show_output, double cc, Exterior *sol);
extern "C" int Search_Start_Points(int ngevonden, int *lijstpnr, double *lijstxpi,
                        double *lijstypi, double *lijstxpas, double *lijstypas,
                        double *lijstzpas, int *a, int *b, int *c, int *d);
/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                   R E L A T I V E   O R I E N T A T I O N

  Source  : $DATABASES/src/Lib/RelativeDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
  General comments :
      Only the coordinates of the cameras and the rotation matrices 
      are stored in the file. When the routine Get_Relative is
      called, these are read from the file, and the other elements
      of the exterior structures are calculated automatically.
      
      The data of both cameras are stored in to the same file.
  
=========================================================================
*/

/*----------------------[ general functions ]--------------------------*/
extern "C" int Get_Relative(const char *filename, Exterior *cam_1, Exterior *cam_2);
extern "C" int Put_Relative(Exterior *cam_1, Exterior *cam_2, const char *filename);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                   A B S O L U T E   O R I E N T A T I O N

  Source  : $DATABASES/src/Lib/AbsoluteDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
   
=========================================================================
*/
typedef struct {
  double x, y, z;          /* Translation parameters      */
  double w[2], f[2], k[2]; /* Omega, Phi and Kappa angles */
  double a, b, c;          /* Quaternion elements         */
  double rot[3][3];        /* Rotation matrix             */
  double lambda;           /* scale factor                */
  int status;
} Absolute;   

/*----------------------[ general functions ]--------------------------*/
extern "C" Absolute *Get_Absolute(const char *filename, int *error);
extern "C" int       Put_Absolute(Absolute *absolute, const char *filename);
extern "C" void      Print_Absolute(const Absolute *absolute);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                             2 D   G R I D

  Source  : $DATABASES/src/Lib/GridDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
   
=========================================================================
*/
typedef struct {
  double x, y;             /* Coordinates of upper left pixel */
  double pixelsize;        /* Pixel spacing in (m)            */
} Grid;   

/*----------------------[ general functions ]--------------------------*/
extern "C" Grid   *Get_Grid(const char *filename);
extern "C" int    Put_Grid(const Grid *grid, const char *filename);
extern "C" void   Print_Grid(const Grid *grid);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
                   2 D   O B J E C T    P O I N T S
               
  Source  : $DATABASES/src/Lib/2DPointsDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

/*------ Object Coordinates in m ------*/
typedef struct {
  double x, y;                /* measurement in an object space (x,y) */
  int    num;                 /* pointnumber of the measurement     */
  double v_x, cv_xy, v_y;     /* variance and covariance factors    */
} ObjPt2D;
  
typedef struct {
  int  num_pts;               /* Number of points in the database   */
  ObjPt2D *pts;                 /* Array of measured points           */
} ObjPts2D;
  
/*-------------[ general functions for 2d object points ]-------------*/
extern "C" ObjPts2D *Get_ObjPts2D(const char *filename);
extern "C" int     Put_ObjPts2D(ObjPts2D *objpts2d, const char *filename);
extern "C" void    Print_ObjPts2D(const ObjPts2D *objpts2d);
extern "C" void    Free_ObjPts2D(ObjPts2D *objpts2d);	

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                             3 D   G R I D

  Source  : $DATABASES/src/Lib/Grid3DDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
   
=========================================================================
*/
typedef struct {
  double x, y;             /* Coordinates of upper left pixel */
  double pixelsize;        /* Pixel spacing in (m)            */
  double data_offset;      /* Offset of the imaged data       */
  double data_scale;       /* Scale of the imaged data        */
} Grid3D;   

/*----------------------[ general functions ]--------------------------*/
extern "C" Grid3D   *Get_Grid3D(const char *filename);
extern "C" int      Put_Grid3D(const Grid3D *grid, const char *filename);
extern "C" void     Print_Grid3D(const Grid3D *grid);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                           T R I A N G L E S

  Source  : $DATABASES/src/Lib/TriangleDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
   
=========================================================================
*/
typedef struct
{ int node[3];            /* Point numbers of triangle corner */
  int nb[3];              /* Indices of neighbouring triangles */
} Triangle;

typedef struct
{ int      num_tri;       /* Number of triangles */
  Triangle *tri;          /* List of triangles */
} Triangles;

extern "C" Triangles *Get_Triangles(const char *filename);
extern "C" int       Put_Triangles(const Triangles *tin, const char *filename);
extern "C" void      Free_Triangles(Triangles *tin);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================
  
            2 D   P R O J E C T I V E   T R A N S F O R M A T I O N
               
  Source  : $DATABASES/src/Lib/ProjTrans2DDatabase.c
  Library : $DIGPHOT/lib/libDatabase.a  ( -lDatabase )
  
=========================================================================
*/

extern "C" int Put_ProjTrans(const char *filename, double *parameters);
extern "C" int Get_ProjTrans(const char *filename, double *parameters);

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

               L E A S T   S Q U A R E S   M A T C H I N G
 
  Source  : $MEASURMENTS/src/Lib/lvmatch.c
  Library : $DIGPHOT/lib/libMatching.a  ( -lMatching )
  
=========================================================================
*/

/*------ Constants for the stop_status, selection of criteria -----------*/
#define NO_CRITERIA   0
#define DX_TRANS      1
#define DX_OTHER      2

/*------ Constants for the accept_status, selection of criteria ---------*/
#define SD_TRANS      4
#define CCC           8

/*------ Constants for the transformation type ---------*/
#define MATCH_AFFINE  0
#define MATCH_SHIFTS  1

typedef struct {
  int    output;       /* True if output is wanted                     */
  char  *filename;     /* Name of outputfile, NULL if output to stdout */
  int    max_iter;     /* Maximum number of iterations allowed         */
  int    patch_size;   /* Size of the patch in pixels (square patches) */
  double dx_trans;     /* Translation stop criteria                    */
  double dx_other;     /* Other parameters stop criteria               */
  double sd_trans;     /* Standard deviation of translation            */
  double ccc;          /* Cross correlation criterium                  */
  int    status;       /* Criteria for convergence check               */
  int    gradients;    /* True > gradients from templ, else from patch */
  int    templ_match;  /* True if we're doing template matching        */
  int    debug_info;   /* Don't use this if you're not a programmer    */
  int    trafo;        /* Transformation type to be used in matching   */
} MatchInfo;

/*                               _\\|//_ 
                                 ( O-O )
==============================o00==(_)==00o==============================

                               H P S C A N

  Source  : $DIGPHOT/Adjust/hpscan.f
  Library : $DIGPHOT/lib/libAdjust.a ( -lAdjust )  
  
  General comments :
     Fortran routine van Frank Kenselaar via G. Husti naar ons gekomen.  
=========================================================================
*/
enum adjust_depth { doAdjust, doAll };
enum adjust_corr  { no_corr, unit_corr, diag_corr };

/*--------------------------------------------------------------------
   The next structure is used to give the user information that is 
   specific to the problem at hand and independend of the adjustment
   routine. A fragment of the output of the adjustment routine will
   hopefully explain what the different elements of this structure
   are meant for.
   
   -----------------------[ piece of adjust output ]---------------------
   
Testing and reliability based on conventional alternative hypothesis
   Residuals, w-test (datasnooping)
   Internal reliability, Minimal Detectable Bias       : nabla-y
                         Norm (sqrt)                   : lam-y
   External reliability, Norm all unknowns (sqrt)      : lam-x
   
   Residuals are in 'milimeters'       <--- units member between quotes
   Order per point is 'x, y coodinate' <--- order_string member
 point    residual     w-test   reject?  nabla-y      lam-y      lam-x
---------------------------------------------------------------------------
     1  x -0.010696   0.623607           0.089267   4.637442   2.104987
        y  0.009443   1.165677           0.054795   5.286722   3.297655
     2  x  0.017303   1.625502           0.065345   5.036480   2.879449
        y  0.001813   0.224281           0.059969   5.536728   3.685167
     ^  ^
     |  +-- order_char member
     +----- numbers member with var_num_obs set to false
   
   --------------------[ end of piece of adjust output ]------------------

   --------------------------------------------------------------------*/
     
typedef struct {
   int n_o_p_p;        /* Number of Observations Per Point            */
   FILE *output;       /* File pointer for results                    */
   char *units;        /* Units of the observations (and residuals)   */
   char *order_string; /* Order of the equations (row, column; x,y)   */
   char *order_char;   /* Order of the equations (r c x y z)          */
   int  *numbers;      /* Numbers of the points used for observations */
   int  var_num_obs;   /* If false numbers contains "number of observa-
		 	  tions per point"-elements, else numbers con-
			  tains as much elements as there are observa-
			  tions.                                      */
} AdjustInfo;

/* Declaration of the FORTRAN (!) adjustment routine */
extern "C" int Adjust(int, int, double *, double *, double *, double *,
                      int, int, double, double *, double *, double *, double *,
                      double *, double *, double *, double *, AdjustInfo *);   


#endif     /* Do NOT add anything after this line */
