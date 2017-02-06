
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



/*!
 * \file
 * \brief Interface to Class ExteriorOrientation - Exterior orientation of a camera
 *
 */
/*!
 * \class ExteriorOrientation
 * \ingroup POrientation
 * \brief Interface to Class ExteriorOrientation - Exterior orientation of a camera
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li The exterior orientation of a camera recording is defined by the object
 *  coordinates of the projection centre and the rotation from the camera
 *  coordinate system to the object coordinate system.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */



#ifndef _ExteriorOrientation_h_
#define _ExteriorOrientation_h_

/*
--------------------------------------------------------------------------------
  ExteriorOrientation   - Exterior orientation of a camera
--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Orientation3D.h"
#include "Database4Cpp.h"

//------------------------------------------------------------------------------
// Exterior orientation
/* The exterior orientation of a camera recording is defined by the object
    coordinates of the projection centre and the rotation from the camera
    coordinate system to the object coordinate system.
*/
//------------------------------------------------------------------------------

class ExteriorOrientation : public Orientation3D {

  protected:

  public:
    /// Default constructor
    ExteriorOrientation() : Orientation3D() {}

    /// Construct from a projection centre and a rotation
    ExteriorOrientation(const Vector3D *projection_centre,
                        const Rotation3D *rot)
      : Orientation3D(projection_centre, rot) {}

    /// Construct by reading an exterior orientation file
    ExteriorOrientation(const char *filename) : Orientation3D()
      {Read(filename);}

    /// Construct by converting a C object
    ExteriorOrientation(Exterior *extor) : Orientation3D()
      {C2Cpp(extor);}
	
    /// Default destructor
    ~ExteriorOrientation() {};

    /// Conversion from C to C++ object
    void C2Cpp(Exterior *);
 
    /// Conversion from C++ to C object
    void Cpp2C(Exterior **) const;
    
    /// Read exterior orientation from a file
    int Read(const char *filename);
    
    /// Write exterior orientation to a file
    int Write(const char *filename) const;

    /// Print the exterior orientation data to stdout
    void Print() const;
    
    ///Direct spatial resection
    int Direct(int num_pts, int *lijstpnr, double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix,  double cc);
    ///Indirect spatial resection
    int InDirect(int num_pts, int *lijstpnr, double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix, 
		            double *lijstxpixvar, double *lijstypixvar,double *lijstxycovar, double cc);		            
};
#endif /* _ExteriorOrientation_h_ */   /* Do NOT add anything after this line */
