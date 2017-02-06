
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
 * \brief Interface to Class InteriorOrientation - Interior orientation of a camera
 *
 */
/*!
 * \class InteriorOrientation
 * \ingroup POrientation
 * \brief Interface to Class InteriorOrientation - Interior orientation of a camera
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Interior orientation
 * The interior orientation describes the relation between the image coordinate
 *   system and the camera coordinate systems. It is parameterised by the
 *   pixel coordinates of the principle point, the camera constant, the pixel
 *   sizes (in row and column direction), lens distortion parameters and a shear
 *   and rotation parameter.
 *
 *   NOTE: this class needs to be extended with BINGO parameters that are
 *   contained in the C structure Interior.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _InteriorOrientation_h_
#define _InteriorOrientation_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  InteriorOrientation   - Interior orientation of a camera

--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Database4Cpp.h"

//------------------------------------------------------------------------------
/// Interior orientation
/** The interior orientation describes the relation between the image coordinate
    system and the camera coordinate systems. It is parameterised by the
    pixel coordinates of the principle point, the camera constant, the pixel
    sizes (in row and column direction), lens distortion parameters and a shear
    and rotation parameter.

    NOTE: this class needs to be extended with BINGO parameters that are
    contained in the C structure Interior.
*/
//------------------------------------------------------------------------------

class InteriorOrientation {

  protected:
    /// Camera constant (mm)
    double cc;

    /// Row coordinate of the principal point (pixel)
    double rh;

    /// Column coordinate of the principal point (pixel)
    double ch;

    /// Radial lens distortion parameter k1
    double k1;

    /// Radial lens distortion parameter k2
    double k2;

    /// Radial lens distortion parameter k3
    double k3;

    /// Lens decentering distortion parameter p1
    double p1;

    /// Lens decentering distortion parameter p2
    double p2;

    /// Shear factor
    double shear;

    /// Rotation angle (degrees)
    double rot;

    /// Pixel spacing in row direction (mm)
    double spacing_r;

    /// Pixel spacing in row direction (mm)
    double spacing_c;

    /// Number of image rows
    double dim_r;

    /// Number of image columns
    double dim_c;

  public:
    /// Default constructor
    InteriorOrientation()
      { cc = rh = ch = k1 = k2 = k3 = p1 = p2 = 0.0;
        shear = rot = spacing_r = spacing_c = 0.0;
        dim_r = dim_c = 0;
      }

    /// Construct from specified values
    /** @param pcc    Camera constant (mm)
        @param prh    Row coordinate of principle point
        @param pch    Column coordinate of principle point
        @param pk1    Radial lens distortion parameter k1
        @param pk2    Radial lens distortion parameter k2
        @param pk3    Radial lens distortion parameter k3
        @param pp1    Lens decentering distortion parameter p1
        @param pp2    Lens decentering distortion parameter p2
        @param pshear Shear factor between image and camera coordinate systems
        @param prot   Rotation between image and camera coordinate systems
                      (degrees)
        @param psr    Pixel spacing in row direction
        @param psc    Pixel spacing in column direction
        @param pdr    Number of rows
        @param pdc    Number of columns
    */
    InteriorOrientation(double pcc, double prh, double pch,
    		        double pk1, double pk2, double pk3,
    		        double pp1, double pp2, double pshear, 
    		        double prot, double psr, double psc,
    		        double pdr, double pdc)
    	{ cc = pcc; rh = prh; ch = pch; 
    	  k1 = pk1; k2 = pk2; k3 = pk3;
    	  p1 = pp1; p2 = pp2; shear = pshear;
    	  rot = prot; spacing_r = psr; spacing_c = psc;
    	  dim_r = pdr; dim_c = pdc;
    	}  	 	

    /// Construct by reading a interior orientation file
    InteriorOrientation(const char *filename)
    	{ Read(filename); }         	

    /// Construct by converting a C object
    InteriorOrientation(Interior *into)
      { C2Cpp(into); }
	
    /// Default destructor
    ~InteriorOrientation() {};

    /// Conversion from C to C++ object
    void C2Cpp(Interior *);
 
    /// Conversion from C++ to C object
    void Cpp2C(Interior **) const;
    
    /// Read interior orientation from a file
    /** @param filename File with interior orientation data
        @return Success status, 0 - failure, 1 - success
    */
    int Read(const char *filename);
    
    /// Write interior orientation to a file
    /** @param filename File for interior orientation data
        @return Success status, 0 - failure, 1 - success
    */
    int Write(const char *filename) const;

    /// Print the interior orientation data to stdout
    void Print() const;

    /// Return the camera constant
    double CameraConstant() const 
    	{ return cc; }
    	
    /// Return the pixel size in X direction
    double SpacingX() const
        { return spacing_c; }		

    /// Return the pixel size in Y direction
    double SpacingY() const
        { return spacing_r; }		

    /// Return the principal point coordinate in X (=column) direction
    double PrincipalPointX() const
        { return ch; }

    /// Return the principal point coordinate in Y (=row) direction
    double PrincipalPointY() const
        { return rh; }
        
    /// Return the number of rows
    double DimensionX() const
        { return dim_c; }		

    /// Return the number of columns
    double DimensionY() const
        { return dim_r; }	
};
#endif /* _InteriorOrientation_h_ */   /* Do NOT add anything after this line */
