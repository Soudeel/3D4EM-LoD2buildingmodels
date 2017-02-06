
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
 * \brief Interface to Class AbsoluteOrientation - Absolute orientation of a model
 *
 */
/*!
 * \class AbsoluteOrientation
 * \ingroup POrientation
 * \brief Interface to Class AbsoluteOrientation - Absolute orientation of a model
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Absolute orientation, the 3D similarity transformation between
 * a model coordinate system and an object coordinate system.
 *
 * \remark \li The absolute orientation is described by a rotation matrix
 * from model space to object space, a vector containing the object space 
 * coordinates of the origin of the model space, and a scale factor between 
 * the model and object space. The first two elements are inherited from the 
 * Orientation3D class.
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _AbsoluteOrientation_h_
#define _AbsoluteOrientation_h_

/*
--------------------------------------------------------------------------------
  AbsoluteOrientation   - Absolute orientation of a model
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
// Absolute orientation, the 3D similarity transformation between a model 
// coordinate system and an object coordinate system.
//
// The absolute orientation is described by a rotation matrix from model space
// to object space, a vector containing the object space coordinates of the 
// origin of the model space, and a scale factor between the model and object
// space. The first two elements are inherited from the Orientation3D class.
//
//------------------------------------------------------------------------------

class AbsoluteOrientation : public Orientation3D {

  protected:
    /// Model to object space scale factor
    double lambda;

  public:

    /// Construct default absolute orientation
    AbsoluteOrientation() : Orientation3D() {};

    /// Construct absolute orientation from specified parameters
    /** @param org   Object coordinates of the origin of the model coordinate
                     system
        @param rot   Rotation from model coordinate system to object coordinate
                     system
        @param scale Scale factor between model and object coordinate system
    */
    AbsoluteOrientation(const Vector3D *org, const Rotation3D *rot,
                        double scale)
      : Orientation3D(org, rot)
      {lambda = scale;}

    /// Construct absolute orientation by reading parameters from a file
    /** @param filename Name of the file with absolute orientation parameters
    */
    AbsoluteOrientation(char *filename) : Orientation3D()
      {Read(filename);} 

    /// Construct absolute orientation by converting C structure Absolute
    /** @param absor C structure with absolute orientation parameters
    */
    AbsoluteOrientation(Absolute *absor) : Orientation3D()
      {C2Cpp(absor);}

    /// Copy constructor
    AbsoluteOrientation(const AbsoluteOrientation &absor);	

    /// Destructor
    ~AbsoluteOrientation() {};

    /// Copy assignament
    AbsoluteOrientation& operator=(const AbsoluteOrientation &absor);	

    /// Returns a constant reference to the object
    const AbsoluteOrientation& AbsOrientRef() const
     	{ return *this; }
      
    /// Returns a writeble reference to the object   
    AbsoluteOrientation& AbsOrientRef()
	{ return *this; }

    /// Conversion from C to C++ object
    void C2Cpp(Absolute *);
 
    /// Conversion from C++ to C object
    void Cpp2C(Absolute **) const;
    
    /// Read absolute orientation from a file
    /** @param filename Name of the file with absolute orientation parameters
    */
    int Read(char *filename);
    
    /// Write absolute orientation to a file
    /** @param filename Name of the file for the absolute orientation parameters
    */
    int Write(char *filename) const;

    /// Print absolute orientation parameters to stdout
    void Print() const;
};

#endif /* _AbsoluteOrientation_h_ */   /* Do NOT add anything after this line */
