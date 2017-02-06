
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



/*********************************************************************
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Orientation3D
*
**********************************************************************/

#include <math.h>

#include "Orientation3D.h"
#include "RotationParameter.h"

// Copy constructor
Orientation3D::Orientation3D(const Orientation3D &p) :
	Vector3D(p.vect()), Rotation3D(p.rotation()) {}

// Copy assignment
const Orientation3D &Orientation3D::operator = (const Orientation3D &p) {
    // Check for self assignment
    if (this == &p) return *this;
    memmove(this, &p, sizeof(Orientation3D));
    return *this;
}

/*
transf Orientation3D::as_transf() const {

	position p = as_position();
	transf t = coordinate_transf( p,
		unit_vector(r[0][0], r[1][0], r[2][0]),
		unit_vector(r[0][1], r[1][1], r[2][1]));
	return t;
}


const Orientation3D &Orientation3D::operator = (const transf &t) {

	vect() = t.translation();
	rotation() = t.affine();
	return *this;
}
*/


// Orientation3D constructor **********************************************
/*
Orientation3D::Orientation3D(char *fn) {		// compatible with Database.h
	int err;
	char *file = Global.projectHeader->FileLocation(fn);
	Exterior *e = Get_Exterior(file, &err);
	*this = *e;
	free(file);
	free(e);
}
*/


// Orientation3D operators **********************************************

const Orientation3D &Orientation3D::operator=(const Exterior &e) {

	x[0] = e.x; x[1] = e.y; x[2] = e.z;
	for (int j=0; j<W_DIM; j++)
		for (int i=0; i<W_DIM; i++)	r[j][i] = e.rot[j][i];
	return *this;
}

const Orientation3D &Orientation3D::operator *=(const Orientation3D &o) {

	Orientation3D or1 = *this * o;
	*this = or1;
	return *this;
}

Orientation3D Orientation3D::Inverse() const {

	Rotation3D transp = Transpose();
	Vector3D pos = transp*vect();
	pos *= -1;
	Orientation3D o(pos, transp);
	return o;
}


Orientation3D operator*(const Orientation3D &o1, const Orientation3D &o2) {

	Orientation3D o;
	o.rotation() = o1.rotation()*o2.rotation();
	o.vect() = o1.rotation()*o2.vect() + o1.vect();
	return o;
}

