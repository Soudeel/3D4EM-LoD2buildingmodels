
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : July 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Provides general utility functions.
*
*--------------------------------------------------------------------*/
#ifndef _GENERAL_UTILITY_H_
#define _GENERAL_UTILITY_H_
#include <string>
#include <Vector3D.h>
#include <cstdio> 
#include <stdarg.h>
using std::string;

/** Starts the random generator with current time as seed.
	
    @param 	none
    @return	void
*/
void Randomize();

/** Generate a random number between the given bounds.
	On the first call of the program intialize the random number generator with time as seed.
	
	@param lower	lower bound of result.
	@param upper 	upper bound of result.
	@return	 a random double in given bounds.
*/		
double 
GenerateRandom(double lower=0, double upper=1);

/** Generate Normally distributed Random number by using numerical recipes' random generator.
	
	@param mean and std dev
	@return double.
*/
double GenerateRandomG(double stdDev=1,double mean=0);


/** Rotate a point p by angle theta around an arbitrary axis r
    Return the rotated point.
    Positive angles are anticlockwise looking down the axis
    towards the origin.
    Assume right hand coordinate system.
*/
Vector3D 
ArbitraryRotate(const Vector3D& p,double theta,Vector3D r,bool normalize);

/**	Given a non-zero vector generates an orthonormal basis.
	Input vector is in w, if not already normalized set normalize.
	Output vectors u,v,w form an orthonormal basis.
*/
void 
GenerateOrthonormalBasis (Vector3D& u, Vector3D& v,Vector3D& w, bool normalize);

/** Generate Random number by using numerical recipes' random generator.
	
	@param none
	@return double.
*/
double Ran2();

/** Generate a random color

	@return Vector3D
*/
Vector3D GenerateRandomColor();

/*
Checks to see if a file exists
*/
bool FileExists(string fileName);

			
/** General functions for debugging
*/
void Debug(Vector3D v,const char* msg=NULL);		
void Debug(double v,const char* msg=NULL);
void Debug(const char* msg);

//printf function for strings.
string& vsprintf(string& s, const char *format, va_list ap);
string& sprintf(string& s, const char *format, ...);	
//append and print i.e. string = string + sprintf(string1,...)
string& sprintfa(string& s, const char *format, ...);	

int fprintf(FILE* pFile,const string& s);


#endif //_GENERAL_UTILITY_H_


