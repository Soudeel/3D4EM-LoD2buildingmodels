
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
*   Purpose   : Provides general utility functions
*
*--------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <stdarg.h>
#include <time.h>

#include "DebugFunc.h"


#include "GeneralUtility.h"

/** Starts the random generator with current time as seed.
	
    @param 	none
    @return	void
*/
void Randomize()
{
	time_t seed;
	seed = time(&seed);
	srand(seed);
}

/** Generate a random number between the given bounds.
	On the first call of the program intialize the random number generator with time as seed.
	
	@param lower	lower bound of result.
	@param upper 	upper bound of result.
	@return	 a random double in given bounds.
*/		
double GenerateRandom(double lower, double upper)
{
	//initialize once.
	static bool initialized = false;
	if(!initialized)
	{
		initialized = true;
		Randomize();
	}
	
	return ((rand()/(double)RAND_MAX)*(upper-lower)+lower);
	//return ((Ran2())*(upper-lower)+lower);
}

/** Generate Random number by using numerical recipes' random generator.
	
	@param none
	@return double.
*/
double Ran2()
{
	static long idum = -1234567;
	//return ran2(&idum);
}


/** Generate normally distributed random number.
	
	@param mean and std dev
	@return double.
*/
double GenerateRandomG(double stdDev,double mean)
{
  double u1, u2, pi = 4.0 * atan(1.0);
  
  do { u1 = GenerateRandom(0.0, 1.0); } while (u1 == 0.0);
  u2 = GenerateRandom(0.0, 1.0);
  return stdDev * sqrt(-2.0 * log(u1)) * cos(2.0 * pi * u2) + mean;
  // Note: using the sine instead of the cosine would give a second
  //       independent Gaussian distributed value.
}

/** Rotate a point p by angle theta around an arbitrary axis r
    Return the rotated point.
    Positive angles are anticlockwise looking down the axis
    towards the origin.
    Assume right hand coordinate system.
*/
Vector3D 
ArbitraryRotate(const Vector3D& p,double theta,Vector3D r,bool normalize)
{
	Vector3D q(0,0,0);
	double costheta,sintheta;

	if(normalize)
	{
		r = r.Normalize();
	}

	costheta = cos(theta);
	sintheta = sin(theta);
	
	q.X() = q.X() + (costheta + (1 - costheta) * r.X() * r.X()) * p.X();
	q.X() = q.X() + ((1 - costheta) * r.X() * r.Y() - r.Z() * sintheta) * p.Y();
	q.X() = q.X() + ((1 - costheta) * r.X() * r.Z() + r.Y() * sintheta) * p.Z();

	q.Y() = q.Y() + ((1 - costheta) * r.X() * r.Y() + r.Z() * sintheta) * p.X();
	q.Y() = q.Y() + (costheta + (1 - costheta) * r.Y() * r.Y()) * p.Y();
	q.Y() = q.Y() + ((1 - costheta) * r.Y() * r.Z() - r.X() * sintheta) * p.Z();

	q.Z() = q.Z() + ((1 - costheta) * r.X() * r.Z() - r.Y() * sintheta) * p.X();
	q.Z() = q.Z() + ((1 - costheta) * r.Y() * r.Z() + r.X() * sintheta) * p.Y();
	q.Z() = q.Z() + (costheta + (1 - costheta) * r.Z() * r.Z()) * p.Z();

	return(q);
}

/**	Given a non-zero vector generates an orthonormal basis.
	Input vector is in w, if not already normalized set normalize.
	Output vectors u,v,w form an orthonormal basis.
*/
void 
GenerateOrthonormalBasis (Vector3D& u, Vector3D& v,Vector3D& w, bool normalize)
{
	if (normalize)
		w = w.Normalize();

	if ( fabs(w.X()) >=fabs(w.Y()) )
	{
		// W.x or W.z is the largest magnitude component, swap them
		u.X() = -w.Z();
		u.Y() = 0.0;
		u.Z() = +w.X();
	}
	else
	{
		// W.y or W.z is the largest magnitude component, swap them
		u.X() = 0.0f;
		u.Y() = w.Z();
		u.Z() = -w.Y();
	}
	u = u.Normalize();
	v = w.VectorProduct(u);
	v = v.Normalize();
}

/** Generate a random color

	@return Vector3D
*/
Vector3D GenerateRandomColor()
{	
	return Vector3D(GenerateRandom(0.3,1),GenerateRandom(0.3,1),GenerateRandom(0.3,1));
}



/** General function for debugging
*/
void
Debug(Vector3D v,const char* msg)
{
	const char name[]="Vector3D";
	const char* p = (msg)?msg:name;;
	fprintf(stderr,"%s: (%f, %f, %f)\n",p,v.X(),v.Y(),v.Z());
}

void Debug(double v,const char* msg)
{
	const char name[]="Double value: ";
	const char* p = (msg)?msg:name;;
	fprintf(stderr,"%s: %f\n",p,v);
}
void Debug(const char* msg)
{
	const char name[]="Message ";
	const char* p = (msg)?msg:name;;
	fprintf(stderr,"%s\n",p);
}


/*
Checks to see if a file exists
*/
bool FileExists(string fileName)
{
	FILE* pFile = fopen(fileName.c_str(),"rb");
	if(pFile)
	{
		fclose(pFile);
		return true;
	}
	return false;
}


//printf function for strings.
//printf function for strings.
string& vsprintf(string& s, const char *format, va_list ap)
{
	const int buffSize = 1024*16;
	char buff[buffSize];
	
	//Print to internal buffer.
	vsnprintf(buff,buffSize,format,ap);
	
	//Equate to string.
	s = buff;
	
	//return string.
	return s;
}


string& sprintf(string& s, const char *format, ...)
{
	//Print to internal buffer.
	va_list ap;
	va_start(ap, format);
	vsprintf(s,format,ap);
	va_end(ap);
	
	//return string.
	return s;
}


//append and print i.e. string = string + sprintf(string1,...)
string& sprintfa(string& total, const char *format, ...)
{
	string s;
	//Print to internal buffer.
	va_list ap;
	va_start(ap, format);
	vsprintf(s,format,ap);
	va_end(ap);
	
	
	total += s;
	return total;
}

int fprintf(FILE* pFile,const string& s)
{
	return fprintf(pFile,s.c_str());
}
		

