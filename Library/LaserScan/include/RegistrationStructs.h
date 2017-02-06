
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
*   File made : March 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Structure definitions for keeping information related to registration.
*--------------------------------------------------------------------*/
#ifndef __REGISTRATION_STRUCTS__h___
#define __REGISTRATION_STRUCTS__h___

#include "Vector3D.h"
#include "Rotation3D.h"
#include "RotationParameter.h"
#include "ExteriorOrientation.h"
#include "ENLSIPFitting.h"
#include "LaserObjects.h"
#include "LaserTransform3D.h"
class LaserPoints;



#define SHOW_DEBUG 0

//--------------------------------------------------------------
//Type for keeping scan and target number together. Used as a key 
//for the map for direct lookup.
struct RegTargetId
//--------------------------------------------------------------
{
	//The id of the target.
	int regTargetNumber;
	
	//The id of the scan
	int scanNumber;
	
	//Constructor.
	RegTargetId(int sn=-1,int on=-1);
		
	//operator ==
	bool operator==(const RegTargetId& o2)const;
		
	//operator >
	bool operator>(const RegTargetId& o2)const;
		
	//operator <
	bool operator<(const RegTargetId& o2)const;
	
};

//--------------------------------------------------------------
//A generic registration target. Keeps type as a string and a list of parameter values.
//--------------------------------------------------------------
struct RegistrationTarget
{
	//id of the target.
	RegTargetId id;
	
	//Type name, like "LaserCylinder", "LaserPlane", etc.
	string type;
	
	//A list of parameters.
	vector<double> parameters;
	
	//Constructor.
	RegistrationTarget(const RegTargetId& a=RegTargetId(),const char* tp="",const vector<double>& pr=vector<double>());
	
	//Print some information to a given file or by default to stderr.
	bool Print(FILE* pFile=stderr);
};
//--------------------------------------------------------------
//struct for transformation. Just has a combination of Rotation and translation
//Applies the transformation to RegistrationTargets directly.
//--------------------------------------------------------------

//--------------------------------------------------------------
//full information for object-parameter based registration.
//--------------------------------------------------------------
struct RegistrationData
{
	//Table of exterior orientations.
	//maps a scan id to its transform.
	typedef map<int,LaserTransform3D> TransformMap;
	TransformMap transforms;
	
	//Table of all targets.
	//map a target-id to its data.
	typedef map<RegTargetId,RegistrationTarget> TargetMap;
	TargetMap targets;
	
	//Map of correspondences.
	typedef map<RegTargetId, vector<RegTargetId> > CorrMap;
	CorrMap correspondences;
	
	//define an iterator type.
	typedef CorrMap::iterator CorrMapIter;
	
	//Clears all maps etc.
	void ClearAll();
	
	//Read from file.
	bool LoadFromFile(const char* fileName);
	
	//Save to file.
	bool SaveToFile(const char* fileName);
	
	//Print to a file or stderr
	bool Print(FILE* pFile=stderr);
	
	//Print corresponding object parameters.
	bool PrintCorrespondences(FILE* pFile=stderr);
	//Get Observation count.
	int GetObsCount();
	
	//Get Parameter Count
	int GetParamCount();
	
	//Check if we have transform for a given id.
	bool HasTransform(int id);
	
	//Load transformvalues from an iterator.
	template<class T>
	bool LoadTransform(int scanId,T vals)
	{
		if(HasTransform(scanId))
		{
			TransformMap::iterator iter = transforms.begin();
			int index = 0;
			for(;iter!=transforms.end();iter++)
			{
				if(iter->first == scanId)
				{
					break;
				}
				index++;
			}
			//We have to find the offset.
			int offset = index*7;
			transforms[scanId].LoadFromVector(vals+offset);
		}
		else
		{
			cerr<<"No transform for id: "<<scanId<<" in LoadTransform()\n";
		}	
	}
	
	template<class T>
	bool SaveTransform(int scanId,T vals)
	{
		if(HasTransform(scanId))
		{
			TransformMap::iterator iter = transforms.begin();
			int index = 0;
			for(;iter!=transforms.end();iter++)
			{
				if(iter->first == scanId)
				{
					break;
				}
				index++;
			}
			//We have to find the offset.
			int offset = index*7;
			transforms[scanId].SaveToVector(vals+offset);
		}
		else
		{
			cerr<<"No transform for id: "<<scanId<<" in LoadTransform()\n";
		}	
	}
	
	
	
	//See how many scans have objects in correspondence map
	int GetUsedScanCount();
};

#endif //__REGISTRATION_STRUCTS__h___
