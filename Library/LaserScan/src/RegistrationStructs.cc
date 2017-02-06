
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
#include "Vector3D.h"
#include "Rotation3D.h"
#include "RotationParameter.h"
#include "ExteriorOrientation.h"
#include "ENLSIPFitting.h"
#include "LaserObjects.h"
#include "LaserTransform3D.h"
#include "RegistrationStructs.h"
#include "TextFileUtilities.h"
class LaserPoints;



#define SHOW_DEBUG 0


//Constructor.
RegTargetId::RegTargetId(int sn,int on)
:regTargetNumber(on),scanNumber(sn)
{

}
	
//operator ==
bool RegTargetId::operator==(const RegTargetId& o2)const
{
	return (this->regTargetNumber==o2.regTargetNumber
		&&  this->scanNumber==o2.scanNumber);
}
	
//operator >
bool RegTargetId::operator>(const RegTargetId& o2)const
{
	if(this->scanNumber>o2.scanNumber)
		return true;
	else if(this->scanNumber<o2.scanNumber)
		return false;
	else
		return this->regTargetNumber>o2.regTargetNumber;
}
	
//operator <
bool RegTargetId::operator<(const RegTargetId& o2)const
{
	if(this->scanNumber < o2.scanNumber)
		return true;
	else if(this->scanNumber > o2.scanNumber)
		return false;
	else
		return this->regTargetNumber<o2.regTargetNumber;
}


//Constructor.
RegistrationTarget::RegistrationTarget(const RegTargetId& a,const char* tp,const vector<double>& pr)
:id(a),type(tp),parameters(pr)
{

}

//Print some information to a given file or by default to stderr.
bool RegistrationTarget::Print(FILE* pFile)
{
	fprintf(pFile,"Target: %s id:(%d, %d) ",type.c_str(),id.scanNumber,id.regTargetNumber);
	fprintf(pFile," params< ");

	for(int i=0;i<parameters.size();i++)
		fprintf(pFile,"%f  ",parameters[i]);

	fprintf(pFile,">\n ");

	return true;
}

//--------------------------------------------------------------
//struct for transformation. Just has a combination of Rotation and translation
//Applies the transformation to RegistrationTargets directly.
//--------------------------------------------------------------

//--------------------------------------------------------------
//full information for object-parameter based registration.
//--------------------------------------------------------------
	//Clears all maps etc.
void RegistrationData::ClearAll()
{
	transforms.clear();
	targets.clear();
	correspondences.clear();
}

//Print to a file or stderr
bool RegistrationData::Print(FILE* pFile)
{
	fprintf(pFile,"Registration Data %p\n",(void*)this);

	fprintf(pFile,"%d transform objects\n",transforms.size());
	fprintf(pFile,"%d target objects\n",targets.size());
	fprintf(pFile,"%d correspondence objects\n",correspondences.size());

	for(TargetMap::iterator i=targets.begin();i!=targets.end();i++)
		i->second.Print(pFile);
}

//Print corresponding object parameters.
bool RegistrationData::PrintCorrespondences(FILE* pFile)
{
	fprintf(pFile,"%d Correspondence objects\n",correspondences.size());

	for(CorrMapIter i=correspondences.begin();i!=correspondences.end();i++)
	{
		//Get the reference and registration target.
		RegistrationTarget tRef = this->targets[i->first];
		RegistrationTarget tReg = this->targets[i->second[0]];

		vector<double> refParams = tRef.parameters;

		int scanNumber = i->second[0].scanNumber;
		vector<double> trParams = this->transforms[scanNumber].ApplyToObject(tReg);

		fprintf(pFile,"Ref: ");
		for(int j=0;j<refParams.size();j++)
		{
			fprintf(pFile,"%8.3f   ",refParams[j]);
		}
		fprintf(pFile,"\n");

		fprintf(pFile,"Reg: ");
		for(int j=0;j<trParams.size();j++)
		{
			fprintf(pFile,"%8.3f   ",trParams[j]);
		}
		fprintf(pFile,"\n");
	}
}

//Check if we have transform for a given id.
bool RegistrationData::HasTransform(int id)
{
	return (transforms.find(id) != transforms.end());
}
	
		
//See how many scans have objects in correspondence map
int RegistrationData::GetUsedScanCount()
{
	set<int> scanSet;

	for(CorrMapIter it=correspondences.begin();it!=correspondences.end();it++)
	{
		scanSet.insert(it->first.scanNumber);
		vector<RegTargetId> v = it->second;

		for(int i=0;i<v.size();i++)
			scanSet.insert(v[i].scanNumber);
	}

	//remove the negative number.
	set<int>::iterator iter = scanSet.begin();

	int count = 0;
	for(;iter!=scanSet.end();iter++)
	{
		if(*iter>=0)
			count++;
	}
	return count;	
}

//Save to file.
bool RegistrationData::SaveToFile(const char* fileName)
{
	
}

//Read from file.
bool RegistrationData::LoadFromFile(const char* fileName)
{
	FILE* pFile = fopen(fileName,"rt");
	
	if(!pFile)
	{
		return false;
	}
	
	cerr<<"Loading RegistrationData from "<<fileName<<"...";
	while(!feof(pFile))
	{
		vector<string> tokens = ReadTokensFromLine(pFile);
		
		if(tokens.empty())
			continue;
		
		//Its a comment so skip it.
		if(tokens[0][0]=='#')
			continue;
		
		
		if(tokens[0]=="TargetObject")
		{
			//Target Object has the following format.
			//"TargetObject" scan_id object_id Object_type Object_parameters
			
			//Make an id.
			int scan_id = atoi(tokens[1].c_str());
			int object_id = atoi(tokens[2].c_str());
			RegTargetId id(scan_id,object_id);
			
			//Get the parameters.
			vector<double> params = ConvertToDouble(tokens.begin()+4,tokens.end());
						
			if(tokens[3]=="LaserPlane")
			{	
				
			}
			else if(tokens[3]=="LaserCylinder")
			{
				
			}
			else if(tokens[3]=="LaserTorus")
			{
				
			}
			else if(tokens[3]=="LaserSphere")
			{
				
			}
			else
			{
				cerr<<"Uknown type "<<tokens[3]<<" in "<<fileName<<endl;
				continue;
			}
			RegistrationTarget target(id,tokens[3].c_str(),params);
			
			//Add the target to our map.
			targets[id] = target;
		}
		else if(tokens[0] == "Correspondence")
		{
			//first one is the ref object.
			RegTargetId ref_id(atoi(tokens[1].c_str()),atoi(tokens[2].c_str()));
			
			vector<int> ids = ConvertToInt(tokens.begin()+3,tokens.end());
			
			for(int i=0;i<ids.size();i+=2)
				correspondences[ref_id].push_back(RegTargetId(ids[i],ids[i+1]));		
		}
		else if(tokens[0]=="Transform")
		{
			//Get the parameters.
			int scan_id = atoi(tokens[1].c_str());
			vector<double> params = ConvertToDouble(tokens.begin()+2,tokens.end());
						
			Vector3D a;
			
			Vector3D trans(params[4],params[5],params[6]);
			
			QuaternionRotation rot(params[0],params[1],params[2],params[3]);
			
			LaserTransform3D transform(rot,trans);
			
			transform.Print();
			
			//Add the transform to the table.
			transforms[scan_id] = transform;
			
			cerr<<scan_id<<endl;
			transforms[scan_id].Print();
		}
		else
		{
			cerr<<"Uknown base type "<<tokens[0]<<" in "<<fileName<<endl;
		}
	}
	cerr<<"Done!!!\n";
}


//Get Observation count.
int RegistrationData::GetObsCount()
{
	int count = 0;
	for(RegistrationData::CorrMap::iterator i=this->correspondences.begin();i!=this->correspondences.end();i++)
	{
		//Get the reference and registration target.
		RegistrationTarget tRef = this->targets[i->first];
		RegistrationTarget tReg = this->targets[i->second[0]];
		
		count += tReg.parameters.size();
	}
	return count;
}

//Get Parameter Count
int RegistrationData::GetParamCount()
{
	//We need seven parameters per scan.
	return this->transforms.size()*7;	
}
