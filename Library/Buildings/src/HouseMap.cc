
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
--------------------------------------------------------------------------------
 Collection of functions for class HouseMap

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "HouseMap.h"
#include "LineTopologies.h"

/* 
#define XHOUSE1	205700
#define YHOUSE1	489220
#define XHOUSE2	205750
#define YHOUSE2	489250
*/

/*
// school
#define XHOUSE1	205600
#define YHOUSE1	489150
#define XHOUSE2	205750
#define YHOUSE2	489250
*/

/*
// bottom
#define XHOUSE1	205600
#define YHOUSE1	488900
#define XHOUSE2	205700
#define YHOUSE2	489000
*/
/*
// big area
#define XHOUSE1	205600
#define YHOUSE1	488900
#define XHOUSE2	205700
#define YHOUSE2	489250
*/
/*
// - H.G -reg1 
#define XHOUSE1	205570
#define YHOUSE1	489250
#define XHOUSE2	205715
#define YHOUSE2	489320
*/

// - H.G -reg2 
#define XHOUSE1	205530
#define YHOUSE1	488850
#define XHOUSE2	205755
#define YHOUSE2	489400

/*
--------------------------------------------------------------------------------
                                Constructor
--------------------------------------------------------------------------------
*/


HouseMap::HouseMap(char *file_name, double xmin, double ymin, 
		double xmax, double ymax)
{
	name = new char[strlen(file_name)+1];
	strcpy(name, file_name);
	ReadMap(name, xmin, ymin, xmax, ymax);
	CheckConnections(); 
}	


/*
--------------------------------------------------------------------------------
                                Read the map from a map file
--------------------------------------------------------------------------------
*/

int HouseMap::ReadMap(char *name, double xmin, double ymin, 
		double xmax, double ymax)
{
printf("Reading map ...\n");
	FILE* fd;
	/*unsigned*/ char buff[67];	
	/*unsigned*/ char s[]="MB0";	/* search string for houses */
	int pos, cnt;
	long nr=0, fd_pos, nrpt = 0;
	int bfound=0, bHouse;	
	House* pHouse = NULL;
	ObjectPoint2D point;
	double cx, cy;
	/*unsigned*/ char tmp[10];
	int end04; 	

	if (!(fd=fopen(name,"rb")))
	{
		printf("Error: Open file %s\n", name);
		return 0;
	}

	/* 1st record */
	fread(buff, 1, 66, fd);
	buff[66]='\0';

	/* 2nd record */
	fread(buff, 1, 66, fd);
	buff[66]='\0';

	/* next records */
	while (!feof(fd))
	{
		fread(buff, 1, 66, fd);

		if ((buff[0] == '0') && (buff[1] == '3'))
		{
			/* record type 3 */
			bfound = 0;
		    if (strstr(buff, s))
				bfound = 1;
		}

		if ((buff[0] == '0') && (buff[1] == '4'))
		{
			/* record type 4 */
			if (bfound)
			{
				end04 = 0;
				cnt = 2;
				if (buff[63] == '0')
				{
					fd_pos = ftell(fd);
					cnt += 3*SizeOfList04(fd);
					fseek(fd, fd_pos, 0);
				}
                if (pHouse)
                   delete pHouse;		
				pHouse = new House(nr);
				bHouse = 0;
				while(!end04)
				{
					pos = 2;
					while(pos < 62)
					{
						if (buff[pos] == 'X')
						{
							strncpy(tmp, buff + pos + 1, 9);
							tmp[9] = '\0';
							cx = atol(tmp);
							cx = cx / 1000; //(mm -> m)
						}
						if (buff[pos] == 'Y')
						{
							strncpy(tmp, buff + pos + 1, 9);
							tmp[9] = '\0';
							cy = atol(tmp);
							cy = cy / 1000;
							point = ObjectPoint2D(cx, cy, nrpt++, 0, 0, 0);
//							if ((labs(point.GetX()-XHOUSE)< DIFFX)
//								&& (labs(point.GetY()-YHOUSE)<DIFFY))
							if (point.GetX()>xmin && point.GetX()<xmax
								&& point.GetY()>ymin && point.GetY()<ymax)
								{
									bHouse = 1;
									//nrpt++;
								}	
							pHouse->push_back(point);
						}
						pos += 10;
						if (pos == 62)
						{
							if (buff[63] == '0')
								fread(buff, 1, 66, fd);
							else end04 = 1;
						}
					} //while pos
				} // while end04
				if (bHouse == 1)
					if (pHouse->size() >= 2)
					{
						push_back(*pHouse);
						nr++;
					}
				bfound = 0;
			}
		}

		if ((buff[0] == '0') && (buff[1] == '5'))
		{
			/* record type 5 - symbol */
			/* nothing to do */
			bfound = 0;
		}

		if ((buff[0] == '0') && (buff[1] == '6'))
		{
			/* record type 6 - text */
			/* nothing to do */
			bfound = 0;
		}
	}

	fclose(fd);

printf("Reading completed ... \n");	
	return 1;
}	

/*
--------------------------------------------------------------------------------
                         Print the list of the houses to stdout
--------------------------------------------------------------------------------
*/
		

void HouseMap::Print() const
{ 	
	printf("No of buildings %d\n", size());
	HouseMap::const_iterator i;
	//House::const_iterator j;
	ObjectPoint2D pt;

	for(i = begin(); i < end(); i++)	
	{
		printf("Building number %d:   Nr of points: %d\n", 
					i->Number(), i->size());
/*		for(j = i->begin(); j < i->end(); j++)
		{
			long cx = j->X();
			long cy = j->Y();
			printf("% d: %ld, %ld   ", j->Number(), cx, cy);
		}
		printf("\n"); */
	}
}

/*
--------------------------------------------------------------------------------
            Get the number of fields of an M04 entry from the map file
--------------------------------------------------------------------------------
*/


int HouseMap::SizeOfList04(FILE* fd)
{
	unsigned char buff[66];
	int n=0;
	
	do{
		fread(buff,1,66,fd);
		n++;
	}while (buff[63]=='0');
	
	return n;
}


/*
--------------------------------------------------------------------------------
                                Look for unclosed houses
--------------------------------------------------------------------------------
*/

void HouseMap::CheckConnections()
{
printf("Check Connections ...\n");
	ObjectPoint2D pt1, pt2, pt;
	int bConnect=0;
    HouseMap::iterator i, k;  
    House house, house_k;
    i = begin();
	while(i < end())	
	{
		bConnect=0;
		house = *i; 
		if (house.IsComplet())
			i++;
		else
		{
			pt1 = house[0];
			pt2 = house[house.size() - 1];
			for(k = i + 1; k < end(); k++)
			{
				house_k = *k;
				if (house_k.IsComplet());
				else
				{
					int m = 0;
					for(int l = 0; l < 2; l++)
					{
						pt = house_k[m];
						//if (pt1 == pt)
						if (pt1.vect() == pt.vect()) 
						{
							printf("Connect %d, %d\n", house.Number(), house_k.Number());
							bConnect=1;
							ConnectHouses(i, k, 0, l);
							break;
						}
						//if (pt2 == pt)
						if (pt2.vect() == pt.vect())
						{
							printf("Connect %d, %d\n", house.Number(), house_k.Number());
							bConnect=1;
							ConnectHouses(i, k, 1, l);
							break;
						}

						m = house_k.size()-1;
					}
				}
                if (bConnect)
                  break;
			}
			if (!bConnect)
				i++;
	 } // else
	}
printf("Check Connections Completed...\n");	
}

/*
--------------------------------------------------------------------------------
                                Connect two houses
--------------------------------------------------------------------------------
*/

void HouseMap::ConnectHouses(HouseMap::iterator i, HouseMap::iterator j, int k, int l)
{
	House house1 = *i;
	House house2 = *j;
	House new_house(house1.Number());

	if ((l == 0) && (k == 0))
	{
		for(int ii = house1.size() - 1; ii >= 0; ii--)
			new_house.push_back(house1[ii]);
		for(int jj = 1; jj < house2.size(); jj++)
			new_house.push_back(house2[jj]);
	}
	if (( l!= 0) && (k == 0))
	{
		for(int ii = house1.size() - 1 ; ii >= 0; ii--)
			new_house.push_back(house1[ii]);
		for(int jj = house2.size() - 2; jj >= 0; jj--)
			new_house.push_back(house2[jj]);
	}
	if ((l == 0) && (k == 1))
	{
		for(int ii = 0; ii < house1.size(); ii++)
			new_house.push_back(house1[ii]);
		for(int jj = 1 ; jj < house2.size(); jj++)
			new_house.push_back(house2[jj]);
	}
	if ((l != 0) && (k == 1))
	{
		for(int ii = 0; ii < house1.size() ;ii++)
			new_house.push_back(house1[ii]);
		for(int jj = house2.size()-2; jj >= 0; jj--)
			new_house.push_back(house2[jj]);
	}

	erase(j);
	erase(i);
	insert(i, new_house);
}


void HouseMap::MergeClosePoints(double dist)
{
   HouseMap::iterator i;
   
   for(i = begin(); i < end(); i++)	
      i->MergeClosePoints(dist);
}      


void HouseMap::DeleteCollinearPoints()
{
   HouseMap::iterator i;
   
   for(i = begin(); i < end(); i++)	
      i->DeleteCollinearPoints();
}      
	

void HouseMap::Convert2LineTops(LineTopologies *lines, ObjectPoints2D *objpts)const
{
   HouseMap::const_iterator i;
   LineTopology line;
   
   for(i = begin(); i < end(); i++)	
   {
      line = LineTopology(i->Number());
      i->Convert2LineTop(&line, objpts); 
      if (!line.empty())
      {
	 lines->push_back(line);
	 line.erase(line.begin(), line.end());
      }  
   }	
}
