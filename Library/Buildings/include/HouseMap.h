
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



#ifndef _HouseMap_h_
#define _HouseMap_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  HouseMap  - A list of houses

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "House.h"

class LineTopologies;

/*
--------------------------------------------------------------------------------
                                   HouseMap
--------------------------------------------------------------------------------
*/

class HouseMap : public std::vector<House>
{
	char *name;		        /* Name of the map file		      */
	
	int SizeOfList04(FILE* fd) ;	/* Return the number of fields of     */
					/* an M04 entry of the map file       */

	void CheckConnections();	/* Look for unclosed houses	      */	
					
	void ConnectHouses(HouseMap::iterator i,      /* Connect two unclosed */
		HouseMap::iterator j, int k, int l);  /* houses               */
public:
	HouseMap() 			/* Default constructor                */
		{ name = NULL;}
	
	HouseMap(char *file_name, double xmin, double ymin, 
		double xmax, double ymax);        
					/* Construct by reading a map file    */	
		
	~HouseMap() 			/* Default destructor                 */
		{ delete [] name; }

	int ReadMap(char *file_name, double xmin, double ymin, 
		double xmax, double ymax);        /* Read a map file          */
		
        void MergeClosePoints(double dist);		
        
        void DeleteCollinearPoints();
	
	void Print() const;		/* Print map to stdout                */


	void Convert2LineTops(LineTopologies *, ObjectPoints2D *) const;
			/* Construct the corresponding LineTop structure      */

};	

#endif /* _HouseMap_h_ */   /* Do NOT add anything after this line */
