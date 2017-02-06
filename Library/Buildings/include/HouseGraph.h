
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



#include "ObjectPoints2D.h"

/************************************************************/
/*                 HOUSE PARTIONING              	    */
/************************************************************/

struct HouseGraphNode{
      ObjectPoint2D 		pt;
      std::vector<HouseGraphNode*> 	neighbours;
};	

class HouseGraph : public std::vector<HouseGraphNode*>
{
public:
    HouseGraph() {};
    
    ~HouseGraph();
    
    void FindRects(std::vector<ObjectPoints2D> &rs) const;

    void FindRect(const HouseGraphNode* node, ObjectPoints2D &r, 
      std::vector<ObjectPoints2D> &rs, const ObjectPoints2D &visited_nodes) const;
    			
    bool Neighbours(const ObjectPoint2D &node1, const ObjectPoint2D &node2)const;
    
    int GraphPoint(const Position2D &pos, double err_dist) const;
    
    void InsertGraphNode(HouseGraphNode* node, const Position2D &pos1, 
    			const Position2D &pos2);
    			
    void DeleteNeighbour(HouseGraphNode* node, const Position2D &pos);        	
    
    void PrintNode(const HouseGraphNode &node) const;
    void PrintNode(const PointNumber &no) const;    

    void Print() const;
};
