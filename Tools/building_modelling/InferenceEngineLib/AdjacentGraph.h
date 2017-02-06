/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: April 9, 2012
 |
 |
------------------------------------------------------------------- */
#ifndef _CONSTRAINT_ADJACENT_GRAPH_H_
#define _CONSTRAINT_ADJACENT_GRAPH_H_

#include "InferenceEngine.h"
//#include "InferenceEngine_DLL.h"
#include "LineTopologies.h" 
#include <vector>
#include <map>
#include <float.h>

class LineTopologies;
class PointNumberList;
class LaserPoints;

	class IEDll_API GTreeNode {
	public:
		int m_nNNum;//node num
		GTreeNode* m_pFather;
		std::vector<GTreeNode*> m_Children;
	public:
		GTreeNode();
		GTreeNode (GTreeNode* pFather, int nNodeNum) ;
		~GTreeNode ();
		GTreeNode& operator = (const GTreeNode& src);
	};

	class IEDll_API GraphTree {
	public:
		GraphTree():m_RootNode(NULL) {};
		GraphTree(const LineTopologies& topLines,
			const std::map<int, bool>& mapNodeValid, int curNode);
		~GraphTree();
		GraphTree& operator = (const GraphTree& src);

		std::vector<std::vector<int> > GetLeastCliques();
	private:
		void ConsTreeNode(GTreeNode* pCurNode, 
			int curNodeNum);
		std::vector<int> SortClique(GTreeNode* pNode1, GTreeNode* pNode2);

	private:
		GTreeNode* m_RootNode;
		std::map<int, bool> m_mapNValid;//node validity
		LineTopologies m_TopLines;
	};

class IEDll_API AdjacentGraph
{
private:


public:
	typedef std::map<std::vector<int>, Position3D> MAPCLIPOS;

	AdjacentGraph(const LaserPoints& gLaserPnts, LineTopologies* localTopLines,
		const std::vector<PointNumberList>& vecLocalSegPnts);
	AdjacentGraph(const std::vector<int>& vecFaceNums, LineTopologies* localTopLines);
	~AdjacentGraph(void);

	AdjacentGraph& operator = (const AdjacentGraph& srcGraph);

	void DoSearchCliques();
	bool PruneInvalidCliques(const ObjectPoints& locObjPts);
	MAPCLIPOS ComputeCliCorner(const ObjectPoints& locObjPnts);
	std::vector< std::vector<int> > GetLeastCliques();
	bool WriteCliques(char* path);

private:
	AdjacentGraph();
	void Search1NodeClique();
	void Search2NodesClique();
	void Search3NodesClique();
	void SearchXNodesClique();
	void AddValidClique(std::vector<std::vector<int> > inCliques);

private:
	LineTopologies* m_pTopLines;
	std::map<int, bool> m_mapNodeValid;//first: node number; second: validity
	std::vector< std::vector<int> > m_vecCliques;
	std::vector<int> m_vecNodeNums;
	bool m_bSearched;
};

std::vector<int> GetValidNodes(const std::map<int, bool>& mapNodeValid,
	const std::vector<int>& vecInNodeNums);

#endif
