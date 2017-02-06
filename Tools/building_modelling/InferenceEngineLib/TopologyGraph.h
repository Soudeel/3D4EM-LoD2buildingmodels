#ifndef _TOPOLOGY_GRAPH_H_
#define _TOPOLOGY_GRAPH_H_

#include <map>
#include <vector>

#include "InferenceEngine_DLL.h"
#include "LaserPoints.h"
#include "PointNumberList.h"
#include "Plane.h"

class LineTopologies;
class Building;
//Roof Topology Graph
//Main functions are construct graph, search primitive buildings, 
//and compare building topology graph
IEDll_API class TopologyGraph
{
	typedef std:: pair <int, int> RoofPair;
	struct TreeNode {
		int data;      //  DataType define int
		std::vector<TreeNode*> children;
		TreeNode* father;
	};
	typedef std::multimap<TreeNode*, TreeNode*> MapReachNodes;
	typedef std::pair<TreeNode*, TreeNode*> PairReachNode;
	typedef std::multimap<TreeNode*, TreeNode*>::iterator ItrReachNodes;
	typedef std::pair<ItrReachNodes, ItrReachNodes> SetReachNodes;
	typedef std::vector< std::vector<int> > VectorPrimBlds;

	class RoofTree {
	public:
		RoofTree();
		RoofTree(int RootRoofNum,TopologyGraph* graph);
		~RoofTree();
		RoofTree& operator = (const RoofTree& src);
	public:
		TreeNode* m_RootNode;
	private:
		void CompleteNode(TreeNode* pCurNode, std::map<int, char>& mapRoofSearched);
		void ReleaseNode(TreeNode* pCurNode);
		void CopyNode(TreeNode* pDstNode, TreeNode* pSrcNode);
	private:
		TopologyGraph* m_graph;
	};

public:
	TopologyGraph(void){};
	TopologyGraph(Building& bld);
	TopologyGraph(LineTopologies& topLines);
	~TopologyGraph(void) {};

	TopologyGraph operator = (const TopologyGraph& rGraph);
public:
	void SearchPrimBlds();
	VectorPrimBlds GetPrimBlds() {return m_vecPrimBlds;}
	void ComparePrimBlds(const TopologyGraph& rGraph);
private:
	std::vector<int> SearchValidAdjRoofs(int curRoofNum);
	void SearchOneFacePrimBlds();
	void SearchTwoFacePrimBlds();
	void SearchThreeFacePrimBlds();
	void SearchXFacePrimBlds();
	bool ExploreTreeNode(TreeNode* visitor, TreeNode* host,
		MapReachNodes& mapReachNodes, VectorPrimBlds& vecPrimBlds);
	bool HasReached(TreeNode* visitor, TreeNode* host, MapReachNodes& mapReachNodes);
private:
	LineTopologies m_RidgeLines;
	std::map<int, char> m_mapRoofFlag;//to record whether the roof is valid
	VectorPrimBlds m_vecPrimBlds;
	friend RoofTree;
};

#endif
