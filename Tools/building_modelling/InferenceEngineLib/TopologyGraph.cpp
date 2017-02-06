#include "TopologyGraph.h"
#include <vector>
#include "Vector3D.h"
#include "Building.h"
#include "InferenceEngine.h"
#include "LineTopologies.h"
#include "LineTopology.h"

#include <map>
#include <stack>
#include <queue>

using namespace std;
//using namespace TopologyGraph;

TopologyGraph::TopologyGraph(Building& bld)
{
	if (bld.MapDataPtr()!= NULL)
		//m_RidgeLines = *bld.MapDataPtr();
		*this = TopologyGraph(*(bld.MapDataPtr()));
}

TopologyGraph::TopologyGraph(LineTopologies& topLines)
{
	m_RidgeLines = topLines;
	LineTopologies::iterator line;
	vector<int> vecRoofNum;
	int leftNum, rightNum;

	for (line=m_RidgeLines.begin(); line!=m_RidgeLines.end(); ++line) {
		leftNum = line->Attribute(SegmentLabel);
		rightNum = line->Attribute(SecondSegmentLabel);
		if (leftNum!=-1)
			m_mapRoofFlag[leftNum] = 1;//valid, un-processed
		if (rightNum != -1)
			m_mapRoofFlag[rightNum] = 1;
	}
}

TopologyGraph TopologyGraph::operator = (const TopologyGraph& rGraph)
{
	this->m_RidgeLines = rGraph.m_RidgeLines;
	this->m_mapRoofFlag = rGraph.m_mapRoofFlag;
	this->m_vecPrimBlds.clear();
	
	map<int, char>::iterator mapItr;
	for (mapItr = m_mapRoofFlag.begin(); mapItr!= m_mapRoofFlag.end(); ++mapItr) {
		mapItr->second = 1;
	}

	return *this;
}

vector<int> TopologyGraph::SearchValidAdjRoofs(int curRoofNum)
{
	vector<int> vecValidRoofs;
	vector<int> vecAdjRoofs;
	int temRoofNum;
	SearchAdjacentFaces(m_RidgeLines, curRoofNum, vecAdjRoofs);

	for (int i=0; i<vecAdjRoofs.size(); ++i) {
		temRoofNum = vecAdjRoofs[i];
		if(m_mapRoofFlag[temRoofNum] == 1)
			vecValidRoofs.push_back(temRoofNum);
	}

	return vecValidRoofs;
}

void TopologyGraph::SearchOneFacePrimBlds()
{
	map<int, char>::iterator mapItr;
	vector<int> curPrimBld;
	int curRoofNum, roofNum1;
	vector<int> validAdjRoofs;

	for (mapItr = m_mapRoofFlag.begin(); mapItr!= m_mapRoofFlag.end(); ++mapItr) {
		if (mapItr->second != 1)//invalid, already processed
			continue;

		curRoofNum = mapItr->first;
		validAdjRoofs = SearchValidAdjRoofs(curRoofNum);

		//single node 
		if (!validAdjRoofs.empty()) continue;
		curPrimBld.clear();
		curPrimBld.push_back(curRoofNum);
		m_vecPrimBlds.push_back(curPrimBld);
		m_mapRoofFlag[curRoofNum] = 0;//already processed, invalid
	}
}

void TopologyGraph::SearchTwoFacePrimBlds()
{
	map<int, char>::iterator mapItr;
	vector<int> curPrimBld;
	int curRoofNum, roofNum1;
	vector<int> validAdjRoofs;

	for (mapItr = m_mapRoofFlag.begin(); mapItr!= m_mapRoofFlag.end(); ++mapItr) {
		if (mapItr->second != 1)//invalid, already processed
			continue;

		curRoofNum = mapItr->first;
		validAdjRoofs = SearchValidAdjRoofs(curRoofNum);

		//iterative search adjacent two roof primitive building
		while (validAdjRoofs.size() == 1) {
			curPrimBld.clear();
			roofNum1 = validAdjRoofs[0];
			curPrimBld.push_back(curRoofNum);
			curPrimBld.push_back(roofNum1);
			m_vecPrimBlds.push_back(curPrimBld);
			m_mapRoofFlag[curRoofNum] = 0;//already processed, set to be invalid

			curRoofNum = roofNum1;
			validAdjRoofs = SearchValidAdjRoofs(curRoofNum);
			if (validAdjRoofs.empty())
				m_mapRoofFlag[curRoofNum] = 0;//the last node of two-roof prim bld
		} 
	}
}

//this method try to find three faces prim-bld, 
//only the bld with one node who has two adj node is searched
void TopologyGraph::SearchThreeFacePrimBlds()
{
	map<int, char>::iterator mapItr;
	vector<int> curPrimBld;
	int curRoofNum, roofNum1, roofNum2;
	vector<int> validAdjRoofs, validAdjRoofs1;
	LineTopologies::iterator curLineTop;
	int indLineTop;
	stack<int> roofStack;

	for (mapItr = m_mapRoofFlag.begin(); mapItr!= m_mapRoofFlag.end(); ++mapItr) {
		if (mapItr->second != 1)//invalid, already processed
			continue;

		//iterative search adjacent two roof primitive building
		curRoofNum = mapItr->first;
		roofStack.push(curRoofNum);
		while(!roofStack.empty()) {
			curRoofNum = roofStack.top();
			roofStack.pop();

			if (m_mapRoofFlag[curRoofNum] != 1)//invalid, already processed
				continue;
			validAdjRoofs = SearchValidAdjRoofs(curRoofNum);
			if (validAdjRoofs.size() !=2 ) continue;
			roofNum1 = validAdjRoofs[0];
			roofNum2 = validAdjRoofs[1];
			if (!SearchLineBy2Faces(m_RidgeLines, roofNum1, roofNum2, indLineTop))
				continue;
				
			curLineTop = m_RidgeLines.begin()+indLineTop;
				
			curPrimBld.clear();
			curPrimBld.push_back(curRoofNum);
			curPrimBld.push_back(roofNum1);
			curPrimBld.push_back(roofNum2);
			m_vecPrimBlds.push_back(curPrimBld);
			m_mapRoofFlag[curRoofNum] = 0;//already processed, set to be invalid

			roofStack.push(roofNum1);
			roofStack.push(roofNum2);
			validAdjRoofs = SearchValidAdjRoofs(roofNum1);
			validAdjRoofs1 = SearchValidAdjRoofs(roofNum2);
			//the last two roofs for the tree-roof prim bld
			if (validAdjRoofs.size() == 1)	{
				m_mapRoofFlag[roofNum1] = 0;
				m_mapRoofFlag[roofNum2] = 0;
			}
		}
	}//end main loop
}

//Search primitive buildings with more then 3 roofs.
//building a tree, root at the start roof
//child node inherit from his father's exploring field
//it means, one node does not need to search a node if his predecessors have reached
void TopologyGraph::SearchXFacePrimBlds()
{
	map<int, char>::iterator mapItr;
	//vector<int> curPrimBld;
	int curRoofNum, roofNum1, roofNum2;
	vector<int> validAdjRoofs, validAdjRoofs1;
	LineTopology curLineTop;
	queue<TreeNode*> treeNodeQueue;
	stack<int> adjRoofStack;
	RoofTree tree;

	//record the reached nodes for one node
	multimap<TreeNode*, TreeNode*> mapReachNodes;// map<TreeNode*, TreeNode*>
	//record the reached node for current node, and its predecessors
	vector<TreeNode*> vecReachNodes;
	TreeNode* curNode, *temNode;
	//setVistNodes = mapVisitNodes.equal_range(itrVistNodes.first);

	for (mapItr = m_mapRoofFlag.begin(); mapItr!= m_mapRoofFlag.end(); ++mapItr) {
		//invalid, already processed
		if (mapItr->second != 1) continue;
		curRoofNum = mapItr->first;
		tree = RoofTree(curRoofNum, this);
		mapReachNodes.clear();

		//use tree to search cycles
		for (int i=0; i<tree.m_RootNode->children.size(); ++i) {
			curNode = (tree.m_RootNode->children)[i];
			treeNodeQueue.push(curNode);
			while (!treeNodeQueue.empty()) {
				curNode = treeNodeQueue.front();
				treeNodeQueue.pop();

				for (int j=i+1; j<tree.m_RootNode->children.size(); ++j) {
					temNode = (tree.m_RootNode->children)[j];
					ExploreTreeNode(curNode, temNode, mapReachNodes, m_vecPrimBlds);
				}
				//goto children nodes
				for (int iChild=0; iChild<curNode->children.size(); ++iChild) 
					treeNodeQueue.push((curNode->children)[iChild]);				
			}
		}

		m_mapRoofFlag[curRoofNum] = 0;

		///////////////////////////////////////////////////////////////
		//clear the left part of the cycle
		bool bHasRoof = false;//contain current roof
		//search prim buildings contain current roof
		for (int iPrim=0; iPrim<m_vecPrimBlds.size(); ++iPrim) {
			for (int iRoof=0; iRoof<m_vecPrimBlds[iPrim].size();++iRoof) {
				if (m_vecPrimBlds[iPrim][iRoof] == curRoofNum) {
					bHasRoof = true;
					break;
				}
			}
			if (!bHasRoof) continue;
			for (int iRoof=0; iRoof<m_vecPrimBlds[iPrim].size();++iRoof) {
				roofNum1 = m_vecPrimBlds[iPrim][iRoof];
				if (roofNum1 != curRoofNum
					&& m_mapRoofFlag[roofNum1] == 1) 
					adjRoofStack.push(roofNum1);
			}
		}

		//clear
		while (!adjRoofStack.empty()) {
			roofNum1 = adjRoofStack.top();
			adjRoofStack.pop();
			if (m_mapRoofFlag[roofNum1] != 1)
				continue;
			validAdjRoofs = SearchValidAdjRoofs(roofNum1);
			if (validAdjRoofs.size() == 1) {
				m_mapRoofFlag[roofNum1] = 0;
				adjRoofStack.push(validAdjRoofs[0]);
			}
		}

		//when one cycle found, some three face prim-bld can be searched and 
		//removed
		SearchThreeFacePrimBlds();
	}
}

//if visitor is adjacent to host, return true
bool TopologyGraph::ExploreTreeNode(TreeNode* visitor, TreeNode* host,
	MapReachNodes& mapReachNodes, VectorPrimBlds& vecPrimBlds)
{
	if (!visitor || !host) return false;

	//it or its father already reached this host
	if (HasReached(visitor, host, mapReachNodes))
		return false;

	LineTopologies::iterator lineTop;
	int indLineTop;
	vector<int> primBld;
	vector<int> leftPath, rightPath;
	TreeNode* temNode;
	//the two roofs are adjacent, form a primitive building
	if (SearchLineBy2Faces(m_RidgeLines, visitor->data, host->data, indLineTop)) {
		lineTop = m_RidgeLines.begin() + indLineTop;
		mapReachNodes.insert(PairReachNode(visitor, host));

		temNode = visitor;
		while(temNode) {
			leftPath.push_back(temNode->data);
			temNode = temNode->father;
		}

		temNode = host;
		while(temNode) {
			rightPath.push_back(temNode->data);
			temNode = temNode->father;
		}
		
		primBld.insert(primBld.begin(), leftPath.begin(), leftPath.end());
		for (int i=rightPath.size()-2; i>=0; --i) 
			primBld.push_back(rightPath[i]);
		m_vecPrimBlds.push_back(primBld);
		
		return true;
	}

	//explore the children of host
	TreeNode* childNode;
	bool isAdj = false;
	for (int i=0; i<host->children.size(); ++i) {
		childNode = host->children[i];
		if(ExploreTreeNode(visitor, childNode, mapReachNodes, vecPrimBlds))
			isAdj = true;
	}

	return isAdj;
}

bool TopologyGraph::HasReached(TreeNode* visitor, TreeNode* host, MapReachNodes& mapReachNodes)
{
	if (!visitor || !host) return false;

	SetReachNodes setRNodes = mapReachNodes.equal_range(visitor);
	ItrReachNodes itrRNodes;
	for (itrRNodes=setRNodes.first; itrRNodes!=setRNodes.second; ++itrRNodes) {
		if (itrRNodes->second == host) 
			return true;
	}

	//no father, so the visitor cannot inherits from its father 
	if (!visitor->father) return false;

	visitor = visitor->father;
	return HasReached(visitor, host, mapReachNodes);
}

void TopologyGraph::SearchPrimBlds()
{
	//the order is very important
	SearchOneFacePrimBlds();
	SearchTwoFacePrimBlds();
	SearchThreeFacePrimBlds();
	SearchXFacePrimBlds();
}

TopologyGraph::RoofTree::RoofTree()
	:m_graph(NULL), m_RootNode(NULL)
{

}

TopologyGraph::RoofTree::RoofTree(int RootRoofNum, TopologyGraph* graph)
	:m_graph(NULL), m_RootNode(NULL)
{
	if (!graph) return;

	TreeNode* pCurNode = new TreeNode;
	if (!pCurNode) return;

	m_graph = graph;
	m_RootNode = pCurNode;
	m_RootNode->data = RootRoofNum;
	map<int, char> mapRoofValid = graph->m_mapRoofFlag;
	m_RootNode->father = NULL;
	//CompleteNode(m_RootNode, mapRoofSearched);

	queue<TreeNode*> nodeQueue;
	nodeQueue.push(pCurNode);
	mapRoofValid[RootRoofNum] = 0;//set to be invalid
	int curRoof;
	vector<int> vecAdjFaceNums;
	TreeNode* pChildNode;

	while(!nodeQueue.empty()) {
		pCurNode = nodeQueue.front();
		nodeQueue.pop();
		if (!pCurNode) continue;
		curRoof = pCurNode->data;
		SearchAdjacentFaces(m_graph->m_RidgeLines, curRoof, vecAdjFaceNums);

		for (int iFace=0; iFace <vecAdjFaceNums.size(); ++iFace) {
			if (mapRoofValid[vecAdjFaceNums[iFace]] != 1)//invalid
				continue;

			pChildNode = new TreeNode;
			pChildNode->data = vecAdjFaceNums[iFace];
			pChildNode->father = pCurNode;
			(*pCurNode).children.push_back(pChildNode);
			mapRoofValid[vecAdjFaceNums[iFace]] = 0;//set to be invalid
			nodeQueue.push(pChildNode);
		}
	}
}

TopologyGraph::RoofTree::~RoofTree()
{
	if (!m_RootNode) return;
	ReleaseNode(m_RootNode);
}

TopologyGraph::RoofTree& TopologyGraph::RoofTree::operator = (const RoofTree& src)
{
	if (this == &src) return *this;

	if (this->m_RootNode) {
		ReleaseNode(this->m_RootNode);
		this->m_RootNode = NULL;
	}

	if (src.m_RootNode)	{
		this->m_RootNode = new TreeNode;
		this->m_RootNode->father = NULL;
		CopyNode(this->m_RootNode, src.m_RootNode);
	}
	this->m_graph = src.m_graph;

	return *this;
}

//depth search child node
//first search all children for current node,
//then iterative search children's children
//First come, first serve
void TopologyGraph::RoofTree::CompleteNode(TreeNode* pCurNode, map<int, char>& mapRoofSearched)
{
	if (!pCurNode) return ;

	int RootRoofNum = pCurNode->data;
	mapRoofSearched[RootRoofNum] = 0;
	vector<int> vecAdjFaceNums;
	TreeNode* childNode;
	SearchAdjacentFaces(m_graph->m_RidgeLines, RootRoofNum, vecAdjFaceNums);

	for (int iFace=0; iFace <vecAdjFaceNums.size(); ++iFace) {
		if (mapRoofSearched[vecAdjFaceNums[iFace]] != 1)//invalid
			continue;

		childNode = new TreeNode;
		childNode->data = vecAdjFaceNums[iFace];
		childNode->father = pCurNode;
		(*pCurNode).children.push_back(childNode);
		mapRoofSearched[vecAdjFaceNums[iFace]] = 0;//invalid
	}

	//iterative search child node
	for (int iChild=0; iChild<pCurNode->children.size(); ++iChild) {
		childNode = (pCurNode->children)[iChild];
		CompleteNode(childNode, mapRoofSearched);
	}
}

void TopologyGraph::RoofTree::ReleaseNode(TreeNode* pCurNode)
{
	if (!pCurNode) return;

	TreeNode* childNode;
	for (int iChild=0; iChild<pCurNode->children.size(); ++iChild) {
		childNode = (pCurNode->children)[iChild];
		if (!childNode) continue;
		ReleaseNode(childNode);
	}
	delete pCurNode;
	pCurNode = NULL;
}

//depth copy
void TopologyGraph::RoofTree::CopyNode(TreeNode* pDstNode, TreeNode* pSrcNode)
{
	if (!pSrcNode || !pDstNode) return;

	pDstNode->data = pSrcNode->data;
//	pDstNode->father = pSrcNode->father;

	TreeNode* pSrcChild;
	TreeNode* pDstChild;
	for (int iChild=0; iChild<pSrcNode->children.size(); ++iChild) {
		pSrcChild = (pSrcNode->children)[iChild];
		if (!pSrcChild) continue;
		pDstChild = new TreeNode;
		if (!pDstChild) {
			return;//not enough memory
		}
		pDstNode->children.push_back(pDstChild);
		pDstChild->father = pDstNode;
		CopyNode(pDstChild, pSrcChild);
	}
}

bool MatchPrimBld(const vector<int>& lBld, const vector<int>& rBld)
{
	vector<int> temLBld = lBld;
	vector<int> temRBld = rBld;
	std::sort(temLBld.begin(), temLBld.end());
	std::sort(temRBld.begin(), temRBld.end());

	return(temLBld==temRBld);
}

void TopologyGraph::ComparePrimBlds(const TopologyGraph& rGraph)
{
	VectorPrimBlds lBlds = this->m_vecPrimBlds;
	VectorPrimBlds rBlds = rGraph.m_vecPrimBlds;

	vector<char> vecValid = vector<char>(rBlds.size(), 1);

	for (int i=0; i<lBlds.size(); i++) {
		for (int j=0; j<rBlds.size(); j++) {
			if(MatchPrimBld(lBlds[i], rBlds[j]))
				break;
		}
	}
}
