/*----------------------------------------------------------
|Creation:	Xiong Biao
|Time:		Sep 21, 2012
----------------------------------------------------------*/
#include <iostream>
#include <fstream>
#include "InlineArguments.h"
#include "LaserBlock.h"
#include <string>
#include "BNF_io.h"
#include "Buildings.h"
#include "InferenceEngine.h"
#include "AdjacentGraph.h"
#include "GraphEditDictionary.h"

#include <map>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

PrimSubGraph::tNode::tNode () 
{
	m_father = NULL;
	m_data = -1;
}

PrimSubGraph::tNode::tNode (tNode* father, int data) 
{
	m_father = father;
	m_data = data;
}

PrimSubGraph::tNode::~tNode () 
{
	for (int i=0; i<m_child.size(); ++i) {
		if (m_child[i]) {
			delete m_child[i];
			m_child[i] = NULL;
		}
	}
	m_father = NULL;
}

PrimSubGraph::tNode& 
	PrimSubGraph::tNode::operator = (const tNode& src)
{
	if (this == &src)
		return *this;

	if (m_father == NULL) //does not have father
		m_father = src.m_father;	
	m_data = src.m_data;

	//clear old children
	for (int i=0; i<m_child.size(); ++i) {
		if (m_child[i]) {
			delete m_child[i];
			m_child[i] = NULL;
		}
	}
	m_child.clear();

	tNode* curNode, *curSrcNode;
	for (int i=0; i<src.m_child.size(); ++i) {
		curSrcNode = src.m_child[i];
		curNode = new tNode(this, curSrcNode->m_data);
		*curNode = *curSrcNode;//copy
		m_child.push_back(curNode);
	}

	return *this;
}

vector<int> PrimSubGraph::tNode::GetBranchData()
{
	vector<int> brachData;

	tNode* pNode = this;
	do {
		if (pNode->m_data != -1) 
			brachData.push_back(pNode->m_data);
		pNode = pNode->m_father;
	} while (pNode);
	std::reverse(brachData.begin(), brachData.end());
	//brachData.reserve()

	return brachData;
}

//depth first
void PrimSubGraph::ConstructSearchTree()
{
	m_SearchTree.clear();

	tNode* treeRoot = new tNode(NULL, -1);
	tNode* temNode, *curNode;
	stack<tNode*> staNodes;
	int curData, temData;
	vector<int> vecAdjNodes, vecBrachData;
	vector<vector<int> > vecPaths;
	vector<int> temPath, curPath;

	for (int i=m_NodeNums.size()-1; i>=0; --i) {
		temNode = new tNode(treeRoot, m_NodeNums[i]);
		treeRoot->m_child.push_back(temNode);
		staNodes.push(temNode);
	}

	//
	vector<int>::iterator itr;
	bool bHasAdj;
	//////////////////////////////////////////////////////////////////////////
	//search all path in the graph
	while (!staNodes.empty()) {
		curNode = staNodes.top();
		staNodes.pop();

		curData = curNode->m_data;
		vecAdjNodes = this->SearchAdjNodes(curData);
		vecBrachData = curNode->GetBranchData();
		temPath = vecBrachData;
		std::sort(vecBrachData.begin(), vecBrachData.end());

		//add children nodes
		bHasAdj = false;
		for (int i=vecAdjNodes.size()-1; i>=0; --i) {
			itr = std::find(vecBrachData.begin(), 
				vecBrachData.end(), vecAdjNodes[i]);
			//add new nodes into the searching path
			if (itr==vecBrachData.end()) {
				temNode = new tNode(curNode, vecAdjNodes[i]);
				curNode->m_child.push_back(temNode);
				staNodes.push(temNode);
				bHasAdj = true;
			}
		}

		if (!bHasAdj) m_SearchTree.push_back(temPath);
	}

	delete treeRoot;
	//return vecPaths;
}

/*
//wide first
void PrimSubGraph::ConstructSearchTree()
{
m_SearchTree.clear();

tNode* treeRoot = new tNode(NULL, -1);
tNode* temNode, *curNode;
queue<tNode*> queNodes;
int curData, temData;
vector<int> vecAdjNodes, vecBrachData;
vector<vector<int> > vecPaths;
vector<int> temPath, curPath;

for (int i=0; i<m_NodeNums.size(); ++i) {
temNode = new tNode(treeRoot, m_NodeNums[i]);
treeRoot->m_child.push_back(temNode);
queNodes.push(temNode);
}

//
vector<int>::iterator itr;
bool bHasAdj;
//////////////////////////////////////////////////////////////////////////
//search all path in the graph
while (!queNodes.empty()) {
curNode = queNodes.front();
queNodes.pop();

curData = curNode->m_data;
vecAdjNodes = this->SearchAdjNodes(curData);
vecBrachData = curNode->GetBranchData();
temPath = vecBrachData;
std::sort(vecBrachData.begin(), vecBrachData.end());

//add children nodes
bHasAdj = false;
for (int i=0; i<vecAdjNodes.size(); ++i) {
itr = std::find(vecBrachData.begin(), 
vecBrachData.end(), vecAdjNodes[i]);
//add new nodes into the searching path
if (itr==vecBrachData.end()) {
temNode = new tNode(curNode, vecAdjNodes[i]);
curNode->m_child.push_back(temNode);
queNodes.push(temNode);
bHasAdj = true;
}
}

if (!bHasAdj) m_SearchTree.push_back(temPath);
}

//////////////////////////////////////////////////////////////////////////
//remove redundant elements
//	bool bSame = false;
//	for (int i=0; i<m_SearchTree.size(); ++i) {
//		curPath = m_SearchTree[i];
//		bSame = false;
//		for (int j=i+1; j<m_SearchTree.size(); j++)	{
//			temPath = m_SearchTree[j];
//			if (curPath == temPath) 
//				bSame = true;

//			if (!bSame) {
//				std::reverse(temPath.begin(), temPath.end());
//				if (curPath == temPath)
//					bSame = true;
//			}

//			if (bSame) {
//				m_SearchTree.erase(m_SearchTree.begin() + j);
//				break;
//			}
//		}
//	}

delete treeRoot;
//return vecPaths;
}
*/

void PrimSubGraph::ConstructSearchPath()
{
	this->ConstructSearchTree();

	int nNodeCount = m_NodeNums.size();
	vector<int> temPath;
	//just pick one path that has all nodes
	for (int i=0; i<m_SearchTree.size(); i++) {
		if (m_SearchTree[i].size()==nNodeCount)	{
			temPath = m_SearchTree[i];
			break;
		}
	}

	m_SearchTree.clear();
	m_SearchTree.push_back(temPath);
}

float PrimSubGraph::GetNodeScore(int nodeNum)
{
	map<int, float>::iterator itr;
	itr = m_NodeScores.find(nodeNum);
	return itr!=m_NodeScores.end()?itr->second:0.0;
}

float PrimSubGraph::GetEdgeScore(edge e)
{
	map<edge, float>::iterator itr;
	itr = m_EdgeScores.find(e);
	return itr!=m_EdgeScores.end()?itr->second:0.0;
}

PrimSubGraph::edge::edge(int inNode1, int inNode2)
{
	assert(inNode1 != inNode2);
	if (inNode1 > inNode2)
		std::swap(inNode1, inNode2);
	node1 = inNode1;
	node2 = inNode2;
}

/*
std::ostream& operator<<(std::ostream& stream, PrimSubGraph::edge& e)
{
stream<<e.node1;
stream<<e.node1;
return stream;
}

std::istream& operator>>(std::istream& stream, PrimSubGraph::edge& e)
{
int node1, node2;
stream>>node1;
stream>>node2;
e = PrimSubGraph::edge(node1, node2);
return stream;
}
*/
bool operator < (const PrimSubGraph::edge &a, const PrimSubGraph::edge &b)
{
	if (a == b) return false;

	if (a.node1 < b.node1) 
		return true;
	else if (a.node1 > b.node1)
		return false;
	else {//node1 == inEdge.Node1
		if (a.node2 < b.node2)
			return true;
		else 
			return false;
	}
}

bool operator == (const PrimSubGraph::edge &a, const PrimSubGraph::edge &b)
{
	return (a.node1==b.node1 && a.node2==b.node2);
}

std::vector<int> PrimSubGraph::SearchAdjNodes(int curNode)
{
	edge temEdge;
	int temNode;
	map<edge, float>::iterator itrEdge;
	vector<int> vecAdjNodes;

	for (int i=0; i<m_NodeNums.size(); ++i) {
		temNode = m_NodeNums[i];
		if (temNode == curNode) continue;

		temEdge = edge(temNode,curNode);
		itrEdge = m_EdgeScores.find(temEdge);
		if (itrEdge != m_EdgeScores.end())
			vecAdjNodes.push_back(temNode);
	}

	std::sort(vecAdjNodes.begin(), vecAdjNodes.end());

	return vecAdjNodes;
}

//distance when maching the two nodes
float PrimSubGraph::NodeDistance(int curNode, PrimSubGraph& refGraph, int refNode)
{
	vector<int> curAdjNodes = this->SearchAdjNodes(curNode);
	vector<int> refAdjNodes = refGraph.SearchAdjNodes(refNode);

	vector<float> curScores, refScores;
	edge temEdge;
	float temScore;
	for (int i=0; i<curAdjNodes.size(); i++) {
		temEdge = edge(curNode, curAdjNodes[i]);
		temScore = m_EdgeScores[temEdge];
		temScore += m_NodeScores[curAdjNodes[i]];
		curScores.push_back(temScore);
	}

	for (int i=0; i<refAdjNodes.size(); i++) {
		temEdge = edge(refNode, refAdjNodes[i]);
		temScore = (refGraph.m_EdgeScores)[temEdge];
		temScore += (refGraph.m_NodeScores)[refAdjNodes[i]];
		refScores.push_back(temScore);
	}

	if (curScores.size() < refScores.size()) 
		curScores.insert(curScores.end(),refScores.size()-curScores.size(),0.0);
	else
		refScores.insert(refScores.end(),curScores.size()-refScores.size(),0.0);

	//construct suduku matrix to match nodes
	int wid = curScores.size();
	vector<float> suduku = vector<float>(wid*wid, 0.0);
	vector<char> valid = vector<char>(suduku.size(), true);
	for (int i=0; i<curScores.size(); ++i) 
		for (int j=0; j<refScores.size(); ++j) 
			suduku[i*wid+j] = fabs(curScores[i]-refScores[j]);

	//match nodes between current graph and reference graph
	float matchScore = 0.0;
	int pos, arr, col;
	for (int i=0; i<wid; i++) {
		matchScore += MinElement(valid, suduku, pos);
		arr = pos/wid;
		col = pos%wid;

		//invalid its column and row elements
		for (int j=0; j<wid; ++j) {
			valid[arr*wid+j] = false;
			valid[j*wid+col] = false;
		}
	}

	matchScore += fabs(this->m_NodeScores[curNode]-refGraph.m_NodeScores[refNode]);

	return matchScore;
}

//find minimum element in a suduku matrix
float PrimSubGraph::MinElement(const std::vector<char>& valid, 
	const std::vector<float>& suduku, int& pos)
{
	float min = 1000.0;
	for (int i=0; i<suduku.size(); i++)	{
		if (!valid[i]) continue;

		if (suduku[i] < min) {
			min = suduku[i];
			pos = i;
		}
	}

	return min;
}

//20120715
//use matching tree to match Isomorphism
float PrimSubGraph::EditDistance(PrimSubGraph& refGraph, vector<int>& vecMatchNodes)
{
	assert(this->m_SearchTree.size() == 1);

	vector<vector<int> >::iterator itrSrcPath, itrRefPath;
	itrSrcPath = this->m_SearchTree.begin();

	float minDist, temDist;
	minDist = 100000.0;
	int node11, node12, node21, node22;
	float score1, score2;
	map<edge, float>::iterator itrEdge;
	vector<float> vecDist;
	int bestInd;
	int nCount;

	for (itrRefPath=refGraph.m_SearchTree.begin(); 
		itrRefPath!=refGraph.m_SearchTree.end(); 
		++itrRefPath)	{
			//only care about exact match
			//and only consider the begging part of the reference path
			if (itrSrcPath->size() > itrRefPath->size())
				continue;

			if(itrSrcPath->size()==4 && (*itrRefPath)[0]==2989 && 
				(*itrRefPath)[1]==2988 && (*itrRefPath)[2]==2994 && (*itrRefPath)[3]==2990)
			{
				int aaaa = 1;
			}

			//node score
			temDist = 0.0;
			for (int i=0; i<itrSrcPath->size(); ++i) {
				node11 = (*itrSrcPath)[i];
				node21 = (*itrRefPath)[i];
				score1 = this->m_NodeScores[node11];
				score2 = refGraph.m_NodeScores[node21];
				temDist += fabs(score1 - score2);
			}
			//temDist /=itrSrcPath->size();

			//edge score
			nCount = 0;
			double score=0;
			for (int i=0; i<itrSrcPath->size(); ++i) {
				node11 = (*itrSrcPath)[i];
				node21 = (*itrRefPath)[i];

				for (int j=i+1; j<itrSrcPath->size(); ++j)	{
					node12 = (*itrSrcPath)[j];
					node22 = (*itrRefPath)[j];

					score1 = score2 = 50.0;
					itrEdge = this->m_EdgeScores.find(edge(node11, node12));
					if (itrEdge != this->m_EdgeScores.end())
						score1 = itrEdge->second;

					itrEdge = refGraph.m_EdgeScores.find(edge(node21, node22));
					if (itrEdge != refGraph.m_EdgeScores.end())
						score2 = itrEdge->second;

					score += fabs(score1-score2);
					nCount++;
				}
			}

			//if (nCount==0)
			//	score = 0;
			//else
			//	score /= nCount;
			temDist += score;

			vecDist.push_back(temDist);
			if (temDist < minDist)	{
				minDist = temDist;
				bestInd = itrRefPath-refGraph.m_SearchTree.begin();
			}
	}

	//get match nodes
	//vector<int> vecMatchNodes;
	vecMatchNodes.clear();
	if (minDist != 100000.0) {
		for (int i=0; i<itrSrcPath->size(); ++i) {
			itrRefPath = refGraph.m_SearchTree.begin() + bestInd;
			vecMatchNodes.push_back((*itrSrcPath)[i]);
			vecMatchNodes.push_back((*itrRefPath)[i]);
		}
	}

	return minDist;
}

//current subgraph --> destination graph
void PrimSubGraph::Difference(const PrimSubGraph& destGraph, vector<int>& removedSeg,
	vector<int>& newSeg, vector<int>& removedEdge, vector<int>& newEdge)
{
	int segNum11, segNum12, segNum21, segNum22;
	vector<int>::const_iterator itrSeg;
	//m_NodeNums;
	//m_EdgeScores;
	for (int i=0; i<m_NodeNums.size(); ++i) {
		segNum11 = m_NodeNums[i];
		itrSeg = std::find(destGraph.m_NodeNums.begin(), 
			destGraph.m_NodeNums.end(), segNum11);
		if (itrSeg == destGraph.m_NodeNums.end())
			removedSeg.push_back(segNum11);
	}

	for (int i=0; i<destGraph.m_NodeNums.size(); ++i) {
		segNum21 = destGraph.m_NodeNums[i];
		itrSeg = std::find(m_NodeNums.begin(), m_NodeNums.end(), segNum21);
		if (itrSeg == m_NodeNums.end())
			newEdge.push_back(segNum11);
	}

	edge temEdge;
	map<edge, float>::const_iterator itrEdge1, itrEdge2;
	for (itrEdge1=m_EdgeScores.begin(); itrEdge1!=m_EdgeScores.end(); ++itrEdge1) {
		temEdge = itrEdge1->first;
		itrEdge2 = destGraph.m_EdgeScores.find(temEdge);
		if (itrEdge2 == destGraph.m_EdgeScores.end()) {
			removedEdge.push_back(temEdge.node1);
			removedEdge.push_back(temEdge.node2);
		}
	}

	for (itrEdge1=destGraph.m_EdgeScores.begin(); itrEdge1!=destGraph.m_EdgeScores.end(); ++itrEdge1) {
		temEdge = itrEdge1->first;
		itrEdge2 = m_EdgeScores.find(temEdge);
		if (itrEdge2 == m_EdgeScores.end()) {
			newEdge.push_back(temEdge.node1);
			newEdge.push_back(temEdge.node2);
		}
	}
}

//20120715
//use Suduku matrix to find optimize match
/*float PrimSubGraph::EditDistance(PrimSubGraph& refGraph)
{
int hei = this->m_NodeScores.size();
int wid = refGraph.m_NodeScores.size();
vector<float> suduku = vector<float>(hei*wid, 0.0);
vector<char> valid = vector<char>(suduku.size(), true);

//////////////////////////////////////////////////////////////////////////
//construct suduku matrix
int node1, node2;
float nodeDis;
for (int i=0; i<hei; ++i) {
node1 = this->m_NodeNums[i];
for (int j=0; j<wid; ++j) {
node2 = refGraph.m_NodeNums[j];
nodeDis = NodeDistance(node1, refGraph, node2);

suduku[i*wid+j] = nodeDis;
}
}

//////////////////////////////////////////////////////////////////////////
//search best match
float matchScore = 0.0;
int pos, col, arr;
vector<int> matchNodes;
for (int i=0; i < (hei<wid?hei:wid); i++) {
matchScore += MinElement(valid, suduku, pos);
arr = pos/wid;
col = pos%wid;
matchNodes.push_back((this->m_NodeNums)[arr]);
matchNodes.push_back((refGraph.m_NodeNums)[col]);

//invalid its column and row elements
for (int j=0; j<wid; ++j)
valid[arr*wid+j] = false;
for (int j=0; j<hei; ++j)
valid[j*wid+col] = false;
}

return matchScore;
}*/


void PrimSubGraph::Clear()
{
	m_NodeNums.clear();
	m_NodeScores.clear();
	m_EdgeScores.clear();
	m_SearchTree.clear();
}

void PrimSubGraph::AddNode(int nodeNum, float nodeScore)
{
	m_NodeScores[nodeNum] = nodeScore;

	bool bFind = false;
	for (int i=0; i<m_NodeNums.size(); i++) {
		if (m_NodeNums[i] == nodeNum) {
			bFind = true;
			break;
		}
	}

	if (!bFind) m_NodeNums.push_back(nodeNum);
}

void PrimSubGraph::AddEdge(int nodeNum1, int nodeNum2, float edgeScore)
{
	edge temEdge = edge(nodeNum1, nodeNum2);
	m_EdgeScores[temEdge] = edgeScore;
}

std::ostream& operator<< (ostream& stream, PrimSubGraph& graph)
{
	stream<<graph.m_NodeScores.size();
	map<int, float>::iterator nodeItr;
	for (nodeItr=graph.m_NodeScores.begin(); 
		nodeItr!=graph.m_NodeScores.end(); nodeItr++) {
			stream<<nodeItr->first;
			stream<<nodeItr->second;
	}

	stream<<graph.m_EdgeScores.size();
	map<PrimSubGraph::edge, float>::iterator edgeItr;
	for (edgeItr=graph.m_EdgeScores.begin(); 
		edgeItr!=graph.m_EdgeScores.end(); edgeItr++) {
			stream<<edgeItr->first.node1<<edgeItr->first.node2;
			stream<<edgeItr->second;
	}

	return stream;
}

std::istream& operator >> (istream& stream, PrimSubGraph& graph)
{
	int nodeNum, edgeNum;
	int node1, node2;
	float score;
	typedef pair <int, float> node_Pair;
	typedef pair <PrimSubGraph::edge, float> edge_Pair;
	PrimSubGraph::edge temedge;
	graph.m_EdgeScores.clear();
	graph.m_NodeScores.clear();
	graph.m_NodeNums.clear();

	stream>>nodeNum;
	for (int i=0; i<nodeNum; ++i) {
		stream>>node1;
		stream>>score;
		graph.m_NodeScores.insert(node_Pair(node1, score));
		graph.m_NodeNums.push_back(node1);
	}

	stream>>edgeNum;
	for (int i=0; i<edgeNum; ++i) {
		stream>>node1>>node2;
		stream>>score;
		temedge = PrimSubGraph::edge(node1, node2);
		graph.m_EdgeScores.insert(edge_Pair(temedge, score));
	}

	return stream;
}

void GraphDiction::Read(const char* filepath)
{
	if (!filepath) return;
	ifstream stream(filepath);
	if (!stream.bad()) {
		stream>>*this;
		stream.close();
	}
}

void GraphDiction::Write(const char* filepath)
{
	if (!filepath) return;
	ofstream stream(filepath);
	if (stream.bad()) {
		stream<<*this;
		stream.close();
	}
}

int GraphDiction::LookUp(PrimSubGraph& indGraph, vector<int>& vecMatchNodes, float threshold)
{
	int index = -1;
	float minDist = 100.00, temDist;
	//vector<vector<int> > indSTree = indGraph.ConstSearchTree();
	vector<int> temMatchNodes;

	for (int i=0; i<m_Entries.size(); i++) {
		temDist = m_Entries[i].EditDistance(indGraph, temMatchNodes);
#ifdef _DEBUG
		printf("%.3f \t", temDist);
		for (int i=0; i<temMatchNodes.size(); ++i)
			printf("%d ", temMatchNodes[i]);
		printf("\n");
#endif
		if (temDist<threshold && temDist<minDist) {
			minDist = temDist;
			vecMatchNodes = temMatchNodes;
			index = i;
		}
	}

#ifdef _DEBUG
	printf("\n");
#endif

	return index;
}

void GraphDiction::CorrectGraph(const vector<int>& vecMatchNodes, int index, 
	ObjectPoints& modelObjPnts, LineTopologies& lineTop)
{
	assert(index>=0 && index<this->m_Entries.size());
	PrimSubGraph entry = this->m_Entries[index];
	PrimSubGraph explain = this->m_Explains[index][0];
	vector<int> removedSeg, newSeg, removedEdge, newEdge;
	entry.Difference(explain, removedSeg, newSeg, removedEdge, newEdge);

	int mNode1, mNode2, dNode1, dNode2;
	int indLine;
	//remove lines
	for (int i=0; i<removedEdge.size()/2; ++i) {
		mNode1 = removedEdge[i*2];
		mNode2 = removedEdge[i*2+1];
		dNode1 = DataGraphNode(vecMatchNodes, mNode1);
		dNode2 = DataGraphNode(vecMatchNodes, mNode2);
		if(SearchLineBy2Faces(lineTop, dNode1, dNode2, indLine))
			lineTop.erase(lineTop.begin()+indLine);
	}

	//add lines
	LineTopology temLine;
	ObjectPoint temPnt;
	for (int i=0; i<newEdge.size()/2; ++i) {
		mNode1 = newEdge[i*2];
		mNode2 = newEdge[i*2+1];
		dNode1 = DataGraphNode(vecMatchNodes, mNode1);
		dNode2 = DataGraphNode(vecMatchNodes, mNode2);

		temLine.clear();
		temLine.SetAttribute(SegmentLabel, dNode1);
		temLine.SetAttribute(SecondSegmentLabel, dNode2);
		
		temPnt.Number() = modelObjPnts.size();
		modelObjPnts.push_back(temPnt);
		temLine.push_back(temPnt.Number());

		temPnt.Number() = modelObjPnts.size();
		modelObjPnts.push_back(temPnt);
		temLine.push_back(temPnt.Number());

		lineTop.push_back(temLine);
	}
//	entry.
}

int GraphDiction::DataGraphNode(const vector<int>& vecMatchNodes, int modelNode)
{
	for (int i=0; i<vecMatchNodes.size()/2; ++i) {
		if(vecMatchNodes[2*i]==modelNode)
			return vecMatchNodes[2*i+1];
	}
	return -1;
}

std::ostream& operator<<(std::ostream& stream, GraphDiction& dic)
{
	assert(dic.m_Entries.size() == dic.m_Explains.size());
	int entryCount = dic.m_Entries.size();
	int expCount;//explain
	stream<<entryCount;

	for (int i=0; i<entryCount; i++) {
		stream<<(dic.m_Entries)[i];

		expCount = (dic.m_Explains)[i].size();
		stream<<expCount;
		for (int j=0; j<expCount; ++j) {
			stream<<(dic.m_Explains)[i][j];
		}
	}
	return stream;
}

std::istream& operator>>(std::istream& stream, GraphDiction& dic)
{
	int entryCount;//entry
	int expCount;//explain
	PrimSubGraph entry, explain;
	vector<PrimSubGraph> vecExps;

	stream>>entryCount;
	for (int i=0; i<entryCount; i++) {
		stream>>entry;
		entry.ConstructSearchPath();
		dic.m_Entries.push_back(entry);

		stream>>expCount;
		vecExps.clear();
		for (int j=0; j<expCount; ++j) {
			stream>>explain;
			explain.ConstructSearchPath();
			vecExps.push_back(explain);
			dic.m_Explains.push_back(vecExps);
		}
	}

	return stream;
}

//old
/*
//learn post probability between objective model and 
void LearnModelRelation()
{
char* strPntPath = "E:\\data\\data5\\3-25-MIDDELLAND.objpts";
char* strTopPath = "E:\\data\\data5\\3-25-MIDDELLAND.top";
char* strLasPath = "E:\\data\\data5\\5.laser";

Buildings pcmBlds;
ObjectPoints model_points, bldObjPnts;
DataBounds3D temBounds;
vector<DataBounds3D> vecBldBounds;
std::vector <LineTopologies *> * ltopsRoofs;
vector <LineTopologies *>::iterator itrRoofTops;
LineTopologies::iterator itrTop;
LineTopology::iterator itrPntNum;
LaserPoints totLasPnts;
LaserPoints bldLasPnts;
vector<PointNumberList> vecBldPnl;
vector<PointNumberList> vecSegPnl;

pcmBlds.ImportPCMModelData(strPntPath, strTopPath, model_points);
totLasPnts.Read(strLasPath);
MyDeriveSegPNL(totLasPnts, vecBldPnl, PlaneNumberTag);
int BldNum, index;
PointNumberList curBldPnl;
PointNumberList curSegPnl;
SegmentationParameters segParameters;
segParameters.MaxDistanceSurface() = 0.2;

for (int iBld=0; iBld<pcmBlds.size(); ++iBld) {
BldNum = pcmBlds[iBld].Number();
index = IndexSegBySegNumber(totLasPnts, vecBldPnl, BldNum, PlaneNumberTag);
if(index<0) continue;
curBldPnl = vecBldPnl[index];
bldLasPnts.clear();
for (int iPnt=0; iPnt<curBldPnl.size(); ++iPnt)	
bldLasPnts.push_back(totLasPnts[curBldPnl[iPnt].Number()]);
bldLasPnts.SurfaceGrowing(segParameters);

MyDeriveSegPNL(bldLasPnts, vecSegPnl, SegmentNumberTag);
//get build roof number for each segment
//one build roof number can only be assigned to one segment
//the segment which has no roof number will be given -1 to represent it's situation


}
}*/

//search corresponding building in a different models
Buildings::iterator SearchBldByBld(Buildings::iterator dstBld, Buildings& srcBlds)
{
	vector<int> srcSegNums, dstSegNums;
	SearchSegNumsInBld(*dstBld, dstSegNums);
	Buildings::iterator srcBld;
	Buildings::iterator rstBld=srcBlds.end();
	bool bFind;
	int dstSegNum;

	for (srcBld=srcBlds.begin(); srcBld!=srcBlds.end(); ++srcBld) {
		SearchSegNumsInBld(*srcBld, srcSegNums);

		bFind = false;
		for (int iseg=0; iseg<dstSegNums.size(); ++iseg) {
			dstSegNum = dstSegNums[iseg];
			for (int jseg=0; jseg<srcSegNums.size(); ++jseg) {
				if (dstSegNum == srcSegNums[jseg]) {
					rstBld = srcBld;
					goto endflag;
				}
			}
		}
	}

endflag:
	return rstBld;
}

//compare whether two primitives are adjacent or not
bool IsAdjacent(const vector<int>& prim1, const vector<int>& prim2)
{
	bool isAdj = false;
	vector<int>::const_iterator itr;
	int segNum;
	for (int i=0; i<prim2.size(); ++i) {
		segNum = prim2[i];
		itr = std::find(prim1.begin(), prim1.end(), segNum);
		if (itr != prim1.end())	{
			isAdj = true;
			break;
		}
	}
	return isAdj;
}

//////////////////////////////////////////////////////////////////////////
//search candidate primitives in original and improved graphs
//search new or deleted edges, then find their corresponding subgraphs
//if two subgraphs are connected, they should combine to be a bigger one
vector<vector<int> > SearchCandSubGraphs(vector<int>& vecFaceNums, LineTopologies& inLineTops, 
	vector<char>& vecLineRepeat)
{
	vector<vector<int> > rstPrims;
	vector<int> temPrim;
	if (vecLineRepeat.size() != inLineTops.size()) return rstPrims;

	//search all primitives
	AdjacentGraph adjGraph = AdjacentGraph(vecFaceNums, &inLineTops);
	adjGraph.DoSearchCliques();
	vector<vector<int> > vecCliques = adjGraph.GetLeastCliques();

	//find candidate primitives
	vector<int>::iterator itr1, itr2;
	int segNum1, segNum2;
	for (int iLine=0; iLine<vecLineRepeat.size(); ++iLine) {
		if (vecLineRepeat[iLine]) continue;//skip repeat line
		if (!inLineTops[iLine].HasAttribute(SegmentLabel)
			||!inLineTops[iLine].HasAttribute(SecondSegmentLabel))
			continue;	
		segNum1 = inLineTops[iLine].Attribute(SegmentLabel);
		segNum2 = inLineTops[iLine].Attribute(SecondSegmentLabel);
		if (segNum1>segNum2) std::swap(segNum1, segNum2);
		if (segNum1==-1 || segNum1==segNum2) continue;

		for (int i=0; i<vecCliques.size(); ++i) {
			temPrim = vecCliques[i];
			itr1 = std::find(temPrim.begin(), temPrim.end(), segNum1);
			itr2 = std::find(temPrim.begin(), temPrim.end(), segNum2);
			if (itr1!=temPrim.end() && itr2!=temPrim.end())	{
				rstPrims.push_back(temPrim);
			}
		}
	}

	//find subgraphs
	//if two primitives are connected, combine them to be one subgraph
	for (int i=0; i<rstPrims.size(); i++) {
		temPrim = rstPrims[i];
		for (int j=i+1; j<rstPrims.size(); j++) {
			if (IsAdjacent(temPrim, rstPrims[j])) {
				temPrim.insert(temPrim.end(), rstPrims[j].begin(), rstPrims[j].end());
				rstPrims.erase(rstPrims.begin()+j);
				j--;
			}
		}

		std::sort(temPrim.begin(), temPrim.end());
		itr1 = std::unique(temPrim.begin(), temPrim.end());
		temPrim = vector<int>(temPrim.begin(), itr1);
		rstPrims[i] = temPrim;
	}

	return rstPrims;
}

//find matching subgraphs in original and improved graphs
vector<vector<int> > MatchSubGraphs(const vector<int>& orgSegNums, const vector<int>& impSegNums, 
	const vector<vector<int> >& orgSubGraphs, const vector<vector<int> >& impSubGraphs)
{
	vector<vector<int> > matchGraphs;
	vector<int> temGraph1, temGraph2;
	vector<int>::const_iterator itr;

	//find corresponding nodes in improved graph 
	//for each subgraph of original graph
	for (int i=0; i<orgSubGraphs.size(); ++i) {
		temGraph1 = orgSubGraphs[i];

		temGraph2.clear();
		for (int j=0; j<temGraph1.size(); ++j) {
			itr = std::find(impSegNums.begin(), impSegNums.end(), temGraph1[j]);
			if (itr!=impSegNums.end())
				temGraph2.push_back(temGraph1[j]);
		}

		matchGraphs.push_back(temGraph1);
		matchGraphs.push_back(temGraph2);
	}

	//find corresponding nodes in improved graph 
	//for each subgraph of original graph
	for (int i=0; i<impSubGraphs.size(); ++i) {
		temGraph1 = impSubGraphs[i];

		temGraph2.clear();
		for (int j=0; j<temGraph1.size(); ++j) {
			itr = std::find(orgSegNums.begin(), orgSegNums.end(), temGraph1[j]);
			if (itr!=orgSegNums.end())
				temGraph2.push_back(temGraph1[j]);
		}

		matchGraphs.push_back(temGraph2);
		matchGraphs.push_back(temGraph1);
	}

	//////////////////////////////////////////////////////////////////////////
	//combine adjacent subgraphs
	vector<int> temGraph3, temGraph4;
	vector<int>::iterator temItr;
	bool bFind;
	for (int i=0; i<matchGraphs.size()/2; ++i)	{
		temGraph1 = matchGraphs[2*i];
		temGraph2 = matchGraphs[2*i+1];

		for (int j=i+1; j<matchGraphs.size()/2; ++j) {
			temGraph3 = matchGraphs[2*j];
			temGraph4 = matchGraphs[2*j+1];

			bFind = false;
			for (int k=0; k<temGraph3.size(); k++) {
				temItr = std::find(temGraph1.begin(), temGraph1.end(), temGraph3[k]);
				if (temItr != temGraph1.end()) {
					bFind = true;
					break;
				}
			}
			for (int k=0; k<temGraph4.size(); k++) {
				temItr = std::find(temGraph2.begin(), temGraph2.end(), temGraph4[k]);
				if (temItr != temGraph2.end()) {
					bFind = true;
					break;
				}
			}

			if (bFind) {
				temGraph1.insert(temGraph1.end(), temGraph3.begin(), temGraph3.end());
				std::sort(temGraph1.begin(), temGraph1.end());
				temItr = std::unique(temGraph1.begin(), temGraph1.end());
				temGraph1 = vector<int>(temGraph1.begin(), temItr);
				matchGraphs[2*i] = temGraph1;

				temGraph2.insert(temGraph2.end(), temGraph4.begin(), temGraph4.end());
				std::sort(temGraph2.begin(), temGraph2.end());
				temItr = std::unique(temGraph2.begin(), temGraph2.end());
				temGraph2 = vector<int>(temGraph2.begin(), temItr);
				matchGraphs[2*i+1] = temGraph2;

				matchGraphs.erase(matchGraphs.begin()+2*j);
				matchGraphs.erase(matchGraphs.begin()+2*j);
				j--;
			}
		}
	}

	return matchGraphs;
}

void CompareImprovement()
{
	LaserPoints orgCloud, impCloud;
	Buildings orgBlds, impBlds;
	ObjectPoints orgObjPnts, impObjPnts;
	LineTopologies orgLines, impLines;
	vector<int> orgSegNums, impSegNums;
	int lindInd;
	vector<vector<int> > orgSubGraphs, impSubGraphs;
	//	char* strOrgCloud = "F:\\data1\\hogeland_nord.laser";
	//	char* strOrgMapPnt = "F:\\data1\\hogeland_nord.objpts";
	//	char* strOrgMapTop = "F:\\data1\\hogeland_nord.top";
	//	char* strImpCloud = "F:\\data1\\hogeland_nord_after.laser";;
	//	char* strImpMapPnt = "F:\\data1\\hogeland_nord_after.objpts";
	//	char* strImpMapTop = "F:\\data1\\hogeland_nord_after.top";
	//	char* strCompareRst = "F:\\data1\\hogeland_nord_rst.txt";

	//	char* strOrgCloud = "F:\\data1\\00getfert_2_bld.laser";
	//	char* strOrgMapPnt = "F:\\data1\\00getfert_2_bld.objpts";
	//	char* strOrgMapTop = "F:\\data1\\00getfert_2_bld.top";
	//	char* strImpCloud = "F:\\data1\\00getfert_2_bld_imp.laser";;
	//	char* strImpMapPnt = "F:\\data1\\00getfert_2_bld_imp.objpts";
	//	char* strImpMapTop = "F:\\data1\\00getfert_2_bld_imp.top";
	//	char* strCompareRst = "F:\\data1\\00getfert_2_bld_rst.txt";


	char* strOrgCloud =  "F:\\data1\\gerfet_wet_1.laser";
	char* strOrgMapPnt = "F:\\data1\\gerfet_wet_1.objpts";
	char* strOrgMapTop = "F:\\data1\\gerfet_wet_1.top";
	char* strImpCloud =  "F:\\data1\\gerfet_wet_1_imp.laser";
	char* strImpMapPnt = "F:\\data1\\gerfet_wet_1_imp.objpts";
	char* strImpMapTop = "F:\\data1\\gerfet_wet_1_imp.top";
	char* strCompareRst = "F:\\data1\\gerfet_wet_1_rst.txt";

	orgCloud.Read(strOrgCloud);
	impCloud.Read(strImpCloud);
	orgBlds.ImportPCMMapData(strOrgMapPnt, strOrgMapTop, orgObjPnts);
	impBlds.ImportPCMMapData(strImpMapPnt, strImpMapTop, impObjPnts);
	FILE* file = fopen(strCompareRst, "w+");
	if (!file) return;
	vector<PointNumberList> orgPnlSegs, impPnlSegs;
	MyDeriveSegPNL(orgCloud, orgPnlSegs);
	MyDeriveSegPNL(impCloud, impPnlSegs);

	//vector<int> impSegNums;
	Buildings::iterator impBld, orgBld;
	for (impBld=impBlds.begin(); impBld!=impBlds.end(); ++impBld) {
		//impLines = *impBlds[iBld].MapDataPtr();
		//SearchSegNumsInBld(impBlds[iBld], impSegNums);
		//find corresponding building
		orgBld = SearchBldByBld(impBld, orgBlds);
		if (orgBld == orgBlds.end()) continue;

		//////////////////////////////////////////////////////////////////////////
		orgLines = SearchRidgesInBld(*orgBld);
		impLines = SearchRidgesInBld(*impBld);
		SearchSegNumsInBld(*orgBld, orgSegNums);
		SearchSegNumsInBld(*impBld, impSegNums);

		vector<char> orgSegValid(orgSegNums.size(), false);
		vector<char> impSegValid(impSegNums.size(), false);
		int temSegNum;
		//tag repeat segments
		for (int iSeg=0; iSeg<orgSegNums.size(); ++iSeg) {
			temSegNum = orgSegNums[iSeg];
			for (int jSeg=0; jSeg<impSegNums.size(); ++jSeg) {
				//repeat segments
				if (temSegNum==impSegNums[jSeg]) {
					orgSegValid[iSeg] = true;
					impSegValid[jSeg] = true;
				}
			}
		}
		//tag repeat ridge lines
		LineTopologies::iterator itrLine1, itrLine2;
		int segNum11, segNum12, segNum21, segNum22;
		vector<char> orgLineRepeat(orgLines.size(), false);
		vector<char> impLineRepeat(impLines.size(), false);
		for (itrLine1=orgLines.begin(); itrLine1!=orgLines.end(); ++itrLine1) {
			if (!itrLine1->HasAttribute(SegmentLabel) || !itrLine1->HasAttribute(SecondSegmentLabel))
				continue;
			segNum11 = itrLine1->Attribute(SegmentLabel);
			segNum12 = itrLine1->Attribute(SecondSegmentLabel);

			for (itrLine2=impLines.begin(); itrLine2!=impLines.end(); ++itrLine2) {
				if (!itrLine2->HasAttribute(SegmentLabel) || !itrLine2->HasAttribute(SecondSegmentLabel))
					continue;
				segNum21 = itrLine2->Attribute(SegmentLabel);
				segNum22 = itrLine2->Attribute(SecondSegmentLabel);
				if ((segNum11==segNum21 && segNum12==segNum22)
					|| (segNum11==segNum22 && segNum12==segNum21))
				{
					orgLineRepeat[itrLine1-orgLines.begin()] = true;
					impLineRepeat[itrLine2-impLines.begin()] = true;
				}
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//search candidate primitives in original and improved graphs
		//search new or deleted edges, then find their corresponding subgraphs
		//if two subgraphs are connected, they should combine to be a bigger one
		orgSubGraphs = SearchCandSubGraphs(orgSegNums, orgLines, orgLineRepeat);
		impSubGraphs = SearchCandSubGraphs(impSegNums, impLines, impLineRepeat);
		vector<vector<int> > matchSubGraphs = MatchSubGraphs(orgSegNums, 
			impSegNums, orgSubGraphs, impSubGraphs);
		if (matchSubGraphs.empty()) continue;

		//////////////////////////////////////////////////////////////////////////
		//write out
		fprintf(file, "Building %d\n", impBld-impBlds.begin());
		vector<int> temSubGraph;
		int segNum1, segNum2, segInd1, segInd2;
		int pntCoef;
		LaserPoints::iterator pnt;
		for (int iSub=0; iSub<matchSubGraphs.size()/2; iSub++) {
			fprintf(file, "Sub Building %d\n", iSub);

			//original subgraph
			//write out segment coefficient
			temSubGraph = matchSubGraphs[2*iSub];
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];

				segInd1 = IndexSegBySegNumber(orgCloud, orgPnlSegs, segNum1);
				if (segInd1 == -1) continue;
				pnt = orgCloud.begin()+orgPnlSegs[segInd1][0].Number();
				if (!pnt->HasAttribute(LabelTag)) continue;
				fprintf(file, "%d  %.2f\n", segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
			}
			fprintf(file, "\n");

			//write out ridge coefficient
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];
				for (int jSeg=iSeg+1; jSeg<temSubGraph.size(); ++jSeg) {
					segNum2 = temSubGraph[jSeg];
					if(!SearchLineBy2Faces(orgLines, segNum1, segNum2,lindInd))
						continue;
					if (!orgLines[lindInd].HasAttribute(GradientTag))
						continue;
					fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
						orgLines[lindInd].Attribute(GradientTag)/7.0);
				}
			}
			fprintf(file, "\n");

			//improved subgraph
			//write out segment coefficient
			temSubGraph = matchSubGraphs[2*iSub+1];
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];

				segInd1 = IndexSegBySegNumber(impCloud, impPnlSegs, segNum1);
				if (segInd1 == -1) continue;
				pnt = impCloud.begin()+impPnlSegs[segInd1][0].Number();
				if (!pnt->HasAttribute(LabelTag)) continue;
				fprintf(file, "%d %.2f\n", segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
			}
			fprintf(file, "\n");

			//write out ridge coefficient
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];
				for (int jSeg=iSeg+1; jSeg<temSubGraph.size(); ++jSeg) {
					segNum2 = temSubGraph[jSeg];
					if(!SearchLineBy2Faces(impLines, segNum1, segNum2, lindInd))
						continue;
					if (!impLines[lindInd].HasAttribute(GradientTag))
						continue;
					fprintf(file, "%d %d %.2f\n", segNum1, segNum2, 
						impLines[lindInd].Attribute(GradientTag)/7.0);
				}
			}
			fprintf(file, "\n");
		}
	}
	fclose(file);
}

void GraphDiction::Learn(char* srtOLaser, char* srtOMapTop, char* strOMapPnt,
	char* strILaser, char* strIMapTop, char* srtIMapPnt)
{
	assert(srtOLaser && srtOMapTop && strOMapPnt 
		&& strILaser && strIMapTop && srtIMapPnt);

	LaserPoints orgCloud, impCloud;
	Buildings orgBlds, impBlds;
	ObjectPoints orgObjPnts, impObjPnts;
	LineTopologies orgLines, impLines;
	vector<int> orgSegNums, impSegNums;
	int lindInd;
	vector<vector<int> > orgSubGraphs, impSubGraphs;

	orgCloud.Read(srtOLaser);
	impCloud.Read(strILaser);
	orgBlds.ImportPCMMapData(strOMapPnt, srtOMapTop, orgObjPnts);
	impBlds.ImportPCMMapData(srtIMapPnt, strIMapTop, impObjPnts);
	//	FILE* file = fopen(strCompareRst, "w+");
	//	if (!file) return;
	vector<PointNumberList> orgPnlSegs, impPnlSegs;
	MyDeriveSegPNL(orgCloud, orgPnlSegs);
	MyDeriveSegPNL(impCloud, impPnlSegs);

	//vector<int> impSegNums;
	Buildings::iterator impBld, orgBld;
	for (impBld=impBlds.begin(); impBld!=impBlds.end(); ++impBld) {
		//impLines = *impBlds[iBld].MapDataPtr();
		//SearchSegNumsInBld(impBlds[iBld], impSegNums);
		//find corresponding building
		orgBld = SearchBldByBld(impBld, orgBlds);
		if (orgBld == orgBlds.end()) continue;

		//////////////////////////////////////////////////////////////////////////
		orgLines = SearchRidgesInBld(*orgBld);
		impLines = SearchRidgesInBld(*impBld);
		SearchSegNumsInBld(*orgBld, orgSegNums);
		SearchSegNumsInBld(*impBld, impSegNums);

		vector<char> orgSegValid(orgSegNums.size(), false);
		vector<char> impSegValid(impSegNums.size(), false);
		int temSegNum;
		//tag repeat segments
		for (int iSeg=0; iSeg<orgSegNums.size(); ++iSeg) {
			temSegNum = orgSegNums[iSeg];
			for (int jSeg=0; jSeg<impSegNums.size(); ++jSeg) {
				//repeat segments
				if (temSegNum==impSegNums[jSeg]) {
					orgSegValid[iSeg] = true;
					impSegValid[jSeg] = true;
				}
			}
		}
		//tag repeat ridge lines
		LineTopologies::iterator itrLine1, itrLine2;
		int segNum11, segNum12, segNum21, segNum22;
		vector<char> orgLineRepeat(orgLines.size(), false);
		vector<char> impLineRepeat(impLines.size(), false);
		for (itrLine1=orgLines.begin(); itrLine1!=orgLines.end(); ++itrLine1) {
			if (!itrLine1->HasAttribute(SegmentLabel) || !itrLine1->HasAttribute(SecondSegmentLabel))
				continue;
			segNum11 = itrLine1->Attribute(SegmentLabel);
			segNum12 = itrLine1->Attribute(SecondSegmentLabel);

			for (itrLine2=impLines.begin(); itrLine2!=impLines.end(); ++itrLine2) {
				if (!itrLine2->HasAttribute(SegmentLabel) || !itrLine2->HasAttribute(SecondSegmentLabel))
					continue;
				segNum21 = itrLine2->Attribute(SegmentLabel);
				segNum22 = itrLine2->Attribute(SecondSegmentLabel);
				if ((segNum11==segNum21 && segNum12==segNum22)
					|| (segNum11==segNum22 && segNum12==segNum21))
				{
					orgLineRepeat[itrLine1-orgLines.begin()] = true;
					impLineRepeat[itrLine2-impLines.begin()] = true;
				}
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//search candidate primitives in original and improved graphs
		//search new or deleted edges, then find their corresponding subgraphs
		//if two subgraphs are connected, they should combine to be a bigger one
		orgSubGraphs = SearchCandSubGraphs(orgSegNums, orgLines, orgLineRepeat);
		impSubGraphs = SearchCandSubGraphs(impSegNums, impLines, impLineRepeat);
		vector<vector<int> > matchSubGraphs = MatchSubGraphs(orgSegNums, 
			impSegNums, orgSubGraphs, impSubGraphs);
		if (matchSubGraphs.empty()) continue;

		//////////////////////////////////////////////////////////////////////////
		//write out
		//	fprintf(file, "Building %d\n", impBld-impBlds.begin());
		vector<int> temSubGraph;
		int segNum1, segNum2, segInd1;//, segInd2
		PrimSubGraph orgPsGraph, impPsGraph;
		//		int pntCoef;
		LaserPoints::iterator pnt;
		for (int iSub=0; iSub<matchSubGraphs.size()/2; iSub++) {
			//fprintf(file, "Sub Building %d\n", iSub);

			//original subgraph
			//write out segment coefficient
			temSubGraph = matchSubGraphs[2*iSub];
			orgPsGraph.Clear();
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];

				segInd1 = IndexSegBySegNumber(orgCloud, orgPnlSegs, segNum1);
				if (segInd1 == -1) continue;
				pnt = orgCloud.begin()+orgPnlSegs[segInd1][0].Number();
				if (!pnt->HasAttribute(LabelTag)) continue;
				//fprintf(file, "%d  %.2f\n", segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
				orgPsGraph.AddNode(segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
			}
			//fprintf(file, "\n");

			//write out ridge coefficient
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];
				for (int jSeg=iSeg+1; jSeg<temSubGraph.size(); ++jSeg) {
					segNum2 = temSubGraph[jSeg];
					if(!SearchLineBy2Faces(orgLines, segNum1, segNum2,lindInd))
						continue;
					if (!orgLines[lindInd].HasAttribute(GradientTag))
						continue;
					//fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
					//	orgLines[lindInd].Attribute(GradientTag)/7.0);
					orgPsGraph.AddEdge(segNum1, segNum2, 
						orgLines[lindInd].Attribute(GradientTag)/7.0);
				}
			}
			//fprintf(file, "\n");

			//////////////////////////////////////////////////////////////////////////
			//Look up in the dictionary
			vector<int> vecMatchNodes;
			int idx = this->LookUp(orgPsGraph, vecMatchNodes, 1.5);

			//improved subgraph
			//write out segment coefficient
			temSubGraph = matchSubGraphs[2*iSub+1];
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];

				segInd1 = IndexSegBySegNumber(impCloud, impPnlSegs, segNum1);
				if (segInd1 == -1) continue;
				pnt = impCloud.begin()+impPnlSegs[segInd1][0].Number();
				if (!pnt->HasAttribute(LabelTag)) continue;
				//fprintf(file, "%d  %.2f\n", segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
			}
			//fprintf(file, "\n");

			//write out ridge coefficient
			for (int iSeg=0; iSeg<temSubGraph.size(); ++iSeg) {
				segNum1 = temSubGraph[iSeg];
				for (int jSeg=iSeg+1; jSeg<temSubGraph.size(); ++jSeg) {
					segNum2 = temSubGraph[jSeg];
					if(!SearchLineBy2Faces(impLines, segNum1, segNum2, lindInd))
						continue;
					if (!impLines[lindInd].HasAttribute(GradientTag))
						continue;
					//fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
					//	impLines[lindInd].Attribute(GradientTag)/7.0);
				}
			}
			//fprintf(file, "\n");
		}
	}// end each building
	//fclose(file);
}

void GraphDiction::CorrectGraph(char* srtLaser, char* srtMapTop, char* strMapPnt)
{
	assert(srtLaser && srtMapTop && strMapPnt);

	LaserPoints orgCloud;
	Buildings orgBlds;//, impBlds
	ObjectPoints orgObjPnts;
	LineTopologies orgLines;
	vector<int> orgSegNums;
	int lindInd;
	vector<vector<int> > orgSubGraphs;

	orgCloud.Read(srtLaser);
	orgBlds.ImportPCMMapData(strMapPnt, srtMapTop, orgObjPnts);
	//	FILE* file = fopen(strCompareRst, "w+");
	//	if (!file) return;
	vector<PointNumberList> orgPnlSegs, impPnlSegs;
	MyDeriveSegPNL(orgCloud, orgPnlSegs);

	//vector<int> impSegNums;
	Buildings::iterator itrBld;
	PrimSubGraph orgPsGraph, impPsGraph;
	int segNum1, segNum2, segInd1;
	LaserPoints::iterator pnt;
	double label;
	Line3D line;
	for (itrBld=orgBlds.begin(); itrBld!=orgBlds.end(); ++itrBld) {
		orgLines = SearchRidgesInBld(*itrBld);
		SearchSegNumsInBld(*itrBld, orgSegNums);

		orgPsGraph.Clear();
		//add nodes
		for (int iSeg=0; iSeg<orgSegNums.size(); ++iSeg) {
			segNum1 = orgSegNums[iSeg];
			segInd1 = IndexSegBySegNumber(orgCloud, orgPnlSegs, segNum1);
			if (segInd1 == -1) continue;
			pnt = orgCloud.begin()+orgPnlSegs[segInd1][0].Number();
			if (!pnt->HasAttribute(LabelTag)) continue;
			orgPsGraph.AddNode(segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
		}

		//add edges
		for (int iLine=0; iLine<orgLines.size(); ++iLine) {
			segNum1 = orgLines[iLine].Attribute(SegmentLabel);
			segNum2 = orgLines[iLine].Attribute(SecondSegmentLabel);
			if (segNum1 == segNum2) continue;
			if (!orgLines[iLine].HasAttribute(GradientTag))
				continue;
			//fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
			//	orgLines[lindInd].Attribute(GradientTag)/7.0);
			label = orgLines[iLine].Attribute(GradientTag)/7.0;
			line = Line3D(orgObjPnts[orgLines[iLine][0].Number()], 
				orgObjPnts[orgLines[iLine][1].Number()]);
			if (!line.IsHorizontal(10*3.1415926/180))
				label += 2.0;

			orgPsGraph.AddEdge(segNum1, segNum2, label);
		}

		orgPsGraph.ConstructSearchTree();
		vector<int> vecMatchNodes;
		int idx = this->LookUp(orgPsGraph, vecMatchNodes, 1.5);
	}// end each building
	//fclose(file);
}

bool GraphDiction::CorrectGraph(LaserPoints& gLasPnts,vector<Plane>& vecLocalPlanes, 
	 vector<PointNumberList>& gVecSegNums, Building& bld, ObjectPoints& modelObjPnts)
{
	PrimSubGraph orgPsGraph, impPsGraph;
	LineTopologies::iterator topLine;
	int segNum1, segNum2;
	int pntNum1, pntNum2;
	int segInd1, segInd2;
	LaserPoints::iterator pnt;
	double label;
	Line3D line;
	bool bNeedCorrect = false;
	LineTopologies* bldLineTops = bld.MapDataPtr();
	LineTopologies orgTopLines = SearchRidgesInBld(bld);
	vector<int> orgSegNums;
	SearchSegNumsInBld(bld, orgSegNums);
		
	//construct topology graph
	//add nodes
	for (int iSeg=0; iSeg<orgSegNums.size(); ++iSeg) {
		segNum1 = orgSegNums[iSeg];
		segInd1 = IndexSegBySegNumber(gLasPnts, gVecSegNums, segNum1);
		if (segInd1 == -1) continue;
		pnt = gLasPnts.begin()+gVecSegNums[segInd1][0].Number();
		if (!pnt->HasAttribute(LabelTag)) continue;
		label = 1.0-pnt->Attribute(LabelTag)/10.0;
		orgPsGraph.AddNode(segNum1, label);
		if (label<0.5) bNeedCorrect = true;
	}

	//directly return if do not need correction
	if (!bNeedCorrect) 
		return bNeedCorrect;

	//add edges
	bool hasBadRidge = false;
	float nodeLab1, nodeLab2;
	for (int iLine=0; iLine<orgTopLines.size(); ++iLine) {
		segNum1 = orgTopLines[iLine].Attribute(SegmentLabel);
		segNum2 = orgTopLines[iLine].Attribute(SecondSegmentLabel);
		if (segNum1 == segNum2) continue;
		if (!orgTopLines[iLine].HasAttribute(GradientTag))
			continue;
		//fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
		//	orgTopLines[lindInd].Attribute(GradientTag)/7.0);
		//directly delete poor edges
		label = orgTopLines[iLine].Attribute(GradientTag)/7.0;
		nodeLab1 = orgPsGraph.GetNodeScore(segNum1);
		nodeLab2 = orgPsGraph.GetNodeScore(segNum2);

		/*if (label < 0.3  || (nodeLab1<0.3&&nodeLab2<0.3&&label<0.6)) {
			hasBadRidge = true;
			orgTopLines.erase(orgTopLines.begin()+iLine);
			iLine--;
		}
		else
		{
			line = Line3D(modelObjPnts[orgTopLines[iLine][0].Number()], 
				modelObjPnts[orgTopLines[iLine][1].Number()]);
			if (!line.IsHorizontal(10*3.1415926/180))
				label += 2.0;

			orgPsGraph.AddEdge(segNum1, segNum2, label);
		}*/

		line = Line3D(modelObjPnts[orgTopLines[iLine][0].Number()], 
			modelObjPnts[orgTopLines[iLine][1].Number()]);
		if (!line.IsHorizontal(10*3.1415926/180)) label += 2.0;
		orgPsGraph.AddEdge(segNum1, segNum2, label);

		if (label < 0.3  || (nodeLab1<0.3&&nodeLab2<0.3&&label<0.6)) {
			hasBadRidge = true;
			orgTopLines.erase(orgTopLines.begin()+iLine);
			iLine--;
		}
	}

	LineTopologies 	impTopLines = orgTopLines;
	//if poor edges are deleted, no corrections are needed anymore.
	if (!hasBadRidge)  {
		orgPsGraph.ConstructSearchTree();
		vector<int> vecMatchNodes;
		int idx = this->LookUp(orgPsGraph, vecMatchNodes, 2.0);
		if (idx==-1) return false;//not found in the dictionary
	
		this->CorrectGraph(vecMatchNodes, idx, modelObjPnts, impTopLines);
	}

	//////////////////////////////////////////////////////////////////////////
	//recompute the ridge lines
	Position3D pos1, pos2;
	double scalar;
	PointNumberList pnlSeg1,pnlSeg2;
	Plane plane1, plane2;
	int indPlane1, indPlane2;
	int indSeg1, indSeg2;
	for (int i=0; i<impTopLines.size(); i++) {
		topLine = impTopLines.begin()+i;
		topLine->Attribute(2) = 0;
		segNum1 = topLine->Attribute(SegmentLabel);
		segNum2 = topLine->Attribute(SecondSegmentLabel);
		indPlane1 = IndexPlaneBySegNumber(vecLocalPlanes, segNum1);
		indPlane2 = IndexPlaneBySegNumber(vecLocalPlanes, segNum2);
		indSeg1 = IndexSegBySegNumber(gLasPnts, gVecSegNums, segNum1);
		indSeg2 = IndexSegBySegNumber(gLasPnts, gVecSegNums, segNum2);
		gLasPnts.IntersectFaces(gVecSegNums[indSeg1], gVecSegNums[indSeg2], 
			vecLocalPlanes[indPlane1], vecLocalPlanes[indPlane2],
			gBldRecPar.minPntDist, pos1, pos2);
		modelObjPnts[(*topLine)[0].Number()].Position3DRef() = pos1;
		modelObjPnts[(*topLine)[1].Number()].Position3DRef() = pos2;
	}

	//////////////////////////////////////////////////////////////////////////
	//out put
	bld.MapDataPtr()->clear();
	for (int i=0; i<impTopLines.size(); ++i) {
		bld.AddMapData(impTopLines[i]);
	}

	return bNeedCorrect;
}

void GraphDiction::CorrectGraph(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	//////////////////////////////////////////////////
	//do household work
	//remove boundary lines
	LineTopologies::iterator topLine;
	int segNum1, segNum2;
	int pntNum1, pntNum2;
	localLineTops = SearchRidgesInBld(localLineTops);
	vector<int> orgSegNums;
	SearchSegNumsInBld(localLineTops, orgSegNums);

	//remove useless object points
	ObjectPoints temLocalObjPnts;
	temLocalObjPnts.reserve(localLineTops.size()*2);
	int pntCount = 0;
	for (int i=0; i<localLineTops.size(); i++) {
		topLine = localLineTops.begin()+i;
		pntNum1 = (*topLine)[0].Number();
		pntNum2 = (*topLine)[1].Number();
		temLocalObjPnts.push_back(localObjPnts[pntNum1]);
		temLocalObjPnts.push_back(localObjPnts[pntNum2]);
		pntCount += 2;
		temLocalObjPnts[pntCount-2].Number() = pntCount-2;
		temLocalObjPnts[pntCount-1].Number() = pntCount-1;
		(*topLine)[0].Number() = pntCount-2;
		(*topLine)[1].Number() = pntCount-1;
	}
	localObjPnts = temLocalObjPnts;

	//////////////////////////////////////////////////////////////////////////
	//do refine roof top graph
	PrimSubGraph orgPsGraph, impPsGraph;
	int segInd1;
	LaserPoints::iterator pnt;
	double label;
	Line3D line;

	//add nodes
	for (int iSeg=0; iSeg<orgSegNums.size(); ++iSeg) {
		segNum1 = orgSegNums[iSeg];
		segInd1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum1);
		if (segInd1 == -1) continue;
		pnt = gLaserPnts.begin()+vecLocalSegPnts[segInd1][0].Number();
		if (!pnt->HasAttribute(LabelTag)) continue;
		orgPsGraph.AddNode(segNum1, 1.0-pnt->Attribute(LabelTag)/10.0);
	}

	//add edges
	for (int iLine=0; iLine<localLineTops.size(); ++iLine) {
		segNum1 = localLineTops[iLine].Attribute(SegmentLabel);
		segNum2 = localLineTops[iLine].Attribute(SecondSegmentLabel);
		if (segNum1 == segNum2) continue;
		if (!localLineTops[iLine].HasAttribute(GradientTag))
			continue;
		//fprintf(file, "%d-%d %.2f\n", segNum1, segNum2, 
		//	orgLines[lindInd].Attribute(GradientTag)/7.0);
		label = localLineTops[iLine].Attribute(GradientTag)/7.0;
		line = Line3D(localObjPnts[localLineTops[iLine][0].Number()], 
			localObjPnts[localLineTops[iLine][1].Number()]);
		if (!line.IsHorizontal(10*3.1415926/180))
			label += 2.0;

		orgPsGraph.AddEdge(segNum1, segNum2, label);
	}

	orgPsGraph.ConstructSearchTree();
	vector<int> vecMatchNodes;
	int idx = this->LookUp(orgPsGraph, vecMatchNodes, 1.5);


	//////////////////////////////////////////////////////////////////////////
	//recompute the ridge lines
	Position3D pos1, pos2;
	double scalar;
	PointNumberList pnlSeg1,pnlSeg2;
	Plane plane1, plane2;
	int indPlane1, indPlane2;
	int indSeg1, indSeg2;
	for (int i=0; i<localLineTops.size(); i++) {
		topLine = localLineTops.begin()+i;
		topLine->Attribute(2) = 0;
		segNum1 = topLine->Attribute(SegmentLabel);
		segNum2 = topLine->Attribute(SecondSegmentLabel);
		indPlane1 = IndexPlaneBySegNumber(vecLocalPlanes, segNum1);
		indPlane2 = IndexPlaneBySegNumber(vecLocalPlanes, segNum2);
		indSeg1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum1);
		indSeg2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum2);
		gLaserPnts.IntersectFaces(vecLocalSegPnts[indSeg1], vecLocalSegPnts[indSeg2], 
			vecLocalPlanes[indPlane1], vecLocalPlanes[indPlane2],
			gBldRecPar.minPntDist, pos1, pos2);
		localObjPnts[(*topLine)[0].Number()].Position3DRef() = pos1;
		localObjPnts[(*topLine)[1].Number()].Position3DRef() = pos2;
	}
}

void TestGraphDiction()
{
	/*	char* strOrgCloud = "F:\\data1\\building2_org.laser";
	char* strOrgMapPnt = "F:\\data1\\building2_org.objpts";
	char* strOrgMapTop = "F:\\data1\\building2_org.top";
	char* strImpCloud = "F:\\data1\\building2_imp.laser";
	char* strImpMapPnt = "F:\\data1\\building2_imp.objpts";
	char* strImpMapTop = "F:\\data1\\building2_imp.top";*/
	//	char* strOrgCloud = "F:\\data1\\gerfet_wet_2.laser";
	//	char* strOrgMapPnt = "F:\\data1\\gerfet_wet_2.objpts";
	//	char* strOrgMapTop = "F:\\data1\\gerfet_wet_2.top";

	//	LaserPoints temLasPnt;
	//	temLasPnt.Read("F:\\data\\data5\\5.laser");

	char* strOrgCloud  = "F:\\data1\\error5_1_org.laser";
	char* strOrgMapPnt = "F:\\data1\\error5_1_org.objpts";
	char* strOrgMapTop = "F:\\data1\\error5_1_org.top";

	char* strImpCloud = "F:\\data1\\gerfet_wet_imp.laser";
	char* strImpMapPnt = "F:\\data1\\gerfet_wet_imp.objpts";
	char* strImpMapTop = "F:\\data1\\gerfet_wet_imp.top";

	GraphDiction diction;
	diction.Read("f:\\test_data\\graph_dic.txt");
	//	diction.Learn(strOrgCloud, strOrgMapTop, strOrgMapPnt, 
	//		strImpCloud, strImpMapTop, strImpMapPnt);
	//diction.CorrectGraph(strOrgCloud, strOrgMapTop, strOrgMapPnt);
	printf("F:\\data1\\error5_1_org.laser\n");

	//	diction.CorrectGraph("F:\\data1\\000.laser","F:\\data1\\000.top","F:\\data1\\000.objpts");
	//diction.CorrectGraph("F:\\data1\\error2_1_org.laser","F:\\data1\\error2_1_org.top","F:\\data1\\error2_1_org.objpts");
	//diction.CorrectGraph("F:\\data1\\error2_2_org.laser","F:\\data1\\error2_2_org.top","F:\\data1\\error2_2_org.objpts");
	diction.CorrectGraph("F:\\data1\\error1_1_org.laser","F:\\data1\\error1_1_org.top","F:\\data1\\error1_1_org.objpts");
	diction.CorrectGraph("F:\\data1\\error1_2_org.laser","F:\\data1\\error1_2_org.top","F:\\data1\\error1_2_org.objpts");

	//	diction.CorrectGraph("F:\\data1\\error4_1_org.laser","F:\\data1\\error4_1_org.top","F:\\data1\\error4_1_org.objpts");

	//	printf("F:\\data1\\error5_1_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_1_org.laser","F:\\data1\\error5_1_org.top","F:\\data1\\error5_1_org.objpts");
	//	printf("F:\\data1\\error5_2_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_2_org.laser","F:\\data1\\error5_2_org.top","F:\\data1\\error5_2_org.objpts");
	//	printf("F:\\data1\\error5_3_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_3_org.laser","F:\\data1\\error5_3_org.top","F:\\data1\\error5_3_org.objpts");
	//	printf("F:\\data1\\error5_4_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_4_org.laser","F:\\data1\\error5_4_org.top","F:\\data1\\error5_4_org.objpts");
	//	printf("F:\\data1\\error5_5_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_5_org.laser","F:\\data1\\error5_5_org.top","F:\\data1\\error5_5_org.objpts");
	//	printf("F:\\data1\\error5_6_org.laser\n");
	//	diction.CorrectGraph("F:\\data1\\error5_6_org.laser","F:\\data1\\error5_6_org.top","F:\\data1\\error5_6_org.objpts");
}
