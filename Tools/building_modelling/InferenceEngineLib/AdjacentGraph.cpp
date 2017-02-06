//#include "InferenceEngine.h"
#include "AdjacentGraph.h"
#include <vector>
#include "Vector3D.h"
#include "LaserPoints.h"
#include <queue>
#include <stack>
#include <float.h>

using namespace std;
Position3D INVALIDPOS(DBL_MAX, DBL_MAX, DBL_MAX);

AdjacentGraph::AdjacentGraph(const LaserPoints& gLaserPnts, LineTopologies* pLocalTopLines,
	const std::vector<PointNumberList>& vecLocalSegPnts)
{
	m_pTopLines = pLocalTopLines;
	m_vecCliques.clear();
	m_vecNodeNums.clear();
	//m_vecNodeNums.clear();
	m_mapNodeValid.clear();
	m_bSearched = false;
	 
	int curSegNum;
	vector<PointNumberList>::const_iterator itrSeg;
	LaserPoints::const_iterator itrPnt;
	for (itrSeg=vecLocalSegPnts.begin(); itrSeg!=vecLocalSegPnts.end(); ++itrSeg) {
		assert(!itrSeg->empty()
			&& "segment should not be empty");
		itrPnt = gLaserPnts.begin()+(*itrSeg)[0].Number();
		assert(itrPnt->HasAttribute(SegmentNumberTag) 
			&& "laser point should have segment number tag");
		curSegNum = itrPnt->Attribute(SegmentNumberTag);
		m_vecNodeNums.push_back(curSegNum);
		m_mapNodeValid[curSegNum] = true;
	}

	std::sort(m_vecNodeNums.begin(), m_vecNodeNums.end());
	vector<int>::iterator itr = std::unique(m_vecNodeNums.begin(), m_vecNodeNums.end());
	m_vecNodeNums = vector<int>(m_vecNodeNums.begin(), itr);
}

AdjacentGraph::AdjacentGraph(const std::vector<int>& vecFaceNums, LineTopologies* pLocalTopLines)
{
	m_pTopLines = pLocalTopLines;
	m_vecCliques.clear();
	m_vecNodeNums.clear();
	//m_vecNodeNums.clear();
	m_mapNodeValid.clear();
	m_bSearched = false;
	 
	int curSegNum;
	vector<int>::const_iterator itrSeg;
	for (itrSeg=vecFaceNums.begin(); itrSeg!=vecFaceNums.end(); ++itrSeg) {
		curSegNum = *itrSeg;
		m_vecNodeNums.push_back(curSegNum);
		m_mapNodeValid[curSegNum] = true;
	}

	std::sort(m_vecNodeNums.begin(), m_vecNodeNums.end());
	vector<int>::iterator itr = std::unique(m_vecNodeNums.begin(), m_vecNodeNums.end());
	m_vecNodeNums = vector<int>(m_vecNodeNums.begin(), itr);
}

AdjacentGraph::~AdjacentGraph(void)
{

}

AdjacentGraph& AdjacentGraph::operator = (const AdjacentGraph& srcGraph)
{
	if (this == &srcGraph)//do not copy itself
		return *this;

	this->m_pTopLines = srcGraph.m_pTopLines;
	this->m_vecCliques = srcGraph.m_vecCliques;
	this->m_vecNodeNums = srcGraph.m_vecNodeNums;
	this->m_mapNodeValid = srcGraph.m_mapNodeValid;
	this->m_bSearched = srcGraph.m_bSearched;

	return *this;
}

void AdjacentGraph::DoSearchCliques()
{
	m_vecCliques.clear();
	std::map<int, bool>::iterator itr;
	//clean the old information
	for (itr=m_mapNodeValid.begin(); itr!=m_mapNodeValid.end(); itr++)
		itr->second = true;

	Search1NodeClique();
	Search2NodesClique();
	Search3NodesClique();
	SearchXNodesClique();
	m_bSearched = true;
}

bool AdjacentGraph::PruneInvalidCliques(const ObjectPoints& locObjPts)
{
	MAPCLIPOS mapCliPos = ComputeCliCorner(locObjPts);
	vector<Position3D> vecCorners;
	vector<vector<vector<int> > > vecCliqs;
	Position3D temPos;

	MAPCLIPOS::iterator itrCliPos;
	bool bOverlap;
	for (itrCliPos=mapCliPos.begin(); itrCliPos!=mapCliPos.end(); itrCliPos++) {
		bOverlap = false;
		
		//compare the new clique with old ones
		for (int i=0; i<vecCorners.size(); i++) {
			double dist = vecCorners[i].Distance(itrCliPos->second);
			if (itrCliPos->second==INVALIDPOS || vecCorners[i].Distance(itrCliPos->second)>0.6) 
				continue;
			vecCliqs[i].push_back(itrCliPos->first);
			//update the corner
			vecCorners[i] = (vecCliqs[i].size()*vecCorners[i]+itrCliPos->second)/(vecCliqs[i].size()+1);
			bOverlap = true;
		}

		//initialize 
		if (itrCliPos==mapCliPos.begin() || !bOverlap) {
			vecCorners.push_back(itrCliPos->second);
			vector<vector<int> > temGroupCliqs;
			temGroupCliqs.push_back(itrCliPos->first);
			vecCliqs.push_back(temGroupCliqs);
			continue;
		}
	}
	
	bool bPruned = false;
	vector<int> vecIndTops;
	vector<double> vecLens;
	int curIndTop;
	double minLen = 99999.99;
	double temLen;
	vector<bool> vecValidTops;

	for (int i=0; i<vecCliqs.size(); i++) {
		if (vecCliqs[i].size()<=1) continue;
		vecIndTops.clear();

		//get all ridges for the overlapping cliques
		for (int j=0; j<vecCliqs[i].size(); j++) {
			for (int k=0; k<vecCliqs[i][j].size(); k++) {
				int& seg1 = vecCliqs[i][j][k];
				int& seg2 = (k==vecCliqs[i][j].size()-1)? vecCliqs[i][j][0]:vecCliqs[i][j][k+1];
				if(SearchLineBy2Faces(*m_pTopLines, seg1, seg2, curIndTop)) 
					vecIndTops.push_back(curIndTop);
			}
		}

		std::sort(vecIndTops.begin(), vecIndTops.end());
		vecIndTops.erase(std::unique(vecIndTops.begin(), vecIndTops.end()), vecIndTops.end());

		//get the length of the ridges
		vecLens.clear();
		for (int j=0; j<vecIndTops.size(); j++)	{
			LineTopology& curTop = (*m_pTopLines)[vecIndTops[j]];
			temLen = locObjPts[curTop[0].Number()].Distance(locObjPts[curTop[1].Number()]);
			vecLens.push_back(temLen);
			if (temLen<minLen) {
				curIndTop = vecIndTops[j];
				minLen = temLen;
			}
		}

		//Tag the invalid ridges
		//invalid ridge number equals to number of cliques - 1
		vecValidTops = vector<bool>(m_pTopLines->size(), true);
		for (int j=0; j<vecCliqs[i].size()-1; j++) {
			vector<double>::iterator itrMinLen = std::min_element(vecLens.begin(),vecLens.end());
			vector<int>::iterator itrMinRid = vecIndTops.begin()+(itrMinLen-vecLens.begin());
			vecValidTops[*itrMinRid] = false;
			vecLens.erase(itrMinLen);
			vecIndTops.erase(itrMinRid);
		}

		//do prune
		for (int j=0; j<vecValidTops.size(); ++j) {
			if (vecValidTops[j]) continue;
			m_pTopLines->erase(m_pTopLines->begin()+j);
			vecValidTops.erase(vecValidTops.begin()+j);
			j--;
		}

		bPruned = true;
	}

	//research the cliques based on the new top lines
	if (bPruned) DoSearchCliques();

	return bPruned;
}

vector< vector<int> > AdjacentGraph::GetLeastCliques()
{
	if (!m_bSearched)	{
		DoSearchCliques();
		m_bSearched = true;
	}

	return m_vecCliques;
}

bool AdjacentGraph::WriteCliques(char* path)
{
	if (!path) return false;
	FILE* file = fopen(path, "w");
	if (!file) return false;

	//nodes
	fprintf(file, "Nodes:\n");
	for (int i=0; i<m_vecNodeNums.size(); i++) {
		fprintf(file, "%4d\n", m_vecNodeNums[i]);
	}

	int indTop;
	fprintf(file, "\n\nEdges:\n");
	for(int i=0; i<m_vecNodeNums.size(); i++){
		int & node1 = m_vecNodeNums[i];
		for (int j=i+1; j<m_vecNodeNums.size(); j++) {
			int& node2 = m_vecNodeNums[j];
			if(SearchLineBy2Faces(*m_pTopLines, node1, node2, indTop))
				fprintf(file, "%4d %4d,", node1, node2);
		}
		fprintf(file, "\n");
	}

	//cliques
	fprintf(file, "\n\nCliques:\n");
	for (int i=0; i<m_vecCliques.size(); i++) {
		for (int j=0; j<m_vecCliques[i].size(); j++) {
			fprintf(file, "%4d ", m_vecCliques[i][j]);
		}
		fprintf(file, "\n");
	}

	fclose(file);
}

void AdjacentGraph::Search1NodeClique()
{
	vector<int> vecLineInds;
	int curSegNum;
	vector<int> clique(1,0);

	for (int iNode=0; iNode<m_vecNodeNums.size(); ++iNode) {
		curSegNum = m_vecNodeNums[iNode];
		if (!m_mapNodeValid[curSegNum])
			continue;

		SearchLinesOnPlane(*m_pTopLines, curSegNum, vecLineInds);
		if (!vecLineInds.empty()) continue;

		clique[0] = curSegNum;
		m_vecCliques.push_back(clique);
		m_mapNodeValid[curSegNum] = false;
	}
}

void AdjacentGraph::Search2NodesClique()
{
	vector<int> vecNodeNums, vecValidNodeNums;
	int curSegNum, nextSegNum=-1;
	vector<int> clique(2, 0);

	for (int iNode=0; iNode<m_vecNodeNums.size(); ++iNode) {
		curSegNum = m_vecNodeNums[iNode];
		nextSegNum=-1;
		if (!m_mapNodeValid[curSegNum])
			continue;

		for( ; ;){
			SearchAdjacentFaces(*m_pTopLines, curSegNum, vecNodeNums);
			vecValidNodeNums = GetValidNodes(m_mapNodeValid, vecNodeNums);

			if (vecValidNodeNums.size() != 1) {
				//last node in a sequence of two node clique
				if (vecValidNodeNums.empty()&&nextSegNum!=-1)
					m_mapNodeValid[nextSegNum] = false;
				break;//stop search
			}
			
			//out put
			m_mapNodeValid[curSegNum] = false;
			nextSegNum = vecValidNodeNums[0];
			clique[0] = curSegNum;
			clique[1] = nextSegNum;
			m_vecCliques.push_back(clique);

			//prepare for next loop
			curSegNum = nextSegNum;
		}
	}
}

void AdjacentGraph::Search3NodesClique()
{
	vector<int> vecNodeNums;
	vector<int> vecValidNodes, vecValidNodes2;
	int curSegNum, segNum1, segNum2;
	vector<int> clique(3, 0);
	stack<int> stackNodes;
	int lineInd;

	for (int iNode=0; iNode<m_vecNodeNums.size(); ++iNode) {
		curSegNum = m_vecNodeNums[iNode];
		if (!m_mapNodeValid[curSegNum])
			continue;

		//iterative search 3 node cliques
		//breath first search
		stackNodes.push(curSegNum);
		while(!stackNodes.empty()) {
			curSegNum = stackNodes.top();
			stackNodes.pop();
			if (!m_mapNodeValid[curSegNum])
				continue;//this node is searched
			SearchAdjacentFaces(*m_pTopLines, curSegNum, vecNodeNums);
			vecValidNodes = GetValidNodes(m_mapNodeValid, vecNodeNums);

			if (vecValidNodes.size() != 2) {
				continue;//stop search
			}

			//out put
			segNum1 = vecValidNodes[0];
			segNum2 = vecValidNodes[1];
			if(!SearchLineBy2Faces(*m_pTopLines, segNum1, segNum2, lineInd))
				continue;//node 1 and node 2 is not connected

			m_mapNodeValid[curSegNum] = false;
			clique[0] = curSegNum;
			clique[1] = segNum1;
			clique[2] = segNum2;
			m_vecCliques.push_back(clique);

			//prepare for next loop
			SearchFaceBy2Faces(*m_pTopLines, segNum1, segNum2, vecNodeNums);
			vecValidNodes2 = GetValidNodes(m_mapNodeValid, vecNodeNums);
			
			if (vecValidNodes.empty()) {
				//last edge of the three node clique
				//stop iteration
				m_mapNodeValid[segNum1] = false;
				m_mapNodeValid[segNum2] = false;
			}
			else {
				stackNodes.push(segNum2);
				stackNodes.push(segNum1);
			}
		}//end while
	}//end for

}

void AdjacentGraph::SearchXNodesClique()
{
	int curSegNum, temSegNum;
	GraphTree tree;
	vector<vector<int> > vecCliques;
	stack<int> staNodes;
	vector<int> vecAdjNodes, vecVAdjNodes;
	int segNum1, segNum2;
	int lineInd;
	vector<int> clique;
	LineTopologies::const_iterator itrLine;

	for (int iNode=0; iNode<m_vecNodeNums.size(); ++iNode) {
		curSegNum = m_vecNodeNums[iNode];
		if (!m_mapNodeValid[curSegNum])
			continue;

		tree = GraphTree(*m_pTopLines, m_mapNodeValid, curSegNum);
		vecCliques = tree.GetLeastCliques();
		AddValidClique(vecCliques);
		m_mapNodeValid[curSegNum] = false;

		////////////////////////////////////////////////////////////////////////////////////////
		//remove cracked edges after extracting cliques
		for (int j=0; j<vecCliques.size(); ++j) {
			for (int k=0; k<vecCliques[j].size(); ++k) {
				temSegNum = vecCliques[j][k];
				if (m_mapNodeValid[temSegNum])
					staNodes.push(temSegNum);
			}
		}

		while(!staNodes.empty()) {
			temSegNum = staNodes.top();
			staNodes.pop();
			if (!m_mapNodeValid[temSegNum]) continue;
			SearchAdjacentFaces(*m_pTopLines, temSegNum, vecAdjNodes);
			vecVAdjNodes = GetValidNodes(m_mapNodeValid, vecAdjNodes);
			if (vecVAdjNodes.size() == 1) {
				m_mapNodeValid[temSegNum] = false;
				staNodes.push(vecVAdjNodes[0]);
			}
			else if (vecVAdjNodes.empty()) {
				m_mapNodeValid[temSegNum] = false;
			}
		}//end while

		//some two nodes clique will appear after clearing found cliques
		for (itrLine = m_pTopLines->begin(); itrLine != m_pTopLines->end(); ++itrLine) {
			segNum1 = itrLine->Attribute(SegmentLabel);
			segNum2 = itrLine->Attribute(SecondSegmentLabel);
			if (segNum1 == -1 || segNum2 == -1) continue;
			//one node is valid
			if ((m_mapNodeValid[segNum1]&&m_mapNodeValid[segNum2])
				||(!m_mapNodeValid[segNum1]&&!m_mapNodeValid[segNum2]))
				continue;

			//has third node to connect them
			if (SearchLineBy2Faces(*m_pTopLines, segNum1, segNum2, lineInd))
				continue;
			clique.clear();
			clique.push_back(segNum1);
			clique.push_back(segNum2);
			m_vecCliques.push_back(clique);
		}
	}//end for
}

 // vector<int> AdjacentGraph::GetValidNodes(vector<int> vecInNodeNums)
 vector<int> GetValidNodes(const std::map<int, bool>& mapNodeValid, const vector<int>& vecInNodeNums)
{
	vector<int> vecOutNodeNums;
	int temNodeNum;
	std::map<int, bool>::const_iterator itr;

	for (int i=0; i<vecInNodeNums.size(); ++i) {
		temNodeNum = vecInNodeNums[i];
		itr = mapNodeValid.find(temNodeNum);
		
		if (itr!=mapNodeValid.end() && itr->second) 
			vecOutNodeNums.push_back(temNodeNum);
	}

	return vecOutNodeNums;
}

 void AdjacentGraph::AddValidClique(std::vector<std::vector<int> > inCliques)
 {
	 bool validClique;
	 std::vector<std::vector<int> >::iterator clique;

	 for (clique=inCliques.begin(); clique!=inCliques.end(); clique++) {
		 validClique = true;
		 for (int jNode=0; jNode<clique->size(); ++jNode)	{
			 if (!m_mapNodeValid[(*clique)[jNode]])	{
				 validClique = false;
				 break;
			 }
		 }

		 if (validClique)
			 m_vecCliques.push_back(*clique);
		// m_vecCliques.insert(m_vecCliques.end(), vecCliques.begin(), vecCliques.end());
	 }
 }

GTreeNode::GTreeNode () 
{
	m_pFather = NULL;
	m_nNNum = -1;
}

GTreeNode::GTreeNode (GTreeNode* pFather, int nNodeNum) 
{
	m_pFather = pFather;
	m_nNNum = nNodeNum;
}

GTreeNode::~GTreeNode () 
{
	for (int i=0; i<m_Children.size(); ++i) {
		if (m_Children[i]) {
			delete m_Children[i];
			m_Children[i] = NULL;
		}
	}
	m_pFather = NULL;
}

GTreeNode& 
	GTreeNode::operator = (const GTreeNode& src)
{
	if (this == &src)
		return *this;

	if (m_pFather == NULL) //does not have father
		m_pFather = src.m_pFather;	
	m_nNNum = src.m_nNNum;
	
	//clear old children
	for (int i=0; i<m_Children.size(); ++i) {
		if (m_Children[i]) {
			delete m_Children[i];
			m_Children[i] = NULL;
		}
	}
	m_Children.clear();

	GTreeNode* curNode, *curSrcNode;
	for (int i=0; i<src.m_Children.size(); ++i) {
		curSrcNode = src.m_Children[i];
		curNode = new GTreeNode(this, curSrcNode->m_nNNum);
		*curNode = *curSrcNode;//copy
		m_Children.push_back(curNode);
	}
	
	return *this;
}

GraphTree::GraphTree(const LineTopologies& topLines, 
	const std::map<int, bool>& mapNodeValid, int curNode)
{
	m_mapNValid = mapNodeValid;
	std::map<int, bool>::iterator pair;
	for (pair=m_mapNValid.begin(); pair!=m_mapNValid.end(); pair++) {
		pair->second = true;
	}

	m_RootNode = new GTreeNode(NULL, curNode);
	m_TopLines = topLines;

	queue<GTreeNode*> queNodes;
	GTreeNode* pCurNode = m_RootNode;
	vector<int> vecANodes, vecVANodes; //adjacent nodes, valid adjacent nodes
	vecANodes.reserve(50);
	vecVANodes.reserve(50);
	int childNum;
	GTreeNode* pChildNode;

	m_mapNValid[pCurNode->m_nNNum] = false;
	queNodes.push(pCurNode);
	while(!queNodes.empty()) {
		pCurNode = queNodes.front();
		queNodes.pop();
		if (!pCurNode ) 
			continue;

		SearchAdjacentFaces(m_TopLines, pCurNode->m_nNNum, vecANodes);
		vecVANodes = GetValidNodes(m_mapNValid, vecANodes);
		//vecVANodes = vecANodes;

		for (int iNode=0; iNode<vecVANodes.size(); ++iNode) {
			childNum = vecVANodes[iNode];
			if (!m_mapNValid[childNum])
				continue;

			pChildNode = new GTreeNode(pCurNode, childNum);
			pCurNode->m_Children.push_back(pChildNode);
			m_mapNValid[childNum] = false;
			queNodes.push(pChildNode);
		}
	}
}

std::vector<std::vector<int> > 
	GraphTree::GetLeastCliques()
{
	vector<vector<int> > vecLCliques;
	vector<int> clique;
	GTreeNode *pNode1, *pNode2;
	int topLineInd;
	stack<GTreeNode*> staNodes1, staNodes2;
	bool bFind;//find least clique
	
	if (!m_RootNode){//no tree
		return vecLCliques;
	}

	//one node clique
	if (m_RootNode->m_Children.empty())	{
		clique.push_back(m_RootNode->m_nNNum);
		vecLCliques.push_back(clique);
		return vecLCliques;
	}
	//two nodes clique
	else if (m_RootNode->m_Children.size() == 1) {
		pNode1 = m_RootNode;
		pNode2 = m_RootNode->m_Children[0];
		clique.push_back(pNode1->m_nNNum);
		clique.push_back(pNode2->m_nNNum);
		vecLCliques.push_back(clique);
		return vecLCliques;
	}
	
	for (int i=0; i<m_RootNode->m_Children.size()-1; i++)	{
		//go to root node's children
		for (int j=i+1; j<m_RootNode->m_Children.size(); ++j) {
			//Check whether node1 can form least clique
			//If not, deep-first search its children. 
			//The result of one node does not affect its siblings.
			pNode1 = m_RootNode->m_Children[i];
			if (!pNode1) continue;
			staNodes1.push(pNode1);
			while(!staNodes1.empty()) {
				pNode1 = staNodes1.top();
				staNodes1.pop();

				bFind = false;
				pNode2 = m_RootNode->m_Children[j];
				if (pNode2) staNodes2.push(pNode2);

				//iterative search least cliques
				//deep first search
				while (!staNodes2.empty()) {
					pNode2 = staNodes2.top();
					staNodes2.pop();

					if (SearchLineBy2Faces( m_TopLines, 
						pNode1->m_nNNum, pNode2->m_nNNum, topLineInd))
					{//stop search this node
						clique = SortClique(pNode1, pNode2);
						vecLCliques.push_back(clique);
						bFind = true;
					}
					else {//go to children
						for (int j=pNode2->m_Children.size()-1; j>=0; --j) 
							staNodes2.push(pNode2->m_Children[j]);
					}
				}//end staNode2

				//node1 cannot form least clque
				//go deep into its children
				if (!bFind) {
					for (int j=pNode1->m_Children.size()-1; j>=0; --j) 
						staNodes1.push(pNode1->m_Children[j]);
				}
			}
		}//stop staNode1

	}//end for


	return vecLCliques;
}

std::vector<int> GraphTree::SortClique(GTreeNode* pNode1, GTreeNode* pNode2)
{
	assert(pNode1 && pNode2 && "node should not be empty");

	vector<int> clque;
	vector<int> temVec;
	GTreeNode* pTemNode = NULL;

	pTemNode = pNode1;
	while (pTemNode) {
		temVec.push_back(pTemNode->m_nNNum);
		pTemNode = pTemNode->m_pFather;
	}
	std::reverse(temVec.begin(), temVec.end());
	clque.insert(clque.end(), temVec.begin(), temVec.end());

	temVec.clear();
	pTemNode = pNode2;
	while (pTemNode) {
		temVec.push_back(pTemNode->m_nNNum);
		pTemNode = pTemNode->m_pFather;
	}
	assert(!temVec.empty() && "the list should not be empty");
	clque.insert(clque.end(), temVec.begin(), temVec.end()-1);

	return clque;
}

GraphTree::~GraphTree()
{
	if (m_RootNode) {
		delete m_RootNode;
		m_RootNode = NULL;	
	}	
}

GraphTree& 
	GraphTree::operator = (const GraphTree& src)
{
	if (this == &src)
		return *this;

	//clear old data and allocate new space
	if (m_RootNode) {
		delete m_RootNode;
		m_RootNode = NULL;
	}
	m_RootNode = new GTreeNode; 

	//copy
	*m_RootNode = *src.m_RootNode;
	m_TopLines = src.m_TopLines;
	m_mapNValid = src.m_mapNValid;

	return *this;
}

void GraphTree::ConsTreeNode(GTreeNode* pCurNode, 
	int curNodeNum)
{
	if (!pCurNode) return;
	m_mapNValid[curNodeNum] = false;
	
	GTreeNode* pChild = NULL;
	vector<int> vecANodes, vecVANodes; //adjacent nodes, valid adjacent nodes
	vecANodes.reserve(50);
	vecVANodes.reserve(50);
	SearchAdjacentFaces(m_TopLines, curNodeNum, vecANodes);
	vecVANodes = GetValidNodes(m_mapNValid, vecANodes);
	int childNum;

	for (int iNode=0; iNode<vecVANodes.size(); ++iNode) 	{
		childNum = vecVANodes[iNode];
		pChild = new GTreeNode(pCurNode, childNum);
		pCurNode->m_Children.push_back(pChild);
		m_mapNValid[childNum] = false;
	}

	for (int iChild=0; iChild= pCurNode->m_Children.size(); iChild++) {
		pChild = pCurNode->m_Children[iChild];
		if (pChild)
			ConsTreeNode(pChild, pChild->m_nNNum);
	}
}

AdjacentGraph::MAPCLIPOS AdjacentGraph::ComputeCliCorner(const ObjectPoints& locObjPnts)
{
	vector<int> clique;
	MAPCLIPOS mapCliPos;
	//vector<bool> vecValidCorners;
	vector<Position3D> vecObjPnts;
	Position3D aveObjPnt, temObjPnt, ObjPnt1, ObjPnt2;
	int segNum1, segNum2;
	double temDist1, temDist2, leastDist;
	double meanDist;//mean distance of cross points to mean cross point
	double meanGap;//mean gap of end points to mean cross point
	vector<double> vecDists;
	LineTopologies vecTopLines;
	LineTopology temTopLine;
	bool bFailFlag;
	int indLine;
	vector<Position3D> vecIntPnts;
	vector<Line3D> vecLines;
	vector<Line3D>::iterator itrLine1, itrLine2;
	int nParLine;

	for (int iC=0; iC<m_vecCliques.size(); ++iC) {
		clique = m_vecCliques[iC];
		if (clique.size()<3) {
			mapCliPos.insert(MAPCLIPOS::value_type(clique, INVALIDPOS));
			continue;
		}

		//get ridge lines
		bFailFlag = false;
		vecTopLines.clear();
		vecLines.clear();
		vecDists.clear();
		vecIntPnts.clear();

		for (int iNode=0; iNode<clique.size(); ++iNode) {
			segNum1 = clique[iNode];
			segNum2 = iNode==clique.size()-1?clique[0]:clique[iNode+1];

			SearchLineBy2Faces(*m_pTopLines, segNum1, segNum2, indLine);
			if (indLine == -1) {
				bFailFlag = true;
				break;
			}
			vecTopLines.push_back((*m_pTopLines)[indLine]);
			ObjPnt1 = locObjPnts[(*m_pTopLines)[indLine][0].Number()];
			ObjPnt2 = locObjPnts[(*m_pTopLines)[indLine][1].Number()];
			vecLines.push_back(Line3D(ObjPnt1, ObjPnt2));
		}
		if (bFailFlag)  {
			mapCliPos.insert(MAPCLIPOS::value_type(clique, INVALIDPOS));
			continue;
		}

		//get intersection point
		aveObjPnt = Position3D(0.0, 0.0, 0.0);
		meanDist = 0.0;
		meanGap = 0.0;
		nParLine = 0;
		for (int iLine=0; iLine<vecLines.size(); ++iLine) {
			itrLine1 = vecLines.begin()+iLine;
			if (iLine==vecLines.size()-1)
				itrLine2 = vecLines.begin();
			else
				itrLine2 = vecLines.begin()+iLine+1;
			if (MyIsParaller(itrLine1->Direction(), itrLine2->Direction(), 10*PI/180)) {
				nParLine++;
			}
			else {
				MyIntersect2Lines(*itrLine1, *itrLine2, temObjPnt);

				ObjPnt1 = locObjPnts[vecTopLines[iLine][0].Number()];
				ObjPnt2 = locObjPnts[vecTopLines[iLine][1].Number()];
				temDist1 = temObjPnt.Distance(ObjPnt1);
				temDist2 = temObjPnt.Distance(ObjPnt2);
				leastDist = temDist1<temDist2?temDist1:temDist2;

				vecDists.push_back(leastDist);
				vecIntPnts.push_back(temObjPnt);
				aveObjPnt += temObjPnt;
				meanGap += leastDist/vecLines.size();
			}
		}

		if (vecLines.size()!=nParLine)	{
			aveObjPnt /= (vecLines.size()-nParLine);
			meanDist = 0.0;
			for (int i=0; i<vecIntPnts.size(); i++)
				meanDist += vecIntPnts[i].Distance(aveObjPnt)/vecIntPnts.size();
		}
		else
			meanDist = 10000.0;

		//vecValidCorners.push_back(meanDist>2.5);
		if (meanDist<0.5 && meanGap<2.5)
			mapCliPos.insert(MAPCLIPOS::value_type(clique, aveObjPnt));
		else
			mapCliPos.insert(MAPCLIPOS::value_type(clique, INVALIDPOS));		
	}

	return mapCliPos;
}

void TestAdjGraph () 
{
	LineTopologies lineTops;
	vector<vector<int> > vecCliques;
}
