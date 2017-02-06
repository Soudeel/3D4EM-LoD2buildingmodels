#ifndef __LEARNING_H_2012_06_08__
#define __LEARNING_H_2012_06_08__
#include <vector>
#include <map>
#include <iostream>
#include "InferenceEngine.h"

class IEDll_API PrimSubGraph
{
public:
	struct edge{
		int node1;
		int node2;
		edge(){node1=-1; node2 = -1;}
		edge(int inNode1, int inNode2);

		friend bool operator < (const edge &a, const edge &b);
		//bool operator <(edge inEdge) ;
		friend bool operator == (const edge &a, const edge &b);
	//	friend std::ostream& operator<<(std::ostream& stream, edge& e);
	//	friend std::istream& operator>>(std::istream& stream, edge& e);
	};

	class tNode {
	public:
		tNode();
		tNode(tNode* father, int data);
		~tNode();
		tNode& operator = (const tNode& src);

		std::vector<int> GetBranchData();
	public:
		int m_data;
		tNode* m_father; // NULL for root
		std::vector<tNode*> m_child; // NULLs for leaves
	};
public:
	float EditDistance(PrimSubGraph& refGraph, std::vector<int>& vecMatchNodes);
	void Difference(const PrimSubGraph& destGraph, std::vector<int>& removedSeg,
		std::vector<int>& newSeg, std::vector<int>& removedEdge, std::vector<int>& newEdge);
	void Clear();
	void AddNode(int nodeNum, float nodeScore);
	void AddEdge(int nodeNum1, int nodeNum2, float edgeScore);
	void ConstructSearchTree();//get all possible search paths
	void ConstructSearchPath();//get one search path that has all nodes
	float GetNodeScore(int nodeNum);
	float GetEdgeScore(int nodeNum1, int nodeNum2) {return GetEdgeScore(edge(nodeNum1, nodeNum2));}
	float GetEdgeScore(edge e);
	//tNode* ConstructSearchTree();
	
	friend std::ostream& operator<< (std::ostream& stream, PrimSubGraph& graph);
	friend std::istream& operator>> (std::istream& stream, PrimSubGraph& graph);
private:
	std::vector<int> SearchAdjNodes(int curNode);
	float NodeDistance(int curNode, PrimSubGraph& refGraph, int refNode);
	float MinElement(const std::vector<char>& valid, 
		const std::vector<float>& suduku, int& pos);
private:
	std::map<int, float> m_NodeScores;//node number and score
	std::vector<int> m_NodeNums;
	std::map<edge, float> m_EdgeScores;
	std::vector<std::vector<int> > m_SearchTree;
};

class IEDll_API GraphDiction
{
public:
	GraphDiction(){};

public:
	void Read(const char* filepath);
	void Read(const std::string& filepath) {Read(filepath.c_str());}
	void Write(const char* filepath);
	void Write(const std::string& filepath){Write(filepath.c_str());}
	void Learn(char* srtOLaser, char* srtOMapTop, char* strOMapPnt,
		char* strILaser, char* strIMapTop, char* srtIMapPnt);
	int LookUp(PrimSubGraph& indGraph,std::vector<int>& vecMatchNodes,float threshold = 0.8);
	void CorrectGraph(char* srtLaser, char* srtMapTop, char* strMapPnt);
	bool CorrectGraph(LaserPoints& gLasPnts, std::vector<Plane>& vecLocalPlanes,
		std::vector<PointNumberList>& gPnlSegs, Building& bld, ObjectPoints& modelObjPnts);
	void CorrectGraph(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
		std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
	friend std::ostream& operator<<(std::ostream& stream, GraphDiction& dic);
	friend std::istream& operator>>(std::istream& stream, GraphDiction& dic);

private:
	void CorrectGraph(const std::vector<int>& vecMatchNodes, int index, ObjectPoints& modelObjPnts, LineTopologies& lineTop);
	int DataGraphNode(const std::vector<int>& vecMatchNodes, int modelNode);
private:
	std::vector<PrimSubGraph> m_Entries;
	std::vector<std::vector<PrimSubGraph> > m_Explains;
};

#endif