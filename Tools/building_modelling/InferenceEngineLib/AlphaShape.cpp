/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : March 23, 2014
Description: 
Revise:
----------------------------------------------------------*/
#include "LaserPoints.h"
#include "TINEdges.h"
#include "TINEdgeSet.h"
#include "LineTopology.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "PointNumber.h"
#include "PointNumberList.h"
#include "Plane.h"
#include "Vector3D.h"
#include "Position3D.h"
#include "TINEdges.h"
#include "Vector3D.h"
#include "HoughSpace.h"
#include "Positions3D.h"
#include"ObjectPoint2D.h"
#include "ObjectPoints2D.h"
#include "InferenceEngine.h"

//#include <Eigen/dense>
#include <vector>
#include <math.h>
#include<algorithm>


//two centers exist, use isFirst tag to get first center or second one
inline void ComputeCircleCenter(double x1, double y1, double x2, double y2, 
								Position2D& center1, Position2D& center2, double alpha)
{
	double difx, dify, dist, w;
	difx = x1-x2;
	dify = y1-y2;
	dist = sqrt(difx*difx + dify*dify);
	center1.X() = center2.X() = (x1 + x2)/2;
	center1.Y() = center2.Y() = (y1 + y2)/2;

	//the two point is too near, use their middle point as center
	if (dist < 0.000001) 	return;
	w = sqrt((alpha*alpha)/(dist*dist) - 0.25);

	center1.X() += w*dify;
	center1.Y() -= w*difx;

	center2.X() -= w*dify;
	center2.Y() += w*difx;

	double normal = w * sqrt(difx*difx+dify*dify);
	double normal2 = sqrt(alpha*alpha-0.25*(dist*dist));

	double dis1 = sqrt((center1.X()-x1)*(center1.X()-x1) + (center1.Y()-y1)*(center1.Y()-y1));
	double dis2 = sqrt((center1.X()-x2)*(center1.X()-x2) + (center1.Y()-y2)*(center1.Y()-y2));
	double dis3 = sqrt((center2.X()-x1)*(center2.X()-x1) + (center2.Y()-y1)*(center2.Y()-y1));
	double dis4 = sqrt((center2.X()-x2)*(center2.X()-x2) + (center2.Y()-y2)*(center2.Y()-y2));
}

//two centers exist, use isFirst tag to get first center or second one
inline void ComputeSphareCenter(const Vector3D& p1, const Vector3D& p2, 
								const Vector3D& p3, Vector3D& c1, Vector3D& c2, double alpha)
{
	Vector3D dp1 = p2 - p1;
	Vector3D dp2 = p3 - p1;
	Vector3D mid1 = (p2+p1)/2.0;
	Vector3D mid2 = (p3+p1)/2.0;

	//s = [ -dp1(2), dp2(2); dp1(1), -dp2(1) ] \ [ -mid1 + mid2 ]';

	//	cpc = mid1 + s(1) * [ -dp1(2), dp1(1) ];
	//cr = norm ( p(ct(1),:) - cpc );

	//pc(it,:) = cpc;
	//r(it,1) = cr;
}

//use alpha shape to detect contour points, which is in order
//the points are projected to its plane, and treated as two dimensional points
IEDll_API bool DeriveContour_AlphaShape(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
										PointNumberList& contour, const double factor, const Plane* pSegPlane)
{
	contour.clear();

	//if there are only 3 node, just return them
	if (pnlSeg.size() == 3) {
		contour.push_back(pnlSeg[0].Number());
		contour.push_back(pnlSeg[1].Number());
		contour.push_back(pnlSeg[2].Number());
		contour.push_back(pnlSeg[0].Number());
		return true;
	}
	else if (pnlSeg.size()<3) 
		return false;

	//fit plane 
	//	int segNumber = gLaserPnts[pnlSeg[0].Number()].Attribute(SegmentNumberTag);
	Plane plane; 
	Vector3D refDir;
	if (pSegPlane) plane = *pSegPlane;
	else plane = gLaserPnts.FitPlane(pnlSeg, 0);
	refDir = Vector3D(0,1,0);

	//transform points to reference plane
	//initialize a local laser points and TIN
	PointNumberList localPnl;//(pnlSeg.size(), PointNumber(-1));
	LaserPoints localPoints;
	LaserPoint temLaserPnt;
	for (int i=0; i< (int)pnlSeg.size(); ++i)	
		localPnl.push_back(PointNumber(i));

	Position3D center;
	Project2RefPlane(gLaserPnts, pnlSeg, plane, center, localPoints);

	localPoints.DeriveTIN();
	TINEdges tinEdges; 
	tinEdges.Derive(localPoints.TINReference());
	double meanEdgeLen = localPoints.MedianInterPointDistance((int)min(1000.0, (double)localPoints.size()-2));
	meanEdgeLen = factor*meanEdgeLen;
	//		meanEdgeLen =1.0;
	double alpha = meanEdgeLen;

	vector<char> vecIsValidTag(pnlSeg.size(), true);
	Position2D curPos2D, temPos2D;
	TINEdges::iterator iEdgeSet;
	TINEdgeSet::iterator iEdge;

	vector<int> vecLNodes;
	vector<int> vecRNodes;
	vector<char> vecValidEdge;

	for (int iPnt=0; iPnt < pnlSeg.size(); ++iPnt) {
		iEdgeSet = tinEdges.begin()+iPnt;
		for (iEdge = iEdgeSet->begin(); iEdge != iEdgeSet->end(); ++iEdge)	{
			if (iEdge->Number()<iPnt) 
				continue;//only record once for a edge
			vecLNodes.push_back(iPnt);
			vecRNodes.push_back(iEdge->Number());
		}
	}
	vecValidEdge = vector<char>(vecRNodes.size(), true);

	///////////////////////////////////////////////////////////////////////////////////////
	//search  all edges on hull
	int lNode, rNode;
	TINEdges::iterator lEdgeSet, rEdgeSet;
	Position2D lCenter, rCenter;
	bool bLEmpty, bREmpty;//has point in left/right circle or not
	for (int i=0; i<vecLNodes.size(); ++i) {
		lNode = vecLNodes[i];
		rNode = vecRNodes[i];
		lEdgeSet = tinEdges.begin() + lNode;
		rEdgeSet = tinEdges.begin() + rNode;
		bLEmpty = bREmpty = true;
		ComputeCircleCenter(localPoints[lNode].X(), localPoints[lNode].Y(), 
			localPoints[rNode].X(), localPoints[rNode].Y(), lCenter, rCenter, alpha);

		for (iEdge = lEdgeSet->begin(); iEdge != lEdgeSet->end(); ++iEdge) {
			if (iEdge->Number() == rNode) continue;//right node
			if (localPoints[iEdge->Number()].Position2DOnly().Distance(lCenter) < alpha)
				bLEmpty = false;
			if (localPoints[iEdge->Number()].Position2DOnly().Distance(rCenter) < alpha)
				bREmpty = false;
			if (!bLEmpty && !bREmpty)
				goto StopFlag;
		}

		for (iEdge = rEdgeSet->begin(); iEdge != rEdgeSet->end(); ++iEdge) {
			if (iEdge->Number() == lNode) continue;//left node
			if (localPoints[iEdge->Number()].Position2DOnly().Distance(lCenter) < alpha)
				bLEmpty = false;
			if (localPoints[iEdge->Number()].Position2DOnly().Distance(rCenter) < alpha)
				bREmpty = false;
			if (!bLEmpty && !bREmpty)
				goto StopFlag;
		}

StopFlag:
		if (!bLEmpty && !bREmpty )
			vecValidEdge[i] = false;
	}

	//detect too long edge
	double temEdgeLen;
	LaserPoints::iterator itr1, itr2;
	for (int i=vecValidEdge.size()-1; i >= 0; --i ) {
		if (!vecValidEdge[i]) continue;

		itr1 = localPoints.begin() + vecLNodes[i];
		itr2 = localPoints.begin() + vecRNodes[i] ;
		temEdgeLen = itr1->Position2DOnly().Distance(itr2->Position2DOnly());

		if (temEdgeLen> 2 * alpha)
			vecValidEdge[i] = false;
	}

	//remove edges inside hull
	for (int i=vecValidEdge.size()-1; i >= 0; --i ) {
		if (!vecValidEdge[i]) {
			vecValidEdge.erase(vecValidEdge.begin()+i);
			vecLNodes.erase(vecLNodes.begin()+i);
			vecRNodes.erase(vecRNodes.begin()+i);
		}
	}


	vector<PointNumberList> contours;
	PointNumberList temContour;
	int curNode;

	temContour.clear();
	for (int i=0; i<vecValidEdge.size(); ++i) {
		if (!vecValidEdge[i]) continue;

		int curInd = i;
		temContour.clear();
		temContour.push_back(vecLNodes[i]);
		bool bFinish = false;
		bool bFind = false;

		while(!bFinish) {
			temContour.push_back(vecRNodes[curInd]);
			curNode = vecRNodes[curInd];
			vecValidEdge[curInd] = false;

			//search next node
			bFind = false;
			for (int j=i+1; j<vecValidEdge.size(); ++j) {
				if (!vecValidEdge[j]) continue;

				if (vecLNodes[j] == curNode) {
					bFind = true;
					curInd = j;
					break;
				}
				else if (vecRNodes[j] == curNode) {
					bFind = true;
					curInd = j;
					std::swap(vecRNodes[j], vecLNodes[j]);
					break;
				}
			}

			//finish search
			if (!bFind) bFinish = true;
		}

		if (bFinish)
			contours.push_back(temContour);
	}

	int maxContourPnt=0, indMaxContour;
	for (int i=0; i<contours.size(); i++) {
		if (contours[i].size() > maxContourPnt)	{
			maxContourPnt = contours[i].size();
			indMaxContour = i;
		}
	}
	contour = contours[indMaxContour];

	//transform to original number
	for (int i=0; i<contour.size(); ++i) {
		contour[i] = pnlSeg[ contour[i].Number() ];
	}
}

bool LessPointNumber(const PointNumber& lhv, const PointNumber& rhv) 
{
	return lhv.Number()<rhv.Number();
}

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
//CGAL::Alpha_shape_vertex_base_2
//#include "Alpha_shape_vertex_base_2_WithIndex.h"
#include "KNNFinder.h"

IEDll_API bool DeriveContour_AlphaShape2D(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
	 PointNumberList& contour, double inAlpha, bool bAdaptiveAlpha)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef K::FT FT;
	typedef K::Point_2  Point;
	typedef K::Segment_2  Segment;
	//typedef CGAL::Alpha_shape_vertex_base_2_with_index<K> Vb;
	typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
	typedef CGAL::Alpha_shape_face_base_2<K> Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb,Fb> Tds;
	typedef CGAL::Delaunay_triangulation_2<K,Tds> Triangulation_2;
	typedef CGAL::Alpha_shape_2<Triangulation_2>  Alpha_shape_2;
	typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;
	typedef Alpha_shape_2::Alpha_shape_vertices_iterator Alpha_shape_vertices_iterator;
	typedef Alpha_shape_2::Alpha_iterator Alpha_iterator;
	typedef Alpha_shape_2::Vertex_handle Vertex_handle;
	typedef Alpha_shape_2::Edge edge;

	if (pnlSeg.size()<3) return false;
	std::vector<Point> vecPnts;
	vecPnts.reserve(pnlSeg.size());
	//Point p;
	LaserPoints::const_iterator itrPnt;
	for (int it=0; it<pnlSeg.size(); it++) {
		itrPnt = gLaserPnts.begin()+pnlSeg[it].Number();
		vecPnts.push_back(Point(itrPnt->X(),itrPnt->Y()));
	}
	KNNFinder<Point> kdtree(vecPnts, 2);

	//////////////////////////////////////////////////////////////////////////
	//compute alpha shape
	Alpha_shape_2 as(vecPnts.begin(),vecPnts.end());
	//unsigned int index=0;
	//for (Alpha_shape_2::Vertex_iterator v=as.vertices_begin(); v!=as.vertices_end(); v++)
		//v->index = index++;
	//	v->set_index(index++);
	
	as.set_mode(Alpha_shape_2::REGULARIZED);
	Alpha_iterator opt = as.find_optimal_alpha(1);
	double alpha_value = *opt;

	if (!bAdaptiveAlpha && inAlpha>3*alpha_value)
		as.set_alpha(inAlpha);
	else
		as.set_alpha(3*alpha_value);

//	as.set_mode(Alpha_shape_2::GENERAL);

	assert(as.number_of_solid_components() == 1);

	//////////////////////////////////////////////////////////////////////////
	//get alpha_shape vertexes
	std::map<edge, bool> mapHullEdge;
	for(Alpha_shape_edges_iterator it=as.alpha_shape_edges_begin();
		it != as.alpha_shape_edges_end();
		++it) {
			mapHullEdge.insert(make_pair(*it, true));
	}


	//////////////////////////////////////////////////////////////////////////
	//get sequent vertexes
	//may have several polygons
	std::vector< std::vector<Vertex_handle> > vecSeqVertexs;
	std::vector<Vertex_handle> seqVertexs;
	std::map<edge, bool>::iterator itr0, itr1;
	for (itr0=mapHullEdge.begin(); itr0!=mapHullEdge.end(); itr0++) {
		if (itr0->second == false) continue;

		seqVertexs.clear();
		Vertex_handle vertStart, vertNext, vertCur;

		edge curEdge = itr0->first;
		vertStart = curEdge.first->vertex(curEdge.first->ccw(curEdge.second));
		vertNext = curEdge.first->vertex(curEdge.first->cw(curEdge.second));
		seqVertexs.push_back(vertStart);
		seqVertexs.push_back(vertNext);
		itr0->second = false;

		int nLoop = 0;
		while (vertStart != vertNext) {
			if (nLoop++>10000) break;

			Vertex_handle vert0, vert1;
			vertCur = seqVertexs[seqVertexs.size()-1];
			vertNext = vertCur;

			for (itr1=mapHullEdge.begin(); itr1!=mapHullEdge.end(); itr1++) {
				if (itr1->second == false) continue;
				curEdge = itr1->first;
				vert0 = curEdge.first->vertex(curEdge.first->ccw(curEdge.second));
				vert1 = curEdge.first->vertex(curEdge.first->cw(curEdge.second));

				if (vertCur==vert0)	vertNext = vert1;
				else if (vertCur==vert1) vertNext = vert0;

				if (vertNext!=vertCur) {
					itr1->second = false;
					seqVertexs.push_back(vertNext);
					break;
				}
			}

			//cannot find next vertex
			if(vertNext==vertCur) break;
		}

		if (vertStart==vertNext)//closed
			vecSeqVertexs.push_back(seqVertexs);
	}

	//////////////////////////////////////////////////////////////////////////
	//split self-cross polygon
	for (unsigned int i=0; i<vecSeqVertexs.size(); i++) {
		std::vector<Vertex_handle>& curPolygon = vecSeqVertexs[i];
		curPolygon.erase(curPolygon.end()-1);//
		std::vector<std::vector<Vertex_handle> > subPolygons;
		std::vector<Vertex_handle>::iterator curVer, crossVert;


		for (unsigned int j=0; j<curPolygon.size(); j++) {
			curVer = curPolygon.begin()+j;
			crossVert = curPolygon.end();

			for (unsigned int k=j+1; k<curPolygon.size(); k++) {
				if (curPolygon[k]->point() != (*curVer)->point()) continue;
				crossVert = curPolygon.begin()+k;
				break;
			}

			if (crossVert != curPolygon.end()) {
				std::vector<Vertex_handle> newSubPol = std::vector<Vertex_handle>(curVer, crossVert);
				newSubPol.push_back(newSubPol[0]);
				subPolygons.push_back(newSubPol);
				curPolygon.erase(curVer, crossVert);
			}
		}

		curPolygon.push_back(curPolygon[0]);
		if (!subPolygons.empty())
			vecSeqVertexs.insert(vecSeqVertexs.end(), subPolygons.begin(), subPolygons.end());
	}

	//////////////////////////////////////////////////////////////////////////
	//only keep the exterior polygon, which has the largest number of points
	assert(!vecSeqVertexs.empty());
	if (vecSeqVertexs.empty()) return false;

	seqVertexs = vecSeqVertexs[0];
	for (unsigned int i=1; i<vecSeqVertexs.size(); i++) {
		if (seqVertexs.size()<vecSeqVertexs[i].size())
			seqVertexs = vecSeqVertexs[i];
	}

	//////////////////////////////////////////////////////////////////////////
	//back to global point number list
	
	contour.clear();
	std::vector<Point>::iterator itrVecPnt;
	for(std::vector<Vertex_handle>::iterator itr=seqVertexs.begin();
		itr!=seqVertexs.end(); ++itr) 
	{
		//Vertex_handle temVH = as.vertices_begin();
		//index = (*itr)->get_index();
		//while (temVH++!=*itr)
		//	index++;
		int index = kdtree.FindIndex((*itr)->point());
		contour.push_back(pnlSeg[index]);
		//itrVecPnt = std::find(vecPnts.begin(), vecPnts.end(), (*itr)->point());
		//if (itrVecPnt==vecPnts.end()) continue;
		//contour.push_back(pnlSeg[itrVecPnt-vecPnts.begin()]);
	}

	//////////////////////////////////////////////////////////////////////////
	//make sure the first point has minimum point number
	std::vector<int> temContour0, temContour1;
	for (unsigned int i=0; i<contour.size()-1; i++) 
		temContour0.push_back(contour[i].Number());

	std::vector<int>::iterator itrMinNum;
	itrMinNum = std::min_element(temContour0.begin(), temContour0.end());
	if (itrMinNum!=temContour0.begin()) {
		temContour1 = std::vector<int>(itrMinNum, temContour0.end());
		temContour1.insert(temContour1.end(), temContour0.begin(), itrMinNum);

		contour.clear();
		for (unsigned int i=0; i<temContour1.size(); i++) 
			contour.push_back(PointNumber(temContour1[i]));
		contour.push_back(contour[0]);
	}

	
	return true;
}

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Alpha_shape_3.h>

#include <fstream>
#include <list>
#include <cassert>


IEDll_API bool DeriveContour_AlphaShape3D(LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
										  PointNumberList& contour, const double factor)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Gt;
	typedef CGAL::Alpha_shape_vertex_base_3<Gt>          Vb;
	typedef CGAL::Alpha_shape_cell_base_3<Gt>            Fb;
	typedef CGAL::Triangulation_data_structure_3<Vb,Fb>  Tds;
	typedef CGAL::Delaunay_triangulation_3<Gt,Tds>       Triangulation_3;
	typedef CGAL::Alpha_shape_3<Triangulation_3>         Alpha_shape_3;
	typedef Gt::Point_3                                  Point;
	typedef Gt::Tetrahedron_3 Tetrahedron;
	typedef Alpha_shape_3::Alpha_iterator                Alpha_iterator;
	typedef Alpha_shape_3::Cell_handle                  Cell_handle;
	typedef Alpha_shape_3::Edge                          Edge;
	typedef Alpha_shape_3::Facet                         Facet;
	typedef Alpha_shape_3::Vertex_handle                 Vertex_handle;
	typedef Alpha_shape_3::Cell                          Cell;

	//read input
	gLaserPnts.SetAttribute(LabelTag, 0);
	std::list<Point> lp;
	Point p;
	LaserPoints::const_iterator itrPnt;
	for (int it=0; it<pnlSeg.size(); it++) {
		itrPnt = gLaserPnts.begin()+pnlSeg[it].Number();
		lp.push_back(Point(itrPnt->X(),itrPnt->Y(),itrPnt->Z()));
	}

	// compute alpha shape
	Alpha_shape_3 as(lp.begin(),lp.end());
	std::cout << "Alpha shape computed in REGULARIZED mode by default"<< std::endl;

	// find optimal alpha value
	Alpha_iterator opt = as.find_optimal_alpha(1);
	std::cout << "Optimal alpha value to get one connected component is "<<*opt<< std::endl;
	as.set_alpha(*opt);
	//as.set_alpha(0.5);
	assert(as.number_of_solid_components() == 1);

	std::list<Edge> edges;
	std::list<Facet> facets;
	std::list<Vertex_handle> vertexs;
	//as.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape_3::SINGULAR);
	//as.get_alpha_shape_edges(std::back_inserter(edges),	Alpha_shape_3::REGULAR);
	as.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape_3::EXTERIOR);
	//as.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape_3::INTERIOR);

	//as.get_alpha_shape_vertices(std::back_inserter(vertexs), Alpha_shape_3::SINGULAR);
	as.get_alpha_shape_vertices(std::back_inserter(vertexs), Alpha_shape_3::REGULAR);
	//as.get_alpha_shape_vertices(std::back_inserter(vertexs), Alpha_shape_3::EXTERIOR);
	//as.get_alpha_shape_vertices(std::back_inserter(vertexs), Alpha_shape_3::INTERIOR);

	as.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape_3::EXTERIOR);

	//////////////////////////////////////////////////////////////////////////
	//calculate the volume
	std::list<Cell_handle> cells;
	as.get_alpha_shape_cells(std::back_inserter(cells),  Alpha_shape_3::INTERIOR);
	Tetrahedron tetr;
	double totVol = 0.0;
	std::list<Cell_handle>::iterator itrCell;
	for (itrCell = cells.begin(); itrCell!=cells.end(); itrCell++) {
		tetr = as.tetrahedron(*itrCell) ;
		totVol += tetr.volume();
	}
	int aa = 0;


	return 0;
}



