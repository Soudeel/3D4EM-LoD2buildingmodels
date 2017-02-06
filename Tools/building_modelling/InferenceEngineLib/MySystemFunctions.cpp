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
#include "LineSegments2D.h"

#include "InferenceEngine.h"
#include <vector>
#include <math.h>
#include <Eigen\Dense>
#include <map>

using namespace Eigen;
using namespace std;

IEDll_API bool MyFitLine(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, Line2D& line)
{
	Line3D line3d;
	bool bFlag;
	Position2D p1, p2;
	double residual = 0;

	bFlag = MyFitLine(laserPnts, pnlLineSeg, line3d);
	p1 = Position2D(line3d.FootPoint().X(), line3d.FootPoint().Y());
	p2 = Position2D(line3d.Direction().X(), line3d.Direction().Y());
	p2.X() = 1000.0*p2.X() + p1.X(); 
	p2.Y() = 1000.0*p2.Y() + p1.Y();

	line = Line2D(p1,p2);

	return bFlag;
} 

IEDll_API bool MyFitLine(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, Line3D& line)
{
	Vector3D centre, temPnt, normal;
	double tmp1[3],  eigen_vec[3][3];
	double moments[9], central_moments[9];
	int      dim=3, matz=1;
	int numPnts = pnlLineSeg.size();

	// Check number of points
	if (numPnts < 3) return false;

	// Centre of gravity of all points
	for (int i=0; i<pnlLineSeg.size(); i++)
		centre += laserPnts[pnlLineSeg[i].Number()].Position3DRef();
	centre /= (double)numPnts;

	memset(moments, 0, 9*sizeof(double));
	
	for (int i=0; i<pnlLineSeg.size(); i++)
	{
		temPnt = laserPnts[pnlLineSeg[i].Number()].Position3DRef();
		for (int j=0; j<3; j++)
			for (int k=0; k<=j; k++)
				moments[j*3+k] += (temPnt[j]-centre[j])*(temPnt[k]-centre[k]);
	}

	// Derive centralised moments
	for (int i=0; i<3; i++)
		for (int j=0; j<=i; j++)
			central_moments[i*3+j] = moments[i*3+j]/(numPnts-1);

	// Mirror moments array to make it full
	central_moments[1] = central_moments[3];
	central_moments[2] = central_moments[6];
	central_moments[5] = central_moments[7]; 

	Matrix3d A;
	A(0,0)=central_moments[0];A(0,1)=central_moments[1];A(0,2)=central_moments[2];
	A(1,0)=central_moments[3];A(1,1)=central_moments[4];A(1,2)=central_moments[5];
	A(2,0)=central_moments[6];A(2,1)=central_moments[7];A(2,2)=central_moments[8];
	Matrix3d V;
	Vector3d D;
	SelfAdjointEigenSolver<Matrix3d> solver(A);
	D = solver.eigenvalues();
	V = solver.eigenvectors();


	normal.X() = V(2,0);
	normal.Y() = V(2,1);
	normal.Z() = V(2,2);
	normal = normal.Normalize();

	line = Line3D(Position3D(centre), normal);
	//line.FootPoint() = centre;
	//line.Direction() = normal;

	return true;
}


IEDll_API Position2D Project2RefPlane(const Plane& refPlane, const Position3D& inPos)
{
	Vector3D dir = refPlane.Normal();
	//double product = dir.DotProduct(Vector3D(0,0,1));
	double angle = MyAngle(refPlane.Normal(), Vector3D(0, 0, 1));
	Position2D outPs;

	//if(fabs(product) < 10*3.1415/180){//vertical
	if(angle > 80*3.1415/180){//vertical
		if (fabs(dir.X())>fabs(dir.Y())) {//on y-z plane
			outPs.X() = inPos.Z(); outPs.Y() = inPos.Y();
		}
		else {//on x-z plane
			outPs.X() = inPos.X(); outPs.Y() = inPos.Z();
		}
	}
	else {
		outPs.X() = inPos.X(); outPs.Y() = inPos.Y();
	}
	/*if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
	{
		outPs.X() = inPos.X(); outPs.Y() = inPos.Y();
	}
	else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
	{
		outPs.X() = inPos.Z(); outPs.Y() = inPos.Y();
	}
	else//on x-z plane
	{
		outPs.X() = inPos.X(); outPs.Y() = inPos.Z();
	}*/

	return outPs;
}

//old one
//transform back to 3d coordinate system to xy, xz, or yz reference plane coordinate system
IEDll_API Position3D Repoject2OrgPlane(const Plane& refPlane, const Position2D& inPos)
{
	//re-transform to original plane
	Line3D temLine;
	Position3D pos3D;
	Vector3D temDir;
	Vector3D dir = refPlane.Normal();
	//double product = dir.DotProduct(Vector3D(0,0,1));
	double angle = MyAngle(refPlane.Normal(), Vector3D(0, 0, 1));

	if(angle > 80*3.1415/180){//vertical
		if (fabs(dir.X())>fabs(dir.Y())) {//on y-z plane
			pos3D.X() = 0.0; pos3D.Y() = inPos.Y(); pos3D.Z() = inPos.X();
			temDir = Vector3D(1,0,0);
		}
		else {//on x-z plane
			pos3D.X() = inPos.X(); pos3D.Y() = 0.0; pos3D.Z() = inPos.Y();
			temDir = Vector3D(0,1,0);
		}
	}
	else {
		pos3D.X() = inPos.X(); pos3D.Y() = inPos.Y(); pos3D.Z() = 0.0;
		temDir = Vector3D(0,0,1);
	}

	/*if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
	{
		pos3D.X() = inPos.X(); pos3D.Y() = inPos.Y(); pos3D.Z() = 0.0;
		temDir = Vector3D(0,0,1);
	}
	else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
	{
		pos3D.X() = 0.0; pos3D.Y() = inPos.Y(); pos3D.Z() = inPos.X();
		temDir = Vector3D(1,0,0);
	}
	else//on x-z plane
	{
		pos3D.X() = inPos.X(); pos3D.Y() = 0.0; pos3D.Z() = inPos.Y();
		temDir = Vector3D(0,1,0);
	}*/

	temLine = Line3D(pos3D,temDir);
	IntersectLine3DPlane(temLine, refPlane, pos3D);

	return pos3D;
}

//new one
//re-project to reference plane, from the x-y plane back to reference plane
IEDll_API void Project2RefPlane(const LaserPoints& inLaserPnts, const PointNumberList& pnlSeg, 
	const Plane& refPlane, Position3D& center, LaserPoints& outPnts)
{
	outPnts.clear();

	Position3D temPnt;
	Vector3D plumb = Vector3D(0.0, 0.0, 1.0);
	double rotAngle = MyAngle(plumb, refPlane.Normal());
	center = inLaserPnts.CentreOfGravity(pnlSeg);
	//do not need to rotate
	if (rotAngle==0.0) {
		for (int i=0; i<pnlSeg.size(); i++) {
			outPnts.push_back(inLaserPnts[pnlSeg[i].Number()].Position3DRef());
			//outPnts[i].Z() = 0;//?? why donot set to be zero
			outPnts[i] -= center;
			outPnts[i].Z() = 0;
		}

		return;
	}

	Vector3D rotAxle = refPlane.Normal().VectorProduct(plumb);
	rotAxle = rotAxle.Normalize();
	Rotation3D rot =Rotation3D(rotAxle, rotAngle);
	Position3D proCenter = rot.Rotate(center);

	for (int i=0; i<pnlSeg.size(); i++) {
//		temPnt = rot.Rotate(inLaserPnts[pnlSeg[i].Number()].Position3DRef());
		temPnt = rot.Rotate(inLaserPnts[pnlSeg[i].Number()].Position3DRef()-center);
		temPnt.Z() = 0.0;
		outPnts.push_back(temPnt);
	}

//	PointNumberList pnl2;
//	for (int i=0; i<pnlSeg.size(); i++)
//		pnl2.push_back(PointNumber(i));
//	Position3D center2 = outPnts.CentreOfGravity(pnl2);
}

//new one
//project to reference plane, the reference plane becomes the x-y plane
IEDll_API void ReProject2OrgPlane (const LaserPoints& inPnts, const Plane& refPlane, 
	const Position3D& center, LaserPoints& outPnts)
{
	outPnts.clear();

	Position3D temPnt;
	Vector3D plumb = Vector3D(0.0, 0.0, 1.0);
	double rotAngle = MyAngle(plumb, refPlane.Normal());

	//do not need to rotate
	if (rotAngle==0.0) {
		outPnts = inPnts;
		for (int i=0; i<inPnts.size(); i++) 
			outPnts[i] += center;
		
		return;
	}

	Vector3D rotAxle = refPlane.Normal().VectorProduct(plumb);
	rotAxle = rotAxle.Normalize();
	Rotation3D rot =Rotation3D(rotAxle, -rotAngle);

	for (int i=0; i<inPnts.size(); i++) {
		temPnt = rot.Rotate(inPnts[i].Position3DRef());
		temPnt += center;
		outPnts.push_back(temPnt);
	}
}

#include "Eigen/Dense"
using namespace Eigen;
#include "KNNFinder.h"

IEDll_API void MyComputeNormal(const LaserPoints& lsPnts, vector<Vector3D>& normals, int nNeibPnts)
{
	normals.resize(lsPnts.size());

	vector<Vector3D>::iterator normal;
	KNNFinder <LaserPoint> kdTree(lsPnts, 3);
	vector<int>            neigInds;
	TINEdges::iterator     edgeset;
	int                    num_nb = nNeibPnts;
	Vector3D mean;
	double divX, divY, divZ;
	Eigen::Matrix3d A;
	Eigen::Matrix3d eigenVec;
	Eigen::Vector3d eigenVal;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver;

	//compute normal for each point
	for (int i=0; i<(int) lsPnts.size(); ++i) {
		normal = normals.begin()+i;
		neigInds = kdTree.FindIndices(lsPnts[i], num_nb);
		A = Eigen::Matrix3d::Zero();

		//computer mean value of point coordinates
		mean.X() = 0; mean.Y() = 0; mean.Z() = 0;
		for(int j=0;j<neigInds.size();++j) 
			mean += lsPnts[neigInds[j]].Position3DRef();
		mean /= neigInds.size();

		for(int j=0;j<neigInds.size();++j) {
			divX = lsPnts[neigInds[j]].X()-mean.X();
			divY = lsPnts[neigInds[j]].Y()-mean.Y();
			divZ = lsPnts[neigInds[j]].Z()-mean.Z();

			A(0,0) += divX*divX; A(0,1) += divX*divY; A(0,2) += divX*divZ;
			A(1,1) += divY*divY; A(1,2) += divY*divZ; A(2,2) += divZ*divZ;
		}
		A(1,0) = A(0,1); A(2,0) = A(0,2); A(2,1) = A(1,2);

		eigenSolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(A);
		eigenVal = eigenSolver.eigenvalues();
		eigenVec = eigenSolver.eigenvectors();

		normal->X() = eigenVec.col(0)(0);
		normal->Y() = eigenVec.col(0)(1);
		normal->Z() = eigenVec.col(0)(2);
		normal->Normalize();
	}

	//if(pResidual) *pResidual = eigenVal(0);
}


IEDll_API bool MyDeriveSegPNL(const LaserPoints& laserPnts, vector<PointNumberList>& vecSegPnls, LaserPointTag tag) 
{
	//LaserPoints debugLasPnts;
	vecSegPnls.clear();
	if (laserPnts.empty()) return false;
	int segNum, maxSegNum = -999;
	for (int i=0; i<laserPnts.size(); ++i){
		if ( !laserPnts[i].HasAttribute(tag))
			continue;
		segNum = laserPnts[i].Attribute(tag);
		if(segNum > maxSegNum)
			maxSegNum = segNum;

	//	if (segNum>100000)
	//		debugLasPnts.push_back(laserPnts[i]);
	}
	//if (!debugLasPnts.empty())
	//{
	//	debugLasPnts.Write("e:\\debuglaserpoints.laser");
	//}

	if (maxSegNum<0) return false;
	vector<PointNumberList>* pTemVecSegPnls = new vector<PointNumberList>(maxSegNum+1);
	for (int i=0; i<laserPnts.size(); ++i){
		if ( !laserPnts[i].HasAttribute(tag))
			continue;
		segNum = laserPnts[i].Attribute(tag);
		if (segNum<0) continue;
		pTemVecSegPnls->at(segNum).push_back(PointNumber(i));
	}

	vecSegPnls.clear();
	vecSegPnls.reserve(maxSegNum/4);
	int size;
	for (int i=0; i<pTemVecSegPnls->size(); ++i) {
		if (!pTemVecSegPnls->at(i).empty()) {
			//size = (*pTemVecSegPnls)[i].size();
			//segNum = laserPnts[(*pTemVecSegPnls)[i][0].Number()].Attribute(SegmentNumberTag);
			vecSegPnls.push_back(pTemVecSegPnls->at(i));
		}
	}
	delete pTemVecSegPnls;//force to release memory

	return true;
}

/*
IEDll_API int MyCleanRedupPnts( LaserPoints& laserPnts, int dimention)
{
	assert(dimention==2||dimention==3);
	TINEdges* edges;
	SegmentationParameters parameters;
	parameters.NeighbourhoodStorageModel() = 2; //kd-tree
	parameters.NumberOfNeighbours() = 50;
	edges = laserPnts.DeriveEdges(parameters);
	TINEdges::iterator iEdgeSet;
	TINEdgeSet::iterator iEdge;
	vector<char> vecIsRepeat(laserPnts.size(), false);
	LaserPoints::iterator curPnt, temPnt;
	double curX, curY, curZ;
	int repeatPntInd, redupPntNum;

	//search redundant point
	if (dimention==3) {
		for (int iPnt=0; iPnt<laserPnts.size(); ++iPnt) {
			if (vecIsRepeat[iPnt]) 
				continue;//this point or its repeat point is already stored;

			curPnt = laserPnts.begin() + iPnt;
			iEdgeSet = edges->begin()+iPnt;

			repeatPntInd = -1;
			for (iEdge=iEdgeSet->begin(); iEdge!=iEdgeSet->end(); ++iEdge) {
				temPnt = laserPnts.begin() + iEdge->Number();
				if (temPnt == curPnt) continue; //skip current point
				if (curPnt->Position3DRef() == temPnt->Position3DRef()) {//repeat point
					repeatPntInd = iEdge->Number();
					vecIsRepeat[repeatPntInd] = true;	 //repeat point
				}
			}
		}
	}
	else if (dimention==2) {
		for (int iPnt=0; iPnt<laserPnts.size(); ++iPnt) {
			if (vecIsRepeat[iPnt]) 
				continue;//this point or its repeat point is already stored;

			curPnt = laserPnts.begin() + iPnt;
			iEdgeSet = edges->begin()+iPnt;

			repeatPntInd = -1;
			for (iEdge=iEdgeSet->begin(); iEdge!=iEdgeSet->end(); ++iEdge) {
				temPnt = laserPnts.begin() + iEdge->Number();
				if (temPnt == curPnt) continue; //skip current point
				if (curPnt->Position2DOnly() == temPnt->Position2DOnly()) {//repeat point
					repeatPntInd = iEdge->Number();
					vecIsRepeat[repeatPntInd] = true;	 //repeat point
				}
			}
		}
	}
	

	//remove redundant point
	int nValidPnts = 0;
	int nUnValidPnt = 0;
	LaserPoints::iterator pnt1, pnt2;
	for (int i=0; i<laserPnts.size()-nUnValidPnt; ++i) {
		pnt1 = laserPnts.begin()+i;
		if (!vecIsRepeat[i]) {
			nValidPnts ++;
			continue;
		}
		else {
			for (int j=laserPnts.size()-1-nUnValidPnt; j>=nValidPnts; --j) {
				pnt2 = laserPnts.begin()+j;
				nUnValidPnt++;
				if (!vecIsRepeat[j]) {
					std::swap(*pnt1, *pnt2);
					std::swap(vecIsRepeat[i], vecIsRepeat[j]);
					nValidPnts ++;
					break;
				}
			}
		}
	}

	laserPnts.erase(laserPnts.begin()+nValidPnts, laserPnts.end());;

	return nUnValidPnt;
}*/

IEDll_API int MyCleanRedupPnts( LaserPoints& laserPnts, int dimention)
{
	assert(dimention==2||dimention==3);
	int nUnValidPnt = laserPnts.RemoveDoublePoints(dimention==2);

	return nUnValidPnt;
}

//clean point with out segment tag
IEDll_API int MyCleanOddPnts( LaserPoints& laserPnts)
{
	int nValidPnts = 0;
	int nUnValidPnt = 0;
	LaserPoints::iterator pnt1, pnt2;
	for (int i=0; i<laserPnts.size()-nUnValidPnt; ++i) {
		pnt1 = laserPnts.begin()+i;
		if (pnt1->HasAttribute(SegmentNumberTag)) {
			nValidPnts ++;
			continue;
		}
		else {
			for (int j=laserPnts.size()-1-nUnValidPnt; j>=nValidPnts; --j) {
				pnt2 = laserPnts.begin()+j;
				nUnValidPnt++;
				if (pnt2->HasAttribute(SegmentNumberTag)) {
					std::swap(*pnt1, *pnt2);
					nValidPnts ++;
					break;
				}
			}
		}
	}

	laserPnts.erase(laserPnts.begin()+nValidPnts, laserPnts.end());;

	return nUnValidPnt;
}

IEDll_API bool RefineSegmentIslands(LaserPoints& laserPnts, vector<PointNumberList>& vecSegPnls)
{
	//use IsSelectedTag to store component index
	//laserPnts.SetAttribute(IsProcessedTag, 0);
	LaserPoints::iterator point, neibPnt;
//	int new_label;
	vector<vector<int> > vecComponents;
	vector<int> curComponent;
	vector<int>::iterator node;
	TINEdges::const_iterator  neighbours;
	PointNumberList::const_iterator nbPntNum;
	PointNumberList curPnl;
	PointNumberList::iterator itrPntNum;
	int curPntNum;
	LaserPoints temLaserPnts;
	//laserPnts.RemoveAttribute(IsProcessedTag);
	int curSegNum;
	vector<int> vecPnlInds;
	TINEdges tinEdges;

	int maxSegNumber = laserPnts[vecSegPnls[vecSegPnls.size()-1][0].Number()].Attribute(SegmentNumberTag);

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//Main loop
	//vector<PointNumberList>::iterator itrSegPnls;
	for (int iSeg=0; iSeg<vecSegPnls.size(); ++iSeg) {
		if (vecSegPnls[iSeg].size()<3) continue;

		vecComponents.clear();
		curSegNum = laserPnts[vecSegPnls[iSeg][0].Number()].SegmentNumber();
		vecPnlInds.clear();
		for(int iPnt=0; iPnt<vecSegPnls[iSeg].size(); ++iPnt) 
			vecPnlInds.push_back(vecSegPnls[iSeg][iPnt].Number());
		temLaserPnts = laserPnts.Select(vecPnlInds);
		temLaserPnts.SetAttribute(IsProcessedTag, 0);
		tinEdges.Derive(*temLaserPnts.DeriveTIN()); 
		//temLaserPnts.DeriveEdges()

		//temLaserPnts = laserPnts.AddTaggedPoints()
		//////////////////////////////////////////////////////////////////////////
		//searching for islands in the whole segment points 
		for (int iPnt=0; iPnt<vecPnlInds.size(); ++iPnt) {
			point = temLaserPnts.begin()+iPnt;
			if (point->Attribute(IsProcessedTag) == 1) continue;

			curComponent.clear();
			curComponent.push_back(iPnt);
			point->SetAttribute(IsProcessedTag, 1);

			//component growing
			//for (node=curComponent.begin(); node!=curComponent.end(); ++node) {
			for (int iNode=0; iNode!=curComponent.size(); ++iNode) {
				neighbours = tinEdges.begin() + curComponent[iNode];
				
				for (nbPntNum=neighbours->begin(); nbPntNum!=neighbours->end(); ++nbPntNum) {
					neibPnt = temLaserPnts.begin() + nbPntNum->Number();
					//if (neibPnt->SegmentNumber() != curSegNum//not on same segment
					//	|| neibPnt->Attribute(IsProcessedTag) == 1)//already processed
					//	continue;
					if (neibPnt->Attribute(IsProcessedTag) == 1
						|| neibPnt->Distance(temLaserPnts[curComponent[iNode]]) > 1.5)
					continue;

					neibPnt->SetAttribute(IsProcessedTag, 1);
					curComponent.push_back(nbPntNum->Number());
				}
			}
			vecComponents.push_back(curComponent);
		}

		//transform local point numbers into global ones
		int localNumber;
		for (int iComp=0; iComp<vecComponents.size(); ++iComp) {
			for (int iPnt=0; iPnt<vecComponents[iComp].size(); ++iPnt) {
				localNumber = vecComponents[iComp][iPnt];
				vecComponents[iComp][iPnt] = vecPnlInds[localNumber];
			}
		}

		//make sure first component has the biggest point numbers
		for (int iComp=1; iComp<vecComponents.size(); ++iComp) {
			if(vecComponents[iComp].size()>vecComponents[0].size())
				std::swap(vecComponents[iComp], vecComponents[0]);
		}

		//////////////////////////////////////////////////////////////////////////
		//keep first component
		//delete the tiny components, and renumber the relative big components
		int nPntNum = vecSegPnls[iSeg].size();
		for (int iComp=1; iComp<vecComponents.size(); ++iComp) {
			double ratio = (double)vecComponents[iComp].size()/(double)nPntNum;
			if(ratio<0.05) {//set points as no segment number
				for (int iPnt=0; iPnt<vecComponents[iComp].size(); ++iPnt) {
					curPntNum = vecComponents[iComp][iPnt];
					point = laserPnts.begin()+curPntNum;
					point->RemoveAttribute(SegmentNumberTag);
				}
			}
			else {//set component as another segment
				curPnl.clear();
				maxSegNumber++;
				for (int iPnt=0; iPnt<vecComponents[iComp].size(); ++iPnt) {
					curPntNum = vecComponents[iComp][iPnt];
					point = laserPnts.begin()+curPntNum;
					point->SetAttribute(SegmentNumberTag, maxSegNumber);
					curPnl.push_back(PointNumber(curPntNum));
				}
				vecSegPnls.push_back(curPnl);
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//remove deleted points from current point number list
		for (int iPnt=0; iPnt<vecSegPnls[iSeg].size(); ++iPnt) {
			itrPntNum = vecSegPnls[iSeg].begin()+iPnt;
			if (laserPnts[itrPntNum->Number()].SegmentNumber() != curSegNum) {
				vecSegPnls[iSeg].erase(itrPntNum);
				--iPnt;
			}
		}
	}

	return true;
}

IEDll_API bool MyIntersect2Lines(const Line3D &lin1, const Line3D &lin2, Vector3D &pos)
{
	
	if (lin1.Direction().Length() < 0.000001 || lin2.Direction().Length() < 0.000001)
		return false;
	
	if ( Distance2Lines(lin1, lin2) > 0.5)
		return false;
	
	if (MyIsParaller(lin1.Direction(), lin2.Direction(), 0.1*3.1415926/360))
		return false;	

	double angle = Angle2Lines(lin1, lin2);

	//first intersection
	Vector3D pnt1 = lin1.FootPoint();
	Vector3D dir1 = lin1.Direction().Normalize();
	double dist1 = lin2.DistanceToPoint(pnt1);
	Vector3D intsectPnt1 = pnt1 + dir1 * dist1/sin(angle); 
	double temDist1 = lin2.DistanceToPoint(intsectPnt1);
	if (lin2.DistanceToPoint(intsectPnt1) > 0.1)
		intsectPnt1 = pnt1 - dir1 * dist1/sin(angle); //inverse direction

	//second intersection
	Vector3D pnt2 = lin2.FootPoint();
	Vector3D dir2 = lin2.Direction().Normalize();
	double dist2 = lin1.DistanceToPoint(pnt2);
	Vector3D intsectPnt2 = pnt2 + dir2 * dist2/sin(angle); 
	double temDist2 = lin1.DistanceToPoint(intsectPnt2);
	if (lin1.DistanceToPoint(intsectPnt2) > 0.1)
		intsectPnt2 = pnt2 - dir2 * dist2/sin(angle); //inverse direction

	pos = (intsectPnt1 + intsectPnt2)/2;
	//	pos.SetX((intsectPnt1.X()+intsectPnt2.X())/2);
	//	pos.SetY((intsectPnt1.Y()+intsectPnt2.Y())/2);
	//	pos.SetZ((intsectPnt1.Z()+intsectPnt2.Z())/2);

	/*
	Vector3D cross = lin1.Direction().VectorProduct(lin2.Direction());
	Plane plane1 = Plane(lin1.FootPoint(), cross);
	Plane plane2 = Plane(lin2.FootPoint(), cross);
	*/

	return true;
}

#include "LineSegment2D.h"
IEDll_API bool MyIsCrossing(const LineSegment2D& seg0, const LineSegment2D& seg1, double err_angle)
{
	const Line2D& line0 = seg0.Line2DReference();
	const Line2D& line1 = seg1.Line2DReference();
	if (ParallelLines(line0, line1, err_angle)) return false;
	
	Position2D crossPnt;
	Intersection2NonParallelLines(line0, line1, crossPnt);
	double scalar0 = seg0.Scalar(crossPnt);
	double scalar1 = seg1.Scalar(crossPnt);

	//add 0.0001 for double precision problem
	//for exclude the end point
	if (scalar0>seg0.ScalarBegin()+0.00001 && scalar0<seg0.ScalarEnd()-0.00001 &&
		scalar1>seg1.ScalarBegin()+0.00001 && scalar1<seg1.ScalarEnd()-0.00001)
		return true;

	return false;
}

IEDll_API bool MyIntersectLine3DPlane(const Line3D &line, const Plane &plane, Position3D &point)
{
	Vector3D normal, direction;
	double   denom, scalar;

	/* Retrieve directions */
	normal = plane.Normal();
	direction = line.Direction();
	normal = normal.Normalize();
	direction = direction.Normalize();

	/* Check intersection angle */
	//if the intersection is perpendicular, 
	//it means this line and plane are pararell, 
	denom = normal.DotProduct(direction);
	if (fabs(denom) < 0.087)//cos(85*PI/180)
      return false;

	/* Calculate intersection point */
	scalar = (plane.Distance() - line.FootPoint().DotProduct(normal)) / denom;
	point = line.Position(scalar);

	return true;
}

IEDll_API bool MyIsParaller(const Vector3D& lDir, const Vector3D& rDir, double maxAngle)
{
    Vector3D dir1 = lDir.Normalize();
    Vector3D dir2 = rDir.Normalize();
    double lLen = dir1.Length();
    double rLen = dir2.Length();
    if(lLen<0.0000001 || rLen<0.0000001)
        return false;
    double value = dir1.DotProduct(dir2);
    if(value>1) value = 1;
    else if(value<-1) value = -1;
	double angle = acos(value );

	if (angle > PI/2)
		angle = PI - angle;

	if (angle < maxAngle)
		return true;
	else 
		return false;
}

IEDll_API bool IsMergeAble(const LaserPoints& gLaserPnts, const PointNumberList& lSeg, 
	const PointNumberList& rSeg, const Plane& lPlane, const Plane& rPlane)
{
	//for debug
	/*int segNum1 = gLaserPnts[lSeg[0].Number()].Attribute(SegmentNumberTag);
	int segNum2 = gLaserPnts[rSeg[0].Number()].Attribute(SegmentNumberTag);
	if ((segNum1 == 880 && segNum2 == 1078)
		|| (segNum2 == 880 && segNum1 == 1078))
	{
		int aaa = 0;
	}*/

	double planDist = 10000.0, temDist;
	Position3D lCentPnt = gLaserPnts.CentreOfGravity(lSeg) ;
	Position3D rCentPnt = gLaserPnts.CentreOfGravity(rSeg) ;
	double angle = MyAngle(lPlane.Normal(), rPlane.Normal());
	bool isParaller = MyIsParaller(lPlane.Normal(), rPlane.Normal(), 10*PI/180.0);
	if (isParaller) {
		temDist = fabs(lPlane.Distance(rCentPnt));
		if (temDist < planDist)
			planDist = temDist;
		temDist = fabs(rPlane.Distance(lCentPnt));
		if (temDist < planDist)
			planDist = temDist;
	}

	return (isParaller && planDist < 0.3);
}

//check whether one intersection line could be removed
//this type of intersection line is usually formed by one large horizontal roof and a small oblique roof
IEDll_API bool IsDeleteAble(const LaserPoints& gLaserPnts, const PointNumberList& lSeg, 
	const PointNumberList& rSeg, const Plane& lPlane, const Plane& rPlane)
{
	double pointRatio = 1.0;
	if (lPlane.IsHorizontal(10*PI/360.0)) 
		pointRatio = 1.0*rSeg.size() / lSeg.size();
	else if (rPlane.IsHorizontal(10*PI/360.0)) 
		pointRatio =  1.0*lSeg.size() / rSeg.size() ;

	return (pointRatio < 0.1 );
}


//0 - pi
IEDll_API double MyAngle(const Vector3D& lDir, const Vector3D& rDir)
{
	Vector3D dir1 = lDir.Normalize();
	Vector3D dir2 = rDir.Normalize();
	double lLen = dir1.Length();
	double rLen = dir2.Length();
	if(lLen<0.0000001 || rLen<0.0000001)
		return 0.0;
	double value = dir1.DotProduct(dir2);
	if(value>1) value = 1;
	else if(value<-1) value = -1;
	double angle = acos(value );
	return angle;
	
	//return 0.0;
}


IEDll_API bool MyIsParaller(const Plane& plane, const Line3D& line, double maxAngle)
{
	Vector3D lDir = plane.Normal();
	Vector3D rDir = line.Direction();
	double lLen = lDir.Length();
	double rLen = rDir.Length();
    if(lLen<0.0000001 || rLen<0.0000001)
        return false;
    double value = lDir.DotProduct(rDir)/(lLen * lLen);    
    if(value>1) value = 1;
    else if(value<-1) value = -1;
	double angle = acos(value );

	if (angle > PI/2)
		angle = PI - angle;

	if (PI/2 - angle < maxAngle)
		return true;
	else
		return false;
}

IEDll_API bool GetCountour(const LaserPoints& gLaserPnts, const vector<PointNumberList>& vecLocalSegPnts, 
    ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
//	int PlaneNum;
	PointNumberList pnlFace;
	TINEdges tinEdges;// = gLaserPnts.DeriveTIN();
	tinEdges.Derive(gLaserPnts.TINReference());
	LineTopology temLineTop;
	//LaserPoints FaceLaserPnts;
	localLineTops.clear();
	localObjPnts.clear();
	ObjectPoint temObjPnt(0, 0, 0,0, 0, 0, 0, 0, 0, 0);
	double pntSpace = gLaserPnts.MedianInterPointDistance((int)min(1000.0, (double)gLaserPnts.size()-2));

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlFace = vecLocalSegPnts[i];
		MyDeriveContour(gLaserPnts, pnlFace, temLineTop);

		for (int j=0; j<temLineTop.size(); j++)
		{
			temObjPnt.X() = gLaserPnts[temLineTop[j].Number()].X();
			temObjPnt.Y() = gLaserPnts[temLineTop[j].Number()].Y();
			temObjPnt.Z() = gLaserPnts[temLineTop[j].Number()].Z();

			temObjPnt.Number() = localObjPnts.size();
			temLineTop[j].Number() = temObjPnt.Number();
			localObjPnts.push_back(temObjPnt);
		}
		temLineTop.Number() = localLineTops.size();
		localLineTops.push_back(temLineTop);
	}

	return true;
}

//clockwise search for boundary points
//factor:  a weight to determine minimum length of edge
//minimum length = factor * mean point space
IEDll_API bool MyDeriveContour(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg, PointNumberList& contour, 
	 const double factor, const Plane* pSegPlane)
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

	TINEdgeSet				      edgeset;
	PointNumber startpoint, tempoint, prevpoint, nextpoint, curpoint;
	double                          maxX, X;
	bool bFoundNext;
//	int startIndx, searchCount;
	int iterateNum;
	double temAng, prevAng, minAng;

	//fit plane 
	//	int segNumber = gLaserPnts[pnlSeg[0].Number()].Attribute(SegmentNumberTag);
    Plane plane; 
	Vector3D refDir;
	if (pSegPlane) plane = *pSegPlane;
	else plane = gLaserPnts.FitPlane(pnlSeg, 0);
	refDir = Vector3D(0.0, 1.0, 0.0);

/*		LaserPoints las = gLaserPnts;
	las.Write("e:\\seg.laser", 0, 0);
	FILE* file = new FILE;
	file = fopen("e:\\info.txt","w");
	fprintf(file, "%.4f, %.4f, %.4f, %.4f", plane.Normal().X(), plane.Normal().Y(), 
		plane.Normal().Z(), plane.Distance());
	fclose(file);*/
	
	//find reference direction
//	if (plane.IsVertical(PI*10/180))
//		refDir = Vector3D(0,0,1);
//	else
//		refDir = Vector3D(0,1,0);

	//transform points to reference plane
	//initialize a local laser points and TIN
	Position2D temPos2D;
	LaserPoints lpLocalSeg;
	PointNumberList pnlLocalSeg;//(pnlSeg.size(), PointNumber(-1));
	vector<bool> vecIsValidTag(pnlSeg.size(), true);
	LaserPoint temLaserPnt;
	for (int i=0; i< (int)pnlSeg.size(); ++i)	
		pnlLocalSeg.push_back(PointNumber(i));
/*
	for (int i=0; i< (int)pnlSeg.size(); ++i)	{
		pnlLocalSeg.push_back(PointNumber(i));
		temLaserPnt = gLaserPnts[pnlSeg[i].Number()];
		temPos2D = Project2RefPlane(plane, temLaserPnt.Position3DRef());
		temLaserPnt.X() = temPos2D.X();
		temLaserPnt.Y() = temPos2D.Y();
		temLaserPnt.Z() = 0.0;
		lpLocalSeg.push_back(temLaserPnt);
	}*/
	
	Position3D center;
	Project2RefPlane(gLaserPnts, pnlSeg, plane, center, lpLocalSeg);

	
#ifdef _DEBUG
	//for debug
	LaserPoints projPnts;
	for (int i=0; i<pnlLocalSeg.size(); i++) {
		projPnts.push_back(gLaserPnts[pnlLocalSeg[i].Number()]);
	}
	
	projPnts.Write("proj.laser", 0, 0);
	lpLocalSeg.Write("debug.laser");
#endif	
//	return false;*/
	
	lpLocalSeg.DeriveTIN();
	TINEdges tinEdges; 
	tinEdges.Derive(lpLocalSeg.TINReference());
	if(tinEdges.size()!=lpLocalSeg.size()) return false;

	double minEdgeLen ;
	//minEdgeLen = lpLocalSeg.MedianInterPointDistance((int)min(1000.0, (double)lpLocalSeg.size()-2));
	//minEdgeLen = factor*minEdgeLen;
	//minEdgeLen = 1.2;
	minEdgeLen = gBldRecPar.alphaDia;
	/////////////////////////////////////////////
	//search for start point, 
	//those points have not enough neighbor points will be skipped
	int neibEdgeCount;
	iterateNum = 0;
	do 	{
		maxX = -9999999;
		for (int i=0; i<pnlLocalSeg.size(); i++) {
			curpoint = pnlLocalSeg[i];
			if (vecIsValidTag[curpoint.Number()] == false) continue;
			//if (refDir == Vector3D(0,0,1))
			//	X = lpLocalSeg[curpoint.Number()].Z();
			//else
			//	X = lpLocalSeg[curpoint.Number()].X();
			X = lpLocalSeg[curpoint.Number()].X();
			if (X > maxX) {
				maxX = X; 
				startpoint = curpoint;
			}
		}

		//find whether this point has enough neighbor points
		neibEdgeCount = 0;
		if (vecIsValidTag[startpoint.Number()] != false) {
			edgeset = tinEdges[startpoint.Number()];
			LaserPoint& p1 = lpLocalSeg[startpoint.Number()];
			for (int i=0; i<edgeset.size(); i++) {
				tempoint = edgeset[i];
				if (vecIsValidTag[tempoint.Number()] == false) continue;
				LaserPoint& p2 = lpLocalSeg[tempoint.Number()];
				if (p1.Distance(p2) < minEdgeLen) ++neibEdgeCount;
			}
			if(neibEdgeCount < 2) vecIsValidTag[startpoint.Number()] = false;
		}
		++iterateNum;
	} while (neibEdgeCount < 2 && iterateNum < 100);
	

#ifdef _DEBUG
	lpLocalSeg[startpoint.Number()].SetAttribute(ComponentNumberTag, 0);
	lpLocalSeg.Write("debug.laser");
#endif

	contour.push_back(startpoint);

	curpoint = startpoint;
	prevpoint.Number() = -1;
	iterateNum = 0;
	Vector2D vec2d;
	double distance;
	do {
		edgeset = tinEdges[curpoint.Number()];
		bFoundNext = false;
		minAng = 4*PI;

		//search for next point
		for (int i=0; i<edgeset.size(); i++) {
			tempoint = edgeset[i];

			//skip invalid node and previous node
			if (vecIsValidTag[tempoint.Number()] == false
				||tempoint.Number() == prevpoint.Number())
				continue;

			vec2d.X() = lpLocalSeg[tempoint.Number()].X() - lpLocalSeg[curpoint.Number()].X();
			vec2d.Y() = lpLocalSeg[tempoint.Number()].Y() - lpLocalSeg[curpoint.Number()].Y();
			distance = vec2d.Length();
			if (distance > minEdgeLen) continue;//this line is too long, skip

			temAng = TINEdgeAngle(lpLocalSeg, curpoint, tempoint, plane, refDir);
			//first point of the contour
			if (prevpoint.Number() == -1) {
				if (temAng < minAng ) {
					bFoundNext = true;
					minAng = temAng;
					nextpoint = tempoint;
				}
			}
			else {//not first point of the contour
				double angleDiff = temAng - prevAng;
				if (angleDiff <= 0)	angleDiff += 2*PI;
				if ( angleDiff < minAng) {
					bFoundNext = true;
					minAng = angleDiff;
					nextpoint = tempoint;
				}
			}
		}

		if (bFoundNext) {
			prevpoint = curpoint;
			curpoint = nextpoint;
			contour.push_back(curpoint);		  
			prevAng = TINEdgeAngle(lpLocalSeg, curpoint, prevpoint, plane, refDir);
		}
		else {
			//if cannot find next node, current node is invalid
			//go back to previous node and search for another neighbor node
			vecIsValidTag[curpoint.Number()] = false;
			contour.erase(contour.end()-1);
			
			if (contour.empty())
				bFoundNext = false;
			else if (contour.size() == 1) {
				curpoint = contour[contour.size()-1];
				prevpoint.Number() = -1;
				bFoundNext = true;
			}
			else  {
				curpoint = contour[contour.size()-1];
				prevpoint = contour[contour.size()-2];
				prevAng = TINEdgeAngle(lpLocalSeg, curpoint, prevpoint, plane, refDir);
				bFoundNext = true;
			}
		}

		iterateNum++;
	} while (bFoundNext && nextpoint.Number() != startpoint.Number() && iterateNum < 2000);

	//transform to original number
	for (int i=0; i<contour.size(); i++)
		contour[i] = pnlSeg[contour[i].Number()];

	//for debug
/*	ObjectPoint objPnt;
	LineTopology lineTop;
	LineTopologies lineTops;
	ObjectPoints objPnts;
	LaserPoints lpTem;
	LaserPoint lp;
	for (int i=0; i<contour.size(); i++){
		lp = lpLocalSeg[contour[i].Number()];
		objPnt.X()=lp.X(); objPnt.Y()=lp.Y(); objPnt.Z()=lp.Z();
		objPnt.Number() = i;
		objPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());
	}
	lineTops.push_back(lineTop);
	objPnts.Write("E:\\zzzBoundaryPoints.objpts");
	lineTops.Write("E:\\zzzBoundaryPoints.top");*/
/*	ObjectPoint objPnt;
	LineTopology lineTop;
	LineTopologies lineTops;
	ObjectPoints objPnts;
	LaserPoints lpTem;
	LaserPoint lp;
	for (int i=0; i<contour.size(); i++){
		lp = gLaserPnts[contour[i].Number()];
		//objPnt.X()=lp.X(); objPnt.Y()=lp.Y(); objPnt.Z()=lp.Z();
		objPnt.Position3DRef() = lp.Position3DRef();
		objPnt.Number() = i;
		objPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());
	}
	lineTops.push_back(lineTop);
	objPnts.Write("E:\\zzzBoundaryPoints.objpts");
	lineTops.Write("E:\\zzzBoundaryPoints.top");*/
	
	return true;
}

//project points onto the reference plane
//and then computer the angle with reference direction
IEDll_API double TINEdgeAngle(const LaserPoints& gLaserPoints, const PointNumber& startPntNum, 
	const PointNumber& endPntNum, const Plane& refPlane,const Vector3D& refDirection)
{
	double angle;
	Vector3D startPnt, endPnt;
	startPnt.X() = gLaserPoints[startPntNum.Number()].X();
	startPnt.Y() = gLaserPoints[startPntNum.Number()].Y();
	startPnt.Z() = gLaserPoints[startPntNum.Number()].Z();

	endPnt.X() = gLaserPoints[endPntNum.Number()].X();
	endPnt.Y() = gLaserPoints[endPntNum.Number()].Y();
	endPnt.Z() = gLaserPoints[endPntNum.Number()].Z();

	if (refDirection == Vector3D(0,1,0))
	{//project to X-Y plane
		endPnt.Z() = startPnt.Z(); 
		angle = Angle(endPnt-startPnt, refDirection);

		if (endPnt.X() < startPnt.X())
			angle = 2*PI - angle;
	}
	else 
	{
		if (fabs(refPlane.Normal().X()) > fabs(refPlane.Normal().Y()))
		{//project to Y-Z plane
			endPnt.X() = startPnt.X(); 
			angle = Angle(endPnt-startPnt, refDirection);

			if (endPnt.Y() < startPnt.Y())
				angle = 2*PI - angle;
		}
		else
		{//project to X-Z plane
			endPnt.Y() = startPnt.Y(); 
			angle = Angle(endPnt-startPnt, refDirection);

			if (endPnt.X() < startPnt.X())
				angle = 2*PI - angle;
		}
	}

	//(0-2*PI]
	if (angle == 0.0) angle = 2*PI;

	return angle;
}

IEDll_API void MyRemoveInvalidPnts(LaserPoints& cloud, vector<char>& vecValid)
{
	assert(cloud.size() == vecValid.size());

	int nInValid = 0;
	for (int i=0; i<vecValid.size()-nInValid-1; ++i) {
		if (vecValid[i]) continue;

		for (int j=vecValid.size()-nInValid-1; j>i; --j) {
			nInValid++;
			if (vecValid[j]) {
				std::swap(vecValid[i], vecValid[j]);
				std::swap(cloud[i], cloud[j]);
				break;
			}
		}
	}

	cloud.erase(cloud.begin()+cloud.size()-nInValid, cloud.end());
}

IEDll_API double Distance2Polygons(const LineTopologies& orgLineTops, 
	const ObjectPoints& ObjPnts, const Position3D& inPnt)
{
	double minDist = 99999999.9, temDist;
	Line3D line;
	int start, end;

	for (int i=0; i<orgLineTops.size(); ++i) {
		for (int j=0; j<orgLineTops[i].size()-1; ++j)
		{
			start = orgLineTops[i][j].Number();
			end = orgLineTops[i][j+1].Number();

			line = Line3D(ObjPnts[start], ObjPnts[end]);
			temDist = line.DistanceToPoint(inPnt);
			if (temDist<minDist)
				minDist = temDist;
		}
	}

	return minDist;
}

IEDll_API double Distance2Polygons(const vector<Line3D>& vecPolyLines, const Position3D& inPnt)
{
	double minDist = 99999999.9, temDist;
	Line3D line;
	int start, end;

	for (int i=0; i<vecPolyLines.size(); ++i) {
		temDist = vecPolyLines[i].DistanceToPoint(inPnt);	
		if (temDist<minDist) minDist = temDist;
	}

	return minDist;
}

int ifile=0;
IEDll_API vector<int> Histgram(const vector<double>& inValues, double min, double max, double binWid)
{
	/*char c[50];
	sprintf(c, "%d",ifile++);
	string filePath = ".\\residual\\histogram_";
	filePath += c;
	filePath += ".txt";

	//sprintf(f);
	FILE*  file = fopen(filePath.c_str(), "w+");
	for (int i=0; i<inValues.size(); i++) {
		fprintf(file, "%.6f\n", inValues[i]);
	}
	fclose(file);*/

//	assert(max>min && min>=0.0 && binWid>0);
	assert(max>min && binWid>0);
	
	int bins = (max-min)/binWid;
	vector<int> hist = vector<int>(bins, 0);

	int ind;
	for (int i=0; i<inValues.size(); i++) {
		ind = (inValues[i]-min)/binWid;
		if (ind<0) ind = 0;
		else if(ind>bins-1) ind = bins-1;
		hist[ind]++;
	}

	return hist;
}

IEDll_API vector<int> Histgram(const vector<double>& inValues, int binSize, double& binWid)
{
	assert(binSize>0);

	vector<double>::const_iterator itr;
	itr = std::max_element(inValues.begin(), inValues.end());
	double max = *itr;
	//itr = std::min_element(inValues.begin(), inValues.end());
	double min = 0.0;
	binWid = max/binSize;

	int ind;
	vector<int> hist = vector<int>(binSize, 0);
	
	for (int i=0; i<inValues.size(); i++) {
		ind = (inValues[i]-min)/binWid;
		if (ind<0) ind = 0;
		else if(ind>binSize-1) ind = binSize-1;
		hist[ind]++;
	}

	return hist;
}


IEDll_API int MaxFrequentValue(const std::vector<int>& inValues)
{
	std::map<int, int> container;
	std::map<int, int>::iterator itrCon;
	int maxFrequent = 0;
	int maxFrequentValue = -1;

	for (unsigned int i=0; i<inValues.size(); i++) {
		itrCon = container.find(inValues[i]);
		
		if (container.end() == itrCon) {
			container.insert(make_pair(inValues[i],1));
			if (maxFrequent<1) {
				maxFrequent = 1;
				maxFrequentValue = inValues[i];
			}
		}
		else {
			itrCon->second++;
			if (maxFrequent<itrCon->second) {
				maxFrequent = itrCon->second;
				maxFrequentValue = itrCon->first;
			}
		}
	}

	return maxFrequentValue;
}

//Biao, Nov 19, 2012
//check whether or not a polygon cross itself.
IEDll_API bool MyIsSelfCross(const LineTopology& polygon, const ObjectPoints& objPnts)
{
	if (!polygon.IsClosed()) return false;

	Line3D line, temLine;
	int pntNum1,pntNum2;
	double scale11, scale12;
	double scale21, scale22;
	double temScale1, temScale2;
	Vector3D intersectPnt;
	ObjectPoints::const_iterator itrPnt1, itrPnt2;

	//check corner points
	for (int iPnt=0; iPnt<polygon.size()-2; iPnt++) {
		pntNum1 = polygon[iPnt].Number();
		itrPnt1 = objPnts.begin()+pntNum1;

		for (int jPnt=iPnt+1; jPnt<polygon.size()-1; jPnt++) {
			pntNum2 = polygon[jPnt].Number();
			itrPnt2 = objPnts.begin()+pntNum2;
			if (itrPnt1->Position3DRef()==itrPnt2->Position3DRef())
				return true;
		}
	}


	//check lines
	//Biao Oct 22, 2014
	LineSegments2D lineSegs = LineSegments2D(objPnts, polygon);
	LineSegments2D::iterator line0, line1;
	PointNumber pntNum00, pntNum01, pntNum10, pntNum11;
	for (line0=lineSegs.begin(); line0!=lineSegs.end(); line0++) {
		pntNum00 = polygon[line0-lineSegs.begin()];
		pntNum01 = polygon[line0-lineSegs.begin()+1];

		for (line1=line0+1; line1!=lineSegs.end(); line1++) {
			pntNum10 = polygon[line1-lineSegs.begin()];
			pntNum01 = polygon[line1-lineSegs.begin()+1];
			
			//ignore lines with same end points
			if (pntNum00==pntNum10 || pntNum00==pntNum11 ||
				pntNum01==pntNum10 || pntNum01==pntNum10)
				continue;

			if (objPnts[pntNum00.Number()].Position3DRef()==objPnts[pntNum10.Number()] ||
				objPnts[pntNum00.Number()].Position3DRef()==objPnts[pntNum11.Number()] ||
				objPnts[pntNum01.Number()].Position3DRef()==objPnts[pntNum10.Number()] ||
				objPnts[pntNum01.Number()].Position3DRef()==objPnts[pntNum11.Number()] )
				continue;

			if (MyIsCrossing(*line0, *line1)) {
#ifdef _DEBUG
				
				LineTopologies temLines;
				LineTopology temLine;

				temLine.push_back( pntNum00);
				temLine.push_back( pntNum01);
				temLines.push_back(temLine);

				temLine.clear();
				temLine.push_back( pntNum10);
				temLine.push_back( pntNum01);
				temLines.push_back(temLine);

				temLines.Write("debug.top");
				objPnts.Write("debug.objpts");
				MyIsCrossing(*line0, *line1);
#endif

				return true;
			}
		}
	}
	
		/*
	for (int iPnt=0; iPnt<polygon.size()-2; iPnt++) {
		pntNum1 = polygon[iPnt].Number();
		pntNum2 = polygon[iPnt+1].Number();
		line = Line3D(objPnts[pntNum1],	objPnts[pntNum2]);
		scale11 = line.Scalar(objPnts[pntNum1]);
		scale12 = line.Scalar(objPnts[pntNum2]);
		if (scale11>scale12) std::swap(scale11, scale12);

		for (int jPnt=iPnt+1; jPnt<polygon.size()-2; jPnt++) {
			pntNum1 = polygon[jPnt].Number();
			pntNum2 = polygon[jPnt+1].Number();
			temLine = Line3D(objPnts[pntNum1],	objPnts[pntNum2]);
			scale21 = temLine.Scalar(objPnts[pntNum1]);
			scale22 = temLine.Scalar(objPnts[pntNum2]);
			if (scale21>scale22) std::swap(scale21, scale22);

#ifdef _DEBUG
			if (iPnt==4 && jPnt==6 && pntNum1==59 && pntNum2==61)
			{
				LineTopologies temLines;
				LineTopology temLine;

				temLine.push_back( polygon[iPnt]);
				temLine.push_back( polygon[iPnt+1]);
				temLines.push_back(temLine);

				temLine.clear();
				temLine.push_back( polygon[jPnt]);
				temLine.push_back( polygon[jPnt+1]);
				temLines.push_back(temLine);
				
				temLines.Write("debug.top");
				objPnts.Write("debug.objpts");
			}
#endif
			//parallel lines
			if (!MyIntersect2Lines(line, temLine, intersectPnt))
				continue;
			temScale1 = line.Scalar(intersectPnt);
			temScale2 = temLine.Scalar(intersectPnt);

			if (temScale1>scale11 && temScale1<scale12
				&& temScale2>scale21 && temScale2<scale22)
				return true;
			//temLine.
		}
	}
	*/

	return false;
}


IEDll_API bool MyIsInsidePoly(const LaserPoint& cand, const LineTopology& polygon, const ObjectPoints& polyPnts)
{
	ObjectPoints::const_iterator begPnt, pt0, pt1;
	PointNumberList::const_iterator node;
	int   num_intersections;
	double  Y_test;

	//Check if the polygon is closed and has at least three points
	if (polygon.begin()->Number()!=(polygon.end()-1)->Number()) 
		return false;
	if (polygon.size() < 4) return false;

	begPnt = polyPnts.begin();
	pt0 = begPnt+polygon[0].Number();
	num_intersections = 0;
	const double candX = cand.X();
	const double candY = cand.Y();
	double dx1, dx2;

//#pragma omp parallel for private(pt1, dx1, dx2)
	for (int i=1; i<polygon.size(); ++i) {
		pt1 = begPnt+polygon[i].Number();
		dx1 = candX - pt0->X();
		dx2 = pt1->X() - candX;
		if (pt1->X() != candX && dx1*dx2>=0) {
			Y_test = (dx2*pt0->Y() + dx1*pt1->Y())/(pt1->X()-pt0->X());
			if (Y_test > candY) num_intersections++;
		}

		pt0 = pt1;
	}

	if ((num_intersections/2)*2 == num_intersections) return(0);
	else return(1);
}

/*
IEDll_API bool MyIsInsidePoly(const LaserPoint& cand, const LineTopology& polygon, const ObjectPoints& polyPnts)
{
	ObjectPoint                     *pt0, *pt1;
	PointNumberList::const_iterator node;
	int                             num_intersections;
	double                          Y_test;

	//Check if the polygon is closed and has at least three points
	if (polygon.begin()->Number() != (polygon.end()-1)->Number()) return(0);
	if (polygon.size() < 4) return(0); // 4 since begin and end point are the same

	//Get the first polygon point 
	pt0 = polyPnts.GetPoint(*(polygon.begin()));
	if (!pt0) {
		fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", polygon.begin()->Number());
		return(0);
	}

	// Count the number of intersections of polygon edges with the line from the
	// point (X, Y) to (X, inf).
	num_intersections = 0;
	for (node=polygon.begin()+1; node!=polygon.end(); node++) {
		//Get the next polygon point
		pt1 = polyPnts.GetPoint(*node);
		if (!pt1) {
			fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", node->Number());
			return(0);
		}

		// Check whether the lines intersect
		if (pt1->X()!=cand.X() && (pt0->X()-cand.X())*(pt1->X()-cand.X()) <= 0) {
			Y_test = ((pt1->X()-cand.X())*pt0->Y() +
				(cand.X()-pt0->X())*pt1->Y())/(pt1->X()-pt0->X());
			if (Y_test > cand.Y()) num_intersections++;
		}

		//Get ready for the next edge
		pt0 = pt1;
	}

	//Return 1 for an odd number of intersections
	if ((num_intersections/2)*2 == num_intersections) return(0);
	else return(1);
}*/

IEDll_API bool MyFitRectangle(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, 
	LineTopologies& localTops, ObjectPoints& localObjPts, double maxWid)
{
	localTops.clear();
	localObjPts.clear();

	Plane plane = laserPnts.FitPlane(pnlLineSeg);
	Line3D line;
	if(!MyFitLine(laserPnts, pnlLineSeg, line)) return false;

	double temScale, maxScale=-999999.9, minScale=999999.9;
	double temDist, farDist=-999999.9;
	LaserPoints::const_iterator pnt;
	Position3D temPos;

	for (int i=0; i<pnlLineSeg.size(); i++)	{
		pnt = laserPnts.begin()+pnlLineSeg[i].Number();

		temPos = plane.Project(pnt->Position3DRef());
		temScale = line.Scalar(temPos);
		if (temScale<minScale) minScale = temScale;
		if (temScale>maxScale) maxScale = temScale;

		temDist = line.DistanceToPoint(temPos);
		if (farDist<temDist) farDist = temDist;
	}
	
	if (farDist>maxWid/2.0) return false;

	/*3-------4
	| |       |
	|-1-------2------
	| |       |
	| 5-------6*/
	Vector3D dir = line.Direction().VectorProduct(plane.Normal());
	dir.Normalize();

	Position3D pos1 = line.Position(minScale);
	Position3D pos2 = line.Position(maxScale);
	Position3D pos3 = pos1 + farDist*dir;
	Position3D pos4 = pos1 - farDist*dir;
	Position3D pos5 = pos2 + farDist*dir;
	Position3D pos6 = pos2 - farDist*dir;

	AddTopLine(localObjPts, localTops, pos3, pos4);
	AddTopLine(localObjPts, localTops, pos4, pos6);
	AddTopLine(localObjPts, localTops, pos6, pos5);
	AddTopLine(localObjPts, localTops, pos5, pos3);
	return true;
}


bool SimplifyContour(const LaserPoints& gLaserPnts, Plane plane,
	PointNumberList& pnlCountour, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	localObjPnts.clear();
	localLineTops.clear();

	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts, pnlTemSegPnts;
	int curPlaneNum;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	int minPntNums = 3;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNums;
	PointNumberList pnlLineSeg;
	LaserPoints selLaserPnts, temLaserPnts, temBackProjPnts;
	LaserPoint temLaserPnt;
	Vector3D dir;
	Line2D  line;
	Position2D pos2d1, pos2d2, temPos2D;
	Position3D temPos, pos3d1, pos3d2;
	LineTopologies addedLineTops;
	vector<int> vecCornerPos;
	pnlTemBoundPnts.reserve(500);

	bool bClosed = (pnlCountour[0].Number()==pnlCountour[pnlCountour.size()-1]);
	pnlOrgTemBoundPnts = pnlCountour;

	selLaserPnts.clear();
	for (int j=0; j<pnlOrgTemBoundPnts.size(); j++) {
		selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
		pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
	}

	if (selLaserPnts.empty()) return false;
	dir = plane.Normal();

	temLaserPnts = selLaserPnts;
	PointNumberList pnlTemSeg;
	Position3D center;
	for (int i=0; i<selLaserPnts.size(); i++)
		pnlTemSeg.push_back(PointNumber(i));
	Project2RefPlane(selLaserPnts, pnlTemSeg, plane, center, temLaserPnts);
	ReProject2OrgPlane(temLaserPnts, plane, center, temBackProjPnts);

	pos3d1 = temLaserPnts[0].Position3DRef();
	pos3d2 = temLaserPnts[temLaserPnts.size()-1].Position3DRef();
	if (pos3d1.X() != pos3d2.X() && pos3d1.Y() != pos3d2.Y()) {
		temLaserPnts.push_back(temLaserPnts[0]);//enclose
		temBackProjPnts.push_back(temBackProjPnts[0]);
	}

	//use curvature to detect corner
	vector<double> vecK(temLaserPnts.size()-1, 0.0);//curvature
	//calculate curvature
	//use 3 sequent point, first and last point to fit line and calculate
	//distance of second point to the line
	for (int j=0; j<(int)temLaserPnts.size()-1; j++) {
		if (j==0) {
			temPos = temLaserPnts[0].Position3DRef();
			pos3d1 = temLaserPnts[temLaserPnts.size()-2].Position3DRef();
			pos3d2 = temLaserPnts[1].Position3DRef();
		}
		else {
			temPos = temLaserPnts[j].Position3DRef();
			pos3d1 = temLaserPnts[j-1].Position3DRef();
			pos3d2 = temLaserPnts[j+1].Position3DRef();
		}

		temLine = Line3D(pos3d1, pos3d2);
		dist = temLine.DistanceToPoint(temPos);
		double temk = dist/(pos3d1.Distance(pos3d2) + 0.000001);

		vecK[j] = dist/(pos3d1.Distance(pos3d2) + 0.000001);
	}

	//////////////////////////////////////////////////////////////////////////
	//detect boundary corner according to boundary points
	vector<int> vecCorner;
	int num1, num2;
	//int curNum;
	if(temLaserPnts.size() < 3) return false;//not enough points
	//////////////////////////////////////////////////////////////////////////
	//set the first point to be the one with biggest curvature,
	//and initialize the contour

	vector<double> vecTemK = vecK;
	std::sort(vecTemK.begin(), vecTemK.end());
	int indBeg;
	vector<double>::iterator itrFind;
	itrFind = std::find(vecK.begin(), vecK.end(), vecTemK[vecTemK.size()-1]);
	indBeg = itrFind-vecK.begin();
	vecK.insert(vecK.end(), vecK.begin(), vecK.begin()+indBeg);
	vecK.erase(vecK.begin(), vecK.begin()+indBeg);
	//the last point is redundant
	temLaserPnts.reserve(temLaserPnts.size()*2);
	temBackProjPnts.reserve(temBackProjPnts.size()*2);
	temLaserPnts.pop_back();
	temLaserPnts.insert(temLaserPnts.end(),temLaserPnts.begin(),temLaserPnts.begin()+indBeg);
	temLaserPnts.erase(temLaserPnts.begin(),temLaserPnts.begin()+indBeg);
	temLaserPnts.push_back(temLaserPnts[0]);
	temBackProjPnts.pop_back();
	temBackProjPnts.insert(temBackProjPnts.end(),temBackProjPnts.begin(),temBackProjPnts.begin()+indBeg);
	temBackProjPnts.erase(temBackProjPnts.begin(),temBackProjPnts.begin()+indBeg);
	temBackProjPnts.push_back(temBackProjPnts[0]);

	vecTemK = vecK;
	std::sort(vecTemK.begin(), vecTemK.end());
	vecCorner.push_back(0);
	itrFind = std::find(vecK.begin(), vecK.end(), vecTemK[vecTemK.size()-2]);
	vecCorner.push_back(itrFind-vecK.begin());
	vecCorner.push_back(temLaserPnts.size()-1);

	//get corners
	for (int j=0; j<(int)vecCorner.size()-1; j++) {
		num1 = vecCorner[j];
		num2 = vecCorner[j+1];
		pos2d1 = temLaserPnts[num1].Position2DOnly();
		pos2d2 = temLaserPnts[num2].Position2DOnly();
		line = Line2D(pos2d1, pos2d2);

		//search for possible corner on this boundary
		double maxDist = -999999;
		int candidatNum;
		for (int k=num1; k<num2; k++) {
			temLaserPnt = temLaserPnts[k];
			dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());

			if (dist > maxDist) {
				maxDist = dist;
				candidatNum = k;
			}
		}

		if (maxDist > 0.4) {
			vecCorner.insert(vecCorner.begin()+j+1, candidatNum);
			j--;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//refine corners
	//find point with max curvature in the rang
	for (int j=1; j<(int)vecCorner.size()-1; j++) {
		if (vecCorner.size() ==2) break;

		num1 = vecCorner[j-1];
		num2 = vecCorner[j+1];
		pos2d1 = temLaserPnts[num1].Position2DOnly();
		pos2d2 = temLaserPnts[num2].Position2DOnly();
		line = Line2D(pos2d1, pos2d2);

		//search for possible corner on this boundary
		double maxDist = -999999;
		int candidatNum;
		for (int k=num1; k<num2; k++) {
			temLaserPnt = temLaserPnts[k];
			dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());

			if (dist > maxDist) {
				maxDist = dist;
				candidatNum = k;
			}
		}

		if (maxDist > 0.1)
			vecCorner[j] = candidatNum;//adjust a corner
		else {
			vecCorner.erase(vecCorner.begin()+j);
			j--;
			continue;//remove a corner, go back to new line segment
		}

		pnlLineSeg.clear();
		for (int k=num1; k<num2; k++)
			pnlLineSeg.push_back(PointNumber(k));
		if (!MyFitLine(temLaserPnts, pnlLineSeg, line)) continue;
		double dev = 0.0;
		for (int k=num1; k<num2; k++)
			dev += pow(line.DistanceToPoint(temLaserPnts[k].Position2DOnly()), 2);
		dev /= (num2-num1-1);
		dev = sqrt(dev);

		//dist = line.DistanceToPoint(temLaserPnts[j].Position2DOnly());
		if (dev < 0.1 ) vecCorner.erase(vecCorner.begin()+j);
	}

	//out put
	vecCornerPos.clear();
	for (int j=0; j<(int)vecCorner.size()-1; j++) {
		num1 = vecCorner[j]; num2 = vecCorner[j+1];

		pos2d1 = temLaserPnts[num1].Position2DOnly();
		pos2d2 = temLaserPnts[num2].Position2DOnly();

		if (pos2d2.X()==pos2d1.X() && pos2d2.Y()==pos2d1.Y())
			pos2d2 = temLaserPnts[num2-1].Position2DOnly();//remove the start point

		pos3d1 = temBackProjPnts[num1].Position3DRef();
		pos3d2 = temBackProjPnts[num2].Position3DRef();

		//out put
		AddTopLine(localObjPnts, localLineTops, pos3d1, pos3d2, plane.Number(), -1);
	}

	return true;
}

int GetDominaintSegNum(const LaserPoints& lasPnts, PointNumberList pnlGroup)
{
	std::map<int, int> mapSeg2PntCount;
	std::map<int, int>::iterator itrMap;

	int segNum;
	for (int i=0; i<pnlGroup.size(); i++) {
		if (!lasPnts[pnlGroup[i].Number()].HasAttribute(SegmentNumberTag)) continue;
		segNum = lasPnts[pnlGroup[i].Number()].Attribute(SegmentNumberTag);
		itrMap = mapSeg2PntCount.find(segNum);
		if (itrMap!=mapSeg2PntCount.end())
			itrMap->second++;
		else
			mapSeg2PntCount.insert(pair<int,int>(segNum,0));
	}

	int maxCount = 0;
	int dominateSegNum = -1;
	for (itrMap=mapSeg2PntCount.begin(); itrMap!=mapSeg2PntCount.end(); itrMap++) {
		if (itrMap->second > maxCount) {
			maxCount = itrMap->second;
			dominateSegNum = itrMap->first;
		}
	}

	return dominateSegNum;
}
