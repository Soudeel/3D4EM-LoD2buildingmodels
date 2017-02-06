/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 12-03-2011
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

#include "InferenceEngine.h"
#include <vector>
#include <math.h>

using namespace std;
//April 05, 2011
//Biao Xiong
//try to enclose polygon based on detected ridges and corners
//Line Topology attribute: 
//0: left plane number; 1: right plane number; 2: label

//if the two end points are repeat, the line is stable
bool IsLineTopStable(const vector<bool>& vecRepOPnts, const LineTopology& lineTop)
{
	if (lineTop.size()!=2) return false;
	int num0 = lineTop[0].Number();
	int num1 = lineTop[1].Number();
	if (num0<0||num0>=vecRepOPnts.size()||num1<0||num1>=vecRepOPnts.size())
		return false;

	return (vecRepOPnts[num0]&&vecRepOPnts[num1]);
}

double ComputeEndPointScalar(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, const Line3D& line)
{
	double maxScalar = -9999999;
	double scalar;
	vector<double> vecScal(pnlSegPnts.size(),0.0);
	PointNumberList::const_iterator itrPnl;
	LaserPoints::const_iterator itrPnt;
	for (itrPnl = pnlSegPnts.begin(); itrPnl != pnlSegPnts.end(); ++itrPnl) {
		itrPnt = gLaserPnts.begin()+itrPnl->Number();
		//if (line.DistanceToPoint(itrPnt->Position3DRef())>2*INTERSECT_FACE_MIN_DIST)
		//	continue;//only computer points in a buffer area
		scalar = line.Scalar(itrPnt->Position3DRef());
		vecScal[itrPnl-pnlSegPnts.begin()] = scalar;
		//if (scalar > maxScalar)
		//	maxScalar = scalar;
	}

	std::sort(vecScal.begin(), vecScal.end());
	int ind = vecScal.size()-2;
	ind = ind>=0? ind:0;
	return vecScal[ind];

	//return maxScalar;
}

/*
//Sep 11, 2011, Biao Xiong
//old version is not precise when plane is a bit oblique
//In this version, end point is determined by the points in the buffer area which is along the line
Position3D ComputeEndPnt(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, 
	const Plane& plane, const Line3D& line) 
{
	Vector3D  curDir;
	curDir = line.Direction();
	Position3D temPos;
	double maxScal = -99999.9, temScale;
	LaserPoints::const_iterator itrPnt;

	for(int i=0; i<pnlSegPnts.size(); ++i) {
		itrPnt = gLaserPnts.begin()+pnlSegPnts[i].Number();
		temPos = plane.Project(itrPnt->Position3DRef());

		if (line.DistanceToPoint(temPos) > 0.5)
			continue;
		temScale = line.Scalar(temPos);
		if (temScale < 0.0) continue;

		if(temScale > maxScal)
			maxScal = temScale;
	}

	return line.Position(maxScal);
}*/


//use auxiliary line which is on the plane and should point to ground
Position3D ComputeEndPnt(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, 
	const Plane& plane, const Line3D& line) 
{
	Vector3D  curDir;
	curDir = line.Direction();
	Position3D endPos;
	double maxScal;
	//double myangle = Angle(curDir, Vector3D(0,0,1));

	//when plane is horizontal or vertical
	if (Angle(curDir, Vector3D(0,0,1))<10*PI/180 
		|| curDir.IsHorizontal(10*PI/180)
		|| plane.IsHorizontal(10*PI/180)) {
			maxScal = ComputeEndPointScalar(gLaserPnts, pnlSegPnts, line);
			endPos = line.Position(maxScal);
	}
	else {
		Vector3D auxDir = plane.Normal().VectorProduct(Vector3D(0,0,1));
		auxDir = plane.Normal().VectorProduct(auxDir);
		auxDir = auxDir.Normalize();
		if (Angle(auxDir, line.Direction()) > PI/2) auxDir *= -1;//same direction with current line
		Line3D auxLine =  Line3D(line.FootPoint(), auxDir);
		maxScal = ComputeEndPointScalar(gLaserPnts, pnlSegPnts, auxLine);
		Line3D lLine = Line3D(auxLine.Position(maxScal), 
			auxLine.Direction().VectorProduct(plane.Normal()));
		Intersection2Lines(lLine, line, endPos);
	}

	return endPos;
}

Position3D ComputeEndPnt(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops,
	const vector<int>& lineNums, const LineSegment3D& line)
{
	ObjectPoints::const_iterator pnt11, pnt12, pnt21, pnt22;
	Line3D temLine;
	Position3D crossPos, endPos;
	double temDist, minDist;
	minDist = 999999.9;
	Position3D lineEndPos = line.EndPoint();
	endPos = Position3D(9999999.9, 9999999.9, 9999999.9);

	for (int iLine=0; iLine<lineNums.size(); ++iLine) {
		const LineTopology& temTop = localLineTops[lineNums[iLine]];
		pnt21 = localObjPnts.begin()+temTop[0].Number();
		pnt22 = localObjPnts.begin()+temTop[1].Number();

		if (line.DistanceToPoint(pnt21->Position3DRef())<0.001
			&&line.DistanceToPoint(pnt22->Position3DRef())<0.001)
			continue;//the same line, skip
		if (pnt22->Distance(*pnt21)<0.5) continue;//too short, skip
		temLine = Line3D(pnt21->Position3DRef(), pnt22->Position3DRef());
		if (MyIsParaller(temLine.Direction(), line.Direction(), 15*PI/180))
			continue;//parallel line, skip

		Intersection2Lines(temLine, line, crossPos);
		temDist = lineEndPos.Distance(crossPos);
		if (temDist<minDist) {
			endPos = crossPos;
			minDist=temDist;
		}
	}

	return endPos;
}

//Feb 21, 2013, use lines on both surface to check end point
//choose the nearest point
Position3D ComputeEndPnt(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops,
	int planeNum, const LineSegment3D& line)
{
	vector<int> vecTLNums;
	SearchLinesOnPlane(localLineTops, planeNum, vecTLNums);
	return ComputeEndPnt(localObjPnts, localLineTops, vecTLNums, line);
}

double ComputeMeanPointScalar(LaserPoints& gLaserPnts, PointNumberList& pnlSegPnts, const Line3D& Line)
{
	double maxScalar = 0.0;
	for (int i=0; i<pnlSegPnts.size(); ++i) {
		maxScalar += Line.Scalar(gLaserPnts[pnlSegPnts[i].Number()].Position3DRef());
	}
	return maxScalar/(pnlSegPnts.size()+0.00001);
}

//computer coefficient of a boundary line exists
//compute the ratio of point number on both sides
//update March 12, 2012
//split the buffer area into bins, and average the coefficients in all bins
double ComputeBoundLineCoef(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, 
	const Plane& plane, const Line3D& line, double bufferWidth, double bufferLeng)
{
	Vector3D curDir = line.Direction();
	curDir = curDir.Normalize();
	Position3D curPnt = line.FootPoint();
	Vector3D auxDir = curDir.VectorProduct(plane.Normal());
	Position3D temPnt;
	double temSpace;
	double dotProduct;
	double conf = 0.0;
	int lSpaceCount =0, rSpaceCount = 0;
	PointNumberList::const_iterator itrPnl;
	LaserPoints::const_iterator itrPnt;
	int binNums = 10;
	vector<double> vecCoefs(binNums, 0.0);
	vector<int> vecBoundPnts1(binNums, 0);
	vector<int> vecBoundPnts2(binNums, 0);
	double binLenth = bufferLeng/binNums;
	double distance;
	double meanDist;

	LaserPoints insidePnts;
	int indBin;
	vector<int> vecIndBins;
	vector<double> vecMinDist = vector<double>(binNums, bufferWidth);

	for (itrPnl = pnlSegPnts.begin(); itrPnl!=pnlSegPnts.end(); ++itrPnl) {
		itrPnt = gLaserPnts.begin()+itrPnl->Number();
		temPnt = plane.Project(itrPnt->Position3DRef());
		distance = line.DistanceToPoint(temPnt);
		if (distance > bufferWidth)
			continue;//only computer points in the buffer zone
		dotProduct = curDir.DotProduct(temPnt-curPnt);
		//this laser point does not have same direction with current line direction
		//and only computer points in the buffer zone
		if (dotProduct<=0 || dotProduct>=bufferLeng)
			continue;

		temSpace = auxDir.DotProduct(temPnt-curPnt);
		//point is too near the boundary line
		if (fabs(temSpace) < 0.005) continue;
		indBin = dotProduct/binLenth;
//		insidePnts.push_back(temPnt);
//		vecIndBins.push_back(indBin);
		if (indBin>=binNums)
			indBin -= 1;

		if(distance < vecMinDist[indBin])
			vecMinDist[indBin] = distance;

		if (temSpace>0.0){
			++lSpaceCount;
			//conf += exp(-0.5*pow(distance/0.2,2));
			vecBoundPnts1[indBin]++;
		}
		else{
			++rSpaceCount;
			//conf -= exp(-0.5*pow(distance/0.2,2));
			vecBoundPnts2[indBin]++;
		}
	}
	
	double temCoef, coef;
	coef = 0.0;
	meanDist = 0.0;
	for (int i=0; i<binNums; i++) {
		meanDist += vecMinDist[i]/bufferWidth;
		temCoef = (vecBoundPnts1[i]+0.00001)/(vecBoundPnts2[i]+0.00001);
		if (temCoef > 1.0)
			temCoef = 1.0/temCoef;
		coef += temCoef;
	}
	meanDist /= binNums;
	coef /= binNums;
	coef = 1.0-coef;


	return coef;
}


bool IsOnSameLine(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops,
	const vector<int>& vecLTNums, int lPntNum, int rPntNum)
{
	LineTopologies::const_iterator itrLT;
	bool bFlag = false;
	ObjectPoints::const_iterator itrOp1, itrOp2, itrOp3, itrOp4;
	itrOp1 = localObjPnts.begin() + lPntNum;
	itrOp2 = localObjPnts.begin() + rPntNum;

	//Feb 25, 2013
	//the two lines are already connected
	if (itrOp1->pos()==itrOp2->pos()) return true;

	for (int i=0; i<vecLTNums.size(); ++i) {
		itrLT = localLineTops.begin() + vecLTNums[i];
		itrOp3 = localObjPnts.begin() + itrLT->at(0).Number();
		itrOp4 = localObjPnts.begin() + itrLT->at(1).Number();

		if (itrOp1->Position3DRef()==itrOp3->Position3DRef()
			&& itrOp2->Position3DRef()==itrOp4->Position3DRef())
			bFlag = true;
		else if (itrOp1->Position3DRef()==itrOp4->Position3DRef()
			&& itrOp2->Position3DRef()==itrOp3->Position3DRef())
			bFlag = true;
	}

	return bFlag;
}


bool ComputeNextLineDir(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts,
	const Plane& plane, const Position3D& curPoint, const Vector3D& refDir, 
	int longerFlag, Vector3D& outDir)
{
	double angErr = 10.0*PI/180.0;
	//for debug
	if (plane.Number() == 3){
		int aaa = 0;
	}

	vector<Vector3D> vecDir(4);
	if (plane.IsVertical(angErr)) {
		outDir = Vector3D(0,0,-1);
		return true;
	}
	else if (plane.IsHorizontal(angErr/2)) 
		vecDir[0] = refDir.VectorProduct(plane.Normal());
	else 
		vecDir[0] = Vector3D(0,0,1).VectorProduct(plane.Normal());

	vecDir[0] = vecDir[0].Normalize();
	vecDir[1] = vecDir[0].VectorProduct(plane.Normal());
	vecDir[2] = vecDir[0]*(-1);
	vecDir[3] = vecDir[1]*(-1);
	//make sure vecDir[1] points to sky
	if (vecDir[1].Z() < -0.005) 
		std::swap(vecDir[1], vecDir[3]);


	bool bInializeOutDir = false;
	if ( !refDir.IsHorizontal(angErr)) {
		if (refDir.Z() < 0){//reference direction project to ground
			if (longerFlag == 1 ){ 
				outDir = vecDir[3];
				return true;
			}
			else {
				outDir = vecDir[0];
				bInializeOutDir = true;
			}
		}
		else {//reference direction project to sky
			if (longerFlag == -1 ){ 
				outDir = vecDir[3];
				return true;
			}
			else {
				outDir = vecDir[0];
				bInializeOutDir = true;
			}
		}
	}
	
	//first give an initial value
	//initial direction is perpendicular to reference direction
	//and point to the same direction, ground or sky, of reference direction.
	if (!bInializeOutDir) {
		outDir = vecDir[0];
		if(MyIsParaller(outDir, refDir, angErr))
			outDir = vecDir[1];
		if (!Vector3D(refDir).IsHorizontal(angErr) && outDir.Z()*refDir.Z()<0)
			outDir = vecDir[3];
	}
	
	Vector3D curDir;

	Plane parPlane = Plane(curPoint, plane.Normal().VectorProduct(refDir));
	LaserPoints::const_iterator itrPnt;
	PointNumberList::const_iterator itrPnl;
	Vector3D  temDir, temRefDir;


	double temSpace;
	double lSpaceCount, rSpaceCount;
	bool bFlag = false;
	double minRatio = 2.0;
	double temAng;
	double distance, conf;//confidence
	double maxConf = -10.0;
	for (int iDir=0; iDir<4; ++iDir) {
		curDir = vecDir[iDir];
		temSpace = curDir.DotProduct(parPlane.Normal());


		if ( !refDir.IsHorizontal(angErr)) {
			if (longerFlag==0 || longerFlag==-1) {
				if (fabs(curDir.Z()) > 0.0005) 
					continue;			
			}
		}


		if (!Vector3D(refDir).IsHorizontal(angErr) && curDir.Z()*refDir.Z()<0)
			continue;

		temAng = MyAngle(curDir, refDir);
		if (temAng>PI-angErr)  continue;

		temRefDir = curDir.VectorProduct(plane.Normal());

		lSpaceCount = rSpaceCount = 0;
		conf = 0.0;
		for (itrPnl = pnlSegPnts.begin(); itrPnl!=pnlSegPnts.end(); ++itrPnl) {
			itrPnt = gLaserPnts.begin()+itrPnl->Number();
			if (curPoint.Distance(itrPnt->Position3DRef()) > 3.0)
				continue;
			temDir = itrPnt->Position3DRef()-curPoint;
			temAng = Angle(curDir, temDir);
			if (temAng > 45*PI/180 )
				continue;//out of 45 degree bugger zone
			distance = cos(temAng)*temDir.Length();
			temSpace = temRefDir.DotProduct(temDir);
			if (temSpace>0){
				++lSpaceCount;
				conf += exp(-0.5*pow(distance/0.2,2));
			}
			else{
				++rSpaceCount;
				conf -= exp(-0.5*pow(distance/0.2,2));
			}
		}


		conf = fabs(conf);
		//		printf("line_conf: %.4f\n", fabs(conf));
		if (lSpaceCount > rSpaceCount)
			std::swap(lSpaceCount, rSpaceCount);
		if (lSpaceCount < 3 && rSpaceCount < 3)
			continue;//both half spaces are empty
		if ((lSpaceCount+1)/(rSpaceCount+1) < minRatio){
			outDir = curDir;
			minRatio = (lSpaceCount+1)/(rSpaceCount+1);
		}

	}
	//	printf("\n");

	return true;
}


double SearchNeastPntPair(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops,
	const vector<bool>& vecRepOPnts, int lLTNum, int rLTNum, int& lPntNum, int& rPntNum)
{

	vector<double> vecDist = vector<double>(4, 9999.9);
	const LineTopology& curLTop = localLineTops[lLTNum];
	const LineTopology& temLTop = localLineTops[rLTNum];

	int idxOP11 = curLTop[0].Number();
	int idxOP12 = curLTop[1].Number();
	int idxOP21 = temLTop[0].Number();
	int idxOP22 = temLTop[1].Number();

	bool validP11 = vecRepOPnts[idxOP11];
	bool validP12 = vecRepOPnts[idxOP12];
	bool validP21 = vecRepOPnts[idxOP21];
	bool validP22 = vecRepOPnts[idxOP22];

	if (!validP11 && !validP21)
		vecDist[0] = localObjPnts[idxOP11].Distance(localObjPnts[idxOP21]);
	if (!validP11 && !validP22)
		vecDist[1] = localObjPnts[idxOP11].Distance(localObjPnts[idxOP22]);
	if (!validP12 && !validP21)
		vecDist[2] = localObjPnts[idxOP12].Distance(localObjPnts[idxOP21]);
	if (!validP12 && !validP22)
		vecDist[3] = localObjPnts[idxOP12].Distance(localObjPnts[idxOP22]);

	double minDist = 9999.9;
	int indPntPair;
	//find the nearest point pair
	for (int iPair=0; iPair<4; iPair++) {
		if (vecDist[iPair]<minDist) {
			minDist = vecDist[iPair];
			indPntPair = iPair;
		}
	}

	//make sure first point is stable
	switch (indPntPair)
	{
	case 0: lPntNum=idxOP11; rPntNum=idxOP21; break;
	case 1:	lPntNum=idxOP11; rPntNum=idxOP22; break;
	case 2:	lPntNum=idxOP12; rPntNum=idxOP21; break;
	case 3:	lPntNum=idxOP12; rPntNum=idxOP22; break;
	}

	return minDist;
}


bool SearchNearstLinePair(const ObjectPoints& localObjPnts, LineTopologies& localLineTops,
	const vector<bool>& vecRepOPnts, const vector<int>& vecTLNums, int& lLTNum, int& rLTNum)
{
	Line3D lLine, rLine;
	double dist1, dist2, dist;
	//double dist11, dist12, dist21, dist22;
	vector<double> vecDist(4, 0.0);
	double minDist = 999999.9;
	int idxOP11, idxOP12, idxOP21, idxOP22;
	int lineNum1, lineNum2;
	bool bFindFlag = false;
	bool validP11, validP12, validP21, validP22;
	int lPntNum, rPntNum;

	for (int iLine=0; iLine<vecTLNums.size(); ++iLine) {
		lineNum1 = vecTLNums[iLine];
		LineTopology& curLTop = localLineTops[lineNum1];
		//if (curLTop.Attribute(2) == 1) continue;//stable line
		if (IsLineTopStable(vecRepOPnts, curLTop)) continue;//stable line

		validP11 = vecRepOPnts[curLTop[0].Number()];
		validP12 = vecRepOPnts[curLTop[1].Number()];
		//make sure the first point is stable
		if (!validP11 && validP12) {
			std::swap(curLTop[0], curLTop[1]);
			std::swap(validP11, validP12);
		}
		//if (!vecRepOPnts[curLTop[0].Number()]) std::swap(idxOP11, idxOP12);
		idxOP11 = curLTop[0].Number();
		idxOP12 = curLTop[1].Number();
		lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());

		//search for best suitable line for enclose
		for (int jLine=iLine+1; jLine<vecTLNums.size(); ++jLine) {
			lineNum2 = vecTLNums[jLine];
			LineTopology& temLTop = localLineTops[lineNum2];
			//if (temLTop.Attribute(2) == 1) continue;//stable line
			if (IsLineTopStable(vecRepOPnts, temLTop)) continue;//stable line

			validP21 = vecRepOPnts[temLTop[0].Number()];
			validP22 = vecRepOPnts[temLTop[1].Number()];
			//make sure the first point is stable
			if (!validP21 && validP22) {
				std::swap(temLTop[0], temLTop[1]);
				std::swap(validP21, validP22);
			}
			idxOP21 = temLTop[0].Number();
			idxOP22 = temLTop[1].Number();
			rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());

			if (validP11 && validP21) {//two unstable points
				//dist1 = lLine.DistanceToPoint(localObjPnts[idxOP21].Position3DRef());
				//dist2 = rLine.DistanceToPoint(localObjPnts[idxOP11].Position3DRef());
				//dist = (dist1+dist2)/2;
				//Feb 14, 2013, Biao Xiong
				dist1 = lLine.DistanceToPoint(localObjPnts[idxOP22].Position3DRef());
				dist2 = rLine.DistanceToPoint(localObjPnts[idxOP12].Position3DRef());
				dist = (dist1+dist2)/2;

				//skip already connected line segments
				if (localObjPnts[idxOP11].Position3DRef()==localObjPnts[idxOP21].Position3DRef())
					dist += 999999.9;
			}
			else {//three or four unstable points
				dist = SearchNeastPntPair(localObjPnts, localLineTops, 
					vecRepOPnts, lineNum1, lineNum2, lPntNum, rPntNum);
			}

			//if (IsOnSameLine(localObjPnts, localLineTops, vecTLNums, idxOP11, idxOP21)
			//	&& Angle(lLine.Direction(), rLine.Direction()) > 165*PI/180)
			//	continue;//inverse direction
			if (dist < minDist) {
				minDist = dist;
				bFindFlag = true;
				lLTNum = lineNum1;
				rLTNum = lineNum2;
			}
		}
	}


	if (bFindFlag) {
		SearchNeastPntPair(localObjPnts, localLineTops, vecRepOPnts, lLTNum, rLTNum, lPntNum, rPntNum);

		idxOP11 = localLineTops[lLTNum][0].Number();
		if (!vecRepOPnts[idxOP11] && idxOP11==lPntNum)
			std::swap(localLineTops[lLTNum][0], localLineTops[lLTNum][1]);

		idxOP21 = localLineTops[rLTNum][0].Number();
		if (!vecRepOPnts[idxOP21] && idxOP21==rPntNum)
			std::swap(localLineTops[rLTNum][0], localLineTops[rLTNum][1]);

	}

	return bFindFlag;
}


bool SearchNearstLinePair2(const ObjectPoints& localObjPnts, LineTopologies& localLineTops,
	const vector<bool>& vecRepOPnts, const vector<int>& vecTLNums, int& lLTNum, int& rLTNum)
{
	//LineTopology curLTop, temLTop;
	Line3D lLine, rLine;
	double dist1, dist2, dist;
	//double dist11, dist12, dist21, dist22;
	vector<double> vecDist(4, 0.0);
	double minDist = 999999.9;
	int idxOP11, idxOP12, idxOP21, idxOP22;
	int lineNum1, lineNum2;
	bool bFindFlag = false;
	bool validP11, validP12, validP21, validP22;
	int lPntNum, rPntNum;

	for (int iLine=0; iLine<vecTLNums.size(); ++iLine) {
		lineNum1 = vecTLNums[iLine];
		LineTopology& curLTop = localLineTops[lineNum1];
		//if (curLTop.Attribute(2) == 1) continue;//stable line
		if (IsLineTopStable(vecRepOPnts, curLTop)) continue;//stable line

		validP11 = vecRepOPnts[curLTop[0].Number()];
		validP12 = vecRepOPnts[curLTop[1].Number()];
		//make sure the first point is stable
		if (!validP11 && validP12) {
			std::swap(curLTop[0], curLTop[1]);
			std::swap(validP11, validP12);
		}
		//if (!vecRepOPnts[curLTop[0].Number()]) std::swap(idxOP11, idxOP12);
		idxOP11 = curLTop[0].Number();
		idxOP12 = curLTop[1].Number();
		if(localObjPnts[idxOP11].Distance(localObjPnts[idxOP12])<0.01) continue;
		//lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());

		//search for best suitable line for enclose
		for (int jLine=iLine+1; jLine<vecTLNums.size(); ++jLine) {
			lineNum2 = vecTLNums[jLine];
			LineTopology& temLTop = localLineTops[lineNum2];
			//if (temLTop.Attribute(2) == 1) continue;//stable line
			if (IsLineTopStable(vecRepOPnts, temLTop)) continue;//stable line

			validP21 = vecRepOPnts[temLTop[0].Number()];
			validP22 = vecRepOPnts[temLTop[1].Number()];
			//make sure the first point is stable
			if (!validP21 && validP22) {
				std::swap(temLTop[0], temLTop[1]);
				std::swap(validP21, validP22);
			}
			idxOP21 = temLTop[0].Number();
			idxOP22 = temLTop[1].Number();
			if(localObjPnts[idxOP21].Distance(localObjPnts[idxOP22])<0.01) continue;

			dist = SearchNeastPntPair(localObjPnts, localLineTops, 
				vecRepOPnts, lineNum1, lineNum2, lPntNum, rPntNum);

			if (dist < minDist) {
				minDist = dist;
				bFindFlag = true;
				lLTNum = lineNum1;
				rLTNum = lineNum2;
			}
		}
	}


	if (bFindFlag) {
		SearchNeastPntPair(localObjPnts, localLineTops, vecRepOPnts, lLTNum, rLTNum, lPntNum, rPntNum);

		idxOP11 = localLineTops[lLTNum][0].Number();
		if (!vecRepOPnts[idxOP11] && idxOP11==lPntNum)
			std::swap(localLineTops[lLTNum][0], localLineTops[lLTNum][1]);

		idxOP21 = localLineTops[rLTNum][0].Number();
		if (!vecRepOPnts[idxOP21] && idxOP21==rPntNum)
			std::swap(localLineTops[rLTNum][0], localLineTops[rLTNum][1]);

	}

	return bFindFlag;
}

IEDll_API bool RefineOutBoundaryLine6(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	if (vecLocalSegPnts.size()<=1) return false;
	typedef ObjectPoints::iterator OPIterator;
	typedef LineTopologies::iterator LTopIterator;
	OPIterator objPnt1, objPnt2;
	vector<bool> vecRepOPnts(localObjPnts.size(), false);
	bool bLeftRepeat, bRightRepeat;

	for (int i=0; i<localLineTops.size(); ++i) {
		objPnt1 = localObjPnts.begin() + localLineTops[i][0].Number();
		objPnt2 = localObjPnts.begin() + localLineTops[i][1].Number();
		bLeftRepeat = bRightRepeat = false;
		for (OPIterator itrOp=localObjPnts.begin(); itrOp!=localObjPnts.end(); ++itrOp) {
			if (objPnt1 != itrOp && objPnt1->X()==itrOp->X()
				&& objPnt1->Y()==itrOp->Y() && objPnt1->Z()==itrOp->Z()) {
					bLeftRepeat = true; 
					vecRepOPnts[itrOp-localObjPnts.begin()] = true;
			}
			else if (objPnt2 != itrOp && objPnt2->X()==itrOp->X()
				&& objPnt2->Y()==itrOp->Y()&&objPnt2->Z()==itrOp->Z()){
					bRightRepeat = true;
					vecRepOPnts[itrOp-localObjPnts.begin()] = true;
			}
		}

		if (bLeftRepeat && bRightRepeat)
			localLineTops[i].SetAttribute(2, 1);//stable line
		else
			localLineTops[i].SetAttribute(2, -1);//un-stable line
	}

	int lPlaneNum, rPlaneNum, lPlaneInd, rPlaneInd, lSegInd, rSegInd;
	Line3D curLine, lLine, rLine, lAuxLine, rAuxLine;
	//Vector3D curDir(1,1,1),lAuxDir(1,1,1), rAuxDir(1,1,1);
	Position3D lTemPos, rTemPos, startPos, endPos;
	typedef LaserPoints::iterator LPntIterator;
	LPntIterator temLPnt;

	Position3D 	posEndPnt;
	LineTopology temLineTop;
	ObjectPoint temObjPnt;

	int orgLineNum = localLineTops.size();
	vector<int> vecNodeInds;
	vecNodeInds.reserve(20);
	for (int i=0; i<orgLineNum; ++i) {
		if(localLineTops[i].Attribute(2) == 1) continue;
		lPlaneNum = localLineTops[i].Attribute(SegmentLabel);
		rPlaneNum = localLineTops[i].Attribute(SecondSegmentLabel);
		if ((lPlaneNum == 3 && rPlaneNum == 1)
			||(lPlaneNum == 1 && rPlaneNum == 3))
		{
			int aaa = 0;
		}

		lPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, lPlaneNum);
		rPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, rPlaneNum);
		lSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, lPlaneNum);
		rSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, rPlaneNum);
		if (lSegInd==-1 || rSegInd==-1) continue;
		//objPnt1 = localObjPnts.begin() + localLineTops[i][0].Number();
		//objPnt2 = localObjPnts.begin() + localLineTops[i][1].Number();
		bLeftRepeat = vecRepOPnts[localLineTops[i][0].Number()];
		bRightRepeat = vecRepOPnts[localLineTops[i][1].Number()];


		vecNodeInds.clear();
		if (!bLeftRepeat && !bRightRepeat) {//no stable node point
			vecNodeInds.push_back(localLineTops[i][0].Number());
			vecNodeInds.push_back(localLineTops[i][1].Number());
			vecNodeInds.push_back(localLineTops[i][1].Number());
			vecNodeInds.push_back(localLineTops[i][0].Number());
		}
		else if (bLeftRepeat  && !bRightRepeat){//left node is stable
			vecNodeInds.push_back(localLineTops[i][0].Number());
			vecNodeInds.push_back(localLineTops[i][1].Number());
		}
		else if (!bLeftRepeat  && bRightRepeat){//right node is stable
			vecNodeInds.push_back(localLineTops[i][1].Number());
			vecNodeInds.push_back(localLineTops[i][0].Number());
		}


		for (int iNode=0; iNode<vecNodeInds.size(); iNode+=2) {
			double dis1, dis2;
			int longerFlag = 0;//-1:right longer then left; 0: equal; 1 left longer then right
			objPnt1 = localObjPnts.begin() + vecNodeInds[iNode+0];
			objPnt2 = localObjPnts.begin() + vecNodeInds[iNode+1];
			curLine = Line3D(*objPnt1, *objPnt2);

			if (lPlaneInd >= 0)
				lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[lSegInd], 
				vecLocalPlanes[lPlaneInd], curLine);
			if (rPlaneInd >= 0) 
				rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[rSegInd], 
				vecLocalPlanes[rPlaneInd], curLine);

			if (lPlaneInd>=0 && rPlaneInd>=0) {
				dis1 = lTemPos.Distance(objPnt1->Position3DRef());
				dis2 = rTemPos.Distance(objPnt1->Position3DRef());


				if (dis1>dis2) {
					longerFlag = 1;
					if (dis1-dis2<0.5) {
						posEndPnt = lTemPos;
						longerFlag = 0;
					}
					else 
						posEndPnt = rTemPos;
				}
				else {
					longerFlag = -1;
					if (dis2-dis1<0.5) {
						posEndPnt = rTemPos;
						longerFlag = 0;
					}
					else 
						posEndPnt = lTemPos;
				}
			

			}

			Vector3D temDir;
			if (lPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal());
				ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[lSegInd],
					vecLocalPlanes[lPlaneInd],posEndPnt,curLine.Direction(), 
					longerFlag, temDir);
				lLine = Line3D(posEndPnt,temDir);
			}
			if (rPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(vecLocalPlanes[rPlaneInd].Normal());
				ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[rSegInd],
					vecLocalPlanes[rPlaneInd],posEndPnt,curLine.Direction(),
					-longerFlag, temDir);
				rLine = Line3D(posEndPnt,temDir);
			}


			if (objPnt1->Distance(posEndPnt) > 0.005)
				objPnt2->Position3DRef() = posEndPnt;
			vecRepOPnts[objPnt2-localObjPnts.begin()] = true;
			localLineTops[i].SetAttribute(2, 1);//stable


			if (lPlaneInd >= 0){
				AddTopLine(localObjPnts, localLineTops, lLine.FootPoint(), 
					lLine.FootPoint()+lLine.Direction(), lPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}

			if (rPlaneInd >= 0) {
				AddTopLine(localObjPnts, localLineTops, rLine.FootPoint(), 
					rLine.FootPoint()+rLine.Direction(), rPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}
		}
	}
	

	vector<int> vecTLNums;
	LineTopology curLTop, temLTop, lLTop, rLTop;
	int idxOP11, idxOP12, idxOP21, idxOP22;
	int lLTNum, rLTNum;
	int nSearchNum;
	//LineTopologies::iterator itrLTop;
	for (int iSeg=0; iSeg<vecLocalSegPnts.size(); ++iSeg) {
		lPlaneNum = gLaserPnts[vecLocalSegPnts[iSeg][0].Number()].Attribute(SegmentNumberTag);
		lPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, lPlaneNum);

		bool bNewLine = true;
		int nLoops = 0;

		while (bNewLine && nLoops<2 ){
			bNewLine = false;
			nLoops++;

			nSearchNum = 0;
			//for (;nSearchNum<5;) {
			while(nSearchNum < 5) {
				nSearchNum++;
				SearchLinesOnPlane(localLineTops, lPlaneNum, vecTLNums);
				if (!SearchNearstLinePair(localObjPnts, localLineTops,vecRepOPnts, vecTLNums, lLTNum, rLTNum)) 
					break;

				curLTop = localLineTops[lLTNum];
				idxOP11 = curLTop[0].Number();
				idxOP12 = curLTop[1].Number();
				//make sure the first point is stable
				if (!vecRepOPnts[curLTop[0].Number()]) std::swap(idxOP11, idxOP12);
				lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());

				temLTop = localLineTops[rLTNum];
				idxOP21 = temLTop[0].Number();
				idxOP22 = temLTop[1].Number();
				//make sure the first point is stable
				if (!vecRepOPnts[temLTop[0].Number()]) std::swap(idxOP21, idxOP22);
				rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());


				bool bIntersect = true;
				bool bParallel = MyIsParaller(lLine.Direction(), rLine.Direction(), 15*PI/180);
				if (!bParallel) {
					Intersection2Lines(lLine, rLine, posEndPnt);
					double dist = (posEndPnt.Distance(localObjPnts[idxOP11].Position3DRef()) +
						posEndPnt.Distance(localObjPnts[idxOP21].Position3DRef()) )/2;
					if (dist>100) bIntersect = false;//too far away;
				}

				//if (MyIsParaller(lLine.Direction(), rLine.Direction(), 15*PI/180)) {//Parallel
				if (bParallel) {//Parallel
					double dist1 = lLine.DistanceToPoint(localObjPnts[idxOP21].Position3DRef());
					double dist2 = rLine.DistanceToPoint(localObjPnts[idxOP11].Position3DRef());
					double dist = dist1<dist2?dist1:dist2;
					if (dist<0.3) {//very near,combine them
						localObjPnts[idxOP12].Position3DRef() = localObjPnts[idxOP21].Position3DRef();
						localObjPnts[idxOP22].Position3DRef() = localObjPnts[idxOP11].Position3DRef();

						vecRepOPnts[idxOP12] = true;
						vecRepOPnts[idxOP22] = true;
						localLineTops[lLTNum].SetAttribute(2,1);
						localLineTops[rLTNum].SetAttribute(2,1);
					}
					else {//
						double lCoef = ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[iSeg],
							vecLocalPlanes[lPlaneInd], lLine, dist<1.5?dist:1.5);
						double rCoef = ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[iSeg],
							vecLocalPlanes[lPlaneInd], rLine, dist<1.5?dist:1.5);

						if (lCoef<rCoef) {//make left line stabler then right line
							std::swap(idxOP11, idxOP21);
							std::swap(idxOP12, idxOP22);
							std::swap(lCoef, rCoef);
							std::swap(lLTNum, rLTNum);
						}


						rCoef = 1.0;// biao xiong 10 July, 2011, assume all 
						if (rCoef < 0.6 )	{// assume a perpendicular line from its end point
							lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
							rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), 
								lLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal()));
							Intersection2Lines(lLine, rLine, posEndPnt);
							localObjPnts[idxOP12].Position3DRef() = posEndPnt;
							localObjPnts[idxOP22].Position3DRef() = posEndPnt;
							vecRepOPnts[idxOP12] = true;
							vecRepOPnts[idxOP22] = true;
							localLineTops[lLTNum].SetAttribute(2,1);
							localLineTops[rLTNum].SetAttribute(2,1);

						}
						else {//assume a new line to connect them
							lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
							rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());
							bool bSameDirect = (lLine.Direction().DotProduct(rLine.Direction())>0);
							bool bLinkedByOneLine = IsOnSameLine(localObjPnts,localLineTops,vecTLNums,idxOP11,idxOP21);
							if (bSameDirect) {
								lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
								rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
								localObjPnts[idxOP12].Position3DRef() = lTemPos;
								localObjPnts[idxOP22].Position3DRef() = rTemPos;
								vecRepOPnts[idxOP12] = true;
								vecRepOPnts[idxOP22] = true;
								localLineTops[lLTNum].SetAttribute(2,1);
								localLineTops[rLTNum].SetAttribute(2,1);	
								//add a new line
								AddTopLine(localObjPnts, localLineTops, lTemPos, rTemPos, lPlaneNum, -1, 1);
								vecRepOPnts.push_back(true);
								vecRepOPnts.push_back(true);
							}
							else {//inverse direction, 
								//linked by one line, expand them and find their next direction
								if (bLinkedByOneLine) {
									lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
									rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
									localObjPnts[idxOP12].Position3DRef() = lTemPos;
									localObjPnts[idxOP22].Position3DRef() = rTemPos;
									vecRepOPnts[idxOP12] = true;
									vecRepOPnts[idxOP22] = true;
									localLineTops[lLTNum].SetAttribute(2,1);
									localLineTops[rLTNum].SetAttribute(2,1);

									Vector3D temDir;
									ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
										vecLocalPlanes[lPlaneInd], lTemPos, lLine.Direction(), 2, temDir);
									AddTopLine(localObjPnts,localLineTops, lTemPos, lTemPos+temDir, lPlaneNum, -1, -1);
									ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
										vecLocalPlanes[lPlaneInd], rTemPos, rLine.Direction(), 2, temDir);
									AddTopLine(localObjPnts,localLineTops, rTemPos, rTemPos+temDir, lPlaneNum, -1, -1);
									vecRepOPnts.push_back(true);
									vecRepOPnts.push_back(false);
									vecRepOPnts.push_back(true);
									vecRepOPnts.push_back(false);
									bNewLine = true;
								}
								else {//change direction of right line, as it is less stable
									lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
									rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), 
										lLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal()));
									Intersection2Lines(lLine, rLine, posEndPnt);
									localObjPnts[idxOP12].Position3DRef() = posEndPnt;
									localObjPnts[idxOP22].Position3DRef() = posEndPnt;
									vecRepOPnts[idxOP12] = true;
									vecRepOPnts[idxOP22] = true;
									localLineTops[lLTNum].SetAttribute(2,1);
									localLineTops[rLTNum].SetAttribute(2,1);
								}
							}
						}


					}
				} 
				//else {
				else if(!bParallel&&bIntersect){//nonparallel, intersect them

					if (!IsOnSameLine(localObjPnts, localLineTops, vecTLNums, idxOP11, idxOP21)){
						Intersection2Lines(lLine, rLine, posEndPnt);
						localObjPnts[idxOP12].Position3DRef() = posEndPnt;
						localObjPnts[idxOP22].Position3DRef() = posEndPnt;
					}
					else {
						lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
						rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());
						lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
						rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
						localObjPnts[idxOP12].Position3DRef() = lTemPos;
						localObjPnts[idxOP22].Position3DRef() = rTemPos;
						Vector3D temDir;
						ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
							vecLocalPlanes[lPlaneInd], lTemPos, lLine.Direction(), 2, temDir);
						AddTopLine(localObjPnts,localLineTops, lTemPos, lTemPos+temDir, lPlaneNum, -1, -1);
						ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
							vecLocalPlanes[lPlaneInd], rTemPos, rLine.Direction(),2, temDir);
						AddTopLine(localObjPnts,localLineTops, rTemPos, rTemPos+temDir, lPlaneNum, -1, -1);
						vecRepOPnts.push_back(true);
						vecRepOPnts.push_back(false);
						vecRepOPnts.push_back(true);
						vecRepOPnts.push_back(false);
						bNewLine = true;
					}

					vecRepOPnts[idxOP12] = true;
					vecRepOPnts[idxOP22] = true;
					localLineTops[lLTNum].SetAttribute(2,1);
					localLineTops[rLTNum].SetAttribute(2,1);	
				}
			}//end nearest neighbor lines
		}//end while loop
	}//end for iSeg



   return true;
}



IEDll_API bool RefineOutBoundaryLine7(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{

	typedef ObjectPoints::iterator OPIterator;
	typedef LineTopologies::iterator LTopIterator;
	OPIterator objPnt1, objPnt2;
	vector<bool> vecRepOPnts(localObjPnts.size(), false);
	bool bLeftRepeat, bRightRepeat;

	for (int i=0; i<localLineTops.size(); ++i) {
		//if(localLineTops[i].Attribute(2) == 1) continue;//processed
		objPnt1 = localObjPnts.begin() + localLineTops[i][0].Number();
		objPnt2 = localObjPnts.begin() + localLineTops[i][1].Number();
		bLeftRepeat = bRightRepeat = false;
		for (OPIterator itrOp=localObjPnts.begin(); itrOp!=localObjPnts.end(); ++itrOp) {
			if (objPnt1!=itrOp && objPnt1->pos()==itrOp->pos()) {
				bLeftRepeat = true; 
				//vecRepOPnts[itrOp-localObjPnts.begin()] = true;
				vecRepOPnts[localLineTops[i][0].Number()] = true;
			}
			else if (objPnt2!=itrOp && objPnt2->pos()==itrOp->pos()){
				bRightRepeat = true;
				vecRepOPnts[localLineTops[i][1].Number()] = true;
				//vecRepOPnts[itrOp-localObjPnts.begin()] = true;
			}
			if (bLeftRepeat&&bRightRepeat) break;
		}

		if (IsLineTopStable(vecRepOPnts, localLineTops[i]))
			localLineTops[i].SetAttribute(2, 1);//stable line
		else
			localLineTops[i].SetAttribute(2, 1);//stable line;
		//if (bLeftRepeat && bRightRepeat)
		//	localLineTops[i].SetAttribute(2, 1);//stable line
		//else
		//	localLineTops[i].SetAttribute(2, -1);//un-stable line
	}
	

	int lPlaneNum, rPlaneNum, lPlaneInd, rPlaneInd, lSegInd, rSegInd;
	Line3D curLine, lLine, rLine, lAuxLine, rAuxLine;
	//Vector3D curDir(1,1,1),lAuxDir(1,1,1), rAuxDir(1,1,1);
	Position3D lTemPos, rTemPos, startPos, endPos;
	typedef LaserPoints::iterator LPntIterator;
	LPntIterator temLPnt;
	//	double scalar, maxRScal, maxLScal;
	Position3D 	posEndPnt;
	LineTopology temLineTop;
	ObjectPoint temObjPnt;
//	bool bVerHorLine;
	int orgLineNum = localLineTops.size();
	vector<int> vecNodeInds;
	vecNodeInds.reserve(20);
	for (int i=0; i<orgLineNum; ++i) {
		//if(localLineTops[i].Attribute(2) == 1) continue;
		if (IsLineTopStable(vecRepOPnts, localLineTops[i])) continue;
		lPlaneNum = localLineTops[i].Attribute(SegmentLabel);
		rPlaneNum = localLineTops[i].Attribute(SecondSegmentLabel);
#ifdef _DEBUG
		if ((lPlaneNum==568&&rPlaneNum==567)||
			(lPlaneNum==567&&rPlaneNum==568))
		{
			int aaa =0;
		}
#endif

		lPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, lPlaneNum);
		rPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, rPlaneNum);
		lSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, lPlaneNum);
		rSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, rPlaneNum);
		if (lSegInd==-1 || rSegInd==-1) continue;

		bLeftRepeat = vecRepOPnts[localLineTops[i][0].Number()];
		bRightRepeat = vecRepOPnts[localLineTops[i][1].Number()];


		vecNodeInds.clear();
		if (!bRightRepeat){//right node is unstable
			vecNodeInds.push_back(localLineTops[i][0].Number());
			vecNodeInds.push_back(localLineTops[i][1].Number());
		}
		if (!bLeftRepeat){//left node is unstable
			vecNodeInds.push_back(localLineTops[i][1].Number());
			vecNodeInds.push_back(localLineTops[i][0].Number());
		}


		for (int iNode=0; iNode<vecNodeInds.size(); iNode+=2) {
			double dis1, dis2;
			int longerFlag = 0;//-1:right longer then left; 0: equal; 1 left longer then right
			objPnt1 = localObjPnts.begin() + vecNodeInds[iNode+0];
			objPnt2 = localObjPnts.begin() + vecNodeInds[iNode+1];
			curLine = Line3D(*objPnt1, *objPnt2);
			LineSegment3D curLineSeg = LineSegment3D(*objPnt1, *objPnt2);

			if (lPlaneInd >= 0) {
				lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[lSegInd], 
					vecLocalPlanes[lPlaneInd], curLine);

				vector<int> vecTLNums;
				SearchLinesOnPlane(localLineTops, lPlaneNum, vecTLNums);
				Position3D temEndPos = ComputeEndPnt(localObjPnts, localLineTops, vecTLNums, curLineSeg);
				dis1 = temEndPos.Distance(lTemPos);
				if (temEndPos.Distance(lTemPos)<1.5)//line intersection is more stable
					lTemPos = temEndPos;

				posEndPnt = lTemPos;
			}
			if (rPlaneInd >= 0) {
				rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[rSegInd], 
					vecLocalPlanes[rPlaneInd], curLine);

				vector<int> vecTLNums;
				SearchLinesOnPlane(localLineTops, rPlaneNum, vecTLNums);
				Position3D temEndPos = ComputeEndPnt(localObjPnts, localLineTops, vecTLNums, curLineSeg);
				//if (temEndPos.Distance(*objPnt2)<lTemPos.Distance(*objPnt2))
				dis1 = temEndPos.Distance(rTemPos);
				if (temEndPos.Distance(rTemPos)<1.5)//line intersection is more stable
					rTemPos = temEndPos;

				posEndPnt = rTemPos;
			}


			if (lPlaneInd>=0&&rPlaneInd>=0) {
				dis1 = lTemPos.Distance(objPnt1->Position3DRef());
				dis2 = rTemPos.Distance(objPnt1->Position3DRef());

				posEndPnt = dis1<dis2?lTemPos:rTemPos;
				if (fabs(dis1-dis2)>0.5)
					longerFlag = dis1<dis2?-1:1;
				else longerFlag = 0;
			}
			//Feb 17, 2013, Biao, Do not change the end point
			//posEndPnt = objPnt2->Position3DRef();

			Vector3D temDir;
			if (lPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal());
				ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[lSegInd],
					vecLocalPlanes[lPlaneInd],posEndPnt,curLine.Direction(), 
					longerFlag, temDir);
				lLine = Line3D(posEndPnt,temDir);
			}
			if (rPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(vecLocalPlanes[rPlaneInd].Normal());
				ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[rSegInd],
					vecLocalPlanes[rPlaneInd],posEndPnt,curLine.Direction(),
					-longerFlag, temDir);
				rLine = Line3D(posEndPnt,temDir);
			}


			if (objPnt1->Distance(posEndPnt) > 0.005)
				objPnt2->Position3DRef() = posEndPnt;
			vecRepOPnts[objPnt2-localObjPnts.begin()] = true;



			if (lPlaneInd >= 0){
				AddTopLine(localObjPnts, localLineTops, lLine.FootPoint(), 
					lLine.FootPoint()+0.1*lLine.Direction(), lPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}

			if (rPlaneInd >= 0) {
				AddTopLine(localObjPnts, localLineTops, rLine.FootPoint(), 
					rLine.FootPoint()+0.1*rLine.Direction(), rPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}
		}
	}
	
	

	vector<int> vecTLNums;
	LineTopology curLTop, temLTop, lLTop, rLTop;
	int idxOP11, idxOP12, idxOP21, idxOP22;
	int lLTNum, rLTNum;
	int nSearchNum;

	for (int iSeg=0; iSeg<vecLocalSegPnts.size(); ++iSeg) {
		lPlaneNum = gLaserPnts[vecLocalSegPnts[iSeg][0].Number()].Attribute(SegmentNumberTag);
		lPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, lPlaneNum);
		SearchLinesOnPlane(localLineTops, lPlaneNum, vecTLNums);
#ifdef _DEBUG
		if (lPlaneNum==1135)	{
			LineTopologies debugLines;
			for (int debui=0; debui<vecTLNums.size(); debui++)
				debugLines.push_back(localLineTops[vecTLNums[debui]]);
			debugLines.Write("debug.top");
			localObjPnts.Write("debug.objpts");
		}
#endif


		int loop=0;
		while(++loop<500 && SearchNearstLinePair2(localObjPnts, 
			localLineTops, vecRepOPnts, vecTLNums, lLTNum, rLTNum))
		{
			curLTop = localLineTops[lLTNum];
			idxOP11 = curLTop[0].Number();
			idxOP12 = curLTop[1].Number();
			//make sure the first point is stable
			//if (!vecRepOPnts[curLTop[0].Number()]) std::swap(idxOP11, idxOP12);
			lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());

			temLTop = localLineTops[rLTNum];
			idxOP21 = temLTop[0].Number();
			idxOP22 = temLTop[1].Number();
			//make sure the first point is stable
			//if (!vecRepOPnts[temLTop[0].Number()]) std::swap(idxOP21, idxOP22);
			rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());

			if (lLTNum==12&&rLTNum==14) {
				int aaaa =0;
			}

			bool bIntersect = true;
			bool bParallel = MyIsParaller(lLine.Direction(), rLine.Direction(), 10*PI/180);
			if (!bParallel) {
				Intersection2Lines(lLine, rLine, posEndPnt);

				double dist = (posEndPnt.Distance(localObjPnts[idxOP12].Position3DRef()) +
					posEndPnt.Distance(localObjPnts[idxOP22].Position3DRef()) )/2;
				if (dist>15.0) bIntersect = false;//too far away;
			}

			//if (MyIsParaller(lLine.Direction(), rLine.Direction(), 15*PI/180)) {//Parallel
			if (bParallel) {//Parallel
				double dist1 = lLine.DistanceToPoint(localObjPnts[idxOP21].Position3DRef());
				double dist2 = rLine.DistanceToPoint(localObjPnts[idxOP11].Position3DRef());
				double dist = dist1<dist2?dist1:dist2;
				if (dist<0.15) {//very near,combine them
					localObjPnts[idxOP12].Position3DRef() = localObjPnts[idxOP21].Position3DRef();
					localObjPnts[idxOP22].Position3DRef() = localObjPnts[idxOP11].Position3DRef();

			
					vecRepOPnts[idxOP11] = (vecRepOPnts[idxOP11]||vecRepOPnts[idxOP22]);
					vecRepOPnts[idxOP12] = (vecRepOPnts[idxOP12]||vecRepOPnts[idxOP21]);
					vecRepOPnts[idxOP21] = true;
					vecRepOPnts[idxOP22] = true;

				}
				else {//
					double lCoef = ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[iSeg],
						vecLocalPlanes[lPlaneInd], lLine, dist<1.5?dist:1.5);
					double rCoef = ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[iSeg],
						vecLocalPlanes[lPlaneInd], rLine, dist<1.5?dist:1.5);

					bool bLeftStable = true;
					if (!vecLocalPlanes[lPlaneInd].IsHorizontal(20*PI/180))	{
						if (fabs(lLine.Direction().Z())< 0.05 &&
							localObjPnts[idxOP11].Z()>localObjPnts[idxOP21].Z())
							bLeftStable = false;
					}
					else {
						if (lCoef<rCoef) bLeftStable = false;
					}
					
					//make left line stabler then right line
					if (!bLeftStable) {
						std::swap(idxOP11, idxOP21);
						std::swap(idxOP12, idxOP22);
						std::swap(lCoef, rCoef);
						std::swap(lLTNum, rLTNum);
					}


					lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
					rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());
					bool bSameDirect = (lLine.Direction().DotProduct(rLine.Direction())>0);
					bool bLinkedByOneLine = IsOnSameLine(localObjPnts,localLineTops,vecTLNums,idxOP11,idxOP21);
					if (bSameDirect) {
						lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
						rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
						localObjPnts[idxOP12].Position3DRef() = lTemPos;
						localObjPnts[idxOP22].Position3DRef() = rTemPos;
						vecRepOPnts[idxOP12] = true;
						vecRepOPnts[idxOP22] = true;
						//localLineTops[lLTNum].SetAttribute(2,1);
						//localLineTops[rLTNum].SetAttribute(2,1);	
						//add a new line
						AddTopLine(localObjPnts, localLineTops, lTemPos, rTemPos, lPlaneNum, -1, 1);
						vecRepOPnts.push_back(true);
						vecRepOPnts.push_back(true);
					}
					else {
						if (bLinkedByOneLine) {
							lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
							rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
							localObjPnts[idxOP12].Position3DRef() = lTemPos;
							localObjPnts[idxOP22].Position3DRef() = rTemPos;
							vecRepOPnts[idxOP12] = true;
							vecRepOPnts[idxOP22] = true;


							Vector3D temDir;
							ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
								vecLocalPlanes[lPlaneInd], lTemPos, lLine.Direction(), 2, temDir);
							AddTopLine(localObjPnts,localLineTops, lTemPos, lTemPos+temDir, lPlaneNum, -1, -1);
							ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
								vecLocalPlanes[lPlaneInd], rTemPos, rLine.Direction(), 2, temDir);
							AddTopLine(localObjPnts,localLineTops, rTemPos, rTemPos+temDir, lPlaneNum, -1, -1);
							vecRepOPnts.push_back(true);
							vecRepOPnts.push_back(false);
							vecRepOPnts.push_back(true);
							vecRepOPnts.push_back(false);
						}
						else {

							if (vecLocalPlanes[lPlaneInd].IsHorizontal(20.0*PI/180)
								|| (localObjPnts[idxOP12].Distance(localObjPnts[idxOP11])>1.0&&
								localObjPnts[idxOP22].Distance(localObjPnts[idxOP21])>1.0)) {
								rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());
								lLine = Line3D(localObjPnts[idxOP12].Position3DRef(),
									rLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal()));
								Intersection2Lines(lLine, rLine, posEndPnt);
								localObjPnts[idxOP22].Position3DRef() = posEndPnt;
								AddTopLine(localObjPnts,localLineTops, localObjPnts[idxOP12], posEndPnt, lPlaneNum, -1, -1);
								vecRepOPnts[idxOP12] = true;
								vecRepOPnts[idxOP22] = true;
								vecRepOPnts.push_back(true);
								vecRepOPnts.push_back(true);
							}
							//change direction of right line, as it is less stable
							else {
								lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
								rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), 
									lLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal()));
								Intersection2Lines(lLine, rLine, posEndPnt);
								localObjPnts[idxOP12].Position3DRef() = posEndPnt;
								localObjPnts[idxOP22].Position3DRef() = posEndPnt;
								vecRepOPnts[idxOP12] = true;
								vecRepOPnts[idxOP22] = true;
							}
						}
					}
				}
			} 
			//else {
			else if(!bParallel){
				if (bIntersect&&!IsOnSameLine(localObjPnts,localLineTops,vecTLNums,idxOP11,idxOP21)){
					Intersection2Lines(lLine, rLine, posEndPnt);
					localObjPnts[idxOP12].Position3DRef() = posEndPnt;
					localObjPnts[idxOP22].Position3DRef() = posEndPnt;
				}
				else {
					lLine = Line3D(localObjPnts[idxOP11].Position3DRef(), localObjPnts[idxOP12].Position3DRef());
					rLine = Line3D(localObjPnts[idxOP21].Position3DRef(), localObjPnts[idxOP22].Position3DRef());
					lTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], lLine);
					rTemPos = ComputeEndPnt(gLaserPnts,vecLocalSegPnts[iSeg],vecLocalPlanes[lPlaneInd], rLine); 
					localObjPnts[idxOP12].Position3DRef() = lTemPos;
					localObjPnts[idxOP22].Position3DRef() = rTemPos;
					Vector3D temDir;
					ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
						vecLocalPlanes[lPlaneInd], lTemPos, lLine.Direction(), 2, temDir);
					AddTopLine(localObjPnts,localLineTops, lTemPos, lTemPos+temDir, lPlaneNum, -1, -1);
					ComputeNextLineDir(gLaserPnts, vecLocalSegPnts[iSeg],
						vecLocalPlanes[lPlaneInd], rTemPos, rLine.Direction(),2, temDir);
					AddTopLine(localObjPnts,localLineTops, rTemPos, rTemPos+temDir, lPlaneNum, -1, -1);
					vecRepOPnts.push_back(true);
					vecRepOPnts.push_back(false);
					vecRepOPnts.push_back(true);
					vecRepOPnts.push_back(false);
				}

				vecRepOPnts[idxOP12] = true;
				vecRepOPnts[idxOP22] = true;
				//if(vecRepOPnts[idxOP11]) localLineTops[lLTNum].SetAttribute(2,1);
				//if(vecRepOPnts[idxOP21]) localLineTops[rLTNum].SetAttribute(2,1);	
			}

			SearchLinesOnPlane(localLineTops, lPlaneNum, vecTLNums);
		}//end while loop 
	}//end for iSeg
	/**/
   return true;
}



bool RefineOutBoundaryLine5(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{

	typedef ObjectPoints::iterator OPIterator;
	typedef LineTopologies::iterator LTopIterator;
	OPIterator objPnt1, objPnt2;
	bool bLeftRepeat, bRightRepeat;
	vector<bool> vecRepOPnts(localObjPnts.size(), false);
	for (int i=0; i<localLineTops.size(); ++i) {
		if(localLineTops[i].Attribute(2) == 1) continue;//processed
		objPnt1 = localObjPnts.begin() + localLineTops[i][0].Number();
		objPnt2 = localObjPnts.begin() + localLineTops[i][1].Number();
		bLeftRepeat = bRightRepeat = false;
		for (OPIterator itrOp=localObjPnts.begin(); itrOp!=localObjPnts.end(); ++itrOp) {
			if (objPnt1 != itrOp && objPnt1->X()==itrOp->X()
				&& objPnt1->Y()==itrOp->Y() && objPnt1->Z()==itrOp->Z()) {
					bLeftRepeat = true; 
					vecRepOPnts[itrOp-localObjPnts.begin()] = true;
			}
			else if (objPnt2 != itrOp && objPnt2->X()==itrOp->X()
				&& objPnt2->Y()==itrOp->Y()&&objPnt2->Z()==itrOp->Z()){
					bRightRepeat = true;
					vecRepOPnts[itrOp-localObjPnts.begin()] = true;
			}
		}

		if (bLeftRepeat && bRightRepeat)
			localLineTops[i].SetAttribute(2, 1);//stable line
		else
			localLineTops[i].SetAttribute(2, -1);//un-stable line
	}

 
	int lPlaneNum, rPlaneNum, lPlaneInd, rPlaneInd, lSegInd, rSegInd;
	Line3D curLine, lLine, rLine, lAuxLine, rAuxLine;
	Vector3D curDir(1,1,1),lAuxDir(1,1,1), rAuxDir(1,1,1);
	Position3D lTemPos, rTemPos, startPos, endPos;
	typedef LaserPoints::iterator LPntIterator;
	LPntIterator temLPnt;
	double maxRScal, maxLScal;
	Position3D 	posEndPnt;
	LineTopology temLineTop;
	ObjectPoint temObjPnt;
	bool bVerHorLine;
	int orgLineNum = localLineTops.size();
	for (int i=0; i<orgLineNum; ++i) {
		if(localLineTops[i].Attribute(2) == 1) continue;
		lPlaneNum = localLineTops[i].Attribute(SegmentLabel);
		rPlaneNum = localLineTops[i].Attribute(SecondSegmentLabel);
		if (i==1){
			int aaa = 0;
		}

		lPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, lPlaneNum);
		rPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, rPlaneNum);
		lSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, lPlaneNum);
		rSegInd = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, rPlaneNum);
		if (lSegInd==-1 || rSegInd==-1) continue;
		objPnt1 = localObjPnts.begin() + localLineTops[i][0].Number();
		objPnt2 = localObjPnts.begin() + localLineTops[i][1].Number();
		bLeftRepeat = vecRepOPnts[localLineTops[i][0].Number()];
		bRightRepeat = vecRepOPnts[localLineTops[i][1].Number()];
		//transform point 1 and  point 2, make stable point to be start point
		if (!bLeftRepeat && bRightRepeat) 
			std::swap(objPnt1, objPnt2);
		curDir = (Vector3D(*objPnt2-*objPnt1)).Normalize();

		//determine auxiliary line direction
		if (Angle(curDir,Vector3D(0,0,1))<10*PI/180 || curDir.IsHorizontal(10*PI/180))
			bVerHorLine = true;
		else 
			bVerHorLine = false;


		if (!bVerHorLine) {
			if (lPlaneInd >=0) {
				lAuxDir = vecLocalPlanes[lPlaneInd].Normal().VectorProduct(Vector3D(0,0,1));
				lAuxDir = vecLocalPlanes[lPlaneInd].Normal().VectorProduct(lAuxDir);
				lAuxDir = lAuxDir.Normalize();
				if (Angle(lAuxDir, curDir) > PI/2 ) lAuxDir *= -1;//same direction with current line
				//if (lAuxDir.Z()*curDir.Z()<0) lAuxDir *= -1;
				//if (lAuxDir.Z() > 0) lAuxDir *= -1;
			}
			if (rPlaneInd >=0) {
				rAuxDir = vecLocalPlanes[rPlaneInd].Normal().VectorProduct(Vector3D(0,0,1));
				rAuxDir = vecLocalPlanes[rPlaneInd].Normal().VectorProduct(rAuxDir);
				rAuxDir = rAuxDir.Normalize();
				if (Angle(rAuxDir, curDir) > PI/2 ) rAuxDir *= -1;//same direction with current line
				//if (rAuxDir.Z() *curDir.Z()<0) rAuxDir *= -1;
				//if (rAuxDir.Z() > 0) rAuxDir *= -1;
			}
		}


		if (bLeftRepeat || bRightRepeat) {		
			curLine = Line3D(Position3D(*objPnt1), Vector3D(*objPnt2-*objPnt1));
			if (!bVerHorLine){
				lAuxLine = Line3D(Position3D(*objPnt1), lAuxDir);
				rAuxLine = Line3D(Position3D(*objPnt1), rAuxDir);
			}
			else {
				//lAuxDir = curLine.Direction();
				//rAuxDir = curLine.Direction();
				lAuxLine = curLine;
				rAuxLine = curLine;
			}

			if (lPlaneInd >= 0){
				maxLScal = ComputeEndPointScalar(gLaserPnts, vecLocalSegPnts[lSegInd], lAuxLine);
				//Position3D pos1 = lAuxLine.Position(maxLScal);
				//Vector3D vec1 = (lAuxLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal())).Normalize();
				lLine = Line3D(lAuxLine.Position(maxLScal), 
					lAuxLine.Direction().VectorProduct(vecLocalPlanes[lPlaneInd].Normal()));
				Intersection2Lines(lLine, curLine, lTemPos);
			}
			if (rPlaneInd >= 0) {
				maxRScal = ComputeEndPointScalar(gLaserPnts, vecLocalSegPnts[rSegInd], rAuxLine);
				//Position3D pos2 = rAuxLine.Position(maxRScal);
				//Vector3D vec2 = (rAuxLine.Direction().VectorProduct(vecLocalPlanes[rPlaneInd].Normal())).Normalize();
				rLine = Line3D(rAuxLine.Position(maxRScal), 
					rAuxLine.Direction().VectorProduct(vecLocalPlanes[rPlaneInd].Normal()));
				Intersection2Lines(rLine, curLine, rTemPos);
			}

			if (lPlaneInd>=0 && rPlaneInd>=0) {
				double dis1 = lTemPos.Distance(Position3D(*objPnt1));
				double dis2 = rTemPos.Distance(Position3D(*objPnt1));
				if (fabs(dis1-dis2)<0.3)
					posEndPnt = (lTemPos+rTemPos)/2;
				else if (dis1<dis2)
					posEndPnt = lTemPos;
				else
					posEndPnt = rTemPos;
				/*if (lTemPos.Distance(rTemPos)<0.3)
				posEndPnt = (lTemPos+rTemPos)/2;
				else {
				if (lTemPos.Z()<rTemPos.Z())
				posEndPnt = rTemPos;
				else
				posEndPnt = lTemPos;
				}*/
				lLine.FootPoint(posEndPnt);
				rLine.FootPoint(posEndPnt);
			}

			Vector3D temDir = curDir.VectorProduct(vecLocalPlanes[lPlaneInd].Normal());
			ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[lSegInd],
				vecLocalPlanes[lPlaneInd],posEndPnt,curDir,2,temDir);
			lLine = Line3D(posEndPnt,temDir);
			temDir = curDir.VectorProduct(vecLocalPlanes[rPlaneInd].Normal());
			ComputeNextLineDir(gLaserPnts,vecLocalSegPnts[rSegInd],
				vecLocalPlanes[rPlaneInd],posEndPnt,curDir,2,temDir);
			rLine = Line3D(posEndPnt,temDir);


			objPnt2->X() = posEndPnt.X(); 
			objPnt2->Y() = posEndPnt.Y(); 
			objPnt2->Z() = posEndPnt.Z();
			vecRepOPnts[objPnt2-localObjPnts.begin()] = true;
			localLineTops[i].SetAttribute(2, 1);//stable


			Position3D temPos;
			double scalar;
			if (lPlaneInd >= 0){
				temObjPnt.X() = lLine.FootPoint().X();
				temObjPnt.Y() = lLine.FootPoint().Y();
				temObjPnt.Z() = lLine.FootPoint().Z();
				//temObjPnt.X() = posEndPnt.X();
				//temObjPnt.Y() = posEndPnt.Y();
				//temObjPnt.Z() = posEndPnt.Z();
				temObjPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(temObjPnt);
				vecRepOPnts.push_back(true);

				//scalar = ComputeMeanPointScalar(gLaserPnts, vecLocalSegPnts[lSegInd], lLine);
				scalar = ComputeEndPointScalar(gLaserPnts, vecLocalSegPnts[lSegInd], lLine);
				temPos = lLine.Position(scalar);
				//temObjPnt.X() = temPos.X();
				//temObjPnt.Y() = temPos.Y();
				//temObjPnt.Z() = temPos.Z();
				temObjPnt.X() = rLine.FootPoint().X()+lLine.Direction().X();
				temObjPnt.Y() = rLine.FootPoint().Y()+lLine.Direction().Y();
				temObjPnt.Z() = rLine.FootPoint().Z()+lLine.Direction().Z();
				temObjPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(temObjPnt);
				vecRepOPnts.push_back(false);

				temLineTop.clear();
				temLineTop.push_back(PointNumber(localObjPnts.size()-2));
				temLineTop.push_back(PointNumber(localObjPnts.size()-1));
				temLineTop.SetAttribute(SegmentLabel, lPlaneNum);
				temLineTop.SetAttribute(SecondSegmentLabel, -1);
				temLineTop.SetAttribute(2, -1);
				localLineTops.push_back(temLineTop);
			}

			if (rPlaneInd >= 0) {
				temObjPnt.X() = rLine.FootPoint().X();
				temObjPnt.Y() = rLine.FootPoint().Y();
				temObjPnt.Z() = rLine.FootPoint().Z();
				//temObjPnt.X()=posEndPnt.X(); 
				//temObjPnt.Y()=posEndPnt.Y(); 
				//temObjPnt.Z()=posEndPnt.Z();
				temObjPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(temObjPnt);
				vecRepOPnts.push_back(true);

				//scalar = ComputeMeanPointScalar(gLaserPnts, vecLocalSegPnts[rSegInd], rLine);
				scalar = ComputeEndPointScalar(gLaserPnts, vecLocalSegPnts[rSegInd], rLine);
				temPos = rLine.Position(scalar);
				//temObjPnt.X() = temPos.X();
				//temObjPnt.Y() = temPos.Y();
				//temObjPnt.Z() = temPos.Z();
				temObjPnt.X()=rLine.FootPoint().X()+rLine.Direction().X(); 
				temObjPnt.Y()=rLine.FootPoint().Y()+rLine.Direction().Y();
				temObjPnt.Z()=rLine.FootPoint().Z()+rLine.Direction().Z();
				temObjPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(temObjPnt);
				vecRepOPnts.push_back(false);

				temLineTop.clear();
				temLineTop.push_back(PointNumber(localObjPnts.size()-2));
				temLineTop.push_back(PointNumber(localObjPnts.size()-1));
				temLineTop.SetAttribute(SegmentLabel, rPlaneNum);
				temLineTop.SetAttribute(SecondSegmentLabel, -1);
				temLineTop.SetAttribute(2, -1);
				localLineTops.push_back(temLineTop);
			}
		}

		else if (!bLeftRepeat && !rPlaneNum) {
		}
	}

	return true;
}


bool RefineOutBoundaryLine4(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts;
	PointNumberList pnlTemSegPnts;
	int curPlaneNum;
	LaserPoint temLaserPnt;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.4;
	bool bNearEdge;
	Plane curPlane;
	int pntNums;
	int minPntNums = 3;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNums;
	PointNumberList pnlLineSeg;
	LaserPoints selLaserPnts, temLaserPnts;
	Vector3D dir;
	Line2D  line;
	Position2D pos2d1, pos2d2;
	Position3D temPos, pos3d1, pos3d2;
	vector<PointNumberList> vecSubLineSegs;
	vector<double> vecDirection;
	LineTopologies addedLineTops;

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{

		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		int curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curPlaneNum);
		pnlOrgTemBoundPnts.clear();

		if (curPlaneInd == -1) continue;//cannot find suitable plane, skip
		curPlane = vecLocalPlanes[curPlaneInd];

		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);
		MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts, 3.0, &curPlane);


		for (int j=0; j<pnlTemBoundPnts.size(); j++)  {
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];
			temLaserPnt.Position3DRef() = curPlane.Project(temLaserPnt.Position3DRef());

			bNearEdge = false;
			for (int k=0; k<localLineTops.size(); k++)	{
				p1 = localObjPnts[localLineTops[k][0].Number()];
				p2 = localObjPnts[localLineTops[k][1].Number()];
				temLine = Line3D(p1, p2);
				double scale1, scale2, scale;
				scale1 = temLine.Scalar(p1); 
				scale2 = temLine.Scalar(p2);
				if (scale1 > scale2) std::swap(scale1, scale2);
				scale = temLine.Scalar(temLaserPnt.Position3DRef());
				dist = temLine.DistanceToPoint( temLaserPnt.Position3DRef());

				if (dist < minDist && scale > scale1
					&& scale < scale2)	{
						bNearEdge = true;
						break;
				}
			}

			if (!bNearEdge) pnlOrgTemBoundPnts.push_back(pnlTemBoundPnts[j]);
		}

		selLaserPnts.clear();
		for (int j=0; j<pnlOrgTemBoundPnts.size(); j++)	{
			selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
			pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
		}

		if(selLaserPnts.empty()) continue;

		//computer constraints direction
		//has ridges on this surface
		vecDirection.clear();
		Vector3D plumdir = Vector3D(0, 0, 1);
		Vector3D planDir = curPlane.Normal();
		Vector3D dir;
		double direction;

		if (!curPlane.IsHorizontal(5*PI/180)) {
			dir = plumdir.VectorProduct(planDir);
			pos3d1.X() = 0.0; pos3d1.Y() = 0.0; pos3d1.Z() = 0.0;
			pos3d2.X() = dir.X(); pos3d2.Y() = dir.Y(); pos3d2.Z() = dir.Z();
			pos2d1 = Project2RefPlane(curPlane, pos3d1);
			pos2d2 = Project2RefPlane(curPlane, pos3d2);
			direction = atan((pos2d1.Y()-pos2d2.Y())/(pos2d1.X()-pos2d2.X()+0.000001));
			if (direction<0) direction += PI;
			vecDirection.push_back(direction);
			//perpendicular angle
			if (direction>PI/2)
				vecDirection.push_back(direction-PI/2);
			else 
				vecDirection.push_back(direction+PI/2);


		}


		std::sort(vecDirection.begin(), vecDirection.end());
		for (int j=1; j<vecDirection.size(); j++) {
			if (fabs(vecDirection[j]-vecDirection[j-1]) < 3*PI/180)	{
				vecDirection.erase(vecDirection.begin()+j);
				j--;
			}
		}

		temLaserPnts = selLaserPnts;

		for (int j=0; j<selLaserPnts.size(); j++) {
			pos2d1 = Project2RefPlane(curPlane, selLaserPnts[j].Position3DRef());
			temLaserPnts[j].X() = pos2d1.X();
			temLaserPnts[j].Y() = pos2d1.Y();
			temLaserPnts[j].Z() = 0;
		}


		pos3d1 = temLaserPnts[0].Position3DRef();
		pos3d2 = temLaserPnts[temLaserPnts.size()-1].Position3DRef();
		if (pos3d1.X() != pos3d2.X() && pos3d1.Y() != pos3d2.Y())
			temLaserPnts.push_back(temLaserPnts[0]);//enclose


		temLaserPnts.DeriveDataBounds(-1);
		DataBoundsLaser bounds = temLaserPnts.Bounds();
		double maxDistHough = sqrt(bounds.XRange() * bounds.XRange() + bounds.YRange() * bounds.YRange()) / 2.0;
		if ( maxDistHough < 0.1) continue;//this segment is too small
		HoughSpace houghspace;

		if (vecDirection.empty())	{
			houghspace.Initialise(int(60), int(2*maxDistHough/0.1), 0.0, -maxDistHough, PI, maxDistHough, 1);
		}
		else {
			double* pDir = new double[vecDirection.size()];
			for (int j=0; j<vecDirection.size(); j++) 
				pDir[j] = vecDirection[j];
			houghspace.Initialise(vecDirection.size(), pDir, int(2*maxDistHough/0.1), -maxDistHough, maxDistHough);
			delete[] pDir;
		}

		houghspace.SetOffsets(bounds.MidPoint().X(), bounds.MidPoint().Y(), 0.0);
		for (int j=0; j<temLaserPnts.size(); j++)
			houghspace.AddPoint(temLaserPnts[j].Position2DOnly());

		int iteratNum = 0;
		do {
			//pntNums = 0;
			line = houghspace.BestLine(&pntNums, 0, 9); 
			if (pntNums < minPntNums) continue;


			pntNums = 0;
			pnlLineSeg.clear();
			for (int j=0; j<temLaserPnts.size(); j++) {	
				temLaserPnt = temLaserPnts[j];
				dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());
				if (dist > minDist/2) continue;
				pntNums++;
				pnlLineSeg.push_back(PointNumber(j));
			}

			//get sub line segment
			vecSubLineSegs.clear();
			for (int j=0; j<pnlLineSeg.size()-1; j++) {
				int num1 = pnlLineSeg[j].Number();
				int num2 = pnlLineSeg[j+1].Number();
				//there is a sub-line in this line or it is the last two node
				if (j!=pnlLineSeg.size()-2 && abs(num2-num1) <= minPntNums) continue;
				//it is the last two node
				if (j==pnlLineSeg.size()-2 && abs(num2-num1) <= minPntNums
					&& temLaserPnts[num1].Position3DRef().Distance(temLaserPnts[num2].Position3DRef()) < 0.2) 
					j += 1;

				PointNumberList pnlTemSubLine;
				pnlTemSubLine.insert(pnlTemSubLine.begin(), pnlLineSeg.begin(), pnlLineSeg.begin()+j+1);

				if (j==pnlLineSeg.size()-1) 
					pnlLineSeg.erase(pnlLineSeg.begin(), pnlLineSeg.begin()+j);
				else
					pnlLineSeg.erase(pnlLineSeg.begin(), pnlLineSeg.begin()+j+1);

				vecSubLineSegs.push_back(pnlTemSubLine);				
				j=-1;
			}


			for (int iSubLine=0; iSubLine<vecSubLineSegs.size(); iSubLine++) {
				PointNumberList pnlTemSubLine = vecSubLineSegs[iSubLine];


				double scalar, minScalar = 9999999, maxScalar = -99999999;
				for (int j=0; j<pnlTemSubLine.size(); j++)	{
					temLaserPnt = temLaserPnts[pnlTemSubLine[j].Number()];
					houghspace.RemovePoint(temLaserPnt.Position2DOnly());
					scalar = line.Scalar(temLaserPnt.Position2DOnly());
					if (scalar < minScalar) minScalar = scalar;
					if (scalar > maxScalar) maxScalar = scalar;
				}

				pos2d1 = line.Position(minScalar);
				pos2d2 = line.Position(maxScalar);
				pos3d1 = Repoject2OrgPlane(curPlane, pos2d1);
				pos3d2 = Repoject2OrgPlane(curPlane, pos2d2);


				ObjectPoint objPnt;
				LineTopology lineTop;

				objPnt.X() = pos3d1.X(); objPnt.Y() = pos3d1.Y(); objPnt.Z() = pos3d1.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				objPnt.X() = pos3d2.X(); objPnt.Y() = pos3d2.Y(); objPnt.Z() = pos3d2.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				lineTop.Number() = localLineTops.size();
				lineTop.SetAttribute(SegmentLabel, curPlane.Number());
				lineTop.SetAttribute(SecondSegmentLabel, -1);
				addedLineTops.push_back(lineTop);
			}

			iteratNum ++;
		} while (pntNums > minPntNums && iteratNum<40);
	}

	for (int j=0; j<addedLineTops.size(); j++)
	{
		LineTopology lineTop = addedLineTops[j];
		lineTop.Number() =  localLineTops.size();
		localLineTops.push_back(lineTop);
	}

	selLaserPnts.clear();
	for (int j=0; j<pnlOrgAllBoundPnts.size(); j++)
		selLaserPnts.push_back(gLaserPnts[pnlOrgAllBoundPnts[j].Number()]);
	selLaserPnts.Write("E:\\test_data\\boundarypoints.laser", 0, true);

	return true;
}

bool RefineOutBoundaryLine3(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts;
	PointNumberList pnlTemSegPnts;
	int curPlaneNum;
	LaserPoint temLaserPnt;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	bool bNearEdge;
	Plane curPlane;
	int pntNums;
	int minPntNums = 3;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNums;
	PointNumberList pnlLineSeg;
	LaserPoints selLaserPnts, temLaserPnts;
	Vector3D dir;
	Line2D  line;
	Position2D pos2d1, pos2d2;
	Position3D temPos, pos3d1, pos3d2;
	vector<PointNumberList> vecSubLineSegs;
	LineTopologies addedLineTops;

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		int curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curPlaneNum);
		pnlOrgTemBoundPnts.clear();

		if (curPlaneInd == -1) continue;//cannot find suitable plane, skip
		curPlane = vecLocalPlanes[curPlaneInd];

		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);
		MyDeriveContour(gLaserPnts, pnlTemSegPnts,pnlTemBoundPnts, 3.0);

		for (int j=0; j<pnlTemBoundPnts.size(); j++)		{
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];
			temLaserPnt.Position3DRef() = curPlane.Project(temLaserPnt.Position3DRef());

			bNearEdge = false;
			for (int k=0; k<localLineTops.size(); k++)			{
				p1 = localObjPnts[localLineTops[k][0].Number()];
				p2 = localObjPnts[localLineTops[k][1].Number()];
				temLine = Line3D(p1, p2);

				dist = temLine.DistanceToPoint( temLaserPnt.Position3DRef());
				if (dist < minDist)				{
					bNearEdge = true;
					break;
				}
			}

			if (!bNearEdge) pnlOrgTemBoundPnts.push_back(pnlTemBoundPnts[j]);
		}

		selLaserPnts.clear();
		for (int j=0; j<pnlOrgTemBoundPnts.size(); j++)	{
			selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
			pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
		}

		//dir = curPlane.Normal();
		temLaserPnts = selLaserPnts;
		//transform points to x-y plane according to their original plane
		for (int j=0; j<selLaserPnts.size(); j++) {
			pos2d1 = Project2RefPlane(curPlane, selLaserPnts[j].Position3DRef());
			temLaserPnts[j].X() = pos2d1.X(); 
			temLaserPnts[j].Y() = pos2d1.Y();
			temLaserPnts[j].Z() = 0;
		}

		pos3d1 = temLaserPnts[0].Position3DRef();
		pos3d2 = temLaserPnts[temLaserPnts.size()-1].Position3DRef();
		if (pos3d1.X() != pos3d2.X() && pos3d1.Y() != pos3d2.Y())
			temLaserPnts.push_back(temLaserPnts[0]);//enclose

		temLaserPnts.DeriveDataBounds(-1);
		DataBoundsLaser bounds = temLaserPnts.Bounds();
		double maxDistHough = sqrt(bounds.XRange() * bounds.XRange() + bounds.YRange() * bounds.YRange()) / 2.0;
		if ( maxDistHough < 0.5) continue;
		HoughSpace houghspace;
		houghspace.Initialise(int(60), int(2*maxDistHough/0.2), 0.0, -maxDistHough, PI, maxDistHough, 1);
		houghspace.SetOffsets(bounds.MidPoint().X(), bounds.MidPoint().Y(), 0.0);

		for (int j=0; j<temLaserPnts.size(); j++)
			houghspace.AddPoint(temLaserPnts[j].Position2DOnly());

		int iteratNum = 0;
		do {
			//pntNums = 0;
			line = houghspace.BestLine(&pntNums, 0, 9); 
			if (pntNums < minPntNums) continue;


			pntNums = 0;
			pnlLineSeg.clear();
			for (int j=0; j<temLaserPnts.size(); j++) {	
				temLaserPnt = temLaserPnts[j];
				dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());
				if (dist > minDist/2) continue;
				pntNums++;
				pnlLineSeg.push_back(PointNumber(j));
			}


			vecSubLineSegs.clear();
			for (int j=0; j<pnlLineSeg.size()-1; j++) {
				int num1 = pnlLineSeg[j].Number();
				int num2 = pnlLineSeg[j+1].Number();
				if (j!=pnlLineSeg.size()-2 && abs(num2-num1) < 3) continue;



				if (j==pnlLineSeg.size()-2 && abs(num2-num1) < 3) 
					j += 1;

				PointNumberList pnlTemSubLine;
				pnlTemSubLine.insert(pnlTemSubLine.begin(), pnlLineSeg.begin(), pnlLineSeg.begin()+j+1);

				if (j==pnlLineSeg.size()-1) 
					pnlLineSeg.erase(pnlLineSeg.begin(), pnlLineSeg.begin()+j);
				else
					pnlLineSeg.erase(pnlLineSeg.begin(), pnlLineSeg.begin()+j+1);



				vecSubLineSegs.push_back(pnlTemSubLine);				
				j=-1;
			}

	
			for (int iSubLine=0; iSubLine<vecSubLineSegs.size(); iSubLine++) {
				PointNumberList pnlTemSubLine = vecSubLineSegs[iSubLine];
			
				if (!MyFitLine(temLaserPnts, pnlTemSubLine, line)) continue;

				double scalar, minScalar = 9999999, maxScalar = -99999999;
				for (int j=0; j<pnlTemSubLine.size(); j++)	{
					temLaserPnt = temLaserPnts[pnlTemSubLine[j].Number()];
					houghspace.RemovePoint(temLaserPnt.Position2DOnly());
					scalar = line.Scalar(temLaserPnt.Position2DOnly());
					if (scalar < minScalar) minScalar = scalar;
					if (scalar > maxScalar) maxScalar = scalar;
				}

				pos2d1 = line.Position(minScalar);
				pos2d2 = line.Position(maxScalar);
				pos3d1 = Repoject2OrgPlane(curPlane, pos2d1);
				pos3d2 = Repoject2OrgPlane(curPlane, pos2d2);


				ObjectPoint objPnt;
				LineTopology lineTop;

				objPnt.X() = pos3d1.X(); objPnt.Y() = pos3d1.Y(); objPnt.Z() = pos3d1.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				objPnt.X() = pos3d2.X(); objPnt.Y() = pos3d2.Y(); objPnt.Z() = pos3d2.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				lineTop.Number() = localLineTops.size();
				lineTop.SetAttribute(SegmentLabel, curPlane.Number());
				lineTop.SetAttribute(SecondSegmentLabel, -1);
				addedLineTops.push_back(lineTop);
			}

			iteratNum ++;
		} while (pntNums > minPntNums && iteratNum<40);
	}

	for (int j=0; j<(int)addedLineTops.size(); ++j)
	{
		LineTopology lineTop = addedLineTops[j];
		lineTop.Number() =  localLineTops.size();
		localLineTops.push_back(lineTop);
	}

	selLaserPnts.clear();
	for (int j=0; j<pnlOrgAllBoundPnts.size(); j++)
		selLaserPnts.push_back(gLaserPnts[pnlOrgAllBoundPnts[j].Number()]);
	selLaserPnts.Write("E:\\test_data\\boundarypoints.laser", 0, true);

	return true;
}


bool RefineOutBoundaryLine1(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts;
	PointNumberList pnlTemSegPnts;
	int curPlaneNum;
	LaserPoint temLaserPnt;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	bool bNearEdge;
	Plane curPlane;
	int pntNums;
	int minPntNums = 3;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNums;
	PointNumberList pnlLineSeg;
	LaserPoints selLaserPnts, temLaserPnts;
	Vector3D dir;
	Line2D  line;
	Position2D pos2d1, pos2d2;
	Position3D temPos, pos3d1, pos3d2;
	vector<PointNumberList> vecLineSegs;
	LineTopologies addedLineTops;

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		//search for out boundary points
		//which first are boundary points and don't near inner edge
		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		int curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curPlaneNum);
		pnlOrgTemBoundPnts.clear();

		if (curPlaneInd == -1) continue;//cannot find suitable plane, skip
		curPlane = vecLocalPlanes[curPlaneInd];

		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);
		MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts, 3.0);

		//remove points near ridge
		for (int j=0; j<pnlTemBoundPnts.size(); j++)
		{
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];
			temLaserPnt.Position3DRef() = curPlane.Project(temLaserPnt.Position3DRef());

			bNearEdge = false;
			for (int k=0; k<localLineTops.size(); k++)
			{
				p1 = localObjPnts[localLineTops[k][0].Number()];
				p2 = localObjPnts[localLineTops[k][1].Number()];
				temLine = Line3D(p1, p2);

				dist = temLine.DistanceToPoint( temLaserPnt.Position3DRef());
				if (dist < minDist)
				{
					bNearEdge = true;
					break;
				}
			}

			if (!bNearEdge) pnlOrgTemBoundPnts.push_back(pnlTemBoundPnts[j]);
		}

		selLaserPnts.clear();
		for (int j=0; j<pnlOrgTemBoundPnts.size(); j++)
		{
			selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
			pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
		}

		//dir = curPlane.Normal();
		temLaserPnts = selLaserPnts;
		//transform points to x-y plane according to their original plane
		for (int j=0; j<selLaserPnts.size(); j++)
		{
			pos2d1 = Project2RefPlane(curPlane, selLaserPnts[j].Position3DRef());
			temLaserPnts[j].X() = pos2d1.X(); 
			temLaserPnts[j].Y() = pos2d1.Y();
			temLaserPnts[j].Z() = 0;
		}


		temLaserPnts.DeriveDataBounds(-1);
		DataBoundsLaser bounds = temLaserPnts.Bounds();
		double maxDistHough = sqrt(bounds.XRange() * bounds.XRange() + bounds.YRange() * bounds.YRange()) / 2.0;
		if ( maxDistHough < 0.5) continue;
		HoughSpace houghspace;
		houghspace.Initialise(int(60), int(2*maxDistHough/0.2), 0.0, -maxDistHough, PI, maxDistHough, 1);
		houghspace.SetOffsets(bounds.MidPoint().X(), bounds.MidPoint().Y(), 0.0);

		for (int j=0; j<temLaserPnts.size(); j++)
			houghspace.AddPoint(temLaserPnts[j].Position2DOnly());

		do 
		{
			
			line = houghspace.BestLine(&pntNums, 0, 9); 
			if (pntNums < minPntNums) continue;

	
			pntNums = 0;
			pnlLineSeg.clear();
			for (int j=0; j<temLaserPnts.size(); j++)
			{	
				temLaserPnt = temLaserPnts[j];
				dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());
				if (dist > minDist/2) continue;
				pntNums++;
				houghspace.RemovePoint(temLaserPnt.Position2DOnly());
				pnlLineSeg.push_back(PointNumber(j));
			}

			if(pntNums < minPntNums) continue;
			//fit the line again
			if (!MyFitLine(temLaserPnts, pnlLineSeg, line))
				continue;

			double scalar;
			vector<double> vecScalar;
			
			for (int j=0; j<pnlLineSeg.size(); j++)
			{
				temLaserPnt = temLaserPnts[pnlLineSeg[j].Number()];
				scalar = line.Scalar(temLaserPnt.Position2DOnly());
				vecScalar.push_back(scalar);				
			}

			std::sort(vecScalar.begin(), vecScalar.end());

			double minDifScal=0;// = (vecScalar[vecScalar.size()-1] - vecScalar[0])/4;
			for (int j=1; j<vecScalar.size();j++)
				minDifScal += vecScalar[j]-vecScalar[j-1];
			minDifScal = 3.0*minDifScal/(vecScalar.size()-2);


			vector<Position2D> vecLine2DNodes;
			for (int j=0; j<vecScalar.size()-1;j++)
			{
				double scalar1 = vecScalar[j];
				double scalar2 = vecScalar[j+1];

				if (j!=vecScalar.size()-2 && fabs(scalar1-scalar2)<minDifScal )
					continue;//continued



				if (j+1>minPntNums )
				{
					scalar1 = vecScalar[0];
					if (j==vecScalar.size()-2 && fabs(scalar1-scalar2)<minDifScal)
						scalar2 = vecScalar[j+1];//last point of this line segment
					else
						scalar2 = vecScalar[j];

					vecLine2DNodes.push_back(line.Position(scalar1));
					vecLine2DNodes.push_back(line.Position(scalar2));
				}

			
				vecScalar.erase(vecScalar.begin(), vecScalar.begin()+j);
				j=1;

			}

			for (int iSubLine=0; iSubLine<vecLine2DNodes.size(); iSubLine+=2)
			{
				pos2d1 = vecLine2DNodes[iSubLine];
				pos2d2 = vecLine2DNodes[iSubLine+1];
				pos3d1 = Repoject2OrgPlane(curPlane, pos2d1);
				pos3d2 = Repoject2OrgPlane(curPlane, pos2d2);

			
				ObjectPoint objPnt;
				LineTopology lineTop;

				objPnt.X() = pos3d1.X(); objPnt.Y() = pos3d1.Y(); objPnt.Z() = pos3d1.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				objPnt.X() = pos3d2.X(); objPnt.Y() = pos3d2.Y(); objPnt.Z() = pos3d2.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

			
				lineTop.Number() = addedLineTops.size();
				lineTop.SetAttribute(SegmentLabel, curPlane.Number());
				lineTop.SetAttribute(SecondSegmentLabel, -1);
				addedLineTops.push_back(lineTop);
			}

		} while (pntNums > minPntNums);
	}

	for (int j=0; j<addedLineTops.size(); j++)
	{
		LineTopology lineTop = addedLineTops[j];
		lineTop.Number() =  localLineTops.size();
		localLineTops.push_back(lineTop);
	}

	selLaserPnts.clear();
	for (int j=0; j<(int)pnlOrgAllBoundPnts.size(); ++j)
		selLaserPnts.push_back(gLaserPnts[pnlOrgAllBoundPnts[j].Number()]);
	selLaserPnts.Write("E:\\test_data\\boundarypoints.laser", 0, true);

	return true;
}


bool RefineOutBoundaryLine2(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts, pnlTemSegPnts;
	int curPlaneNum;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	Plane curPlane;
//	int pntNums;
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

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		int curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curPlaneNum);
		pnlOrgTemBoundPnts.clear();

		if (curPlaneInd == -1) continue;//cannot find suitable plane, skip
		curPlane = vecLocalPlanes[curPlaneInd];

		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);

	    MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts,4);
		LineTopology temLineTop;

		for (int j=0; j<pnlTemBoundPnts.size(); j++) {
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];
			temLaserPnt.Position3DRef() = curPlane.Project(temLaserPnt.Position3DRef());

			bool bNearEdge = false;
			for (int k=0; k<vecTemLineNums.size(); k++) {
				temLineTop = localLineTops[vecTemLineNums[k]];
				p1 = localObjPnts[temLineTop[0].Number()];
				p2 = localObjPnts[temLineTop[1].Number()];
				temLine = Line3D(p1, p2);

				dist = temLine.DistanceToPoint( temLaserPnt.Position3DRef());
				if (dist < minDist) {
					bNearEdge = true;
					break;
				}
			}

			if (!bNearEdge) pnlOrgTemBoundPnts.push_back(pnlTemBoundPnts[j]);
		}

		selLaserPnts.clear();
		for (int j=0; j<pnlOrgTemBoundPnts.size(); j++) {
			selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
			pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
		}

		if (selLaserPnts.empty()) continue;
		dir = curPlane.Normal();

		temLaserPnts = selLaserPnts;
		PointNumberList pnlTemSeg;
		Position3D center;
		for (int i=0; i<selLaserPnts.size(); i++)
			pnlTemSeg.push_back(PointNumber(i));
		Project2RefPlane(selLaserPnts, pnlTemSeg, curPlane, center, temLaserPnts);
		ReProject2OrgPlane(temLaserPnts, curPlane, center, temBackProjPnts);

		//for debug
		/*temLaserPnts.Write("e:\\zzzproject.laser",0,0);
		temBackProjPnts.Write("zzzboundary.laser",0,0);*/

		pos3d1 = temLaserPnts[0].Position3DRef();
		pos3d2 = temLaserPnts[temLaserPnts.size()-1].Position3DRef();
		if (pos3d1.X() != pos3d2.X() && pos3d1.Y() != pos3d2.Y()) {
			temLaserPnts.push_back(temLaserPnts[0]);//enclose
			temBackProjPnts.push_back(temBackProjPnts[0]);
		}

		//use curvature to detect corner
		Line3D temLine;
		vector<double> vecK(temLaserPnts.size()-1, 0.0);//curvature
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

		vector<int> vecCorner;
		int num1, num2;
		//int curNum;
		if(temLaserPnts.size() < 3) continue;//not enough points
		vecCorner.push_back(0);
		vecCorner.push_back(temLaserPnts.size()/2);
		vecCorner.push_back(temLaserPnts.size()-1);
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

		vecCornerPos.clear();
		for (int j=0; j<(int)vecCorner.size()-1; j++) {
			num1 = vecCorner[j]; num2 = vecCorner[j+1];
			//if (abs(num2-num1)==1) continue;//not enough points

			pos2d1 = temLaserPnts[num1].Position2DOnly();
			pos2d2 = temLaserPnts[num2].Position2DOnly();

			if (pos2d2.X()==pos2d1.X() && pos2d2.Y()==pos2d1.Y())
				pos2d2 = temLaserPnts[num2-1].Position2DOnly();//remove the start point

			//pos3d1 = Repoject2OrgPlane(curPlane, pos2d1);
			//pos3d2 = Repoject2OrgPlane(curPlane, pos2d2);
			pos3d1 = temBackProjPnts[num1].Position3DRef();
			pos3d2 = temBackProjPnts[num2].Position3DRef();
			//re-transform to original plane
			//to do, refine out boundary lines

			//out put
			AddTopLine(localObjPnts, localLineTops, pos3d1, pos3d2, curPlane.Number(), -1);
		}
	}


	return true;
}



bool RefineOutBoundaryLine2(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	PointNumberList& vecCurSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts, pnlTemSegPnts;
	int curPlaneNum;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	Plane curPlane;
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
	//vector<int> vecCornerPos;
	pnlTemBoundPnts.reserve(500);

	//search for out boundary points
	//which first are boundary points and don't near inner edge
	pnlTemSegPnts = vecCurSegPnts;
	curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
	int curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curPlaneNum);
	pnlOrgTemBoundPnts.clear();

	if (curPlaneInd == -1) return false;//cannot find suitable plane, skip
	curPlane = vecLocalPlanes[curPlaneInd];

	//DeriveContour_AlphaShape(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts,4);
	MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts,6);
	pnlOrgTemBoundPnts = pnlTemBoundPnts;

	selLaserPnts.clear();
	for (int j=0; j<pnlOrgTemBoundPnts.size(); j++) {
		selLaserPnts.push_back(gLaserPnts[pnlOrgTemBoundPnts[j].Number()]);
		pnlOrgAllBoundPnts.push_back(pnlOrgTemBoundPnts[j]);
	}

	if (selLaserPnts.empty()) return false;
	dir = curPlane.Normal();

	temLaserPnts = selLaserPnts;
	PointNumberList pnlTemSeg;
	Position3D center;
	for (int i=0; i<selLaserPnts.size(); i++)
		pnlTemSeg.push_back(PointNumber(i));
	Project2RefPlane(selLaserPnts, pnlTemSeg, curPlane, center, temLaserPnts);
	ReProject2OrgPlane(temLaserPnts, curPlane, center, temBackProjPnts);



	pos3d1 = temLaserPnts[0].Position3DRef();
	pos3d2 = temLaserPnts[temLaserPnts.size()-1].Position3DRef();
	if (pos3d1.X() != pos3d2.X() && pos3d1.Y() != pos3d2.Y()) {
		temLaserPnts.push_back(temLaserPnts[0]);//enclose
		temBackProjPnts.push_back(temBackProjPnts[0]);
	}

	//use curvature to detect corner
	vector<double> vecK(temLaserPnts.size()-1, 0.0);//curvature

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

	//get contour
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
	//vecCornerPos.clear();
	for (int j=0; j<(int)vecCorner.size()-1; j++) {
		num1 = vecCorner[j]; num2 = vecCorner[j+1];

		pos2d1 = temLaserPnts[num1].Position2DOnly();
		pos2d2 = temLaserPnts[num2].Position2DOnly();

		if (pos2d2.X()==pos2d1.X() && pos2d2.Y()==pos2d1.Y())
			pos2d2 = temLaserPnts[num2-1].Position2DOnly();//remove the start point

		pos3d1 = temBackProjPnts[num1].Position3DRef();
		pos3d2 = temBackProjPnts[num2].Position3DRef();

		//out put
		AddTopLine(localObjPnts, localLineTops, pos3d1, pos3d2, curPlane.Number(), -1);
	}

	return true;
}

void PorjectTopLinesOntoPlane(const Plane& plane, const LineTopology& localLineTop, ObjectPoints& localObjPnts)
{
	Line3D plumb;
	Position3D cross;

	for (int i=0; i<localLineTop.size(); i++) {
		ObjectPoint& temObjPnt = localObjPnts[localLineTop[i].Number()];
		plumb = Line3D(temObjPnt.Position3DRef(), Vector3D(0.0, 0.0, 1.0));
		if(!MyIntersectLine3DPlane(plumb, plane, cross)) continue;
		temObjPnt.Position3DRef()=cross;
	}
}

#include "PolylineGeneralization.h"

bool RefineOutBoundaryLine8(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
							PointNumberList& vecCurSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList contour;
	ObjectPoints temObjPnts;
	LineTopology temLineTop;
	DeriveContour_AlphaShape2D(gLaserPnts, vecCurSegPnts, contour, gBldRecPar.alphaDia, gBldRecPar.bAdaptiveAlpha);
	PolylineGeneralizeByLineGrow(gLaserPnts, contour, temObjPnts, temLineTop);

	//project to plane
	int segNum = gLaserPnts[vecCurSegPnts[0].Number()].Attribute(SegmentNumberTag);
	int indPlane = IndexPlaneBySegNumber(vecLocalPlanes, segNum);
	assert(indPlane!=-1);
	PorjectTopLinesOntoPlane(vecLocalPlanes[indPlane], temLineTop, temObjPnts);

	//output
	if (!temLineTop.empty()) {
		ObjectPoints::iterator pnt0, pnt1;
		for (unsigned int i=0; i<temLineTop.size()-1; i++) {
			pnt0 = temObjPnts.begin()+temLineTop[i].Number();
			pnt1 = temObjPnts.begin()+temLineTop[i+1].Number();
			AddTopLine(localObjPnts, localLineTops, *pnt0, *pnt1, segNum, -1);
		}
	}

	return true;
}


IEDll_API bool MySimplifyContour(const LaserPoints& gLaserPnts, const PointNumberList& pnlTemBoundPnts, 
	const Plane& curPlane, ObjectPoints& outObjPnts, LineTopology& outTop)
{
	Line3D temLine;
	double dist;
	double minDist = 0.5;
	PointNumberList pnlLineSeg;
	LaserPoints selLaserPnts, temLaserPnts, temBackProjPnts;
	LaserPoint temLaserPnt;
	Vector3D dir;
	Line2D  line;
	Position2D pos2d1, pos2d2, temPos2D;
	Position3D temPos, pos3d1, pos3d2;

	selLaserPnts.clear();
	for (int j=0; j<pnlTemBoundPnts.size(); j++) {
		selLaserPnts.push_back(gLaserPnts[pnlTemBoundPnts[j].Number()]);
	}

	if (selLaserPnts.empty()) return false;
	dir = curPlane.Normal();

	temLaserPnts = selLaserPnts;
	PointNumberList pnlTemSeg;
	Position3D center;
	for (int i=0; i<selLaserPnts.size(); i++)
		pnlTemSeg.push_back(PointNumber(i));
	Project2RefPlane(selLaserPnts, pnlTemSeg, curPlane, center, temLaserPnts);
	ReProject2OrgPlane(temLaserPnts, curPlane, center, temBackProjPnts);


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

		if (pos3d1.X()==pos3d2.X() && 
			pos3d1.Y()==pos3d2.Y() && 
			pos3d1.Z()==pos3d2.Z()) 
			return false;
		temLine = Line3D(pos3d1, pos3d2);
		dist = temLine.DistanceToPoint(temPos);
		double temk = dist/(pos3d1.Distance(pos3d2) + 0.000001);

		vecK[j] = dist/(pos3d1.Distance(pos3d2) + 0.000001);
	}


	vector<int> vecCorner;
	int num1, num2;
	if(temLaserPnts.size() < 3) return false;//not enough points


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
	itrFind = std::find(vecK.begin(), vecK.end(), vecTemK[vecTemK.size()-3]);
	vecCorner.push_back(itrFind-vecK.begin());
	std::sort(vecCorner.begin(),vecCorner.end());
//	vecCorner.push_back(itrFind-vecK.begin());
	vecCorner.push_back(temLaserPnts.size()-1);

	//get contour
	for (int j=0; j<(int)vecCorner.size()-1 && vecCorner.size()>3; j++) {
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


	if (vecCorner.size()>4)	{
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
	}


	outTop.clear();
	int objPntNum = outObjPnts.size();
	ObjectPoint temObjPnt;
	for (int j=0; j<(int)vecCorner.size()-1; j++) {
		num1 = vecCorner[j]; 
		temObjPnt.Position3DRef() = temBackProjPnts[num1].Position3DRef();
		temObjPnt.Number() = objPntNum++;
		outObjPnts.push_back(temObjPnt);
		outTop.push_back(temObjPnt.NumberRef());
	}
	outTop.push_back(outTop[0]);
	return true;
}


IEDll_API void RefineRoofTopGrap(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{

	LineTopologies::iterator topLine;
	int segNum1, segNum2;
	int pntNum1, pntNum2;

	for (int i=0; i<localLineTops.size(); i++) {
		topLine = localLineTops.begin()+i;
		segNum1 = topLine->Attribute(SegmentLabel);
		segNum2 = topLine->Attribute(SecondSegmentLabel);
		if (topLine->Attribute(SegmentLabel)==-1 || topLine->Attribute(SecondSegmentLabel)==-1 ) {
			localLineTops.erase(topLine);
			i--;
		}
	}


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


	int lineCoef;
	for (int i=0; i<localLineTops.size(); i++) {
		topLine = localLineTops.begin()+i;
		lineCoef = topLine->Attribute(3);
		if (lineCoef <= 3) {
			localLineTops.erase(topLine);
			i--;
		}
	}


	Line3D line;
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
