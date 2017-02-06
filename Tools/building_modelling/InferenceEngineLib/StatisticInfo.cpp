/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 26-01-2011
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
#include "Buildings.h"
#include <vector>
#include <math.h>
#include <map>

using namespace std;

//search for line top based on end node position
int IndLineTopByPnts(ObjectPoints& localObjPnts, LineTopologies& localLineTops, 
	const Position3D& point1, const Position3D& point2)
{
	LineTopologies::iterator line;
	int ind = -1;
	int pntNum1, pntNum2;
	double dist1, dist2, dist3, dist4;

	for (line=localLineTops.begin(); line!=localLineTops.end(); ++line) 
	{
		pntNum1 = (*line)[0].Number();
		pntNum2 = (*line)[1].Number();
		dist1 = point1.Distance(localObjPnts[pntNum1].Position3DRef());
		dist2 = point2.Distance(localObjPnts[pntNum2].Position3DRef());
		dist3 = point1.Distance(localObjPnts[pntNum2].Position3DRef());
		dist4 = point2.Distance(localObjPnts[pntNum1].Position3DRef());
//		if ((localObjPnts[pntNum1].Position3DRef() == point1 &&
//			localObjPnts[pntNum2].Position3DRef() == point2) || 
//			(localObjPnts[pntNum2].Position3DRef() == point2 &&
//			localObjPnts[pntNum1].Position3DRef() == point1) )
		if((dist1<0.05 && dist2<0.05) 
			|| (dist3<0.05 && dist4<0.05))
		{
			ind = line - localLineTops.begin();
			break;
		}
	}
	return ind;
}

int DetermineLineLabel(const Plane& plane1, const Plane& plane2, const Line3D& line)
{
//	pPlane1 = &vecRoofPlane[ind1];
//	pPlane2 = &vecRoofPlane[ind2];
	int label;
	if (line.IsHorizontal(10*PI/180)) {
		if (plane1.IsHorizontal(10*PI/180) 
			|| plane2.IsHorizontal(10*PI/180))
			label = 2;
		else label = 3;
	}
//	else {

//	}

	return 0;
}

bool StatisticRoofInfo(char* strInLaserPntPath, char* strOutObjPntsPath, char* strOutTopLinesPath)
{
	LaserPoints g_laserPnts;
	vector<PointNumberList> g_vecSegPnls;

	if (!g_laserPnts.Read(strInLaserPntPath)) {
		printf("Error reading laser points from file: %s\n", strInLaserPntPath);
		return false;	
	}

	int segNum, maxSegNum = -999;
	for (unsigned int i=0; i< (unsigned int)g_laserPnts.size(); ++i) {
		if ( !g_laserPnts[i].HasAttribute(SegmentNumberTag))
			continue;
		segNum = g_laserPnts[i].Attribute(SegmentNumberTag);
		if(segNum > maxSegNum)
			maxSegNum = segNum;
	}

	vector<PointNumberList>* pTemVecSegPnls = new vector<PointNumberList>(maxSegNum+1);
	for (int i=0; i<g_laserPnts.size(); ++i){
		if ( !g_laserPnts[i].HasAttribute(SegmentNumberTag))
			continue;
		segNum = g_laserPnts[i].Attribute(SegmentNumberTag);
		pTemVecSegPnls->at(segNum).push_back(PointNumber(i));
	}
	g_vecSegPnls.clear();
	g_vecSegPnls.reserve(maxSegNum/4);
	int size;
	//remove empty point number list
	for (unsigned int i=0; i< (unsigned int)pTemVecSegPnls->size(); ++i) {
		if (!pTemVecSegPnls->at(i).empty()) 
			g_vecSegPnls.push_back(pTemVecSegPnls->at(i));
	}
	delete pTemVecSegPnls;//force to release memory

	PointNumberList pnlCurSegPnts;
	Plane curPlane;
	Vector3D zenithDir(0.0, 0.0, 1.0);
	//	double pntSpace = g_laserPnts.MedianInterPointDistance(std::min(5000, (int)g_laserPnts.size()-2));
	//double pntSpace = lps.MedianInterPointDistance((int)min(5000.0, (double)lps.size()-2));
	//	g_laserPnts.poi
	double angle, area;
	double areaPerPnt = 1.0/20.0;//pntSpace*pntSpace;
	const double degree(180/PI);
	int nPntNum, iRoof=0;
	FILE* file = fopen("c:\\roof_info.txt", "w+");
	if (!file) {
		printf("fail to open roof_info.txt");
		return false;
	}

	fprintf(file, "roof slop(degree) area(m2)\n");

	for (int iSeg=0; iSeg<g_vecSegPnls.size(); ++iSeg)
	{//reconstruct the building which contain current laser point segment
		pnlCurSegPnts = g_vecSegPnls[iSeg];
		nPntNum = pnlCurSegPnts.size();
		if (nPntNum <  100) 
			continue;

		iRoof++;
		curPlane = g_laserPnts.FitPlane(pnlCurSegPnts);
		angle = Angle(zenithDir, curPlane.Normal());
		if (angle>PI/2) angle = PI-angle;
		angle *= degree;
		area = nPntNum*areaPerPnt;

		fprintf(file, "%d \t%.1f \t%.1f\n", iRoof, angle, area);
	}

	fclose(file);

	return true;
}

//line topology:
//0, 1 :left and right neighbor segment number
//2 : is horizontal or not
//3: line label
void StatistBuildingInfo(const char* strObjPntsPath, const char* strTopLinesPath, const char* strInfoPath)
{
	ObjectPoints model_points;
	LineTopologies model_top;
	LineTopologies::iterator temLineTop;
	int bldNumber;
	map<int, vector<LineTopology> > BldMap;
	typedef map<int, vector<LineTopology> >::const_iterator BldMapItr;
	typedef vector<LineTopology>::const_iterator RoofVecItr;

	if (!model_points.Read(strObjPntsPath)) 
		return ;
	if (!model_top.Read(strTopLinesPath, false)) 
		return ;

	//read roof surface into building
	//each building was stored in one topologies
	for (temLineTop=model_top.begin(); temLineTop!=model_top.end(); ++temLineTop) {
		if (!temLineTop->HasAttribute(LineLabelTag)
			||temLineTop->Attribute(LineLabelTag) != RoofClass)
			continue;//not roof
		if (!temLineTop->HasAttribute(BuildingNumberTag))
			continue;// no building number

		bldNumber = temLineTop->Attribute(BuildingNumberTag);
		BldMap[bldNumber].push_back(*temLineTop); 
	}

	vector<Plane> vecRoofPlane;
	Line3D curLine;
	Plane temPlane;
	Plane *pPlane1, *pPlane2;
	ObjectPoints curBldPnts;
	LineTopologies curBldTops;
	LineTopology curLineTop(2);
	LineTopologies::iterator itrTop;
	ObjectPoints::iterator obj1, obj2;
	int nRoof;
	double dist, dist1, dist2;
	bool isflatRoof, isHorizontal;
	int indRoof, nCurPlaneNum, nNeibPlaneNum;
	int planeNum1, planeNum2, planeNum3, planeNum4;
	int lineLabel1, lineLabel2, lineLabel3, lineLabel4;
	vector<vector<int> > vecNeigFaceNum;
	int BldClassHist[18];
	memset(BldClassHist, 0, 18*sizeof(int));
	//single face: flat: 0 ; oblique: 1 
	//two face: 2 one flat face;


	for (BldMapItr itrBld=BldMap.begin(); itrBld!=BldMap.end(); itrBld++) {
		//itrBld->second;
		curBldPnts.clear();
		curBldTops.clear();
		vecRoofPlane.clear();

		//////////////////////////////////////////////////////////////////////////
		//pre-processing
		nRoof = itrBld->second.size();
		for (RoofVecItr itrRoof= (itrBld->second).begin(); itrRoof!=(itrBld->second).end(); ++itrRoof) {
			if (itrRoof->Number() == 228)
			{
				int aaa = 0;
			}
			if(itrRoof->size() < 4)
				continue;
			temPlane = Plane(model_points[itrRoof->at(0).Number()].Position3DRef(), 
				model_points[itrRoof->at(1).Number()].Position3DRef(),
				model_points[itrRoof->at(2).Number()].Position3DRef());
			isflatRoof = true;
			//determine whether this roof is flat plane or not
			for (int iPos=3; iPos<itrRoof->size(); ++iPos) {
				dist = temPlane.Distance(model_points[itrRoof->at(iPos).Number()].Position3DRef());
				if (fabs(dist) > 0.05) {
					isflatRoof = false;
					break;
				}
			}

			//////////////////////////////////////////////////////////////////////////
			//add current building objects and top lines as local information
			if (!isflatRoof) continue;
			nCurPlaneNum = itrRoof->Number();
			temPlane.Number() = nCurPlaneNum;
			vecRoofPlane.push_back(temPlane);
			for (int iPos=0; iPos<itrRoof->size()-1; ++iPos ) {
				obj1 = model_points.begin() + itrRoof->at(iPos).Number();
				obj2 = model_points.begin() + itrRoof->at(iPos+1).Number();

				indRoof = IndLineTopByPnts(curBldPnts, curBldTops, *obj1, *obj2);
				if (indRoof == -1)	{
					nNeibPlaneNum = -1;
					//search for neighbor plane
					for (int iPlain=0; iPlain<vecRoofPlane.size()-1;++iPlain) {
						dist1 = vecRoofPlane[iPlain].Distance(obj1->Position3DRef());
						dist2 = vecRoofPlane[iPlain].Distance(obj2->Position3DRef());
						if (fabs(dist1) < 0.001 && fabs(dist2) < 0.001) {
								nNeibPlaneNum = vecRoofPlane[iPlain].Number();
								break;
						}
					}

					AddTopLine(curBldPnts, curBldTops, *obj1, *obj2, nCurPlaneNum, nNeibPlaneNum, -1);
					if(nNeibPlaneNum != -1)
						curBldTops[curBldTops.size()-1].SetAttribute(3,2);//convex
				}
				else {
					curBldTops[indRoof].SetAttribute(SecondSegmentLabel, nCurPlaneNum);
				}
			}
		}

//		for (itrTop=curBldTops.begin(); itrTop!=curBldTops.end(); ++ itrTop) {
			//if (itrTop->Attribute(0)==-1 || itrTop->Attribute(1)==-1)
//			printf("%d  %d\n", itrTop->Attribute(0), itrTop->Attribute(1));
//		}

		//////////////////////////////////////////////////////////////////////////
		//label lines
		for (itrTop=curBldTops.begin(); itrTop!=curBldTops.end(); ++ itrTop) {
			if (itrTop->Attribute(SegmentLabel)==-1 || itrTop->Attribute(SecondSegmentLabel)==-1)
				continue;//this line only has one neighbor face
			obj1 = curBldPnts.begin() + itrTop->at(0).Number();
			obj2 = curBldPnts.begin() + itrTop->at(1).Number();
			curLine = Line3D(obj1->Position3DRef(), obj2->Position3DRef());
			//pPlane1 =
			int ind1 = IndexPlaneBySegNumber(vecRoofPlane, itrTop->Attribute(SegmentLabel));
			int ind2 = IndexPlaneBySegNumber(vecRoofPlane, itrTop->Attribute(SecondSegmentLabel));
			if (ind1==-1 || ind2==-1) continue;//one neighbor face does not exist
			pPlane1 = &vecRoofPlane[ind1];
			pPlane2 = &vecRoofPlane[ind2];
			double angle = Angle(curLine.Direction(), Vector3D(0,0,1));
			angle = angle * 180/PI;
			angle -= 90.0;
			if (curLine.IsHorizontal(15*PI/180)) {//horizontal ridge
				if (pPlane1->IsHorizontal(15*PI/180) ||
					pPlane2->IsHorizontal(15*PI/180))
					itrTop->SetAttribute(3,0);//horizontal ridge with one flat face
				else {
					if (itrTop->Attribute(3) != 2)//Concave horizontal ridge
						itrTop->SetAttribute(3, 1);//Convex horizontal ridge
				}
			}
			else {
				if (itrTop->Attribute(3) == 2)
					itrTop->SetAttribute(3, 4);//Concave oblique ridge
				else
					itrTop->SetAttribute(3, 3);//Convex oblique ridge
			}
		}

//		for (itrTop=curBldTops.begin(); itrTop!=curBldTops.end(); ++ itrTop) {
//			itrTop->Label() = itrTop->Attribute(3);
//		}
//		curBldTops.Write("c:\\2.top", false);
//		curBldPnts.Write("c:\\2.objpts");
		//////////////////////////////////////////////////////////////////////////
		//statistic building types
		//single face
		vector<int> temNeibFace;
		for (int iPlane=0; iPlane<vecRoofPlane.size(); ++iPlane) {
			planeNum1 = vecRoofPlane[iPlane].Number();
			SearchAdjacentFaces(curBldTops, planeNum1, temNeibFace);
			if (temNeibFace.empty()) {
				if (vecRoofPlane[iPlane].IsHorizontal(10*PI/180))
					++BldClassHist[0];
				else
					++BldClassHist[1];
			}
		}
		//two faces
		Search2AdjFaces(vecRoofPlane, curBldTops, vecNeigFaceNum);
		for (int iCircle=0; iCircle<vecNeigFaceNum.size(); iCircle++) {
			planeNum1 = vecNeigFaceNum[iCircle][0];
			planeNum2 = vecNeigFaceNum[iCircle][1];

			LineTopology temLineTop;
			int indTemLineTop;
			SearchLineBy2Faces(curBldTops, planeNum1, planeNum2, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabel1 = temLineTop.Attribute(3);
			
			if (lineLabel1 == 0)
				++BldClassHist[2];
			else if (lineLabel1 == 1)
				++BldClassHist[3];
			else 
				++BldClassHist[4];
		}

		//three faces circle
		Search3AdjFaces(vecRoofPlane, curBldTops, vecNeigFaceNum);
		for (int iCircle=0; iCircle<vecNeigFaceNum.size(); iCircle++) {
			planeNum1 = vecNeigFaceNum[iCircle][0];
			planeNum2 = vecNeigFaceNum[iCircle][1];
			planeNum3 = vecNeigFaceNum[iCircle][2];

			LineTopology temLineTop;
			int indTemLineTop;
			SearchLineBy2Faces(curBldTops, planeNum1, planeNum2, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabel1 = temLineTop.Attribute(3);
			SearchLineBy2Faces(curBldTops, planeNum2, planeNum3, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabel2 = temLineTop.Attribute(3);
			SearchLineBy2Faces(curBldTops, planeNum3, planeNum1, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabel3 = temLineTop.Attribute(3);
			if (lineLabel1>lineLabel2) std::swap(lineLabel1, lineLabel2);
			if (lineLabel1>lineLabel3) std::swap(lineLabel1, lineLabel3);
			if (lineLabel2>lineLabel3) std::swap(lineLabel2, lineLabel3);

			if (lineLabel1==1 && lineLabel2==3 && lineLabel3==3)
				++BldClassHist[5];
			else if (lineLabel1==1 && lineLabel2==4 && lineLabel3==4)
				++BldClassHist[6];
			else if (lineLabel1==1 && lineLabel2==3 && lineLabel3==4)
				++BldClassHist[7];
			else if (lineLabel1==2 && lineLabel2==4 && lineLabel3==4)
				++BldClassHist[8];
			else if (lineLabel1==0 && lineLabel2==0 && lineLabel3==3)
				++BldClassHist[9];
			else if (lineLabel1==0 && lineLabel2==0 && lineLabel3==4)
				++BldClassHist[10];
			else if (lineLabel1==3 && lineLabel2==3 && lineLabel3==3)
				++BldClassHist[11];
			else 
				++BldClassHist[12];
		}

		//three faces circle
		Search4AdjFaces(vecRoofPlane, curBldTops, vecNeigFaceNum);
		for (int iCircle=0; iCircle<vecNeigFaceNum.size(); iCircle++) {
			planeNum1 = vecNeigFaceNum[iCircle][0];
			planeNum2 = vecNeigFaceNum[iCircle][1];
			planeNum3 = vecNeigFaceNum[iCircle][2];
			planeNum4 = vecNeigFaceNum[iCircle][3];

			LineTopology temLineTop;
			int indTemLineTop;
			vector<int> lineLabels;
			SearchLineBy2Faces(curBldTops, planeNum1, planeNum2, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabels.push_back(temLineTop.Attribute(3));
			SearchLineBy2Faces(curBldTops, planeNum2, planeNum3, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabels.push_back(temLineTop.Attribute(3));
			SearchLineBy2Faces(curBldTops, planeNum3, planeNum4, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabels.push_back(temLineTop.Attribute(3));
			SearchLineBy2Faces(curBldTops, planeNum4, planeNum1, indTemLineTop);
			temLineTop = curBldTops[indTemLineTop];
			lineLabels.push_back(temLineTop.Attribute(3));
			std::sort(lineLabels.begin(), lineLabels.end());

			if (lineLabels[0]==3&&lineLabels[1]==3&&lineLabels[2]==3&&lineLabels[3]==3)
				++BldClassHist[15];
			else if (lineLabels[0]==1&&lineLabels[1]==3&&lineLabels[2]==3&&lineLabels[3]==3)
				++BldClassHist[14];
			else 
				++BldClassHist[15];
		}

	}//end building 

	FILE* file = fopen(strInfoPath, "w");
	if (!file) {
		printf("Cannot write %s file", strInfoPath);
		return;
	}

	for (int i=0; i<18; ++i) 
		fprintf(file, "%d\t%d\n", i, BldClassHist[i]);
	fclose(file);
}//end function

void ComputeAreaSlop(const ObjectPoints& objPnts, const LineTopology& lineTops, 
	double& area, double& slope, Vector3D& gravity)
{
	if (lineTops.size() < 3) 
		return;

	Plane temPlane = Plane(objPnts[lineTops[0].Number()].Position3DRef(), 
		objPnts[lineTops[1].Number()].Position3DRef(),
		objPnts[lineTops[2].Number()].Position3DRef());
	slope = Angle(temPlane.Normal(), Vector3D(0, 0, 1));
	if (slope > PI/2) slope -= PI/2;

	gravity = Vector3D(0, 0, 0);
	for (int iPnt=0; iPnt<lineTops.size(); ++iPnt) {
		gravity += objPnts[lineTops[iPnt].Number()];
	}
	gravity /= lineTops.size();
}

#include <fstream>
void StatisticRoofSlop(const char* strLasPntPath, const char* strSlopPath)
{
	assert(strLasPntPath && strSlopPath);
	LaserPoints lasPnts;
	lasPnts.Read(strLasPntPath);
	std::vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(lasPnts, vecPnlSegs);
	Plane plane;
	Vector3D plumb(0.0, 0.0, 1.0);
	std::ofstream stream;
	stream.open(strSlopPath);
	double slope;
	Vector3D normal;
	vector<int> vecHist = vector<int>(90, 0);

	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		if (vecPnlSegs[i].size()<150) continue;
		plane = lasPnts.FitPlane(vecPnlSegs[i]);
		normal = plane.Normal();
		if (normal.Z()<0.0) 
			normal *= -1.0;
		slope = 180.0*MyAngle(plumb, normal)/3.1415926;
		vecHist[int(slope)]++;
		stream<<slope<<endl;
	}

	stream<<endl<<endl;
	for (unsigned int i = 0; i < vecHist.size(); i++)
	{
		stream<<i<<"  "<<vecHist[i]<<endl;
	}

	stream.close();
}
