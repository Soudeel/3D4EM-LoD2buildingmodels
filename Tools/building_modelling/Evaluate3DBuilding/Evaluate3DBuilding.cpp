/*-----------------------------------------------------------
May 07, 2016
Biao Xiong
Evaluate Models according to point distance to model planes, lines, and corners
//------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "Buildings.h"
#include "InferenceEngine.h"
#include <string>
#include "OccupyMap.h"
#include "ModelBldWithOccMap.h"
#include "IndexPointInPolygon.h"

double gBadAreaThre = 5.0;//meter
double gBadPntResThresh = 0.5;
double gCornerResThresh = 0.4;
double gLineResThresh = 0.4;

using namespace std;

void PrintUsage()
{
  printf("Usage: Evaluate3DBuildings -i <input laser points>\n"); 
  printf("							-iop <input 3D object points>\n"); 
  printf("                       -iot <input topology lines >\n");
}

void StatisticHist(FILE* file, const LaserPoints& cloud, Buildings& blds, ObjectPoints& map_points,
	const vector<PointNumberList>& pnlGloSegs, LaserPointTag tag);
void StatisticHist2(FILE* file, const LaserPoints& cloud, Buildings& blds, ObjectPoints& map_points,
				   const vector<PointNumberList>& pnlGloSegs, LaserPointTag tag);
void StatisticCornerResiHist(FILE* file, const LaserPoints& cloud, Buildings& blds,
	const vector<PointNumberList>& pnlGloSegs);
void StatisticLineResiHist(FILE* file, const LaserPoints& cloud, Buildings& blds,
	const vector<PointNumberList>& pnlGloSegs);

double ContBadArea(const LaserPoints& cloud, Building& bld,
	const vector<PointNumberList>& pnlGloSegs, double resiThr, double areaThr);
double ContBadArea2(const LaserPoints& cloud, Building& bld,
	const vector<PointNumberList>& pnlGloSegs, double resiThr, double areaThr);
double ContLineResidual(const LaserPoints& cloud, Building& bld, ObjectPoints& map_points);
double ContCornerResidual(const LaserPoints& cloud, const vector<PointNumberList>& gPnlSegs, Building& bld, ObjectPoints& map_points);

int ContBadBldNumber(vector<double>& vecResidual, double residThr);
void StatisticAverageAndSTD(vector<double>& vecResidual, double& ave, double& std);

PointNumberList RemoveNoiseAndWallPnts(const LaserPoints& lasPnts, const PointNumberList& pnlBld, double virtAngle, int maxSegPnt);
LaserPoints Select(const LaserPoints& inLasPnts, const PointNumberList& pnlSelet);
void ComputeDist2Roof(LaserPoints& lasPnt, const ObjectPoints& objPnts, const LineTopologies& lineTops);
Plane Polygon2Plane(const ObjectPoints& objPnts, const LineTopology& lineTop);
void ComputeDistance2Roof(LaserPoints& cloud, Buildings& blds, ObjectPoints& model_points);
void ComputeDist2Roof(LaserPoints& lasPnts, ObjectPoints& objPnts, LineTopologies& lineTops);

//Evaluate PCM models, 
//models are stored as models in building classes
//-i Middelburg.laser -iop Middelburg.Objpts -iot Middelburg.top -a 5.0 -r 0.5
int main(int argc, char *argv[])
{
	LaserPoints inPnts;
	InlineArguments args(argc, argv);

	// Check on required input files
	if (args.Contains("-usage") || !args.Contains("-i")
		|| !args.Contains("-iop") || !args.Contains("-iot"))
	{
		if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
		PrintUsage();
		return EXIT_SUCCESS;
	}

	LaserPoints laser_points;
	Buildings::iterator bld;
	Buildings buildings;
	ObjectPoints model_points;
	vector<PointNumberList> gPnlSegs;

	string path1 = args.String("-i");
	string path3 = args.String("-iop");
	string path4 = args.String("-iot");

	if (args.Contains("-a")) gBadAreaThre = args.Double("-a", 5.0);
	if (args.Contains("-r")) gBadPntResThresh = args.Double("-r", 0.5);

	char c[30];
	string histPath = path1.substr(0, path1.find_last_of('.'));
	sprintf(c, "%.1f", gBadAreaThre);
	histPath += "_"; histPath += c;
	sprintf(c, "%.1f", gBadPntResThresh);
	histPath += "_"; histPath += c;
	histPath += "_hist.txt";
	FILE* histFile = fopen(histPath.c_str(), "w+");
	if (!histFile)	{
		printf("Cannot open %s file/n", histPath.c_str());
		return 1;
	}

	laser_points.Read(args.String("-i"));

	//	map_points.Read()
	buildings.ImportPCMModelData(args.String("-iop"), args.String("-iot"), model_points);
	
//	std::vector<int> vecSegNums;
//	vecSegNums.reserve(buildings.size()*20>4000?buildings.size()*20:4000);
	//SearchSegNumsInBlds(buildings, vecSegNums);
	ComputeDistance2Roof(laser_points, buildings, model_points);
	laser_points.Write(args.String("-i"));
	MyDeriveSegPNL(laser_points, gPnlSegs, PolygonIDTag);
	StatisticHist2(histFile, laser_points, buildings, model_points, gPnlSegs, ResidualTag);

//	fclose(histFile);

    return EXIT_SUCCESS;
}

/*
void StatisticHist(FILE* file, const LaserPoints& cloud, Buildings& blds, ObjectPoints& map_points,
	const vector<PointNumberList>& pnlGloSegs, LaserPointTag tag)
{
	if (!file) return;

	vector<int> vecSegNums;
	vecSegNums.reserve(500);
//	LaserPoints bldCloud;
	int indSeg;
	//vector<double> vecResidual;
	double binWid = 0.05;
	int binNum = 2.0/binWid;
	//vector<int> vecResHis(binNum+1, 0);
	double temRes;
	int badPntNum;
	//int pntNumThresh = 400;
	//double badResThresh = 0.5;
	int pntNumThresh = gBadAreaThre;
	double badResThresh = gBadPntResThresh;
	
	int badBldNum = 0;
	vector<int> vecBadPntsPerBldHist = vector<int>(21, 0);

	//////////////////////////////////////////////////////////////////////////
	//compute component area
	//int compontNUm;
	double badarea, lineRes, pntRes;
	vector<double> vecBadArea, vecLineRes, vecPntRes;
	for (int iBld=0; iBld<blds.size(); ++iBld)	{
		if (iBld==30)	{
			int aaa = 0;
		}
		//printf("Building %i\r", iBld);
		badarea = ContBadArea(cloud, blds[iBld], pnlGloSegs,badResThresh,pntNumThresh/50.0);
		//if (compontNUm>0) badBldNum++;
		lineRes = ContLineResidual(cloud, blds[iBld], map_points);
		pntRes = ContCornerResidual(cloud, pnlGloSegs, blds[iBld], map_points);

		vecBadArea.push_back(badarea);
		vecLineRes.push_back(lineRes);
		vecPntRes.push_back(pntRes);
		//fprintf(file, "%.3f,	%.3f,	%.3f\n", badarea, lineRes, pntRes);
	}

	int badBldByBadArea = ContBadBldNumber(vecBadArea, gBadPntResThresh);
	int badBldByCornerRes = ContBadBldNumber(vecPntRes, gCornerResThresh);
	int badBldByLineRes = ContBadBldNumber(vecLineRes, gLineResThresh);
	fprintf(file, "Total Blds:%d\n", blds.size());
	fprintf(file, "Bad Bld Number: \n");
	fprintf(file, "By bad area: %4.2f, thr: %4.2f\n", badBldByBadArea*1.0/blds.size(), gBadAreaThre);
	fprintf(file, "By corner Res: %4.2f, thr: %4.2f\n", badBldByCornerRes*1.0/blds.size(), gCornerResThresh);
	fprintf(file, "By Line Res: %4.2f, thr: %4.2f\n", badBldByLineRes*1.0/blds.size(), gLineResThresh);


	fprintf(file, "bad area(m),	Line Residual(ratio),	Corner Residual(m)\n");
	for (int iBld=0; iBld<blds.size(); ++iBld)
		fprintf(file, "%6.3f,	%.3f,	%.3f\n", vecBadArea[iBld], vecLineRes[iBld], vecPntRes[iBld]);

}*/

void StatisticHist2(FILE* file, const LaserPoints& cloud, Buildings& blds, ObjectPoints& model_points,
					const vector<PointNumberList>& pnlGloSegs, LaserPointTag tag)
{
	if (!file) return;

//	vector<int> vecSegNums;
//	vecSegNums.reserve(500);
//	int indSeg;
//	double binWid = 0.05;
//	int binNum = 2.0/binWid;
//	double temRes;
//	int badPntNum;
	int pntNumThresh = gBadAreaThre;
	double badResThresh = gBadPntResThresh;
	
//	int badBldNum = 0;
//	vector<int> vecBadPntsPerBldHist = vector<int>(21, 0);

	//////////////////////////////////////////////////////////////////////////
	//compute component area
	//int compontNUm;
	double badarea, lineRes, pntRes;
	vector<double> vecBadArea;//, vecLineRes, vecPntRes;
	for (int iBld=0; iBld<blds.size(); ++iBld) {
		if (iBld==30) {
			int aaa = 0;
		}
		//printf("Building %i\r", iBld);
		badarea = ContBadArea2(cloud, blds[iBld], pnlGloSegs,badResThresh,pntNumThresh/50.0);
		//if (compontNUm>0) badBldNum++;
//		lineRes = ContLineResidual(cloud, blds[iBld], model_points);
//		pntRes = ContCornerResidual(cloud, pnlGloSegs, blds[iBld], model_points);

		vecBadArea.push_back(badarea);
//		vecLineRes.push_back(lineRes);
//		vecPntRes.push_back(pntRes);
		//fprintf(file, "%.3f,	%.3f,	%.3f\n", badarea, lineRes, pntRes);
	}

	//compute average residual
	double aveRes = 0.0;
	for (unsigned int i=0; i<cloud.size(); i++)
	{
		if (!cloud[i].HasAttribute(ResidualTag)) continue;
		aveRes += cloud[i].Residual()/cloud.size();
	}

	int badBldByBadArea = ContBadBldNumber(vecBadArea, gBadAreaThre);
//	int badBldByCornerRes = ContBadBldNumber(vecPntRes, gCornerResThresh);
//	int badBldByLineRes = ContBadBldNumber(vecLineRes, gLineResThresh);
	fprintf(file, "Total Blds: %d\n", blds.size());
//	fprintf(file, "Bad Bld Number: \n", badBldByBadArea);
	fprintf(file, "Bad Blds: %4.2f percent\n", 100*badBldByBadArea*1.0/blds.size());
	fprintf(file, "Area threhold: %4.2f m2, Residual threshold: %4.2f m\n",gBadAreaThre,gBadPntResThresh);
	fprintf(file, "Average Residual: %4.2f m\n", aveRes);
//	fprintf(file, "By corner Res: %4.2f, thr: %4.2f\n", badBldByCornerRes*1.0/blds.size(), gCornerResThresh);
//	fprintf(file, "By Line Res: %4.2f, thr: %4.2f\n", badBldByLineRes*1.0/blds.size(), gLineResThresh);


	fprintf(file, "Bad area(m) per building:\n");
	for (int iBld=0; iBld<blds.size(); ++iBld)
		fprintf(file, "%6.3f\n", vecBadArea[iBld]);
	//fprintf(file, "%6.3f,	%.3f,	%.3f\n", vecBadArea[iBld], vecLineRes[iBld], vecPntRes[iBld]);

}


void ComputeDistance2Roof(LaserPoints& cloud, Buildings& blds, ObjectPoints& model_points)
{
	vector<PointNumberList> vecPnlBlds;
	MyDeriveSegPNL(cloud, vecPnlBlds, PolygonIDTag);

	int indSeg ;//= IndexSegBySegNumber(cloud, vecPnlBlds, bldNum, PolygonIDTag);
	std::vector <LineTopologies *>* roofFaces;
	int bldNum;
	PointNumberList pnlRoofPnts;

	for (unsigned int i=0; i<blds.size(); i++) {
		roofFaces = blds[i].RoofFaces();
		if(roofFaces == NULL) continue;
		LineTopologies& lrFaces = *(*roofFaces)[0];
		if (lrFaces.empty()) continue;

		bldNum = lrFaces[0].Attribute(IDNumberTag);
		indSeg = IndexSegBySegNumber(cloud, vecPnlBlds, bldNum, PolygonIDTag);
		if (indSeg==-1) continue;
		pnlRoofPnts = RemoveNoiseAndWallPnts(cloud, vecPnlBlds[indSeg], 15.0, 20);

		LaserPoints lLasPnts = Select(cloud, pnlRoofPnts);
		ComputeDist2Roof(lLasPnts, model_points, lrFaces);

		for (unsigned int j=0; j<lLasPnts.size(); j++)
			cloud[pnlRoofPnts[j].Number()].Residual() = lLasPnts[j].Residual();
	}
}


//virtAngle: [0-180) degree
//maxSegPnt: segment number
PointNumberList RemoveNoiseAndWallPnts(const LaserPoints& lasPnts, const PointNumberList& pnlBld, double virtAngle, int maxSegPnt)
{
	//construct local point
	LaserPoints localPnt;
	localPnt.reserve(pnlBld.size());
	for (unsigned int i=0; i<pnlBld.size(); i++)
		localPnt.push_back(lasPnts[pnlBld[i].Number()]);

	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(localPnt, vecSegPnls, SegmentNumberTag);

	Plane plane;
	vector<PointNumberList>::iterator itrSeg;
	PointNumberList pnlRstLocal;

	//tag valid point	
	for (itrSeg=vecSegPnls.begin(); itrSeg!=vecSegPnls.end(); itrSeg++)	{
		if (itrSeg->size() < maxSegPnt) continue;
		plane = localPnt.FitPlane(*itrSeg);
		if (plane.IsVertical(virtAngle*PI/180.0)) continue;

		pnlRstLocal.insert(pnlRstLocal.end(), itrSeg->begin(), itrSeg->end());
	}

	PointNumberList pnlRst;
	for (unsigned int i=0; i<pnlRstLocal.size(); i++)
		pnlRst.push_back(pnlBld[pnlRstLocal[i].Number()]);
	
	return pnlRst;
}

LaserPoints Select(const LaserPoints& inLasPnts, const PointNumberList& pnlSelet)
{
	LaserPoints rstLasPnts;
	int pntNum;
	rstLasPnts.reserve(pnlSelet.size());

	for (unsigned int i=0; i<pnlSelet.size(); i++) {
		pntNum = pnlSelet[i].Number();
		
		if (pntNum>=0 && pntNum<inLasPnts.size())
			rstLasPnts.push_back(inLasPnts[pntNum]);
	}

	return rstLasPnts;
}

Plane Polygon2Plane(const ObjectPoints& objPnts, const LineTopology& lineTop)
{
	if (lineTop.size()<3) return Plane();

	LaserPoints lasPnt;
	PointNumberList pnl;

	for (unsigned int i=0; i<lineTop.size()-1; i++) {
		lasPnt.push_back(objPnts[lineTop[i].Number()].Position3DRef());
		pnl.push_back(PointNumber(i));
	}
	
	if (!lineTop.IsClosed()) {
		lasPnt.push_back(objPnts[lineTop[lineTop.size()-1].Number()].Position3DRef());
		pnl.push_back(PointNumber(lineTop.size()-1));
	}

	Plane plane = lasPnt.FitPlane(pnl);

	return plane;
}

void ComputeDist2Roof(LaserPoints& lasPnts, ObjectPoints& objPnts, LineTopologies& lineTops)
{
	Plane plane;
	lasPnts.SetAttribute(ResidualTag, 2.0f);
	vector<bool> vecProcessed (lasPnts.size(), false);

	for (unsigned int i=0; i<lineTops.size(); i++) {
		plane = Polygon2Plane(objPnts, lineTops[i]);
	
		for (unsigned int j=0; j<lasPnts.size(); j++) {
			if (vecProcessed[j]) continue;

			if (lasPnts[j].InsidePolygon(objPnts, lineTops[i]))	{
				double distance = plane.Distance(lasPnts[j]);
				lasPnts[j].Residual() = fabs(distance);
				vecProcessed[j] = true;
			}
		}
	}
}

//check the area of bad components
//return the number of components whose area is larger than threshold
double ContBadArea(const LaserPoints& cloud, Building& bld,
	const vector<PointNumberList>& pnlGloSegs, double resiThr, double areaThr)
{
	vector<int> vecSegNums;
	vecSegNums.reserve(500);
	SearchSegNumsInBld(bld, vecSegNums);
	int indSeg;
	LaserPoints localPnts;
	LaserPoints::const_iterator pnt;

	//collect all points inside the building
	for (int iSeg=0; iSeg<vecSegNums.size(); ++iSeg) {
		indSeg = IndexSegBySegNumber(cloud, pnlGloSegs, vecSegNums[iSeg], SegmentNumberTag);
		if (indSeg==-1) continue;

		for (int iPnt=0; iPnt<pnlGloSegs[indSeg].size(); ++iPnt) {
			pnt = cloud.begin()+pnlGloSegs[indSeg][iPnt].Number();
			if (pnt->Residual()>resiThr)
				localPnts.push_back(*pnt);
		}
	}

	localPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(localPnts.TINReference());
	if (edges.size()!=localPnts.size()) return 0;
	localPnts.RemoveLongEdges(edges, 0.75, false);
	localPnts.LabelComponents(edges, ComponentNumberTag);

	vector<PointNumberList> vecComps;
	vecComps.reserve(20);
	MyDeriveSegPNL(localPnts, vecComps, ComponentNumberTag);
	LineTopology contour;
	ObjectPoints contourPnts;
	ObjectPoint temPnt;
	for (int i=0; i<localPnts.size(); i++){
		temPnt.Position3DRef() = localPnts[i].Position3DRef();
		temPnt.Number() = i;
		contourPnts.push_back(temPnt);
	}

	double area,badAreaSum=0.0;
	int badComp = 0;
	LineTopologies allContours;
	PointNumberList pnlCountour;
	for (int i=0; i<vecComps.size(); ++i){
		//skip good component
		//contour = localPnts.DeriveContour(0, vecComps[i], edges);
		//contour.push_back(contour[0]);

		if(vecComps[i].size()<30) continue;
		MyDeriveContour(localPnts, vecComps[i], pnlCountour);
		//DeriveContour_AlphaShape(localPnts, vecComps[i], pnlCountour,10);
		if (pnlCountour.empty()) continue;
		contour.clear();
		for (int j=0; j<pnlCountour.size(); ++j) 
			contour.push_back(pnlCountour[j]);

		area = contour.CalculateArea(contourPnts);
		if (area == -1.0){
			allContours.push_back(contour);
			allContours.Write("allcontours.top");
			contourPnts.Write("allcontours.objpts");
			localPnts.Write("allcomponent.laser");
		}
		if (area>areaThr) {
			badComp++;
		}
		if (area>0.0) badAreaSum +=area;
		//MyDeriveContour(localPnts, edges, vecComps[i]);
	}

	return badAreaSum;
}

//Jan 8, 2016
//check the area of bad components
//return the number of components whose area is larger than threshold
double ContBadArea2(const LaserPoints& cloud, Building& bld,
				   const vector<PointNumberList>& pnlGloSegs, double resiThr, double areaThr)
{
	//find laser points for this building
	LineTopologies ltModel;
	bld.CollectModelData(ltModel);
	if (ltModel.empty()) return 0.0;
	int bldID = ltModel[0].Attribute(IDNumberTag);
	int indSeg = IndexSegBySegNumber(cloud, pnlGloSegs, bldID, PolygonIDTag);
	if (indSeg == -1) return 0.0;

	//find laser points with large residual
	LaserPoints localPnts;
	LaserPoints::const_iterator pnt;

	for (int iPnt=0; iPnt<pnlGloSegs[indSeg].size(); ++iPnt) {
		pnt = cloud.begin()+pnlGloSegs[indSeg][iPnt].Number();
		if (pnt->Residual()>resiThr)
			localPnts.push_back(*pnt);
	}

	//derive components for points with large residual
	localPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(localPnts.TINReference());
	if (edges.size()!=localPnts.size()) return 0;
	localPnts.RemoveLongEdges(edges, 0.75, false);
	localPnts.LabelComponents(edges, ComponentNumberTag);

	vector<PointNumberList> vecComps;
	vecComps.reserve(20);
	MyDeriveSegPNL(localPnts, vecComps, ComponentNumberTag);
	LineTopology contour;
	ObjectPoints contourPnts;
	ObjectPoint temPnt;
	for (int i=0; i<localPnts.size(); i++){
		temPnt.Position3DRef() = localPnts[i].Position3DRef();
		temPnt.Number() = i;
		contourPnts.push_back(temPnt);
	}

	double area,badAreaSum=0.0;
	int badComp = 0;
	LineTopologies allContours;
	PointNumberList pnlCountour;
	for (int i=0; i<vecComps.size(); ++i){
		//skip good component
		//contour = localPnts.DeriveContour(0, vecComps[i], edges);
		//contour.push_back(contour[0]);

		if(vecComps[i].size()<30) continue;
		MyDeriveContour(localPnts, vecComps[i], pnlCountour);
		//DeriveContour_AlphaShape(localPnts, vecComps[i], pnlCountour,10);
		if (pnlCountour.empty()) continue;
		contour.clear();
		for (int j=0; j<pnlCountour.size(); ++j) 
			contour.push_back(pnlCountour[j]);

		area = contour.CalculateArea(contourPnts);
		if (area == -1.0){
			allContours.push_back(contour);
			allContours.Write("allcontours.top");
			contourPnts.Write("allcontours.objpts");
			localPnts.Write("allcomponent.laser");
		}
		if (area>areaThr) {
			badComp++;
		}
		if (area>0.0) badAreaSum +=area;
		//MyDeriveContour(localPnts, edges, vecComps[i]);
	}

	return badAreaSum;
}


void StatisticCornerResiHist(FILE* file, const LaserPoints& cloud, Buildings& blds,
	const vector<PointNumberList>& pnlGloSegs)
{

}

void StatisticLineResiHist(FILE* file, const LaserPoints& cloud, Buildings& blds)
{
	
}

double ContLineResidual(const LaserPoints& cloud, Building& bld, ObjectPoints& map_points)
{
	LineTopologies* mapLines = bld.MapDataPtr();
	double residual=0.0;//in fact it is ratio of points between two flank area.
	double temRatio;
	double len = 0.0;
	double lenSum = 0.00001;
	LineTopologies::iterator line;

	for (int i=0; i<mapLines->size(); i++) {
		line = mapLines->begin()+i;
		temRatio = line->Attribute(GradientTag);
		temRatio = 1.0-(temRatio-0.5)/6.0;
		len = map_points[(*line)[0].Number()].Distance(map_points[(*line)[1].Number()]);
		residual += temRatio*len;
		lenSum += len;
	}

	residual /= lenSum;
	if (residual>1.0) residual = 1.0;
	return residual;
}

double ContCornerResidual(const LaserPoints& cloud, const vector<PointNumberList>& gPnlSegs, Building& bld, ObjectPoints& map_points)
{
	double residual=0.0;
	//collect corners
	LineTopologies* mapLines = bld.MapDataPtr();
	vector<Position3D> corners;
	LineTopologies::iterator line;
	for (int i=0; i<mapLines->size(); i++) {
		int iii = i;
		line = mapLines->begin()+i;
		corners.push_back(map_points[(*line)[0].Number()]);
	}
	std::sort(corners.begin(), corners.end());
	

	corners.erase(std::unique(corners.begin(), corners.end()), corners.end());

	//collect laser points for this building
	LaserPoints localLasPnts;
	vector<int> vecSegNums;
	int indSeg;
	SearchSegNumsInBld(bld, vecSegNums);
	for (int iseg=0; iseg<vecSegNums.size(); iseg++) {
		indSeg = IndexSegBySegNumber(cloud, gPnlSegs, vecSegNums[iseg]);
		if (indSeg==-1) continue;
		
		for (int i=0; i<gPnlSegs[indSeg].size(); i++)
			localLasPnts.push_back(cloud[gPnlSegs[indSeg][i].Number()]);
	}

	//search the nearest laser point for each corner
	double minDist,temDist;
	vector<double> vecRes;
	
	for (vector<Position3D>::iterator itrC=corners.begin(); itrC!=corners.end(); itrC++) {
		minDist = 99999.9;
		for (int ipnt=0; ipnt<localLasPnts.size(); ipnt++) {
			temDist = itrC->Distance(localLasPnts[ipnt]);
			if (temDist<minDist) minDist = temDist;
			if (minDist<0.001) break;
		}

		vecRes.push_back(minDist);
		residual += minDist;
	}
	residual /= vecRes.size();

	return residual;
}


//////////////////////////////////////////////////////////////////////////
//for sander's model
//-im Middelburg.laser -ic Middelburg.top
void StatisticHistSander(FILE* file, LaserPoints& LasPntsModel, vector<PointNumberList>& pnlModelSegs, 
	LaserPoints& LasPntsCorner, vector<PointNumberList>& pnlCornerSegs);
double ContBadArea(const LaserPoints& cloud, PointNumberList& pnlBld);
double ContCornerResidual(const LaserPoints& cloud, const PointNumberList& pnlBld);
void StatisticHistSander(FILE* file, LaserPoints& LasPntsModel, vector<PointNumberList>& pnlModelSegs, 
	LaserPoints& LasPntsCorner, vector<PointNumberList>& pnlCornerSegs);
void TagBldNumberToCorners(LaserPoints& lasPntBlds, LaserPoints& lasPntCorners);


void StatisticHistSander(FILE* file, LaserPoints& LasPntsModel, vector<PointNumberList>& pnlModelSegs, 
	LaserPoints& LasPntsCorner, vector<PointNumberList>& pnlCornerSegs)
{
	if (!file)	return;

	//////////////////////////////////////////////////////////////////////////
	//overall average and STD
	double toModelAve, toModelSTD, toCornerAve, toCornerSTD;
	vector<double> vecResidual;
	for (int i=0; i<LasPntsModel.size(); i++)
		vecResidual.push_back(LasPntsModel[i].Residual());
	StatisticAverageAndSTD(vecResidual, toModelAve, toModelSTD);
	vecResidual.clear();
	for (int i=0; i<LasPntsCorner.size(); i++)
		vecResidual.push_back(LasPntsCorner[i].Residual());
	StatisticAverageAndSTD(vecResidual, toCornerAve, toCornerSTD);

	//////////////////////////////////////////////////////////////////////////
	//per building
	vector<int> vecBldNum;
	for (int i=0; i<pnlModelSegs.size(); i++)
		vecBldNum.push_back(LasPntsModel[pnlModelSegs[i][0].Number()].Attribute(PolygonNumberTag));

	double badArea, cornerRes;
	int indPnlModel, indPnlCorner;
	vector<double> vecBadArea, vecCornerRes;
	for (int iBld=0; iBld<vecBldNum.size(); ++iBld) {
		badArea = 10.0; cornerRes = 1.0;

		indPnlModel = IndexSegBySegNumber(LasPntsModel, pnlModelSegs, vecBldNum[iBld], PolygonNumberTag);
		if (indPnlModel!=-1)
			badArea = ContBadArea(LasPntsModel, pnlModelSegs[iBld]);

		indPnlCorner = IndexSegBySegNumber(LasPntsCorner, pnlCornerSegs, vecBldNum[iBld], PolygonNumberTag);
		if (indPnlCorner!=-1)
			cornerRes = ContCornerResidual(LasPntsCorner, pnlCornerSegs[indPnlCorner]);
		vecBadArea.push_back(badArea);
		vecCornerRes.push_back(cornerRes);
	}

	int badBldByBadArea = ContBadBldNumber(vecBadArea, gBadAreaThre);
	int badBldByCornerRes = ContBadBldNumber(vecCornerRes, gCornerResThresh);

	fprintf(file, "Total Point Numbers:%d\n", LasPntsModel.size());
	fprintf(file, "Overall residual average and STD:\n");	
	fprintf(file, "  To model: %.3f , %.3f; To corner: %.3f, %.3f\n", toModelAve, toModelSTD, toCornerAve, toCornerSTD);
	
	fprintf(file, "\n\nTotal Blds:%d\n", vecBldNum.size());
	fprintf(file, "Bad Bld Percent: By bad area: %4.2f%% , thr: %4.2f; By corner res: %4.2f%% , thr: %4.2f\n", 
		badBldByBadArea*100.0/vecBldNum.size(), gBadAreaThre, badBldByCornerRes*100.0/vecBldNum.size(), gCornerResThresh);

	fprintf(file, "bad area(m),	corner residual(m)\n");
	for (int i=0; i<vecBadArea.size(); i++)
		fprintf(file, "%8.3f ,	%.3f\n", vecBadArea[i], vecCornerRes[i]);
}

//check the area of bad components
//return the number of components whose area is larger than threshold
double ContBadArea(const LaserPoints& cloud, PointNumberList& pnlBld)
{
	//collect all points in side the building
	LaserPoints localPnts;
	LaserPoints::const_iterator pnt;
	for (int i=0; i<pnlBld.size(); i++) {
		pnt = cloud.begin()+pnlBld[i].Number();
		if (pnt->Residual()>0.5)
			localPnts.push_back(*pnt);
	}

#ifdef _DEBUG
	localPnts.Write("debug.laser");
#endif

	//
	localPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(localPnts.TINReference());
	if (edges.size()!=localPnts.size()) return 0;
	localPnts.RemoveLongEdges(edges, 0.75, false);
	localPnts.LabelComponents(edges, ComponentNumberTag);

	vector<PointNumberList> vecComps;
	vecComps.reserve(20);
	MyDeriveSegPNL(localPnts, vecComps, ComponentNumberTag);
	LineTopology contour;
	ObjectPoints contourPnts;
	ObjectPoint temPnt;
	for (int i=0; i<localPnts.size(); i++){
		temPnt.Position3DRef() = localPnts[i].Position3DRef();
		temPnt.Number() = i;
		contourPnts.push_back(temPnt);
	}

	double area, badAreaSum=0.0;
	int badComp = 0;
	LineTopologies allContours;
	PointNumberList pnlCountour;
	for (int i=0; i<vecComps.size(); ++i){
		//skip good component
		//contour = localPnts.DeriveContour(0, vecComps[i], edges);
		//contour.push_back(contour[0]);

		if(vecComps[i].size()<30) continue;
		MyDeriveContour(localPnts, vecComps[i], pnlCountour);
		//DeriveContour_AlphaShape(localPnts, vecComps[i], pnlCountour,10);
		if (pnlCountour.empty()) continue;
		contour.clear();
		for (int j=0; j<pnlCountour.size(); ++j) 
			contour.push_back(pnlCountour[j]);

		area = contour.CalculateArea(contourPnts);
		if (area == -1.0){
			allContours.push_back(contour);
			allContours.Write("allcontours.top");
			contourPnts.Write("allcontours.objpts");
			localPnts.Write("allcomponent.laser");
		}

		if (area>0.0) badAreaSum +=area;
	}

	return badAreaSum;
}


double ContCornerResidual(const LaserPoints& cloud, const PointNumberList& pnlBld)
{
	double residual = 0.0;

	for (int i=0; i<pnlBld.size(); i++) {
		residual += cloud[pnlBld[i].Number()].Residual();
	}

	return residual/pnlBld.size();
}

int ContBadBldNumber(vector<double>& vecResidual, double residThr)
{
	int count = 0;
	for (int i=0; i<vecResidual.size(); i++) {
		if (vecResidual[i]>residThr) 
			count ++;
	}

	return count;
}

void StatisticAverageAndSTD(vector<double>& vecResidual, double& ave, double& std)
{
	if (vecResidual.empty()) return;

	ave = 0.0;
	for (int i=0; i<vecResidual.size(); ++i)
		ave += vecResidual[i];
	ave /= vecResidual.size();

	std = 0.0;
	for (int i=0; i<vecResidual.size(); ++i)
		std += (vecResidual[i]-ave)*(vecResidual[i]-ave);

	std /= vecResidual.size();
	std = sqrt(std);
}

//las points of corners have same segment numbers with building points
//but have no polygon numbers
//tag polygon numbers of building points to corner points
void TagBldNumberToCorners(LaserPoints& lasPntBlds, LaserPoints& lasPntCorners)
{
	vector<PointNumberList> vecPnlRoofOfBldPnts;
	MyDeriveSegPNL(lasPntBlds, vecPnlRoofOfBldPnts, SegmentNumberTag);

	int indSeg;
	LaserPoints::iterator pnt, pnt2;
	for (int i=0; i<lasPntCorners.size(); i++)	{
		pnt = lasPntCorners.begin()+i;
		indSeg = IndexSegBySegNumber(lasPntBlds, vecPnlRoofOfBldPnts, pnt->Attribute(SegmentNumberTag), SegmentNumberTag);
		if (indSeg==-1) continue;
		pnt2 = lasPntBlds.begin()+vecPnlRoofOfBldPnts[indSeg][0].Number();
		pnt->SetAttribute(PolygonNumberTag, pnt2->Attribute(PolygonNumberTag));
	}
}
