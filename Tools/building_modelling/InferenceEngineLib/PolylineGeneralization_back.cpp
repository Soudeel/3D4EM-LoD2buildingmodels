/* -----------------------------------------------------------------
|Initial Creation: Biao Xiong
|Data: Jan, 17, 2014
|
|
------------------------------------------------------------------- */

#include "PolylineGeneralization.h"
#include "houghspace.h"
#include "DataBoundsLaser.h"
#include "LaserPoints.h"
#include "LineSegment2D.h"
#include "LineSegments2D.h"
#include "InferenceEngine.h"

#include <limits>

using namespace std;

//#define PI 4*atan(1.0)

HoughSpace* CreateHoughSpace2D(const LaserPoints& inLasPnts, 
							   const OutliningParameters& para=OutliningParameters(), 
							   const std::vector<double>& vecConstDirs=std::vector<double>());
void DestroyHoughSpace2D(HoughSpace* houghSpace);
bool AddOnePointToPolygon(ObjectPoints& outObjPnts, LineTopology& outLineTops, const Position2D& inPnt);



#include <set>

	//April 24, 2015
IEDll_API 
	void PolylineGeneralizeByHoughTransform(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const OutliningParameters& outlining_pars,
	const std::vector<double>& vecConstDirs	)
{
	bool bCoverAllPnts = true;
	outObjPnts.clear();
	outLineTop.clear();
	if (pnlPolyline.size()<2) return;

	//OutliningParameters outlining_pars;
	//outlining_pars.MaximumDistancePointOutline() = 1.0;
	LaserPoints localLasPnts;
	LineTopology localLasContour, polygon;
	for (unsigned int i=0; i<pnlPolyline.size(); i++) {
		localLasPnts.push_back(lasPnts[pnlPolyline[i].Number()]);
		localLasContour.push_back(PointNumber(i));
	}
	localLasPnts.Label(-1);

	//is closed polyline
	bool bPolygon = (pnlPolyline[0]==pnlPolyline[pnlPolyline.size()-1]);
	if (bPolygon) {
		localLasPnts.erase(localLasPnts.begin()+localLasPnts.size()-1);
		localLasContour[localLasContour.size()-1] = localLasContour[0];
	}

#ifdef _DEBUG
	localLasPnts.Write("debug.laser");
#endif

	HoughSpace* houghSpace = CreateHoughSpace2D(localLasPnts,  outlining_pars, vecConstDirs);
	if (!houghSpace) return;

	Line2D line, outer_edge, bisector;
	int num_pts, label=0;
	LineTopologies segment_contours, contour_segments, contour_edges, polygons;
	vector<Line2D> vecLines;
	localLasPnts.SetAttribute(PlaneNumberTag, -1);

	int nLoop=0;
	line = houghSpace->BestLine(&num_pts, 0, 1);
	while (num_pts >= outlining_pars.MinimumNumberOfPointsInOutlineSegment() &&nLoop++<500) 
	{
		//Label possible points belong to this line
		double meanDist = 0.0;
		int numPntsPossible = 0;
		for (LaserPoints::iterator lasPnt=localLasPnts.begin(); lasPnt!=localLasPnts.end()-1; lasPnt++) 
		{
			if (lasPnt->Attribute(LabelTag)!=-1) continue;
			if (line.DistanceToPoint(lasPnt->Position2DOnly())>=1.6*outlining_pars.MaximumDistancePointOutline())
				continue;

			lasPnt->Label(label);
			numPntsPossible++;
			meanDist += line.DistanceToPointSigned(lasPnt->Position2DOnly());
		}

		//refit the line, and precisely label possible points belong to this line
		meanDist /= numPntsPossible;
		line.SetDistanceToOrigin(line.DistanceToOrigin()+meanDist);

		for (LaserPoints::iterator lasPnt=localLasPnts.begin(); lasPnt!=localLasPnts.end()-1; lasPnt++) 
		{
			if (lasPnt->Attribute(LabelTag)!=label) continue;

			if (line.DistanceToPoint(lasPnt->Position2DOnly())>=outlining_pars.MaximumDistancePointOutline())
				lasPnt->SetAttribute(LabelTag,-1);
			else
				houghSpace->RemovePoint(lasPnt->Position2DOnly());
		}

		// Extract the connected point sequences with of this line, allowing
		// gaps of a certain number of points and a certain size
		contour_segments = localLasPnts.TaggedSequences(localLasContour, label, LabelTag, 
			outlining_pars.MaximumPointGapInOutlineSegment(),
			outlining_pars.MaximumGapSizeInOutlineSegment());

		
		// Select segments larger than a minimum size
		for (LineTopologies::iterator contour_segment=contour_segments.begin();
			contour_segment!=contour_segments.end(); contour_segment++) {
				//un-label short segment
				if (contour_segment->size() < outlining_pars.MinimumNumberOfPointsInOutlineSegment()) {
					for (LineTopology::iterator node=contour_segment->begin(); node!=contour_segment->end(); node++)
						localLasPnts[node->Number()].SetAttribute(LabelTag,-1);
					continue;
				}
				else {
					// If the first edge of a contour segment makes a large angle
					// with the line, remove that edge and put the point back into the
					// Hough space. From what is left also put the point back into the
					// Hough space.
					LaserPoints::iterator laser_point, laser_point2;
					laser_point = localLasPnts.begin()+contour_segment->begin()->Number();
					laser_point2 = localLasPnts.begin()+(contour_segment->begin()+1)->Number();
					outer_edge = Line2D(laser_point->Position2DOnly(), laser_point2->Position2DOnly());
					houghSpace->AddPoint(laser_point->Position2DOnly());
					laser_point->SetAttribute(LabelTag, -1);
					if (Angle2Lines(line, outer_edge) > atan(1.0)) {
						contour_segment->erase(contour_segment->begin());
						houghSpace->AddPoint(laser_point2->Position2DOnly());
						laser_point->SetAttribute(LabelTag, -1);
					}

					// Do the same for the last edge
					laser_point = localLasPnts.begin()+(contour_segment->end()-1)->Number();
					laser_point2 = localLasPnts.begin()+(contour_segment->end()-2)->Number();
					outer_edge = Line2D(laser_point->Position2DOnly(), laser_point2->Position2DOnly());
					houghSpace->AddPoint(laser_point->Position2DOnly());
					laser_point->SetAttribute(LabelTag, -1);
					if (Angle2Lines(line, outer_edge) > atan(1.0)) {
						contour_segment->erase(contour_segment->end()-1);
						houghSpace->AddPoint(laser_point2->Position2DOnly());
						laser_point->SetAttribute(LabelTag, -1);
					}
				}

		}

		// Extract the next line from the Hough space    
		vecLines.push_back(line);
		label++;
		line = houghSpace->BestLine(&num_pts, 0, 1);
	}

#ifdef _DEBUG
	localLasPnts.Write("debug.laser");
#endif

	//////////////////////////////////////////////////////////////////////////
	//re-assign unlabeled points to nearby line segments
	vector<int> vecPntInds; 
	int lable0, lable1, nextSegStartInd;
	for (int i=0; i<localLasContour.size(); i++) {
		lable0 = localLasPnts[localLasContour[i].Number()].Label();
		nextSegStartInd = (i+1)%(localLasContour.size()-1);
		lable1 = localLasPnts[localLasContour[nextSegStartInd].Number()].Label();

		if (lable0==-1 || lable1!=-1) continue;
		
		//get the unlabeled pnl
		vecPntInds.clear();
		for (int j=(i+1)%(localLasContour.size()-1); j!=i; j=(++j)%(localLasContour.size()-1)) {
			if (localLasPnts[localLasContour[j].Number()].Label()==-1)
				vecPntInds.push_back(j);
			else
				break;
		}

		nextSegStartInd = (vecPntInds[vecPntInds.size()-1]+1)%(localLasContour.size()-1);
		lable1 = localLasPnts[localLasContour[nextSegStartInd].Number()].Label();
		//inside a segment
		if (lable0==lable1)	{
			for (int j=0; j<vecPntInds.size(); j++)
				localLasPnts[localLasContour[vecPntInds[j]].Number()].Label(lable0);
		}
		else {//two different segments
			Line2D& line0 = vecLines[lable0];
			Line2D& line1 = vecLines[lable1];
			int indBreakPos = vecPntInds.size()/2;

			for (int j=0; j<vecPntInds.size(); j++)	{
				LaserPoint& lasPnt = localLasPnts[localLasContour[vecPntInds[j]].Number()];
				double dist0 = line0.DistanceToPoint(lasPnt.Position2DOnly());
				double dist1 = line1.DistanceToPoint(lasPnt.Position2DOnly());

				if (line0.DistanceToPoint(lasPnt.Position2DOnly()) > line1.DistanceToPoint(lasPnt.Position2DOnly())) {
					indBreakPos = j;
					break;
				}
			}

			for (int j=0; j<indBreakPos; j++)
				localLasPnts[localLasContour[vecPntInds[j]].Number()].Label(lable0);
			for (int j=indBreakPos; j<vecPntInds.size(); j++)
				localLasPnts[localLasContour[vecPntInds[j]].Number()].Label(lable1);
		}
	}



#ifdef _DEBUG
	localLasPnts.Write("debug.laser");
	outObjPnts.Write("debug.objpts");
	contour_edges.Write("debug.top");
#endif

	//////////////////////////////////////////////////////////////////////////
	//construct line segments
	double scalar, scalar_min, scalar_max, dist, max_dist, min_dist;
	Position2D pos2D;
	for (int i=0; i<=label; i++) {
		contour_segments = localLasPnts.TaggedSequences(localLasContour, i, LabelTag, 
			outlining_pars.MaximumPointGapInOutlineSegment(),
			outlining_pars.MaximumGapSizeInOutlineSegment());
		
		for (LineTopologies::iterator itrContSeg=contour_segments.begin(); itrContSeg!=contour_segments.end(); itrContSeg++) {
			//get max and min distance of line shifting to original point
			min_dist = std::numeric_limits<double>::max();
			max_dist = std::numeric_limits<double>::min();
			line = vecLines[localLasPnts[contour_segments[0][0].Number()].Label()];

			for (LineTopology::iterator node=itrContSeg->begin();node!=itrContSeg->end(); node++) {
				LaserPoints::iterator laser_point = localLasPnts.begin() + node->Number();
				dist = line.DistanceToPointSigned(laser_point->Position2DOnly());
				if (dist > max_dist) max_dist = dist;
				if (dist < min_dist) min_dist = dist;
			}

			scalar_min = line.Scalar(localLasPnts[(*itrContSeg)[0].Number()].Position2DOnly());
			scalar_max = line.Scalar(localLasPnts[(*itrContSeg)[(*itrContSeg).size()-1].Number()].Position2DOnly());

			if (bCoverAllPnts) {
				// Correct the line distance to origin
				line.SetDistanceToOrigin(line.DistanceToOrigin() + max_dist);
				// Get the mid point of the line segment
				pos2D = line.Position((scalar_min + scalar_max) / 2.0);
				LaserPoint check_point;
				check_point.X() = pos2D.X();
				check_point.Y() = pos2D.Y();

				// Correct the distance if the wrong side was taken and flip the line
				if (check_point.InsidePolygon(localLasPnts, localLasContour))
					line.SetDistanceToOrigin(line.DistanceToOrigin()-max_dist+min_dist);
			}

			//expand the start and end points
			for (LineTopology::iterator node=itrContSeg->begin();node!=itrContSeg->end(); node++) {
				LaserPoints::iterator laser_point = localLasPnts.begin() + node->Number();
				scalar = line.Scalar(laser_point->Position2DOnly());
				
				if (scalar_min < scalar_max) {
					if (scalar < scalar_min)
						scalar_min = scalar;
					else if (scalar > scalar_max)
						scalar_max = scalar;
				} else {
					if (scalar > scalar_min)
						scalar_min = scalar;
					else if (scalar < scalar_max)
						scalar_max = scalar;
				}
			}

			// Get the end points of the segment
			pos2D = line.Position(scalar_min);
			ObjectPoint point;
			point.X() = pos2D.X();
			point.Y() = pos2D.Y();
			point.Number() = outObjPnts.size();
			outObjPnts.push_back(point);
			polygon.clear();
			polygon.push_back(point.Number());

			pos2D = line.Position(scalar_max);
			point.X() = pos2D.X();
			point.Y() = pos2D.Y();
			point.Number() = outObjPnts.size();
			outObjPnts.push_back(point);
			polygon.push_back(point.Number());


			polygon.Number() = localLasContour.FindPoint(*(itrContSeg->end()-2));
			polygons.push_back(polygon);
			contour_edges.push_back(polygon);
		}
	}

#ifdef _DEBUG
	localLasPnts.Write("debug.laser");
	outObjPnts.Write("debug.objpts");
	contour_edges.Write("debug.top");
#endif

	// Check that we have at least three contour edges
	if (contour_edges.size() < 3) return ;


	// Put the contour segments in the right order by sorting them on the line number
	contour_edges.Sort();

	// Close gaps by intersecting contour segments and constructing perpendicular
	// connections
	LineTopologies::iterator contour_segment, contour_segment2;
	LineTopology unused_points;
	contour_segment = contour_edges.end() - 1;
	for (contour_segment2=contour_edges.begin();
		contour_segment2!=contour_edges.end();
		contour_segment=contour_segment2, contour_segment2++) 
	{
		LineSegment2D edge = *(LineSegments2D(outObjPnts, *contour_segment).begin());
		LineSegment2D edge2 = *(LineSegments2D(outObjPnts, *contour_segment2).begin());
		// Intersect lines if angle is larger than 45 degrees
		if (Angle2Lines(edge.Line2DReference(), edge2.Line2DReference()) > atan(1.0)) {
			Position2D pos2D;
			Intersection2Lines(edge.Line2DReference(), edge2.Line2DReference(), pos2D);
			// Reset one end point to the new corner point
			ObjectPoints::iterator corner_point = outObjPnts.PointIterator(*(contour_segment->end()-1));
			corner_point->X() = pos2D.X();
			corner_point->Y() = pos2D.Y();
			outLineTop.push_back(*(contour_segment->end()-1));
			// Store the point number of the other point for deletion later on
			unused_points.push_back(*(contour_segment2->begin()));
		}
		// Otherwise, construct a perpendicular connection line
		else {
			// Construct bisector
			bisector = Bisector(edge.Line2DReference(), edge2.Line2DReference());
			// Get scalars of projections of both points on bisector
			ObjectPoints::iterator corner_point = outObjPnts.PointIterator(*(contour_segment->end()-1));
			double scalar = bisector.Scalar(corner_point->Position2DOnly());
			ObjectPoints::iterator corner_point2 = outObjPnts.PointIterator(*(contour_segment2->begin()));
			double scalar2 = bisector.Scalar(corner_point2->Position2DOnly());
			// Get midpoint projection on bisector
			Position2D pos2D = bisector.Position((scalar+scalar2)/2.0);
			// Construct perpendicular line through this point
			line = bisector.PerpendicularLine(pos2D);
			// Intersect this line with the two segment edges to construct new
			// corner points
			Intersection2Lines(edge.Line2DReference(), line, pos2D);
			corner_point->X() = pos2D.X();
			corner_point->Y() = pos2D.Y();
			outLineTop.push_back(corner_point->NumberRef());
			Intersection2Lines(edge2.Line2DReference(), line, pos2D);
			corner_point2->X() = pos2D.X();
			corner_point2->Y() = pos2D.Y();
			outLineTop.push_back(corner_point2->NumberRef());
		}
	}

	// Close the polygon
	outLineTop.push_back(*(outLineTop.begin()));
	// Clear all old polygons
	polygons.Erase();

	LineTopologies temLineTops;
	temLineTops.push_back(outLineTop);
	outObjPnts.RemoveUnusedPnts(temLineTops);
	outLineTop = temLineTops[0];
}

LineSegment2D CreateExteriorLineSegment2D(
	const LaserPoints& lasPnts,
	const PointNumberList& pnlPolyline, 
	const PointNumberList& pnlSeg)
{
	if(pnlSeg.size()<2 || pnlPolyline.size()<2) return LineSegment2D();
	bool bClosePolyline = pnlPolyline[0]==pnlPolyline[pnlPolyline.size()-1];

	//LineSegment2D lineSegment;
	Positions2D temObjPnts2D;
	//double scalar0, scalar1;

	for (unsigned int i=0; i<pnlSeg.size(); ++i)
		temObjPnts2D.push_back(lasPnts[pnlSeg[i].Number()].Position2DOnly());

	Line2D temLine = Line2D(temObjPnts2D);
	double scalar0 = temLine.Scalar(temObjPnts2D[0]);
	double scalar1 = temLine.Scalar(temObjPnts2D[temObjPnts2D.size()-1]);

	//transform the line to be outside of the contour
	if (bClosePolyline) {
		double maxDist = -99999999.9;
		double minDist = 99999999.9;
		double dist;

		for (unsigned int i=0; i<pnlSeg.size(); ++i) {
			dist = temLine.DistanceToPointSigned(lasPnts[pnlSeg[i].Number()].Position2DOnly());
			if (dist>maxDist) maxDist = dist;
			if (dist<minDist) minDist = dist;
		}

		// Correct the line distance to origin
		temLine.SetDistanceToOrigin(temLine.DistanceToOrigin()+maxDist);
		// Get the mid point of the line segment
		Position2D pos2D = temLine.Position((scalar0+scalar1)/2.0);
		LaserPoint check_point;
		check_point.X() = pos2D.X();
		check_point.Y() = pos2D.Y();

		if (check_point.InsidePolygon(lasPnts, pnlPolyline))
			temLine.SetDistanceToOrigin(temLine.DistanceToOrigin()-maxDist+minDist);
	}

	//construct line segment
	LineSegment2D lineSegment = LineSegment2D(temLine.Position(scalar0), temLine.Position(scalar1));

	return lineSegment;
}

#include "PolygonFunctions.h"

void AdjustLineSegsWithOrientationContraints(
	const LaserPoints& lasPnts, 
	vector<PointNumberList>& vecPnlSegs,
	LineSegments2D& vecLineSegs,
	const OutliningParameters& outlining_par,
	const std::vector<double>& constraintDirections)
{
#ifdef _DEBUG
	ObjectPoints debugObjPts2;
	LineTopologies debugLineTops2;
	LineTopology debugLineTop2;
	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		debugLineTop2.clear();
		AddOnePointToPolygon(debugObjPts2, debugLineTop2, vecLineSegs[i].BeginPoint());
		AddOnePointToPolygon(debugObjPts2, debugLineTop2, vecLineSegs[i].EndPoint());
		debugLineTops2.push_back(debugLineTop2);
	}
	debugLineTops2.Write("debug.top");
	debugObjPts2.Write("debug.objpts");
#endif

	//////////////////////////////////////////////////////////////////////////
	//adjust directions
	LaserPoints::const_iterator itrPnt;
	double temDir, minDirDif, destDir;
	double meanDist = 0.0;
	LineSegment2D temLineSeg;
	for (int i=0; i<vecLineSegs.size(); i++) {
		//find nearest direction
		temDir = vecLineSegs[i].NormalAngle();
		if (temDir<0) temDir += 2*PI;
		if (temDir>PI) temDir -= PI;


		minDirDif = 3.1415926;

		for (int j=0; j<constraintDirections.size(); j++) {
			if (fabs(constraintDirections[j]-temDir) < minDirDif) {
				minDirDif = fabs(constraintDirections[j]-temDir);
				destDir = constraintDirections[j];
			}
		}
	
		temLineSeg = lasPnts.FitLineSegment(vecPnlSegs[i], Vector2D(cos(destDir+PI/2), sin(destDir+PI/2)));
		vecLineSegs[i] = temLineSeg;
	}

#ifdef _DEBUG
	ObjectPoints debugObjPts0;
	LineTopologies debugLineTops0;
	LineTopology debugLineTop;
	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		debugLineTop.clear();
		AddOnePointToPolygon(debugObjPts0, debugLineTop, vecLineSegs[i].BeginPoint());
		AddOnePointToPolygon(debugObjPts0, debugLineTop, vecLineSegs[i].EndPoint());
		debugLineTops0.push_back(debugLineTop);
	}
	debugLineTops0.Write("debug.top");
	debugObjPts0.Write("debug.objpts");
#endif


	//////////////////////////////////////////////////////////////////////////
	//merge nearby line segments
	if (vecLineSegs.size()<=2) return;
	LineSegments2D::iterator lineSeg0, lineSeg1;
	vector<PointNumberList>::iterator pnlSeg0, pnlSeg1;
	double dist0, dist1;
	vector<bool> vecValidSegs(vecLineSegs.size(), true);
	for (int i=0; i<vecLineSegs.size(); i++) {
		if (!vecValidSegs[i]) continue;
		lineSeg0 = vecLineSegs.begin()+i;

		for (int j = i!=vecLineSegs.size()-1?i+1:0;
			j!=i; 
			j!=vecLineSegs.size()-1 ? j++:j=0) 
		{
			if (!vecValidSegs[j]) continue;
			lineSeg1 = vecLineSegs.begin()+j;
			if (lineSeg0->Normal() != lineSeg1->Normal()) break;

			dist0 = lineSeg0->DistanceToOrigin();
			dist1 = lineSeg1->DistanceToOrigin();
			if (fabs(dist0-dist1) > outlining_par.MaximumDistancePointOutline()) break;

			//set distance to original point
			meanDist = (lineSeg0->Length()*dist0 + lineSeg1->Length()*dist1)/(lineSeg0->Length()+lineSeg1->Length());
			lineSeg0->SetDistanceToOrigin(meanDist);
			

			
			pnlSeg0 = vecPnlSegs.begin()+(lineSeg0-vecLineSegs.begin());
			pnlSeg1 = vecPnlSegs.begin()+(lineSeg1-vecLineSegs.begin());
			pnlSeg0->insert(pnlSeg0->end(), pnlSeg1->begin(), pnlSeg1->end());

			vecValidSegs[j] = false;
			i = j;
		}
	}

	for (int i=0; i<vecValidSegs.size(); i++) {
		if (vecValidSegs[i]) continue;
		vecValidSegs.erase(vecValidSegs.begin()+i);
		vecPnlSegs.erase(vecPnlSegs.begin()+i);
		vecLineSegs.erase(vecLineSegs.begin()+i);
		i--;
	}

	double scalar0, scalar1;
	for (int i=0; i<vecValidSegs.size(); i++) {
		lineSeg0 = vecLineSegs.begin()+i;
		pnlSeg0 = vecPnlSegs.begin()+i;
		scalar0 = lineSeg0->Scalar(lasPnts[(*pnlSeg0)[0].Number()].Position2DOnly());
		scalar1 = lineSeg0->Scalar(lasPnts[(*pnlSeg0)[pnlSeg0->size()-1].Number()].Position2DOnly());
		lineSeg0->ScalarBegin() = scalar0;
		lineSeg0->ScalarEnd() = scalar1;
	}



#ifdef _DEBUG
	ObjectPoints debugObjPts1;
	LineTopologies debugLineTops1;
	LineTopology debugLineTop1;
	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		debugLineTop1.clear();
		AddOnePointToPolygon(debugObjPts1, debugLineTop1, vecLineSegs[i].BeginPoint());
		AddOnePointToPolygon(debugObjPts1, debugLineTop1, vecLineSegs[i].EndPoint());
		debugLineTops1.push_back(debugLineTop1);
	}
	debugLineTops1.Write("debug.top");
	debugObjPts1.Write("debug.objpts");
#endif
}

IEDll_API 
	void PolylineGeneralizeByLineGrow(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const OutliningParameters& outlining_par,
	const std::vector<double>& constraintDirections)
{
	outObjPnts.clear();
	outLineTop.clear();
	if (pnlPolyline.size()<2) return;

	bool bClosedPolyline = pnlPolyline[0]==pnlPolyline[pnlPolyline.size()-1];
	//LaserPoints temLasPnts;
	LineTopology temContourTop;
	ObjectPoints temContourPnts;
	//PointNumberList temContourPnl;
	//OutliningParameters outlining_par;
	//outlining_par.MaximumDistancePointIntersectionLine() = 0.5;

#ifdef _DEBUG
	if (lasPnts[pnlPolyline[0].Number()].HasAttribute(SegmentNumberTag) && 
		lasPnts[pnlPolyline[0].Number()].Attribute(SegmentNumberTag)==278)
	{
		int aaa = 0;
	}
#endif

	//DeriveContour_AlphaShape2D(lasPnts, pnlPolyline, temContourPnl);
	LineSegmentGrow Grower = LineSegmentGrow(outlining_par);
	vector<PointNumberList> vecPnlSegs = Grower.DoGrow(lasPnts, pnlPolyline);

	double area, primeter, ratio;
	{


		area = ComputePolygonArea<LaserPoint>(lasPnts, pnlPolyline);
		primeter = ComputePolylineLength<LaserPoint>(lasPnts, pnlPolyline);
		ratio = 4*3.1415926*area/(primeter*primeter);
	}
	//DataBoundsLaser bbox = lasPnts.ObjectBounds(pnlPolyline);
	//double area = bbox.Bounds2D().XRange()*bbox.Bounds2D().YRange();

	//fit rectangle
	if (vecPnlSegs.size()<=2 || area<3.5) {
		//PolylineGeneralizeByFitPCARectangle(lasPnts, pnlPolyline, outObjPnts, outLineTop, constraintDirections);
		PolylineGeneralizeByMinAreaRectangle(lasPnts, pnlPolyline, outObjPnts, outLineTop);
		return;
	}
	//	else if(ratio<0.1) {
	//		PolylineGeneralizeByRectangleStrip(lasPnts, pnlPolyline, outObjPnts, outLineTop);
	//		return;
	//	}

	//////////////////////////////////////////////////////////////////////////
	//construct all line segments;
	LineSegments2D vecLineSegs;
	//LineSegment2D temLineSeg;
	//Positions2D temObjPnts2D;
	for (unsigned int i=0; i<vecPnlSegs.size(); ++i)
		vecLineSegs.push_back(CreateExteriorLineSegment2D(lasPnts, pnlPolyline, vecPnlSegs[i]));

	if (!constraintDirections.empty())
		AdjustLineSegsWithOrientationContraints(lasPnts, vecPnlSegs, vecLineSegs, outlining_par, constraintDirections);

	//////////////////////////////////////////////////////////////////////////
	//create corners
	LineSegments2D::iterator lineSeg0=vecLineSegs.end()-1, lineSeg1;
	Position2D crossPos;
	ObjectPoint temObjPnt; 
	Line2D temLine;
	double scalar0, scalar1;

	for (lineSeg1=vecLineSegs.begin(); lineSeg1!=vecLineSegs.end(); lineSeg0=lineSeg1, lineSeg1++) {
		//parallel
		if(ParallelLines(lineSeg0->Line2DReference(), lineSeg1->Line2DReference(), 15*3.1415926/180)) {
			// Construct bisector
			Line2D bisector = Bisector(lineSeg0->Line2DReference(), lineSeg1->Line2DReference());
			scalar0 = bisector.Scalar(lineSeg0->EndPoint());
			scalar1 = bisector.Scalar(lineSeg1->BeginPoint());
			Position2D midPos = bisector.Position((scalar0+scalar1)/2.0);
			// Construct perpendicular line through the mid point
			temLine = bisector.PerpendicularLine(midPos);

			Intersection2Lines(lineSeg0->Line2DReference(), temLine, crossPos);
			AddOnePointToPolygon(outObjPnts, outLineTop, crossPos);

			Intersection2Lines(lineSeg1->Line2DReference(), temLine, crossPos);
			AddOnePointToPolygon(outObjPnts, outLineTop, crossPos);
		}
		else {
			Intersection2Lines(lineSeg0->Line2DReference(), lineSeg1->Line2DReference(), crossPos);
			double dist0 = crossPos.Distance(lineSeg0->EndPoint());
			double dist1 = crossPos.Distance(lineSeg1->BeginPoint());


			if (dist0>3.0 || dist1>3.0) {
				AddOnePointToPolygon(outObjPnts, outLineTop, lineSeg0->EndPoint());
				AddOnePointToPolygon(outObjPnts, outLineTop, lineSeg1->BeginPoint());
			}
			//use the cross point
			else
				AddOnePointToPolygon(outObjPnts, outLineTop, crossPos);
		}
	}

	if(bClosedPolyline) outLineTop.push_back(outLineTop[0]);


	Plane plane = lasPnts.FitPlane(pnlPolyline);
	Line3D plumb;
	Position3D pos3D;
	for (unsigned int i=0; i<outObjPnts.size(); i++) {
		plumb = Line3D(outObjPnts[i], Vector3D(0.0, 0.0, 1.0));
		if(IntersectLine3DPlane(plumb, plane, pos3D))
			outObjPnts[i].Z() = pos3D.Z();
	}
}

IEDll_API 
	void PolylineGeneralizeByFitPCARectangle(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const std::vector<double>& constraintDirections)
{
	outObjPnts.clear();
	outLineTop.clear();
	if (pnlPolyline.size()<2) return;
	if (pnlPolyline[0] != pnlPolyline[pnlPolyline.size()-1]) return;

	Positions2D localPnts;
	for (PointNumberList::const_iterator itr=pnlPolyline.begin(); itr!=pnlPolyline.end(); itr++)
		localPnts.push_back(lasPnts[itr->Number()].Position2DOnly());
	Position3D cent = lasPnts.CentreOfGravity(pnlPolyline);
	Line2D principle = Line2D(localPnts);
	Line2D perpen = principle.PerpendicularLine(cent.Position2DOnly());

	//////////////////////////////////////////////////////////////////////////
	//get the four corners
	//	double scalar00, scalar01, scalar10, scalar11;
	double maxSca0, minSca0, maxSca1, minSca1, temSca0, temSca1;
	maxSca0 = maxSca1 = -99999999.9;
	minSca0 = minSca1 = 99999999.9;


	for (unsigned int i=0; i<localPnts.size(); i++) {
		temSca0 = principle.Scalar(localPnts[i]);
		temSca1 = perpen.Scalar(localPnts[i]);

		if (temSca0>maxSca0) 
			maxSca0 = temSca0;
		if (temSca0<minSca0) 
			minSca0 = temSca0;

		if (temSca1>maxSca1) 
			maxSca1 = temSca1;
		if (temSca1<minSca1) 
			minSca1 = temSca1;
	}


	Line2D line0 = principle.PerpendicularLine(principle.Position(minSca0));
	Line2D line2 = principle.PerpendicularLine(principle.Position(maxSca0));
	Line2D line1 = perpen.PerpendicularLine(perpen.Position(minSca1));
	Line2D line3 = perpen.PerpendicularLine(perpen.Position(maxSca1));

	Position2D pos0, pos1, pos2, pos3;
	Intersection2Lines(line0, line1, pos0);
	Intersection2Lines(line1, line2, pos1);
	Intersection2Lines(line2, line3, pos2);
	Intersection2Lines(line3, line0, pos3);


	AddOnePointToPolygon(outObjPnts, outLineTop, pos0);
	AddOnePointToPolygon(outObjPnts, outLineTop, pos1);
	AddOnePointToPolygon(outObjPnts, outLineTop, pos2);
	AddOnePointToPolygon(outObjPnts, outLineTop, pos3);
	outLineTop.push_back(outLineTop[0]);


	Plane plane = lasPnts.FitPlane(pnlPolyline);
	Line3D plumb;
	Position3D pos3D;
	for (unsigned int i=0; i<outObjPnts.size(); i++) {
		plumb = Line3D(outObjPnts[i], Vector3D(0.0, 0.0, 1.0));
		if(IntersectLine3DPlane(plumb, plane, pos3D))
			outObjPnts[i].Z() = pos3D.Z();
	}
}

//search for minimum area rectangle
IEDll_API 
	void PolylineGeneralizeByMinAreaRectangle(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop)
{
	//PointNumberList component, convex_corners;
	LineTopology::const_iterator node, node1, node2;//, prev_node, next_node, convex_node
	LaserPoints::const_iterator laser_point, laser_point1, laser_point2;
	//int next_number;
	Line2D line, bb_lines[4], perp_line;
	double dist, dist_min1, dist_max1, dist_min2, dist_max2;
	Position2D corners[4];

	double area, best_area = 1.0e30;
	node1 = pnlPolyline.begin();
	laser_point1 = lasPnts.begin() + node1->Number();
	for (node1++; node1!=pnlPolyline.end(); node1++) {
		// Construct a line for this convex hull edge
		laser_point2 = laser_point1;
		laser_point1 = lasPnts.begin() + node1->Number();
		line = Line2D(laser_point1->Position2DOnly(), laser_point2->Position2DOnly());

		// Determine the sizes of the bounding rectangle
		for (node=pnlPolyline.begin(); node!=pnlPolyline.end()-1; node++) {
			laser_point = lasPnts.begin() + node->Number();
			dist = line.DistanceToPointSigned(laser_point->Position2DOnly());
			if (node == pnlPolyline.begin()) {
				dist_min1 = dist_max1 = dist;
			}
			else {
				if (dist < dist_min1) dist_min1 = dist;
				if (dist > dist_max1) dist_max1 = dist;
			}
		}

		perp_line = line.PerpendicularLine(laser_point->Position2DOnly());
		dist_min2 = dist_max2 = 0.0;
		for (node=pnlPolyline.begin(); node!=pnlPolyline.end()-1; node++) {
			laser_point = lasPnts.begin() + node->Number();
			dist = perp_line.DistanceToPointSigned(laser_point->Position2DOnly());
			if (dist < dist_min2) dist_min2 = dist;
			if (dist > dist_max2) dist_max2 = dist;
		}

		// Derive the bounding lines if this is the best area so far
		area = (dist_max1-dist_min1) * (dist_max2-dist_min2);
		if (area < best_area) {
			best_area = area;
			bb_lines[0] = line; 
			bb_lines[0].SetDistanceToOrigin(line.DistanceToOrigin() + dist_min1);
			bb_lines[1] = line;
			bb_lines[1].SetDistanceToOrigin(line.DistanceToOrigin() + dist_max1);
			bb_lines[2] = perp_line; 
			bb_lines[2].SetDistanceToOrigin(perp_line.DistanceToOrigin() + dist_min2);
			bb_lines[3] = perp_line;
			bb_lines[3].SetDistanceToOrigin(perp_line.DistanceToOrigin() + dist_max2);
		}
	}


	Intersection2Lines(bb_lines[0], bb_lines[2], corners[0]);
	Intersection2Lines(bb_lines[0], bb_lines[3], corners[1]);
	Intersection2Lines(bb_lines[1], bb_lines[3], corners[2]);
	Intersection2Lines(bb_lines[1], bb_lines[2], corners[3]);


	AddOnePointToPolygon(outObjPnts, outLineTop, corners[0]);
	AddOnePointToPolygon(outObjPnts, outLineTop, corners[1]);
	AddOnePointToPolygon(outObjPnts, outLineTop, corners[2]);
	AddOnePointToPolygon(outObjPnts, outLineTop, corners[3]);
	outLineTop.push_back(outLineTop[0]);


	Plane plane = lasPnts.FitPlane(pnlPolyline);
	Line3D plumb;
	Position3D pos3D;
	for (unsigned int i=0; i<outObjPnts.size(); i++) {
		plumb = Line3D(outObjPnts[i], Vector3D(0.0, 0.0, 1.0));
		if(IntersectLine3DPlane(plumb, plane, pos3D))
			outObjPnts[i].Z() = pos3D.Z();
	}
}


IEDll_API 
	void PolylineGeneralizeByRectangleStrip(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop)
{
	LineTopology temContourTop;
	ObjectPoints temContourPnts;
	OutliningParameters para;
	para.MaximumDistancePointIntersectionLine() = 1.0;

	LineSegmentGrow Grower = LineSegmentGrow(para);
	vector<PointNumberList> vecPnlSegs = Grower.DoGrow(lasPnts, pnlPolyline);

	//////////////////////////////////////////////////////////////////////////
	//merge close and parallel line segments
	LineSegments2D lineSegs;
	for (unsigned int i=0; i<vecPnlSegs.size(); i++)
		lineSegs.push_back(lasPnts.FitLineSegment(vecPnlSegs[i]));

	double dist0, dist1, angle;
	for (unsigned int i=0; i<lineSegs.size(); i++)	{
		LineSegment2D& lineSeg0 = lineSegs[i];

		for (unsigned int j=i+1; j<lineSegs.size(); j++) {
			LineSegment2D& lineSeg1 = lineSegs[j];
			dist0 = lineSeg0.Line2DReference().DistanceToPoint(lineSeg1.BeginPoint());
			dist1 = lineSeg0.Line2DReference().DistanceToPoint(lineSeg1.EndPoint());
			angle = Angle2Lines(lineSeg0.Line2DReference(), lineSeg1.Line2DReference());
			angle = 180*angle/3.1415926;

			if (angle<15.0 && dist0<1.0 && dist1<1.0) {
				vecPnlSegs[i].insert(vecPnlSegs[i].end(), vecPnlSegs[j].begin(), vecPnlSegs[j].end());
				lineSeg0 = lasPnts.FitLineSegment(vecPnlSegs[i]);
				vecPnlSegs.erase(vecPnlSegs.begin()+j);
				lineSegs.erase(lineSegs.begin()+j);
				j--;
			}	
		}
	}

	if (lineSegs.size()<=1)
		return PolylineGeneralizeByMinAreaRectangle(lasPnts, pnlPolyline, outObjPnts, outLineTop);

	double minDist = 99999999.9;
	int indLine0, indLine1;
	std::vector<double> vecDists;
	std::vector<double>::iterator itrMinDist;
	for (unsigned int i=0; i<lineSegs.size(); i++)	{
		LineSegment2D& lineSeg0 = lineSegs[i];

		for (unsigned int j=i+1; j<lineSegs.size(); j++) {
			LineSegment2D& lineSeg1 = lineSegs[j];
			vecDists.clear();
			vecDists.push_back( lineSeg0.BeginPoint().Distance(lineSeg1.BeginPoint()) );
			vecDists.push_back( lineSeg0.BeginPoint().Distance(lineSeg1.EndPoint()) );
			vecDists.push_back( lineSeg0.EndPoint().Distance(lineSeg1.BeginPoint()) );
			vecDists.push_back( lineSeg0.EndPoint().Distance(lineSeg1.EndPoint()) );
			itrMinDist = std::min_element(vecDists.begin(), vecDists.end());
			if (*itrMinDist<minDist) {
				minDist = *itrMinDist;
				indLine0 = i;
				indLine1 = j;
			}
		}
	}

	//sort the two start line pair
	//std::vector<bool> vecBUsedLine = std::vector<bool>(lineSegs.size(), false);
	std::vector<int> vecSortedLineInd;
	Position2D temPnt;
	std::vector<Position2D> vecPLinePnts;

	LineSegment2D lineSeg0 = lineSegs[indLine0];
	LineSegment2D lineSeg1 = lineSegs[indLine1];
	//vecBUsedLine[indLine0] = true;
	//vecBUsedLine[indLine1] = true;

	if(!Intersection2Lines(lineSeg0.Line2DReference(), lineSeg1.Line2DReference(), temPnt)) return;
	{
		dist0 = lineSeg0.BeginPoint().Distance(temPnt);
		dist1 = lineSeg0.EndPoint().Distance(temPnt);
		if (dist0 > dist1)
			vecPLinePnts.push_back(lineSeg0.BeginPoint());
		else
			vecPLinePnts.push_back(lineSeg0.EndPoint());

		vecPLinePnts.push_back(temPnt);

		dist0 = lineSeg1.BeginPoint().Distance(temPnt);
		dist1 = lineSeg1.EndPoint().Distance(temPnt);
		if (dist0 > dist1)
			vecPLinePnts.push_back(lineSeg1.BeginPoint());
		else
			vecPLinePnts.push_back(lineSeg1.EndPoint());

		if (vecPLinePnts[0].X()>vecPLinePnts[2].X()) {
			std::swap(vecPLinePnts[0], vecPLinePnts[2]);
			vecSortedLineInd.push_back(indLine1);
			vecSortedLineInd.push_back(indLine0);
		}
		else {
			vecSortedLineInd.push_back(indLine0);
			vecSortedLineInd.push_back(indLine1);
		}
	}


	Position2D frontPnt, backPnt;
	double dist2, dist3;
	std::vector<int> vecInsertPos;
	std::map<double, int> vecMapDistToLine;
	int nLoops = 0;

	do {
		//vecDists.clear();
		//vecPnts.clear();
		vecMapDistToLine.clear();
		backPnt = vecPLinePnts[0];
		frontPnt = vecPLinePnts[vecPLinePnts.size()-1];


		for (unsigned int i=0; i<lineSegs.size(); i++) {
			bool bUsed = vecSortedLineInd.end()!=std::find(vecSortedLineInd.begin(), vecSortedLineInd.end(), i);
			if (bUsed) continue;
			vecDists.clear();
			vecDists.push_back(frontPnt.Distance(lineSegs[i].BeginPoint()));
			vecDists.push_back(frontPnt.Distance(lineSegs[i].EndPoint()));
			vecDists.push_back(backPnt.Distance(lineSegs[i].BeginPoint()));
			vecDists.push_back(backPnt.Distance(lineSegs[i].EndPoint()));
			itrMinDist = std::min_element(vecDists.begin(), vecDists.end());
			//vecDists.push_back(*itrMinDist);
			vecMapDistToLine.insert(std::make_pair(*itrMinDist, i));
		}

		//find nearest end point
		if (!vecMapDistToLine.empty()) {
			indLine0 = vecMapDistToLine.begin()->second;
			Line2D temLine;

			vecDists.clear();
			vecDists.push_back(backPnt.Distance(lineSegs[indLine0].BeginPoint()));
			vecDists.push_back(backPnt.Distance(lineSegs[indLine0].EndPoint()));
			vecDists.push_back(frontPnt.Distance(lineSegs[indLine0].BeginPoint()));
			vecDists.push_back(frontPnt.Distance(lineSegs[indLine0].EndPoint()));

			itrMinDist = std::min_element(vecDists.begin(), vecDists.end());

			if (itrMinDist-vecDists.begin()>1) {//front, 2, 3
				temLine = Line2D(vecPLinePnts[vecPLinePnts.size()-2], vecPLinePnts[vecPLinePnts.size()-1]);
				Intersection2Lines(lineSegs[indLine0], temLine, temPnt);
				vecPLinePnts[vecPLinePnts.size()-1] = temPnt;
				vecSortedLineInd.push_back(indLine0);

				if (itrMinDist-vecDists.begin()==2)
					vecPLinePnts.push_back(lineSegs[indLine0].EndPoint());
				else
					vecPLinePnts.push_back(lineSegs[indLine0].BeginPoint());
			}
			else {//back, 0, 1
				temLine = Line2D(vecPLinePnts[0], vecPLinePnts[1]);
				Intersection2Lines(lineSegs[indLine0], temLine, temPnt);
				vecPLinePnts[0] = temPnt;
				vecSortedLineInd.insert(vecSortedLineInd.begin(), indLine0);

				if (itrMinDist-vecDists.begin()==0)
					vecPLinePnts.insert(vecPLinePnts.begin(), lineSegs[indLine0].EndPoint());
				else
					vecPLinePnts.insert(vecPLinePnts.begin(), lineSegs[indLine0].BeginPoint());
			}
		}		
	} while (!vecMapDistToLine.empty() && nLoops++<500);


	double maxDist = -9999.9, temDist;
	vecDists.clear();
	for (unsigned int i=0; i<vecSortedLineInd.size(); i++) {
		PointNumberList& pnlCurSeg = vecPnlSegs[vecSortedLineInd[i]];
		lineSeg0 = LineSegment2D(vecPLinePnts[i], vecPLinePnts[i+1]);

		for (unsigned int j=0; j<pnlCurSeg.size(); j++) {
			LaserPoints::const_iterator itrPnt=lasPnts.begin()+pnlCurSeg[j].Number();
			temDist = lineSeg0.Line2DReference().DistanceToPoint(itrPnt->Position2DOnly());
			vecDists.push_back(temDist);
			if (temDist>maxDist) 
				maxDist = temDist;
		}
	}
	std::sort(vecDists.begin(), vecDists.end());
	maxDist = vecDists[int(vecDists.size()*0.95)];

	//////////////////////////////////////////////////////////////////////////
	//construct the rectangle
	Line2D line00, line01, line10, line11, linePerp;
	LineTopology upLineTop, botLineTop;
	for (unsigned int i=0; i<vecPLinePnts.size()-2; i++) {
		lineSeg0 = LineSegment2D(vecPLinePnts[i], vecPLinePnts[i+1]);
		line01 = line00 = lineSeg0.Line2DReference();
		line00.SetDistanceToOrigin(lineSeg0.DistanceToOrigin()+maxDist);
		line01.SetDistanceToOrigin(lineSeg0.DistanceToOrigin()-maxDist);

		lineSeg1 = LineSegment2D(vecPLinePnts[i+1], vecPLinePnts[i+2]);
		line10 = line11 = lineSeg1.Line2DReference();
		line10.SetDistanceToOrigin(lineSeg1.DistanceToOrigin()+maxDist);
		line11.SetDistanceToOrigin(lineSeg1.DistanceToOrigin()-maxDist);

		if (i==0) {
			linePerp = lineSeg0.PerpendicularLine(vecPLinePnts[0]);
			Intersection2Lines(linePerp, line00, temPnt);
			AddOnePointToPolygon(outObjPnts, upLineTop, temPnt);
			Intersection2Lines(linePerp, line01, temPnt);
			AddOnePointToPolygon(outObjPnts, botLineTop, temPnt);
		}

		Intersection2Lines(line00, line10, temPnt);
		AddOnePointToPolygon(outObjPnts, upLineTop, temPnt);
		Intersection2Lines(line01, line11, temPnt);
		AddOnePointToPolygon(outObjPnts, botLineTop, temPnt);

		if (i==vecPLinePnts.size()-3) {
			linePerp = lineSeg1.PerpendicularLine(vecPLinePnts[vecPLinePnts.size()-1]);
			Intersection2Lines(linePerp, line10, temPnt);
			AddOnePointToPolygon(outObjPnts, upLineTop, temPnt);
			Intersection2Lines(linePerp, line11, temPnt);
			AddOnePointToPolygon(outObjPnts, botLineTop, temPnt);
		}
	}

	outLineTop = upLineTop;
	//std::reverse(botLineTop.begin(), botLineTop.end());
	outLineTop.insert(outLineTop.end(), botLineTop.rbegin(), botLineTop.rend());
	outLineTop.push_back(outLineTop[0]);

	//////////////////////////////////////////////////////////////////////////
	//project the points to the plane of the points
	Plane plane = lasPnts.FitPlane(pnlPolyline);
	Line3D plumb;
	Position3D pos3D;
	for (unsigned int i=0; i<outObjPnts.size(); i++) {
		plumb = Line3D(outObjPnts[i], Vector3D(0.0, 0.0, 1.0));
		if(IntersectLine3DPlane(plumb, plane, pos3D))
			outObjPnts[i].Z() = pos3D.Z();
	}
}


//check the histogram of local fitting normal
IEDll_API 
	std::vector<double> DeriveDominateDirections(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	int knn)
{
	std::vector<double> directions;
	directions.reserve(pnlPolyline.size());
	LineSegment2D temLine;
	PointNumberList temPnl;
	double temAngle;

	int ind;
	for (unsigned int i=0; i<pnlPolyline.size()-1; ++i) {
		temPnl.clear();
		for (int j=-knn/2; j<knn/2; j++) {
			ind = i+j;
			if (ind<0) ind += pnlPolyline.size()-1;
			else if (ind>pnlPolyline.size()-1) ind -= pnlPolyline.size()-1;

			temPnl.push_back(pnlPolyline[ind]);
		}

		temLine = lasPnts.FitLineSegment(temPnl);
		temAngle = temLine.NormalAngle();
		if (temAngle<0) temAngle += PI;
		if (temAngle>=PI/2) temAngle -= PI/2;
		directions.push_back(temAngle);
	}

	std::vector<int> vecFreqs=Histgram(directions, 0, PI/2, PI/180);

#ifdef _DEBUG
	FILE* file = fopen("Histogram.txt", "w+");
	for (unsigned int i=0; i<vecFreqs.size(); i++) {
		fprintf(file, "%.3f\n", vecFreqs[i]*1.0/pnlPolyline.size());
	}
	fclose(file);
#endif

	return std::vector<double>();
}

//#include "GMM/em_gmm.h"
double GaussianProbability(double mean, double stv, double x) 
{
	return exp(-(x-mean)*(x-mean)/(2*stv*stv))/(sqrt(2*PI)*stv);
}


bool AddOnePointToPolygon(ObjectPoints& outObjPnts, LineTopology& outLineTop, const Position2D& inPnt)
{
	ObjectPoint temObjPnt;

	temObjPnt.X() = inPnt.X();
	temObjPnt.Y() = inPnt.Y();
	temObjPnt.Number() = outObjPnts.size();
	outObjPnts.push_back(temObjPnt);
	outLineTop.push_back(temObjPnt.Number());

	return true;
}

//constrained directions: [0-pi), 
//all constrained directions should be prepaired before
HoughSpace* CreateHoughSpace2D(const LaserPoints& inLasPnts, 
							   const OutliningParameters& para,
							   const std::vector<double>& vecConstDirs)
{
	//OutliningParameters outlining_pars;
	HoughSpace* houghSpace = new HoughSpace();
	if (!houghSpace) return NULL;
	double degree = atan(1.0)/45.0;

	DataBoundsLaser bBox;
	for (unsigned int i=0; i<inLasPnts.size(); i++)
		bBox.Update(inLasPnts[i], false);

	double maxDistHough = sqrt(bBox.XRange()*bBox.XRange()+bBox.YRange()*bBox.YRange())/2.0;
	double* hough_dirs;

	//direction & distance
	if (!vecConstDirs.empty()) {
		hough_dirs = (double *) malloc(vecConstDirs.size()*sizeof(double));
		if (!hough_dirs) {
			delete houghSpace;
			return NULL;
		}

		memcpy(hough_dirs, &(vecConstDirs[0]), vecConstDirs.size()*sizeof(double));

		houghSpace->Initialise(vecConstDirs.size(), hough_dirs,
			int(2.0*maxDistHough/para.HoughBinSizeDistance()),
			-maxDistHough, maxDistHough);

		free(hough_dirs);
	}
	else {
		houghSpace->Initialise(int(180*degree/para.HoughBinSizeDirection()),
			int(2*maxDistHough/para.HoughBinSizeDistance()),
			0.0, -maxDistHough,	180*degree, maxDistHough);
	}

	//offset
	houghSpace->SetOffsets(bBox.MidPoint().X(), bBox.MidPoint().Y(), 0.0);

	// Put the points into the Hough space
	for (LaserPoints::const_iterator node=inLasPnts.begin(); node!=inLasPnts.end(); node++)
		houghSpace->AddPoint(node->Position2DOnly());

	return houghSpace;
}

void DestroyHoughSpace2D(HoughSpace* houghSpace)
{
	if(houghSpace) 
		delete houghSpace;
}

IEDll_API 
	void TestPolyGeneByHoughTransf(
	const LaserPoints& inLasPnts, 
	ObjectPoints& outPnts, 
	LineTopologies& outTops)
{
	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(inLasPnts, vecPnlSegs);
	outPnts.clear();
	outTops.clear();

	//LaserPoints temLasPnts;
	LineTopology temContourTop;
	ObjectPoints temContourPnts;
	PointNumberList temContourPnl;

	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		//temLasPnts.clear();
		//for (unsigned int j=0; j<vecPnlSegs[i].size(); j++)
		//	temLasPnts.push_back(inLasPnts[vecPnlSegs[i][j].Number()]);
		DeriveContour_AlphaShape2D(inLasPnts, vecPnlSegs[i], temContourPnl);
		PolylineGeneralizeByHoughTransform(inLasPnts, temContourPnl, temContourPnts, temContourTop, OutliningParameters());

		LineTopologies temLineTops;
		temLineTops.push_back(temContourTop);
		temLineTops.ReNumber(temContourPnts, outPnts.size(), outTops.size());
		outPnts.insert(outPnts.end(), temContourPnts.begin(), temContourPnts.end());
		outTops.insert(outTops.end(), temLineTops.begin(), temLineTops.end());
	}

	return;
}

void TestPolyGeneByHoughTransf0()
{
	ObjectPoints outLinePnts;
	LineTopologies outLineTops;
	LaserPoints inLasPnts;
	inLasPnts.Read("1_1.laser");

	TestPolyGeneByHoughTransf(inLasPnts, outLinePnts, outLineTops);

	outLinePnts.Write("1_1_contour.objpts");
	outLineTops.Write("1_1_contour.top");
}

void TestPolyGeneByHoughTransf1()
{
	ObjectPoints outLinePnts;
	LineTopologies outLineTops;
	LaserPoints inLasPnts;
	inLasPnts.Read("1_4.laser");

	TestPolyGeneByHoughTransf(inLasPnts, outLinePnts, outLineTops);

	outLinePnts.Write("1_4_contour.objpts");
	outLineTops.Write("1_4_contour.top");
}


IEDll_API void TestPolyGeneByLineGrow(const LaserPoints& inLasPnts, ObjectPoints& outPnts, LineTopologies& outTops)
{
	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(inLasPnts, vecPnlSegs);
	outPnts.clear();
	outTops.clear();

	//LaserPoints temLasPnts;
	LineTopology temContourTop;
	ObjectPoints temContourPnts;
	PointNumberList temContourPnl;

	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		DeriveContour_AlphaShape2D(inLasPnts, vecPnlSegs[i], temContourPnl);
		PolylineGeneralizeByLineGrow(inLasPnts, temContourPnl, temContourPnts, temContourTop);

		LineTopologies temLineTops;
		temLineTops.push_back(temContourTop);
		temLineTops.ReNumber(temContourPnts, outPnts.size(), outTops.size());
		outPnts.insert(outPnts.end(), temContourPnts.begin(), temContourPnts.end());
		outTops.insert(outTops.end(), temLineTops.begin(), temLineTops.end());
	}

	return;
}

void TestPolyGeneByLineGrow0()
{
	ObjectPoints outLinePnts;
	LineTopologies outLineTops;
	LaserPoints inLasPnts;
	inLasPnts.Read("2.laser");

	TestPolyGeneByLineGrow(inLasPnts, outLinePnts, outLineTops);

	outLinePnts.Write("2_contour.objpts");
	outLineTops.Write("2_contour.top");
}

LineSegmentGrow::LineSegmentGrow()
	:m_bClosed(true), m_lasPnts(NULL), m_pnlCtr(NULL)
{

}

LineSegmentGrow::LineSegmentGrow(const OutliningParameters& para)
	:m_bClosed(true), m_lasPnts(NULL), m_pnlCtr(NULL), m_para(para)
{

}

LineSegmentGrow::~LineSegmentGrow()
{
	//CleanData();
}

void LineSegmentGrow::CleanData()
{
	if (m_lasPnts) {
		delete m_lasPnts;
		m_lasPnts = NULL;
	}

	if (m_pnlCtr) {
		delete m_pnlCtr;
		m_pnlCtr = NULL;
	}

	std::map<int, Line2D*>::iterator itrMap;
	for (itrMap=m_mapLines.begin(); itrMap!=m_mapLines.end(); itrMap++) {
		if (itrMap->second) 
			delete itrMap->second;
	}
}

void LineSegmentGrow::CopyToLocalData(const LaserPoints& lasPnts, const PointNumberList& pnlCtr)
{
	CleanData();

	m_lasPnts = new LaserPoints();
	m_pnlCtr = new PointNumberList();
	m_lasPnts->reserve(pnlCtr.size());
	m_pnlCtr->reserve(pnlCtr.size());

	for (unsigned int i=0; i<pnlCtr.size(); i++) {
		m_lasPnts->push_back(lasPnts[pnlCtr[i].Number()]);
		m_pnlCtr->push_back(PointNumber(i));
	}

	m_lasPnts->SetAttribute(SegmentNumberTag, -1);

	m_bClosed = pnlCtr[0] == pnlCtr[pnlCtr.size()-1];
	if (m_bClosed) m_pnlCtr->erase(m_pnlCtr->end()-1);
}

bool LessPointNumberList(const PointNumberList& lhv, const PointNumberList& rhv)
{
	if (lhv.empty() && !rhv.empty()) return true;
	else if (!lhv.empty() && rhv.empty()) return false;
	else if (lhv.empty() && rhv.empty()) return false;
	else return lhv[lhv.size()-1]<rhv[rhv.size()-1];
}

std::vector<PointNumberList> LineSegmentGrow::CopyToOutData(const LaserPoints& lasPnts, const PointNumberList& pnlCtr)
{
	vector<PointNumberList> vecSegs;
	MyDeriveSegPNL(*m_lasPnts, vecSegs, SegmentNumberTag);

	//clean small segments
	for (unsigned int i=0; i<vecSegs.size(); i++) {
		if (vecSegs[i].size()<2) {
			vecSegs.erase(vecSegs.begin()+i);
			i--;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//setp 0: sort the point number list
	//sort point number list 
	for (unsigned int i=0; i<vecSegs.size(); i++)
		std::sort(vecSegs[i].begin(), vecSegs[i].end());

	//sort line segment
	std::sort(vecSegs.begin(), vecSegs.end(), LessPointNumberList);

	//take care of line segment that cover the end point
	PointNumberList curSeg;
	PointNumberList::iterator itrGap;
	//	bool bCoverGap;
	for (unsigned int i=0; i<vecSegs.size(); i++) {
		curSeg = vecSegs[i];
		if (curSeg.size()<2) continue;

		itrGap = curSeg.end();
		for (unsigned int j=0; j<curSeg.size()-1; j++) {
			if(1 + curSeg[j].Number() != curSeg[j+1].Number()) {
				itrGap = curSeg.begin()+j+1;
				break;
			}
		}

		if (itrGap != curSeg.end()) {
			vecSegs[i].clear();
			vecSegs[i].insert(vecSegs[i].end(), itrGap, curSeg.end());
			vecSegs[i].insert(vecSegs[i].end(), curSeg.begin(), itrGap);
		} 
	}


	//////////////////////////////////////////////////////////////////////////
	//step 1
	//return to original point number
	int orgNum;
	for (unsigned int i=0; i<vecSegs.size(); i++) {
		for (unsigned int j=0; j<vecSegs[i].size(); j++) {
			orgNum = pnlCtr[vecSegs[i][j].Number()].Number();
			vecSegs[i][j].Number() = orgNum;
		}
	}
	return vecSegs;
}

PointNumberList LineSegmentGrow::GetNeibsWithItself(const PointNumber& curPnt, int nNeibs)
{
	if (nNeibs<0) nNeibs = 0;
	PointNumberList pnlNeibs;
	PointNumberList::iterator itrCur;
	itrCur = std::find(m_pnlCtr->begin(), m_pnlCtr->end(), curPnt);
	if (itrCur == m_pnlCtr->end()) return pnlNeibs;//cannot find this point

	//odd 
	if (nNeibs%2==1) nNeibs+=1;
	int aaa = -12%5;

	int ind;
	int nTotalPnts = m_pnlCtr->size();
	for (int i=-nNeibs/2; i<=nNeibs/2; i++) {
		ind = i+(itrCur-m_pnlCtr->begin());

		if (ind>=0 && ind<nTotalPnts)
			pnlNeibs.push_back((*m_pnlCtr)[ind]);
		else if (m_bClosed)	{
			//d > i+nd >= 0
			//1-i/d > n >=-i/d
			//int nIncrement = int(1-ind/nTotalPnts);
			//ind += nIncrement*nTotalPnts;
			//ind %= nTotalPnts;
			int temInd = ind%nTotalPnts;
			if (temInd<0) temInd += nTotalPnts;
			pnlNeibs.push_back((*m_pnlCtr)[temInd]);
		}
	}

	return pnlNeibs;
}

PointNumberList LineSegmentGrow::GetUnusedPoints(const PointNumberList& inPnts)
{
	PointNumberList outPnl;

	for (unsigned int i=0; i<inPnts.size(); i++) {
		if (!IsUsedPoint(inPnts[i]))
			outPnl.push_back(inPnts[i]);
	}

	return outPnl;
}



PointNumberList LineSegmentGrow::CreateSeedPoints()
{
	PointNumberList pnlNeibs;
	double temScore, maxScore;
	PointNumber curPntNum, dstPntNum;

	//collect scores of a point belonging to a line
	//only use the unused point
	maxScore = -999.9;
	for (unsigned int i=0; i<m_pnlCtr->size(); i++) {
		curPntNum = (*m_pnlCtr)[i];
		if (IsUsedPoint(curPntNum)) continue;

		temScore = GetScoreAsLine(curPntNum);
		if (temScore>maxScore) {
			maxScore = temScore;
			dstPntNum = curPntNum;
		}
	}

	if (maxScore==-999.9) return PointNumberList();
	else return GetNeibsWithItself(dstPntNum);
}

#include "Line2D.h"
Line2D LineSegmentGrow::CreatLine2D(const LaserPoints& lasPnts, const PointNumberList& pnl, double& maxDistPnt2Line)
{
	Position2D objPnt;
	Positions2D temObjPnts;
	LaserPoints::const_iterator pnt;

	for (unsigned int i=0; i<pnl.size(); i++) {
		pnt = lasPnts.begin()+pnl[i].Number();
		objPnt.X() = pnt->X();
		objPnt.Y() = pnt->Y();
		temObjPnts.push_back(objPnt);
	}

	Line2D line = Line2D(temObjPnts);

	double maxDist = -99999.9, temDist;
	for (unsigned int i=0; i<pnl.size(); i++) {
		pnt = lasPnts.begin()+pnl[i].Number();
		temDist = line.DistanceToPoint(pnt->Position2DOnly());
		if (temDist>maxDist) maxDist = temDist;
	}
	maxDistPnt2Line = maxDist;

	return line;
}


Line2D LineSegmentGrow::CreatLine2D(const LaserPoints& lasPnts, const PointNumberList& pnl)
{
	double maxDistPnt2Line = 0;
	return CreatLine2D(lasPnts, pnl, maxDistPnt2Line);
}

#include <queue>
PointNumberList LineSegmentGrow::ExpandLineSegment(const PointNumberList& seedPnl, int lineNum)
{
	bool bRefintLine = false;
	PointNumberList neibs, unUsedNeibs, totalSegment;
	Line2D temLine = CreatLine2D(*m_lasPnts, seedPnl);
	std::queue<PointNumber> queSeeds;
	Line2D* line = new Line2D(temLine);
	m_mapLines[lineNum] = line;

	for (unsigned int i=0; i<seedPnl.size(); i++) {
		queSeeds.push(seedPnl[i]);
	}

	totalSegment = seedPnl;
	TagLineNumber(seedPnl, lineNum);
	WriteDebugLaserData();

	PointNumber curPnt, temPnt;
	LaserPoints::iterator lasPnt;
	double dist;
	int nLoops = 0;
	*line = CreatLine2D(*m_lasPnts, seedPnl, dist);

	while (!queSeeds.empty() && nLoops++<500) {
		curPnt = queSeeds.front();
		queSeeds.pop();
		neibs = GetNeibsWithItself(curPnt, 2);
		//unUsedNeibs = GetUnusedPoints(neibs);
		//re-fit line, and check the distance from old point to the line
		if (bRefintLine) {
			*line = CreatLine2D(*m_lasPnts, totalSegment, dist);
			//June 6, 2015
			if (dist>=m_para.MaximumDistancePointOutline()/4) continue;
		}
		//if (dist>=m_para.MaximumDistancePointOutline()) continue;

		for (unsigned int i=0; i<neibs.size(); i++) {
			temPnt = neibs[i];
			int oldLineNum = GetLineNumber(temPnt);
			//belonging to current line
			if (oldLineNum==lineNum) continue;

			lasPnt = m_lasPnts->begin()+temPnt.Number();
			dist = line->DistanceToPoint(lasPnt->Position2DOnly());
			if (dist>=m_para.MaximumDistancePointOutline()) continue;

			//do competition to old line
			bool bCanAdd = true;
			if (IsUsedPoint(temPnt)) {
				Line2D* oldLine = m_mapLines[oldLineNum];
				double dist2OldLine = 999999.9;
				if (oldLine) 
					dist2OldLine = oldLine->DistanceToPoint(lasPnt->Position2DOnly());

				//competition failed
				if (dist2OldLine<dist) bCanAdd = false;
			}

			if (bCanAdd) {
				TagLineNumber(temPnt, lineNum);
				queSeeds.push(temPnt);
				totalSegment.push_back(temPnt);
			}
		}
	}

	return totalSegment;
}

void LineSegmentGrow::WriteDebugLaserData()
{
#ifdef _DEBUG
	for (unsigned int i=0; i<m_lasPnts->size(); i++) {
		if ((*m_lasPnts)[i].Attribute(SegmentNumberTag)==-1)
			(*m_lasPnts)[i].RemoveAttribute(SegmentNumberTag);
	}
	m_lasPnts->Write("debug.laser");
#endif
}

void LineSegmentGrow::TagLineNumber(const PointNumber& curPnt, int lineNum)
{
	assert(curPnt.Number()>=0 && curPnt.Number()<m_lasPnts->size());
	(*m_lasPnts)[curPnt.Number()].SetAttribute(SegmentNumberTag, lineNum);
}

void LineSegmentGrow::TagLineNumber(const PointNumberList& inPnts, int lineNum)
{
	PointNumberList::const_iterator curPnt;
	for (curPnt=inPnts.begin(); curPnt!=inPnts.end(); curPnt++) {
		TagLineNumber(*curPnt, lineNum);
	}
}

int LineSegmentGrow::GetLineNumber(const PointNumber& curPnt)
{
	assert(curPnt.Number()>=0 && curPnt.Number()<m_lasPnts->size());
	LaserPoints::iterator lasPnt = m_lasPnts->begin()+curPnt.Number();
	if (!lasPnt->HasAttribute(SegmentNumberTag)) 
		return -1;
	else
		return lasPnt->Attribute(SegmentNumberTag);	
}

bool LineSegmentGrow::IsUsedPoint(const PointNumber& curPnt)
{
	return -1 != GetLineNumber(curPnt);
}

std::vector<PointNumberList> LineSegmentGrow::DoGrow(const LaserPoints& lasPnts, const PointNumberList& pnlCtr)
{
	if (pnlCtr.size()<4) return std::vector<PointNumberList>();

	CleanData();
	CopyToLocalData(lasPnts, pnlCtr);
	ScoreInitialisingLine(4);

	PointNumberList pnlSeed = CreateSeedPoints();
	WriteDebugLaserData();

	int lineNum = 0;
	int nLoops = 0;
	while (!pnlSeed.empty() && nLoops++<5000) {
		ExpandLineSegment(pnlSeed, lineNum++);
		pnlSeed = CreateSeedPoints();
	}

	//WriteDebugLaserData();
	CleanSmallSegments();

	return CopyToOutData(lasPnts, pnlCtr);
}

void LineSegmentGrow::ScoreInitialisingLine(int nNeibs)
{
	LaserPoints::iterator curPnt, temPnt;
	PointNumberList pnlNeibs;
	Line2D line;
	double meanDist, score;
	vector<double> vecScores;
	double deviation = m_para.MaximumDistancePointIntersectionLine()/3;
	deviation *= deviation;

	for (unsigned int i=0; i<m_pnlCtr->size(); ++i) {
		curPnt = m_lasPnts->begin()+(*m_pnlCtr)[i].Number();
		pnlNeibs = GetNeibsWithItself((*m_pnlCtr)[i], nNeibs);
		line = CreatLine2D(*m_lasPnts, pnlNeibs);

		meanDist = 0.0;
		for (unsigned int j=0; j<pnlNeibs.size(); ++j) {
			temPnt = m_lasPnts->begin()+pnlNeibs[j].Number();
			meanDist += line.DistanceToPoint(temPnt->Position2DOnly())/pnlNeibs.size();
		}

		score = exp(-meanDist*meanDist/deviation);
		SetScoreAsLine((*m_pnlCtr)[i], score);
	}
}

void LineSegmentGrow::SetScoreAsLine(const PointNumber& curPnt, double score)
{
	assert(curPnt.Number()>=0 && curPnt<m_lasPnts->size());
	LaserPoints::iterator curLasPnt = m_lasPnts->begin()+curPnt.Number();
	//curLasPnt->DoubleAttribute(v_xTag) = score;
	curLasPnt->SetDoubleAttribute(v_xTag, score);
}

double LineSegmentGrow::GetScoreAsLine(const PointNumber& curPnt)
{
	assert(curPnt.Number()>=0 && curPnt<m_lasPnts->size());
	LaserPoints::iterator curLasPnt = m_lasPnts->begin()+curPnt.Number();

	if (!curLasPnt->HasAttribute(v_xTag)) return 0.0;
	else return curLasPnt->DoubleAttribute(v_xTag);
}

void  LineSegmentGrow::CleanSmallSegments()
{

}

IEDll_API void TestLineSegmentGrow(LaserPoints& inLasPnts)
{
	vector<PointNumberList> vecPnlSegs;
	vector<PointNumberList> curLineSegs;
	MyDeriveSegPNL(inLasPnts, vecPnlSegs);

	//LaserPoints temLasPnts;
	LineTopology temContourTop;
	ObjectPoints temContourPnts;
	PointNumberList temContourPnl;
	OutliningParameters para;
	para.MaximumDistancePointIntersectionLine() = 0.5;

	int compNum = 0;
	for (unsigned int i=0; i<vecPnlSegs.size(); i++) {
		DeriveContour_AlphaShape2D(inLasPnts, vecPnlSegs[i], temContourPnl);
		LineSegmentGrow Grower = LineSegmentGrow(para);
		curLineSegs = Grower.DoGrow(inLasPnts, temContourPnl);

		for (int j=0; j<curLineSegs.size(); j++) {
			for (unsigned int k=0; k<curLineSegs[j].size(); k++)
				inLasPnts[curLineSegs[j][k].Number()].SetAttribute(ComponentNumberTag, compNum+j);
		}

		compNum += curLineSegs.size();
	}

	return;
}

IEDll_API void TestLineSegmentGrow0()
{
	LaserPoints lasPnts;
	lasPnts.Read("1_1.laser");
	TestLineSegmentGrow(lasPnts);
	lasPnts.Write("1_1_lineGrow.laser");
}

IEDll_API void TestLineSegmentGrow1()
{
	LaserPoints lasPnts;
	lasPnts.Read("1_2.laser");
	TestLineSegmentGrow(lasPnts);
	lasPnts.Write("1_2_lineGrow.laser");
}

IEDll_API void TestLineSegmentGrow2()
{
	LaserPoints lasPnts;
	lasPnts.Read("1.laser");
	TestLineSegmentGrow(lasPnts);
	lasPnts.Write("1_lineGrow.laser");
}

#include "PolygonFunctions.h"
IEDll_API void TestBoundedSidePolygon()
{
	ObjectPoints polObjPnts;
	LineTopology polTop;
	AddOnePointToPolygon(polObjPnts,polTop, Position2D(-1.0, 0.0));
	AddOnePointToPolygon(polObjPnts,polTop, Position2D(0.0, 1.0));
	AddOnePointToPolygon(polObjPnts,polTop, Position2D(1.0, 0.0));
	AddOnePointToPolygon(polObjPnts,polTop, Position2D(0.0, -1.0));
	polTop.push_back(polTop[0]);

	ObjectPoint pnt;

	pnt.X() = 0.0; pnt.Y() = 0.0;
	int bounded0 = BoundedSidePolygon(polObjPnts, polTop, pnt);

	pnt.X() = 1.0; pnt.Y() = 0.0;
	int bounded1 = BoundedSidePolygon(polObjPnts, polTop, pnt);

	pnt.X() = 0.0; pnt.Y() = 1.0;
	int bounded2 = BoundedSidePolygon(polObjPnts, polTop, pnt);

	pnt.X() = 0.5; pnt.Y() = 0.5;
	int bounded3 = BoundedSidePolygon(polObjPnts, polTop, pnt);

	pnt.X() = 0.2; pnt.Y() = 0.2;
	int bounded4 = BoundedSidePolygon(polObjPnts, polTop, pnt);

	pnt.X() = 0.6; pnt.Y() = 0.6;
	int bounded5 = BoundedSidePolygon(polObjPnts, polTop, pnt);
}
