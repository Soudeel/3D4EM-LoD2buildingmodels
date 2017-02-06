#include "ConstraintLSF.h"
#include <vector>
#include "Vector3D.h"
#include <Eigen/Dense>
#include "InferenceEngine.h"

using namespace Eigen;
using namespace std;

void TestCLSF_Vertical()
{
	char* strLasPnts = "f:\\test_data\\rdam5.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Plane orgP0, orgP1, desP0, desP1;
	Vector3D orgDir, desDir;

	orgP0 = lasPnts.FitPlane(pnlSegs[0]);
	orgDir = orgP0.Normal();
//	orgP1 = lasPnts.FitPlane(pnlSegs[1]);
//	orgDir = orgP1.Normal().VectorProduct(orgP0.Normal());
//	orgDir.Normalize();

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(1);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0], orgP0);
//	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1], orgP1);
	//cslfSovler.AddHoriLineCons(0, 1);
	cslfSovler.AddConstraint(Cons_Horizontal, 0);
	cslfSovler.Solve();
	cslfSovler.GetPlanes(vecPlanes);
	desP0 = vecPlanes[0];
	desDir = desP0.Normal();
	double len = desDir.Length();
//	desP1 = vecPlanes[1];
//	desDir = desP0.Normal().VectorProduct(desP1.Normal());
//	desDir.Normalize();

	//printf("orignial intersection: %.4f")
}

void TestCLSF_perpend()
{
	char* strLasPnts = "f:\\test_data\\pers\\perpendicular.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Plane orgP0, orgP1, desP0, desP1;
	Vector3D orgDir, desDir;

	orgP0 = lasPnts.FitPlane(pnlSegs[0]);
	orgP1 = lasPnts.FitPlane(pnlSegs[1]);
	orgDir = orgP1.Normal().VectorProduct(orgP0.Normal());
	double orgProd = orgP1.Normal().DotProduct(orgP0.Normal());
	//orgDir.Normalize();

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(2);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0], orgP0);
	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1], orgP1);
	cslfSovler.AddConstraint(Cons_Orthogonal, 0, 1);
	//cslfSovler.AddPerpCons(0, 1);
	cslfSovler.Solve();
	cslfSovler.GetPlanes(vecPlanes);
	desP0 = vecPlanes[0];
	desP1 = vecPlanes[1];
	desDir = desP0.Normal().VectorProduct(desP1.Normal());
	double dstProd = desP0.Normal().DotProduct(desP1.Normal());
	double n0 = desP0.Normal().Length();
	double n1 = desP1.Normal().Length();
	//desDir.Normalize();

	//printf("orignial intersection: %.4f")
}

void TestCLSF_EqualSlop()
{
	//char* strLasPnts = "f:\\test_data\\pers\\perpendicular.laser";
	char* strLasPnts = "f:\\test_data\\pers\\equal_slop2.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	ConstraintLSF cslfSovler(2);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0]);
	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1]);

	EquSlopCons* pHorCon0 = static_cast<EquSlopCons*>
		(cslfSovler.AddConstraint(Cons_Equal_Slop, 0, 1));
	double oldSlopDif = pHorCon0->GetConstraintValue();

	cslfSovler.Solve();

	double newSlopDif = pHorCon0->GetConstraintValue();
}

void TestCLSF_concurrent()
{
	char* strLasPnts = "f:\\test_data\\pers\\concurrent3.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Plane orgP0, orgP1, orgP2, orgP3;
	Plane desP0, desP1, desP2, desP3;
	Vector3D orgDir, desDir;

	//do least square fit 
	vector<Plane> vecPlanes, vecPlanes0;
	ConstraintLSF cslfSovler(6);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0]);
	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1]);
	cslfSovler.AddPlane(2, lasPnts, pnlSegs[2]);
	cslfSovler.AddPlane(3, lasPnts, pnlSegs[3]);
	cslfSovler.AddPlane(4, lasPnts, pnlSegs[4]);
	cslfSovler.AddPlane(5, lasPnts, pnlSegs[5]);

	cslfSovler.GetPlanes(vecPlanes0);
	//cslfSovler.AddPerpCons(0, 1);
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(2);
	vecCurrentPlanes.push_back(3);
	vecCurrentPlanes.push_back(4);
	vecCurrentPlanes.push_back(5);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	Constraint* pCons = cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);
	ConcurrentCons* pConCons = static_cast<ConcurrentCons*>(pCons);
	double oldDist = pConCons->CroosPntDist2AllPlanes();
	cslfSovler.Solve();
	double newDist = pConCons->CroosPntDist2AllPlanes();

	//printf("orignial intersection: %.4f")
}

void TestCLSF_concurrent2()
{
	char* strLasPnts = "f:\\test_data\\pers\\concurrent.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Plane orgP0, orgP1, orgP2, orgP3;
	Plane desP0, desP1, desP2, desP3;
	Vector3D orgDir, desDir;

	//do least square fit 
	vector<Plane> vecPlanes, vecPlanes0;
	ConstraintLSF cslfSovler(4);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0]);
	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1]);
	cslfSovler.AddPlane(2, lasPnts, pnlSegs[2]);
	cslfSovler.AddPlane(3, lasPnts, pnlSegs[3]);

	cslfSovler.GetPlanes(vecPlanes0);
	//cslfSovler.AddPerpCons(0, 1);
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(2);
	vecCurrentPlanes.push_back(3);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	Constraint* pCons = cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);
	ConcurrentCons* pConCons = static_cast<ConcurrentCons*>(pCons);
	double oldDist = pConCons->CroosPntDist2AllPlanes();
	cslfSovler.Solve();
	double newDist = pConCons->CroosPntDist2AllPlanes();

	int ii = 0;
}


void TestCLSF_Hori_Intersect()
{
	//char* strLasPnts = "E:\\test_data\\pers\\horizontal_intersection.laser";
	char* strLasPnts = "f:\\test_data\\rdam4.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	lasPnts.AddNoise(1.0);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Vector3D norm1, norm2;

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(pnlSegs.size());
	for (int i=0; i<pnlSegs.size(); ++i) {
		cslfSovler.AddPlane(i, lasPnts, pnlSegs[i]);
	}
	cslfSovler.GetPlanes(vecPlanes);

	norm1 = vecPlanes[0].Normal().VectorProduct(vecPlanes[1].Normal());
	
	//cslfSovler.AddHoriLineCons(0, 1);
	//cslfSovler.AddEqualSlopCons(0,1);
//	cslfSovler.AddConstraint(Cons_Equal_Slop, 0, 1);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 0, 1);
	cslfSovler.Solve();
	cslfSovler.GetPlanes(vecPlanes);

	norm2 = vecPlanes[0].Normal().VectorProduct(vecPlanes[1].Normal());
	double a = vecPlanes[0].Normal().Length();
	double b = vecPlanes[1].Normal().Length();
	double c = norm2.Length();

	//printf("orignial intersection: %.4f")
}

void TestCLSF_Mixture()
{
	char* strLasPnts = "F:\\test_data\\pers\\mixture.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Plane orgP0, orgP1, orgP2, orgP3;
	Plane desP0, desP1, desP2, desP3;
	Vector3D orgDir, desDir;

	int planeNum1, planeNum2, planeNum3, planeNum4;
	planeNum1 = lasPnts[pnlSegs[0][0].Number()].Attribute(SegmentNumberTag);
	planeNum2 = lasPnts[pnlSegs[1][0].Number()].Attribute(SegmentNumberTag);
	planeNum3 = lasPnts[pnlSegs[2][0].Number()].Attribute(SegmentNumberTag);
	planeNum4 = lasPnts[pnlSegs[3][0].Number()].Attribute(SegmentNumberTag);

	orgP0 = lasPnts.FitPlane(pnlSegs[0]);
	orgP1 = lasPnts.FitPlane(pnlSegs[1]);
	orgP2 = lasPnts.FitPlane(pnlSegs[2]);
	orgP3 = lasPnts.FitPlane(pnlSegs[3]);

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(4);
	cslfSovler.AddPlane(0, lasPnts, pnlSegs[0], orgP0);
	cslfSovler.AddPlane(1, lasPnts, pnlSegs[1], orgP1);
	cslfSovler.AddPlane(2, lasPnts, pnlSegs[2], orgP2);
	cslfSovler.AddPlane(3, lasPnts, pnlSegs[3], orgP3);
	//cslfSovler.AddPerpCons(0, 1);
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(2);
	vecCurrentPlanes.push_back(3);

//	cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);
	cslfSovler.AddConstraint(Cons_Orthogonal, planeNum1, planeNum2);
	cslfSovler.AddConstraint(Cons_Orthogonal, planeNum3, planeNum4);
	cslfSovler.AddConstraint(Cons_Hor_Inter, planeNum1, planeNum2);
	cslfSovler.AddConstraint(Cons_Hor_Inter, planeNum3, planeNum4);
	cslfSovler.Solve();
	cslfSovler.GetPlanes(vecPlanes);

	int nPlane = vecPlanes.size();
	MatrixXd W = MatrixXd::Zero(4, nPlane);
	int curPlaneNum;
	for (int i=0; i<nPlane; ++i) {
		W(0, i) = vecPlanes[i].Normal().X();
		W(1, i) = vecPlanes[i].Normal().Y();
		W(2, i) = vecPlanes[i].Normal().Z();
		W(3, i) = -vecPlanes[i].Distance();
	}
	MatrixXd WW = W*W.transpose();
	double det = WW.determinant();

	//printf("orignial intersection: %.4f")
}

void TestCLSF_Mixture2()
{
	char* strLasPnts = "F:\\test_data\\pers\\rdam2.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(pnlSegs.size());
	for (int i=0; i<pnlSegs.size(); ++i) {
		cslfSovler.AddPlane(i, lasPnts, pnlSegs[i]);
	}

	//cslfSovler.AddHoriLineCons(0, 1);
	//cslfSovler.AddHoriLineCons(2, 3);
	//cslfSovler.AddHoriLineCons(3, 5);
	//cslfSovler.AddHoriLineCons(3, 4);
	//cslfSovler.AddHoriLineCons(0, 3);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 0, 1);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 2, 3);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 3, 5);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 3, 4);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 0, 3);

//	cslfSovler.AddPerpCons(5, 3);
//	cslfSovler.AddPerpCons(0, 3);

	//cslfSovler.AddPerpCons(0, 1);
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(2);
	vecCurrentPlanes.push_back(3);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	Constraint* pCon = cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);
	ConcurrentCons* pConCons = static_cast<ConcurrentCons*>(pCon);
	double oldDist = pConCons->CroosPntDist2AllPlanes();
	cslfSovler.Solve();
	double newDist = pConCons->CroosPntDist2AllPlanes();
	cslfSovler.GetPlanes(vecPlanes);

	//printf("orignial intersection: %.4f")
}

void TestCLSF_Mixture3()
{
	char* strLasPnts = "E:\\test_data\\rdam3.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	MyDeriveSegPNL(lasPnts,pnlSegs);

	//do least square fit 
	vector<Plane> vecPlanes;
	ConstraintLSF cslfSovler(pnlSegs.size());
	for (int i=0; i<pnlSegs.size(); ++i) {
		cslfSovler.AddPlane(i, lasPnts, pnlSegs[i]);
	}

	//cslfSovler.AddHoriLineCons(0, 1);
	//cslfSovler.AddHoriLineCons(1, 2);
	//cslfSovler.AddHoriLineCons(1, 3);
	//cslfSovler.AddHoriLineCons(1, 8);
	//cslfSovler.AddHoriLineCons(6, 3);
	//cslfSovler.AddHoriLineCons(5, 0);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 0, 1);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 1, 2);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 1, 3);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 1, 8);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 6, 3);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 5, 0);


	//	cslfSovler.AddPerpCons(5, 3);
	//	cslfSovler.AddPerpCons(0, 3);

	//cslfSovler.AddPerpCons(0, 1);
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(5);
	vecCurrentPlanes.push_back(8);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);

	vecCurrentPlanes.clear();
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(8);
	vecCurrentPlanes.push_back(3);
	vecCurrentPlanes.push_back(6);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);

	cslfSovler.Solve();
	cslfSovler.GetPlanes(vecPlanes);

	//printf("orignial intersection: %.4f")
}

void TestCLSF_Mixture4()
{
	char* strLasPnts = "f:\\test_data\\U-shape.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
//	lasPnts.AddNoise(1.5);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Vector3D norm1, norm2, norm3, norm4, norm5, norm6;

	//do least square fit 
	vector<Plane> vecPlanes,vecPlanes2;
	ConstraintLSF cslfSovler(pnlSegs.size());
	for (int i=0; i<pnlSegs.size(); ++i) {
		cslfSovler.AddPlane(i, lasPnts, pnlSegs[i]);
	}
	cslfSovler.GetPlanes(vecPlanes);
	vecPlanes2 = vecPlanes;

	norm1 = vecPlanes[0].Normal().VectorProduct(vecPlanes[3].Normal());
	norm2 = vecPlanes[1].Normal().VectorProduct(vecPlanes[4].Normal());
	norm3 = vecPlanes[5].Normal().VectorProduct(vecPlanes[6].Normal());
	double ang1 = vecPlanes[0].Normal().DotProduct(vecPlanes[3].Normal());
	double ang2 = vecPlanes[1].Normal().DotProduct(vecPlanes[4].Normal());
	double ang3 = vecPlanes[5].Normal().DotProduct(vecPlanes[6].Normal());

	
//	cslfSovler.AddHoriLineCons(0, 3);
//	cslfSovler.AddHoriLineCons(1, 4);
//	cslfSovler.AddHoriLineCons(5, 6);
//	cslfSovler.AddPerpCons(0, 3);
//	cslfSovler.AddPerpCons(1, 4);
//	cslfSovler.AddPerpCons(5, 6);

	cslfSovler.AddConstraint(Cons_Hor_Inter, 0, 3);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 1, 4);
	cslfSovler.AddConstraint(Cons_Hor_Inter, 5, 6);
//	cslfSovler.AddConstraint(Cons_Equal_Slop, 0, 3);
//	cslfSovler.AddConstraint(Cons_Equal_Slop, 1, 4);
//	cslfSovler.AddConstraint(Cons_Equal_Slop, 5, 6);
	
	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(0);
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(4);
	vecCurrentPlanes.push_back(3);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	Constraint* pcon1 = cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);

	vecCurrentPlanes.clear();
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(4);
	vecCurrentPlanes.push_back(5);
	vecCurrentPlanes.push_back(6);
	//cslfSovler.AddConcurrentCons(vecCurrentPlanes);
	Constraint* pcon2 = cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes);

	ConcurrentCons* pCurCon1 = static_cast<ConcurrentCons*>(pcon1);
	ConcurrentCons* pCurCon2 = static_cast<ConcurrentCons*>(pcon2);
	double oldDist1 = pCurCon1->CroosPntDist2AllPlanes();
	double oldDist2 = pCurCon2->CroosPntDist2AllPlanes();
	cslfSovler.Solve();

	double newDist1 = pCurCon1->CroosPntDist2AllPlanes();
	double newDist2 = pCurCon2->CroosPntDist2AllPlanes();
	cslfSovler.GetPlanes(vecPlanes);

	norm4 = vecPlanes[0].Normal().VectorProduct(vecPlanes[3].Normal());
	norm5 = vecPlanes[1].Normal().VectorProduct(vecPlanes[4].Normal());
	norm6 = vecPlanes[5].Normal().VectorProduct(vecPlanes[6].Normal());

	double ang4 = vecPlanes[0].Normal().DotProduct(vecPlanes[3].Normal());
	double ang5 = vecPlanes[1].Normal().DotProduct(vecPlanes[4].Normal());
	double ang6 = vecPlanes[5].Normal().DotProduct(vecPlanes[6].Normal());

	double a = vecPlanes[0].Normal().Length();
	double b = vecPlanes[1].Normal().Length();
	double c = vecPlanes[2].Normal().Length();
	double d = vecPlanes[3].Normal().Length();
	double e = vecPlanes[4].Normal().Length();
	double f = vecPlanes[5].Normal().Length();
	double g = vecPlanes[6].Normal().Length();

	for (int i=0; i<vecPlanes.size(); i++) {
		if (vecPlanes[i].Normal().Z() < 0.0 )
			vecPlanes[i].Normal() *= -1.0;
		
		if (vecPlanes2[i].Normal().Z() < 0.0 )
			vecPlanes2[i].Normal() *= -1.0;
	}
	
	Vector3D plumb(0, 0, 1.0);
	double angle11 = fabs(MyAngle(vecPlanes[0].Normal(), plumb)-MyAngle(vecPlanes[3].Normal(), plumb))*180/3.14159;
	double angle12 = fabs(MyAngle(vecPlanes[1].Normal(), plumb)-MyAngle(vecPlanes[4].Normal(), plumb))*180/3.14159;
	double angle13 = fabs(MyAngle(vecPlanes[5].Normal(), plumb)-MyAngle(vecPlanes[6].Normal(), plumb))*180/3.14159;

	double angle21 = fabs(MyAngle(vecPlanes2[0].Normal(), plumb)-MyAngle(vecPlanes2[3].Normal(), plumb))*180/3.14159;
	double angle22 = fabs(MyAngle(vecPlanes2[1].Normal(), plumb)-MyAngle(vecPlanes2[4].Normal(), plumb))*180/3.14159;
	double angle23 = fabs(MyAngle(vecPlanes2[5].Normal(), plumb)-MyAngle(vecPlanes2[6].Normal(), plumb))*180/3.14159;
	//printf("orignial intersection: %.4f")
}

void TestCLSF_Mixture5()
{
	char* strLasPnts = "f:\\test_data\\pers\\mixture5.laser";
	LaserPoints lasPnts;
	vector<PointNumberList> pnlSegs;
	lasPnts.Read(strLasPnts);
	lasPnts.AddNoise(3);
	MyDeriveSegPNL(lasPnts,pnlSegs);
	Vector3D norm1, norm2, norm3, norm4, norm5, norm6;

	//do least square fit 
	vector<Plane> vecPlanes,vecPlanes2;
	ConstraintLSF cslfSovler(pnlSegs.size());
	for (int i=0; i<pnlSegs.size(); ++i) {
		cslfSovler.AddPlane(i, lasPnts, pnlSegs[i]);
	}
	cslfSovler.GetPlanes(vecPlanes);
	vecPlanes2 = vecPlanes;

	//////////////////////////////////////////////////////////////////////////
	//add constraint
//	HorInterCons* pHorCon0 = static_cast<HorInterCons*>(cslfSovler.AddConstraint(Cons_Hor_Inter, 1, 2));
//	HorInterCons* pHorCon1 = static_cast<HorInterCons*>(cslfSovler.AddConstraint(Cons_Hor_Inter, 3, 4));
	EquSlopCons* pEqualCon0 = (EquSlopCons*)(cslfSovler.AddConstraint(Cons_Equal_Slop, 1, 2));
	EquSlopCons* pEqualCon1 = (EquSlopCons*)(cslfSovler.AddConstraint(Cons_Equal_Slop, 3, 4));
	HoriCons* pHorCon0 = (HoriCons*)cslfSovler.AddConstraint(Cons_Horizontal, 6);
	HoriCons* pHorCon1 = (HoriCons*)cslfSovler.AddConstraint(Cons_Horizontal, 7);
	HoriCons* pHorCon2 = (HoriCons*)cslfSovler.AddConstraint(Cons_Horizontal, 8);


	vector<int> vecCurrentPlanes;
	vecCurrentPlanes.push_back(1);
	vecCurrentPlanes.push_back(2);
	vecCurrentPlanes.push_back(4);
	vecCurrentPlanes.push_back(3);
//	ConcurrentCons* pCurCon1 = static_cast<ConcurrentCons*>(cslfSovler.AddConstraint(Cons_Concurrent, vecCurrentPlanes));

	//////////////////////////////////////////////////////////////////////////
	//get constraint before optimization
//	double oldAng0 = pHorCon0->GetConstraintValue();
//	double oldAng1 = pHorCon1->GetConstraintValue();
	double oldSlopDif0 = pEqualCon0->GetConstraintValue();
	double oldSlopDif1 = pEqualCon1->GetConstraintValue();
//	double oldDist1 = pCurCon1->CroosPntDist2AllPlanes();
	double oldSlop0 = pHorCon0->GetConstraintValue();
	double oldSlop1 = pHorCon1->GetConstraintValue();
	double oldSlop2 = pHorCon2->GetConstraintValue();

	cslfSovler.Solve();

	//////////////////////////////////////////////////////////////////////////
	//get constraint after optimization
//	double newAng0 = pHorCon0->GetConstraintValue();
//	double newAng1 = pHorCon1->GetConstraintValue();
	double newSlopDif0 = pEqualCon0->GetConstraintValue();
	double newSlopDif1 = pEqualCon1->GetConstraintValue();
//	double newDist1 = pCurCon1->CroosPntDist2AllPlanes();
	double newSlop0 = pHorCon0->GetConstraintValue();
	double newSlop1 = pHorCon1->GetConstraintValue();
	double newSlop2 = pHorCon2->GetConstraintValue();
}

#include <Eigen/Geometry>
void TestRotate()
{
	LaserPoints srcPnts, dstPnts;
	srcPnts.Read("E:\\test_data\\ripperda4.laser");

	PointNumberList segPnl;
	for (int i=0; i<srcPnts.size(); ++i) 
		segPnl.push_back(PointNumber(i));

	Plane refPlane =srcPnts.FitPlane(segPnl);
	Vector3D plumbDir = Vector3D(0.0, 0.0, 1.0);
	Vector3D refDir = refPlane.Normal();
	refDir = refDir.Normalize();
	//Vector3D rotAxis = Vector3D(0.0, 0.0, 1.0);
	Vector3D rotAxis = refDir.VectorProduct(plumbDir);
	rotAxis = rotAxis.Normalize();
	double angle = acos(refDir.DotProduct(plumbDir));
	Rotation3D rotMatrix = Rotation3D(rotAxis.X(), rotAxis.Y(), rotAxis.Z(), angle);
	Vector3D dstDir2 = rotMatrix.Rotate(refDir);

	dstPnts = srcPnts;
	for (int i=0; i<srcPnts.size(); ++i)
		dstPnts[i].Position3DRef() = rotMatrix.Rotate(srcPnts[i].Position3DRef());

	Plane dstPlane = dstPnts.FitPlane(segPnl);
	dstPnts.Write("E:\\test_data\\zzzdstpnts.laser");

	Rotation3D bacRotMatrix = Rotation3D(rotAxis.X(), rotAxis.Y(), rotAxis.Z(), -angle);
	LaserPoints bacRotPnts = dstPnts;
	for (int i=0; i<srcPnts.size(); ++i)
		bacRotPnts[i].Position3DRef() = bacRotMatrix.Rotate(dstPnts[i].Position3DRef());
	Plane bacRotPlane = bacRotPnts.FitPlane(segPnl);
	bacRotPnts.Write("E:\\test_data\\zzzbacRotpnts.laser");
}