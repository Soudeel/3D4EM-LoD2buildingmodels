#include "ConstraintLSF.h"
#include <vector>
#include "Vector3D.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


Eigen::MatrixXd CircleProduct(Eigen::MatrixXd lMatrix, Eigen::MatrixXd rMatrix)
{
	int lrow = lMatrix.rows();
	int lcol =  lMatrix.cols();
	int rrow = rMatrix.rows();
	int rcol =  rMatrix.cols();
	MatrixXd dest = MatrixXd::Zero(lrow*rrow, lcol*rcol);

	double curElement;
	for (int i=0; i<lrow; ++i) {
		for (int j=0; j<lcol; ++j) {
			curElement = lMatrix(i, j);

			for (int m=0; m<rrow; ++m) {
				for (int n=0; n<rcol; ++n) {
					dest(i*rrow+m, j*rcol+n) = curElement*rMatrix(m, n);
				}
			}
		}//end j
	}//end i

	return dest;
}

Eigen::MatrixXd CommutationMatrix(int m, int n)
{
	assert(m>0 && n>0);
	int len = m*n;
	MatrixXd K = MatrixXd::Zero(len, len);
	int col=0;
	for (int arr=0; arr<len; ++arr) {
		col %= len;
		K(arr, col) = 1.0;
		col += m;
	}

	return K;
}


QuadricCons::QuadricCons(int nPlanes, Eigen::VectorXd* pParas)
	:Constraint(nPlanes, pParas)
{
	m_A = Eigen::MatrixXd::Zero(4*m_nPlanes, 4*m_nPlanes);
	m_B = Eigen::VectorXd::Zero(4*m_nPlanes);
	m_C = 0.0;
}

double QuadricCons::GetEngergy() 
{
	return AMPLIFICATION*(m_paras->transpose()*m_A*(*m_paras)+m_B.dot(*m_paras)+m_C);
}

Eigen::VectorXd QuadricCons::Get1stDer() 
{
	return AMPLIFICATION*((m_A+m_A.transpose())*(*m_paras)+m_B);
}

Eigen::MatrixXd QuadricCons::Get2stDer() 
{
	return AMPLIFICATION*(m_A+m_A.transpose());
}

//for concurrent constraint
void QuadricCons::AddParameters(int nAddParaNum)
{
	Eigen::MatrixXd oldA = m_A;
	m_A = Eigen::MatrixXd::Zero(oldA.rows()+nAddParaNum, oldA.cols()+nAddParaNum);
	for (int i=0; i<oldA.rows(); i++) {
		for (int j=0; j<oldA.cols(); j++) {
			m_A(i,j) = oldA(i, j);
		}
	}

	Eigen::VectorXd oldB = m_B;
	m_B = Eigen::VectorXd::Zero(oldA.rows()+nAddParaNum);
	for (int i=0; i<oldB.rows(); i++) {
		m_B(i) = oldB(i);
	}
}

SecQuadricCons::SecQuadricCons(int nPlanes, Eigen::VectorXd* pParas)
	:Constraint(nPlanes, pParas)
{
	m_A = Eigen::MatrixXd::Zero(4*m_nPlanes, 4*m_nPlanes);
	m_B = Eigen::VectorXd::Zero(4*m_nPlanes);
	m_C = 0.0;
}

void SecQuadricCons::InsideUpdata()
{
	/*
#ifdef _DEBUG
	printf("New Parameters:\n");
	for (int i=0; i<m_paras->size(); i++)
		printf("%.3f ", (*m_paras)(i));
	printf("\n\n");

	printf("Matrix m_A:\n");
	for (int i=0; i<m_A.rows(); i++) {
		for (int j=0; j<m_A.cols(); j++) {
			printf("%.3f ", m_A(i, j));
		}
		printf("\n\n");
	}

	printf("Matrix m_B:\n");
	for (int i=0; i<m_B.rows(); i++) {
		printf("%.3f ", m_B(i));
	}
#endif
	*/

	m_e = m_paras->transpose()*m_A*(*m_paras)+m_B.dot(*m_paras)+m_C;
	m_AA = m_A + m_A.transpose();
}

//(p'*A*P+B*p+C).^2
double SecQuadricCons::GetEngergy()
{
#ifdef _DEBUG
	InsideUpdata();
#endif
	
	return AMPLIFICATION*m_e*m_e;
}

//2(p'*A*P+B*p+C)((A+A')p+B')
Eigen::VectorXd SecQuadricCons::Get1stDer()
{
	return 2*AMPLIFICATION*m_e*(m_AA*(*m_paras)+m_B);
}

//2*(A+A')pp'(A+A')+2(A+A')pB'+2Bp'(A+A')+2BB'+2(pAp'+B'p+C)(A+A')
Eigen::MatrixXd SecQuadricCons::Get2stDer()
{
//	MatrixXd PPT = (*m_paras)*m_paras->transpose();//pp'
//	MatrixXd PBT = (*m_paras)*m_B.transpose();//pB'

//	return 2*m_AA*PPT*m_AA + 2*m_AA*PBT + 2*PBT.transpose()*m_AA
//		+2*m_B*m_B.transpose() + 2*m_e*m_AA;
//	return 2*m_AA*PPT*m_AA + 2*m_e*m_AA;

//	MatrixXd K = CommutationMatrix(1, m_paras->size());
	MatrixXd AAPB = m_AA*(*m_paras) + m_B;
	return AMPLIFICATION*(2*AAPB*AAPB.transpose() + 2*m_e*m_AA);
}

//for concurrent constraint
void SecQuadricCons::AddParameters(int nAddParaNum)
{
	Eigen::MatrixXd oldA = m_A;
	m_A = Eigen::MatrixXd::Zero(oldA.rows()+nAddParaNum, oldA.cols()+nAddParaNum);
	for (int i=0; i<oldA.rows(); i++) {
		for (int j=0; j<oldA.cols(); j++) {
			m_A(i,j) = oldA(i, j);
		}
	}
	m_AA = m_A + m_A.transpose();

	Eigen::VectorXd oldB = m_B;
	m_B = Eigen::VectorXd::Zero(oldA.rows()+nAddParaNum);
	for (int i=0; i<oldB.rows(); i++) {
		m_B(i) = oldB(i);
	}
}

void HoriCons::AddPlanes(std::vector<int> vecInds) 
{
	assert(vecInds.size()==1);
	//if (vecInds.size()!=1) return;
	QuadricCons::AddPlanes(vecInds);
	int planInd = vecInds[0];
	assert(planInd>=0 && planInd<m_nPlanes);

	m_A(4*planInd+2, 4*planInd+2) = 1.0;
	m_B(4*planInd+2) = -2.0;
	m_C = 1.0;
}

double HoriCons::GetConstraintValue()
{
	Vector3D temN;
	temN.X() = (*m_paras)(4*m_vecIds[0]+0);
	temN.Y() = (*m_paras)(4*m_vecIds[0]+1);
	temN.Z() = (*m_paras)(4*m_vecIds[0]+2);

	double angle = fabs(acos(fabs(temN.Z()))*180.0/PI);
	return angle;
}

void VertCons::AddPlanes(std::vector<int> vecInds) 
{
	assert(vecInds.size()==1);
	//if (vecInds.size()!=1) return;
	QuadricCons::AddPlanes(vecInds);
	int planInd = vecInds[0];
	assert(planInd>=0 && planInd<m_nPlanes);
	m_A(4*planInd+2, 4*planInd+2) = 1.0;
}

void EquSlopCons::AddPlanes(std::vector<int> vecInds) 
{
	assert(vecInds.size()==2);
	//if (vecInds.size()!=1) return;
	SecQuadricCons::AddPlanes(vecInds);
	int planInd1 = vecInds[0];
	int planInd2 = vecInds[1];
	if (planInd2<planInd1) std::swap(planInd1, planInd2);
	assert(planInd1>=0 && planInd2<m_nPlanes);
	//m_A(4*planInd1+2, 4*planInd1+2) = 1.0;
	//m_A(4*planInd1+2, 4*planInd2+2) = -1.0;
	//m_A(4*planInd2+2, 4*planInd1+2) = -1.0;
	//m_A(4*planInd2+2, 4*planInd2+2) = 1.0;
	m_A(4*planInd1+2, 4*planInd1+2) = 1.0;
	m_A(4*planInd2+2, 4*planInd2+2) = -1.0;

	/*
#ifdef _DEBUG
	printf("New Parameters:\n");
	for (int i=0; i<m_paras->size(); i++)
		printf("%.3f ", (*m_paras)(i));
	printf("\n\n");

	printf("Matrix m_A:\n");
	for (int i=0; i<m_A.rows(); i++) {
		for (int j=0; j<m_A.cols(); j++) {
			printf("%.3f ", m_A(i, j));
		}
		printf("\n\n");
	}

	printf("Matrix m_B:\n");
	for (int i=0; i<m_B.rows(); i++) {
		printf("%.3f ", m_B(i));
	}
#endif*/
	
}

double EquSlopCons::GetConstraintValue()
{
	Vector3D temN;
	vector<Vector3D> vecNormals;
	for (int i=0; i<m_vecIds.size(); i++) {
		temN.X() = (*m_paras)(4*m_vecIds[i]+0);
		temN.Y() = (*m_paras)(4*m_vecIds[i]+1);
		temN.Z() = (*m_paras)(4*m_vecIds[i]+2);
		vecNormals.push_back(temN);
	}

	double angle0 = fabs(asin(vecNormals[0].Z())*180.0/PI);
	double angle1 = fabs(asin(vecNormals[1].Z())*180.0/PI);
	return fabs(angle1-angle0);
}

void HorInterCons::AddPlanes(std::vector<int> vecInds) 
{
	assert(vecInds.size()==2);
	//if (vecInds.size()!=1) return;
	SecQuadricCons::AddPlanes(vecInds);
	int planInd1 = vecInds[0];
	int planInd2 = vecInds[1];
	if (planInd2<planInd1) std::swap(planInd1, planInd2);
	assert(planInd1>=0 && planInd2<m_nPlanes);
	m_A(4*planInd1+0, 4*planInd2+1) = 1.0;
	m_A(4*planInd1+1, 4*planInd2+0) = -1.0;
}

double HorInterCons::GetConstraintValue()
{
	Vector3D temN;
	vector<Vector3D> vecNormals;
	for (int i=0; i<m_vecIds.size(); i++) {
		temN.X() = (*m_paras)(4*m_vecIds[i]+0);
		temN.Y() = (*m_paras)(4*m_vecIds[i]+1);
		temN.Z() = (*m_paras)(4*m_vecIds[i]+2);
		vecNormals.push_back(temN);
	}

	Vector3D intDir = vecNormals[0].VectorProduct(vecNormals[1]);
	double angle = asin(intDir.Z())*180.0/PI;
	return fabs(angle);
}

//||a.^2+b.^2+c.^2-1||=0
void UnitCons::AddPlanes(std::vector<int> vecInds)
{
	assert(vecInds.size()==1);
	//if (vecInds.size()!=1) return;
	SecQuadricCons::AddPlanes(vecInds);
	int planInd = vecInds[0];

	m_A(4*planInd+0, 4*planInd+0) = 1.0;
	m_A(4*planInd+1, 4*planInd+1) = 1.0;
	m_A(4*planInd+2, 4*planInd+2) = 1.0;

	m_C = -1.0;
}

void OrthoCons::AddPlanes(std::vector<int> vecInds)
{
	assert(vecInds.size()==2);
	//if (vecInds.size()!=1) return;
	SecQuadricCons::AddPlanes(vecInds);
	int planInd1 = vecInds[0];
	int planInd2 = vecInds[1];
	if (planInd2<planInd1) std::swap(planInd1, planInd2);
	assert(planInd1>=0 && planInd2<m_nPlanes);
	m_A(4*planInd1+0, 4*planInd2+0) = 1.0;
	m_A(4*planInd1+1, 4*planInd2+1) = 1.0;
	m_A(4*planInd1+2, 4*planInd2+2) = 1.0;
}


void InsideConcurrentCons::AddPlanes(std::vector<int> vecInds)
{
	assert(vecInds.size()==1);
	m_vecIds = vecInds;
	int pos = vecInds[0];

	m_A = Eigen::MatrixXd::Zero(m_paras->size(), m_paras->size());
	m_B = Eigen::VectorXd::Zero(m_paras->size());
	m_C = 0.0;

	//m_A
	m_A(4*pos+0, m_indCrossPnt+0) = 1.0;
	m_A(4*pos+1, m_indCrossPnt+1) = 1.0;
	m_A(4*pos+2, m_indCrossPnt+2) = 1.0;

	//m_B
	m_B(4*pos+3) = 1.0;

}


Vector3D ConcurrentCons::CrossPntofPlanes()
{
	Plane temPlane;
	vector<Plane> vecPlanes;
	for (int i=0; i<m_vecIds.size(); i++) {
		temPlane.Normal().X() = (*m_paras)(4*m_vecIds[i]+0);
		temPlane.Normal().Y() = (*m_paras)(4*m_vecIds[i]+1);
		temPlane.Normal().Z() = (*m_paras)(4*m_vecIds[i]+2);
		temPlane.Distance()   = -(*m_paras)(4*m_vecIds[i]+3);
		vecPlanes.push_back(temPlane);
	}

	vector<Line3D> vecLines;
	vector<Plane>::iterator itrP0, itrP1;
	Line3D line;
	for (int i=0; i<vecPlanes.size(); i++)	{
		itrP0 = vecPlanes.begin()+i;
		if (i!=vecPlanes.size()-1)
			itrP1 = vecPlanes.begin()+i+1;
		else
			itrP1 = vecPlanes.begin();
		Intersect2Planes(*itrP0, *itrP1, line);
		vecLines.push_back(line);
	}

	//get intersection point
	double meanDist = 0.0;
	Vector3D temObjPnt;
	vector<Line3D>::iterator itrLine1, itrLine2;
	vector<Vector3D> vecIntPnts;
	for (int iLine=0; iLine<vecLines.size(); ++iLine) {
		itrLine1 = vecLines.begin()+iLine;
		if (iLine==vecLines.size()-1)
			itrLine2 = vecLines.begin();
		else
			itrLine2 = vecLines.begin()+iLine+1;

		if(MyIntersect2Lines(*itrLine1, *itrLine2, temObjPnt))
			vecIntPnts.push_back(temObjPnt);
	}

	meanDist = 0.0;
	Vector3D aveObjPnt(0.0, 0.0, 0.0);	
	for (int i=0; i<vecIntPnts.size(); i++)
		aveObjPnt += vecIntPnts[i]/vecIntPnts.size();

	return aveObjPnt;
}

double ConcurrentCons::CroosPntDist2AllPlanes()
{
	Plane temPlane;
	vector<Plane> vecPlanes;
	for (int i=0; i<m_vecIds.size(); i++) {
		temPlane.Normal().X() = (*m_paras)[4*m_vecIds[i]+0];
		temPlane.Normal().Y() = (*m_paras)[4*m_vecIds[i]+1];
		temPlane.Normal().Z() = (*m_paras)[4*m_vecIds[i]+2];
		temPlane.Normal().Normalize();
		temPlane.Distance()   = -(*m_paras)[4*m_vecIds[i]+3];
		vecPlanes.push_back(temPlane);
	}

	double dist2 = 0.0;
	double energy = sqrt(this->GetEngergy());
	Vector3D crossPnt2;
	crossPnt2.X() = (*m_paras)(m_indCrossPnt+0);
	crossPnt2.Y() = (*m_paras)(m_indCrossPnt+1);
	crossPnt2.Z() = (*m_paras)(m_indCrossPnt+2);
	for (int i=0; i<vecPlanes.size(); i++) {
		dist2 += fabs(vecPlanes[i].Distance(crossPnt2)/vecPlanes.size());
	}

	double dist1 = 0.0;
	Vector3D crossPnt1 = CrossPntofPlanes();
	for (int i=0; i<vecPlanes.size(); i++) {
		dist1 += fabs(vecPlanes[i].Distance(crossPnt1)/vecPlanes.size());
	}

	return dist1;
}

void ConcurrentCons::AddPlanes(std::vector<int> vecInds)
{
	assert(vecInds.size()>3);
	std::sort(vecInds.begin(), vecInds.end());
	m_vecIds = vecInds;

	//////////////////////////////////////////////////////////////////////////
	//construct new parameter vector
	Vector3D crossPnt = CrossPntofPlanes();
	Eigen::VectorXd oldParas = *m_paras;
	m_indCrossPnt = m_paras->size();
	m_paras->resize(m_indCrossPnt+3);
	for (int i=0; i<oldParas.size(); i++)
		(*m_paras)[i] = oldParas[i];//copy old parameters
	(*m_paras)(m_indCrossPnt+0) = crossPnt.X();//add new parameters
	(*m_paras)(m_indCrossPnt+1) = crossPnt.Y();
	(*m_paras)(m_indCrossPnt+2) = crossPnt.Z();
	
	///////////////////////////////////////////////////////////////////////////
	//create the inside constraint
	InsideConcurrentCons* pInsideCons = NULL;
	for (int i=0; i<vecInds.size(); i++) {
		pInsideCons = NULL;
		pInsideCons = new InsideConcurrentCons(m_nPlanes, m_paras);
		if (!pInsideCons) continue;
		pInsideCons->SetIndexOfCrossPntPar(m_indCrossPnt);
		pInsideCons->AddPlanes(vector<int>(1,vecInds[i]));
		m_insideConsts.push_back(pInsideCons);
	}
}

double ConcurrentCons::GetEngergy()
{
	double allEnergy = 0.0;
	for (int i=0; i<m_insideConsts.size(); i++)
		allEnergy += m_insideConsts[i]->GetEngergy();
	
	return allEnergy;
}

Eigen::VectorXd ConcurrentCons::Get1stDer()
{
	Eigen::VectorXd all1stVec = Eigen::VectorXd::Zero(m_paras->size());
	for (int i=0; i<m_insideConsts.size(); i++)
		all1stVec += m_insideConsts[i]->Get1stDer();

	return all1stVec;
}

Eigen::MatrixXd ConcurrentCons::Get2stDer()
{
	Eigen::MatrixXd all2stMat = Eigen::MatrixXd::Zero(m_paras->size(), m_paras->size());
	for (int i=0; i<m_insideConsts.size(); i++)
		all2stMat += m_insideConsts[i]->Get2stDer();

	return all2stMat;
}

void ConcurrentCons::InsideUpdata()
{
	for (int i=0; i<m_insideConsts.size(); i++)
		m_insideConsts[i]->UpDataParas(m_paras);
}

//for concurrent constraint
void ConcurrentCons::AddParameters(int nAddParaNum)
{
	for (int i=0; i<m_insideConsts.size(); i++)
		m_insideConsts[i]->AddParameters(nAddParaNum);
}

//////////////////////////////////////////////
//for constraint least square fit

ConstraintLSF::ConstraintLSF(int nPlaneNum)
{
	m_nPlanes = nPlaneNum;
	m_nL = 0;

	m_H.resize(4*nPlaneNum, 4*nPlaneNum);
	m_H = MatrixXd::Zero(4*nPlaneNum, 4*nPlaneNum);
	m_p.resize(4*nPlaneNum);
	m_p = VectorXd::Zero(4*nPlaneNum);
}

ConstraintLSF::~ConstraintLSF(void)
{
	for (int i=0; i<m_nL; ++i) {
		//delete m_vecCons[i];
		//m_vecCons[i] = NULL;

		delete m_vecPCons[i];
		m_vecPCons[i] = NULL;
	}
}

//add order is very important
//without initial plane parameter
void ConstraintLSF::AddPlane(int planeInd, const LaserPoints& lasPnts, const PointNumberList& pnlSeg)
{
	Plane plane = lasPnts.FitPlane(pnlSeg);
	AddPlane(planeInd, lasPnts, pnlSeg, plane);
}

//with initial plane parameter
void ConstraintLSF::AddPlane(int planeInd, const LaserPoints& lasPnts, 
	const PointNumberList& pnlSeg, const Plane& plane)
{
	if (pnlSeg.size() < 4) 
		return;
	
	AddConstraint(Cons_Unit, planeInd);

	//initialize parameter vector
	m_p(planeInd*4+0) = plane.Normal().X();
	m_p(planeInd*4+1) = plane.Normal().Y();
	m_p(planeInd*4+2) = plane.Normal().Z();
	m_p(planeInd*4+3) = -plane.Distance();

	double ratio = 1.0/(pnlSeg.size());
	Matrix4d matrix = Matrix4d::Zero();
	double X[4];
	int startPos = 4*planeInd;
	m_H(startPos+0, startPos+0) = 0.0; 
	m_H(startPos+0, startPos+1) = 0.0;
	m_H(startPos+0, startPos+2) = 0.0;
	m_H(startPos+0, startPos+3) = 0.0;

	m_H(startPos+1, startPos+1) = 0.0;
	m_H(startPos+1, startPos+2) = 0.0;
	m_H(startPos+1, startPos+3) = 0.0;

	m_H(startPos+2, startPos+2) = 0.0;
	m_H(startPos+2, startPos+3) = 0.0;

	m_H(startPos+3, startPos+3) = 0.0;

	//set heissian matrix
	for (int i=0; i<pnlSeg.size(); ++i) {
		X[0] = lasPnts[pnlSeg[i].Number()].X();
		X[1] = lasPnts[pnlSeg[i].Number()].Y();
		X[2] = lasPnts[pnlSeg[i].Number()].Z();
		X[3] = 1.0;

		m_H(startPos+0, startPos+0) += X[0]*X[0]*ratio;
		m_H(startPos+0, startPos+1) += X[0]*X[1]*ratio;
		m_H(startPos+0, startPos+2) += X[0]*X[2]*ratio;
		m_H(startPos+0, startPos+3) += X[0]*X[3]*ratio;

		m_H(startPos+1, startPos+1) += X[1]*X[1]*ratio;
		m_H(startPos+1, startPos+2) += X[1]*X[2]*ratio;
		m_H(startPos+1, startPos+3) += X[1]*X[3]*ratio;

		m_H(startPos+2, startPos+2) += X[2]*X[2]*ratio;
		m_H(startPos+2, startPos+3) += X[2]*X[3]*ratio;

		m_H(startPos+3, startPos+3) += X[3]*X[3]*ratio;
	}

	m_H(startPos+1, startPos+0) = m_H(startPos+0, startPos+1);
	m_H(startPos+2, startPos+0) = m_H(startPos+0, startPos+2);
	m_H(startPos+2, startPos+1) = m_H(startPos+1, startPos+2);
	m_H(startPos+3, startPos+0) = m_H(startPos+0, startPos+3);
	m_H(startPos+3, startPos+1) = m_H(startPos+1, startPos+3);
	m_H(startPos+3, startPos+2) = m_H(startPos+2, startPos+3);

#ifdef _DEBUG
	printf("Add %th plane\n", planeInd);
	printf("%12.2f\t%12.2f\t%12.2f\t%12.2f\n", m_H(startPos+0, startPos+0), m_H(startPos+0, startPos+1), 
		m_H(startPos+0, startPos+2), m_H(startPos+0, startPos+3));
	printf("%12.2f\t%12.2f\t%12.2f\t%12.2f\n", m_H(startPos+1, startPos+0), m_H(startPos+1, startPos+1), 
		m_H(startPos+1, startPos+2), m_H(startPos+1, startPos+3));
	printf("%12.2f\t%12.2f\t%12.2f\t%12.2f\n", m_H(startPos+2, startPos+0), m_H(startPos+2, startPos+1), 
		m_H(startPos+2, startPos+2), m_H(startPos+2, startPos+3));
	printf("%12.2f\t%12.2f\t%12.2f\t%12.2f\n\n", m_H(startPos+3, startPos+0), m_H(startPos+3, startPos+1), 
		m_H(startPos+3, startPos+2), m_H(startPos+3, startPos+3));
	double energy = m_p.transpose()*m_H*m_p;

	double dist = 0.0;
	double distSum = 0.0;
	for (int i=0; i<pnlSeg.size(); ++i) {
		dist = plane.Distance(lasPnts[pnlSeg[i].Number()]);
		distSum += dist*dist*ratio;
	}
#endif
}

void ConstraintLSF::GetPlanes(std::vector<Plane>& vecPlanes)
{
	vecPlanes.clear();
	Plane plane;
	for (int i=0; i<m_nPlanes; ++i){
		plane.Normal().X() = m_p(4*i+0);
		plane.Normal().Y() = m_p(4*i+1);
		plane.Normal().Z() = m_p(4*i+2);
		plane.Distance() = -m_p(4*i+3);
		vecPlanes.push_back(plane);
	}
}

double ConstraintLSF::ComputeEngergy()
{
/*	double energy = 0.0;
	double temCost;
	
	energy = m_p.transpose()*m_H*m_p;
	for (int iL=0; iL<m_nL; ++iL) {
		temCost = m_lamida[iL]*m_p.transpose()*(*m_vecCons[iL])*m_p;
		temCost = temCost*temCost;
		energy += temCost;
	}
	
	return energy;*/
	return ComputeEngergy(m_p, m_lamida);
}

double ConstraintLSF::ComputeEngergy(VectorXd& para, std::vector<LAMIDA_TYPE>& lamida)
{
	double energy = para.transpose()*m_H*para;
	double temCost;
	Constraint* pCons = NULL;
	for (int iL=0; iL<m_nL; ++iL) {
		pCons = m_vecPCons[iL];
		if (!pCons) continue;

		pCons->UpDataParas(&para);
		temCost = lamida[iL]*pCons->GetEngergy();
		energy += temCost;
	}

	return energy;
}

double ConstraintLSF::ComputeConsEng()
{
	double energy = 0.0;
	double temCost;
	Constraint* pCons = NULL;
	for (int iL=0; iL<m_nL; ++iL) {
		pCons = m_vecPCons[iL];
		if (!pCons) continue;

		pCons->UpDataParas(&m_p);
		temCost = pCons->GetEngergy();
		energy += temCost;
	}

	return energy;
}

double ConstraintLSF::ComputeFitEng()
{
	return m_p.transpose()*m_H*m_p;
}


/*//////////////////////////////////////////////////////////////
|a1|T    |0  1 0 0|   |a2| 
|b1|     |-1 0 0 0|   |b2|
|c1|  *  |0  0 0 0| * |c2| = 0
|d1|	 |0  0 0 0|   |d2|
(n1*n2)(0,0,1,0)'=0
a1*b2-b1*a2=0
//////////////////////////////////////////////////////////////*/
//the intersection line of two planes is horizontal
/*void ConstraintLSF::AddHoriLineCons(int planeInd1, int planeInd2)
{
	if (planeInd1>planeInd2) std::swap(planeInd1, planeInd2);
	if (planeInd1<0 || planeInd1==planeInd2 || planeInd2>=m_nP) 
		return;

	MatrixXd* matrix = new MatrixXd(m_nP, m_nP);
	*matrix = MatrixXd::Zero(m_nP, m_nP);

	(*matrix)(4*planeInd1+0, 4*planeInd2+1) = 1.0;
	(*matrix)(4*planeInd1+1, 4*planeInd2+0) = -1.0;

	m_vecCons.push_back(matrix);
	m_nL++;
}*/

/*//////////////////////////////////////////////////////////////
|a1|T    |1 0 0 0|   |a2| 
|b1|     |0 1 0 0|   |b2|
|c1|  *  |0 0 1 0| * |c2| = 0
|d1|	 |0 0 0 0|   |d2|
n1'n2=0
//////////////////////////////////////////////////////////////*/
//two planes are perpendicular
/*void ConstraintLSF::AddPerpCons(int planeInd1, int planeInd2)
{
	if (planeInd1>planeInd2) std::swap(planeInd1, planeInd2);
	if (planeInd1<0 || planeInd1==planeInd2 || planeInd2>=m_nP) 
		return;
	
	MatrixXd* matrix = new MatrixXd(m_nP, m_nP);
	*matrix = MatrixXd::Zero(m_nP, m_nP);

	(*matrix)(4*planeInd1+0, 4*planeInd2+0) = 1.0;
	(*matrix)(4*planeInd1+1, 4*planeInd2+1) = 1.0;
	(*matrix)(4*planeInd1+2, 4*planeInd2+2) = 1.0;

	m_vecCons.push_back(matrix);
	m_nL++;

#ifdef _DEBUG
	vector<MatrixXd*>::iterator itr = m_vecCons.begin()+m_vecCons.size()-1;
	int pos1 = 4*planeInd1;
	int pos2 = 4*planeInd2;
	printf("%.2f\t%.2f\t%.2f\t%.2f\n", (**itr)(pos1+0, pos2+0), (**itr)(pos1+0, pos2+1), 
		(**itr)(pos1+0, pos2+2), (**itr)(pos1+0, pos2+3) );
	printf("%.2f\t%.2f\t%.2f\t%.2f\n", (**itr)(pos1+1, pos2+0), (**itr)(pos1+1, pos2+1), 
		(**itr)(pos1+1, pos2+2), (**itr)(pos1+1, pos2+3) );
	printf("%.2f\t%.2f\t%.2f\t%.2f\n", (**itr)(pos1+2, pos2+0), (**itr)(pos1+2, pos2+1), 
		(**itr)(pos1+2, pos2+2), (**itr)(pos1+2, pos2+3) );
	printf("%.2f\t%.2f\t%.2f\t%.2f\n", (**itr)(pos1+3, pos2+0), (**itr)(pos1+3, pos2+1), 
		(**itr)(pos1+3, pos2+2), (**itr)(pos1+3, pos2+3) );
	double energy = m_p.transpose()*m_H*m_p;
#endif
}*/

//the main function to solve the constraint least square function
void ConstraintLSF::Solve()
{
	//initialize parameter P and Lagrange multiplier L
	double F = m_p.transpose()*m_H*m_p;
	m_lamida = vector<LAMIDA_TYPE>(m_nL, 0);
	double temCost, sumCost;
	sumCost = F;
	Constraint* pCons = NULL;
	double temLambda;
	for (int iL=0; iL<m_nL; ++iL) {
		pCons = m_vecPCons[iL];
		if (!pCons) continue;

		pCons->UpDataParas(&m_p);
		temCost = pCons->GetEngergy();
		temLambda = (temCost)/F;
		//temLambda = exp(temLambda)+4;
		if (temLambda>20.0) temLambda = 20.0;
		temLambda = 10.0/(1.0+exp(-0.5*temCost))-2;
		m_lamida[iL] = (int)temLambda;
		//m_lamida(iL) = (temCost+0.00001)/F;
		sumCost += temLambda*temCost;
	}

	int nLoops = 0;
	double diffE = 10000.0;
//	double oldE;
	sumCost = 100.0;
	bool bBigLambda = false;

//	while (sumCost>0.000005010 && nLoops<500) {
	while (!bBigLambda && sumCost>0.0000000090 && nLoops<500) {
//	while (sumCost>0.0000000010 && nLoops<500) {
#ifdef _DEBUG
		//double debugEnergy;
		//VectorXd debugLambda = m_lamida;
		//for (int i=0; i<debugLambda.size(); ++i)
		//	debugLambda(i) = 1.0;
		printf("\nMain Loop: %d\n", nLoops);
		printf("Fit Energy: %.8f   Cons Energy: %.8f\n", 
			ComputeFitEng(), ComputeConsEng());
		printf("Lamida: ");
		for (int i=0; i<m_lamida.size(); ++i) {
			printf("%d  ", m_lamida[i]);
		}
		printf("\n");
#endif
		//oldE = ComputeEngergy();
		//oldE = ComputeConsEng();

		//update parameter vector
		MinimizeEnergy();
		//oldE = sumCost;
		//sumCost = ComputeEngergy();
		sumCost = ComputeConsEng();
		//diffE = oldE-sumCost;

//#ifdef _DEBUG
//		printf("Main Loop Energy Dif: %.8f\n\n", diffE);
//#endif

		
		for (int iL=0; iL<m_lamida.size(); ++iL) {
			m_lamida[iL] *= 2;
			if (m_lamida[iL]>INT_MAX/2.0) 
				bBigLambda = true;
		}
		nLoops++;
	}
}

void ConstraintLSF::MinimizeEnergy()
{
	double E = 10000000.0;
	double EUpdata;
	double diffE = 0.0;
	double alpha = 1.0;
	VectorXd grad = VectorXd::Zero(m_p.size());//first derivation
	MatrixXd hess = MatrixXd::Zero(m_p.size(), m_p.size());//second derivation
	VectorXd temGrad;
	MatrixXd temHess;
	Constraint* pCons=NULL;
	int nLoops = 0;
	VectorXd diffP;
	VectorXd PUpdata;

//	MatrixXd temMatrix = m_H;
//	for (int iL=0; iL<m_nL; ++iL) 
//		temMatrix += m_lamida(iL)*(*m_vecCons[iL]);
//	E = m_p.transpose()*temMatrix*m_p;
	E = ComputeEngergy();

	do {
		for (int innerLoops = 0; innerLoops<30; innerLoops++) {
			grad = VectorXd::Zero(m_p.size());
			hess = MatrixXd::Zero(m_p.size(), m_p.size());
			grad += 2*m_H*m_p;//2*H*n
			hess += 2*m_H;//2*H

			for (int iL=0; iL<m_nL; ++iL) {
				pCons = m_vecPCons[iL];
				if (!pCons) continue;
				pCons->UpDataParas(&m_p);
				temGrad = pCons->Get1stDer();
				temHess = pCons->Get2stDer();

				grad += m_lamida[iL]*temGrad;
				hess += m_lamida[iL]*temHess;
			}
			
			//hess += alpha*hess.diagonal();
			for (int i=0; i<m_p.size(); i++)
				hess(i,i) = alpha*hess(i,i);

			//diffP = hess.fullPivLu().solve(grad);
			//diffP = hess.fullPivHouseholderQr().solve(grad);
			diffP = hess.colPivHouseholderQr().solve(grad);
			
			
			PUpdata = m_p-diffP;
			/*
#ifdef _DEBUG
			printf("\nParameter difference:\n");
			for (int i=0; i<m_p.size(); i++)
				printf("%.9f \t", diffP(i));
			printf("\n\n");
#endif
			*/
			
			EUpdata = ComputeEngergy(PUpdata, m_lamida);
			if ( EUpdata>E) {
				alpha *= 3.0;
			}
			else {
				m_p = PUpdata;
				diffE = E- EUpdata;
				E = EUpdata;
				alpha /= 3.0;
				break;
			}
		}

#ifdef _DEBUG
		//VectorXd debugLambda = m_lamida;
		//for (int i=0; i<debugLambda.size(); ++i)
		//	debugLambda(i) = 1.0;
		//printf("%dth iteration, Energy: %.8f, Difference: %.8f\n", nLoops, 
		//	ComputeEngergy(m_p, debugLambda), diffE);
		//printf(" %dth iteration, Energy: %.8f, Dif: %.8f\n", nLoops, E+diffE, diffE);
		printf("%.10f, %.10f\n", ComputeFitEng(), ComputeConsEng());
		//printf("Parameter difference:\n");
		//for (int i=0; i<m_p.size(); i++)
		//	printf("%.8f \t", diffP(i));
		//printf("\n");
#endif
		nLoops++;
	//} while (fabs(diffE) > 0.0005 && nLoops<10);
	//} while (fabs(diffE) > 0.000005 && nLoops<30);
	} while (fabs(diffE) > 0.000005 && nLoops<10);

#ifdef _DEBUG
	//printf("\n\n");
#endif
}

Constraint* ConstraintLSF::AddConstraint(Cons_Type type, int planeId)
{
	vector<int> vecPlaneIds;
	vecPlaneIds.push_back(planeId);
	return AddConstraint(type, vecPlaneIds);
}

Constraint* ConstraintLSF::AddConstraint(Cons_Type type, int planeInd1, int planeInd2)
{
	vector<int> vecPlaneIds;
	vecPlaneIds.push_back(planeInd1);
	vecPlaneIds.push_back(planeInd2);
	return AddConstraint(type, vecPlaneIds);
}

Constraint* ConstraintLSF::AddConstraint(Cons_Type type, std::vector<int> vecPlaneIds)
{
	Constraint* pCons = NULL;
	switch (type)
	{
	case Cons_Unit:
		pCons = new UnitCons(m_nPlanes, &m_p);
		if (pCons) {
			((UnitCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	case Cons_Horizontal:
		pCons = new HoriCons(m_nPlanes, &m_p);
		if (pCons) {
			((HoriCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	case Cons_Vertical:
		pCons = new VertCons(m_nPlanes, &m_p);
		if (pCons) {
			((VertCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	case Cons_Equal_Slop:
		pCons = new EquSlopCons(m_nPlanes, &m_p);
		if (pCons) {
			((EquSlopCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	case Cons_Hor_Inter:
		pCons = new HorInterCons(m_nPlanes, &m_p);
		if (pCons) {
			((HorInterCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	case Cons_Concurrent:
		pCons = new ConcurrentCons(m_nPlanes, &m_p);
		if (pCons) {
			//the construct need the parameters
			pCons->UpDataParas(&m_p);
/*
#ifdef _DEBUG
			printf("Old Parameters:\n");
			for (int i=0; i<m_p.size(); i++)
				printf("%8.3f ", m_p(i));
			printf("\n\n");

			for (int i=0; i<m_H.rows(); i++) {
				for (int j=0; j<m_H.cols(); j++) {
					printf("%12.3f ", m_H(i, j));
				}
				printf("\n\n");
			}

			printf("\n\n\n");				
#endif
			*/

			//update old parameters			
			//m_nP += 3;
			Eigen::MatrixXd oldH = m_H;
			m_H = Eigen::MatrixXd::Zero(oldH.rows()+3, oldH.cols()+3);
			//m_H.resize(oldH.rows()+3, oldH.cols()+3);
			//m_H.Zero(oldH.rows()+3, oldH.cols()+3);
			for (int i=0; i<oldH.rows(); i++)
				for (int j=0; j<oldH.cols(); j++)
					m_H(i, j) = oldH(i, j);


			//update other constraints
			for (int i=0; i<m_vecPCons.size(); i++)
				m_vecPCons[i]->AddParameters(3);

			//add plane
			((ConcurrentCons*)pCons)->AddPlanes(vecPlaneIds);
			
//#ifdef _DEBUG
//			printf("New Parameters:\n");
//			for (int i=0; i<m_p.size(); i++)
//				printf("%8.3f ", m_p(i));
//			printf("\n\n");

//			for (int i=0; i<m_H.rows(); i++) {
//				for (int j=0; j<m_H.cols(); j++) {
//					printf("%12.3f ", m_H(i, j));
//				}
//				printf("\n\n");
//			}
//#endif
			
		}
		break;
	case Cons_Orthogonal:
		pCons = new OrthoCons(m_nPlanes, &m_p);
		if (pCons) {
			((OrthoCons*)pCons)->AddPlanes(vecPlaneIds);
		}
		break;
	}

	if (pCons) {
		//pCons->AddPlanes(vecPlaneIds);
		m_vecPCons.push_back(pCons);
		m_nL++;
	}

	return pCons;
}

/*
void ConcurrentCons::InsideUpdata()
{
	_W = MatrixXd::Zero(4, m_vecIds.size());

	int id;
	//transform parameter vector to matrix
	for (int i=0; i<m_vecIds.size(); i++) {
		id = m_vecIds[i];
		_W(0, i) = (*m_paras)(4*id+0);
		_W(1, i) = (*m_paras)(4*id+1);
		_W(2, i) = (*m_paras)(4*id+2);
		_W(3, i) = (*m_paras)(4*id+3);
	}

	_WW = _W*_W.transpose();
	_invWW = _WW.inverse();
	_DetWW = _WW.determinant();
}

//|WW'|*|WW'|
double ConcurrentCons::GetEngergy()
{
	return _DetWW*_DetWW;
}

//|WW'|*|WW'|*inv(WW')*W
Eigen::VectorXd ConcurrentCons::Get1stDer()
{
	MatrixXd J = GetEngergy()*_invWW*_W;
	VectorXd vecJ = VectorXd::Zero(4*m_nPlanes);

	//transform J to vector
	int id;
	for (int i=0; i<m_vecIds.size(); i++) {
		id = m_vecIds[i];
		vecJ(4*id+0) = J(0, i);
		vecJ(4*id+1) = J(1, i);
		vecJ(4*id+2) = J(2, i);
		vecJ(4*id+3) = J(3, i);
	}

	return vecJ;
}

Eigen::MatrixXd ConcurrentCons::Get2stDer()
{
//	MatrixXd orgH =  MatrixXd::Zero(4*m_nPlanes, 4*m_nPlanes);

//	//double scale = 24*_DetWW;
//	int len = 4*m_vecIds.size();
//	MatrixXd K = CommutationMatrix(len,len);
//	MatrixXd www = _W.transpose()*_invWW;//W'inv(WW')
//	MatrixXd I = MatrixXd::Ones(len, len);
//	MatrixXd H = 24*_DetWW*K*CircleProduct(www.transpose(), www) 
//		+ 4*_DetWW*CircleProduct(I, _invWW);

	MatrixXd orgH =  MatrixXd::Zero(4*m_nPlanes, 4*m_nPlanes);
	double scale = _DetWW*_DetWW;
	int len = m_vecIds.size();
	MatrixXd K = CommutationMatrix(len, 4);
	MatrixXd www = _W.transpose()*_invWW;//W'inv(WW')
	MatrixXd I = MatrixXd::Ones(len, len);
	MatrixXd H = 12*scale*K*CircleProduct(www.transpose(), www)
		+ 4*scale*CircleProduct(I, _invWW) 
		- 4*scale*CircleProduct(_W.transpose()*_invWW*_W, _invWW);
//	MatrixXd H2 = 4*scale*CircleProduct(I, _invWW) ;
//	MatrixXd H3 = - 4*scale*CircleProduct(_W.transpose()*_invWW*_W, _invWW);

	//transform to original patten
	int id1, id2;
	for (int i=0; i<m_vecIds.size(); ++i) {
		id1 = m_vecIds[i];
		for (int j=0; j<m_vecIds.size(); ++j) {
			id2 = m_vecIds[j];
			orgH(4*id1+0, 4*id2+0) = H(4*i+0, 4*j+0);
			orgH(4*id1+0, 4*id2+1) = H(4*i+0, 4*j+1);
			orgH(4*id1+0, 4*id2+2) = H(4*i+0, 4*j+2);
			orgH(4*id1+0, 4*id2+3) = H(4*i+0, 4*j+3);

			orgH(4*id1+1, 4*id2+0) = H(4*i+1, 4*j+0);
			orgH(4*id1+1, 4*id2+1) = H(4*i+1, 4*j+1);
			orgH(4*id1+1, 4*id2+2) = H(4*i+1, 4*j+2);
			orgH(4*id1+1, 4*id2+3) = H(4*i+1, 4*j+3);

			orgH(4*id1+2, 4*id2+0) = H(4*i+2, 4*j+0);
			orgH(4*id1+2, 4*id2+1) = H(4*i+2, 4*j+1);
			orgH(4*id1+2, 4*id2+2) = H(4*i+2, 4*j+2);
			orgH(4*id1+2, 4*id2+3) = H(4*i+2, 4*j+3);

			orgH(4*id1+3, 4*id2+0) = H(4*i+3, 4*j+0);
			orgH(4*id1+3, 4*id2+1) = H(4*i+3, 4*j+1);
			orgH(4*id1+3, 4*id2+2) = H(4*i+3, 4*j+2);
			orgH(4*id1+3, 4*id2+3) = H(4*i+3, 4*j+3);
		}
	}

	return orgH;
}*/