/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Oct 5, 2011
 |
 |
------------------------------------------------------------------- */
#ifndef _CONSTRAINT_LEAST_SQUARE_FIT_H_
#define _CONSTRAINT_LEAST_SQUARE_FIT_H_

#include "InferenceEngine.h"
#include "InferenceEngine_DLL.h"
#include "LaserPoints.h"
#include "PointNumberList.h"
#include "Plane.h"
#include <Eigen/Dense>
#include <vector>
typedef unsigned int LAMIDA_TYPE;

#define AMPLIFICATION 10

enum Cons_Type {
	Cons_Unit = 0,
	Cons_Horizontal=1,
	Cons_Vertical=2,
	Cons_Equal_Slop=3,
	Cons_Hor_Inter=4,
	Cons_Concurrent=5,
	Cons_Orthogonal=6
};

class Constraint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	Constraint(int nPlanes, Eigen::VectorXd* pParas)
		:m_nPlanes(nPlanes), m_paras(pParas){};
	~Constraint() {};

	void UpDataParas(Eigen::VectorXd* para) {
		m_paras = para; 
		InsideUpdata();
	}
	void AddPlanes(std::vector<int> vecInds) {m_vecIds = vecInds;}
	virtual double GetEngergy() = 0;
	virtual Eigen::VectorXd Get1stDer() = 0;
	virtual Eigen::MatrixXd Get2stDer() = 0;
	virtual void AddParameters(int nAddParaNum) = 0;//for concurrent constraint
protected:
	Constraint(){};
	virtual void InsideUpdata() = 0;

	std::vector<int> m_vecIds;
	Eigen::VectorXd* m_paras;
	int m_nPlanes;
};

//p'*A*P+B*p+C=0
class QuadricCons:public Constraint
{
public:
	QuadricCons(int nPlanes, Eigen::VectorXd* pParas);
	~QuadricCons() {};

	virtual double GetEngergy();
	virtual Eigen::VectorXd Get1stDer();
	virtual Eigen::MatrixXd Get2stDer();
	virtual void AddParameters(int nAddParaNum);//for concurrent constraint
protected:
	QuadricCons(){};
	virtual void InsideUpdata(){};

	Eigen::MatrixXd m_A;
	Eigen::VectorXd m_B;
	double m_C;
};

//(p'*A*P+B*p+C).^2=0
class SecQuadricCons:public Constraint
{
public:
	SecQuadricCons(int nPlanes, Eigen::VectorXd* pParas);
	~SecQuadricCons() {};

	virtual double GetEngergy();
	virtual Eigen::VectorXd Get1stDer();
	virtual Eigen::MatrixXd Get2stDer();
	virtual void AddParameters(int nAddParaNum);//for concurrent constraint
protected:
	SecQuadricCons(){};
	virtual void InsideUpdata();

	Eigen::MatrixXd m_A;
	Eigen::VectorXd m_B;
	Eigen::MatrixXd m_AA;//(A+A')
	double m_e;
	double m_C;
};

//horizontal constraint
class HoriCons:public QuadricCons
{
public:
	HoriCons(int nPlanes, Eigen::VectorXd* pParas)
		:QuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
	double GetConstraintValue();
};

//vertical constraint
class VertCons:public QuadricCons
{
public:
	VertCons(int nPlanes, Eigen::VectorXd* pParas)
		:QuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
};

//equal slop constraint
class EquSlopCons:public SecQuadricCons
{
public:
	EquSlopCons(int nPlanes, Eigen::VectorXd* pParas)
		:SecQuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
	double GetConstraintValue();
};

//horizontal intersection constraint
class HorInterCons:public SecQuadricCons
{
public:
	HorInterCons(int nPlanes, Eigen::VectorXd* pParas)
		:SecQuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
	double GetConstraintValue();
};

//a.^2+b.^2+c.^2=1
//||a.^2+b.^2+c.^2-1||=0
class UnitCons:public SecQuadricCons
{
public:
//	UnitCons(int nPlanes){m_nPlanes = nPlanes;}
	UnitCons(int nPlanes, Eigen::VectorXd* pParas)
		:SecQuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
};

//||a1*a2+b1*b2+c1*c2||=0
class OrthoCons:public SecQuadricCons
{
public:
	OrthoCons(int nPlanes, Eigen::VectorXd* pParas)
		:SecQuadricCons(nPlanes, pParas){};
	void AddPlanes(std::vector<int> vecInds);
};

class InsideConcurrentCons:public SecQuadricCons
{
public:
	InsideConcurrentCons(int nPlanes, Eigen::VectorXd* pParas)
		:SecQuadricCons(nPlanes, pParas){m_indCrossPnt=0;}
	void AddPlanes(std::vector<int> vecInds);

	void SetIndexOfCrossPntPar(int ind) {m_indCrossPnt = ind;}

private:
	int m_indCrossPnt;//ind of this cross point in the parameter vector P
};

/*
//concurrent constraint
//E = -b'*inv(A)*b +1
class ConcurrentCons:public Constraint
{
public:
	ConcurrentCons(int nPlanes):Constraint(nPlanes){};
	~ConcurrentCons();

	double GetEngergy();
	Eigen::VectorXd Get1stDer();
	Eigen::MatrixXd Get2stDer();
protected:
	ConcurrentCons(){};
	void InsideUpdata();

	Eigen::MatrixXd _W;
	Eigen::Matrix4d _WW;//WW'
	Eigen::Matrix4d _invWW;//inv(WW')
	//Eigen::Matrix4d _invWW;//inv(WW')
	double _DetWW;//|WW'|
};*/

//ai*X+bi*Y+ci*Z+di=0
class ConcurrentCons:public Constraint
{
public:
	ConcurrentCons(int nPlanes, Eigen::VectorXd* pParas)
		:Constraint(nPlanes, pParas){};
	~ConcurrentCons();

	void AddPlanes(std::vector<int> vecInds);
	double GetEngergy();
	Eigen::VectorXd Get1stDer();
	Eigen::MatrixXd Get2stDer();

	double CroosPntDist2AllPlanes();
	void AddParameters(int nAddParaNum);//for concurrent constraint
protected:
	ConcurrentCons(){};
	void InsideUpdata();
	Vector3D CrossPntofPlanes();

	std::vector<SecQuadricCons*> m_insideConsts;
	int m_indCrossPnt;//ind of this cross point in the parameter vector P
};

//constraint Leas Square Fit
IEDll_API class ConstraintLSF
{
public:
	ConstraintLSF(int nPlaneNum);
	~ConstraintLSF(void);

public:
	void Solve();//solve the constraint least square fit problem
//	void AddCoPointCons();//several planes intersect at one point
//	void AddHoriCons();//one plane is horizontal
//	void AddVertCons();//one plane is vertical
	//two planes are perpendicular 
//	void AddPerpCons(int planeInd1, int planeInd2);
	//the intersection line of two planes is horizontal
//	void AddHoriLineCons(int planeInd1, int planeInd2);
//	void AddConcurCons(std::vector<int> vecPInds);

	Constraint* AddConstraint(Cons_Type type, std::vector<int> vecPlaneIds);
	Constraint* AddConstraint(Cons_Type type, int planeId);
	Constraint* AddConstraint(Cons_Type type, int planeInd1, int planeInd2);

	//add order is very important
	//without initial plane parameter
	void AddPlane(int planeInd, const LaserPoints& lasPnts, const PointNumberList& pnlSeg);
	//with initial plane parameter
	void AddPlane(int planeInd, const LaserPoints& lasPnts, const PointNumberList& pnlSeg, const Plane& plane);
	
	void GetPlanes(std::vector<Plane>& vecPlanes);
private:
	void MinimizeEnergy();
	double ComputeEngergy();
	double ComputeEngergy(Eigen::VectorXd& para, std::vector<LAMIDA_TYPE>& lamida);
	double ComputeConsEng();
	double ComputeFitEng();
private:
	std::vector<double> m_vecGrad; //first derivative
	std::vector<double> m_vecHessian;//second derivative
	int m_nL;//number of Lagrange multiplier, constraint number
	int m_nPlanes;//number of plane number
	Eigen::MatrixXd m_H;
	Eigen::VectorXd m_p;//parameter vector
	//Eigen::VectorXd m_lamida;//Lagrange multiplier
	std::vector<LAMIDA_TYPE> m_lamida;//Lagrange multiplier
	std::vector< Eigen::MatrixXd* > m_vecCons;
	std::vector<Constraint*> m_vecPCons;
};

#endif