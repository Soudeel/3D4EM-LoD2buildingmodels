// CGAL values
#include <CGAL/enum.h>

// CGAL classes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <fstream>
#include <vector>

using namespace std;


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> VB;
typedef CGAL::Constrained_triangulation_face_base_2<K> FB;
typedef CGAL::Triangulation_face_base_2<K, FB> FBWI;
typedef CGAL::Triangulation_data_structure_2<VB, FBWI> TDS;
typedef CGAL::Exact_predicates_tag PT;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, PT> CDT;

typedef CGAL::Constrained_triangulation_plus_2<CDT> Triangulation;
typedef CGAL::Polygon_2<K> Ring;
typedef Triangulation::Point Point;

std::vector<Ring> importPolygons();
bool exportTriangulation(const Triangulation& triangulation, char* path);

void test_cgal_ct()
{
	vector<Ring> rings = importPolygons();
	Triangulation triangulation;
	Triangulation::Face_handle startingSearchFace;
	Triangulation::Vertex_handle sourceVertex, targetVertex;

	for (vector<Ring>::iterator itrRing=rings.begin(); itrRing!=rings.end(); itrRing++)	{
		for (Ring::Edge_const_iterator itrEdge = itrRing->edges_begin(); itrEdge != itrRing->edges_end(); ++itrEdge) {
			sourceVertex = triangulation.insert(itrEdge->source(), startingSearchFace);
			startingSearchFace = triangulation.incident_faces(sourceVertex);
			targetVertex = triangulation.insert(itrEdge->target(), startingSearchFace);
			triangulation.insert_constraint(sourceVertex, targetVertex);
			startingSearchFace = triangulation.incident_faces(targetVertex);
		} 
	}


	exportTriangulation(triangulation, "constrainted_d_t.edg");
}

std::vector<Ring> importPolygons()
{
	std::vector<Ring> rings;
	Ring temRing;

	//first polygon
	temRing.push_back(Point(155015.037,	461751.51));
	temRing.push_back(Point(155010.494,	461747.78));
	temRing.push_back(Point(155013.939,	461736.096));
	temRing.push_back(Point(155007.038,	461733.719));
	temRing.push_back(Point(155000,	461731.511));
	temRing.push_back(Point(154996.622,	461730.405));
	temRing.push_back(Point(154995.081,	461735.506));
	temRing.push_back(Point(154990.405,	461732.113));
	temRing.push_back(Point(154988.614,	461734.504));
	temRing.push_back(Point(154981.703,	461729.293));
	temRing.push_back(Point(154979.824,	461731.755));
	temRing.push_back(Point(154972.986,	461726.498));
	temRing.push_back(Point(154971.212,	461728.933));
	temRing.push_back(Point(154965.462,	461746.749));
	temRing.push_back(Point(154966.227,	461746.985));
	temRing.push_back(Point(154966.183,	461747.808));
	temRing.push_back(Point(154969.099,	461748.704));
	temRing.push_back(Point(154969.525,	461747.998));
	temRing.push_back(Point(154979.34,	461751.015));
	temRing.push_back(Point(154979.541,	461751.815));
	temRing.push_back(Point(154982.19,	461752.712));
	temRing.push_back(Point(154982.89,	461752.112));
	temRing.push_back(Point(154983.59,	461752.36));
	temRing.push_back(Point(154983.79,	461753.21));
	temRing.push_back(Point(154986.54,	461754.107));
	temRing.push_back(Point(154987.29,	461753.457));
	temRing.push_back(Point(154990.805,	461754.639));
	temRing.push_back(Point(154992.972,	461755.323));
	temRing.push_back(Point(154992.969,	461756.007));
	temRing.push_back(Point(154999.598,	461758.11));
	temRing.push_back(Point(154999.982,	461757.525));
	temRing.push_back(Point(155000.704,	461757.762));
	temRing.push_back(Point(155000.738,	461758.512));
	temRing.push_back(Point(155008.295,	461760.968));
	temRing.push_back(Point(155008.854,	461760.496));
	temRing.push_back(Point(155011.323,	461761.302));
	temRing.push_back(Point(155011.48,	461761.891));
	temRing.push_back(Point(155016.106,	461763.39));
	temRing.push_back(Point(155017.015,	461762.881));
	temRing.push_back(Point(155017.577,	461763.363));
	temRing.push_back(Point(155022.534,	461757.579));
	temRing.push_back(Point(155027.086,	461752.266));
	temRing.push_back(Point(155026.835,	461752.045));
	temRing.push_back(Point(155019.848,	461745.913));
	rings.push_back(temRing);

	temRing.clear();
	temRing.push_back(Point(154999.7815,	461757.8304));
	temRing.push_back(Point(154998.9431,	461757.7994));
	temRing.push_back(Point(154998.0113,	461757.4693));
	temRing.push_back(Point(154997.1148,	461757.2361));
	temRing.push_back(Point(154996.2541,	461756.9572));
	temRing.push_back(Point(154995.5183,	461756.586));
	temRing.push_back(Point(154993.0452,	461755.7099));
	temRing.push_back(Point(154992.5369,	461755.1693));
	temRing.push_back(Point(154991.608,	461754.7762));
	temRing.push_back(Point(154991.5208,	461754.7476));
	temRing.push_back(Point(154990.4415,	461754.4551));
	temRing.push_back(Point(154990.2385,	461754.365));
	temRing.push_back(Point(154989.51,	461753.947));
	temRing.push_back(Point(154989.0991,	461753.9526));
	temRing.push_back(Point(154988.6635,	461753.8097));
	temRing.push_back(Point(154987.5595,	461753.5416));
	temRing.push_back(Point(154987.1328,	461753.5932));
	temRing.push_back(Point(154986.7383,	461753.4754));
	temRing.push_back(Point(154986.2183,	461753.7052));
	temRing.push_back(Point(154985.7079,	461753.6273));
	temRing.push_back(Point(154984.8659,	461753.267));
	temRing.push_back(Point(154984.0443,	461753.0866));
	temRing.push_back(Point(154983.6965,	461752.8126));
	temRing.push_back(Point(154982.7994,	461752.1121));
	temRing.push_back(Point(154982.5657,	461752.39));
	temRing.push_back(Point(154982.3566,	461752.4539));
	temRing.push_back(Point(154980.9545,	461751.9996));
	temRing.push_back(Point(154980.5108,	461751.9382));
	temRing.push_back(Point(154979.5524,	461751.5452));
	temRing.push_back(Point(154979.3941,	461751.0316));
	temRing.push_back(Point(154978.5097,	461750.5902));
	temRing.push_back(Point(154978.0596,	461750.6214));
	temRing.push_back(Point(154977.3912,	461750.225));
	temRing.push_back(Point(154976.677,	461750.1796));
	temRing.push_back(Point(154974.3667,	461749.4504));
	temRing.push_back(Point(154973.8388,	461749.324));
	temRing.push_back(Point(154972.7383,	461748.9857));
	temRing.push_back(Point(154971.9837,	461748.6982));
	temRing.push_back(Point(154970.2287,	461748.1442));
	temRing.push_back(Point(154969.4128,	461748.1839));
	temRing.push_back(Point(154968.8565,	461748.1506));
	temRing.push_back(Point(154968.6658,	461748.1325));
	temRing.push_back(Point(154967.7077,	461747.6306));
	temRing.push_back(Point(154967.2941,	461747.668));
	temRing.push_back(Point(154966.5126,	461747.4033));
	temRing.push_back(Point(154966.2638,	461747.2392));
	temRing.push_back(Point(154965.5024,	461746.6524));
	temRing.push_back(Point(154965.6436,	461746.3901));
	temRing.push_back(Point(154965.9002,	461745.3913));
	temRing.push_back(Point(154966.5031,	461743.5232));
	temRing.push_back(Point(154967.15,	461743.53));
	temRing.push_back(Point(154967.71,	461742.64));
	temRing.push_back(Point(154967.69,	461742.4));
	temRing.push_back(Point(154967.1355,	461741.5637));
	temRing.push_back(Point(154968.7386,	461736.5966));
	temRing.push_back(Point(154969.36,	461736.64));
	temRing.push_back(Point(154969.74,	461736.76));
	temRing.push_back(Point(154970.59,	461737.08));
	temRing.push_back(Point(154971.62,	461737.39));
	temRing.push_back(Point(154972.17,	461737.66));
	temRing.push_back(Point(154972.93,	461737.84));
	temRing.push_back(Point(154973.89,	461738.14));
	temRing.push_back(Point(154974.36,	461738.28));
	temRing.push_back(Point(154974.6996,	461738.4694));
	temRing.push_back(Point(154974.7416,	461738.4144));
	temRing.push_back(Point(154976.2419,	461738.6842));
	temRing.push_back(Point(154976.3856,	461739.2563));
	temRing.push_back(Point(154977.33,	461739.21));
	temRing.push_back(Point(154977.57,	461739.33));
	temRing.push_back(Point(154978.38,	461739.63));
	temRing.push_back(Point(154978.83,	461739.61));
	temRing.push_back(Point(154979.77,	461740.07));
	temRing.push_back(Point(154980.25,	461740.19));
	temRing.push_back(Point(154981.21,	461740.45));
	temRing.push_back(Point(154981.64,	461740.58));
	temRing.push_back(Point(154982.55,	461740.91));
	temRing.push_back(Point(154982.91,	461741.02));
	temRing.push_back(Point(154983.95,	461741.31));
	temRing.push_back(Point(154984.42,	461741.44));
	temRing.push_back(Point(154985.33,	461741.75));
	temRing.push_back(Point(154985.64,	461741.94));
	temRing.push_back(Point(154986.72,	461742.2));
	temRing.push_back(Point(154987.65,	461742.64));
	temRing.push_back(Point(154988.08,	461742.62));
	temRing.push_back(Point(154988.53,	461742.72));
	temRing.push_back(Point(154989.16,	461743.26));
	temRing.push_back(Point(154989.78,	461743.15));
	temRing.push_back(Point(154990.36,	461743.46));
	temRing.push_back(Point(154990.88,	461743.64));
	temRing.push_back(Point(154991.38,	461743.92));
	temRing.push_back(Point(154992.0915,	461744.225));
	temRing.push_back(Point(154993.0202,	461744.3302));
	temRing.push_back(Point(154993.38,	461744.32));
	temRing.push_back(Point(154993.52,	461744.32));
	temRing.push_back(Point(154993.81,	461744.41));
	temRing.push_back(Point(154994.81,	461745.03));
	temRing.push_back(Point(154995.4,	461744.93));
	temRing.push_back(Point(154996.0993,	461744.9426));
	temRing.push_back(Point(154996.8887,	461745.6197));
	temRing.push_back(Point(154998.0973,	461745.8034));
	temRing.push_back(Point(154998.4865,	461746.1026));
	temRing.push_back(Point(154999.55,	461746.49));
	temRing.push_back(Point(154999.74,	461746.49));
	temRing.push_back(Point(155000.1915,	461746.4577));
	temRing.push_back(Point(155000.91,	461746.76));
	temRing.push_back(Point(155001.38,	461746.89));
	temRing.push_back(Point(155002.26,	461747.32));
	temRing.push_back(Point(155002.64,	461747.29));
	temRing.push_back(Point(155003.31,	461747.55));
	temRing.push_back(Point(155003.69,	461747.73));
	temRing.push_back(Point(155003.77,	461748.21));
	temRing.push_back(Point(155003.76,	461748.47));
	temRing.push_back(Point(155003.65,	461748.74));
	temRing.push_back(Point(155003.3478,	461749.8399));
	temRing.push_back(Point(155003.0357,	461750.8916));
	temRing.push_back(Point(155002.87,	461751.22));
	temRing.push_back(Point(155002.55,	461752.19));
	temRing.push_back(Point(155002.47,	461752.69));
	temRing.push_back(Point(155002.41,	461752.95));
	temRing.push_back(Point(155002.08,	461752.94));
	temRing.push_back(Point(155001.45,	461753.14));
	temRing.push_back(Point(155000.97,	461754));
	temRing.push_back(Point(155001.7787,	461754.7879));
	temRing.push_back(Point(155001.5691,	461755.1072));
	temRing.push_back(Point(155001.36,	461756.21));
	temRing.push_back(Point(155001.04,	461757.2));
	temRing.push_back(Point(155000.7941,	461757.9981));
	rings.push_back(temRing);

	return rings;
}

bool exportTriangulation(const Triangulation& triangulation, char* path)
{
	std::ofstream output(path);
	CGAL::set_ascii_mode(output);
	if (output) output << triangulation;
	return true;
}

