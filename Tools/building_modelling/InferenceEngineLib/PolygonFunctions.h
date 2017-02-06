
#ifndef _POLYGON_FUNCTIONS_H_
#define _POLYGON_FUNCTIONS_H_

#include "PointNumberList.h"

template<typename PointType>
double ComputePolygonArea(const std::vector<PointType>& vecPoints, const PointNumberList& pnl)
{
	if (pnl.size()<5 || pnl[0]!=pnl[pnl.size()-1]) return 0.0;

	typename std::vector<PointType>::const_iterator point0, point1;
	double area = 0.0;

	for (unsigned int i=0; i<pnl.size()-1; i++) {
		point0 = vecPoints.begin()+pnl[i].Number();
		point1 = vecPoints.begin()+pnl[i+1].Number();

		area += (point1->X()+point0->X())*(point1->Y()-point0->Y());
	}

	area = fabs(area/2);

	return area;
}

template<typename PointType>
double ComputePolylineLength(const std::vector<PointType>& vecPoints, const PointNumberList& pnl,int dimention=2)
{
	typename std::vector<PointType>::const_iterator point0, point1;

	double length = 0.0;
	double dx, dy, dz;

	if (dimention==2) {
		for (unsigned int i=0; i<pnl.size()-1; i++) {
			point0 = vecPoints.begin()+pnl[i].Number();
			point1 = vecPoints.begin()+pnl[i+1].Number();

			dx = point1->X()-point0->X();
			dy = point1->Y()-point0->Y();
			length += sqrt(dx*dx+dy*dy);
		}
	}
	else if(dimention==3) {
		for (unsigned int i=0; i<pnl.size()-1; i++) {
			point0 = vecPoints.begin()+pnl[i].Number();
			point1 = vecPoints.begin()+pnl[i+1].Number();

			dx = point1->X()-point0->X();
			dy = point1->Y()-point0->Y();
			dz = point1->Z()-point0->Z();
			length += sqrt(dx*dx+dy*dy+dz*dz);
		}
	}

	return length;
}


template <class Point>
int compare_x_2(const Point& lhv, const Point& rhv)
{
	if (lhv.X() < rhv.X()) return -1;
	else if (lhv.X() == rhv.X()) return 0;
	else return 1;
}

template <class Point>
int compare_y_2(const Point& lhv, const Point& rhv)
{
	if (lhv.Y() < rhv.Y()) return -1;
	else if (lhv.Y() == rhv.Y()) return 0;
	else return 1;
}


template <class Point>
int which_side_in_slab(Point const &point, Point const &low, Point const &high)
{
	//register double temX=(point.Y()-low.Y())*(high.X()-low.X())/(high.Y()-low.Y())+low.X();
	double temX=(point.Y()-low.Y())*(high.X()-low.X())/(high.Y()-low.Y())+low.X();
	
	if (point.X() < temX) return -1;
	else if(point.X() == temX) return 0;
	else return 1;
}

//-1, outbounded; 0, on boundary; 1, bounded
template<typename PointType>
int BoundedSidePolygon(const std::vector<PointType>& vecPoints, const PointNumberList& pnl, const PointType& point)
{
	if (pnl.size()<5 || pnl[0]!=pnl[pnl.size()-1]) return -1;

	typename std::vector<PointType>::const_iterator current, next;
	current = vecPoints.begin()+pnl[0].Number();
	bool IsInside = false;
	int cur_y_comp_res = compare_y_2(*current, point);
	int next_y_comp_res;

	// check if the segment (current,next) intersects
	// the ray { (t,point.y()) | t >= point.x() }
	for (unsigned int i=1; i<pnl.size(); i++) {
		next = vecPoints.begin()+pnl[i].Number();
		next_y_comp_res = compare_y_2(*next, point);

		switch (cur_y_comp_res) {
		case -1:
			switch (next_y_comp_res) {
			case -1:
				break;
			case 0:
				switch (compare_x_2(point, *next)) {
				case -1: IsInside = !IsInside; break;
				case 0:   return 0;
				case 1:  break;
				}
				break;
			case 1:
				switch (which_side_in_slab(point, *current, *next)) {
				case -1: IsInside = !IsInside; break;
				case  0: return 0;
				}
				break;
			}
			break;
		case 0:
			switch (next_y_comp_res) {
			case -1:
				switch (compare_x_2(point, *current)) {
				case -1: IsInside = !IsInside; break;
				case 0:   return 0;
				case 1:  break;
				}
				break;
			case 0:
				switch (compare_x_2(point, *current)) {
				case -1:
					if (compare_x_2(point, *next) != -1)
						return 0;
					break;
				case 0: return 0;
				case 1:
					if (compare_x_2(point, *next) != 1)
						return 0;
					break;
				}
				break;
			case 1:
				if (compare_x_2(point, *current) == 0) {
					return 0;
				}
				break;
			}
			break;
		case 1:
			switch (next_y_comp_res) {
			case -1:
				switch (which_side_in_slab(point, *next, *current)) {
				case -1: IsInside = !IsInside; break;
				case  0: return 0;
				}
				break;
			case 0:
				if (compare_x_2(point, *next) == 0) {
					return 0;
				}
				break;
			case 1:
				break;
			}
			break;
		}

		current = next;
		cur_y_comp_res = next_y_comp_res;
	}

	return IsInside ? 1 : -1;
}

#endif
