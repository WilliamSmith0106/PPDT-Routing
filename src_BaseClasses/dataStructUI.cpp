#include "dataStructUI.h"
using namespace std;


vector<QPointF> LineUI::getIntPts(bool withP1, bool withP2) {
	int dist = 1;
	//Get all integer points on the line segment, starting from the first point
	vector<QPointF> intPts;
	int x1 = (int)this->p1().x(), y1 = (int)this->p1().y();
	int x2 = (int)this->p2().x(), y2 = (int)this->p2().y();
	if (x1 == x2 && y1 == y2) return {};
	// Vertical line, x is equal
	if (x1 == x2) {
		int step = (y2 > y1) ? dist : -dist;
		int y = y1;
		if (!withP1) y += step;
		while ((step > 0 && y <= y2) || (step < 0 && y >= y2)) {
			intPts.emplace_back(QPointF(x1, y));
			y += step;
		}
		if (withP2 && intPts.back() != QPointF(x2, y2)) {
			intPts.emplace_back(QPointF(x2, y2));
		}
		return intPts;
	}
	// Horizontal line, y is equal
	if (y1 == y2) {
		int step = (x2 > x1) ? dist : -dist;
		int x = x1;
		if (!withP1) x += step;
		while ((step > 0 && x <= x2) || (step < 0 && x >= x2)) {
			intPts.emplace_back(QPointF(x, y1));
			x += step;
		}
		if (withP2 && intPts.back() != QPointF(x2, y2)) {
			intPts.emplace_back(QPointF(x2, y2));
		}
		return intPts;
	}
	// Slant line
	double dx = x2 - x1;
	double dy = y2 - y1;
	double k = dy / dx;
	// Determine whether the absolute value of the slope is >1 to decide whether to step by x or y
	if (abs(k) <= 1) {
		// Gentle line segment, step by x
		int step = (x2 > x1) ? dist : -dist;
		int x = x1;
		if (!withP1) x += step;
		while ((step > 0 && x <= x2) || (step < 0 && x >= x2)) {
			double y = y1 + k * (x - x1);
			int roundedY = (int)round(y);
			intPts.emplace_back(QPointF(x, roundedY));
			x += step;
		}
	}
	else {
		// Steep line segment, step by y
		int step = (y2 > y1) ? dist : -dist;
		int y = y1;
		if (!withP1) y += step;
		while ((step > 0 && y <= y2) || (step < 0 && y >= y2)) {
			double x = x1 + (y - y1) / k;
			int roundedX = (int)round(x);
			intPts.emplace_back(QPointF(roundedX, y));
			y += step;
		}
	}
	// Ensure p2 is added correctly (if withP2=true)
	if (withP2 && (intPts.empty() || intPts.back() != QPointF(x2, y2))) {
		intPts.emplace_back(QPointF(x2, y2));
	}
	return intPts;
}