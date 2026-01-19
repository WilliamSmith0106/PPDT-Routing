#pragma once

#include<math.h>
#include<iostream>
#include <algorithm>
#include <optional>
#include <iomanip>
#include <unordered_set>
#include <fstream>
#include "dataStructAlg.h"

// Functions in Utils
double norm(Point a);
double norm(const Line& a);
double cross(Point a, Point b);											// Cross product
double backMinDistNum(std::vector<double> DistNums);
Point getProjectionPoint(Point basePoint, Line baseLine);				// Calculate the projection point position
Point RotateVec(Point vec, double angle);								// Return the angle after vector rotation || > 0 --> counterclockwise
Point getInsection(Point Vec1, Point p1, Point Vec2, Point p2);			// Given the direction vectors of two line segments and a point on each segment, find the intersection point of the two lines
Point rotate(Point vec, double angle);                                  // Vector rotation,[1] > 0: counterclockwise
double CrossVecs(Point a, Point b);                                     // Cross product of two vectors
void NormalizeVec(Point& NormalVec);                                    // Normalize vector
double pointToLineDist(Point point, Line line1);						// Distance from point to line
double linearDist(Point point1, Point point2);							// Get the linear distance between two points

namespace v2Hash {
	struct PointHash {
		size_t operator()(const Point& pnt)const;
	};
	struct LineHash {
		size_t operator()(const Line& line)const;
	};
}


// Hug
struct PointHash {
	size_t operator()(const Point& data) const {
		return std::hash<double>()(data.x) ^ std::hash<double>()(data.y); //^ std::hash<double>()(data.Pt2.x);
	}
};

bool CheckSegmentAngle(const Line& InputSegment);								 // Check if the routing angle meets the 135° requirement
Point CountLineMidPos(const Point& Point1, const Point& Point2);                 // Return the midpoint of the line connecting two points
Point CountLineMidPos(const Line& BaseSegment);									 // Return the midpoint of a line segment
double CountLineAngle(const Line& BaseSeg);										 // Return the angle of a line (0° - 180°)
Point CountCrossingPoint(Point Vec1, Point p1, Point Vec2, Point p2);            // Find intersection point given two points and their directions
std::vector<Line> PointsToLines(std::vector<Point> points, bool flag);			 // Connect points in order to form lines
std::vector<Point> LinesToPoints(std::vector<Line> lines, bool flag);			 // Lines->Points
Point getCenterOfObstacles(const std::vector<std::vector<Line> >& InputObstacles);                 // Get the center point of a group of obstacles
bool CheckNumInSets(const double& InputNumber, const std::vector<double>& InputNumSets);           // Determine if InputNumber is in the InputNumSets collection
bool CheckNumInSets(const int& InputNumber, const std::vector<int>& InputNumSets);				   // Determine if InputNumber is in the InputNumSets collection
int getPointPosition(const std::vector<Point>& pointsVec, const Point& basePoint);			// Get the position of the current point in the point set
bool isEncloseGraph(std::vector<Line> lines);												// Determine if the input group of lines is an obstacle or a line
bool pointInPointsVec(Point point, std::vector<Point> points);								// Determine if a point is in a point set
void pointsToNewVec(std::vector<Point> points_, std::vector<Point>& newPointsVec);          // Add all points in Points to newPointsVec
std::vector<Line> linkToLine(Point sp_, Point ep_);											// Connect two points directly into a line
Point getMidpoint(Point point1, Point point2);                                              // Find midpoint

