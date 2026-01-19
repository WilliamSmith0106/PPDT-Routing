# include "utils.h"

using namespace std;

double norm(Point a) {    //Vector magnitude
	return sqrt(a.x * a.x + a.y * a.y);
}

double norm(const Line& a) {
	Point Sp = a.Pt1;
	Point Ep = a.Pt2;
	return norm(Ep - Sp);
}

double cross(Point a, Point b) {    //Cross product
	return a.x * b.y - a.y * b.x;
}

//Return the smallest value from a set of distances (select the smallest from a bunch of numbers)
double backMinDistNum(vector<double> DistNums) {
	double minDistance;
	double tempDistance;
	//Init
	minDistance = DistNums[0];
	for (int i = 1; i < DistNums.size(); ++i) {
		tempDistance = DistNums[i];
		if (tempDistance < minDistance) {
			minDistance = tempDistance;
		}
	}

	return minDistance;
}

//Find the position of the projection point
Point getProjectionPoint(Point basePoint, Line baseLine) {
	Point baseLineVec, baseLineVecticalVec;

	baseLineVec = baseLine.Pt2 - baseLine.Pt1;
	baseLineVecticalVec = rotate(baseLineVec, 90);

	return getInsection(baseLineVecticalVec, basePoint, baseLineVec, baseLine.Pt1);

}

Point RotateVec(Point vec, double angle) {
	double rad = angle * PI / 180.0;
	Point rotate_vec = { vec.x * cos(rad) - vec.y * sin(rad), vec.y * cos(rad) + vec.x * sin(rad) };
	return rotate_vec;
}

Point getInsection(Point Vec1, Point p1, Point Vec2, Point p2) {
	Point P1P2 = p2 - p1;
	double t = cross(P1P2, Vec2) / cross(Vec1, Vec2);
	Point outPoint = p1 + Vec1 * t;

	return outPoint;
}
///Vector rotation, [1] > 0: counterclockwise
Point rotate(Point vec, double angle) {
	double rad = angle * PI / 180.0;
	Point rotate_vec = { vec.x * cos(rad) - vec.y * sin(rad), vec.y * cos(rad) + vec.x * sin(rad) };
	return rotate_vec;
}

double CrossVecs(Point a, Point b) {    //Cross product
	return a.x * b.y - a.y * b.x;
}

void NormalizeVec(Point& NormalVec) {
	double VecLength = sqrt(NormalVec.x * NormalVec.x + NormalVec.y * NormalVec.y);
	if (VecLength != 0 && VecLength != 1) {
		NormalVec = NormalVec / VecLength;
	}
}

//Distance from point to line
double pointToLineDist(Point point, Line line1) {
	Point vecLine;
	Point vecNormalLine;
	Point crossingPoint;
	vecLine = line1.Pt2 - line1.Pt1;
	vecNormalLine = rotate(vecLine, 90);
	crossingPoint = getInsection(vecLine, line1.Pt1, vecNormalLine, point);
	double dist = linearDist(point, crossingPoint);

	return dist;
}


//Get the straight-line distance between two points
double linearDist(Point point1, Point point2) {
	double distance;
	distance = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return distance;
}

size_t v2Hash::PointHash::operator()(const Point& pnt) const
{
	return std::hash<double>()(round(pnt.x * 1e6)) ^ std::hash<double>()(round(pnt.y * 1e6));
}

size_t v2Hash::LineHash::operator()(const Line& line) const
{
	return PointHash()(line.Pt1) ^ PointHash()(line.Pt2);
}



// Hug
typedef std::vector<Line> LineSet;

bool CheckSegmentAngle(const Line& InputSegment) {
	double Alpha = CountLineAngle(InputSegment);
	if (
		abs(Alpha) <= MapMinValue ||
		abs(Alpha - 45) <= MapMinValue ||
		abs(Alpha - 90) <= MapMinValue ||
		abs(Alpha - 135) <= MapMinValue ||
		abs(Alpha - 180) <= MapMinValue
		) {

		return true;
	}
	else {
		return false;
	}
}

Point CountLineMidPos(const Point& Point1, const Point& Point2) {
	Point MidPoint;
	MidPoint.x = (Point1.x + Point2.x) / 2;
	MidPoint.y = (Point1.y + Point2.y) / 2;

	return MidPoint;
}

Point CountLineMidPos(const Line& BaseSegment) {
	return CountLineMidPos(BaseSegment.Pt1, BaseSegment.Pt2);
}

double CountLineAngle(const Line& BaseSeg) {
	Point StartPoint = BaseSeg.Pt1;
	Point EndPoint = BaseSeg.Pt2;

	Point SegmentVecDir = EndPoint - StartPoint;
	//SegmentVecDir.normalizeVec();
	NormalizeVec(SegmentVecDir);
	Point InferenceVec = Point(1, 0);
	double CosAlphaValue = SegmentVecDir * InferenceVec;
	double Alpha = acos(CosAlphaValue);  // 0 - Pi
	Alpha = Alpha * 180.0 / PI;
	return Alpha;
}


Point CountCrossingPoint(Point Vec1, Point p1, Point Vec2, Point p2) {
	Point P1P2 = p2 - p1;
	double t = CrossVecs(P1P2, Vec2) / CrossVecs(Vec1, Vec2);
	Point outPoint = p1 + Vec1 * t;

	return outPoint;
}


Point getCenterOfObstacles(const vector<vector<Line> >& InputObstacles) {
	Point obsCenterPoint;
	int countNum = 0;
	vector<Point> ItemPointsVec;
	for (auto& ItemObs : InputObstacles) {
		ItemPointsVec = LinesToPoints(ItemObs, true);
		for (int i = 0; i < ItemPointsVec.size(); ++i) {
			obsCenterPoint = obsCenterPoint + ItemPointsVec[i];
			countNum++;
		}
	}

	obsCenterPoint = obsCenterPoint / countNum;

	return obsCenterPoint;

}

// Points[] -> Line
//flag:[1] false(0): Currently converting points->Line; [2] true(1): Need to close
vector<Line> PointsToLines(vector<Point> points, bool flag) {
	vector<Line> vecLines;
	if (points.size() <= 1) {   // 2023/11/9 update
		return vecLines;
	}

	//vector<Line> vecLines;
	Point startP, endP;
	Line tempLine;
	for (int i = 0; i < points.size() - 1; ++i) {
		startP = points[i];
		endP = points[i + 1];
		tempLine.setPt1(startP);
		tempLine.setPt2(endP);
		vecLines.emplace_back(tempLine);
	}
	if (flag) {
		endP = points[0];
		startP = points[points.size() - 1];
		tempLine.setPt1(startP);
		tempLine.setPt2(endP);
		vecLines.emplace_back(tempLine);
	}
	return vecLines;
}

/*
* brief: Lines -> points
* @flag: Whether the current line is closed flag = true: Closed {does not consider the end point of the last line}
*/
vector<Point> LinesToPoints(vector<Line> lines, bool flag) {
	vector<Point> outPoints;
	//Bug : Exit   // 2023/11/9 update
	if (lines.empty()) {
		return outPoints;
	}

	if (!flag) { // Non-closed line group
		outPoints.emplace_back(lines[0].Pt1);
	}

	for (int i = 0; i < lines.size(); ++i) {
		outPoints.emplace_back(lines[i].Pt2);
		//cout << "Point:" << lines[i].Pt2 << endl;
	}

	return outPoints;
}

bool CheckNumInSets(const double& InputNumber, const vector<double>& InputNumSets) {
	if (InputNumSets.empty()) {
		return false;
	}

	for (double ItemNum : InputNumSets) {
		if (InputNumber == ItemNum) {
			return true;
		}
	}

	return false;

}

bool CheckNumInSets(const int& InputNumber, const vector<int>& InputNumSets) {
	if (InputNumSets.empty()) {
		return false;
	}

	for (double ItemNum : InputNumSets) {
		if (InputNumber == ItemNum) {
			return true;
		}
	}

	return false;

}

// Get the position of the current point in the point set
int getPointPosition(const vector<Point>& pointsVec, const Point& basePoint) {
	int index = -1;
	//Point tempPoint = basePoint;
	for (int i = 0; i < pointsVec.size(); ++i) {
		if (basePoint == pointsVec[i]) {
			index = i;
			break;
		}
	}

	return index;
}


//Determine if the input group of lines is an obstacle or a line
bool isEncloseGraph(vector<Line> lines) {
	bool isPolygon;
	bool newPoint;
	int linesSize;  //number of lines in vector
	Line tempLine;
	vector<Point> vertexPoint;  // Vertices
	//Init
	linesSize = lines.size();

	for (int i = 0; i < lines.size(); ++i) {
		tempLine = lines[i];
		newPoint = pointInPointsVec(tempLine.Pt1, vertexPoint);
		if (!newPoint) {
			vertexPoint.emplace_back(tempLine.Pt1);
		}
		newPoint = pointInPointsVec(tempLine.Pt2, vertexPoint);
		if (!newPoint) {
			vertexPoint.emplace_back(tempLine.Pt2);
		}
	}
	isPolygon = (vertexPoint.size() == linesSize) ? true : false;
	return isPolygon;
}

//Determine if a point is in a point set
bool pointInPointsVec(Point point, vector<Point> points) {
	bool outResult = false;
	if (points.empty()) {
		return false;
	}

	for (int i = 0; i < points.size(); ++i) {
		if (point == points[i]) {
			outResult = true;
			break;
		}
	}
	return outResult;
}

//Add all points in Points to newPointsVec
void pointsToNewVec(vector<Point> points_, vector<Point>& newPointsVec) {
	//Going!
	for (int i = 0; i < points_.size(); ++i) {
		bool NullPoint = pointInPointsVec(points_[i], newPointsVec);
		if (!NullPoint) {
			newPointsVec.emplace_back(points_[i]);
		}
	}
}


//Connect two points directly into a line
vector<Line> linkToLine(Point sp_, Point ep_) {
	vector<Line> outPrintLines;
	Line line;
	line.setPt1(sp_);
	line.setPt2(ep_);
	outPrintLines.emplace_back(line);

	return outPrintLines;
}

//  Find the midpoint of two points
Point getMidpoint(Point point1, Point point2) {
	Point middlePoint;
	middlePoint.x = (point1.x + point2.x) / 2;
	middlePoint.y = (point1.y + point2.y) / 2;
	return middlePoint;
}

//  Find the midpoint of two points
Point getMidpoint(const Line& line) {
	Point middlePoint;
	middlePoint = getMidpoint(line.Pt1, line.Pt2);
	return middlePoint;
}