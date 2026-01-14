#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <cmath>

#include "RoutingNode.h"  // Include PathNode and other related definitions

// Grid cell class

class GridCell {
public:
	GridCell(int x, int y, double cellSize)
		: m_x(x), m_y(y), m_cellSize(cellSize) {
		m_bbox[0] = x * cellSize;      // minX
		m_bbox[1] = y * cellSize;      // minY
		m_bbox[2] = (x + 1) * cellSize; // maxX
		m_bbox[3] = (y + 1) * cellSize; // maxY
	}

	// Check if a point is inside the grid
	bool contains(const Point& point) const {
		return point.x >= m_bbox[0] && point.x <= m_bbox[2] &&
			point.y >= m_bbox[1] && point.y <= m_bbox[3];
	}

	// Check if a line segment intersects with the grid
	bool intersects(const Line& line) const {
		// Use Cohen-Sutherland algorithm for line clipping test
		int code1 = computeCohenSutherlandCode(line.Pt1);
		int code2 = computeCohenSutherlandCode(line.Pt2);

		// Completely inside
		if (code1 == 0 && code2 == 0) return true;

		// Completely outside
		if ((code1 & code2) != 0) return false;

		// Need further intersection check
		return true;
	}

	// Check if a polygon shape intersects with the grid (bounding box test)
	bool intersects(const PolyShape* shape) const {
		// First compute the shape's bounding box
		std::vector<double> shape_bbox = computeShapeBBox(shape);

		// Bounding box quick test
		if (shape_bbox[2] < m_bbox[0] || shape_bbox[0] > m_bbox[2] ||
			shape_bbox[3] < m_bbox[1] || shape_bbox[1] > m_bbox[3]) {
			return false;
		}

		// Detailed edge intersection test
		for (const auto& edge : shape->edges) {
			Line line(edge.p1->pos, edge.p2->pos);
			if (intersects(line)) {
				return true;
			}
		}

		return false;
	}

	// Check if a PinPad intersects with the grid
	bool intersects(const PinPad* pad) const {
		// Use PinPad's box for quick test
		if (pad->box[2] < m_bbox[0] || pad->box[0] > m_bbox[2] ||
			pad->box[3] < m_bbox[1] || pad->box[1] > m_bbox[3]) {
			return false;
		}
		// Check all shapes
		for (const auto& [layer, shape] : pad->shapes) {
			if (intersects(&shape)) {
				return true;
			}
		}

		return false;
	}

	// Add shape to grid
	void addPathLines(PathLine* line, PolyShape* shape) {
		m_pathLines.insert(make_pair(line, shape));
	}
	void removePathLines(PathLine* line) {
		m_pathLines.erase(line);
	}
	void clearAllPathLines() {
		m_pathLines.clear();
	}
	// Add PinPad to grid (add all its shapes)
	void addPinPad(PinPad* pad) {
		m_pinPads.insert(pad);
	}
	void removePinPad(PinPad* pad) {
		m_pinPads.erase(pad);
	}

	// Get all shapes in the grid
	const std::unordered_map<PathLine*, PolyShape*>& getPathLines() {
		return m_pathLines;
	}
	const std::unordered_set<PinPad*>& getPinPads() {
		return m_pinPads;
	}
	// Get grid boundary
	const std::vector<double>& getBBox() const {
		return m_bbox;
	}

	int getX() const { return m_x; }
	int getY() const { return m_y; }

private:
	// Compute the bounding box of a shape
	std::vector<double> computeShapeBBox(const PolyShape* shape) const {
		if (shape->edges.empty()) {
			return { 0, 0, 0, 0 };
		}

		double minX = shape->edges[0].p1->pos.x;
		double minY = shape->edges[0].p1->pos.y;
		double maxX = minX;
		double maxY = minY;

		for (const auto& edge : shape->edges) {
			minX = std::min(minX, std::min(edge.p1->pos.x, edge.p2->pos.x));
			minY = std::min(minY, std::min(edge.p1->pos.y, edge.p2->pos.y));
			maxX = std::max(maxX, std::max(edge.p1->pos.x, edge.p2->pos.x));
			maxY = std::max(maxY, std::max(edge.p1->pos.y, edge.p2->pos.y));
		}

		return { minX, minY, maxX, maxY };
	}

	// Cohen-Sutherland algorithm encoding
	int computeCohenSutherlandCode(const Point& point) const {
		int code = 0;
		if (point.x < m_bbox[0]) code |= 1;   // Left
		if (point.x > m_bbox[2]) code |= 2;   // Right
		if (point.y < m_bbox[1]) code |= 4;   // Bottom
		if (point.y > m_bbox[3]) code |= 8;   // Top
		return code;
	}

private:
	int m_x, m_y;
	double m_cellSize;
	std::vector<double> m_bbox = { 0, 0, 0, 0 }; // minX, minY, maxX, maxY
	std::unordered_map<PathLine*, PolyShape*> m_pathLines;
	std::unordered_set<PinPad*> m_pinPads;
};

// Grid manager class
class GridManager {
public:
	GridManager(double minX, double minY,double maxX, double maxY,double cellSize)
		: m_minX(minX), m_minY(minY),
		m_maxX(maxX), m_maxY(maxY),
		m_cellSize(cellSize)
	{
		m_numCellsX = static_cast<int>(
			std::ceil((m_maxX - m_minX) / m_cellSize));
		m_numCellsY = static_cast<int>(
			std::ceil((m_maxY - m_minY) / m_cellSize));

		m_gridCells.resize(m_numCellsX * m_numCellsY);
		for (int y = 0; y < m_numCellsY; ++y) {
			for (int x = 0; x < m_numCellsX; ++x) {
				m_gridCells[y * m_numCellsX + x] =
					std::make_unique<GridCell>(x, y, m_cellSize);
			}
		}
	}

	// Basic access
	GridCell* getCell(int x, int y) const {
		if (x < 0 || x >= m_numCellsX ||
			y < 0 || y >= m_numCellsY)
			return nullptr;
		return m_gridCells[y * m_numCellsX + x].get();
	}

	GridCell* getCellAtPoint(const Point& p) const {
		return getCell(getCellX(p.x), getCellY(p.y));
	}

	const std::vector<std::unique_ptr<GridCell>>& getAllCells() const {
		return m_gridCells;
	}

	void addPinPad(PinPad* pad) const {
		if (!pad) return;

		int sx = getCellX(pad->box[0]);
		int sy = getCellY(pad->box[1]);
		int ex = getCellX(pad->box[2] - MapMinValue);
		int ey = getCellY(pad->box[3] - MapMinValue);

		for (int y = sy; y <= ey; ++y) {
			for (int x = sx; x <= ex; ++x) {
				GridCell* c = getCell(x, y);
				if (c && c->intersects(pad))
					c->addPinPad(pad);
			}
		}
	}
	void removePinPad(PinPad* pad) const {
		if (!pad) return;
		int sx = getCellX(pad->box[0]);
		int sy = getCellY(pad->box[1]);
		int ex = getCellX(pad->box[2] - MapMinValue);
		int ey = getCellY(pad->box[3] - MapMinValue);
		for (int y = sy; y <= ey; ++y) {
			for (int x = sx; x <= ex; ++x) {
				GridCell* c = getCell(x, y);
				if (!c) continue;

				// ⚠️ Do we need intersects?
				// → No, just erase directly
				c->removePinPad(pad);
			}
		}
	}

	void addShapeLines(PolyShape* shape) const {
		if (!shape) return;
		for (PathLine& e : shape->edges) {
			Line l(e.p1->pos, e.p2->pos);
			std::vector<GridCell*> cells;
			getCellsAlongLine(l, cells);
			for (GridCell* c : cells)
				c->addPathLines(&e, shape);
		}
	}

	void removeOnePath(PolyShape* shape) const {
		if (!shape) return;

		for (PathLine& e : shape->edges) {
			Line l(e.p1->pos, e.p2->pos);
			std::vector<GridCell*> cells;
			getCellsAlongLine(l, cells);
			for (GridCell* c : cells)
				c->removePathLines(&e);
		}

		// Final cleanup
		for (const auto& cp : m_gridCells) {
			auto& mp = cp->getPathLines();
			std::vector<PathLine*> dead;
			for (const auto& kv : mp)
				if (kv.second == shape)
					dead.push_back(kv.first);
			for (auto* k : dead)
				cp->removePathLines(k);
		}
	}

	void clearAllPathLines() const {
		for (const auto& c : m_gridCells)
			c->clearAllPathLines();
	}

	// Box query
	
	void getCellsInBox(const std::vector<double>& b,std::vector<GridCell*>& cells) const{
		if (b.size() < 4) return;
		cells.clear();
		int sx = getCellX(b[0]);
		int sy = getCellY(b[1]);
		int ex = getCellX(b[2] - MapMinValue);
		int ey = getCellY(b[3] - MapMinValue);

		for (int y = sy; y <= ey; ++y)
			for (int x = sx; x <= ex; ++x)
				if (GridCell* c = getCell(x, y))
					cells.push_back(c);
	}
	
	void getCellsInBox1(const vector<double>& bounds, vector<GridCell*>& cells) const {
		if (bounds.size() < 4) {
			return;
		}
		cells.clear();
		const double& minX = bounds[0];
		const double& minY = bounds[1];
		const double& maxX = bounds[2];
		const double& maxY = bounds[3];

		if (minX > maxX || minY > maxY) {
			return;
		}
		// Calculate grid range covered by bounding box (consider left-open right-closed)
		int startX = getCellX(minX);
		int startY = getCellY(minY);
		int endX = getCellX(maxX);
		int endY = getCellY(maxY);

		// Handle boundary case: if max.y is exactly on grid boundary, include previous grid
		double remainderY = std::fmod(maxY - m_minY, m_cellSize);
		if (std::abs(remainderY) < MapMinValue) {
			endY = std::max(0, endY - 1);
		}

		// Ensure indices are within valid range
		startX = std::max(0, startX);
		startY = std::max(0, startY);
		endX = std::min(m_numCellsX - 1, endX);
		endY = std::min(m_numCellsY - 1, endY);

		// Ensure indices are within valid range
		startX = std::max(0, startX);
		startY = std::max(0, startY);
		endX = std::min(m_numCellsX - 1, endX);
		endY = std::min(m_numCellsY - 1, endY);

		// Iterate through all involved grid cells
		for (int y = startY; y <= endY; ++y) {
			for (int x = startX; x <= endX; ++x) {
				if (GridCell* cell = getCell(x, y)) {
					cells.emplace_back(cell);
				}
			}
		}
	}

	// Line segment crosses all cells
	void getCellsAlongLine(const Line& line,
		std::vector<GridCell*>& cells) const
	{
		cells.clear();

		const Point& p0 = line.Pt1;
		const Point& p1 = line.Pt2;

		double dx = p1.x - p0.x;
		double dy = p1.y - p0.y;

		int ix = getCellX(p0.x);
		int iy = getCellY(p0.y);
		int ixEnd = getCellX(p1.x);
		int iyEnd = getCellY(p1.y);

		int stepX = (dx > MapMinValue) ? 1 :
			(dx < -MapMinValue) ? -1 : 0;
		int stepY = (dy > MapMinValue) ? 1 :
			(dy < -MapMinValue) ? -1 : 0;

		GridCell* startCell = getCell(ix, iy);
		if (startCell)
			cells.push_back(startCell);

		if (stepX == 0 && stepY == 0)
			return;

		double tMaxX, tMaxY;
		double tDeltaX, tDeltaY;

		if (stepX != 0) {
			double nextX = m_minX +
				(stepX > 0 ? ix + 1 : ix) * m_cellSize;
			tMaxX = (nextX - p0.x) / dx;
			tDeltaX = m_cellSize / std::abs(dx);
		}
		else {
			tMaxX = tDeltaX =
				std::numeric_limits<double>::infinity();
		}

		if (stepY != 0) {
			double nextY = m_minY +
				(stepY > 0 ? iy + 1 : iy) * m_cellSize;
			tMaxY = (nextY - p0.y) / dy;
			tDeltaY = m_cellSize / std::abs(dy);
		}
		else {
			tMaxY = tDeltaY =
				std::numeric_limits<double>::infinity();
		}

		while (!(ix == ixEnd && iy == iyEnd)) {
			if (tMaxX < tMaxY) {
				ix += stepX;
				tMaxX += tDeltaX;
			}
			else {
				iy += stepY;
				tMaxY += tDeltaY;
			}

			if (ix < 0 || ix >= m_numCellsX ||
				iy < 0 || iy >= m_numCellsY)
				break;

			GridCell* c = getCell(ix, iy);
			if (c && cells.back() != c)
				cells.push_back(c);
		}
	}
	void getCellsAlongLine2(const Line& line,const double& width,std::vector<GridCell*>& cells) const{
		cells.clear();
		if (width <= MapMinValue) {
			// Degenerate to no width
			getCellsAlongLine(line, cells);
			return;
		}
		Point p1 = line.Pt1;
		Point p2 = line.Pt2;

		Point dir = (p2 - p1).normalizeVec();
		if (dir.vecLength() < MapMinValue) {
			if (GridCell* c = getCellAtPoint(p1))
				cells.push_back(c);
			return;
		}

		Point normal = dir;
		normal.rotate90();   // Unit normal vector

		double halfW = 0.5 * width;

		// Extend endpoints
		p1 = p1 - dir * halfW;
		p2 = p2 + dir * halfW;

		Line line1(p1 + normal * halfW, p2 + normal * halfW);
		Line line2(p1 - normal * halfW, p2 - normal * halfW);

		std::unordered_set<GridCell*> visited;
		std::vector<GridCell*> tmp;
		auto collect = [&](const Line& l) {
			tmp.clear();
			getCellsAlongLine(l, tmp);
			for (GridCell* c : tmp) {
				if (visited.insert(c).second) {
					cells.push_back(c);
				}
			}
			};
		collect(line1);
		collect(line2);
	}

private:
	// Left-closed right-open, bottom-closed top-open
	int getCellX(double x) const {
		double gx = (x - m_minX) / m_cellSize;
		int ix = static_cast<int>(std::floor(gx + MapMinValue));
		return std::max(0, std::min(m_numCellsX - 1, ix));
	}

	int getCellY(double y) const {
		double gy = (y - m_minY) / m_cellSize;
		int iy = static_cast<int>(std::floor(gy + MapMinValue));
		return std::max(0, std::min(m_numCellsY - 1, iy));
	}

private:
	double m_minX, m_minY, m_maxX, m_maxY;
	double m_cellSize;
	int m_numCellsX, m_numCellsY;
	std::vector<std::unique_ptr<GridCell>> m_gridCells;
};