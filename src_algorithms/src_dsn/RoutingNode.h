#pragma once
#include <queue>
#include <unordered_set>
#include <optional>

#include "../src_basics/dataStructAlg.h"

using namespace std;
// Forward declaration
struct ViaInfo;
struct NetInfo;
struct PathNode;
struct PathLine;
struct PolyShape;
struct PinPad;
struct PathTree;

struct ViaInfo {
public:
	ViaInfo() : m_radius(0) {};
	ViaInfo(double radius, std::vector<int>& layers)
		: m_radius(radius), m_layers(layers) {
	}
	~ViaInfo() {};

public:
	double m_radius;
	vector<int> m_layers;
};
struct NetInfo {
	string viaName;
	double width = 1;			// Line width (optional)
	double clearance = 0.4;		// Clearance (optional)

	NetInfo() {};
	NetInfo(const string& viaName, const double& width, const double& clear)
		: viaName(viaName), width(width), clearance(clear) {
	}
};
struct PathNode {
	Point pos;
	Point direction;
	PathNode* prev;   // Previous node
	PathNode* next;   // Next node
	string netName;	  // 1. Intermediate path node, represents the path's net; 2. Polygon vertex, represents the pad's net
	// Only for polygon vertices: which shape's planning point this node belongs to; if it's a pad, then nullptr
	PolyShape* shape = nullptr;
	int layer;
	double width = 0;

	PathNode() : prev(nullptr), next(nullptr) {};
	PathNode(const Point& position, PolyShape* shape, const string& netName, const int& layer = 0)
		: pos(position), shape(shape), netName(netName), prev(nullptr), next(nullptr), layer(layer) {
	}
	PathNode(const PathNode* const other) {
		pos = other->pos;
		direction = other->direction;
		prev = nullptr;
		next = nullptr;
		netName = other->netName;
		shape = other->shape;
		layer = other->layer;
	}

	// Insert new node after this node
	void insertAfter(PathNode* newNode) {
		if (!newNode) return;

		newNode->prev = this;
		newNode->next = this->next;

		if (this->next) {
			this->next->prev = newNode;
		}
		this->next = newNode;
	}

	// Insert new node before this node
	void insertBefore(PathNode* newNode) {
		if (!newNode) return;
		newNode->prev = this->prev;
		newNode->next = this;

		if (this->prev) {
			this->prev->next = newNode;
		}
		this->prev = newNode;
	}

	// Remove current node from list
	void removeFromList() {
		if (prev) prev->next = next;
		if (next) next->prev = prev;
		prev = nullptr;
		next = nullptr;
	}
	PathNode* deleteCurruntNode() {
		PathNode* nextNode = this->next;
		removeFromList();
		delete this;
		return nextNode;
	}
	void deleteRelatedNodes() {
		if (!this) return;
		// Disconnect circular connection
		if (prev) prev->next = nullptr;
		if (next) next->prev = nullptr;
		PathNode* prevChain = this->prev;
		PathNode* nextChain = this->next;
		delete this;

		// Safely delete forward chain (prevent circular)
		PathNode* currentPrev = prevChain;
		unordered_set<PathNode*> deletedNodes;
		while (currentPrev && deletedNodes.find(currentPrev) == deletedNodes.end()) {
			PathNode* temp = currentPrev->prev;
			deletedNodes.insert(currentPrev);
			delete currentPrev;
			currentPrev = temp;
		}

		// Safely delete backward chain
		PathNode* currentNext = nextChain;
		while (currentNext && deletedNodes.find(currentNext) == deletedNodes.end()) {
			PathNode* temp = currentNext->next;
			deletedNodes.insert(currentNext);
			delete currentNext;
			currentNext = temp;
		}
	}
};
struct PathLine {	// Deep copy not allowed
	PathLine() : p1(nullptr), p2(nullptr), layer(0), width(0) {};
	PathLine(PathNode* p1, PathNode* p2, int layer, const double& width)
		: p1(p1), p2(p2), layer(layer), width(width) {
	};
	PathNode* p1;
	PathNode* p2;
	double width;
	int layer;
};
struct PolyShape {	// Polygon, circle, path
	vector<PathLine> edges;
	string shapeName;		// For pad polygons it's pinName, for paths (isLine is true) it's netName
	int layer;
	bool isLine;
	double clearance = 0;
	PolyShape() : layer(0), isLine(false) {};
	PolyShape(const vector<PathLine>& edges, const int& layer, const bool isLine, const double& clear, const string shapeName)
		: edges(edges), layer(layer), isLine(isLine), clearance(clear), shapeName(shapeName) {
	};
	void deleteNodes() {
		if (edges.empty()) return;
		edges[0].p1->deleteRelatedNodes();
		edges.clear();
	}
	PolyShape copy() {
		PolyShape result;
		result.shapeName = shapeName;
		result.layer = layer;
		result.isLine = isLine;
		result.clearance = clearance;
		if (edges.empty()) return result;
		PathNode* n1 = new PathNode(edges[0].p1);
		for (int i = 0; i < edges.size() - 1; ++i) {
			const PathLine& line = edges[i];
			PathNode* n2 = new PathNode(line.p2);
			result.edges.emplace_back(PathLine(n1, n2, line.layer, line.width));
			n1->insertAfter(n2);
			n1 = n2;
		}
		if (isLine) {
			const PathLine& line = edges.back();
			PathNode* n2 = new PathNode(line.p2);
			result.edges.emplace_back(PathLine(n1, n2, line.layer, line.width));
			n1->insertAfter(n2);
		}
		else {
			const PathLine& line = edges.back();
			PathNode* n2 = result.edges[0].p1;
			result.edges.emplace_back(PathLine(n1, n2, line.layer, line.width));
			n1->insertAfter(n2);
		}
		return result;
	}
	void setDirection() {
		if (edges.empty()) {
			cout << "Error: no node to setDirection!" << endl;
			return;
		}
		if (isLine) {
			PathNode* cur = edges[0].p1;
			cur->direction = Point(0, 0);// Direction of the first node
			cur = cur->next;
			while (cur && cur->next) {
				bool prevSameLayer = cur->layer == cur->prev->layer;
				bool nextSameLayer = cur->layer == cur->next->layer;
				if (prevSameLayer && nextSameLayer) {
					// Same layer nodes, calculated using vectors
					Point vec1 = (cur->pos - cur->prev->pos).normalizeVec();
					Point vec2 = (cur->pos - cur->next->pos).normalizeVec();
					Point direction = (vec1 + vec2).normalizeVec();
					double coefficient = (vec1 - vec2).vecLength();
					if (coefficient > MapMinValue)
						coefficient = 2 / coefficient;
					else
						coefficient = 2.62;		// Default to 45-degree corner, take 1/sin(22.5)
					cur->direction = direction * coefficient;
				}
				else if (prevSameLayer) {
					cur->direction = (cur->pos - cur->prev->pos).normalizeVec();
				}
				else if (nextSameLayer) {
					cur->direction = (cur->pos - cur->next->pos).normalizeVec();
				}
				else {
					cur->direction = Point(0, 0);
					cout << "Error in class PolyShape function setDirection!" << endl;
				}
				cur = cur->next;
			}
			if (cur)
				cur->direction = Point(0, 0);	// Direction of the last node
		}
		else {
			PathNode* head = edges[0].p1;
			PathNode* cur = head;
			do {
				Point vec1 = (cur->pos - cur->prev->pos).normalizeVec();
				Point vec2 = (cur->pos - cur->next->pos).normalizeVec();
				Point direction = (vec1 + vec2).normalizeVec();
				double coefficient = (vec1 - vec2).vecLength();
				if (coefficient > MapMinValue)
					coefficient = 2 / coefficient;
				else
					coefficient = 2.62;		// Default to 45-degree corner, take 1/sin(22.5)
				cur->direction = direction * coefficient;
				cur = cur->next;
			} while (cur != head);
		}
	}

	PolyShape& operator=(const PolyShape&) = delete;
	bool operator==(const PolyShape& other) const {
		return this == &other;
	}
	size_t hash() const {
		return std::hash<const PolyShape*>{}(this);
	}
};
struct PinPad {
	Point pos;
	string shapeName;
	string netName;
	unordered_map<int, PolyShape> shapes;	// Layer, shape
	double r = 0;	// Radius, only for circles
	vector<double> box = { 0,0,0,0 };
	PinPad() : r(0) {};
	PinPad(const Point& pos, const string& shapeName, const string& netName = "", const double& clear = 0.0)
		: pos(pos), shapeName(shapeName), netName(netName), clearance(clear) {
	};
	~PinPad() {
		for (auto& [layer, shape] : shapes) {
			shape.deleteNodes();
		}
	}
	void setNetName(const string& name) {
		netName = name;
	}
	void addShape(const int& layer, const vector<Line>& edgeLines) {
		if (edgeLines.empty()) return;
		// 1.Set bounding box
		if (box[0] == 0 && box[0] == box[2]) {
			box[0] = box[2] = edgeLines[0].Pt1.x;
			box[1] = box[3] = edgeLines[0].Pt1.y;
			for (const auto& line : edgeLines) {
				if (line.Pt1.x < box[0])
					box[0] = line.Pt1.x;
				else if (line.Pt1.x > box[2])
					box[2] = line.Pt1.x;
				if (line.Pt1.y < box[1])
					box[1] = line.Pt1.y;
				else if (line.Pt1.y > box[3])
					box[3] = line.Pt1.y;

				if (line.Pt2.x < box[0])
					box[0] = line.Pt2.x;
				else if (line.Pt2.x > box[2])
					box[2] = line.Pt2.x;
				if (line.Pt2.y < box[1])
					box[1] = line.Pt2.y;
				else if (line.Pt2.y > box[3])
					box[3] = line.Pt2.y;
			}
		}
		// 2.Generate circular linked list, add edges
		shapes.insert(make_pair(layer, PolyShape(vector<PathLine>(), layer, false, clearance, shapeName)));
		PolyShape* shape = &shapes[layer];
		vector<PathLine>& edges = shape->edges;
		//// Option 1: Original polygon
		//PathNode* p1 = new PathNode(edgeLines[0].Pt1, shape, netName);
		//for (size_t i = 1; i < edgeLines.size(); ++i) {
		//	auto& line = edgeLines[i];
		//	PathNode* p2 = new PathNode(line.Pt1, shape, netName);
		//	p1->insertAfter(p2);
		//	edges.emplace_back(PathLine(p1, p2, layer, width));
		//	p1 = p2;
		//}
		//edges.back().p2->next = edges[0].p1;
		//edges[0].p1->prev = edges.back().p2;
		//edges.emplace_back(PathLine(edges.back().p2, edges[0].p1, layer, width));
		// Option 2: Take bounding box
		PathNode* p1 = new PathNode(Point(box[0], box[1]), shape, netName);
		PathNode* p2 = new PathNode(Point(box[0], box[3]), shape, netName);
		PathNode* p3 = new PathNode(Point(box[2], box[3]), shape, netName);
		PathNode* p4 = new PathNode(Point(box[2], box[1]), shape, netName);
		p1->insertAfter(p2);
		p2->insertAfter(p3);
		p3->insertAfter(p4);
		p4->next = p1;
		p1->prev = p4;
		edges.emplace_back(PathLine(p1, p2, layer, 0));
		edges.emplace_back(PathLine(p2, p3, layer, 0));
		edges.emplace_back(PathLine(p3, p4, layer, 0));
		edges.emplace_back(PathLine(p4, p1, layer, 0));
		// 3.Calculate direction for each node
		shape->setDirection();
	}
	void addShape(const int& layer, const double& radius, const Point& p_move) {
		// 1.Circle related data
		r = radius;
		Point position = pos + p_move;
		shapes.insert(make_pair(layer, PolyShape(vector<PathLine>(), layer, false, clearance, shapeName)));
		PolyShape* shape = &shapes[layer];
		vector<PathLine>& edges = shape->edges;
		// 2.Eight vertices of regular octagon
		double d = radius * tan(PI / 8);
		PathNode* p1 = new PathNode(Point(position.x + radius, position.y + d), shape, netName);
		PathNode* p2 = new PathNode(Point(position.x + radius, position.y - d), shape, netName);
		PathNode* p3 = new PathNode(Point(position.x + d, position.y - radius), shape, netName);
		PathNode* p4 = new PathNode(Point(position.x - d, position.y - radius), shape, netName);
		PathNode* p5 = new PathNode(Point(position.x - radius, position.y - d), shape, netName);
		PathNode* p6 = new PathNode(Point(position.x - radius, position.y + d), shape, netName);
		PathNode* p7 = new PathNode(Point(position.x - d, position.y + radius), shape, netName);
		PathNode* p8 = new PathNode(Point(position.x + d, position.y + radius), shape, netName);
		// 3.Generate circular linked list
		p1->insertAfter(p2);
		p2->insertAfter(p3);
		p3->insertAfter(p4);
		p4->insertAfter(p5);
		p5->insertAfter(p6);
		p6->insertAfter(p7);
		p7->insertAfter(p8);
		p8->next = p1;
		p1->prev = p8;
		// 4.Add edges
		edges.emplace_back(PathLine(p1, p2, layer, 0));
		edges.emplace_back(PathLine(p2, p3, layer, 0));
		edges.emplace_back(PathLine(p3, p4, layer, 0));
		edges.emplace_back(PathLine(p4, p5, layer, 0));
		edges.emplace_back(PathLine(p5, p6, layer, 0));
		edges.emplace_back(PathLine(p6, p7, layer, 0));
		edges.emplace_back(PathLine(p7, p8, layer, 0));
		edges.emplace_back(PathLine(p8, p1, layer, 0));
		// 4.Calculate direction for each node
		shape->setDirection();
		// 5.Set bounding box
		if (box[0] == 0 && box[0] == box[2]) {
			box[0] = position.x - radius;
			box[1] = position.y - radius;
			box[2] = position.x + radius;
			box[3] = position.y + radius;
		}
	}

	PinPad& operator=(const PinPad&) = delete;
	bool operator==(const PinPad& other) const {
		return this == &other;
	}
	size_t hash() const {
		return std::hash<const PinPad*>{}(this);
	}
private:
	double clearance = 0;		// Polygon spacing
};
struct PathTree {
	PathTree() : layer(0), netName("") {};
	PathTree(const Point& position, const int& layer, string netName, PathNode* pNode)
		: pos(position), layer(layer), netName(netName), pNode(pNode) {
	};
	Point pos;
	int layer;
	double g = 0;  // Actual cost from start to current node
	double h = 0;  // Estimated cost from current node to end
	double f = 0;  // Total cost f = g + h
	PathNode* pNode = nullptr;	// Only for planning points, through pNode you can access which shape this planning point belongs to
	string netName;

	PathTree* parent = nullptr;
	unordered_set<PathTree*> children;
	void addChild(PathTree* child, const double& stepG, const double& estimateH) {
		// Delete original relationships (if any) and set connections
		if (child->parent)
			child->parent->children.erase(child);
		child->parent = this;
		children.insert(child);
		// Set child node's fgh values: set g based on parent node, calculate h using end point if available, keep h unchanged if not, update f
		child->g = g + stepG;
		child->h = estimateH;
		child->f = child->g + child->h;
	}
	void updateChild(PathTree* child) {
		if (!child) return;
		double new_g = this->g + pos.distanceTo(child->pos);
		if (new_g < child->g) {
			child->g = new_g;
			child->f = child->g + child->h;
			child->parent->children.erase(child);
			child->parent = this;
			this->children.insert(child);
		}
	}
	void setPos(const Point& position, PathTree* node_end) {
		// Modify current node coordinates
		pos = position;
		if (parent) {
			g = parent->g + pos.distanceTo(parent->pos);
		}
		h = pos.distanceTo(node_end->pos);
		updateTreefgh(20);
	}
	void removeChild(PathTree* child) {
		if (!child) return;
		// Recursively delete child nodes
		children.erase(child);
		child->parent = nullptr;
		deleteSubtree(child);
	}
	void remove() {
		// Delete current node and all its child nodes
		if (parent)
			parent->removeChild(this);
		else
			deleteSubtree(this);
	}
	void copyIn(const PathTree& node) {
		pos = node.pos;
		layer = node.layer;
		g = node.g;
		h = node.h;
		f = node.f;
		pNode = node.pNode;
		netName = node.netName;
		parent = node.parent;
		children = node.children;
	}
	void updateTreefgh(double viaCost) {
		queue<PathTree*> nodes;
		nodes.push(this);
		while (!nodes.empty()) {
			PathTree* current = nodes.front();
			nodes.pop();
			current->f = current->g + current->h;
			for (PathTree* child : current->children) {
				if (child) {
					child->g = current->g + current->pos.distanceTo(child->pos);
					if (layer != child->layer)
						child->g += viaCost;
					child->f = child->g + child->h;
					nodes.push(child);
				}
			}
		}
	}
	bool operator==(const PathTree& other) const {
		return this == &other;
	}
	size_t hash() const {
		return std::hash<const PathTree*>{}(this);
	}
private:
	void deleteSubtree(PathTree* node) {	// Recursive function
		if (!node) return;
		for (auto it = node->children.begin(); it != node->children.end(); ) {
			PathTree* child = *it;
			it = node->children.erase(it);  // First remove from the collection
			deleteSubtree(child);           // Recursively delete
		}
		delete node;
	}
};
struct ComparePathTreePtr {
	bool operator()(const PathTree* a, const PathTree* b) const {
		if (std::abs(a->f - b->f) > MapMinValue) {
			return a->f > b->f;
		}
		if (std::abs(a->pos.x - b->pos.x) > MapMinValue) {
			return a->pos.x > b->pos.x;
		}
		if (std::abs(a->pos.y - b->pos.y) > MapMinValue) {
			return a->pos.y > b->pos.y;
		}
		return a > b;
	}
};

namespace std {
	template<>
	struct hash<PolyShape> {
		size_t operator()(const PolyShape& shape) const {
			return shape.hash();
		}
	};
	template<>
	struct hash<PinPad> {
		size_t operator()(const PinPad& pad) const {
			return pad.hash();
		}
	};
	template<>
	struct hash<PathTree> {
		size_t operator()(const PathTree& node) const {
			return node.hash();
		}
	};
}
