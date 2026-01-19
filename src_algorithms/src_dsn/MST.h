#pragma once
#include <vector>
#include <unordered_map>
#include <memory>
#include <cassert>

#include "RoutingNode.h"

struct SteinerNode : public std::enable_shared_from_this<SteinerNode> {
	PinPad* pin;  // Points to an independently created PinPad object
	Point position;
	double cost;
	weak_ptr<SteinerNode> parent;  // Use weak_ptr to avoid circular references
	vector<shared_ptr<SteinerNode>> children;  // Use shared_ptr
	bool is_original_pin;  // Marks whether it is an original input point

	SteinerNode(PinPad* p, bool original = false, double c = 0,
		std::shared_ptr<SteinerNode> par = nullptr)
		: pin(p), cost(c), is_original_pin(original) {
		assert(p != nullptr && "PinPad must not be null for SteinerNode");
		if (p) {
			position = p->pos;  // Use PinPad's pos as position
		}
		if (par) {
			parent = par;
		}
	}

	// Safe destructor
	~SteinerNode() {}

	// Add child node
	void addChild(std::shared_ptr<SteinerNode> child) {
		if (child && child.get() != this) {
			// If the child already has a parent, remove it from the original parent first
			if (auto parent_ptr = child->parent.lock()) {
				parent_ptr->removeChild(child);
			}
			children.emplace_back(child);
			child->parent = weak_from_this();  // Use weak_ptr
		}
	}

	// Remove child node
	void removeChild(std::shared_ptr<SteinerNode> child) {
		if (!child) return;

		auto it = std::find(children.begin(), children.end(), child);
		if (it != children.end()) {
			children.erase(it);
			child->parent.reset();
		}
	}

	// Disconnect from parent node
	void disconnectFromParent() {
		if (auto parent_ptr = parent.lock()) {
			parent_ptr->removeChild(shared_from_this());
		}
	}

	void getAdjacentNodes(const Point& ignorePos, vector<shared_ptr<SteinerNode>>& neib) {
		neib.clear();
		if (auto p = parent.lock()) {
			if (!(p->position == ignorePos)) {
				neib.emplace_back(p);
			}
		}
		for (const auto& ch : children) {
			if (!ch) continue;
			if (!(ch->position == ignorePos)) {
				neib.emplace_back(ch);
			}
		}
	}

	void changeTopology(const std::shared_ptr<SteinerNode>& node) {
		if (!node || node.get() == this)
			return;
		if (auto oldParent = node->parent.lock()) {
			oldParent->removeChild(node);
			node->parent = weak_from_this();
		}
		this->addChild(node);
	}
};

class SteinerTreeSolver {
public:
	using SteinerTreePtr = std::shared_ptr<SteinerNode>;
	using SteinerTreeMap = std::unordered_map<std::string, SteinerTreePtr>;
	using TreeEdge = std::pair<SteinerTreePtr, SteinerTreePtr>;
	using LayerPair = std::pair<int, int>;
	int m_viaIndex = 0;
	string m_netName;
	const unordered_map<string, NetInfo>* m_netsInfos = nullptr;
	const unordered_map<string, ViaInfo>* m_viaInfos = nullptr;
	int m_minCrossLayerEdgesThreshold = 3;

	SteinerTreeSolver() {};

	~SteinerTreeSolver() {}

	// Compute Steiner trees for all nets
	void computeSteinerTrees(
		std::unordered_map<std::string, std::vector<PinPad*>>& nets,
		std::unordered_map<std::string, NetInfo>& netInfos,
		SteinerTreeMap& forest_roots);
	void setSteinerNodes(const unordered_map<string, NetInfo>& netsInfos, const unordered_map<string, ViaInfo>& viaInfos, SteinerTreeMap& forest_roots, unordered_map<string, PinPad>& preVias);
	void setFlyLines(const SteinerTreeMap& forest_roots, std::vector<std::vector<Line>>& flyLines);


private:
	SteinerTreePtr computeMST(const std::vector<PinPad*>& original_pins);
	void generateFlyLinesFromTree(SteinerTreePtr root, std::vector<Line>& flyLines);

	struct LayerPairHash {
		size_t operator()(const LayerPair& p) const {
			return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
		}
	};
private:
	double getCost(const SteinerTreePtr& node1, const SteinerTreePtr& node2);
	void insertSharedSteinerNodes(SteinerTreePtr& root, unordered_map<string, PinPad>& preVias);
	void collectCrossLayerEdges(SteinerTreePtr root, std::vector<TreeEdge>& crossLayerEdges);	//Traverse the tree to find all "cross-layer edges"
	bool hasCommonLayer(const SteinerTreePtr& a, const SteinerTreePtr& b);						//Check if two nodes are "layer compatible"
	int getRepresentativeLayer(const SteinerTreePtr& node);										//Get a node's "representative layer" (for grouping)
	LayerPair getLayerPairKey(const SteinerTreePtr& u, const SteinerTreePtr& v);					//Calculate the "layer pair key" for an edge
	//Group edges by layer pairs (key for sharing)
	void groupEdgesByLayerPair(const vector<TreeEdge>& crossLayerEdges, unordered_map<LayerPair, vector<TreeEdge>, LayerPairHash>& groupedEdges);
	//Insert a Steiner point for a "group of cross-layer edges" (core functionality)
	void insertOneSharedSteinerNode(const LayerPair& layerPair, const std::vector<TreeEdge>& edges, unordered_map<string, PinPad>& preVias);
	void disconnectTreeEdge(const SteinerTreePtr& u, const SteinerTreePtr& v);					//Disconnect an edge in the tree (safe encapsulation)
	PinPad* createVia(const Point& pos, const LayerPair& layerPair, unordered_map<string, PinPad>& preVias);
};


