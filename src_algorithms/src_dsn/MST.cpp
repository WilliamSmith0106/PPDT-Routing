#include "MST.h"

#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <string>>

void SteinerTreeSolver::computeSteinerTrees(
	std::unordered_map<std::string, std::vector<PinPad*>>& nets,
	std::unordered_map<std::string, NetInfo>& netInfos,
	SteinerTreeMap& forest_roots) {
	forest_roots.clear();
	// Compute Steiner tree for each net one by one
	for (const auto& [netName, pins] : nets) {
		if (pins.size() <= 1) continue;
		const NetInfo& netInfo = netInfos[netName];

		// Compute minimum spanning tree (create independent nodes)
		SteinerTreePtr mst_root = computeMST(pins);
		forest_roots[netName] = mst_root;
	}
}

void SteinerTreeSolver::setSteinerNodes(
	const unordered_map<string, NetInfo>& netsInfos,	//i
	const unordered_map<string, ViaInfo>& viaInfos,		//i
	SteinerTreeMap& forest_roots,						//i,o
	unordered_map<string, PinPad>& preVias				//o
) {
	m_viaIndex = 0;
	m_netsInfos = &netsInfos;
	m_viaInfos = &viaInfos;
	for (auto& [netName, root] : forest_roots) {
		m_netName = netName;
		insertSharedSteinerNodes(root, preVias);
	}
}

void SteinerTreeSolver::setFlyLines(const SteinerTreeMap& forest_roots, std::vector<std::vector<Line>>& flyLines) {
	flyLines.clear();
	for (const auto& [netName, root] : forest_roots) {
		std::vector<Line> net_lines;
		generateFlyLinesFromTree(root, net_lines);
		flyLines.emplace_back(net_lines);
	}
}

// Compute minimum spanning tree (Prim's algorithm) - Returns an independently created node tree
SteinerTreeSolver::SteinerTreePtr SteinerTreeSolver::computeMST(const std::vector<PinPad*>& original_pins) {
	if (original_pins.empty()) return nullptr;
	int n = (int)original_pins.size();
	std::vector<bool> visited(n, false);
	std::vector<double> min_cost(n, std::numeric_limits<double>::max());
	std::vector<int> parent(n, -1);

	std::vector<SteinerTreePtr> nodes;
	for (auto original_pin : original_pins) {
		nodes.emplace_back(std::make_shared<SteinerNode>(original_pin, true));
	}

	min_cost[0] = 0;

	for (int i = 0; i < n; ++i) {
		// Find the unvisited node with minimum cost
		int u = -1;
		double min_val = std::numeric_limits<double>::max();
		for (int j = 0; j < n; ++j) {
			if (!visited[j] && min_cost[j] < min_val) {
				min_val = min_cost[j];
				u = j;
			}
		}

		if (u == -1) break;
		visited[u] = true;

		// Build parent-child relationship
		if (parent[u] != -1)
			nodes[parent[u]]->addChild(nodes[u]);

		// Update adjacent nodes
		for (int v = 0; v < n; ++v) {
			if (!visited[v]) {
				double dist = getCost(nodes[u], nodes[v]);
				if (dist < min_cost[v]) {
					min_cost[v] = dist;
					parent[v] = u;
				}
			}
		}
	}
	return nodes[0]; // Return root node
}

// Generate connections from tree
void SteinerTreeSolver::generateFlyLinesFromTree(SteinerTreePtr root, std::vector<Line>& flyLines) {
	if (!root) return;
	std::queue<SteinerTreePtr> q;
	std::unordered_set<SteinerTreePtr> visited;
	q.push(root);
	visited.insert(root);

	while (!q.empty()) {
		SteinerTreePtr current = q.front();
		q.pop();

		// Add connections to all child nodes
		for (const auto& child : current->children) {
			if (visited.find(child) == visited.end()) {
				flyLines.emplace_back(current->position, child->position);
				q.push(child);
				visited.insert(child);
			}
		}

		// Add connection to parent node (avoid duplication)
		if (const auto& parent_ptr = current->parent.lock()) {
			if (visited.find(parent_ptr) == visited.end()) {
				flyLines.emplace_back(current->position, parent_ptr->position);
			}
		}
	}
}

double SteinerTreeSolver::getCost(const SteinerTreePtr& node1, const SteinerTreePtr& node2) {
	double cost_Euclidean = node1->position.distanceTo(node2->position);
	double cost_Via = cost_Euclidean * 0.5;
	for (const auto& shape : node1->pin->shapes) {
		if(node2->pin->shapes.contains(shape.first))
			cost_Via = 0;
	}
	return cost_Euclidean + cost_Via;
}
void SteinerTreeSolver::insertSharedSteinerNodes(SteinerTreePtr& root, unordered_map<string, PinPad>& preVias) {
	if (!root) return;
	// 1. Collect all "cross-layer edges"
	std::vector<TreeEdge> crossLayerEdges;
	collectCrossLayerEdges(root, crossLayerEdges);

	if (crossLayerEdges.empty()) return;

	// 2. Group by "min-max layer"
	std::unordered_map<LayerPair, std::vector<TreeEdge>, LayerPairHash> groupedEdges;
	groupEdgesByLayerPair(crossLayerEdges, groupedEdges);

	// 3. Insert one shared Steiner point for each group
	for (auto& [layerPair, edges] : groupedEdges) {
		if (edges.size() < m_minCrossLayerEdgesThreshold) continue;
		insertOneSharedSteinerNode(layerPair, edges, preVias);
	}
}

void SteinerTreeSolver::collectCrossLayerEdges(
	SteinerTreePtr root,
	std::vector<TreeEdge>& crossLayerEdges
) {	// Traverse the tree to find all "cross-layer edges"
	if (!root) return;

	std::queue<SteinerTreePtr> q;
	std::unordered_set<SteinerTreePtr> visited;

	q.push(root);
	visited.insert(root);

	while (!q.empty()) {
		auto u = q.front();
		q.pop();

		// Traverse u's child nodes (tree edge u -> v)
		for (auto& v : u->children) {
			if (!v) continue;

			if (!hasCommonLayer(u, v)) {
				crossLayerEdges.emplace_back(u, v);
			}

			if (visited.insert(v).second) {
				q.push(v);
			}
		}

		// Parent node direction (ensure the entire tree can be traversed)
		if (auto p = u->parent.lock()) {
			if (visited.insert(p).second) {
				q.push(p);
			}
		}
	}
}
bool SteinerTreeSolver::hasCommonLayer(
	const SteinerTreePtr& a,
	const SteinerTreePtr& b
) {	// Check if two nodes are "layer compatible"
	if (!a || !b) return false;
	if (!a->pin || !b->pin) return false;
	const auto& shapesA = a->pin->shapes;
	const auto& shapesB = b->pin->shapes;
	if (shapesA.empty() || shapesB.empty())
		return false;
	// Check if layers have intersection
	for (const auto& [layerA, _] : shapesA) {
		if (shapesB.find(layerA) != shapesB.end()) {
			return true;
		}
	}
	return false;
}
int SteinerTreeSolver::getRepresentativeLayer(const SteinerTreePtr& node)
{// Get a node's "representative layer" (used for grouping)
	assert(node && node->pin);
	assert(!node->pin->shapes.empty());
	return node->pin->shapes.begin()->first;
}
SteinerTreeSolver::LayerPair SteinerTreeSolver::getLayerPairKey(
	const SteinerTreePtr& u,
	const SteinerTreePtr& v
) {// Calculate the "layer pair key" for an edge
	int lu = getRepresentativeLayer(u);
	int lv = getRepresentativeLayer(v);
	if (lu > lv) std::swap(lu, lv);
	return LayerPair(lu, lv);
}

void SteinerTreeSolver::groupEdgesByLayerPair(
	const vector<TreeEdge>& crossLayerEdges,
	unordered_map<LayerPair, vector<TreeEdge>,
	LayerPairHash>& groupedEdges
) {// Group edges by layer pairs
	groupedEdges.clear();
	for (const auto& e : crossLayerEdges) {
		const auto& u = e.first;
		const auto& v = e.second;
		LayerPair key = getLayerPairKey(u, v);
		groupedEdges[key].emplace_back(e);
	}
}

void SteinerTreeSolver::insertOneSharedSteinerNode(
	const LayerPair& layerPair,
	const std::vector<TreeEdge>& edges,
	unordered_map<string, PinPad>& preVias
) {// Insert a shared Steiner point for a group of "cross-layer edges" (core functionality)
	assert(!edges.empty());

	// 1. Calculate position
	Point sum(0, 0);
	for (const auto& [u, v] : edges)
		sum = sum + (u->position + v->position) * 0.5;
	Point steinerPos = sum * (1.0 / edges.size());

	// 2. Create Steiner
	PinPad* steinerPad = createVia(steinerPos, layerPair, preVias);
	if (!steinerPad) return;
	auto steinerNode = std::make_shared<SteinerNode>(steinerPad, false);

	// 3. Take over cross-layer edges
	bool steinerAttached = false;
	for (const auto& [u, v] : edges) {
		SteinerTreePtr parent = nullptr;
		SteinerTreePtr child = nullptr;

		if (v->parent.lock() == u) {
			parent = u;
			child = v;
		}
		else if (u->parent.lock() == v) {
			parent = v;
			child = u;
		}
		else {
			continue;
		}

		disconnectTreeEdge(parent, child);

		// Insert Steiner into the tree only the first time
		if (!steinerAttached) {
			parent->addChild(steinerNode);
			steinerAttached = true;
		}

		// Attach all children to Steiner
		steinerNode->addChild(child);
	}
}
void SteinerTreeSolver::disconnectTreeEdge(
	const SteinerTreePtr& u,
	const SteinerTreePtr& v
) {	// Disconnect an edge in the tree (safe encapsulation)
	if (!u || !v) return;

	// u is the parent of v
	if (v->parent.lock() == u) {
		u->removeChild(v);
		return;
	}
	// v is the parent of u
	if (u->parent.lock() == v) {
		v->removeChild(u);
		return;
	}
}

PinPad* SteinerTreeSolver::createVia(const Point& pos, const LayerPair& layerPair, unordered_map<string, PinPad>& preVias) {
	const NetInfo& netInfo = m_netsInfos->at(m_netName);
	const ViaInfo& viaInfo = m_viaInfos->at(netInfo.viaName);

	string viaName = "Steiner_" + to_string(m_viaIndex++);
	preVias.insert(make_pair(viaName, PinPad(pos, viaName, m_netName)));
	PinPad* viaPadPtr = &preVias[viaName];
	int layer1 = layerPair.first;
	int layer2 = layerPair.second;
	for (const int& layer : viaInfo.m_layers) {
		if (layer < layer1)
			layer1 = layer;
		else if (layer > layer2)
			layer2 = layer;
	}
	for (int layer = layer1; layer <= layer2; ++layer) {
		viaPadPtr->addShape(layer, viaInfo.m_radius, pos);
	}
	return viaPadPtr;
}