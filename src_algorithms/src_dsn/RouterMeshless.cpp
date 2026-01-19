#include "RouterMeshless.h"
#include <algorithm>
#include <utility>
#include <map>
#include "../src_basics/utils.h"
#include <cstdio>


using namespace std;

void RouterMeshless::routerReset(double& gridSize) {
	//1. Data continuously supplemented during algorithm execution
	m_debugEnd = false;
	m_vias.clear();
	m_pathHeads.clear();
	m_planningPts.clear();
	m_PinQuaryPad.clear();
	m_GNDConnected.clear();
	m_netFound.clear();
	m_layerGND = 0;
	m_viasSum = 0;

	//2. Result data from algorithm execution
	freePathsAndTrees();

	//3. Establish spatial index
	if (gridSize < 1) {
		double totalPadArea = 0.0;
		m_totalPins = 0;
		if (m_netTrees) {
			for (const auto& [netName, root] : *m_netTrees) {
				if (!root) continue;
				std::function<void(SteinerNode*)> dfs = [&](SteinerNode* node) {
					if (!node) return;
					if (node->pin) {
						const auto& box = node->pin->box;
						double w = box[2] - box[0];
						double h = box[3] - box[1];
						if (w > 0 && h > 0) {
							totalPadArea += w * h;
							m_totalPins++;
						}
					}
					for (auto& child : node->children)
						dfs(child.get());
					};
				dfs(root.get());
			}
		}
		if (m_totalPins > 0) {
			gridSize = m_grideSizeFactor * std::sqrt(totalPadArea / m_totalPins);
		}
		else {
			gridSize = 10.0;   // Or your project's default value
		}
	}
	initializeGrid(gridSize);
}

void RouterMeshless::run(vector<string>& routingInfo) {
	if (m_netTrees == nullptr) {
		cerr << "Error: RouterMeshless::run() m_netTrees == nullptr" << endl;
		return;
	}
	if (!m_priorityRule)
		m_priorityRule = defaultPriorityRule;

	// 1. Decompose steinerTree into multiple start-end pairs and set default priority rule
	vector<pair<PinPad*, PinPad*>> pinPairs;
	for (const auto& [netName, steinerTree] : *m_netTrees) {
		if (!steinerTree) continue;
		extractPinPairs(steinerTree, pinPairs);
		// Set special net information, such as GND
		setSpecialNetInfomation(netName);
	}
	// 2. Sort start-end pairs
	sort(pinPairs.begin(), pinPairs.end(), m_priorityRule);
	// 3. Reset algorithm data
	int pathSum = (int)pinPairs.size();
	int netSum = (int)m_nets->size();
	m_pathFoundNum = 0;
	m_serchTimesLimit = pathSum * 7;
	// 4. Route all start-end pairs
	// Output comment cout
	cout << "============================ Start routing: " << pathSum << " ============================" << endl;
	bool found = findAllPaths(pinPairs);
	// 5. Calculate path length
	m_totalPathLength = getPathsLength();
	// Debug related content
	int netFoundNum = 0;
	for (const auto& [netName, found] : m_netFound) {
		if (found) netFoundNum++;
	}
	double ratePinPairs = pathSum == 0 ? 0.0 : static_cast<double>(m_pathFoundNum) / pathSum * 100.0;
	double rateNets = m_netFound.empty() ? 0.0 : static_cast<double>(netFoundNum) / m_netFound.size() * 100.0;
	int layersSum = static_cast<int>(m_routingLayers.size());
	double avgLen = m_totalPathLength / max(m_pathFoundNum, 1);
	cout << "--------------------------->Summary:" << endl;
	/**/
	cout << "layers:" << layersSum
		<< ", pinPairs:" << pathSum
		<< ", found:" << m_pathFoundNum
		<< ", failed:" << pathSum - m_pathFoundNum
		<< ", ratePinPairs:" << ratePinPairs << "%"
		<< ", avgLength:" << m_totalPathLength / max(m_pathFoundNum, 1)
		<< ", via:" << m_viasSum
		<< ", rateNets:" << fixed << setprecision(2) << rateNets << "%" << endl;
	/**/
	cout << "============================ Routing Ended =================================" << endl;
	routingInfo.clear();
	char buf[32];
	routingInfo.emplace_back(std::to_string(layersSum));
	routingInfo.emplace_back(std::to_string(pathSum));
	routingInfo.emplace_back(std::to_string(m_pathFoundNum));
	routingInfo.emplace_back(std::to_string(pathSum - m_pathFoundNum));

	snprintf(buf, sizeof(buf), "%.2f%%", ratePinPairs);
	routingInfo.emplace_back(buf);

	snprintf(buf, sizeof(buf), "%.2f%", avgLen);
	routingInfo.emplace_back(buf);

	routingInfo.emplace_back(std::to_string(m_viasSum));

	snprintf(buf, sizeof(buf), "%.2f%%", rateNets);
	routingInfo.emplace_back(buf);

}
void RouterMeshless::initializeGrid(const double& gridSize) {
	if (m_bound.size() < 4) {
		cerr << "Error: Invalid boundary data. Expected 4 values: {minX, minY, maxX, maxY}" << endl;
		return;
	}
	// Check if nets is empty
	if (!m_netTrees || m_netTrees->empty()) {
		cout << "Warning: No net trees to process" << endl;
		return;
	}
	double minX = m_bound[0];
	double minY = m_bound[1];
	double maxX = m_bound[2];
	double maxY = m_bound[3];
	if (minX >= maxX || minY >= maxY) {
		cerr << "Error: Invalid boundary range in initializeGrid" << endl;
		return;
	}
	// 1. Create grid manager
	m_gridManager = make_unique<GridManager>(minX, minY, maxX, maxY, gridSize);
	// 2. Add polygon pads corresponding to all pins to the grid
	int totalPinPads = 0;
	for (auto& [padName, pad] : *m_pads) {
		m_gridManager->addPinPad(&pad);
		totalPinPads++;
	}
	// 3. Add polygon pads corresponding to all pre-vias to the grid
	for (auto& [padName, pad] : *m_preVias) {
		m_gridManager->addPinPad(&pad);
		totalPinPads++;
	}
	// 4. Set initial routable layers
	for (const auto& [shapeName, pad] : *m_pads) {
		for (const auto& [layer, shape] : pad.shapes) {
			if (!m_routingLayers.contains(layer))
				m_routingLayers.insert(layer);
		}
	}
	/*
	// Output grid statistics (for debugging)
	if (m_gridManager) {
		auto& allCells = m_gridManager->getAllCells();
		int nonPadEmptyCells = 0;
		int maxPadsInCells = 0;
		for (auto& cell : allCells) {
			int padsCount = cell->getPinPads().size();
			if (padsCount > 0) {
				nonPadEmptyCells++;
				maxPadsInCells = max(maxPadsInCells, padsCount);
			}
		}
		cout << "================Grid statistics================" << endl;
		cout << "totalPinPads" << totalPinPads << endl;
		cout << "\tnon-pad empty cells: " << nonPadEmptyCells << endl;
		cout << "\tmax pads in cells: " << maxPadsInCells << endl;
		cout << "================Grid statistics================" << endl;
	}
	*/
}
void RouterMeshless::freePathsAndTrees() {
	for (auto& net : m_treesHeads) {
		for (auto& node : net.second) {
			node->remove();
		}
	}
	m_treesHeads.clear();
	m_pathTreesOrdered.clear();
	// Free memory for m_paths
	for (auto& [pNode, shape] : m_paths) {
		pNode->deleteRelatedNodes();
	}
	m_paths.clear();
}
// Data preparation
void RouterMeshless::setSpecialNetInfomation(string netName) {
	transform(netName.begin(), netName.end(), netName.begin(), ::toupper);
	if (netName == "GND") {
		m_NetNameGND = netName;	//GND net name
		const ViaInfo& viaInfo = m_viaInfos->at(m_netsInfos->at(netName).viaName);
		for (const auto& layer : viaInfo.m_layers) {
			if (layer > m_layerGND)
				m_layerGND = layer;		//GND layer (use the maximum layer as GND layer)
		}
	}
}
void RouterMeshless::extractPinPairs(const shared_ptr<SteinerNode>& steinerTree, vector<pair<PinPad*, PinPad*>>& pairs) {
	// Use BFS to traverse Steiner tree and collect all connection relationships
	queue<shared_ptr<SteinerNode>> q;
	unordered_set<shared_ptr<SteinerNode>> visited;
	q.emplace(steinerTree);
	visited.insert(steinerTree);
	while (!q.empty()) {
		shared_ptr<SteinerNode> currentNode = q.front();
		q.pop();
		// Process all child nodes of the current node
		for (auto& child : currentNode->children) {
			if (visited.find(child) == visited.end()) {
				visited.insert(child);
				q.emplace(child);
				// Add start-end pair
				if (currentNode->pin && child->pin) {
					setOrderedPinPair(currentNode->pin, child->pin, pairs);
				}
			}
		}
		// Process parent node (if bidirectionally connected)
		if (auto parent = currentNode->parent.lock()) {
			if (visited.find(parent) == visited.end()) {
				visited.insert(parent);
				q.emplace(parent);
				if (currentNode->pin && parent->pin) {
					setOrderedPinPair(currentNode->pin, parent->pin, pairs);
				}
			}
		}
	}
}
void RouterMeshless::setOrderedPinPair(PinPad* pin1, PinPad* pin2, vector<pair<PinPad*, PinPad*>>& pairs) {
	double deltaX = pin1->pos.x - pin2->pos.x;
	if (abs(deltaX) < MapMinValue) {		// Vertical line
		double deltaY = pin1->pos.y - pin2->pos.y;
		if (deltaY > MapMinValue) {		// Vertical line, bottom as start point
			std::swap(pin1, pin2);
		}
	}
	else if (deltaX > MapMinValue) {		// Non-vertical line, > take left as start point, < take right as start point
		std::swap(pin1, pin2);
	}
	pairs.emplace_back(pin1, pin2);
}
bool RouterMeshless::getPlanningPos(PathNode* node, Point& pos, bool isLine) {
	if (node->direction.vecLength() < 3) {
		double minWireSpacing = m_curNetInfo->width / 2 + node->width / 2;
		double maxClearance = m_curNetInfo->clearance;
		if (node->shape) {	// Pad vertex
			maxClearance = max(maxClearance, node->shape->clearance);
		}
		minWireSpacing += maxClearance;
		if (isLine) {
			bool prevSameLayer = node->prev ? node->prev->layer == node->layer : false;
			bool nextSameLayer = node->next ? node->layer == node->next->layer : false;
			if (prevSameLayer && nextSameLayer) {
				pos = node->pos + node->direction * (minWireSpacing + maxClearance);
				m_planningPts.insert(pos);
			}
			else {	// Cross-layer, take current point
				pos = node->pos;
			}
		}
		else {
			pos = node->pos + node->direction * minWireSpacing;
			m_planningPts.insert(pos);
		}
	}
	else {
		pos = node->pos;
		m_planningPts.insert(pos);
		//cout << "getPlanningPos error,pos: "<<pos<<" ,node->direction.vecLength= " << node->direction.vecLength() << endl;
		return false;
	}
	return true;
}
void RouterMeshless::mergeNode(PathNode* node) {
	if (!node) return;
	//1. Move to head node
	while (node->prev)
		node = node->prev;
	if (node->next) {
		node = node->next;
	}
	else
		return;
	//2. Merge collinear nodes
	while (node->next) {
		if (node->layer != node->prev->layer || node->layer != node->next->layer) {
			node = node->next;
			continue;
		}
		if (node->pos.inSameLine(node->prev->pos, node->next->pos))
			node = node->deleteCurruntNode();
		else
			node = node->next;
	}
}
bool RouterMeshless::findAllPaths(vector<pair<PinPad*, PinPad*>>& pinPairs) {
	// Route single start-end pair
	for (m_curPathIndex = 0; m_curPathIndex < pinPairs.size(); m_curPathIndex++) {
		auto& pair = pinPairs[m_curPathIndex];
		//1. Prepare path data for current pathfinding
		if (prepareOnePathData(pair))
			continue;
		//2. Set initial node and target node
		setStartAndEnd();
		debugBreak(m_node_start);
		debugBreak(m_node_end);
		//3. Special routing
		bool isSpecialPath = routeSpecially();
		if (isSpecialPath) continue;
		//5. Routing
		m_searchTimes = 0;
		bool found = runPPDT();
		if (found)
			m_pathFoundNum++;
		else
			m_netFound[m_curNetName] = false;
		if (m_debugEnd)		// Debug interruption for routing
			return true;
	}
	return true;
}
bool RouterMeshless::prepareOnePathData(pair<PinPad*, PinPad*>& pair) {
	//1. Current routing net name
	m_startPad = pair.first;
	m_endPad = pair.second;
	m_curNetName = m_startPad->netName;
	m_curNetInfo = &m_netsInfos->at(m_curNetName);
	m_viaRadius = m_viaInfos->at(m_curNetInfo->viaName).m_radius;
	//2. Initialize net routing information
	auto it = m_netFound.find(m_curNetName);
	if (it == m_netFound.end())
		m_netFound[m_curNetName] = true;
	//3. Adjacent Steiner points of start and end points
	m_steinerQuery[m_startPad->pos]->getAdjacentNodes(m_endPad->pos, m_startNeibs);
	m_steinerQuery[m_endPad->pos]->getAdjacentNodes(m_startPad->pos, m_endNeibs);

	//4. Start and end layers
	m_startLayers.clear();
	m_endLayers.clear();
	for (const auto& [layer, shape] : m_startPad->shapes)
		m_startLayers.insert(layer);
	for (const auto& [layer, shape] : m_endPad->shapes)
		m_endLayers.insert(layer);
	if (m_startLayers.empty() || m_endLayers.empty()) {
		cout << "Path [ " << m_curPathIndex << " ]\t" << "\tError: no layers to run!" << endl;
		return true;
	}
	//5. Reset leaf nodes for path search, reset explored planning points to zero
	m_leafNodesList = priority_queue<PathTree*, vector<PathTree*>, ComparePathTreePtr>();
	m_exploredNodes.clear();
	m_candidateObss = queue<PolyShape*>();
	m_queryObssPassed.clear();
	m_querySTChanged.clear();
	return false;
}
void RouterMeshless::setStartAndEnd() {
	m_standartCost = m_startPad->pos.distanceTo(m_endPad->pos) * m_standardCostFactor;
	//1. Determine routing layer
	int layer_s = *m_startLayers.begin();
	int layer_e = *m_endLayers.begin();
	for (const int& layer : m_startLayers) {
		if (m_endLayers.contains(layer)) {
			layer_s = layer;
			layer_e = layer;
			break;
		}
	}
	//2. Create path tree nodes on selected routing layer
	m_node_start = new PathTree(m_startPad->pos, layer_s, m_curNetName, nullptr);
	m_PinQuaryPad.insert(make_pair(m_node_start, m_startPad));

	m_node_end = new PathTree(m_endPad->pos, layer_e, m_curNetName, nullptr);
	m_PinQuaryPad.insert(make_pair(m_node_end, m_endPad));

	m_pathHeads[m_node_start] = m_curNetName;
	if (m_treesHeads.find(m_curNetName) == m_treesHeads.end())
		m_treesHeads[m_curNetName] = vector<PathTree*>();
	m_treesHeads[m_curNetName].emplace_back(m_node_start);
	m_pathTreesOrdered.emplace_back(m_node_start);

	//3.1 Start point extension and set as child node
	setStartPad(layer_s);
	//4.1 End point extension, get feasible pin exit points
	setEndPad(layer_e);
	return;
}
void RouterMeshless::setStartPad(int layer_s) {
	//3.1 Start point extension, get feasible pin exit points
	PinPad* padPtr;
	int directionType = getPinPadOutDirection(m_node_start, padPtr);
	double minClear = max(m_curNetInfo->clearance, padPtr->shapes[m_node_start->layer].clearance);
	vector<Point> outDirectionsPts;
	bool setStartPadOut = true;
	while (setStartPadOut) {
		setStartPadOut = false;
		if (directionType == 0)			// 8-direction extension
			setPadOuPts(m_node_start, m_node_end, minClear, padPtr->box, 0, shape_out_direction[0], outDirectionsPts);
		else if (directionType == 1)	// Up and down
			setPadOuPts(m_node_start, m_node_end, minClear, padPtr->box, 2, shape_out_direction[1], outDirectionsPts);
		else if (directionType == 2)	// Left and right
			setPadOuPts(m_node_start, m_node_end, minClear, padPtr->box, 0, shape_out_direction[1], outDirectionsPts);
		else if (directionType == 3) 	// Up, down, left, right
			setPadOuPts(m_node_start, m_node_end, minClear, padPtr->box, 0, shape_out_direction[2], outDirectionsPts);
		else {
			cout << "Path [ " << m_curPathIndex << " ]:\t" << "no direction for startPad to rout!" << endl;
			m_leafNodesList.emplace(m_node_start);
		}
		if (outDirectionsPts.empty()) {
			// No feasible exit points found for current pad
			for (const auto& newSteinerNode : m_startNeibs) {
				if (m_querySTChanged.contains(newSteinerNode->position)) continue;
				if (newSteinerNode->position == m_endPad->pos) continue;
				if (changeStartOrEnd(true, newSteinerNode->pin, layer_s)) {
					setStartPadOut = true;
					break;
				}
			}
		}
	}

	//3.2 Start point extension, extend exit points as child nodes
	if (!outDirectionsPts.empty()) {
		vector<PathTree*> outNodes;
		// First set exit points on routing layer
		for (auto& pt : outDirectionsPts) {
			m_planningPts.insert(pt);
			//bool canReach = isReachable(m_node_start->pos, pt, layer, m_endPad);
			bool canReach = isReachable(m_node_start->pos, pt, layer_s, m_startPad, m_endPad);
			if (canReach) {
				PathTree* newNode = addOneChild(m_node_start, pt, layer_s, nullptr, false);
				if (newNode)
					outNodes.emplace_back(newNode);
			}
		}
		// Then set exit points on other layers
		for (const int& layer : m_routingLayers) {
			if (layer == layer_s) continue;
			else if (m_startLayers.contains(layer)) {	// Search tree changes layer at pad center, actual path doesn't need to change layer
				PathTree* newStartNode_inLayer = new PathTree(m_startPad->pos, layer, m_curNetName, nullptr);
				double estimateH = m_startPad->pos.distanceTo(m_endPad->pos);
				double stepG = 0;
				if (!m_startPad->shapes.contains(layer))
					stepG = m_standartCost;
				if (!m_endPad->shapes.contains(layer))
					estimateH += m_standartCost;
				m_node_start->addChild(newStartNode_inLayer, stepG, estimateH);
				for (auto& pt : outDirectionsPts) {
					m_planningPts.insert(pt);
					//bool canReach = isReachable(newStartNode_inLayer->pos, pt, layer, m_endPad);
					bool canReach = isReachable(newStartNode_inLayer->pos, pt, layer, m_startPad, m_endPad);
					if (canReach)
						addOneChild(newStartNode_inLayer, pt, layer, nullptr, false);
				}
			}
			else {		// Search tree changes layer at exit point, actual path also needs to change layer
				for (PathTree* node : outNodes) {
					addOneChild(node, node->pos, layer, nullptr, false);
				}
			}
		}
	}
}
void RouterMeshless::setEndPad(int layer_e) {
	PinPad* padPtr;
	int directionType = getPinPadOutDirection(m_node_end, padPtr);
	double minClear = max(m_curNetInfo->clearance, padPtr->shapes[m_node_end->layer].clearance);
	m_endExits.clear();
	if (directionType == 0)			// Circle, 8-direction extension
		setPadOuPts(m_node_end, m_node_start, minClear, padPtr->box, 0, shape_out_direction[0], m_endExits);
	else if (directionType == 1)	// Rectangle, up and down
		setPadOuPts(m_node_end, m_node_start, minClear, padPtr->box, 2, shape_out_direction[1], m_endExits);
	else if (directionType == 2)	// Rectangle, left and right
		setPadOuPts(m_node_end, m_node_start, minClear, padPtr->box, 0, shape_out_direction[1], m_endExits);
	else if (directionType == 3) 	// Nearly square, up, down, left, right
		setPadOuPts(m_node_end, m_node_start, minClear, padPtr->box, 0, shape_out_direction[2], m_endExits);
	else {
		cout << "Path [ " << m_curPathIndex << " ]:\t" << "no direction for endPad to run!" << endl;
	}
}
void RouterMeshless::setPadOuPts(PathTree* nodeToSet, PathTree* nodeToIgnore, double minClear, const vector<double>& box, int startIdx, int step, vector<Point>& outDirectionsPts) {
	double lineWidth = m_curNetInfo->width / 2;
	double width_x = (box[2] - box[0]) / 2 + minClear + m_viaRadius;		//m_viaRadius
	double height_y = (box[3] - box[1]) / 2 + minClear + m_viaRadius;		//m_viaRadius
	double max_xy = max(width_x, height_y);
	vector<double> outLengths = { width_x ,max_xy, height_y ,max_xy ,width_x ,max_xy, height_y ,max_xy };
	for (int i = startIdx; i < 8; i += step) {
		const Point& d = Direction8[i];
		Point targetPos = nodeToSet->pos + d * outLengths[i];
		PinPad* nodeToSetPad = nullptr;
		if (m_PinQuaryPad.contains(nodeToSet))
			nodeToSetPad = m_PinQuaryPad[nodeToSet];
		PolyShape* firstShape = getFirstShape(nodeToSet->pos, targetPos, nodeToSet->layer, nodeToSetPad);
		bool canReach = false;
		if (firstShape) {
			PinPad* padEnd = getPadPtr(m_PinQuaryPad[nodeToIgnore]->shapeName);
			if (padEnd->shapes.contains(nodeToSet->layer) && firstShape == &padEnd->shapes[nodeToSet->layer]) {
				canReach = true;
			}
		}
		else
			canReach = true;
		if (canReach) {
			outDirectionsPts.emplace_back(targetPos);
			m_planningPts.insert(targetPos);
		}
	}
}
int RouterMeshless::getPinPadOutDirection(PathTree* const nodeSE, PinPad*& padptr) {
	// Get pad lead direction: 0-eight arbitrary directions, 1-vertical up-down, 2-horizontal left-right, 3-nearly square up-down-left-right
	if (!m_PinQuaryPad.contains(nodeSE)) {
		cout << "\tError! pinPad has no shape in net [" << nodeSE->netName << "]" << endl;
		return -1;
	}
	const string& shapeName = m_PinQuaryPad[nodeSE]->shapeName;
	padptr = getPadPtr(shapeName);
	const PinPad& pad = *padptr;
	if (pad.r > MapMinValue) {		// Circular pad, any standard direction is allowed
		return 0;
	}
	else {
		double deltaX = pad.box[2] - pad.box[0];
		double deltaY = pad.box[3] - pad.box[1];
		if (deltaY > rectLengthFactor * deltaX) {		//y direction is longer, vertical long strip 
			return 1;
		}
		else if (deltaX > rectLengthFactor * deltaY) {	//x direction is longer, horizontal long strip
			return 2;
		}
		else {		// Nearly square
			return 3;
		}
	}
	return 0;
}
bool RouterMeshless::changeStartOrEnd(bool isStart, PinPad* pad, int layer) {
	if (!pad) return false;
	//1. Modify Steiner tree topology
	auto itChange = m_steinerQuery.find(isStart ? m_startPad->pos : m_endPad->pos);
	auto itHold = m_steinerQuery.find(isStart ? m_endPad->pos : m_startPad->pos);
	if (itChange == m_steinerQuery.end() || itHold == m_steinerQuery.end()) {
		cerr << "Error! SteinerNode not found in changeStartOrEnd\n";
		return false;
	}
	shared_ptr<SteinerNode> steinerToChange = itChange->second;
	shared_ptr<SteinerNode> steinerToHold = itHold->second;
	shared_ptr<SteinerNode> newST = m_steinerQuery[pad->pos];
	if (!updateSteinerTopology(steinerToChange, steinerToHold, newST)) {
		cerr << "Error! SteinerNode topology change failed in path:" << m_curPathIndex << endl;
		return false;
	}
	m_querySTChanged.insert(itChange->second->position);
	//2. Modify start and end point related information
	PathTree newNode(pad->pos, layer, pad->netName, nullptr);
	// Update pad and coordinate information of start or end point
	if (isStart) {
		m_node_start->copyIn(newNode);
		m_startPad = pad;
		m_PinQuaryPad[m_node_start] = pad;
	}
	else {
		m_node_end->copyIn(newNode);
		m_endPad = pad;
		m_PinQuaryPad[m_node_end] = pad;
	}
	//3. Update adjacent Steiner nodes
	m_steinerQuery[m_startPad->pos]->getAdjacentNodes(m_endPad->pos, m_startNeibs);
	m_steinerQuery[m_endPad->pos]->getAdjacentNodes(m_startPad->pos, m_endNeibs);
	// Update cost base
	m_standartCost = m_node_start->pos.distanceTo(m_node_end->pos) * m_standardCostFactor;
	return true;
}
bool RouterMeshless::updateSteinerTopology(const shared_ptr<SteinerNode>& steinerToChange, const shared_ptr<SteinerNode>& steinerToHold, const shared_ptr<SteinerNode>& newST) {
	auto parentChange = steinerToChange->parent.lock();
	auto parentHold = steinerToHold->parent.lock();
	if (steinerToChange == parentHold) {
		// steinerToChange is the parent node
		newST->changeTopology(steinerToHold);
	}
	else if (steinerToHold == parentChange) {
		// steinerToChange is the child node
		steinerToHold->changeTopology(newST);
	}
	else if (parentChange == parentHold) {
		// Share the same parent node
		if (!parentChange)
			return false;
		parentChange->removeChild(steinerToHold);
		steinerToChange->addChild(steinerToHold);
	}
	else if (!parentHold) {
		steinerToChange->addChild(steinerToHold);
	}
	else if (!parentChange) {
		steinerToHold->addChild(steinerToChange);
	}
	else {
		return false;
	}
	return true;
}
double RouterMeshless::getPathsLength() {
	double length = 0;
	for (auto& [node, shape] : m_paths) {
		if (!node) continue;
		PathNode* cur = node;
		while (cur->next) {
			length += cur->pos.distanceTo(cur->next->pos);
			cur = cur->next;
		}
	}
	return length;
}
int RouterMeshless::getSpecialRoutingType() {
	return 0;
}
bool RouterMeshless::routeSpecially() {
	//1. Start and end points coincide and are on the same layer
	double minPadSize = min(m_startPad->box[2] - m_startPad->box[0], m_startPad->box[3] - m_startPad->box[1]);
	double minPadSize2 = min(m_endPad->box[2] - m_endPad->box[0], m_endPad->box[3] - m_endPad->box[1]);
	minPadSize = max(minPadSize, minPadSize2);
	if (m_node_start->pos.distanceTo(m_node_end->pos) < minPadSize) {
		bool inTheSameLayer = false;
		for (const auto& layer : m_startLayers) {
			if (m_endLayers.contains(layer)) {
				inTheSameLayer = true;
				break;
			}
		}
		if (inTheSameLayer) {
			m_pathFoundNum++;
			return true;
		}
		else {
			return false;
		}
	}
	//2. Special handling for GND
	if (m_GNDRoute && m_curNetName == m_NetNameGND) {
		bool found = route_GND();
		if (found) {
			m_pathFoundNum++;
		}
		else {
			m_netFound[m_curNetName] = false;
		}
		return true;
	}
	return false;
}

//PPDT routing algorithm
bool RouterMeshless::runPPDT() {
	// Execute planning point directed spanning tree search algorithm
	bool pathFound = false;
	//debugBreak(m_node_start);
	while (!m_leafNodesList.empty()) {
		//1 Select the optimal node
		PathTree* minNode = nodeSelection();
		//2 Explore and obtain target obstacle list
		obssExploration(minNode);
		//3 Expand child nodes
		pathFound = nodeExpansion(minNode);
		m_searchTimes++;
		if (m_searchTimes > m_serchTimesLimit) {
			cout << "time out!" << endl;
			break;
		}
		if (pathFound)// Found path, exit loop
			break;
	}
	//4 Backtrack and generate path
	if (pathFound) {
		// Backtrack, post-processing (backtrack pin update)
		if (m_postOn) {
			if (m_pinPairExchange)		//1. Enable: Pin exchange
				checkNewStartNode();
		}
		backTrackOnePath(m_node_end);
		return true;
	}
	else
		cout << "Path[" << m_curPathIndex << "]\t" << "--> \trouting failed : " << m_node_start->pos << m_node_end->pos << endl;
	return false;
}
bool RouterMeshless::route_GND() {
	// Pad already grounded, no need for routing
	bool startGotGnd = m_GNDConnected.find(m_node_start->pos) != m_GNDConnected.end();
	bool endGotGnd = m_GNDConnected.find(m_node_end->pos) != m_GNDConnected.end();
	if (startGotGnd && endGotGnd) {
		return true;
	}
	// Pad on GND layer, no need for routing
	bool startInGndLayer = m_startLayers.contains(m_layerGND);
	bool endInGndLayer = m_endLayers.contains(m_layerGND);
	if (startInGndLayer && endInGndLayer) {
		m_GNDConnected.insert(m_node_start->pos);
		m_GNDConnected.insert(m_node_end->pos);
		return true;
	}
	// Only one or neither is grounded
	PolyShape* firstObs = nullptr;
	bool canReach = isReachable(m_node_start->pos, m_node_end->pos, m_node_start->layer, m_startPad, m_endPad, &firstObs);
	if (canReach) {
		m_node_start->addChild(m_node_end, 0, 0);
		// Generate a path from m_node_start to m_node_end
		backTrackOnePath(m_node_end);
		m_node_end->parent->children.erase(m_node_end);
		m_node_end->parent = nullptr;
		bool alreadyGotGnd = startGotGnd || startInGndLayer || endGotGnd || endInGndLayer;
		if (!alreadyGotGnd) {
			PathTree* viaNode = m_leafNodesList.top();
			double minCost = 999999999;
			//1. Select the optimal via node (at start point)
			while (!m_leafNodesList.empty()) {
				PathTree* selectedNode = m_leafNodesList.top();
				m_leafNodesList.pop();
				if (selectedNode->layer == m_layerGND && selectedNode->g < minCost) {
					minCost = selectedNode->g;
					viaNode = selectedNode;
				}
			}
			//2. Select the optimal via node (at end point)
			Point pt = m_endExits.front();
			Point rst = pt;
			bool found = false;
			int obsSum = getCongestionSum(pt, m_viaRadius, m_node_end->layer, m_layerGND);
			for (int i = 1; i < m_endExits.size(); ++i) {
				pt = m_endExits[i];
				obsSum = getCongestionSum(pt, m_viaRadius, m_node_end->layer, m_layerGND);
				double congestionCostFactor = obsSum * m_decayFactor;
				if (congestionCostFactor < minCost) {
					bool canReach = isReachable(m_node_end->pos, pt, m_node_end->layer, m_startPad, m_endPad);
					if (canReach) {
						found = true;
						minCost = congestionCostFactor;
						rst = pt;
					}
				}
			}
			if (found) {
				PathTree* newNode = addOneChild(m_node_end, rst, m_node_end->layer, nullptr, false);
				viaNode = addOneChild(newNode, rst, m_layerGND, nullptr, false);
			}
			backTrackOnePath(viaNode);
		}
		m_GNDConnected.insert(m_node_start->pos);
		m_GNDConnected.insert(m_node_end->pos);
	}
	else {
		if (m_leafNodesList.empty() || m_endExits.empty()) {
			cout << "Path[" << m_curPathIndex << "]\t" << "--> \trouting failed : " << m_node_start->pos << m_node_end->pos << endl;
			return false;
		}
		if (!startGotGnd && !startInGndLayer) {
			// Start point not grounded
			PathTree* viaNode = m_leafNodesList.top();
			double minCost = 999999999;
			while (!m_leafNodesList.empty()) {
				PathTree* selectedNode = m_leafNodesList.top();
				m_leafNodesList.pop();
				if (selectedNode->layer == m_layerGND && selectedNode->g < minCost) {
					minCost = selectedNode->g;
					viaNode = selectedNode;
				}
			}
			backTrackOnePath(viaNode);
			m_GNDConnected.insert(m_node_start->pos);
		}
		if (!endGotGnd && !endInGndLayer) {
			// End point not grounded
			Point pt = m_endExits.front();
			Point rst = pt;
			int obsSum = getCongestionSum(pt, m_viaRadius, m_node_end->layer, m_layerGND);
			double minCost = obsSum * m_decayFactor;
			for (int i = 1; i < m_endExits.size(); ++i) {
				pt = m_endExits[i];
				obsSum = getCongestionSum(pt, m_viaRadius, m_node_end->layer, m_layerGND);
				double congestionCostFactor = obsSum * m_decayFactor;
				if (congestionCostFactor < minCost) {
					bool canReach = isReachable(m_node_end->pos, pt, m_node_end->layer, m_startPad, m_endPad);
					if (canReach) {
						minCost = congestionCostFactor;
						rst = pt;
					}
				}
			}
			PathTree* newNode = addOneChild(m_node_end, rst, m_node_end->layer, nullptr, false);
			PathTree* viaInEndNode = addOneChild(newNode, rst, m_layerGND, nullptr, false);
			backTrackOnePath(viaInEndNode);
			m_GNDConnected.insert(m_node_end->pos);
		}
		// Maintain tree integrity
		m_node_start->addChild(m_node_end, 0, 0);  // Ensure one path corresponds to one tree
	}
	return true;
}
bool RouterMeshless::isTreeGNDConnected(PathTree* node) {
	if (!node) return false;

	// 1. The current node itself is on GND layer
	if (node->layer == m_layerGND)
		return true;

	// 2. The current node position has already been marked as grounded
	if (m_GNDConnected.find(node->pos) != m_GNDConnected.end())
		return true;

	// 3. Look up (same tree)
	PathTree* p = node->parent;
	while (p) {
		if (p->layer == m_layerGND)
			return true;
		if (m_GNDConnected.find(p->pos) != m_GNDConnected.end())
			return true;
		p = p->parent;
	}
	return false;
}
PathTree* RouterMeshless::nodeSelection() {
	//1. Select the optimal node from leaf nodes for expansion
	PathTree* selectedNode = m_leafNodesList.top();
	m_leafNodesList.pop();
	//2. Select the optimal target node from end point exit points
	m_ptToEnd = m_node_end->pos;
	if (m_endExits.empty()) {
		m_ptToEndCost = 0;
		return selectedNode;
	}
	const Point& pt_s = selectedNode->pos;
	const int& layer = selectedNode->layer;
	int layer1 = layer;
	int layer2 = m_node_end->layer;
	if (layer1 > layer2)
		swap(layer1, layer2);
	//2.1 Exit points are sorted by selection cost from smallest to largest
	int tryTimes = 0;
	multimap<double, Point> ptsToSelect;
	while (tryTimes++ < 2) {
		for (const auto& outPt : m_endExits) {
			bool canReach = isReachable(m_node_end->pos, outPt, layer, m_startPad, m_endPad);
			if (canReach) {
				if (!m_endLayers.contains(layer))
					canReach = isReachable(m_node_end->pos, outPt, m_node_end->layer, m_startPad, m_endPad);
			}
			if (!canReach) continue;
			// Calculate Manhattan distance
			Point sTopt = outPt - pt_s;
			double manhattanDistance = fabs(sTopt.x) + fabs(sTopt.y);
			// Calculate smoothness
			Point vectOut = (m_node_end->pos - outPt).normalizeVec();
			Point vectEnd1 = (outPt - pt_s).normalizeVec();
			double smoothness = vectOut * vectEnd1;
			if (selectedNode->parent) {
				Point vectEnd2 = (pt_s - selectedNode->parent->pos).normalizeVec();
				smoothness += vectEnd1 * vectEnd2;
			}
			// Calculate composite selection cost
			double cost = manhattanDistance - smoothness * manhattanDistance * m_standardCostFactor;
			ptsToSelect.insert(make_pair(cost, outPt));
		}
		//2.2 Select the point with the smallest cost (and reachable) as the target point
		if (!ptsToSelect.empty()) {
			m_ptToEndCost = ptsToSelect.begin()->first;
			m_ptToEnd = ptsToSelect.begin()->second;
			break;
		}
		else {
			for (const auto& outPt : m_endExits) {
				bool canVia = checkViaPos(outPt, layer1, layer2);
				if (canVia) break;
			}
			continue;
		}
	}
	return selectedNode;
}
void RouterMeshless::obssExploration(const PathTree* start) {
	//1. Get which cells are crossed
	const Line line(start->pos, m_ptToEnd);
	vector<GridCell*> cells;
	m_gridManager->getCellsAlongLine2(line, m_curNetInfo->width, cells);
	//2. Get same-layer polygon obstacles in cells (may cross), deduplicate, remove polygon obstacles where endpoints are located
	unordered_set<PolyShape*> obss;
	getValidObssInCells(cells, line, start->layer, obss);
	//3. Get intersecting obstacles (pads, traces)
	getCandidateObss(obss, line);
}
void RouterMeshless::getCandidateObss(const unordered_set<PolyShape*>& obss, const Line& line) {
	// Get intersecting obstacles (pads, traces)
	for (PolyShape* shape : obss) {
		if (shape->edges.empty()) continue;
		if (shape->isLine) {
			m_candidateObss.emplace(shape);
		}
		else {
			for (auto& edge : shape->edges) {
				Point pt1 = edge.p1->pos;
				Point pt2 = edge.p2->pos;
				Line shapeEdge(pt1, pt2);
				double dist = line.distanceToLine(shapeEdge);
				double minDist = m_curNetInfo->width / 2 + shape->clearance;  // max(m_curNetInfo->clearance, shape->clearance)
				if (dist < minDist - MapMinValue) {
					m_candidateObss.emplace(shape);
					break;
				}
			}
		}
	}
}
void RouterMeshless::getValidObssInCells(const vector<GridCell*>& cells, const Line& line, int layer, unordered_set<PolyShape*>& obss) {
	for (auto cell : cells) {
		//1 Candidate linear obstacles, remove duplicates (traces)
		for (auto& [pathline, shape] : cell->getPathLines()) {
			if (pathline->layer != layer) continue;		// Ignore lines on different layers
			if (pathline->p1->netName == m_curNetName) continue;
			Line shapeEdge(pathline->p1->pos, pathline->p2->pos);
			double dist = line.distanceToLine(shapeEdge);
			double minDist = m_curNetInfo->width / 2 + pathline->width / 2 + max(m_curNetInfo->clearance, shape->clearance);
			if (dist < minDist - MapMinValue) {
				obss.insert(shape);
				break;
			}
		}
		//2 Candidate polygon obstacles (pads)
		for (PinPad* pad : cell->getPinPads()) {
			if (!pad->shapes.contains(layer)) continue;		// Ignore shapes on different layers
			if (pad->netName == m_curNetName) continue;	// Ignore vias on this net
			//if (pad == m_startPad || pad == m_endPad) continue;
			PolyShape* obsShape = &pad->shapes[layer];
			if (obss.find(obsShape) == obss.end()) {
				if (abs(line.Pt1.x - line.Pt2.x) < MapMinValue) {
					// Vertical line, use bounding box filter to remove polygons on left and right sides
					if (pad->box[0] > line.Pt1.x || pad->box[2] < line.Pt1.x)
						continue;
				}
				else if (abs(line.Pt1.y - line.Pt2.y) < MapMinValue) {
					// Horizontal line, use bounding box filter to remove polygons on top and bottom sides
					if (pad->box[1] > line.Pt1.y || pad->box[3] < line.Pt1.y)
						continue;
				}
				obss.insert(obsShape);
			}
		}
	}
}
bool RouterMeshless::nodeExpansion(PathTree* start) {
	debugBreak(start);
	if (checkConnectToEnd(start))
		return true;
	while (!m_candidateObss.empty()) {
		PolyShape* obsShape = m_candidateObss.front();
		m_candidateObss.pop();
		m_queryObssPassed.clear();
		nodeExpansionWithObsShape(start, obsShape);
	}
	return false;
}
void RouterMeshless::nodeExpansionWithObsShape(PathTree* start, PolyShape* obsShape) {
	//1. Get blocking type (1 self-blocking, 2 intermediate blocking, 3 target blocking)
	int blockType = getBlockTypeToEndNode(obsShape, start);
	//2. Expand to child nodes
	switch (blockType) {
	case 1:		// Take planning points of neighbor nodes as child nodes
		setNeibNodeAsChild(start, obsShape); break;
	case 2:		// Take all planning points as child nodes
		setAllPtsAsChildren(start, obsShape); break;
	case 3:		// Skip
		break;
	default:
		break;
	}
}
bool RouterMeshless::checkConnectToEnd(PathTree* start) {
	if (m_candidateObss.empty()) {
		double stepG = start->pos.distanceTo(m_ptToEnd);
		PathTree* newNode = addOneChild(start, m_ptToEnd, start->layer, nullptr, false);

		if (newNode->layer == m_node_end->layer)				// Same layer, direct connection
			newNode->addChild(m_node_end, m_ptToEndCost, 0);
		else if (m_endLayers.contains(start->layer)) {			// Different layers, but allow layer change in place (end point includes this layer)
			PathTree* newEndNode_inLayer = new PathTree(m_node_end->pos, newNode->layer, m_curNetName, nullptr);
			newNode->addChild(newEndNode_inLayer, m_ptToEndCost, 0);
			newEndNode_inLayer->addChild(m_node_end, 0, 0);
		}
		else {			// Different layers, not allowed to change layer on-site (i.e., end pad doesn't contain this layer)
			PathTree* newEndNode_inLayer = new PathTree(m_ptToEnd, m_node_end->layer, m_curNetName, nullptr);
			newNode->addChild(newEndNode_inLayer, stepG, 0);	// Layer change node
			newEndNode_inLayer->addChild(m_node_end, 0, 0);
		}
		return true;
	}
	if (m_postOn && m_pinPairExchange) {
		// Try to change end point
		PinPad* newPad = checkNewST(start, false);
		if (newPad) {
			changeStartOrEnd(false, newPad, start->layer);
			double stepG = start->pos.distanceTo(m_endPad->pos);
			start->addChild(m_node_end, stepG, 0);
			return true;
		}
	}
	return false;
}
int RouterMeshless::getBlockTypeToEndNode(PolyShape* shape, PathTree* start)const {
	// Get block type (1-self block, 2-middle block, 3-target block)
	if (start->pNode) {		// Start point is planning point, may return 1, 2, 3
		if (start->pNode->shape == shape)
			return 1;
		else if (m_node_end->pNode && m_node_end->pNode->shape == shape)
			return 3;
		else
			return 2;
	}
	else {		// Start point is not planning point (start point is pad or via), may return 2, 3
		if (m_node_end->pNode && m_node_end->pNode->shape == shape)
			return 3;
		else
			return 2;
	}
}
void RouterMeshless::setNeibNodeAsChild(PathTree* start, PolyShape* shape) {
	if (!start->pNode) return;
	if (start->pNode->next) {	// Process next neighbor
		Point pos;
		if (getPlanningPos(start->pNode->next, pos, shape->isLine))
			connectToPos(start, pos, start->pNode->next, false);
	}
	if (start->pNode->prev) {	// Process previous neighbor
		Point pos;
		if (getPlanningPos(start->pNode->prev, pos, shape->isLine))
			connectToPos(start, pos, start->pNode->prev, false);
	}
}
void RouterMeshless::setAllPtsAsChildren(PathTree* start, PolyShape* shape) {
	// Add all planning points of target obstacle as child nodes (including polygon and linear obstacles)
	for (const PathLine& pline : shape->edges) {
		Point pos;
		if (getPlanningPos(pline.p1, pos, shape->isLine))
			connectToPos(start, pos, pline.p1, true);
	}
	if (shape->isLine) {
		PathNode* entPt = shape->edges.back().p2;
		Point pos;
		if (getPlanningPos(entPt, pos, true)) {
			connectToPos(start, pos, entPt, true);
		}
	}
}
bool RouterMeshless::connectToPos(PathTree* start, const Point& pos, PathNode* vertexNode, bool inserVia) {
	int layer = start->layer;
	// 1. cheap prune
	double G = 0, H = 0;
	getGHVia(start, pos, layer, G, H);
	PathTree* oldNode = posGetNode(pos, layer);
	if (oldNode && G >= oldNode->g)
		return false;
	// 2. Geometric reachability
	PolyShape* firstObs = nullptr;
	if (!isReachable(start->pos, pos, layer, m_startPad, m_endPad, &firstObs)) {
		if (firstObs && !m_queryObssPassed.contains(firstObs)) {
			m_queryObssPassed.insert(firstObs);
			nodeExpansionWithObsShape(start, firstObs);
		}
		return false;
	}

	// 3. State advancement (only entrance)
	addOneChild(start, pos, layer, vertexNode, inserVia);
	return true;
}
PathTree* RouterMeshless::posGetNode(const Point& pos, int layer) {
	auto posIter = m_exploredNodes.find(pos);
	if (posIter != m_exploredNodes.end()) {
		const auto& layerMap = posIter->second;
		auto layerIter = layerMap.find(layer);
		if (layerIter != layerMap.end()) {
			return layerIter->second;
		}
	}
	return nullptr;
}
bool RouterMeshless::getGHVia(PathTree* start, const Point& pos, int layer, double& G, double& H) {
	if (start->layer == layer) {
		if (start->pos == pos)
			G = start->g;
		else
			G = start->g + pos.distanceTo(start->pos);
		// Congestion cost
		int obsSum = getCongestionSum(pos, m_viaRadius, start->layer, layer);
		G = G + m_standartCost * obsSum * m_decayFactor;
		if (m_endLayers.contains(layer))
			H = pos.distanceTo(m_node_end->pos);
		else
			H = pos.distanceTo(m_node_end->pos) + m_standartCost;	// Via cost
		return false;	// Not via
	}
	else {
		if (start->pos == pos)
			G = start->g + m_standartCost;	// Via cost
		else {
			G = start->g + pos.distanceTo(start->pos) + m_standartCost;	// Via cost
			cout << "via has diffrent xy!" << endl;
		}
		int obsSum = getCongestionSum(pos, m_viaRadius, start->layer, layer);
		G = G + m_standartCost * obsSum * m_decayFactor;
		if (m_endLayers.contains(layer))
			H = pos.distanceTo(m_node_end->pos);
		else
			H = pos.distanceTo(m_node_end->pos) + m_standartCost;	// Via cost
		return true;	// Is via
	}

}
void RouterMeshless::updateExistingNode(PathTree* start, PathTree* oldNode, const double& G, const double& H) {
	start->updateChild(oldNode);

	// Update priority queue
	// Not implemented yet
}
PathTree* RouterMeshless::addOneChild(PathTree* start, const Point& pos, int layer, PathNode* vertexNode, bool allowVia) {
	double G = 0, H = 0;
	getGHVia(start, pos, layer, G, H);

	PathTree* oldNode = posGetNode(pos, layer);
	if (oldNode) {
		if (G >= oldNode->g)
			return nullptr;

		updateExistingNode(start, oldNode, G, H);
		return oldNode;
	}

	// New node
	PathTree* newNode = new PathTree(pos, layer, start->netName, vertexNode);
	m_exploredNodes[pos].insert({ layer, newNode });
	start->addChild(newNode, G, H);
	nodeOptimaze(newNode);
	m_leafNodesList.emplace(newNode);

	// Via expansion (Note: only for "new nodes")
	if (allowVia && start->pos.distanceTo(pos) > m_pathViaMinLength) {
		for (int otherLayer : m_routingLayers) {
			if (otherLayer == layer) continue;
			double viaG = 0, viaH = 0;
			getGHVia(newNode, pos, otherLayer, viaG, viaH);
			if (posGetNode(pos, otherLayer))
				continue;
			PathTree* viaNode = new PathTree(pos, otherLayer, start->netName, nullptr);
			m_exploredNodes[pos].insert({ otherLayer, viaNode });
			newNode->addChild(viaNode, viaG, viaH);
			nodeOptimaze(viaNode);
			m_leafNodesList.emplace(viaNode);
		}
	}

	return newNode;
}
int RouterMeshless::getCongestionSum(const Point& pos, double r, int layer1, int layer2 = -1) const {
	if (layer1 > layer2)
		swap(layer1, layer2);
	vector<GridCell*> cells;
	double minDistance = r + m_curNetInfo->clearance;
	double viaBound0 = pos.x - minDistance;
	double viaBound1 = pos.y - minDistance;
	double viaBound2 = pos.x + minDistance;
	double viaBound3 = pos.y + minDistance;
	m_gridManager->getCellsInBox(vector<double>{viaBound0, viaBound1, viaBound2, viaBound3}, cells);
	int obsSum = 0;
	for (auto cell : cells) {
		//2.1 Polygon obstacles (pads)
		for (PinPad* pad : cell->getPinPads()) {
			if (pad->netName == m_curNetName) continue;
			if (pad->box[0] > viaBound2 || pad->box[2] < viaBound0 || pad->box[1] > viaBound3 || pad->box[3] < viaBound1)
				continue;
			PolyShape* obsShape = nullptr;
			for (auto& [layer, shape] : pad->shapes) {
				if (layer >= layer1 && layer <= layer2) {
					obsShape = &shape;	// Find the shape of the layer through which the via passes
					break;
				}
			}
			if (obsShape) {
				// Find the nearest edge
				for (auto& edge : obsShape->edges) {
					double dist = pos.distanceToEdge(edge.p1->pos, edge.p2->pos);
					if (dist < minDistance) {
						obsSum += 1;
					}
				}
			}
		}
	}
	return obsSum;//obsSum > 0 ? obsSum - 1 : obsSum
}
void RouterMeshless::nodeOptimaze(PathTree* node) {
	if (!node || !node->parent || !node->parent->parent) return;  // No parent node, cannot optimize
	PathTree* curAncestor = node->parent->parent;  // Start from grandfather node
	PathTree* bestAncestor = nullptr;
	while (curAncestor) {
		if (node->layer != curAncestor->layer) {  // Not on the same layer
			curAncestor = curAncestor->parent;
			break;
		}
		//bool canReach = isReachable(node->pos, curAncestor->pos, node->layer, m_endPad);
		bool canReach = isReachable(node->pos, curAncestor->pos, node->layer, m_startPad, m_endPad);
		if (canReach) {
			if (curAncestor->pos != m_node_start->pos)
				bestAncestor = curAncestor;
			else
				break; // The second node is an exit point, cannot be optimized, exit loop directly
		}
		curAncestor = curAncestor->parent;
	}

	if (bestAncestor) {
		// 1. Remove current node from original parent node
		PathTree* oldParent = node->parent;
		oldParent->children.erase(node);
		node->parent = nullptr;

		// 2. Connect current node to best ancestor node
		double stepG = bestAncestor->pos.distanceTo(node->pos);
		double estimateH = node->h;  // Keep the original h value
		bestAncestor->addChild(node, stepG, estimateH);
	}
}
PolyShape* RouterMeshless::getFirstShape2(const Point& p1, const Point& p2, int layer, PinPad* ignorePad2) {
	//1. Get which cells are crossed
	vector<GridCell*> cells;
	Line line(p1, p2);
	m_gridManager->getCellsAlongLine2(line, m_curNetInfo->width, cells);
	unordered_set<PolyShape*> obss;

	//2. Get same-layer obstacles in cells (may cross), remove duplicates and obstacles at endpoints
	const NetInfo& netInfo = m_netsInfos->at(m_curNetName);
	double halfLineWidth = netInfo.width / 2;
	for (auto cell : cells) {
		for (PinPad* pad : cell->getPinPads()) {
			if (!pad->shapes.contains(layer)) continue;		// Pad not on this layer
			if (pad->netName == m_curNetName) continue;		// Ignore vias on this net
			if (ignorePad2 && pad == ignorePad2) continue;
			PolyShape* curShape = &pad->shapes[layer];
			if (!obss.contains(curShape)) {
				double distToIgnore = halfLineWidth + max(netInfo.clearance, curShape->clearance);
				if (abs(line.Pt1.x - line.Pt2.x) < MapMinValue) {
					// Vertical line, use bounding box filter to remove polygons on left and right sides
					if (pad->box[0] > line.Pt1.x + distToIgnore || pad->box[2] < line.Pt1.x - distToIgnore)
						continue;
				}
				else if (abs(line.Pt1.y - line.Pt2.y) < MapMinValue) {
					// Horizontal line, use bounding box filter to remove polygons on top and bottom sides
					if (pad->box[1] > line.Pt1.y + distToIgnore || pad->box[3] < line.Pt1.y - distToIgnore)
						continue;
				}
				obss.insert(curShape);
			}
		}
	}

	//3. Get intersecting obstacles (polygon pad obstacles)
	double nearistObsDist = line.getLength();
	PolyShape* firstObs = nullptr;
	for (PolyShape* shape : obss) {
		if (shape->edges.empty()) continue;
		for (auto& edge : shape->edges) {
			Point pt1 = edge.p1->pos;
			Point pt2 = edge.p2->pos;
			Line shapeEdge(pt1, pt2);
			double minDist = halfLineWidth + max(netInfo.clearance, shape->clearance);    // Minimum allowed distance
			double dist = line.distanceToLine(shapeEdge);        // Distance from exploration line to obstacle
			if (dist < minDist - MapMinValue) {    // Blocked
				Point crossingPoint = line.getCrossingPoint(shapeEdge);
				double startPtToObsDist = line.Pt1.distanceTo(crossingPoint);
				if (startPtToObsDist < nearistObsDist) {
					nearistObsDist = startPtToObsDist;
					firstObs = shape;
				}
			}
		}
	}

	//4. Get intersecting obstacles (linear obstacles)
	for (auto cell : cells) {
		for (auto& [pathline, shape] : cell->getPathLines()) {
			if (pathline->layer != layer) continue;        // Ignore lines on different layers
			// Linear obstacles under the same net
			if (pathline->p1->netName == m_curNetName) continue;

			Line shapeEdge(pathline->p1->pos, pathline->p2->pos);
			double dist = line.distanceToLine(shapeEdge);
			double minDist = netInfo.width / 2 + pathline->width / 2 + max(netInfo.clearance, shape->clearance);
			if (dist < minDist - MapMinValue) {
				Point crossingPoint = line.getCrossingPoint(shapeEdge);
				double startPtToObsDist = line.Pt1.distanceTo(crossingPoint);
				double startPtToObsDist2 = line.Pt1.distanceToEdge(pathline->p1->pos, pathline->p2->pos);
				if (startPtToObsDist2 < nearistObsDist) {
					nearistObsDist = startPtToObsDist;
					firstObs = shape;
				}
			}
		}
	}

	return firstObs;
}
bool RouterMeshless::isReachable2(const Point& p1, const Point& p2, int layer, PinPad* ignorePad2, PolyShape** firstObsPtr) {
	// Get the first obstacle from start to end point
	ignorePad2 = nullptr;
	PolyShape* obs = getFirstShape(p1, p2, layer, ignorePad2);
	if (firstObsPtr)
		*firstObsPtr = obs;
	// If there are no obstacles, or the obstacle is ignorePad2 (end point pad), then it is reachable
	return (!obs || (ignorePad2 && ignorePad2->shapes.contains(layer) && obs == &ignorePad2->shapes[layer]));
}
PolyShape* RouterMeshless::getFirstShape(const Point& p1, const Point& p2, int layer, PinPad* ignorePad, PinPad* ignorePad2) {
	//1. Get which cells are crossed
	vector<GridCell*> cells;
	Line line(p1, p2);
	m_gridManager->getCellsAlongLine2(line, m_curNetInfo->width, cells);
	unordered_set<PolyShape*> obss;

	//2. Get same-layer obstacles in cells (may cross), deduplicate, remove obstacles where endpoints are located
	double halfLineWidth = m_curNetInfo->width / 2;
	for (auto cell : cells) {
		for (PinPad* pad : cell->getPinPads()) {
			if (!pad->shapes.contains(layer)) continue;		// Pad not on this layer
			if (pad->netName == m_curNetName) continue;		// Ignore vias on this net
			if (ignorePad && pad == ignorePad) continue;	// Ignore this pad
			if (ignorePad2 && pad == ignorePad2) continue;
			PolyShape* curShape = &pad->shapes[layer];
			if (!obss.contains(curShape)) {
				double distToIgnore = halfLineWidth + max(m_curNetInfo->clearance, curShape->clearance);
				if (abs(line.Pt1.x - line.Pt2.x) < MapMinValue) {
					// Vertical line, use bounding box filter to remove polygons on left and right sides
					if (pad->box[0] > line.Pt1.x + distToIgnore || pad->box[2] < line.Pt1.x - distToIgnore)
						continue;
				}
				else if (abs(line.Pt1.y - line.Pt2.y) < MapMinValue) {
					// Horizontal line, use bounding box filter to remove polygons on top and bottom sides
					if (pad->box[1] > line.Pt1.y + distToIgnore || pad->box[3] < line.Pt1.y - distToIgnore)
						continue;
				}
				obss.insert(curShape);
			}
		}
	}

	//3. Get intersecting obstacles (polygon pad obstacles)
	double nearistObsDist = line.getLength() + halfLineWidth * 2 + m_curNetInfo->clearance;
	PolyShape* firstObs = nullptr;
	for (PolyShape* shape : obss) {
		if (shape->edges.empty()) continue;
		double minDist = halfLineWidth + shape->clearance;    // Minimum allowed distance max(m_curNetInfo->clearance, shape->clearance)
		for (auto& edge : shape->edges) {
			Point pt1 = edge.p1->pos;
			Point pt2 = edge.p2->pos;
			Line shapeEdge(pt1, pt2);
			double dist = line.distanceToLine(shapeEdge);        // Distance from exploration line to obstacle
			if (dist < minDist - MapMinValue) {    // Blocked
				Point crossingPoint = line.getCrossingPoint(shapeEdge);
				double startPtToObsDist = line.Pt1.distanceTo(crossingPoint);
				if (startPtToObsDist < nearistObsDist) {
					nearistObsDist = startPtToObsDist;
					firstObs = shape;
				}
			}
		}
	}

	//4. Get intersecting obstacles (linear obstacles)
	for (auto cell : cells) {
		for (auto& [pathline, shape] : cell->getPathLines()) {
			if (pathline->layer != layer) continue;        // Ignore lines on different layers
			// Linear obstacles under the same net
			if (pathline->p1->netName == m_curNetName) continue;

			Line shapeEdge(pathline->p1->pos, pathline->p2->pos);
			double dist = line.distanceToLine(shapeEdge);
			double minDist = m_curNetInfo->width / 2 + pathline->width / 2 + max(m_curNetInfo->clearance, shape->clearance);
			if (dist < minDist - MapMinValue) {
				Point crossingPoint = line.getCrossingPoint(shapeEdge);
				double startPtToObsDist = line.Pt1.distanceTo(crossingPoint);
				double startPtToObsDist2 = line.Pt1.distanceToEdge(pathline->p1->pos, pathline->p2->pos);
				if (startPtToObsDist2 < nearistObsDist) {
					nearistObsDist = startPtToObsDist;
					firstObs = shape;
				}
			}
		}
	}
	return firstObs;
}
bool RouterMeshless::isReachable(const Point& p1, const Point& p2, int layer, PinPad* ignorePad1, PinPad* ignorePad2, PolyShape** firstObsPtr) {
	// Get the first obstacle from start to end point
	PolyShape* obs = getFirstShape(p1, p2, layer, ignorePad1, ignorePad2);
	if (firstObsPtr)
		*firstObsPtr = obs;
	// If there are no obstacles, or the obstacle is ignorePad2 (end point pad), then it is reachable
	return (!obs || (ignorePad2 && ignorePad2->shapes.contains(layer) && obs == &ignorePad2->shapes[layer]));
}
bool RouterMeshless::checkPushLine(PathNode* M, PathNode* N, const Point pushVec) {
	Point p1 = M->pos + pushVec;
	Point p2 = N->pos + pushVec;
	int layer = N->layer;
	//1. Get which cells are crossed
	vector<GridCell*> cells;
	Line line(p1, p2);
	m_gridManager->getCellsAlongLine2(line, m_curNetInfo->width, cells);
	unordered_set<PolyShape*> obss;
	//2. Get same-layer obstacles in cells (may cross), deduplicate, remove obstacles where endpoints are located
	const NetInfo& netInfo = m_netsInfos->at(M->netName);
	double halfLineWidth = netInfo.width / 2;
	for (auto cell : cells) {
		for (PinPad* pad : cell->getPinPads()) {
			if (!pad->shapes.contains(layer)) continue;		// Pad not on this layer
			if (pad->netName == M->netName) continue;		// Ignore vias on this net
			PolyShape* curShape = &pad->shapes[layer];
			if (!obss.contains(curShape)) {
				double distToIgnore = halfLineWidth + max(netInfo.clearance, curShape->clearance);
				if (abs(line.Pt1.x - line.Pt2.x) < MapMinValue) {
					// Vertical line, use bounding box filter to remove polygons on left and right sides
					if (pad->box[0] > line.Pt1.x + distToIgnore || pad->box[2] < line.Pt1.x - distToIgnore)
						continue;
				}
				else if (abs(line.Pt1.y - line.Pt2.y) < MapMinValue) {
					// Horizontal line, use bounding box filter to remove polygons on top and bottom sides
					if (pad->box[1] > line.Pt1.y + distToIgnore || pad->box[3] < line.Pt1.y - distToIgnore)
						continue;
				}
				obss.insert(curShape);
			}
		}
	}
	//3. Get intersecting obstacles (polygon pad obstacles)
	double nearistObsDist = line.getLength();
	for (PolyShape* shape : obss) {
		if (shape->edges.empty()) continue;
		for (auto& edge : shape->edges) {
			Point pt1 = edge.p1->pos;
			Point pt2 = edge.p2->pos;
			Line shapeEdge(pt1, pt2);
			double minDist = halfLineWidth + max(netInfo.clearance, shape->clearance);    // Minimum allowed distance
			double dist = line.distanceToLine(shapeEdge);        // Distance from exploration line to obstacle
			if (dist < minDist - MapMinValue) {    // Blocked
				return false;
			}
		}
	}
	//4. Get intersecting obstacles (linear obstacles)
	for (auto cell : cells) {
		for (auto& [pathline, shape] : cell->getPathLines()) {
			if (pathline->layer != layer) continue;        // Ignore lines on different layers
			// Linear obstacles under the same net
			if (pathline->p1->netName == M->netName) continue;
			Line shapeEdge(pathline->p1->pos, pathline->p2->pos);
			double dist = line.distanceToLine(shapeEdge);
			double minDist = netInfo.width / 2 + pathline->width / 2 + max(netInfo.clearance, shape->clearance);
			if (dist < minDist - MapMinValue) {
				return false;
			}
		}
	}
	return true;
}
void RouterMeshless::setSEViasNode(PathNode* head, PathNode* tail) {
	// Path must have at least three nodes to determine via position
	if (!head || !head->next || !head->next->next) return;
	if (!tail || !tail->prev || !tail->prev->prev) return;
	//1. Layer change coordinates at start position
	if (head->pos == head->next->pos) {
		if (m_startPad->shapes.contains(head->next->layer)) {
			head->layer = head->next->layer;
		}
		else {
			// Need to insert via at start point
			head->next->pos = head->next->next->pos;
			head->next->layer = head->layer;
		}
	}
	//2. Layer change coordinates at end position
	if (tail->pos == tail->prev->pos) {
		if (m_endPad->shapes.contains(tail->prev->layer)) {
			tail->layer = tail->prev->layer;
		}
		else {
			// Need to insert via at end point
			tail->prev->pos = tail->prev->prev->pos;
			tail->prev->layer = tail->layer;
		}
	}
}
void RouterMeshless::insertVias(PathNode* head, PathNode* tail) {
	PathNode* cur = head;
	while (cur && cur->next) {
		if (cur->pos == cur->next->pos) {
			insertOneVia(cur);
		}
		cur = cur->next;
	}
}
void RouterMeshless::insertOneVia(PathNode* viaPre) {
	if (!viaPre || !viaPre->next) {  // || viaPre->pos != viaPre->next->pos
		cout << "InsertOneVia failed!" << endl;
		return;
	}
	if (viaPre->pos == m_node_start->pos) {
		// If it's a layer change at the pad, the actual path may not need layer change (pad contains this layer)
		if (m_startLayers.contains(viaPre->next->layer)) {
			viaPre->layer = viaPre->next->layer;
		}
		else { // Layer change at pad not feasible because pad doesn't contain this layer
			cout << "move start via failed:" << viaPre->pos << endl;
		}
		return;
	}
	else if (viaPre->pos == m_node_end->pos) {
		// If it's a layer change at the pad, the actual path may not need layer change (pad contains this layer)
		if (m_endLayers.contains(viaPre->layer)) {
			viaPre->next->layer = viaPre->layer;
		}
		else {
			cout << "move end via failed:" << viaPre->pos << endl;

		}
		return;
	}
	int layer1 = viaPre->layer;
	int layer2 = viaPre->next->layer;
	if (layer1 > layer2)
		swap(layer1, layer2);
	if (m_viaPush) {
		bool canVia = checkViaPos(viaPre->pos, layer1, layer2);		// Check via position and push other interfering lines
		if (!canVia)
			canVia = pushViaAndLine(viaPre, layer1, layer2);		// Push via and its associated line
	}
	Point pos = viaPre->pos;
	m_vias.insert(make_pair(pos, PinPad(pos, m_curNetInfo->viaName, m_curNetName)));
	m_viasSum++;
	PinPad& onePad = m_vias[pos];
	for (int i = layer1; i <= layer2; i++) {
		onePad.addShape(i, m_viaRadius, { 0,0 });
	}
	m_gridManager->addPinPad(&onePad);
}
bool RouterMeshless::checkViaPos(const Point& pos, int layer1, int layer2) {
	bool canVia = true;
	vector<GridCell*> cells;
	double minDistance = m_viaRadius + m_curNetInfo->clearance;
	double viaBound0 = pos.x - minDistance;
	double viaBound1 = pos.y - minDistance;
	double viaBound2 = pos.x + minDistance;
	double viaBound3 = pos.y + minDistance;
	m_gridManager->getCellsInBox(vector<double>{viaBound0, viaBound1, viaBound2, viaBound3}, cells);
	for (auto cell : cells) {
		//2.2 Candidate line obstacles (traces)
		int pushTimies = 0;
		bool tryPushLine = true;
		while (tryPushLine) {
			canVia = true;
			if (pushTimies++ > 5)
				break;
			tryPushLine = false;
			std::vector<std::pair<PathLine*, PolyShape*>> pathLinesSnapshot;
			pathLinesSnapshot.reserve(cell->getPathLines().size());
			for (auto& kv : cell->getPathLines()) {
				pathLinesSnapshot.emplace_back(kv);
			}
			for (auto& [pathline, shape] : pathLinesSnapshot) {
				if (!pathline || !shape) continue;
				if (pathline->layer < layer1 || pathline->layer > layer2) continue;     // Ignore lines on different layers
				if (pathline->p1->netName == m_curNetName) continue;                    // Linear obstacles under the same net
				double dist = pos.distanceToEdge(pathline->p1->pos, pathline->p2->pos);
				double minDist = m_viaRadius + pathline->width / 2 + max(m_curNetInfo->clearance, shape->clearance);
				if (dist < minDist - MapMinValue) {
					if (m_viaPush) {
						canVia = false;
						PathNode* M = pathline->p1, * N = pathline->p2;
						while (N && N->pos == M->pos) N = N->next;
						if (!N || M->pos == N->pos) continue;
						Point pushVec = (M->pos - N->pos).rotate90().normalizeVec() * (minDist - dist);
						double distance1 = pos.distanceToEdge(M->pos + pushVec, N->pos + pushVec);
						double distance2 = pos.distanceToEdge(M->pos - pushVec, N->pos - pushVec);
						if (distance1 < distance2)
							pushVec = pushVec * (-1);
						bool canReach = checkPushLine(M, N, pushVec);
						if (canReach) {
							removeOnePathFromGrid(shape);
							PathNode* start = shape->edges.front().p1;
							PathNode* end = shape->edges.back().p2;
							removeVias(start, end);
							pushMoveLine(M, pushVec, shape);
							if (shape && !shape->edges.empty())
								addOnePathToGrid(shape);
							m_gridManager->getCellsInBox(vector<double>{viaBound0, viaBound1, viaBound2, viaBound3}, cells);
							resetVias(start, end);
							tryPushLine = true;
						}
						else {
							pushVec = pushVec * (-1);
							canReach = checkPushLine(M, N, pushVec);
							if (canReach) {
								removeOnePathFromGrid(shape);
								PathNode* start = shape->edges.front().p1;
								PathNode* end = shape->edges.back().p2;
								removeVias(start, end);
								pushMoveLine(M, pushVec, shape);
								if (shape && !shape->edges.empty())
									addOnePathToGrid(shape);
								m_gridManager->getCellsInBox(vector<double>{viaBound0, viaBound1, viaBound2, viaBound3}, cells);
								resetVias(start, end);
								tryPushLine = true;
							}
						}
						if (tryPushLine)
							break;
					}
					else {
						return false;
					}
				}
			}
		}
		//2.1 Polygon obstacles (pads)
		for (PinPad* pad : cell->getPinPads()) {
			if (pad->box[0] > viaBound2 || pad->box[2] < viaBound0 || pad->box[1] > viaBound3 || pad->box[3] < viaBound1)
				continue;
			if (pad->netName == m_curNetName)
				continue;
			PolyShape* obsShape = nullptr;
			for (auto& [layer, shape] : pad->shapes) {
				if (layer >= layer1 && layer <= layer2) {
					obsShape = &shape;	// Find the shape of the layer through which the via passes
					break;
				}
			}
			if (obsShape) {
				// Find the nearest edge
				for (auto& edge : obsShape->edges) {
					double dist = pos.distanceToEdge(edge.p1->pos, edge.p2->pos);
					if (dist < minDistance - MapMinValue) {
						return false;
					}
				}
			}
		}
	}
	return canVia;
}
bool RouterMeshless::pushViaAndLine(PathNode* nodePre, int layer1, int layer2) {
	if (!nodePre->next) return false;
	bool isStart = !nodePre->prev;
	bool isEnd = !nodePre->next->next;
	if (isStart && isEnd) return false;
	if (isStart && !isEnd) {
		return moveSEVia(nodePre, layer1, layer2, isStart);;
	}
	if (isEnd && !isStart) {
		return moveSEVia(nodePre, layer1, layer2, isStart);;
	}
	double moveLength = m_viaRadius - 0.5 * m_curNetInfo->width;
	vector<Point> offsets = {
		Point(moveLength,0),
		Point(0,moveLength),
		Point(-moveLength,0),
		Point(0,-moveLength),
		Point(moveLength,moveLength),
		Point(-moveLength,moveLength),
		Point(-moveLength,-moveLength),
		Point(moveLength,-moveLength) };
	for (const Point& offset : offsets) {
		Point pVia = nodePre->pos + offset;
		// Take the line segment behind or in front for pushing
		Point p2 = nodePre->next->next->pos + offset;
		double projLength2 = fabs(offset * (pVia - p2).normalizeVec().rotate90());
		Point p0 = nodePre->prev->pos + offset;
		double projLength0 = fabs(offset * (pVia - p0).normalizeVec().rotate90());
		if (projLength2 < MapMinValue && projLength0 < MapMinValue) {
			// Collinear front and back, directly move via position
			bool viaCanReach = checkViaPos(pVia, layer1, layer2);
			//bool pViaInLine = pVia.ptInLine(p0, p2);
			if (viaCanReach) {  // && pViaInLine
				nodePre->pos = pVia;
				nodePre->next->pos = pVia;
				return true;
			}
		}
		else {
			// Not collinear, choose appropriate line for pushing
			bool viaCanReach = checkViaPos(pVia, layer1, layer2);
			if (viaCanReach) {
				if (projLength2 > projLength0) {
					//bool lineCanReach = isReachable(pVia, p2, viaPre->next->layer, m_endPad);
					bool lineCanReach = isReachable(pVia, p2, nodePre->next->layer, m_startPad, m_endPad);

					if (lineCanReach) {
						pushMoveLine(nodePre, offset);
						return true;
					}
					else {
						//lineCanReach = isReachable(pVia, p0, viaPre->prev->layer, m_endPad);
						lineCanReach = isReachable(pVia, p0, nodePre->prev->layer, m_startPad, m_endPad);
						if (lineCanReach) {
							pushMoveLine(nodePre->prev, offset);
							return true;
						}
					}
				}
				else {
					//bool lineCanReach = isReachable(pVia, p0, viaPre->prev->layer, m_endPad);
					bool lineCanReach = isReachable(pVia, p0, nodePre->prev->layer, m_startPad, m_endPad);
					if (lineCanReach) {
						pushMoveLine(nodePre->prev, offset);
						return true;
					}
					else {
						//lineCanReach = isReachable(pVia, p2, viaPre->prev->layer, m_endPad);
						lineCanReach = isReachable(pVia, p2, nodePre->prev->layer, m_startPad, m_endPad);
						if (lineCanReach) {
							pushMoveLine(nodePre, offset);
							return true;
						}
					}
				}
			}
		}
	}
	cout << "push via failed:" << nodePre->pos << endl;
	return false;
}
bool RouterMeshless::moveSEVia(PathNode* viaPre, int layer1, int layer2, bool isStart) {
	if (!viaPre || !viaPre->next)
		return false;
	Point dir;
	if (isStart) {
		if (!viaPre->next->next)
			return false;
		dir = (viaPre->pos - viaPre->next->next->pos).normalizeVec();
	}
	else {
		if (!viaPre->prev)
			return false;
		dir = (viaPre->pos - viaPre->prev->pos).normalizeVec();
	}
	Point offset = dir * m_viaRadius;
	Point pVia = viaPre->pos + offset;

	bool viaCanReach = checkViaPos(pVia, layer1, layer2);
	if (viaCanReach) {
		viaPre->pos = pVia;
		viaPre->next->pos = pVia;
		return true;
	}
	return false;
}

void RouterMeshless::backTrackOnePath(PathTree* nodeEnd) {
	PathTree* cur = nodeEnd;
	//2. Backtrack and generate path doubly linked list
	PolyShape* nullshape = nullptr;
	PathNode* tail = new PathNode(cur->pos, nullshape, nodeEnd->netName, cur->layer);
	PathNode* n1 = tail;
	while (cur->parent) {
		if (cur->parent->layer != cur->layer && cur->parent->pos != cur->pos) {
			// Not on the same layer and different coordinates, add a via
			PathNode* nVia = new PathNode(cur->pos, nullshape, cur->netName, cur->parent->layer);
			n1->insertBefore(nVia);
			n1 = nVia;
		}
		PathNode* n2 = new PathNode(cur->parent->pos, nullshape, nodeEnd->netName, cur->parent->layer);
		n1->insertBefore(n2);
		cur = cur->parent;
		n1 = n2;
	}
	PathNode* head = n1;
	//3. Set via nodes at start and end points, must be before mergeNode
	//setSEViasNode(head, tail);
	//3. Merge collinear segments
	mergeNode(head);
	//4. Post-processing 2
	if (m_postOn) {
		if (m_fixWireSpacingOn)
			fixWireSpacing(head);
		if (m_directionStandarlize)			//2. Enable: Trace direction standardization
			directionStandarlize(head);
		if (m_pushMove45Line)				//3. Enable: 45-degree line push
			pushMove45Line(head);
		if (m_cut90Angle)					//4. Enable: Sharp corner cutting
			cutAngle(head);
	}
	mergeNode(head);
	//5. Insert vias
	insertVias(head, tail);
	mergeNode(head);
	//6. Insert vias and generate path
	generateOnePath(head);
}

// Post-processing
PinPad* RouterMeshless::checkNewST(PathTree* curStart, bool changeStart) {
	if (!curStart)
		return nullptr;
	//1. Determine whether to check pin_start or pin_target
	PathTree* pinToChange = nullptr;
	PathTree* pinToHold = nullptr;
	vector<shared_ptr<SteinerNode>>* candidates = nullptr;
	double cost = 0.0;
	if (changeStart) {
		pinToChange = m_node_start;
		pinToHold = m_node_end;
		candidates = &m_startNeibs;
		cost = curStart->g;
	}
	else {
		pinToChange = m_node_end;
		pinToHold = m_node_start;
		candidates = &m_endNeibs;
		cost = curStart->h * m_changeSTFactor;
	}
	if (!pinToChange || !pinToHold)
		return nullptr;
	auto itChange = m_steinerQuery.find(pinToChange->pos);
	auto itHold = m_steinerQuery.find(pinToHold->pos);
	if (itChange == m_steinerQuery.end() || itHold == m_steinerQuery.end())
		return nullptr;
	shared_ptr<SteinerNode> steinerToChange = itChange->second;
	shared_ptr<SteinerNode> steinerToHold = itHold->second;

	//3. Traverse candidate newST
	for (auto& newST : *candidates) {
		PinPad* newPad = newST->pin;
		if (!newPad)
			continue;
		if (!newPad->shapes.contains(curStart->layer))
			continue;

		double stepG = curStart->pos.distanceTo(newPad->pos);
		if (stepG >= cost)
			continue;

		//bool canReach = isReachable(curStart->pos, newPad->pos, curStart->layer, newPad);
		bool canReach = isReachable(curStart->pos, newPad->pos, curStart->layer, m_startPad, newPad);
		if (!canReach)
			continue;
		return newPad;
		//================
		//auto parent1 = steinerToChange->parent.lock();
		//auto parent2 = steinerToHold->parent.lock();
		//cerr << "steinerToChange: " << steinerToChange.get()
		//	<< " parent: " << (parent1 ? parent1.get() : nullptr) << endl;
		//cerr << "steinerToHold: " << steinerToHold.get()
		//	<< " parent: " << (parent2 ? parent2.get() : nullptr) << endl;
		//===============
		//4. Dynamically modify Steiner tree topology
		//auto parentChange = steinerToChange->parent.lock();
		//auto parentHold = steinerToHold->parent.lock();

		//if (steinerToChange == parentHold) {
		//	// steinerToChange is the parent node
		//	newST->changeTopology(steinerToHold);
		//}
		//else if (steinerToHold == parentChange) {
		//	// steinerToChange is the child node
		//	steinerToHold->changeTopology(newST);
		//}
		//else if (parentChange == parentHold) {
		//	// Both share the same parent node, reconnect
		//	parentChange->removeChild(steinerToHold);
		//	steinerToChange->addChild(steinerToHold);
		//}
		//else if (!parentHold) {
		//	// steinerToHold has no parent node
		//	steinerToChange->addChild(steinerToHold);
		//}
		//else if (!parentChange) {
		//	// steinerToChange has no parent node
		//	steinerToHold->addChild(steinerToChange);
		//}
		//else {
		//	cerr << "Error! SteinerNode topology change failed in path:" << m_curPathIndex << endl;
		//	return nullptr;
		//}
		/*
		if (steinerToChange == steinerToHold->parent.lock()) {
			//steinerToChange is the parent node
			newST->changeTopology(steinerToHold);
		}
		else if (steinerToHold == steinerToChange->parent.lock()) {
			//steinerToChange is the child node
			steinerToHold->changeTopology(newST);
		}
		else {
			cerr << "Error! SteinerNode topology change failed in path:" << m_curPathIndex << endl;
			return nullptr;
		}
		/**/
		return newPad;
	}
	return nullptr;
}
void RouterMeshless::checkNewStartNode() {		//2. Backtracking process, pin exchange
	if (!m_node_end || !m_node_start) return;
	PathTree* cur = m_node_end;
	while (cur->parent) {
		PinPad* newPad = checkNewST(cur, true);
		if (newPad) {
			cur->parent->children.erase(cur);
			cur->parent = nullptr;
			unordered_set<PathTree*>& children = m_node_start->children;
			for (PathTree* childToDel : children)
				childToDel->remove();
			m_node_start->children.clear();
			// Update tree root node
			changeStartOrEnd(true, newPad, cur->layer);
			double stepG = cur->pos.distanceTo(newPad->pos);
			m_node_start->addChild(cur, stepG, -1);	// Set an arbitrary estimateH=-1 here, updated in the next line
			double viaCost = m_standartCost;
			m_node_start->updateTreefgh(viaCost);
			return;
		}
		cur = cur->parent;
	}
}
void RouterMeshless::fixWireSpacing(PathNode* head) {
	if (!head || !head->next || !head->next->next || !head->next->next->next) return;
	double minSelfDist = m_curNetInfo->clearance + m_curNetInfo->width;
	//1. Find two segments start and end that are too close and have the farthest indices
	PathNode* start = nullptr, * end = nullptr;
	PathNode* n1 = head, * n2 = head->next;
	PathNode* n3 = n2->next, * n4 = n3->next;
	bool needFix = true;
	while (needFix) {
		needFix = false;
		for (; n2 && n2->next; n1 = n2, n2 = n2->next) {
			if (n1->layer != n2->layer) continue;
			for (; n4; n3 = n4, n4 = n4->next) {
				if (n3->layer != n4->layer) continue;
				Line L1(n1->pos, n2->pos);
				Line L2(n3->pos, n4->pos);
				double dist = L1.distanceToLine(L2);
				if (dist < minSelfDist) {
					start = n1;
					end = n4;
				}
			}
			if (n1)
				break;
		}
		//2. Set the connection relationship of the two segments
		if (start && end) {
			if (start->next->next->next == end) {
				n1 = n2;
				n2 = n2->next;
				continue;	// Only separated by one line segment
			}
			else {
				Line L1(start->pos, start->next->pos);
				Line L2(end->prev->pos, end->pos);
				bool parallel = isParallel(L1.getVector().normalizeVec(), L2.getVector().normalizeVec());
				if (parallel) {
					// Delete nodes between start and end
					PathNode* cur = start->next;
					while (cur && cur != end) {
						PathNode* next = cur->next;
						cur->deleteCurruntNode(); // Safely delete current node
						cur = next;
					}
				}
				else {
					Point crossPt = L1.getCrossingPoint(L2);
					// Change start->next node coordinates to crossPt, delete subsequent nodes until end
					if (start->next) {
						start->next->pos = crossPt;
					}
					// Delete nodes from start->next->next to end
					PathNode* cur = start->next ? start->next->next : nullptr;
					while (cur && cur != end) {
						PathNode* next = cur->next;
						cur->deleteCurruntNode();
						cur = next;
					}
				}
				n1 = end;
				n2 = end->next;
				continue;
			}
		}
	}

}



void RouterMeshless::directionStandarlize(PathNode* head) {    // Direction standardization (for PathNode doubly linked list)
	if (!head || !head->next) return; // At least two nodes needed

	PathNode* cur = head->next;
	// Start from the head of the linked list, process each pair of adjacent nodes
	while (cur) {
		m_postTimes = 0;
		pathOptimaze(cur->prev, cur, cur->next);
		cur = cur->next;
	}
}
void RouterMeshless::pathOptimaze(PathNode* start, PathNode* end, PathNode* child) {
	m_postTimes++;
	Point midPos1, midPos2;
	Line line(start->pos, end->pos);

	if (lineMidPoss(line, midPos1, midPos2)) {
		// 2. Set midPos1 as the optimal solution (smoother solution)
		{
			// 2.1 Smoothing parameter for midPos1
			Point vec1_1 = midPos1 - start->pos;
			double length1 = vec1_1.vecLength();
			vec1_1 = vec1_1 / length1;
			Point vec1_2 = end->pos - midPos1;
			double length2 = vec1_2.vecLength();
			vec1_2 = vec1_2 / length2;
			double smooothness1_1 = 0;
			double smooothness1_2 = 0;

			// 2.2 Smoothing parameter for midPos2
			Point vec2_1 = midPos2 - start->pos;
			vec2_1 = vec2_1 / length2;
			Point vec2_2 = end->pos - midPos2;
			vec2_2 = vec2_2 / length1;
			double smooothness2_1 = 0;
			double smooothness2_2 = 0;

			// 2.3 Calculate the smoothness of the two schemes with previous neighbors
			if (start->prev) {
				Point vecPre = start->pos - start->prev->pos;
				vecPre = vecPre.normalizeVec();
				smooothness1_1 = vecPre * vec1_1;
				smooothness2_1 = vecPre * vec2_1;
			}
			else {
				Point vec_line = line.getVector();
				Point vec_line_x = Point(vec_line.x, 0) / abs(vec_line.x);
				Point vec_line_y = Point(0, vec_line.y) / abs(vec_line.y);
				smooothness1_1 = min(vec_line_x * vec1_1, vec_line_y * vec1_1);
				smooothness2_1 = min(vec_line_x * vec2_1, vec_line_y * vec2_1);
			}

			// 2.4 Calculate the smoothness of the two schemes with next neighbors
			if (child) {
				Point vecNext = child->pos - end->pos;
				vecNext = vecNext.normalizeVec();
				smooothness1_2 = vec1_2 * vecNext;
				smooothness2_2 = vec2_2 * vecNext;
			}
			else {
				Point vec_line = line.getVector();
				Point vec_line_x = Point(vec_line.x, 0) / abs(vec_line.x);
				Point vec_line_y = Point(0, vec_line.y) / abs(vec_line.y);
				smooothness1_2 = min(vec1_2 * vec_line_x, vec1_2 * vec_line_y);
				smooothness2_2 = min(vec2_2 * vec_line_x, vec2_2 * vec_line_y);
			}

			// 2.5 Calculate smoothness and set the optimal solution
			double smooothness1 = smooothness1_1 + smooothness1_2;    // Smoothness, larger is smoother
			double smooothness2 = smooothness2_1 + smooothness2_2;    // Smoothness, larger is smoother
			if (smooothness1 < smooothness2) {
				swap(midPos1, midPos2);
			}
		}

		// 3. Determine if direct reach is possible
		//bool canReach = isReachable(start->pos, midPos1, start->layer, m_endPad) &&
		//	isReachable(midPos1, end->pos, end->layer, m_endPad);
		bool canReach = isReachable(start->pos, midPos1, start->layer, m_startPad, m_endPad) &&
			isReachable(midPos1, end->pos, end->layer, m_startPad, m_endPad);
		if (canReach) {
			PathNode* midNode = new PathNode(midPos1, nullptr, start->netName, start->layer);
			start->insertAfter(midNode);
			return;
		}
		else {
			//canReach = isReachable(start->pos, midPos2, start->layer, m_endPad) &&
			//	isReachable(midPos2, end->pos, end->layer, m_endPad);
			canReach = isReachable(start->pos, midPos2, start->layer, m_startPad, m_endPad) &&
				isReachable(midPos2, end->pos, end->layer, m_startPad, m_endPad);
			if (canReach) {
				PathNode* midNode = new PathNode(midPos2, nullptr, start->netName, start->layer);
				start->insertAfter(midNode);
				return;
			}
		}

		// 4. Obstacles that need to be bypassed through a planning point
		Point ptToBypass;
		getPostPtsToPass(start, end, ptToBypass);

		// 5. Bypass intermediate obstacles and perform post-processing again
		bool passed = postPassObss(start, end, ptToBypass, child);
		if (!passed) {
			// Keep as is, do not insert new node
			cout << "Path [ " << m_curPathIndex << " ]:" << "\tError! post processing failed:" << start->pos << end->pos << endl;
		}
	}
}
void RouterMeshless::getPostPtsToPass(PathNode* start, PathNode* end, Point& ptToBypass) {
	Point& p1 = start->pos;
	Point& p2 = end->pos;
	Line line(p1, p2);
	double x1 = p1.x, y1 = p1.y;    // Bounding box
	double x2 = p2.x, y2 = p2.y;
	double minX = min(x1, x2) + MapMinValue;    // Shrink the effective planning point area to prevent adding the current vertex repeatedly
	double maxX = max(x1, x2) - MapMinValue;
	double minY = min(y1, y2) + MapMinValue;
	double maxY = max(y1, y2) - MapMinValue;

	// 1. Get which cells to search
	vector<GridCell*> cells;
	m_gridManager->getCellsInBox({ minX, minY, maxX, maxY }, cells);

	// 2. Get same-layer obstacles in cells: polygon obstacles, line obstacles
	unordered_set<PolyShape*> obss;
	int startLayer = start->layer;

	for (auto cell : cells) {
		// 2.1 Candidate polygons, deduplicate, remove polygon obstacles (pads) where endpoints are located
		for (PinPad* pad : cell->getPinPads()) {
			if (!pad->shapes.contains(startLayer)) continue;    // Ignore shapes on different layers
			PolyShape* obsShape = &pad->shapes[startLayer];

			// Ignore polygons where endpoints are located
			bool isStartPad = m_startPad->shapes.contains(start->layer) && obsShape == &m_startPad->shapes[start->layer];
			bool isEndPad = m_endPad->shapes.contains(end->layer) && obsShape == &m_endPad->shapes[end->layer];
			if (isStartPad || isEndPad)
				continue;
			if (obss.find(obsShape) == obss.end()) {
				obss.insert(obsShape);
			}
		}

		// 2.2 Candidate line obstacles, deduplicate (traces)
		for (auto& [pathline, shape] : cell->getPathLines()) {
			if (pathline->layer != startLayer) continue;        // Ignore lines on different layers
			// Line obstacles under the same net
			if (pathline->p1->netName == start->netName) continue;
			obss.insert(shape);
		}
	}

	// 3. Get planning points of candidate obstacles in the rectangular area (pads, lines)
	double minDistance = std::numeric_limits<double>::max();
	Point nearistPoint;
	for (PolyShape* shape : obss) {
		if (shape->edges.empty()) continue;
		for (auto& edge : shape->edges) {    // Same filtering logic for pad and line obstacles
			Point pos;
			if (getPlanningPos(edge.p1, pos, shape->isLine)) {
				if (pos.x >= minX && pos.x <= maxX && pos.y >= minY && pos.y <= maxY) {
					double pointToLineDistance = line.distanceToPoint(pos);
					if (pointToLineDistance < minDistance) {
						minDistance = pointToLineDistance;
						nearistPoint = pos;
					}
				}
			}
		}
	}

	// 4. Store in obssToPass in order
	ptToBypass = nearistPoint;
}
bool RouterMeshless::postPassObss(PathNode* start, PathNode* end, const Point& ptToBypass, PathNode* child) {
	// Get pads corresponding to start and end points
	//bool canReach = isReachable(start->pos, ptToBypass, start->layer, m_endPad) &&
	//	isReachable(ptToBypass, end->pos, end->layer, m_endPad);
	bool canReach = isReachable(start->pos, ptToBypass, start->layer, m_startPad, m_endPad) &&
		isReachable(ptToBypass, end->pos, end->layer, m_startPad, m_endPad);
	if (canReach) {
		PathNode* midNode = new PathNode(ptToBypass, nullptr, start->netName, start->layer);
		start->insertAfter(midNode);
		if (m_postTimes < 16) {
			pathOptimaze(start, midNode, end);
			pathOptimaze(midNode, end, child);
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}
bool RouterMeshless::lineMidPoss(const Line& line, Point& midPos1, Point& midPos2) {
	// Convert a diagonal line into two standard direction lines
	double x1 = line.Pt1.x, y1 = line.Pt1.y;
	double x2 = line.Pt2.x, y2 = line.Pt2.y;
	double dx = x2 - x1;
	double dy = y2 - y1;
	double abs_dx = abs(dx);
	double abs_dy = abs(dy);
	if (abs_dx < MapMinValue || abs_dy < MapMinValue || abs(abs_dx - abs_dy) < MapMinValue) {	// Horizontal, vertical, 45-degree traces, not processed
		return false;
	}
	if (abs_dy > abs_dx) {		// dy is larger
		int step_y = dy > 0 ? 1 : -1;
		// Processing method 1, walk horizontal and vertical lines near the start point
		midPos1.x = x1;
		midPos1.y = y2 - abs_dx * step_y;
		// Processing method 2, walk horizontal and vertical lines near the end point
		midPos2.x = x2;
		midPos2.y = y1 + abs_dx * step_y;
	}
	else {		// dx is larger
		int step_x = dx > 0 ? 1 : -1;
		// Processing method 1, walk horizontal and vertical lines near the start point
		midPos1.y = y1;
		midPos1.x = x2 - abs_dy * step_x;
		// Processing method 2, walk horizontal and vertical lines near the end point
		midPos2.y = y2;
		midPos2.x = x1 + abs_dy * step_x;
	}
	return true;
}

void RouterMeshless::pushMove45Line(PathNode* const head) {				//4. Push 45-degree line slightly
	//1. Determine the exit scheme according to pad type
}
void RouterMeshless::cutAngle(PathNode* const head) {					//5. Sharp angle clipping
	if (!head || !head->next) return;
	PathNode* cur = head->next;
	Line line1(cur->prev->pos, cur->pos);
	double length1 = line1.getLength();
	double cos45 = sqrt(2) / 2;
	while (cur->next) {
		Point p3 = cur->next->pos;
		Line line2(cur->pos, cur->next->pos);
		double length2 = line2.getLength();
		bool lengthValid = length1 > MapMinValue && length2 > MapMinValue;
		if (lengthValid) {
			Point vec1 = line1.getVector() / length1;
			Point vec2 = line2.getVector() / length2;
			double consAng = vec1 * vec2;
			bool is90Angle = abs(consAng) < MapMinValue;
			if (is90Angle) {
				cut90Angle(cur, vec1, vec2);
			}
			else {
				bool is135Angle = abs(consAng + cos45) < MapMinValue;	// Vector 135 degrees, routing line 45 degrees
				if (is135Angle) {
					cut45Angle(cur, vec1, vec2);
				}
			}
		}
		// Prepare for next iteration
		line1 = line2;
		length1 = length2;
		cur = cur->next;
	}
}
void RouterMeshless::cut90Angle(PathNode* cur, const Point& vec1, const Point& vec2) {

}
void RouterMeshless::cut45Angle(PathNode* cur, const Point& vec1, const Point& vec2) {

}
void RouterMeshless::generateOnePath(PathNode* const head) {
	//1. Generate path segment collection
	vector<PathLine> pathLines;
	double lineWidth = m_curNetInfo->width;
	PathNode* cur = head;
	while (cur->next) {
		cur->width = lineWidth;
		if (cur->layer != cur->next->layer && cur->pos != cur->next->pos) {
			cout << "layer error when generate the path:" << cur->pos << cur->next->pos << endl;
		}
		pathLines.emplace_back(cur, cur->next, cur->layer, lineWidth);  //m_curNetName
		cur = cur->next;
	}
	//2. Set path nodes where pads (pins) are exited
	//3. End point --> corresponding pad
	//4. End point --> which pad it connects to
	//5. Establish head node --> path shape mapping, tail node --> path shape mapping
	m_paths.insert(make_pair(head, PolyShape(pathLines, 0, true, m_curNetInfo->clearance, m_curNetName)));	// true indicates line obstacle
	PolyShape* pathShape = &m_paths[head];
	cur = head;
	while (cur->next) {
		cur->shape = pathShape;
		cur = cur->next;
	}
	//6. Set the planning point direction of the path
	pathShape->setDirection();
	//7. Add the path to the grid
	m_gridManager->addShapeLines(pathShape);
}

// Push algorithm
void RouterMeshless::pushMoveLine(PathNode* node, const Point& offset, PolyShape* shape) {
	if (!node || !node->next) return;
	// 1. Find the complete range of the pushed line segment
	PathNode* M = node;
	PathNode* N = node->next;
	while (N && N->pos == M->pos) N = N->next;
	if (!M || !N || M == N) return;

	// 2. Calculate push vector
	Point lineVec = N->pos - M->pos;
	if (lineVec.vecLength() < 0.0001) return;
	Point normal(-lineVec.y, lineVec.x);
	normal = normal.normalizeVec();
	double projLength = offset * normal;
	if (fabs(projLength) < 0.01) return;
	Point projOffset = normal * projLength;

	// 3.1 Calculate the affected node range and final coordinates (start point)
	Point tPos1 = M->pos + projOffset;
	Point tPos2 = N->pos + projOffset;
	getMPrevNodeToChange(projLength, normal, M);	// M-->M'
	if (M->prev)
		tPos1 = lineIntersection(tPos1, tPos2, M->pos, M->prev->pos);
	else {	//M is the first point
		M->insertAfter(new PathNode(M));
		M = M->next;
		setSEOutPos(tPos1, tPos2, M->pos, true);
	}
	// 3.1 Calculate the affected node range and final coordinates (end point)
	getNNextNodeToChange(projLength, normal, N);	// N-->N'
	if (N->next)
		tPos2 = lineIntersection(tPos1, tPos2, N->pos, N->next->pos);
	else {	//N is the last point
		N->insertBefore(new PathNode(N));
		N = N->prev;
		setSEOutPos(tPos1, tPos2, N->pos, false);
	}
	// 4. Set connection relationship
	setNodeConnections(M, N, tPos1, tPos2, shape);
}
void RouterMeshless::pushLineDataUpdate(const PolyShape* shapeCopy, PolyShape* pathShape) {
	vector<PathLine>& algPath = pathShape->edges;
	PathNode* start = algPath.front().p1;
	PathNode* end = algPath.back().p2;
	removeVias(start, end);
	removeOnePathFromGrid(pathShape);
	// Update algorithm data
	const vector<PathLine> pathInput = shapeCopy->edges;
	int erase_num = (int)(algPath.size() - pathInput.size());
	if (erase_num > 0) {
		PathNode* end = algPath.back().p2;
		algPath.erase(algPath.end() - erase_num, algPath.end());
		for (int i = 0; i < erase_num; ++i) {
			end->prev->removeFromList();
		}
		algPath.back().p2 = end;
	}
	else if (erase_num < 0) {
		PathNode* end = algPath.back().p2;
		PathNode* end_1 = algPath.back().p1;
		algPath.pop_back();
		size_t push_size = pathInput.size() - algPath.size();
		for (int i = erase_num; i < 0; ++i) {
			PathNode* end_insert = new PathNode({ 0,0 }, nullptr, end_1->netName, 0);
			end_1->insertAfter(end_insert);
			algPath.emplace_back(PathLine(end_1, end_insert, 0, 0));
			end_1 = end_insert;
		}
		algPath.emplace_back(PathLine(end_1, end, 0, 0));
	}
	algPath[0].p1->pos = pathInput[0].p1->pos;
	algPath[0].layer = pathInput[0].p1->layer;
	for (int i = 0; i < algPath.size(); ++i) {
		algPath[i].width = pathInput[i].width;
		algPath[i].layer = pathInput[i].layer;
		algPath[i].p2->pos = pathInput[i].p2->pos;
		algPath[i].p2->layer = pathInput[i].p2->layer;
	}
	pathShape->setDirection();
	addOnePathToGrid(pathShape);
	//// Update via position
	//if (!m_vias) return;
	//vector<CircleUI>& ui_vias = m_data->m_viaCircles;
	//ui_vias.clear();
	//for (const auto& [pos, pad] : *m_vias) {
	//	CircleUI ccPin(pos.x, pos.y, pad.r);
	//	ui_vias.emplace_back(ccPin);
	//}
	start = algPath.front().p1;
	end = algPath.back().p2;
	resetVias(start, end);
}
Point RouterMeshless::lineIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
	double x1 = p1.x, y1 = p1.y;
	double x2 = p2.x, y2 = p2.y;
	double x3 = p3.x, y3 = p3.y;
	double x4 = p4.x, y4 = p4.y;

	double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	if (fabs(denom) < 0.0001) {
		// Parallel or coincident, return midpoint
		return Point((x1 + x3) / 2, (y1 + y3) / 2);
	}

	double px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
	double py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

	return Point(px, py);
}
bool RouterMeshless::isParallel(const Point& v1, const Point& v2) {
	double v1LenSq = v1.x * v1.x + v1.y * v1.y;
	double v2LenSq = v2.x * v2.x + v2.y * v2.y;
	if (v1LenSq < MapMinValue || v2LenSq < MapMinValue)
		return true;
	double dot = v1.x * v2.x + v1.y * v2.y;
	double cosSq = (dot * dot) / (v1LenSq * v2LenSq);
	return cosSq > 0.998001;
}
bool RouterMeshless::isParallel(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
	return isParallel(p1 - p2, p3 - p4);
}
void RouterMeshless::setSEOutPos(Point& tPos1, Point& tPos2, const Point& SE, bool istPos1) {
	vector<Point> directions = {
		Point(1, 0),   // 0 degrees
		Point(1, 1),   // 45 degrees
		Point(0, 1),   // 90 degrees
		Point(-1, 1),  // 135 degrees
	};
	Point vec;
	if (istPos1)
		vec = (tPos1 - tPos2).normalizeVec();
	else
		vec = (tPos2 - tPos1).normalizeVec();
	vector<Point> intersections;
	for (const auto& dir : directions) {
		Point SE_end = SE + dir;
		if (abs(cross(dir, vec)) < MapMinValue)
			continue;	// Ignore parallel lines
		Point intersection = lineIntersection(tPos1, tPos2, SE, SE_end);
		intersections.emplace_back(intersection);
	}
	Point bestIntersection = Point((tPos1.x + tPos2.x) / 2, (tPos1.y + tPos2.y) / 2);
	double maxSmoothness = -MapMinValue;
	for (const Point& intersection : intersections) {
		Point vecNew = (SE - intersection).normalizeVec();
		double smoothness = vec * vecNew;
		if (smoothness > maxSmoothness) {
			maxSmoothness = smoothness;
			bestIntersection = intersection;
		}
	}
	if (istPos1)
		tPos1 = bestIntersection;
	else
		tPos2 = bestIntersection;
}

void RouterMeshless::removeVias(PathNode* node1, PathNode* node2) {
	if (!node1) return;
	while (node1->next && node1 != node2) {
		if (node1->pos == node1->next->pos) {
			auto it = m_vias.find(node1->pos);
			if (it != m_vias.end()) {
				PinPad& viaToRemove = it->second;
				m_gridManager->removePinPad(&viaToRemove);
				m_vias.erase(it);
			}
		}
		node1 = node1->next;
	}

}
void RouterMeshless::resetVias(PathNode* node1, PathNode* node2) {
	if (!node1) return;
	const string& netName = node1->netName;
	auto it = m_netsInfos->find(netName);
	if (it == m_netsInfos->end()) return;
	const NetInfo& netInfo = it->second;
	while (node1->next && node1 != node2) {
		if (node1->pos == node1->next->pos) {
			m_vias.insert(make_pair(node1->pos, PinPad(node1->pos, "viasX", netName)));
			PinPad& newVia = m_vias[node1->pos];
			int layer1 = node1->layer;
			int layer2 = node1->next->layer;
			if (layer1 > layer2) swap(layer1, layer2);
			for (int layer = layer1; layer <= layer2; layer++) {
				newVia.addShape(layer, m_viaRadius, { 0,0 });
			}
			m_gridManager->addPinPad(&newVia);
		}
		node1 = node1->next;
	}

}
void RouterMeshless::getConnectionPoints(PathNode* M, PathNode* N, PathNode*& D, PathNode*& E) {
	D = nullptr;
	E = nullptr;

	// Find D forward
	PathNode* curr = M;
	while (curr) {
		if (curr->prev && curr->prev->pos != curr->pos) {
			D = curr->prev;
			break;
		}
		curr = curr->prev;
	}

	// Find E backward
	curr = N;
	while (curr) {
		if (curr->next && curr->next->pos != curr->pos) {
			E = curr->next;
			break;
		}
		curr = curr->next;
	}
}
void RouterMeshless::getMPrevNodeToChange(const double& projLength, const Point& normal, PathNode*& M) {
	PathNode* nodeToChange = M->prev;
	PathNode* rst = M;
	while (nodeToChange) {
		Point vecMD = (nodeToChange->pos - M->pos);
		double prjMD = vecMD * normal;
		if (prjMD * projLength < -MapMinValue) {		// Push position reversed
			break;
		}
		else {
			if (abs(projLength) >= abs(prjMD) + MapMinValue) { 	// Push position exceeds nodeToChange
				rst = nodeToChange;
				nodeToChange = nodeToChange->prev;
			}
			else 	// Push position does not exceed nodeToChange
				break;
		}
	}
	M = rst;
}
void RouterMeshless::getNNextNodeToChange(const double& projLength, const Point& normal, PathNode*& N) {
	PathNode* nodeToChange = N->next;
	PathNode* rst = N;
	while (nodeToChange) {
		Point vecNE = (nodeToChange->pos - N->pos);
		double prjNE = vecNE * normal;
		if (prjNE * projLength < -MapMinValue) {
			break;
		}
		else {
			if (abs(projLength) >= abs(prjNE) + MapMinValue) {
				rst = nodeToChange;
				nodeToChange = nodeToChange->next;
			}
			else
				break;
		}
	}
	N = rst;
}
void RouterMeshless::setNodeConnections(PathNode* Node1, PathNode* Node2, const Point& tPos1, const Point& tPos2, PolyShape* shape) {
	if (!Node1 || !Node2)
		return;
	// Layer change nodes at endpoints
	PathNode* cur = Node1;
	Point origPos = Node1->pos;
	while (cur->pos == origPos) {
		cur->pos = tPos1;
		cur = cur->next;
	}
	cur = Node2;
	origPos = Node2->pos;
	while (cur->pos == origPos) {
		cur->pos = tPos2;
		cur = cur->prev;
	}
	cur = Node1;
	while (Node1 && Node1 != Node2->next) {
		// Calculate the projection point of the node to be modified onto the slideLine and determine the projection point position
		Point shadePos = Node1->pos.shadePointToLine(tPos1, tPos2);
		Point vec1 = tPos1 - shadePos;
		Point vec2 = tPos2 - shadePos;
		// If the projection point is within the line segment, move to the projection point; if not, move to the line segment endpoint
		if (vec1 * vec2 < MapMinValue)
			Node1->pos = shadePos;
		else if (vec1.vecLength() < vec2.vecLength())
			Node1->pos = tPos1;
		else
			Node1->pos = tPos2;
		Node1 = Node1->next;
	}
	// Merge collinear nodes
	while (cur->prev) cur = cur->prev;
	mergeNode(cur);
	// Update shapeEdges
	if (!shape || shape->edges.empty()) return;
	vector<PathLine>* pathLines = &shape->edges;
	if (!pathLines) return;
	double lineWidth = pathLines->at(0).width;
	pathLines->clear();
	while (cur->next) {
		if (cur->layer != cur->next->layer && cur->pos != cur->next->pos) {
			cout << "layer error in func [setNodeConnections]:" << cur->pos << cur->next->pos << endl;
		}
		pathLines->emplace_back(cur, cur->next, cur->layer, lineWidth);  //m_curNetName
		cur = cur->next;
	}
}