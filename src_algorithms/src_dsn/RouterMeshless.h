#pragma once
#include "MST.h"
#include "Grid.h"
#include "RoutingNode.h"
#include <stack>
#include <string>

class RouterMeshless {
public:
	RouterMeshless()
		:m_netTrees(nullptr), m_pads(nullptr), m_preVias(nullptr), m_viaInfos(nullptr), m_netsInfos(nullptr) {
	};
	RouterMeshless
	(
		unordered_map<string, shared_ptr<SteinerNode>>* netTrees,
		unordered_map<string, PinPad>* pads,
		unordered_map<string, PinPad>* preVias,
		vector<double>* bound,
		unordered_map<string, ViaInfo>* netViaInfos,
		unordered_map<string, NetInfo>* netInfos,
		unordered_map<string, vector<PinPad*>>* nets
	)
		: m_netTrees(netTrees), m_pads(pads), m_preVias(preVias), m_bound(*bound),
		m_viaInfos(netViaInfos), m_netsInfos(netInfos), m_nets(nets)
	{
		m_steinerQuery.clear();
		if (m_netTrees) {
			for (const auto& [netName, root] : *m_netTrees) {
				buildSteinerQuery(root);
			}
		}
	};
	~RouterMeshless() {
		freePathsAndTrees();
	};
	void run(vector<string>& routingInfo);
	void routerReset(double& gridSize);
	void pushMoveLine(PathNode* node, const Point& offset, PolyShape* shape = nullptr);
	void pushLineDataUpdate(const PolyShape* shapeCopy, PolyShape* pathShape);
	void addOnePathToGrid(PolyShape* shape) {
		m_gridManager->addShapeLines(shape);
	}
	void removeOnePathFromGrid(PolyShape* shape) {
		m_gridManager->removeOnePath(shape);
	}
	void setGrideSizeFactor(const double& alpha_g){ m_grideSizeFactor =  alpha_g;}
	void setStandardCostFactor(const double& beta) { m_standardCostFactor = beta; };
	void setRouterOption(const vector<bool>& boolOps) {
		size_t opNum = boolOps.size();
		if (opNum > 0) m_postOn = boolOps[0];
		if  (opNum > 1) m_GNDRoute = boolOps[1];
		if  (opNum > 2) m_VCCRoute = boolOps[2];
		if  (opNum > 3) m_DiffRoute = boolOps[3];
	};
	void setDebugOpt(const string& flexibleOpt, const bool& breakFunOn, const Point& pt, int breakID) {
		for (size_t i = 0; i < 6 && i < flexibleOpt.size(); ++i) {
			char c = flexibleOpt[i];
			bool value = (c != '0');  // All non-'0' characters are true, only '0' is false
			switch (i) {
			case 0: m_pinPairExchange = value; break;
			case 1: m_directionStandarlize = value; break;
			case 2: m_pushMove45Line = value; break;
			case 3: m_cut90Angle = value; break;
			case 4: m_viaPush = value; break;
			case 5:  m_fixWireSpacingOn = value; break;
			}
		}
		m_debugFunOn = breakFunOn;
		m_beeakPt = pt;
		m_breakIndex = breakID;
	};
	vector<PathTree*>* getTreesHeadsOrdered() { return &m_pathTreesOrdered; };
	unordered_map<PathNode*, PolyShape>* getPaths() { return &m_paths; };
	unordered_set<Point, Point::Hash>* getPlanningPts() { return &m_planningPts; };
	unordered_map<Point, PinPad, Point::Hash>* getVias() { return &m_vias; };

private:		// Debug data
	// 1.1 Algorithm execution options
	bool m_postOn = true;
	bool m_GNDRoute = true;
	bool m_VCCRoute = false;
	bool m_DiffRoute = false;
	// 1.2 Algorithm execution options, flexible definition
	bool m_pinPairExchange = true;		//1. Pin pair exchange
	bool m_directionStandarlize = true;	//2. Direction standardization
	bool m_pushMove45Line = true;		//3. 45-degree line push
	bool m_cut90Angle = true;			//4. Sharp corner cutting
	bool m_viaPush = true;				//5. Whether to enable push avoidance
	bool m_fixWireSpacingOn = true;		//6. Whether to enable wire spacing correction

	//2. Debug break parameters
	bool m_debugEnd = false;	// Interrupt routing and return routed results
	bool m_debugFunOn = false;
	Point m_beeakPt;
	int m_breakIndex = 0;
	bool debugBreak(const PathTree* const curNode) {
		return debugBreak(curNode->pos);
	}
	bool debugBreak(const Point& pos) {
		if (!m_debugFunOn)
			return false;
		if (m_breakIndex > 0 && m_breakIndex != m_curPathIndex)
			return false;
		double delta = 1;
		bool xInMid = pos.x > m_beeakPt.x - delta && pos.x < m_beeakPt.x + delta;
		bool yInMid = pos.y > m_beeakPt.y - delta && pos.y < m_beeakPt.y + delta;
		if (xInMid && yInMid) {
			cout << "debugBreak in path index:[ " << m_curPathIndex << " ]======================" << endl;
			m_debugEnd = true;
			return true;
		}
		else
			return false;
	}

private:
	// 1. Algorithm input data
	unordered_map<string, shared_ptr<SteinerNode>>* m_netTrees;		// net_name -> root
	unordered_map<Point, shared_ptr<SteinerNode>, Point::Hash> m_steinerQuery;
	unordered_map<string, PinPad>* m_pads;			// pad_name(pin_name) -> pad(all pads corresponding to pins or vias)
	unordered_map<string, PinPad>* m_preVias;		// pad_name(pin_name) -> pad
	vector<double> m_bound;
	unordered_map<string, ViaInfo>* m_viaInfos;		// via_name -> via
	unordered_map<string, NetInfo>* m_netsInfos;	// net_name -> via
	unordered_map<string, vector<PinPad*>>* m_nets;
	string m_NetNameGND = "GND";
	int m_layerGND = 0;

	// 2. Algorithm hyperparameters
	double m_grideSizeFactor = 1;				//alpha_g
	double m_standardCostFactor = 0.5;			//beta

	double rectLengthFactor = 2.5;				// If aspect ratio is greater than this value, it is considered a long strip pad, and routing can only be led out along the strip direction
	double m_congestionFactor = 0.1;			// Congestion cost coefficient, originally G*m_congestionFactor
	double m_decayFactor = 0.75;				// Density cost decay coefficient
	const double m_changeSTFactor = 0.15;		//beta_st
	double m_pathViaMinLength = 4;				// Minimum length for layer change edges

	// 3. Data continuously supplemented during algorithm execution
	unique_ptr<GridManager> m_gridManager;				// Spatial index manager
	unordered_map<Point, PinPad, Point::Hash> m_vias;	// Vias
	unordered_map<PathTree*, string> m_pathHeads;		// Netlist name corresponding to the path
	unordered_set<Point, Point::Hash> m_planningPts;	// Only used for UI drawing
	unordered_map<const PathTree*, PinPad*> m_PinQuaryPad;		// Pads corresponding to start/end nodes
	unordered_set<Point, Point::Hash> m_GNDConnected;		// Pins already connected to GND layer

	// 4. Data used for single path search
	double m_standartCost = 0;			// Distance between start and end points * m_standardCostFactor
	double m_viaRadius = 1;
	int m_serchTimesLimit = 1000;
	string m_curNetName;				// Current routing net
	NetInfo* m_curNetInfo;
	PathTree* m_node_start;
	PinPad* m_startPad;
	vector<shared_ptr<SteinerNode>> m_startNeibs;
	PathTree* m_node_end;
	PinPad* m_endPad;
	vector<shared_ptr<SteinerNode>> m_endNeibs;
	vector<Point> m_endExits;
	Point m_ptToEnd;
	double m_ptToEndCost = 0;
	unordered_set<int> m_routingLayers;	// Layers that can be routed
	unordered_set<int> m_startLayers;	// Layers where the start point is located
	unordered_set<int> m_endLayers;		// Layers where the end point is located
	int m_postTimes = 0;				// Current post-processing recursion count

	priority_queue<PathTree*, vector<PathTree*>, ComparePathTreePtr> m_leafNodesList;	// Leaf nodes to be expanded
	unordered_map<Point, unordered_map<int, PathTree*>, Point::Hash> m_exploredNodes;	// Explored planning points
	queue<PolyShape*> m_candidateObss;
	unordered_set<PolyShape*> m_queryObssPassed;
	unordered_set<Point, Point::Hash> m_querySTChanged;		// Start/end points that have been modified (passed)

	// 5. Statistical data (reset or modified during algorithm execution)
	int m_totalPins = 0;
	int m_curPathIndex = 0;		// Current path index being searched
	int m_searchTimes = 0;
	int m_pathFoundNum = 0;	// Statistical data
	unordered_map<string, bool> m_netFound;
	int m_viasSum = 0;
	double m_totalPathLength = 0;

	// 6. Algorithm execution results
	unordered_map<string, vector<PathTree*>> m_treesHeads;	// net_name->all paths, tree structure
	vector<PathTree*> m_pathTreesOrdered;					// Search tree root nodes (ordered)
	unordered_map<PathNode*, PolyShape> m_paths;			// Linked list node corresponding to the path start point, the path corresponding to this node (line segment collection)

private:
	function<bool(const pair<PinPad*, PinPad*>&, const pair<PinPad*, PinPad*>&)> m_priorityRule;
	// Default priority rule: sort by distance (shorter distance has higher priority)
	static bool defaultPriorityRule(const pair<PinPad*, PinPad*>& a, const pair<PinPad*, PinPad*>& b) {
		double distA = a.first->pos.distanceTo(a.second->pos);
		double distB = b.first->pos.distanceTo(b.second->pos);
		// First compare distances
		if (std::abs(distA - distB) > MapMinValue) {  // Considering floating-point precision
			return distA < distB;
		}
		double a_minX = std::min(a.first->pos.x, a.second->pos.x);
		double b_minX = std::min(b.first->pos.x, b.second->pos.x);
		if (std::abs(a_minX - b_minX) > MapMinValue) {
			return a_minX < b_minX;
		}
		double a_minY = std::min(a.first->pos.y, a.second->pos.y);
		double b_minY = std::min(b.first->pos.y, b.second->pos.y);
		if (std::abs(a_minY - b_minY) > MapMinValue) {
			return a_minY < b_minY;
		}
		return distA < distB;
	};
	void buildSteinerQuery(const std::shared_ptr<SteinerNode>& root) {
		if (!root) return;

		stack<std::shared_ptr<SteinerNode>> stk;
		stk.push(root);

		while (!stk.empty()) {
			auto node = stk.top();
			stk.pop();
			m_steinerQuery.emplace(node->position, node);
			for (const auto& child : node->children) {
				if (child) {
					stk.push(child);
				}
			}
		}
	}

private:
	// Data preparation and changes
	void freePathsAndTrees();
	void initializeGrid(const double& gridSize);      // Initialize grid
	void setSpecialNetInfomation(string netName);
	void extractPinPairs(const shared_ptr<SteinerNode>& steinerTree, vector<pair<PinPad*, PinPad*>>& pinPairs);
	void setOrderedPinPair(PinPad* pin1, PinPad* pin2, vector<pair<PinPad*, PinPad*>>& pinPairs);
	bool getPlanningPos(PathNode* node, Point& poss, bool isLine);
	void mergeNode(PathNode* node);
	bool findAllPaths(vector<pair<PinPad*, PinPad*>>& pinPairs);
	bool prepareOnePathData(pair<PinPad*, PinPad*>& pair);
	void setStartAndEnd();
	void setStartPad(int layer_s);
	void setEndPad(int layer_e);
	void setPadOuPts(PathTree* nodeToSet, PathTree* nodeToIgnore, double minClear, const vector<double>& box, int startIdx, int step, vector<Point>& outDirectionsPts);
	int getPinPadOutDirection(PathTree* const nodeSE, PinPad*& padptr);
	bool changeStartOrEnd(bool isStart, PinPad* pad, int layer);
	bool updateSteinerTopology(const shared_ptr<SteinerNode>& steinerToChange, const shared_ptr<SteinerNode>& steinerToHold, const shared_ptr<SteinerNode>& newST);
	double getPathsLength();
	PinPad* getPadPtr(const string& pinName) {
		if (m_pads->contains(pinName))
			return &m_pads->at(pinName);
		else if (m_preVias->contains(pinName))
			return &m_preVias->at(pinName);
		else
			return nullptr;
	};
	int getSpecialRoutingType();
	bool routeSpecially();

	//PPDT routing algorithm
	bool runPPDT();
	bool route_GND();
	bool isTreeGNDConnected(PathTree* node);
	PathTree* nodeSelection();
	void obssExploration(const PathTree* start);
	void getCandidateObss(const unordered_set<PolyShape*>& obss, const Line& line);
	void getValidObssInCells(const vector<GridCell*>& cells, const Line& line, int layer, unordered_set<PolyShape*>& obss);
	bool nodeExpansion(PathTree* start);
	void nodeExpansionWithObsShape(PathTree* start, PolyShape* obsShape);
	bool checkConnectToEnd(PathTree* start);
	int getBlockTypeToEndNode(PolyShape* shape, PathTree* start)const;
	void setNeibNodeAsChild(PathTree* start, PolyShape* shape);
	void setAllPtsAsChildren(PathTree* start, PolyShape* currentPad);
	bool connectToPos(PathTree* start, const Point& pos, PathNode* vertexNode, bool inserVia);
	PathTree* posGetNode(const Point& pos, int layer);
	bool getGHVia(PathTree* start, const Point& pos, int layer, double& G, double& H);
	void updateExistingNode(PathTree* start, PathTree* oldNode, const double& G, const double& H);
	PathTree* addOneChild(PathTree* start, const Point& pos, int layer, PathNode* vertexNode, bool allowVia);
	int getCongestionSum(const Point& pos, double r, int layer1, int layer2) const;
	void nodeOptimaze(PathTree* node);

	PolyShape* getFirstShape2(const Point& p1, const Point& p2, int layer, PinPad* ignorePad2 = nullptr);
	bool isReachable2(const Point& p1, const Point& p2, int layer, PinPad* ignorePad2, PolyShape** firstObsPtr = nullptr);

	PolyShape* getFirstShape(const Point& p1, const Point& p2, int layer, PinPad* ignorePad, PinPad* ignorePad2 = nullptr);
	bool isReachable(const Point& p1, const Point& p2, int layer, PinPad* ignorePad1, PinPad* ignorePad2, PolyShape** firstObsPtr = nullptr);


	bool checkPushLine(PathNode* M, PathNode* N, const Point pushVec);

	void setSEViasNode(PathNode* head, PathNode* tail);
	void insertVias(PathNode* head, PathNode* tail);
	void insertOneVia(PathNode* viaPre);
	bool checkViaPos(const Point& pos, int layer1, int layer2);
	bool pushViaAndLine(PathNode* nodePre, int layer1, int layer2);
	bool moveSEVia(PathNode* viaPre, int layer1, int layer2, bool isStart);
	void backTrackOnePath(PathTree* nodeEnd);

	// Post-processing
	const vector<int> shape_out_direction = { 1, 2, 2 };  // Circle, rectangle, nearly square: 1 for 8 directions, 2 for 4 directions, 4 for 2 directions
	const vector<Point> Direction8 = { Point(1,0),Point(sqrt(2) / 2,sqrt(2) / 2),		//0x, 1 existing, 2y...
			Point(0,1), Point(-sqrt(2) / 2,sqrt(2) / 2), Point(-1,0),
			Point(-sqrt(2) / 2,-sqrt(2) / 2), Point(0,-1),
			Point(sqrt(2) / 2,-sqrt(2) / 2) };  //8 standard directions
	PinPad* checkNewST(PathTree* curStart, bool changeEnd);
	//bool checkNewEndNode(PathTree* const start);	//1. Pin switching (growth process)
	void checkNewStartNode();						//2. Pin switching (backtracking process)
	void fixWireSpacing(PathNode* head);
	void directionStandarlize(PathNode* head);	    //4. Wiring direction standardization
	void pathOptimaze(PathNode* start, PathNode* end, PathNode* child);
	void getPostPtsToPass(PathNode* start, PathNode* end, Point& ptToBypass);
	bool postPassObss(PathNode* start, PathNode* end, const Point& ptToBypass, PathNode* child);
	bool lineMidPoss(const Line& line, Point& midPos1, Point& midPos2);

	void pushMove45Line(PathNode* const head);				//5. 45-degree line push
	void cutAngle(PathNode* const head);					//6. Sharp corner cutting
	void cut90Angle(PathNode* cur, const Point& vec1, const Point& vec2);
	void cut45Angle(PathNode* cur, const Point& vec1, const Point& vec2);
	void generateOnePath(PathNode* const head);


	// Push algorithm
	Point lineIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4);
	bool isParallel(const Point& v1, const Point& v2);
	bool isParallel(const Point& p1, const Point& p2, const Point& p3, const Point& p4);
	void setSEOutPos(Point& tPos1, Point& tPos2, const Point& SE, bool istPos1);
	void removeVias(PathNode* node1, PathNode* node2);
	void resetVias(PathNode* node1, PathNode* node2);
	void getConnectionPoints(PathNode* M, PathNode* N, PathNode*& D, PathNode*& E);
	void getMPrevNodeToChange(const double& projLength, const Point& normal, PathNode*& M);
	void getNNextNodeToChange(const double& projLength, const Point& normal, PathNode*& N);
	void setNodeConnections(PathNode* Node1, PathNode* Node2, const Point& tPos1, const Point& tPos2, PolyShape* shape);


private:
	string m_netNameBackup;
	void netNameChange(const string& netName) {
		m_netNameBackup = m_curNetName;
		m_curNetName = netName;
		m_curNetInfo = &m_netsInfos->at(m_curNetName);
	}
	void netNameReset() {
		m_curNetName = m_netNameBackup;
		m_curNetInfo = &m_netsInfos->at(m_curNetName);
	}


};


