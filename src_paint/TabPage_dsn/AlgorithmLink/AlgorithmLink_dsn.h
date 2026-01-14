#pragma once
#include "../Data_dsn.h"
#include "../../../src_config/configUI.h"
#include "../../../src_algorithms/src_dsn/MST.h"
#include "../../../src_algorithms/src_dsn/RouterMeshless.h"

#include <string>

using namespace std;

class AlgorithmLink_dsn {
public:
	AlgorithmLink_dsn(Data_dsn* data, ConfigUI* config)
		:m_data(data), m_config(config) {
	}
	~AlgorithmLink_dsn() {
		//1. Release router
		delete m_router;
	}

	// 1.1 Pathfinding part
	void routingRunBegin();
	void setNetMST();
	void routingRun(const QString& qFileName = "");

private:
	// Read data
	Data_dsn* m_data;
	ConfigUI* m_config;
	SteinerTreeSolver m_solver;
	RouterMeshless* m_router = nullptr;

	// Algorithm input data (generated from m_data before algorithm initialization)
	unordered_map<string, shared_ptr<SteinerNode>> m_netTrees;		// Connection tree
	unordered_map<string, PinPad> m_pads;			// pad_name(pin_name) -> pad
	unordered_map<string, PinPad> m_preVias;		// pad_name(pin_name) -> pad
	vector<double> m_bound;
	unordered_map<string, ViaInfo> m_viaInfos;		// via_name -> via
	unordered_map<string, NetInfo> m_netsInfos;		// net_name -> Net table constraint information
	unordered_map<string, vector<PinPad*>> m_nets;

	// Algorithm execution results
	vector<vector<Line>> m_flyLines;
	vector<PathTree*>* m_treesHeads = nullptr;
	unordered_map<PathNode*, PolyShape>* m_pathsShapes = nullptr;
	unordered_set<Point, Point::Hash>* m_planningPts = nullptr;
	unordered_map<Point, PinPad, Point::Hash>* m_vias = nullptr;

private:
	void experment0(QTextStream* ofs);
	void experment5_2_1(QTextStream* ofs);
	void experment5_2_2(QTextStream* ofs);
	void experment5_3(QTextStream* ofs);
	void dataInit();
	void fillPaintFlyLines();
	void fillPaintPathLines();
	void addPreViasToUi(vector<CircleUI>& ui_vias);
	void fillPaintTrees();
	void fillPlanningPt();

public:
	bool pushLineRun(const QPointF& offset, const bool inGreed, bool algUpdate);
	void pushLineRunBegin(LineUI& line, vector<LineUI>& lines);			// Set algorithm data, starting point of algorithm execution
	bool pushPinRun(const QPointF& offset, bool inGreed = false) { return true; };
	void pushPinRunBegin(QPointF pt, vector<LineUI>& lines) {};			// Set algorithm data, starting point of algorithm execution
private:
	PathNode* m_selectPathPreNode = nullptr;
	QPointF m_selectPt;
	PolyShape* m_selectShape = nullptr;
	vector<LineUI>* m_ui_wires = nullptr;
	bool setPushLineData(PolyShape& shapeInput, PathNode*& nodeInput, Point& offsetInput, const QPointF& offsetUI);
	void pushLineUIDataUpdate(const PolyShape& shape);			// After algorithm execution ends, update front-end data
	bool setPushPinData(vector<Line>& polyLines_input, Point& pt_input, Point& offset_input, const QPointF& offset_ui) { return true; };

};