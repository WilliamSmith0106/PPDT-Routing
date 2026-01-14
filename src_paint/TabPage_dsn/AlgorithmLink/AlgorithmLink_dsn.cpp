#include "AlgorithmLink_dsn.h"
#include "../../../src_algorithms/src_dsn/MST.h"
#include <chrono>

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileInfo>
#include <sstream> 
#include <iostream>

using namespace std;

inline QTextStream* openResultFile(const QString& rawFileName, QFile*& outFile) {
	QString dirPath = "data/result";
	QDir().mkpath(dirPath); // Ensure directory exists

	QFileInfo fi(rawFileName);
	QString finalName = QDateTime::currentDateTime().toString("yyyyMMddHHmmss")
		+ "_" + fi.completeBaseName() + ".txt";
	QString fullPath = dirPath + "/" + finalName;

	outFile = new QFile(fullPath);
	if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text)) {
		qWarning() << "Failed to open file:" << fullPath;
		delete outFile;
		outFile = nullptr;
		return nullptr;
	}
	QTextStream* ts = new QTextStream(outFile);
	ts->setGenerateByteOrderMark(true);
	return ts;
}


// Pathfinding
void AlgorithmLink_dsn::routingRunBegin() {
	//1. Data initialization
	dataInit();
	//2. Compute minimum spanning tree MST (fill m_netTrees, m_flyLines)
	setNetMST();
	//3. Output basic information
	int pinsSum1 = (int)m_pads.size();
	int pinsSum2 = 0;
	for (const auto& [netName, net] : m_nets) {
		pinsSum2 += net.size();
	}
	cout << "Min:(" << static_cast<int>(ceil(m_bound[0])) << ", " << static_cast<int>(ceil(m_bound[1])) << "),"
		<< "Max:(" << static_cast<int>(ceil(m_bound[2])) << ", " << static_cast<int>(ceil(m_bound[3])) << "),"
		<< "\tNet:" << m_nets.size() << ",\pinsSum:" << pinsSum1 << endl;
}
void AlgorithmLink_dsn::setNetMST() {
	m_preVias.clear();
	m_solver.computeSteinerTrees(m_nets, m_netsInfos, m_netTrees);
	if (m_config->m_preViaAlctOn) {		// Steiner tree, pre-via allocation
		m_solver.setSteinerNodes(m_netsInfos, m_viaInfos, m_netTrees, m_preVias);
	}
	m_solver.setFlyLines(m_netTrees, m_flyLines);
	fillPaintFlyLines();
	if (m_router) delete m_router;
	m_router = new RouterMeshless(&m_netTrees, &m_pads, &m_preVias, &m_bound, &m_viaInfos, &m_netsInfos, &m_nets);
}
void AlgorithmLink_dsn::routingRun(const QString& qFileName) {
	//1. Data passed from frontend to algorithm
	Point inputPt(m_config->m_doubleNum1, m_config->m_doubleNum2);
	string flexibleOpt = m_config->m_flexibleOpt.toLatin1().constData();
	//2. Router parameter settings
	vector<bool> boolOps = {
		m_config->m_postOn,
		m_config->m_GNDRouteOn,
		m_config->m_VCCRouteOn,
		m_config->m_diffRouteOn
	};
	m_router->setRouterOption(boolOps);
	m_router->setDebugOpt(flexibleOpt, m_config->m_debugFuncOn, inputPt, m_config->m_intNum3);
	//3. Record experimental data
	QFile* file = nullptr;
	QTextStream* ofs = openResultFile(qFileName, file);
	if (!ofs) return;  // File open failed

	int runExpermentID = 0;
	switch (runExpermentID) {
	case 0: experment0(ofs); break;
	case 1: experment5_2_1(ofs); break;
	case 2: experment5_2_2(ofs); break;
	case 3: experment5_3(ofs); break;
	}

	file->close();
	delete ofs;
	delete file;
	// Feedback to frontend for drawing
	m_treesHeads = m_router->getTreesHeadsOrdered();
	m_pathsShapes = m_router->getPaths();
	m_planningPts = m_router->getPlanningPts();
	m_vias = m_router->getVias();
	fillPaintPathLines();		// Draw path
	fillPlanningPt();
	fillPaintTrees();
	m_config->write_config();	// Update after each execution to prevent property loss due to interrupted debugging
}
void AlgorithmLink_dsn::experment0(QTextStream* ofs) {
	//run only once
	*ofs << "Layers\tPairs\tFound\tFailed\tRateP\tAvgLen\tVia\tRateN\t";
	*ofs << "Time\tgridSize\talpha_g\tbeta\n";
	//4. Router data cleaning, establish spatial grid index, generate initial planning points
	double alpha_g = 3.0;
	double beta = 0.4;
	m_router->setGrideSizeFactor(alpha_g);
	m_router->setStandardCostFactor(beta);
	double gridSize = m_config->m_gridSize;
	auto t1 = chrono::high_resolution_clock::now();
	m_router->routerReset(gridSize);
	vector<string> routingInfo;
	m_router->run(routingInfo);
	auto t2 = chrono::high_resolution_clock::now();
	//5. Output routing time
	auto time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
	cout << "Time:" << time << " ms" << ", gridSize:" << gridSize << ", alpha_g:" << alpha_g << ", beta:" << beta << endl;
	for (auto& info : routingInfo) {
		*ofs << QString::fromLocal8Bit(info.c_str()) << "\t";
	}
	*ofs << time << "\t" << gridSize << "\t" << alpha_g << "\t" << beta << "\n";
	cout << "\n\n\n\n==================================================================================================" << endl;
}
void AlgorithmLink_dsn::experment5_2_1(QTextStream* ofs) {
	int test_times = 10;
	vector<double> alpha_gs = { 0.1,1,2,3,4,5,6,7,8,9,10 };
	for (auto& alpha_g : alpha_gs) {
		m_router->setGrideSizeFactor(alpha_g);
		*ofs << "Layers\tPairs\tFound\tFailed\tRateP\tAvgLen\tVia\tRateN\t";
		*ofs << "Time\talpha_g\n";
		for (int i = 0; i < test_times; i++) {
			cout << endl << "====>" << i + 1 << endl;
			//4. Router data cleaning, establish spatial grid index, generate initial planning points
			auto t1 = chrono::high_resolution_clock::now();
			double gridSize = m_config->m_gridSize;
			m_router->routerReset(gridSize);
			vector<string> routingInfo;
			m_router->run(routingInfo);
			auto t2 = chrono::high_resolution_clock::now();
			//5. Output routing time
			auto time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
			cout << "Time:" << time << " ms" << ", gridSize:" << gridSize << endl;
			for (auto& info : routingInfo) {
				*ofs << QString::fromLocal8Bit(info.c_str()) << "\t";
			}
			*ofs << time << "\t" << alpha_g << "\n";
		}
	}
	cout << "\n\n\n\n==================================================================================================" << endl;
}
void AlgorithmLink_dsn::experment5_2_2(QTextStream* ofs) {
	m_router->setGrideSizeFactor(3);
	int test_times = 10;
	vector<double> betas = { 0,0.2,0.4,0.6,0.8,1 };
	for (auto& beta : betas) {
		m_router->setStandardCostFactor(beta);
		*ofs << "Layers\tPairs\tFound\tFailed\tRateP\tAvgLen\tVia\tRateN\t";
		*ofs << "Time\tbeta\n";
		for (int i = 0; i < test_times; i++) {
			cout << endl << "====>" << i + 1 << endl;
			//4. Router data cleaning, establish spatial grid index, generate initial planning points
			auto t1 = chrono::high_resolution_clock::now();
			double gridSize = m_config->m_gridSize;
			m_router->routerReset(gridSize);
			vector<string> routingInfo;
			m_router->run(routingInfo);
			auto t2 = chrono::high_resolution_clock::now();
			//5. Output routing time
			auto time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
			cout << "Time:" << time << " ms" << ", beta:" << beta << endl;
			for (auto& info : routingInfo) {
				*ofs << QString::fromLocal8Bit(info.c_str()) << "\t";
			}
			*ofs << time << "\t" << beta << "\n";
		}
	}
	cout << "\n\n\n\n==================================================================================================" << endl;
}
void AlgorithmLink_dsn::experment5_3(QTextStream* ofs) {
	double alpha_g = 3.0;
	double beta = 0.4;
	m_router->setGrideSizeFactor(alpha_g);
	m_router->setStandardCostFactor(beta);
	int test_times = 20;
	*ofs << "Layers\tPairs\tFound\tFailed\tRateP\tAvgLen\tVia\tRateN\t";
    *ofs << "Time\tgridSize\talpha_g\tbeta\n";
	for (int i = 0; i < test_times; i++) {
		cout << endl << "====>" << i + 1 << endl;
		//4. Router data cleaning, establish spatial grid index, generate initial planning points
		auto t1 = chrono::high_resolution_clock::now();
		double gridSize = m_config->m_gridSize;
		m_router->routerReset(gridSize);
		vector<string> routingInfo;
		m_router->run(routingInfo);
		auto t2 = chrono::high_resolution_clock::now();
		//5. Output routing time
		auto time = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
		cout << "Time:" << time << " ms" << ", gridSize:" << gridSize << ", alpha_g:" << alpha_g << ", beta:" << beta << endl;
		for (auto& info : routingInfo) {
			*ofs << QString::fromLocal8Bit(info.c_str()) << "\t";
		}
		*ofs << time << "\t" << gridSize << "\t" << alpha_g << "\t" << beta << "\n";
	}
	cout << "\n\n\n\n==================================================================================================" << endl;
}

void AlgorithmLink_dsn::dataInit() {
	m_viaInfos.clear();
	m_pads.clear();
	m_nets.clear();
	m_netsInfos.clear();
	m_netTrees.clear();
	m_flyLines.clear();
	// 1. Set boundaries
	QPointF* minp = m_data->getMinPoint();
	QPointF* maxp = m_data->getMaxPoint();
	m_bound = { minp->x(), minp->y(), maxp->x(), maxp->y() };
	// 2. Fill m_viaInfos, read from QSet<QString> m_dsnVias, i.e., m_data->m_dsnVias
	for (const QString& viaName : m_data->m_dsnVias) {
		const vector<Data_dsn::DSNShape>& shapes = m_data->m_dsnPads[viaName].shapes;
		if (shapes.empty()) continue;
		double radius = shapes[0].circle.radius;
		vector<int> layers;
		for (const Data_dsn::DSNShape& shape : shapes) {
			layers.emplace_back(shape.layer);
		}
		m_viaInfos[viaName.toLatin1().constData()] = ViaInfo(radius, layers);
	}
	// 3. Fill m_pads
	double clear_default_smd = m_data->m_dsnRule["default_smd"];
	for (auto it = m_data->m_dsnPins.begin(); it != m_data->m_dsnPins.end(); ++it) {
		// 3.1 Set pad coordinates
		QString qPinName = it.key();
		string pinPadName(it.key().toLatin1().constData());
		double pinX = it->x();
		double pinY = it->y();
		m_pads.insert(make_pair(pinPadName, PinPad(Point(pinX, pinY), pinPadName, "", 0.5)));
		PinPad& onePad = m_pads[pinPadName];
		// 3.2 Set pad polygon
		if (!m_data->m_dsnPins.contains(qPinName) || !m_data->m_dsnPinPads.contains(qPinName))
			continue;
		QString& padName = m_data->m_dsnPinPads[qPinName];
		if (!m_data->m_dsnPads.contains(padName))
			continue;
		const vector<Data_dsn::DSNShape>& shapes = m_data->m_dsnPads[padName].shapes;
		if (shapes.empty()) {
			qDebug() << "shapes is empty:" << padName;
			continue;
		}
		// 3.3 Process the shape of the first shape first, others are the same as the first
		const Data_dsn::DSNShape& shape0 = shapes[0];
		vector<Line> edges;
		double radius = -1;		// If it's a polygon, keep -1; if it's a circle, update to radius>0
		if (shape0.shapeType == "polygon") {
			double x1 = shape0.Pts[0].x(), y1 = shape0.Pts[0].y();
			double x2 = shape0.Pts[1].x(), y2 = shape0.Pts[1].y();
			size_t pointCount = shape0.Pts.size();
			bool isClosed = (pointCount > 1 && shape0.Pts[0] == shape0.Pts[pointCount - 1]);
			size_t effectivePointCount = isClosed ? pointCount - 1 : pointCount;
			if (effectivePointCount < 2) continue;
			for (size_t i = 0; i < effectivePointCount; ++i) {
				size_t next_i = (i + 1) % effectivePointCount;
				double x1 = shape0.Pts[i].x() + pinX;
				double y1 = shape0.Pts[i].y() + pinY;
				double x2 = shape0.Pts[next_i].x() + pinX;
				double y2 = shape0.Pts[next_i].y() + pinY;
				if (x1 == x2 && y1 == y2) {
					continue;
				}
				Line line(Point(x1, y1), Point(x2, y2));
				edges.emplace_back(line);
			}
		}
		else if (shape0.shapeType == "circle") {
			double x = shape0.circle.x() + pinX;
			double y = shape0.circle.y() + pinY;
			radius = shape0.circle.radius;
		}
		// 3.4 Each shape has the same shape but different layer, construct a PinPad with shape and layer
		if (radius > 0)
			for (const Data_dsn::DSNShape& shape : shapes) {
				onePad.addShape(shape.layer, shape.circle.radius, Point(shape.circle.x(), shape.circle.y()));
			}
		else
			for (const Data_dsn::DSNShape& shape : shapes)
				onePad.addShape(shape.layer, edges);
	}
	// 4. Fill m_nets, m_net_vias
	for (auto it = m_data->m_dsnNets.begin(); it != m_data->m_dsnNets.end(); ++it) {
		string netName(it.key().toLatin1().constData());
		//if (netName == "GND") continue;
		vector<PinPad*> oneNet;
		for (const QString& shapeName : it.value().PinsNames) {
			PinPad* pad = &m_pads[shapeName.toLatin1().constData()];
			pad->setNetName(netName);
			oneNet.emplace_back(pad);
		}
		m_nets[netName] = oneNet;
		string viaName(it.value().via.toLatin1().constData());
		m_netsInfos[netName] = NetInfo(viaName, it->width, it->clearance);
	}
}
void AlgorithmLink_dsn::fillPaintFlyLines() {
	//1. Draw flylines
	vector<vector<LineUI>>& ui_flyLines = m_data->m_flyLines;
	ui_flyLines.clear();
	for (auto& lines : m_flyLines) {
		vector<LineUI> linesUI;
		for (auto& line : lines) {
			LineUI lui(line.Pt1.x, line.Pt1.y, line.Pt2.x, line.Pt2.y);
			linesUI.emplace_back(lui);
		}
		ui_flyLines.emplace_back(linesUI);
	}
	//2. Draw vias
	vector<CircleUI>& ui_vias = m_data->m_viaCircles;
	ui_vias.clear();
	addPreViasToUi(ui_vias);
}
void AlgorithmLink_dsn::fillPaintPathLines() {
	//1. Draw path
	if (m_pathsShapes) {
		vector<vector<LineUI>>& ui_paths = m_data->m_paths;
		ui_paths.clear();
		for (const auto& [pNode, shape] : *m_pathsShapes) {
			const vector<PathLine>& pLines = shape.edges;
			ui_paths.emplace_back(vector<LineUI>());
			vector<LineUI>& onePath = ui_paths.back();
			for (const PathLine& line : pLines) {
				LineUI lui(line.p1->pos.x, line.p1->pos.y, line.p2->pos.x, line.p2->pos.y, line.layer, line.width);
				lui.layer = line.layer;
				onePath.emplace_back(lui);
			}
		}
	}
	//2. Draw vias
	vector<CircleUI>& ui_vias = m_data->m_viaCircles;
	ui_vias.clear();
	//2.1 Pre-allocated vias
	addPreViasToUi(ui_vias);
	//2.2 Routing-generated vias
	if (m_vias) {
		for (const auto& [pos, pad] : *m_vias) {
			CircleUI ccPin(pos.x, pos.y, pad.r);
			ui_vias.emplace_back(ccPin);
		}
	}
}
void AlgorithmLink_dsn::addPreViasToUi(vector<CircleUI>& ui_vias) {
	if (!m_preVias.empty()) {
		for (const auto& [pinName, pad] : m_preVias) {
			const Point& pos = pad.pos;
			CircleUI ccPin(pos.x, pos.y, pad.r);
			ui_vias.emplace_back(ccPin);
		}
	}
}
void AlgorithmLink_dsn::fillPaintTrees() {
	if (!m_treesHeads) return;
	vector<vector<LineUI>>& ui_trees = m_data->m_treesLines;
	ui_trees.clear();
	for (const auto& tree : *m_treesHeads) {
		if (!tree) continue;
		ui_trees.emplace_back(vector<LineUI>());
		vector<LineUI>& ui_oneTree = ui_trees.back();

		queue<PathTree*> nodeQueue;
		nodeQueue.push(tree);
		while (!nodeQueue.empty()) {
			PathTree* currentNode = nodeQueue.front();
			nodeQueue.pop();
			for (PathTree* child : currentNode->children) {
				if (!child) continue;
				LineUI lui(currentNode->pos.x, currentNode->pos.y, child->pos.x, child->pos.y, currentNode->layer);	//Different layer trees have different colors, omit layer is gray
				ui_oneTree.push_back(lui);
				nodeQueue.push(child);
			}
		}
	}
}
void AlgorithmLink_dsn::fillPlanningPt() {
	if (!m_planningPts) return;
	vector<QPointF>& ui_planningPts = m_data->m_planningPts;
	ui_planningPts.clear();
	for (const auto& pt : *m_planningPts) {
		ui_planningPts.emplace_back(QPointF(pt.x, pt.y));
	}
}

void AlgorithmLink_dsn::pushLineRunBegin(LineUI& line, vector<LineUI>& lines) {
	if (!m_pathsShapes) return;
	vector<vector<LineUI>>& ui_paths = m_data->m_paths;
	int i = 0;
	m_selectShape = nullptr;
	m_selectPathPreNode = nullptr;
	for (auto& [pNode, shape] : *m_pathsShapes) {
		if (&ui_paths[i++] != &lines) continue;
		m_selectShape = &shape;
		int j = 0;
		m_selectPathPreNode = nullptr;
		for (auto& nodeLine : shape.edges) {
			if (fabs(line.p1().x() - nodeLine.p1->pos.x) < MapMinValue
				&& fabs(line.p1().y() - nodeLine.p1->pos.y) < MapMinValue
				&& fabs(line.p2().x() - nodeLine.p2->pos.x) < MapMinValue
				&& fabs(line.p2().y() - nodeLine.p2->pos.y) < MapMinValue) {
				m_selectPathPreNode = nodeLine.p1;
				break;
			}
		}
		break;
	}
	m_ui_wires = &lines;
}
bool AlgorithmLink_dsn::pushLineRun(const QPointF& offset, const bool inGreed, bool algUpdate) {
	if (!m_data) {
		cout << "pushLineRun: m_data is nullptr" << endl;
		return false;
	}
	if (!m_selectShape || !m_selectPathPreNode)return false;
	//1. Define algorithm input data
	PolyShape shapeInput = m_selectShape->copy();		//Deep copy, release later
	PathNode* nodeInput = nullptr;
	Point offsetInput;
	bool inputGot = setPushLineData(shapeInput, nodeInput, offsetInput, offset);
	if (!inputGot) return false;
	//2. Execute algorithm
	m_router->pushMoveLine(nodeInput, offsetInput, &shapeInput);
	//3. Feedback to frontend
	pushLineUIDataUpdate(shapeInput);
	if (algUpdate) {
		m_router->pushLineDataUpdate(&shapeInput, m_selectShape);
		//4. Update vias
		vector<CircleUI>& ui_vias = m_data->m_viaCircles;
		ui_vias.clear();
		// Pre-allocated vias
		addPreViasToUi(ui_vias);
		// Routing-generated vias
		if (m_vias) {
			for (const auto& [pos, pad] : *m_vias) {
				CircleUI ccPin(pos.x, pos.y, pad.r);
				ui_vias.emplace_back(ccPin);
			}
		}
	}
	shapeInput.deleteNodes();
	return true;
}
bool AlgorithmLink_dsn::setPushLineData(PolyShape& shapeInput, PathNode*& nodeInput, Point& offsetInput, const QPointF& offsetUI) {
	if (m_selectShape->edges.empty()) return false;
	if (offsetUI.x() == 0 && offsetUI.y() == 0) return false;
	//1. Push input segment nodes
	PathNode* algHead = m_selectShape->edges.front().p1;
	if (algHead == m_selectPathPreNode)
		nodeInput = shapeInput.edges[0].p1;
	else {
		PathNode* cur = algHead->next;
		nodeInput = shapeInput.edges[0].p2;
		while (cur && cur != algHead) {
			if (cur == m_selectPathPreNode)
				break;
			cur = cur->next;
			nodeInput = nodeInput->next;
		}
	}
	offsetInput = Point(offsetUI.x(), offsetUI.y());
	return true;
}
void AlgorithmLink_dsn::pushLineUIDataUpdate(const PolyShape& shape) {
	// Update frontend data
	m_ui_wires->clear();
	const vector<PathLine>& pathInput = shape.edges;
	for (const auto& line : pathInput) {
		const Point& p1 = line.p1->pos;
		const Point& p2 = line.p2->pos;
		m_ui_wires->emplace_back(LineUI(p1.x, p1.y, p2.x, p2.y), line.layer, line.width);
	}
}
/*
void AlgorithmLink_dsn::pushPinRunBegin(QPointF pt, vector<LineUI>& lines) {
	m_selectPt = pt;
	m_beginLines.clear();
	if (&lines == nullptr) {
		m_ui_wires = nullptr;  // Or set to other safe value
		return;
	}
	for (auto& line : lines) {
		double x1 = (double)line.x1(), y1 = (double)line.y1();
		double x2 = (double)line.x2(), y2 = (double)line.y2();
		m_beginLines.emplace_back(Line(Point(x1, y1), Point(x2, y2), line.layer, line.width));
	}
	m_ui_wires = &lines;
}
bool AlgorithmLink_dsn::pushPinRun(const QPointF& offset, bool inGreed) {
	if (!m_data) {
		cout << "pushPinRun: m_data is nullptr" << endl;
		return false;
	}
	//1. Define algorithm input data
	vector<Line> polyLines_input;
	Point pt;
	Point offset_input;
	//3. Set algorithm input data
	bool res = setPushPinData(polyLines_input, pt, offset_input, offset);

	//4. Create push object, set basic parameters m_config->m_onGrids, m_config->m_postOn
	SlideLine slide;
	int mode = 1;
	slide.setMode(mode);
	slide.setInGreed(inGreed);

	//5. Execute algorithm
	int result = slide.pushOnePt(polyLines_input, pt, offset_input);

	//6. Feedback to frontend
	if (result == 0) {	// Push midpoint
		pushLineDataUpdate(polyLines_input);
		return true;
	}
	else if (result == 1) {	// Push Pin, clear routing information
		pushLineDataUpdate(polyLines_input);
		m_data->m_paths.clear();
		return true;
	}
	else {
		cout << "pushLineRun: pushSingleLine failed, error code= " << result << " ###" << endl;
		return false;
	}
	return true;

}
bool AlgorithmLink_dsn::setPushLineData(vector<Line>& polyLines_input, Line& line_input, Point& offset_input, const QPointF& offset_ui) {
	bool zeroOffset = offset_ui.x() == 0 && offset_ui.y() == 0;
	if (zeroOffset || m_beginLines.empty()) return false;	// Offset is 0 or input data is empty, end algorithm
	if (m_selectLine.length() == 0) return false;	// Selected segment is zero length, end algorithm

	polyLines_input = vector<Line>(m_beginLines);
	line_input = Line(Point(m_selectLine.x1(), m_selectLine.y1()), Point(m_selectLine.x2(), m_selectLine.y2()));
	offset_input = Point(offset_ui.x(), offset_ui.y());
	return true;
}
void AlgorithmLink_dsn::pushLineDataUpdate(vector<Line>& polyLines_input) {
	m_ui_wires->clear();
	for (auto& line : polyLines_input) {
		m_ui_wires->emplace_back(LineUI(line.Pt1.x, line.Pt1.y, line.Pt2.x, line.Pt2.y), line.layer, line.width);
	}
}
bool AlgorithmLink_dsn::setPushPinData(vector<Line>& polyLines_input, Point& pt_input, Point& offset_input, const QPointF& offset_ui) {
	polyLines_input = vector<Line>(m_beginLines);
	pt_input = Point(m_selectPt.x(), m_selectPt.y());
	offset_input = Point(offset_ui.x(), offset_ui.y());
	return true;
}
/**/