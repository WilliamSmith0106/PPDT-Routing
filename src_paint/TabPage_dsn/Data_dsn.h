#pragma once
#include "../../src_baseClasses/Data.h"
#include "../../src_baseClasses/DiagramData.h"
#include "Style_dsn.h"
#include <QHash>
#include <QSet>

class Data_dsn : public Data, public DiagramData {
public:
	Data_dsn() { setupStyleManager(); }
	~Data_dsn() {}

	struct DSNComponent {
			QPointF origin;		// Component origin coordinate, memory 16B
			QString direction;	// Component direction, memory 8B
			double rotation;	// Component rotation angle, memory 8B

	};
	struct DSNShape {
			std::vector<QPointF> Pts;	// Store shape parameters, such as circle diameter, rectangle length and width, etc. (with geometric center as coordinate origin)
			CircleUI circle;
			QSet<QString> PinsName;		// Pin names associated with this pad (optional)
			QString shapeType;		// Shape type, such as circle, polygon
			double rotation;		// Rotation angle
			int layer = 1;			// Layer
			int backup;				// Reserved field, memory alignment
	};
	struct DSNPad {
			std::vector<DSNShape> shapes;	// Pad shape collection
			QSet<QString> PinsName;			// Pin names corresponding to the net
	};
	struct DSNNet {
			std::vector<QString> PinsNames;	// Coordinates of pads corresponding to pins (need to be calculated)
			QString via = "via0";		// Default via (optional)
			double width = 1;			// Line width (optional)
			double clearance = 0.4;		// Clearance (optional)
	};
	struct DSNWring {
			LineUI line;			// Line segment, with line width inside, memory 40B
			QString wiringType;		// Line segment type, such as wire, via, etc., memory 8B
			QString viaName;		// Via name, such as via0, via1, etc., memory 8B
			QString netName;		// Belonging network, memory 8B
			QString type;			// Line segment type, such as protect, etc., memory 8B
			int layer;				// Layer, memory 4B
	};

	// Directly read information
	QHash<QString, QString> m_dsnPCBInfo;			// dsn title, exported CAD tool, tool version
	double m_dsnResolution = 100;					// dsn resolution (mil)

	std::vector<double> m_dsnBoundary;				// dsn boundary data
	QSet<QString> m_dsnVias;						// dsn via definitions
	QHash<QString, double>	m_dsnGride;				// Grid snap spacing
	QHash<QString, double> m_dsnRule;				// Clearance default, default_smd, smd_smd
	double m_ruleWidth = 1.0;						// Default line width
	QHash<int, QString> m_dsnLayers;				// dsn layer names, layer 0 signal, layer 1 signal

	QHash<QString, DSNComponent> m_dsnComponent;	// placement

	QHash<QString, DSNPad> m_dsnPads;				// Pads (including shape information)
	QHash<QString, QPointF>	m_dsnPins;				// Pins (including coordinate information)
	QHash<QString, QString> m_dsnPinPads;			// Pin name to pad name mapping
	QHash<QString, QString> m_obssPads;				// Pins not in net to pads

	QHash<QString, DSNNet> m_dsnNets;				// Nets (including pin information)
	std::vector<DSNWring> m_dsnWirings;				// Line segments (including segment information)

	// Drawing and interaction information
	double m_trans = 1;			// 0.254 can be converted to mm
	std::vector<LineUI> m_boundaryLines;			// Boundary
	std::vector<std::vector<QPointF>> m_PinsNet;	// Pins (pad centers)
	std::vector<std::vector<LineUI>> m_PinsPoly;	// Polygon pads
	std::vector<std::vector<CircleUI>> m_circles;	// Circular pads, each container is a layer
	std::vector<CircleUI> m_viaCircles;				// Circular vias, each container is a layer

	std::vector<QPointF> m_planningPts;				// Planning points
	std::vector<std::vector<LineUI>> m_paths;		// Routing, each container is a layer when initially reading data
	std::vector<std::vector<LineUI>> m_treesLines;	// Search trees
	std::vector<std::vector<CircleUI>> m_viaInfos;	// Vias
	std::vector<std::vector<LineUI>> m_flyLines;	// Flying lines

	void transform() { // Convert relevant data to unit, mil*0.254 to get millimeters
		minPt.setX(minPt.x() * m_trans);
		minPt.setY(minPt.y() * m_trans);
		maxPt.setX(maxPt.x() * m_trans);
		maxPt.setY(maxPt.y() * m_trans);
		// 1. Convert m_dsnBoundary (starting from the 1st number)
		if (m_dsnBoundary.size() > 1) {
			std::transform(m_dsnBoundary.begin() + 1, m_dsnBoundary.end(),
				m_dsnBoundary.begin() + 1,
				[this](double x) { return x * m_trans; });
		}

		// 2. Convert m_dsnGride
		for (auto it = m_dsnGride.begin(); it != m_dsnGride.end(); ++it) {
			it.value() *= m_trans;
		}

		// 3. Convert m_dsnRule
		for (auto it = m_dsnRule.begin(); it != m_dsnRule.end(); ++it) {
			it.value() *= m_trans;
		}

		// 4. Convert m_ruleWidth
		m_ruleWidth *= m_trans;

		// 5. Convert m_dsnPads
		for (auto it = m_dsnPads.begin(); it != m_dsnPads.end(); ++it) {
			DSNPad& pad = it.value();
			for (DSNShape& shape : pad.shapes) {
				// Convert all coordinate points in Pts
				if (shape.shapeType == "polygon")
					for (QPointF& point : shape.Pts) {
						point.setX(point.x() * m_trans);
						point.setY(point.y() * m_trans);
					}
				else if (shape.shapeType == "circle") {
					CircleUI& cc = shape.circle;
					cc.setX(cc.x() * m_trans);
					cc.setY(cc.y() * m_trans);
					cc.radius *= m_trans;	// Convert read diameter to millimeters and calculate radius
				}

			}
		}

		// 6. Convert m_dsnPins
		for (auto it = m_dsnPins.begin(); it != m_dsnPins.end(); ++it) {
			QPointF& point = it.value();
			point.setX(point.x() * m_trans);
			point.setY(point.y() * m_trans);
		}

		// 7. Convert m_dsnNets
		for (auto it = m_dsnNets.begin(); it != m_dsnNets.end(); ++it) {
			DSNNet& net = it.value();
			net.width *= m_trans;
			net.clearance *= m_trans;
		}

		// 8. Convert m_dsnWirings
		for (DSNWring& wiring : m_dsnWirings) {
			// If need to convert coordinates in LineUI, can add here
			QPointF p1 = wiring.line.p1();  // Get start point
			QPointF p2 = wiring.line.p2();  // Get end point

			p1.setX(p1.x() * m_trans);
			p1.setY(p1.y() * m_trans);
			p2.setX(p2.x() * m_trans);
			p2.setY(p2.y() * m_trans);

			// Set converted coordinates
			wiring.line.setP1(p1);
			wiring.line.setP2(p2);

			// Also convert the width member of LineUI itself
			wiring.line.width *= m_trans;
		}
	}

	void clear() {
		m_boundaryLines.clear();
		m_PinsNet.clear();
		m_PinsPoly.clear();
		m_circles.clear();
		m_planningPts.clear();
		m_paths.clear();
		m_treesLines.clear();
		m_viaInfos.clear();
		m_flyLines.clear();
	}
	void setPaintData() {
		// Set the part to be drawn in the read data
		m_boundaryLines.clear();
		m_PinsNet.clear();
		m_PinsPoly.clear();
		m_circles.clear();
		//1. Boundary m_boundaryLines
		size_t len_3 = m_dsnBoundary.size() - 3;
		for (int i = 1; i < len_3; i += 2) {
			QPointF pt1(m_dsnBoundary[i], m_dsnBoundary[i + 1]);
			QPointF pt2(m_dsnBoundary[i + 2], m_dsnBoundary[i + 3]);
			m_boundaryLines.emplace_back(LineUI(pt1, pt2));
		}
		double x_end = m_dsnBoundary[len_3 + 1], y_end = m_dsnBoundary[len_3 + 2]; // Closed shape
		double x_start = m_dsnBoundary[1], y_start = m_dsnBoundary[2];
		if (x_end != x_start || y_end != y_start)
			m_boundaryLines.emplace_back(LineUI(x_end, y_end, x_start, y_start));

		//2. Pin center coordinates m_PinsNet, and set keepout area (pins not in net)
		m_obssPads = m_dsnPinPads;
		for (auto& net : m_dsnNets) {
			m_PinsNet.emplace_back(std::vector<QPointF>());
			for (QString& shapeName : net.PinsNames) {
				if (m_dsnPins.contains(shapeName)) {
					m_PinsNet.back().emplace_back(m_dsnPins[shapeName]);
					m_obssPads.remove(shapeName);
				}
				else {
					qDebug() << "shapeName not found in Pins:" << shapeName;
				}
			}
		}

		//3. Pads (polygons or circles)
		for (auto it = m_dsnPinPads.begin(); it != m_dsnPinPads.end(); ++it) {
			const QString& shapeName = it.key();
			const QString& padName = it.value();

			if (!m_dsnPins.contains(shapeName) || !m_dsnPads.contains(padName))
				continue;

			// 2.2 Add polygon for pad
			std::vector<DSNShape>& shapes = m_dsnPads[padName].shapes;
			if (shapes.empty()) {
				qDebug() << "shapes is empty:" << padName;
				continue;
			}
			double pinX = m_dsnPins[shapeName].x();
			double pinY = m_dsnPins[shapeName].y();
			for (const DSNShape& shape : shapes) {
				if (shape.shapeType == "polygon") {
					m_PinsPoly.emplace_back(std::vector<LineUI>());
					double x1 = shape.Pts[0].x(), y1 = shape.Pts[0].y();
					double x2 = shape.Pts[1].x(), y2 = shape.Pts[1].y();
					int layer = shape.layer;
					size_t pointCount = shape.Pts.size();
					bool isClosed = (pointCount > 1 && shape.Pts[0] == shape.Pts[pointCount - 1]);
					size_t effectivePointCount = isClosed ? pointCount - 1 : pointCount;
					if (effectivePointCount < 2) continue;
					for (size_t i = 0; i < effectivePointCount; ++i) {
						size_t next_i = (i + 1) % effectivePointCount;
						double x1 = shape.Pts[i].x() + pinX;
						double y1 = shape.Pts[i].y() + pinY;
						double x2 = shape.Pts[next_i].x() + pinX;
						double y2 = shape.Pts[next_i].y() + pinY;
						if (x1 == x2 && y1 == y2) {
							continue;
						}
						LineUI line(QPointF(x1, y1), QPointF(x2, y2));
						line.layer = layer;
						m_PinsPoly.back().emplace_back(line);
					}
				}
				else if (shape.shapeType == "circle") {
					while (shape.layer > m_circles.size())
						m_circles.emplace_back(std::vector<CircleUI>());
					CircleUI ccPin(shape.circle.x() + pinX, shape.circle.y() + pinY, shape.circle.radius);
					m_circles[shape.layer - 1].emplace_back(ccPin);
				}
			}
		}
		//4. Read wiring data
		updatePaintWires();
	}
	void updatePaintWires() {
		// Read wiring data
		m_paths.clear();
		m_viaInfos.clear();
		for (DSNWring& wire : m_dsnWirings) {
			if (wire.wiringType == "wire") {
				if (wire.layer < 0 || wire.layer > 8) wire.layer = 1;
				while (wire.layer > m_paths.size()) m_paths.emplace_back(std::vector<LineUI>());
				m_paths[wire.layer - 1].emplace_back(wire.line);
			}
			else if (wire.wiringType == "via") {
				if (m_dsnPads.contains(wire.viaName)) {
					std::vector<DSNShape>& shapes = m_dsnPads[wire.viaName].shapes;
					for (auto& shape : shapes) {
						shape.circle.setX(wire.line.p1().x());
						shape.circle.setY(wire.line.p1().y());
						while (shape.layer > m_viaInfos.size()) m_viaInfos.emplace_back(std::vector<CircleUI>());
						m_viaInfos[shape.layer - 1].emplace_back(shape.circle);
					}
				}
			}
		}
	}

	// Implement DiagramData interface
	QPointF* getMinPoint() override { return &minPt; }
	QPointF* getMaxPoint() override { return &maxPt; }

	// 1. Set canvas information (content drawn first)
	void set_canvas_data1(
		std::vector<std::vector<QPointF>*>& pts1, std::vector<PointStyle*>& ss_pts1,
		std::vector<std::vector<LineUI>*>& lines1, std::vector<LineStyle*>& ss_lines1,
		std::vector<std::vector<LineUI>*>& polys1, std::vector<PolygonStyle*>& ss_polys1,
		std::vector<std::vector<LineUI>*>& linePts1, std::vector<PointStyle*>& ss_linePts1,
		std::vector<std::vector<CircleUI>*>& circle1, std::vector<PolygonStyle*>& ss_circle1
	) {
		//1.1 Points: pin, m_PinsNet (pad centers)
		for (auto& net : m_PinsNet)
			pts1.emplace_back(&net);
		size_t netSum = m_PinsNet.size();
		ss_pts1.reserve(ss_pts1.size() + netSum);
		const auto& stylePt = m_styles.getPointStyle(m_styleInfo.ptPadKey);
		std::fill_n(std::back_inserter(ss_pts1), netSum, stylePt);
		//1.2 Points: planningPtKey planning points
		pts1.emplace_back(&m_planningPts);
		ss_pts1.emplace_back(m_styles.getPointStyle(m_styleInfo.planningPtKey));

		//2.1 Lines:
		//3.1 Polygons: Boundary
		polys1.emplace_back(&m_boundaryLines);
		ss_polys1.emplace_back(m_styles.getPolygonStyle(m_styleInfo.boundaryPolyKey));
		//3.1 Polygons: Polygon pads
		if (!m_PinsPoly.empty()) {
			for (int i = 0; i < m_PinsPoly.size(); i++) {
				polys1.emplace_back(&m_PinsPoly[i]);
				int layer = m_PinsPoly[i].front().layer;
				const auto& stylePoly = m_styles.getPolygonStyle(m_styleInfo.getPadPolyKey(layer - 1));
				ss_polys1.emplace_back(stylePoly);
			}
		}
		//4.1 Line segment endpoints:
		//5.1 Circles: Circular pads
		for (int i = (int)m_circles.size() - 1; i >= 0; i--) {
			circle1.emplace_back(&m_circles[i]);
			const auto& stylePoly = m_styles.getPolygonStyle(m_styleInfo.getPadPolyKey(i));
			ss_circle1.emplace_back(stylePoly);
		}
	};
	// 2. Set foreground drawing information (content drawn last, will cover previously drawn content)
	void set_canvas_data2(
		std::vector<std::vector<QPointF>*>& pts2, std::vector<PointStyle*>& ss_pts2,
		std::vector<std::vector<LineUI>*>& lines2, std::vector<LineStyle*>& ss_lines2,
		std::vector<std::vector<LineUI>*>& polys2, std::vector<PolygonStyle*>& ss_polys2,
		std::vector<std::vector<LineUI>*>& linePts2, std::vector<PointStyle*>& ss_linePts2,
		std::vector<std::vector<CircleUI>*>& circle2, std::vector<PolygonStyle*>& ss_circle2
	) {
		//1.1 Points:
		//2.1 Lines: Flying lines
		for (auto& lines : m_flyLines)
			lines2.emplace_back(&lines);
		size_t flyLinesSum = m_flyLines.size();
		ss_lines2.reserve(ss_lines2.size() + flyLinesSum);
		const auto& flyLinesStyle = m_styles.getLineStyle(m_styleInfo.flyLineKey);
		std::fill_n(std::back_inserter(ss_lines2), flyLinesSum, flyLinesStyle);

		//2.2 Lines: Routing
		for (int i = (int)m_paths.size() - 1; i >= 0; i--) {
			lines2.emplace_back(&m_paths[i]);
			const auto& pathStyle = m_styles.getLineStyle(m_styleInfo.getPathLineKey(i));
			ss_lines2.emplace_back(pathStyle);
		}

		//3.1 Polygons:
		//4.1 Line segment endpoints:
		//5.1 Circles: Circular pads
		circle2.emplace_back(&m_viaCircles);
		const auto& styleVia = m_styles.getPolygonStyle(m_styleInfo.viaCicleKey);
		ss_circle2.emplace_back(styleVia);
	};
	// 3. Set dynamic drawing information (drop-down selection to display which data)
	void set_canvas_data3(
		std::vector<std::vector<LineUI>*>& lines3, std::vector<LineStyle*>& ss_lines3
	) {
		//1 Lines: Search trees
		for (auto& lines : m_treesLines)
			lines3.emplace_back(&lines);
		size_t treeLinesSum = m_treesLines.size();
		ss_lines3.reserve(ss_lines3.size() + treeLinesSum);
		const auto& treeLinesStyle = m_styles.getLineStyle(m_styleInfo.treeLineKey);
		std::fill_n(std::back_inserter(ss_lines3), treeLinesSum, treeLinesStyle);
	};

public:
	// Define drawing styles
	Style_dsn m_styleInfo = Style_dsn();
private:

	void setupStyleManager() {
		m_styles.setPointStyle(m_styleInfo.pointStyleKeys, m_styleInfo.pointStyles);
		m_styles.setLineStyle(m_styleInfo.lineStyleKeys, m_styleInfo.lineStyles);
		m_styles.setPolygonStyle(m_styleInfo.polygonStyleKeys, m_styleInfo.polygonStyles);
	}

};
