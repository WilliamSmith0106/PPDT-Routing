#pragma once
#include "../../src_config/DiagramStyle.h"
#include <vector>

// Selection flag description: m_type default is 0
// 0 Not selectable
// 1-999 Fully selectable
// 1001 and above: Line segments can only be selected as segments
// 2001 and above: Line segments can only be selected by endpoints
// 6001: Only allow hovering capture and display, no movement allowed
struct Style_dsn {
	// 1. Define point styles
	QString planningPtKey = "planningPt";
	QString viaKey = "viaCenter";
	QString ptPadKey = "ptPadCenter";
	std::vector<QString> pointStyleKeys = { planningPtKey, viaKey, ptPadKey };
	std::vector<PointStyle> pointStyles = {
		// Outline color, show outline, outline width, fill, fill color, radius, selection flag
		PointStyle(Qt::black,true,1.0,true,Qt::gray,2.0,6001),	// Planning point
		PointStyle(Qt::black,true,1.0,true,Qt::blue,5.0,1),		// Via center point
		PointStyle(Qt::black,false,1.0,true,Qt::red,2.0,6001),	// Pad center point
	};
	// 2. Define line styles
	QString flyLineKey = "flyLine";
	QString treeLineKey = "treeLine";
	std::vector<QString> pathLineKeys = { "pathLine0","pathLine1","pathLine2","pathLine3" };
	std::vector<QString> lineStyleKeys = { flyLineKey, treeLineKey, pathLineKeys[0],pathLineKeys[1] ,pathLineKeys[2] ,pathLineKeys[3] };
	std::vector<LineStyle> lineStyles = {
		// Color, width, line type, selection flag
		LineStyle(Qt::blue, 1.0, Qt::SolidLine,6001),		// Flying line style
		LineStyle(Qt::gray, 2.0, Qt::SolidLine,0),			// Tree style

		LineStyle(Qt::red, 3.0, Qt::SolidLine,1),		//pathLine0
		LineStyle(Qt::blue, 3.0, Qt::SolidLine,1),		//pathLine1
		LineStyle(Qt::yellow, 3.0, Qt::SolidLine,1),	//pathLine2
		LineStyle(Qt::green, 3.0, Qt::SolidLine,1)		//pathLine3
	};
	QString getPathLineKey(int i) const { return pathLineKeys[i % 4]; };

	// 3. Define polygon or circle styles
	QString boundaryPolyKey = "boundaryPolygon";
	QString viaCicleKey = "viaCicleKey";
	std::vector<QString> padPolyKeys = { "padPolygon0","padPolygon1","padPolygon2","padPolygon3" };
	std::vector<QString> polygonStyleKeys = { boundaryPolyKey,viaCicleKey,padPolyKeys[0],padPolyKeys[1] ,padPolyKeys[2] ,padPolyKeys[3] };
	std::vector<PolygonStyle> polygonStyles = {
		// Outline color, whether outline is visible, outline width, whether to fill, fill color, selection flag
		PolygonStyle(Qt::black, true, 4.0,false,Qt::black,0),			// Border
		PolygonStyle(Qt::green, true, 4.0,true,Qt::green,0),			// Via

		PolygonStyle(Qt::darkRed, true, 4.0,false,Qt::gray,6001),		//padPolygon0
		PolygonStyle(Qt::darkBlue, true, 6.0,false,Qt::gray,6001),		//padPolygon1
		PolygonStyle(Qt::darkYellow, true, 6.0,false,Qt::gray,6001),	//padPolygon2
		PolygonStyle(Qt::darkGreen, true, 6.0,false,Qt::gray,6001)		//padPolygon3
	};
	QString getPadPolyKey(int i) const { return padPolyKeys[i % 4]; };
};
