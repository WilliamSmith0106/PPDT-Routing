#pragma once
#include "dataStructUI.h"
#include "../src_config/DiagramStyle.h"
#include <QWidget>
#include <QTransform>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QBasicTimer>
#include <memory>

// Drawing data interface - All Data classes that need to be drawn must implement this interface
class DiagramData {
public:
	virtual ~DiagramData() = default;

	// Data boundaries
	virtual QPointF* getMinPoint() = 0;
	virtual QPointF* getMaxPoint() = 0;

	// 1. Set canvas information (drawn first)
	virtual void set_canvas_data1(
		std::vector<std::vector<QPointF>*>& pts1, std::vector<PointStyle*>& ss_pts1,
		std::vector<std::vector<LineUI>*>& lines1, std::vector<LineStyle*>& ss_lines1,
		std::vector<std::vector<LineUI>*>& polys1, std::vector<PolygonStyle*>& ss_polys1,
		std::vector<std::vector<LineUI>*>& linePts1, std::vector<PointStyle*>& ss_linePts1,
		std::vector<std::vector<CircleUI>*>& circle1, std::vector<PolygonStyle*>& ss_circle1
	) = 0;
	// 2. Set background drawing information (drawn second priority)
	virtual void set_canvas_data2(
		std::vector<std::vector<QPointF>*>& pts2, std::vector<PointStyle*>& ss_pts2,
		std::vector<std::vector<LineUI>*>& lines2, std::vector<LineStyle*>& ss_lines2,
		std::vector<std::vector<LineUI>*>& polys2, std::vector<PolygonStyle*>& ss_polys2,
		std::vector<std::vector<LineUI>*>& linePts2, std::vector<PointStyle*>& ss_linePts2,
		std::vector<std::vector<CircleUI>*>& circle2, std::vector<PolygonStyle*>& ss_circle2
	) = 0;
	virtual void set_canvas_data3(
		std::vector<std::vector<LineUI>*>& lines3, std::vector<LineStyle*>& ss_lines3
	) = 0;
protected:
	virtual void setupStyleManager() = 0;
};