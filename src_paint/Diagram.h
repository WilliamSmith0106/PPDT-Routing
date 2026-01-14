#pragma once
#include "../src_baseClasses/dataStructUI.h"
#include "../src_baseClasses/DiagramData.h"
#include "../src_config/configUI.h"
#include <QWidget>
#include <QTimer>

class Diagram : public QWidget {
	Q_OBJECT
public:
	explicit Diagram(std::shared_ptr<DiagramData> data, ConfigUI* config, QWidget* parent = nullptr)
		: QWidget(parent), m_config(config), m_data(data) {
		m_scaleFactor = 1.0;
		setMouseTracking(true);
		setAttribute(Qt::WA_TransparentForMouseEvents, false);
		setContentsMargins(5, 5, 5, 5);
		setAutoFillBackground(true);
		setMinMax();
		setData1();
		setData2();
		// Delay initialization of view transform
		QTimer::singleShot(0, this, &Diagram::initViewTransform);
	};
	~Diagram() {};

	void initViewTransform();


	// Data update
	void setMinMax();
	void setData1();
	void setData2();
	void setData3();
	void setShowTreeIndex(int index) { m_treeIndex = index; };
	void refresh();

	//public:
private:
	// Core data
	std::shared_ptr<DiagramData> m_data;
	ConfigUI* m_config;
	QPointF* m_minPt;
	QPointF* m_maxPt;

	//1. Background layer drawing information (content drawn with first priority)
	std::vector<std::vector<QPointF>*> m_pts1;
	std::vector<PointStyle*> m_ss_pts1;
	std::vector<std::vector<LineUI>*> m_lines1;
	std::vector<LineStyle*> m_ss_lines1;
	std::vector<std::vector<LineUI>*> m_polys1;
	std::vector<PolygonStyle*> m_ss_polys1;
	std::vector<std::vector<LineUI>*> m_linePts1;
	std::vector<PointStyle*> m_ss_linePts1;
	std::vector<std::vector<CircleUI>*> m_circle1;
	std::vector<PolygonStyle*> m_ss_circle1;

	//2. Foreground layer drawing information (content drawn later, will cover earlier content)
	std::vector<std::vector<QPointF>*> m_pts2;
	std::vector<PointStyle*> m_ss_pts2;
	std::vector<std::vector<LineUI>*> m_lines2;
	std::vector<LineStyle*> m_ss_lines2;
	std::vector<std::vector<LineUI>*> m_polys2;
	std::vector<PolygonStyle*> m_ss_polys2;
	std::vector<std::vector<LineUI>*> m_linePts2;
	std::vector<PointStyle*> m_ss_linePts2;
	std::vector<std::vector<CircleUI>*> m_circle2;
	std::vector<PolygonStyle*> m_ss_circle2;

	//3. Dynamic display data (select whether to display via dropdown)
	int m_treeIndex = -1;
	std::vector<std::vector<LineUI>*> m_lines3;
	std::vector<LineStyle*> m_ss_lines3;




private:	//########################## Drawing Module ##########################

	//0.1 View transform data
	QRectF m_visibleRect;		// Visible area (actual data)
	QTransform m_transform;		// Transformation matrix, actual data coordinates --> screen coordinates
	float m_scaleFactor = 1.0;	// Scaling factor, actual data coordinates --> screen coordinates
	QPointF m_panOffset;		// Offset after scaling (screen coordinate offset)
	bool m_initialized = false;	// View initialization completed

	// 0.2  Mouse interaction data
	bool m_targetSelected = false;
	QPoint m_pressPos;		// Point captured at press moment
	QPoint m_releasePos;
	QPoint m_movePos;		// Point captured during movement
	QBasicTimer m_moveTimer;

	//0.2 Drawing cache
	QPixmap m_canvasCache;      // Off-screen cache image
	QPoint m_cacheOffset;       // Cache offset relative to current view
	bool m_cacheValid = false;  // Whether cache is valid

	//0.3 Drawing style, // Set colors by layer
	std::vector<QColor> m_colors = { Qt::darkGray, Qt::red,  Qt::blue, Qt::yellow,
		Qt::green, Qt::magenta, Qt::darkRed,  Qt::darkYellow, Qt::cyan };
	QColor* getColor(int i) { return &m_colors[i % m_colors.size()]; }

	//1. Visible area drawing, cache drawing
	void paintVisibleContent(const QRect& rect, QPainter& painter);
	void paintContent(QPainter& painter);
	//2. Draw various modules
	void drawGrids(QPainter& painter);	// Draw background grid
	void drawSegments1(QPainter& painter);
	void drawSegments2(QPainter& painter);
	void drawSegments3(QPainter& painter);
	//3. Draw points, lines, polygons
	void drawLines(std::vector<LineUI>* pts, LineStyle* s, QPainter& painter);
	void drawPoints(std::vector<QPointF>* pts, PointStyle* s, QPainter& painter);
	void drawPolygons(std::vector<LineUI>* lines, PolygonStyle* s, QPainter& painter);
	void drawLinePoints(std::vector<LineUI>* lines, PointStyle* s, QPainter& painter);
	void drawCircles(std::vector<CircleUI>* circles, PolygonStyle* s, QPainter& painter);

	//4. Coordinate conversion interface
	QPointF dataToScreen(const QPointF& dataPoint) const;
	LineUI  dataToScreen(const LineUI& dataLine) const;
	QPointF screenToData(const QPointF& screenPoint) const;
	void updateTransform();

	//5. Line clipping
	bool clipLine(QPointF& p1, QPointF& p2, const QRectF& viewport) const;
	bool clipLine(LineUI& line, const QRectF& viewport) const;
	bool clipPt(const QPointF& pt, const QRectF& viewport) const;

protected:	//########################## Event Management ##########################

	//1. Comprehensive events
	void showEvent(QShowEvent* event) override;			// Initialize display
	void resizeEvent(QResizeEvent* event) override;		//resize
	void timerEvent(QTimerEvent* event) override;		// Timer refresh
	void paintEvent(QPaintEvent* event) override;		// Paint event

	//2. Mouse events
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void leaveEvent(QEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;

	//3. Event execution
	void wheelEventRun(const int& delta, const QPointF& screenPos);

private:	//########################## Event Execution ##########################

	//0.1 Push line parameters
	SelectedTarget m_slLine;
	double m_snapThresholdPixels = 10.0;
	// Mouse left button, capture point or line, capture on press, return offset and captured information on release
	void checkHoverTarget(const QPointF& hoverPt);
	// Mouse left button, capture point or line, capture on press, return offset and captured information on release
	bool selectTargetBegin(const QPointF& clickPt);	// Capture point or line
	bool selectTargetActivate();
	bool selectTargetEnd(const QPointF& releasePt);
	// Mouse middle button, move canvas
	void moveCanvasBegin(const QPointF& clickPt);	// Press, execute once
	void moveCanvasActivate();						// Move, execute continuously
	void moveCanvasEnd(const QPoint& releasePt);	// Release, execute once
	// Mouse right button, not yet implemented

	// Snap threshold (data coordinate
	template<typename DataType, typename StyleType>
	bool selectTarget(const QPointF& hoverPt, const bool& hover, std::vector<DataType*>& dataVector, std::vector<StyleType*>& styleVector);

	bool setSelections(const QPointF& clickPt, const bool& hover, std::vector<QPointF>* candidates, PointStyle* s);
	bool setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, LineStyle* s);
	bool setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, PolygonStyle* s);
	bool setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, PointStyle* s);

	bool setSelectedPt(const QPointF& clickPt, const std::vector<QPointF>& line, double snapThreshold);
	bool setSelectedPt(const QPointF& clickPt, const LineUI& line, double snapThreshold);
	bool setSelectedPt(const QPointF& clickPt, std::vector<LineUI>& lines, double snapThreshold);
	bool setSelectedPt(const QPointF& clickPt, std::vector<std::vector<LineUI>>& lines, double snapThreshold);
	bool setSelectedPt(const QPointF& clickPt, std::vector<std::vector<std::vector<LineUI>>>& lines, double snapThreshold);

	bool setSelectedLine(const QPointF& clickPt, const LineUI& line, double snapThreshold);
	bool setSelectedLine(const QPointF& clickPt, std::vector<LineUI>& lines, double snapThreshold);
	bool setSelectedLine(const QPointF& clickPt, std::vector<std::vector<LineUI>>& lines, double snapThreshold);
	bool setSelectedLine(const QPointF& clickPt, std::vector<std::vector<std::vector<LineUI>>>& lines, double snapThreshold);

signals:
	void viewChanged();
	void dataModified();
	void leftPressed(SelectedTarget* target);
	void leftPressedMove(SelectedTarget* target);
	void leftReleaseed(SelectedTarget* target);
};
