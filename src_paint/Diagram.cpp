#include "Diagram.h"
#include "../src_config/DiagramStyle.h"
#include <QTimer>
#include <QTabWidget>
#include <cmath>
#include <memory>
#include <QMessageBox>
#include <QPainterPath>

using namespace std;

// View initialization
void Diagram::initViewTransform() {
	QSize effectiveSize;
	if (QTabWidget* tabWidget = qobject_cast<QTabWidget*>(parentWidget())) {	// Get through parent container
		effectiveSize = tabWidget->currentWidget()->size();
	}
	else {
		effectiveSize = size();		// Get available size directly
	}
	if (effectiveSize.width() <= 10 || effectiveSize.height() <= 10) {
		effectiveSize = QSize(800, 600); // Default size
	}
	// Calculate data bounds
	QRectF dataBounds(*m_minPt, *m_maxPt);

	// Calculate initial scaling factor (5% margin reserved)
	if (m_config->m_rotate_90) {
		qreal xScale = (effectiveSize.width() * 0.95) / dataBounds.height();  // Calculate x scale using height
		qreal yScale = (effectiveSize.height() * 0.95) / dataBounds.width();  // Calculate y scale using width
		m_scaleFactor = qMin(xScale, yScale);
	}
	else {
		qreal xScale = (effectiveSize.width() * 0.95) / dataBounds.width();
		qreal yScale = (effectiveSize.height() * 0.95) / dataBounds.height();
		m_scaleFactor = qMin(xScale, yScale);
	}
	// Calculate initial translation (centering), i.e., after data is scaled to the appropriate size and moved to the far left, then translate by m_panOffset to center
	qreal offsetX = 0, offsetY = 0;
	qreal dataWidth = dataBounds.width() * m_scaleFactor;
	qreal dataHeight = dataBounds.height() * m_scaleFactor;
	qreal viewWidth = effectiveSize.width();
	qreal viewHeight = effectiveSize.height();
	if (m_config->m_flip_left_right) {
		if (m_config->m_flip_up_down) {
			if (m_config->m_rotate_90) {
				offsetX = (viewWidth - dataHeight) / 2 - m_minPt->y() * m_scaleFactor;
				offsetY = (viewHeight - dataWidth) / 2 + m_maxPt->x() * m_scaleFactor;
			}
			else {
				offsetX = (viewWidth + dataWidth) / 2 + m_minPt->x() * m_scaleFactor;
				offsetY = (viewHeight + dataHeight) / 2 + m_minPt->y() * m_scaleFactor;
			}
		}
		else {
			if (m_config->m_rotate_90) {
				offsetX = (viewWidth + dataHeight) / 2 + m_minPt->y() * m_scaleFactor;
				offsetY = (viewHeight - dataWidth) / 2 + m_maxPt->x() * m_scaleFactor;
			}
			else {
				offsetX = (viewWidth + dataWidth) / 2 + m_minPt->x() * m_scaleFactor;
				offsetY = (viewHeight - dataHeight) / 2 - m_minPt->y() * m_scaleFactor;
			}
		}
	}
	else {
		if (m_config->m_flip_up_down) {
			if (m_config->m_rotate_90) {
				offsetX = (viewWidth - dataHeight) / 2 - m_minPt->y() * m_scaleFactor;
				offsetY = (viewHeight + dataWidth) / 2 - m_maxPt->x() * m_scaleFactor;
			}
			else {
				offsetX = (viewWidth - dataWidth) / 2 - m_minPt->x() * m_scaleFactor;
				offsetY = (viewHeight + dataHeight) / 2 + m_minPt->y() * m_scaleFactor;
			}
		}
		else {
			if (m_config->m_rotate_90) {
				offsetX = (viewWidth + dataHeight) / 2 + m_minPt->y() * m_scaleFactor;
				offsetY = (viewHeight - dataWidth) / 2 - m_minPt->x() * m_scaleFactor;
			}
			else {
				offsetX = (viewWidth - dataWidth) / 2 - m_minPt->x() * m_scaleFactor;
				offsetY = (viewHeight - dataHeight) / 2 - m_minPt->y() * m_scaleFactor;
			}
		}
	}
	m_panOffset = QPointF(offsetX, offsetY);
	updateTransform();
}
void Diagram::setMinMax() {
	m_minPt = m_data->getMinPoint();
	m_maxPt = m_data->getMaxPoint();
}
void Diagram::setData1() {
	m_pts1.clear();
	m_ss_pts1.clear();
	m_lines1.clear();
	m_ss_lines1.clear();
	m_polys1.clear();
	m_ss_polys1.clear();
	m_linePts1.clear();
	m_ss_linePts1.clear();
	m_data->set_canvas_data1(m_pts1, m_ss_pts1, m_lines1, m_ss_lines1, m_polys1, m_ss_polys1, m_linePts1, m_ss_linePts1, m_circle1, m_ss_circle1);
	update();
}
void Diagram::setData2() {
	m_pts2.clear();
	m_ss_pts2.clear();
	m_lines2.clear();
	m_ss_lines2.clear();
	m_polys2.clear();
	m_ss_polys2.clear();
	m_linePts2.clear();
	m_ss_linePts2.clear();
	m_data->set_canvas_data2(m_pts2, m_ss_pts2, m_lines2, m_ss_lines2, m_polys2, m_ss_polys2, m_linePts2, m_ss_linePts2, m_circle2, m_ss_circle2);
	update();
}
void Diagram::setData3() {
	m_lines3.clear();
	m_ss_lines3.clear();
	m_data->set_canvas_data3(m_lines3, m_ss_lines3);
}
void Diagram::refresh() {
	update();
}

//########################## Drawing Module (Private) ##########################

//1. Drawing mode
void Diagram::paintVisibleContent(const QRect& rect, QPainter& painter) {
	painter.save();
	//1. Initialization
	m_visibleRect = m_transform.inverted().mapRect(rect);		// Get visible area (data coordinate system)
	//2. Draw grid
	drawGrids(painter);
	//3. Draw other primitives in sequence
	drawSegments1(painter);
	drawSegments2(painter);
	drawSegments3(painter);
	painter.restore();
}
void Diagram::paintContent(QPainter& painter) {
	if (m_config->painState == PUSH_LINE)
		return;
	painter.setRenderHint(QPainter::Antialiasing);
	painter.save();
	QFont font = painter.font();
	font.setPointSize(9);
	painter.setFont(font);

	// Convert to screen coordinates
	QPointF screenPos = dataToScreen(m_slLine.infoPosition);

	// Calculate text size
	QFontMetrics fm(font);
	QRect textRect = fm.boundingRect(QRect(0, 0, 200, 100),
		Qt::AlignLeft | Qt::TextWordWrap,
		m_slLine.infoText);
	textRect.moveTo(screenPos.x() + 25, screenPos.y() + 10);

	// Draw background
	QRect bgRect = textRect.adjusted(-5, -5, 5, 5);
	painter.setBrush(QColor(255, 255, 225, 230));
	painter.setPen(QPen(Qt::black, 1));
	painter.drawRect(bgRect);

	// Draw text
	painter.setPen(Qt::black);
	painter.drawText(textRect, Qt::AlignLeft | Qt::TextWordWrap, m_slLine.infoText);
	painter.restore();
}

//2. Draw various modules
void Diagram::drawGrids(QPainter& painter) {
	if (m_config->m_gridType == 0) return;	// Don't show grid
	if (!m_minPt || !m_maxPt) return;		// Check if necessary variables exist
	painter.save();
	painter.setRenderHint(QPainter::Antialiasing, false);
	float gridSize = m_config->m_gridSize;
	float screenGridSpacing = gridSize * m_scaleFactor;
	int step = 1;
	if (screenGridSpacing < 20.0f) {
		// If screen spacing is less than 5 pixels, calculate appropriate step
		step = static_cast<int>(std::ceil(m_config->m_minimalScreenGridSize / screenGridSpacing));
	}

	if (m_config->m_gridType == 1) {  //1. Grid lines
		painter.setPen(QPen(Qt::gray, 1)); // Grid lines are gray, 1 pixel wide
		float startX = m_minPt->x();
		float startY = m_minPt->y();
		float endX = m_maxPt->x();
		float endY = m_maxPt->y();
		int xCount = static_cast<int>((endX - startX) / gridSize);
		int yCount = static_cast<int>((endY - startY) / gridSize);
		for (int i = 0; i <= xCount; i += step) {
			float x = startX + i * gridSize;
			painter.drawLine(dataToScreen(QPointF(x, startY)), dataToScreen(QPointF(x, endY)));
		}
		for (int i = 0; i <= yCount; i += step) {
			float y = startY + i * gridSize;
			painter.drawLine(dataToScreen(QPointF(startX, y)), dataToScreen(QPointF(endX, y)));
		}
	}
	else if (m_config->m_gridType == 2) {	 //2. Grid points
		/**/
		painter.setPen(QPen(Qt::black, 1));
		// Calculate grid point starting position (aligned to grid)
		float startX = std::floor(m_visibleRect.left() / gridSize) * gridSize;
		float startY = std::floor(m_visibleRect.top() / gridSize) * gridSize;
		float endX = m_visibleRect.right();
		float endY = m_visibleRect.bottom();
		int xCount = static_cast<int>((endX - startX) / gridSize) + 1;
		int yCount = static_cast<int>((endY - startY) / gridSize) + 1;
		painter.setBrush(QBrush(Qt::black));
		QPointF screenPoint;
		for (int i = 0; i < xCount; i += step) {
			float x = startX + i * gridSize;
			for (int j = 0; j < yCount; j += step) {
				float y = startY + j * gridSize;
				screenPoint = dataToScreen(QPointF(x, y));
				// Draw 4 points, increase point size
				painter.drawRect(QRectF(screenPoint.x(), screenPoint.y(), 2, 2));
			}
		}
		painter.setBrush(Qt::NoBrush);
	}
	// Draw border
	painter.setPen(QPen(Qt::black, 3));
	QRectF borderRect(dataToScreen(*m_minPt), dataToScreen(*m_maxPt));
	painter.drawRect(borderRect);
	painter.restore();
}
void Diagram::drawSegments1(QPainter& painter) {
	// 1. Draw lines
	for (int i = 0; i < m_lines1.size(); i++)
		if (m_ss_lines1[i]->isVisible())
			drawLines(m_lines1[i], m_ss_lines1[i], painter);
	// 2. Draw points
	for (int i = 0; i < m_pts1.size(); i++)
		if (m_ss_pts1[i]->isVisible())
			drawPoints(m_pts1[i], m_ss_pts1[i], painter);
	// 3. Draw polygons
	for (int i = 0; i < m_polys1.size(); i++)
		if (m_ss_polys1[i]->isVisible())
			drawPolygons(m_polys1[i], m_ss_polys1[i], painter);
	// 4. Draw line endpoints
	for (int i = 0; i < m_linePts1.size(); i++)
		if (m_ss_linePts1[i]->isVisible())
			drawLinePoints(m_linePts1[i], m_ss_linePts1[i], painter);
	for (int i = 0; i < m_circle1.size(); i++)
		if (m_ss_circle1[i]->isVisible())
			drawCircles(m_circle1[i], m_ss_circle1[i], painter);
};
void Diagram::drawSegments2(QPainter& painter) {
	// 1. Draw lines
	for (int i = 0; i < m_lines2.size(); i++)
		if (m_ss_lines2[i]->isVisible())
			drawLines(m_lines2[i], m_ss_lines2[i], painter);
	// 2. Draw points
	for (int i = 0; i < m_pts2.size(); i++)
		if (m_ss_pts2[i]->isVisible())
			drawPoints(m_pts2[i], m_ss_pts2[i], painter);
	// 3. Draw polygons
	for (int i = 0; i < m_polys2.size(); i++)
		if (m_ss_polys2[i]->isVisible())
			drawPolygons(m_polys2[i], m_ss_polys2[i], painter);
	// 4. Draw line endpoints
	for (int i = 0; i < m_linePts2.size(); i++)
		if (m_ss_linePts2[i]->isVisible())
			drawLinePoints(m_linePts2[i], m_ss_linePts2[i], painter);
	for (int i = 0; i < m_circle2.size(); i++)
		if (m_ss_circle2[i]->isVisible())
			drawCircles(m_circle2[i], m_ss_circle2[i], painter);
}
void Diagram::drawSegments3(QPainter& painter) {
	// Draw lines
	if (!m_config->m_showTrees || m_lines3.empty())		// Don't show, or empty
		return;
	if (m_treeIndex == -1) {	// Show all
		for (int i = 0; i < m_lines3.size(); i++)
			if (m_ss_lines3[i]->isVisible())
				drawLines(m_lines3[i], m_ss_lines3[i], painter);
		return;
	}
	if (m_treeIndex >= 0 && m_treeIndex < m_lines3.size()) {
		// Draw lines with specified index
		drawLines(m_lines3[m_treeIndex], m_ss_lines3[m_treeIndex], painter);
		return;
	}
}

//3. Draw elements
void Diagram::drawLines(std::vector<LineUI>* lines, LineStyle* s, QPainter& painter) {
	// Set line properties
	QPen pen(s->m_color);
	pen.setWidthF(s->m_width);
	pen.setStyle(s->m_style);
	pen.setCapStyle(Qt::RoundCap);      // Eliminate sharp corners
	pen.setJoinStyle(Qt::RoundJoin);    // Eliminate sharp corners
	QPen penLine = pen;
	// Iterate through all line segments and draw
	for (LineUI line : *lines) {
		if (clipLine(line, m_visibleRect)) {
			QLineF screenLine(dataToScreen(line.p1()), dataToScreen(line.p2()));
			float width = line.width * m_scaleFactor;
			if (width > 1 && width < 400) {
				penLine.setWidthF(width);
				if (line.layer > 0) {
					penLine.setColor(*getColor(line.layer));
				}
				painter.setPen(penLine);
			}
			else {
				pen.setWidthF(s->m_width);  // Use basic line width
				if (line.layer > 0) {
					pen.setColor(*getColor(line.layer));
				}
				painter.setPen(pen);
			}
			painter.drawLine(screenLine);
		}
	}
}
void Diagram::drawPoints(std::vector<QPointF>* pts, PointStyle* s, QPainter& painter) {
	painter.save();
	// Set point properties
	if (s->m_fill) 			// Set fill
		painter.setBrush(QBrush(s->m_fillColor));
	else
		painter.setBrush(Qt::NoBrush);

	if (s->m_outlineShow) {	// Set outline
		QPen pen(s->m_outlineColor);
		pen.setWidthF(s->m_outlineWidth);
		painter.setPen(pen);
	}
	else
		painter.setPen(Qt::NoPen);
	if (pts->size() < 400) {	// When quantity is small, draw one by one
		for (const QPointF& pt : *pts) {
			if (clipPt(pt, m_visibleRect)) {	// 1. Clip data (points, directly filter)
				painter.drawEllipse(dataToScreen(pt), s->m_r, s->m_r);// Draw ellipse
			}
		}
	}
	else {		// When quantity is large, simplify drawing process
		painter.setRenderHint(QPainter::Antialiasing, false);
		QPointF screenPoint;
		for (const QPointF& pt : *pts) {
			if (clipPt(pt, m_visibleRect)) {	// 1. Clip data (points, directly filter)
				screenPoint = dataToScreen(pt);
				// Draw 9 points, increase point size
				painter.drawRect(QRectF(screenPoint.x() - 1, screenPoint.y() - 1, 3, 3));
			}
		}
	}
	painter.restore();
}
void Diagram::drawPolygons(std::vector<LineUI>* lines, PolygonStyle* s, QPainter& painter) {
	if (lines->empty()) return;
	// Set polygon fill (not yet implemented)
	if (s->m_fill) {
		painter.setBrush(QBrush(s->m_fillColor));
	}
	else {
		painter.setBrush(Qt::NoBrush);
	}
	// Set outline
	// Set line properties
	QPen pen(s->m_outlineColor);		// Set color
	pen.setWidthF(s->m_outlineWidth);   // Set line width
	QPen penPolyLine = pen;
	if (s->m_outlineShow) {
		float width = lines->at(0).width * m_scaleFactor;
		if (width > 1 && width < 400) {
			penPolyLine.setWidthF(width);
			painter.setPen(penPolyLine);
		}
		else
			painter.setPen(pen);
	}
	else
		painter.setPen(Qt::NoPen);

	// Iterate through all polygon line segments and draw (the following method for drawing polygons cannot implement filling for now)
	for (LineUI line : *lines) {	// Cannot use reference, clipLine will modify line
		// 1. Clip line segment (need to implement clipLine function here)
		if (clipLine(line, m_visibleRect)) {
			// 2. Convert to screen coordinates and draw
			QLineF screenLine(dataToScreen(line.p1()), dataToScreen(line.p2()));
			painter.drawLine(screenLine);
		}
	}
}
void Diagram::drawLinePoints(std::vector<LineUI>* lines, PointStyle* s, QPainter& painter) {
	// Set endpoint properties
	if (s->m_fill) 			// Set fill
		painter.setBrush(QBrush(s->m_fillColor));
	else
		painter.setBrush(Qt::NoBrush);

	if (s->m_outlineShow) {	// Set outline
		QPen pen(s->m_outlineColor);
		pen.setWidthF(s->m_outlineWidth);
		painter.setPen(pen);
	}
	else
		painter.setPen(Qt::NoPen);
	for (const LineUI& line : *lines) {
		QPointF pt1 = line.p1(), pt2 = line.p2();
		if (clipPt(pt1, m_visibleRect)) {	// 1. Clip data (points, directly filter)
			painter.drawEllipse(dataToScreen(pt1), s->m_r, s->m_r);// Draw ellipse
		}
		if (clipPt(pt2, m_visibleRect)) {	// 1. Clip data (points, directly filter)
			painter.drawEllipse(dataToScreen(pt2), s->m_r, s->m_r);// Draw ellipse
		}
	}
}
void Diagram::drawCircles(std::vector<CircleUI>* circles, PolygonStyle* s, QPainter& painter) {
	if (circles->empty()) return;
	if (s->m_fill) 			// Set fill
		painter.setBrush(QBrush(s->m_fillColor));
	else
		painter.setBrush(Qt::NoBrush);

	if (s->m_outlineShow) {	// Set outline
		QPen pen(s->m_outlineColor);
		pen.setWidthF(s->m_outlineWidth);
		painter.setPen(pen);
	}
	else
		painter.setPen(Qt::NoPen);
	float radius = 1;
	for (const CircleUI& cc : *circles) {
		if (clipPt(cc, m_visibleRect)) {	// 1. Clip data (points, directly filter)
			radius = cc.radius * m_scaleFactor;
			painter.drawEllipse(dataToScreen(cc), radius, radius);// Draw ellipse
		}
	}
}

//5.1 Data coordinates to screen coordinates
QPointF Diagram::dataToScreen(const QPointF& dataPoint) const {
	return m_transform.map(dataPoint);
}
LineUI Diagram::dataToScreen(const LineUI& dataLine) const {
	return m_transform.map(dataLine);
}
QPointF Diagram::screenToData(const QPointF& screenPoint) const {
	QPointF dataPoint = m_transform.inverted().map(screenPoint);
	return { dataPoint.x(), dataPoint.y() };
}
void Diagram::updateTransform() {
	m_transform = QTransform();
	// 1. Apply translation (last executed transformation)
	m_transform.translate(m_panOffset.x(), m_panOffset.y());

	// 2. Apply scaling
	m_transform.scale(m_scaleFactor, m_scaleFactor);
	// 3. Rotation and flipping
	if (m_config->m_rotate_90)
		m_transform.rotate(90);
	if (m_config->m_flip_left_right)
		m_transform.scale(-1, 1);
	if (m_config->m_flip_up_down)
		m_transform.scale(1, -1);
}

//5.2 Line clipping
bool Diagram::clipLine(QPointF& p1, QPointF& p2, const QRectF& viewport) const {
	// Use Liang-Barsky algorithm for clipping
	qreal x0 = p1.x(), y0 = p1.y();
	qreal x1 = p2.x(), y1 = p2.y();

	qreal t0 = 0.0, t1 = 1.0;
	qreal dx = x1 - x0, dy = y1 - y0;

	// Iterate through all clipping boundaries
	for (int edge = 0; edge < 4; ++edge) {
		qreal p = 0, q = 0;

		switch (edge) {
		case 0: p = -dx; q = x0 - viewport.left(); break; // left
		case 1: p = dx; q = viewport.right() - x0; break; // right
		case 2: p = -dy; q = y0 - viewport.top(); break; // top
		case 3: p = dy; q = viewport.bottom() - y0; break; // bottom
		}

		if (p == 0) {
			if (q < 0) return false; // Line is parallel and outside clipping region
		}
		else {
			qreal r = q / p;
			if (p < 0) {
				if (r > t1) return false;
				if (r > t0) t0 = r;
			}
			else {
				if (r < t0) return false;
				if (r < t1) t1 = r;
			}
		}
	}
	p1.setX(x0 + t0 * dx);
	p1.setY(y0 + t0 * dy);
	p2.setX(x0 + t1 * dx);
	p2.setY(y0 + t1 * dy);
	return true;
}
bool Diagram::clipLine(LineUI& line, const QRectF& viewport) const {
	// Use Liang-Barsky algorithm for clipping
	qreal x0 = line.p1().x(), y0 = line.p1().y();
	qreal x1 = line.p2().x(), y1 = line.p2().y();

	qreal t0 = 0.0, t1 = 1.0;
	qreal dx = x1 - x0, dy = y1 - y0;

	// Iterate through all clipping boundaries
	for (int edge = 0; edge < 4; ++edge) {
		qreal p = 0, q = 0;

		switch (edge) {
		case 0: p = -dx; q = x0 - viewport.left(); break; // left
		case 1: p = dx; q = viewport.right() - x0; break; // right
		case 2: p = -dy; q = y0 - viewport.top(); break; // top
		case 3: p = dy; q = viewport.bottom() - y0; break; // bottom
		}

		if (p == 0) {
			if (q < 0) return false; // Line is parallel and outside clipping region
		}
		else {
			qreal r = q / p;
			if (p < 0) {
				if (r > t1) return false;
				if (r > t0) t0 = r;
			}
			else {
				if (r < t0) return false;
				if (r < t1) t1 = r;
			}
		}
	}
	line.setLine(x0 + t0 * dx, y0 + t0 * dy, x0 + t1 * dx, y0 + t1 * dy);
	return true;
}
bool Diagram::clipPt(const QPointF& p1, const QRectF& viewport) const {
	if (p1.x() < viewport.left() || p1.x() > viewport.right() ||
		p1.y() < viewport.top() || p1.y() > viewport.bottom()) {
		return false;
	}
	return true;
}


//########################## Event Management (Protected) ##########################

//1. Comprehensive events: [Initialize display], [resize], [Timer refresh], [Paint event]
void Diagram::showEvent(QShowEvent* event) {
	// Automatically called when the control is first displayed or becomes visible from a hidden state, generally used to perform initialization operations
	QWidget::showEvent(event);
}
void Diagram::resizeEvent(QResizeEvent* event) {
	QWidget::resizeEvent(event);
	//initViewTransform();

	if (!m_initialized && width() > 10 && height() > 10) {
		m_initialized = true;
		initViewTransform(); // Correctly calculate m_scaleFactor
	}
	if (m_initialized)
		update();
}
void Diagram::timerEvent(QTimerEvent* event) {
	if (event->timerId() == m_moveTimer.timerId()) {
		m_moveTimer.stop(); // Single trigger
		//moveRun();
	}
}
void Diagram::paintEvent(QPaintEvent* event) {
	QPainter painter(this);
	m_config->m_r = m_scaleFactor >= 5 ? m_scaleFactor / 3 : 2;	// Radius for drawing solid points
	//2. Drawing
	if (m_cacheValid) {		// Use cache for fast drawing
		painter.drawPixmap(m_cacheOffset, m_canvasCache);
	}
	else {					// Normal drawing process
		paintVisibleContent(event->rect(), painter);
	}
	if (m_slLine.showInfo && !m_slLine.infoText.isEmpty())
		paintContent(painter);

}

//2. Mouse events
void Diagram::mousePressEvent(QMouseEvent* event) {
	QPointF clickPt = screenToData(event->pos());
	m_pressPos = event->pos();
	if (event->button() == Qt::LeftButton) {
		m_config->painState = PUSH_LINE;
		m_targetSelected = selectTargetBegin(clickPt);
		if (m_targetSelected)
			emit leftPressed(&m_slLine);
	}
	else if (event->button() == Qt::MiddleButton) {
		m_config->painState = MOVE_CANVAS;	// Move canvas
		setCursor(Qt::ClosedHandCursor); // Change cursor shape
		moveCanvasBegin(clickPt);		// Generate cache area
	}
}
void Diagram::mouseMoveEvent(QMouseEvent* event) {
	m_movePos = event->pos();
	if (!m_moveTimer.isActive()) {
		m_moveTimer.start(20, this);
	}
	if (m_config->painState == PUSH_LINE) {
		if (m_targetSelected)
			selectTargetActivate();
	}
	else if (m_config->painState == MOVE_CANVAS) {
		moveCanvasActivate();		// Continuous update, draw moving canvas
	}
	// Hover detection
	if (m_config->painState == NONE) {
		QPointF hoverPt = screenToData(event->pos());
		checkHoverTarget(hoverPt);
	}
}
void Diagram::mouseReleaseEvent(QMouseEvent* event) {
	m_releasePos = event->pos();
	if (m_config->painState == PUSH_LINE) {
		if (m_slLine.lineSelected || m_slLine.pinSelected) {
			selectTargetEnd(screenToData(m_releasePos));
		}
	}
	else if (m_config->painState == MOVE_CANVAS) {
		setCursor(Qt::ArrowCursor); // Restore cursor shape
		moveCanvasEnd(m_releasePos);
	}
	if (event->button() == Qt::LeftButton) {
		m_config->painState = NONE;
	}
	else if (event->button() == Qt::MiddleButton) {
		m_config->painState = NONE;
	}
}
void Diagram::leaveEvent(QEvent* event) {
	QWidget::leaveEvent(event);

	// When mouse leaves the window, clear mouse information
	if (m_slLine.showInfo) {
		m_slLine.clear();
		m_slLine.showInfo = false;
		update(); // Trigger repaint, clear information box
	}
}
void Diagram::wheelEvent(QWheelEvent* event) {
	if (m_config->painState == NONE) {
		int delta = event->angleDelta().y();		// Mouse wheel rotation degrees
		QPointF mouseScenePos = event->position();	// Floating point coordinates
		wheelEventRun(delta, mouseScenePos);
		event->accept(); // Prevent event from continuing to propagate
	}
}
//3. Mouse event execution logic


// Content executed when scrolling the mouse wheel
void Diagram::wheelEventRun(const int& delta, const QPointF& screenPos) {
	// 1. Get mouse position in data coordinate system
	QPointF mouseDataPos = m_transform.inverted().map(screenPos);

	// 2. Calculate zoom factor (nonlinear scaling is smoother)
	const qreal zoomFactor = 1.1; // Zoom coefficient for each wheel step
	qreal scaleFactor = (delta > 0) ? zoomFactor : 1.0 / zoomFactor;

	// 3. Limit zoom range
	qreal newScale = m_scaleFactor * scaleFactor;
	newScale = qBound(0.0001, newScale, 10000.0); // Limit between 0.001~1000 times

	// 4. Calculate translation to keep mouse point unchanged
	QPointF mousePosBefore = m_transform.map(mouseDataPos);
	m_scaleFactor = newScale;
	updateTransform(); // Update transformation matrix first
	QPointF mousePosAfter = m_transform.map(mouseDataPos);
	m_panOffset += mousePosBefore - mousePosAfter;

	// 5. Apply new transformation
	updateTransform();
	update();
}

//########################## Event Execution (Private) ##########################

// When mouse hovers, capture points or lines, display coordinates and line length
void Diagram::checkHoverTarget(const QPointF& hoverPt) {
	SelectedTarget tempSlLine = m_slLine;	// Save current selection state
	m_slLine.clear();
	m_slLine.showInfo = false; // Keep information display state independent

	bool foundTarget =		// Foreground layer, background layer detection, detect points first
		selectTarget(hoverPt, true, m_pts2, m_ss_pts2) ||
		selectTarget(hoverPt, true, m_linePts2, m_ss_linePts2) ||
		selectTarget(hoverPt, true, m_pts1, m_ss_pts1) ||
		selectTarget(hoverPt, true, m_linePts1, m_ss_linePts1) ||
		selectTarget(hoverPt, true, m_lines2, m_ss_lines2) ||
		selectTarget(hoverPt, true, m_polys2, m_ss_polys2) ||
		selectTarget(hoverPt, true, m_lines1, m_ss_lines1) ||
		selectTarget(hoverPt, true, m_polys1, m_ss_polys1);

	if (foundTarget) {
		// Keep information display, but clear selection state (avoid affecting actual mouse operations)
		m_slLine.pinSelected = false;
		m_slLine.lineSelected = false;
		m_slLine.isMoving = false;
		m_slLine.linesToRun = nullptr;
	}
	else {
		// No object captured, display current mouse position coordinates
		m_slLine.infoText = QString("%1, %2")
			.arg(hoverPt.x(), 0, 'f', 2)
			.arg(hoverPt.y(), 0, 'f', 2);
		m_slLine.infoPosition = hoverPt;
		m_slLine.showInfo = true;

	}
	update();
	if (tempSlLine.pinSelected || tempSlLine.lineSelected) {
		m_slLine = tempSlLine;
	}
}
// Mouse left button, capture points or lines, capture on press, return offset and captured information on release
bool Diagram::selectTargetBegin(const QPointF& clickPt) {
	if (
		selectTarget(clickPt, false, m_pts2, m_ss_pts2) ||		// Foreground layer
		selectTarget(clickPt, false, m_lines2, m_ss_lines2) ||
		selectTarget(clickPt, false, m_polys2, m_ss_polys2) ||
		selectTarget(clickPt, false, m_linePts2, m_ss_linePts2) ||
		selectTarget(clickPt, false, m_pts1, m_ss_pts1) ||		// Background layer
		selectTarget(clickPt, false, m_lines1, m_ss_lines1) ||
		selectTarget(clickPt, false, m_polys1, m_ss_polys1) ||
		selectTarget(clickPt, false, m_linePts1, m_ss_linePts1)
		) {
		return true;
	}
	return false;
}
bool Diagram::selectTargetActivate() {
	m_slLine.isMoving = true;
	QPointF currentPtPos = screenToData(m_movePos);
	m_slLine.offset = currentPtPos - m_slLine.nearistPt;
	emit leftPressedMove(&m_slLine);
	update();
	return false;
}
bool Diagram::selectTargetEnd(const QPointF& releasePt) {
	m_slLine.isMoving = false;
	m_slLine.offset = releasePt - m_slLine.nearistPt;
	emit leftReleaseed(&m_slLine);
	m_slLine.clear();
	update();
	return false;
}
// Mouse middle button, move canvas
void Diagram::moveCanvasBegin(const QPointF& clickPt) {
	m_canvasCache = grab(this->rect());
	m_cacheValid = true;
}
void Diagram::moveCanvasActivate() {
	if (!m_cacheValid) return;
	m_cacheOffset = m_movePos - m_pressPos;
	update();
}
void Diagram::moveCanvasEnd(const QPoint& releasePos) {
	// Calculate actual offset (data coordinate system)
	m_panOffset += releasePos - m_pressPos;
	updateTransform();// Update actual transformation
	m_cacheValid = false;
	update();
	m_canvasCache = QPixmap(); // Clear cache
}

// Snap threshold (data coordinate system)
template<typename DataType, typename StyleType>
bool Diagram::selectTarget(const QPointF& posPt, const bool& hover, std::vector<DataType*>& dataVector, std::vector<StyleType*>& styleVector) {
	for (int i = 0; i < styleVector.size(); i++) {
		if (setSelections(posPt, hover, dataVector[i], styleVector[i])) {
			return true;
		}
	}
	return false;
}
bool Diagram::setSelections(const QPointF& clickPt, const bool& hover, std::vector<QPointF>* candidates, PointStyle* s) {
	if (s->m_type <= 0 || !s->isVisible()) {
		return false;
	}
	double snapThreshold = m_snapThresholdPixels / m_scaleFactor;
	if (s->m_type <= 1000 || (s->m_type > 2000 && s->m_type <= 3000) || (s->m_type == 6001 && hover)) {	// All selectable
		// Set m_wires as push lines
		bool selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else {
		return false;
	}
	return false;
}
bool Diagram::setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, LineStyle* s) {
	if (s->m_type <= 0 || !s->isVisible()) {
		return false;
	}
	double snapThreshold = m_snapThresholdPixels / m_scaleFactor;
	bool selected = false;
	if (s->m_type <= 1000 || (s->m_type == 6001 && hover)) {		// All selectable
		selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
		else selected = setSelectedLine(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else if (s->m_type <= 2000) {	// Selectable line segments
		selected = setSelectedLine(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else if (s->m_type <= 3000) {	// Only endpoints selectable
		selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else {
		return false;
	}
	return false;
}
bool Diagram::setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, PolygonStyle* s) {
	if (s->m_type <= 0 || !s->isVisible()) {
		return false;
	}
	double snapThreshold = m_snapThresholdPixels / m_scaleFactor;
	bool selected = false;
	if (s->m_type <= 1000 || (s->m_type == 6001 && hover)) {		// All selectable
		selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
		else selected = setSelectedLine(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else if (s->m_type <= 2000) {	// Selectable line segments
		selected = setSelectedLine(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else if (s->m_type <= 3000) {	// Only endpoints selectable
		selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else {
		return false;
	}
	return false;
}
bool Diagram::setSelections(const QPointF& clickPt, const bool& hover, std::vector<LineUI>* candidates, PointStyle* s) {
	if (s->m_type <= 0 || !s->isVisible()) {
		return false;
	}
	double snapThreshold = m_snapThresholdPixels / m_scaleFactor;
	bool selected = false;
	if (s->m_type <= 1000 || (s->m_type > 2000 && s->m_type <= 3000) || (s->m_type == 6001 && hover)) {		// All selectable
		selected = setSelectedPt(clickPt, *candidates, snapThreshold);
		if (selected) return true;
	}
	else {
		return false;
	}
	return false;
}

bool Diagram::setSelectedPt(const QPointF& clickPt, const vector<QPointF>& line, double snapThreshold) {
	qreal pickPointTolerance = m_config->m_r / m_scaleFactor;		// Convert to data coordinate system (point radius)
	pickPointTolerance *= 0.6;									// Reduce snap range, 0.8 times the visual point radius
	pickPointTolerance = pickPointTolerance > snapThreshold ? pickPointTolerance : snapThreshold;
	for (const QPointF& pt : line) {
		if (LineUI(clickPt, pt).length() < pickPointTolerance) {
			m_slLine.pinSelected = true;
			m_slLine.nearistPt = pt;
			// Screen display information
			m_slLine.infoText = QString("p ( %1, %2 )").arg(pt.x(), 0, 'f', 2).arg(pt.y(), 0, 'f', 2);
			m_slLine.infoPosition = pt;
			m_slLine.showInfo = true;
			return true;
		}
	}
	return false;
}
bool Diagram::setSelectedPt(const QPointF& clickPt, const LineUI& line, double snapThreshold) {
	qreal pickPointTolerance = m_config->m_r / m_scaleFactor;		// Convert to data coordinate system (point radius)
	pickPointTolerance *= 0.6;									// Reduce snap range, 0.8 times the visual point radius
	pickPointTolerance = pickPointTolerance > snapThreshold ? pickPointTolerance : snapThreshold;
	if (LineUI(clickPt, line.p1()).length() < pickPointTolerance) {
		m_slLine.pinSelected = true;
		m_slLine.nearistPt = line.p1();
		// Screen display information
		m_slLine.infoText = QString("p1 ( %1, %2 )").arg(line.p1().x(), 0, 'f', 2).arg(line.p1().y(), 0, 'f', 2);
		m_slLine.infoPosition = line.p1();
		m_slLine.showInfo = true;
		return true;
	}
	else if (LineUI(clickPt, line.p2()).length() < pickPointTolerance) {
		m_slLine.pinSelected = true;
		m_slLine.nearistPt = line.p2();
		// Screen display information
		m_slLine.infoText = QString("p2 ( %1, %2 )").arg(line.p2().x(), 0, 'f', 2).arg(line.p2().y(), 0, 'f', 2);
		m_slLine.infoPosition = line.p2();
		m_slLine.showInfo = true;
		return true;
	}
	return false;
}
bool Diagram::setSelectedPt(const QPointF& clickPt, vector<LineUI>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedPt(clickPt, line, snapThreshold);
		if (m_slLine.pinSelected) {
			m_slLine.linesToRun = &lines;
			return true;
		}
	}
	return false;
}
bool Diagram::setSelectedPt(const QPointF& clickPt, vector<vector<LineUI>>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedPt(clickPt, line, snapThreshold);
		if (m_slLine.pinSelected) {
			return true;
		}
	}
	return false;
}
bool Diagram::setSelectedPt(const QPointF& clickPt, vector<vector<vector<LineUI>>>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedPt(clickPt, line, snapThreshold);
		if (m_slLine.pinSelected) {
			return true;
		}
	}
	return false;
}

bool Diagram::setSelectedLine(const QPointF& clickPt, const LineUI& line, double snapThreshold) {
	const QPointF p1 = line.p1();
	const QPointF p2 = line.p2();
	// Handle zero-length line segments
	if (p1 == p2) {
		if (sqrt(pow(clickPt.x() - p1.x(), 2) + pow(clickPt.y() - p1.y(), 2)) < const_minValue);
		return false;
	}
	// Vector calculation
	const QPointF AP = clickPt - p1;
	const QPointF AB = p2 - p1;
	const double dotProduct = AP.x() * AB.x() + AP.y() * AB.y();
	const double abLengthSquared = AB.x() * AB.x() + AB.y() * AB.y();
	const double ratio = qBound(0.0, dotProduct / abLengthSquared, 1.0);
	// Calculate nearest point
	const QPointF nearestPoint = p1 + ratio * AB;
	const double distance = sqrt(pow(clickPt.x() - nearestPoint.x(), 2) + pow(clickPt.y() - nearestPoint.y(), 2));
	// Automatic line snapping logic
	qreal pinMinDist = m_config->m_r / m_scaleFactor;
	if (distance < snapThreshold) {
		m_slLine.lineSelected = true;
		m_slLine.sLine = line;
		m_slLine.nearistPt = nearestPoint;
		// Screen display information
		double lineLength = sqrt(abLengthSquared);
		m_slLine.infoText = QString("p1 ( %1, %2 )\np2 ( %3, %4 )\nlen: %5")
			.arg(p1.x(), 0, 'f', 2).arg(p1.y(), 0, 'f', 2)
			.arg(p2.x(), 0, 'f', 2).arg(p2.y(), 0, 'f', 2)
			.arg(lineLength, 0, 'f', 2);
		m_slLine.infoPosition = nearestPoint;
		m_slLine.showInfo = true;
		return true;
	}
	return false;
}
bool Diagram::setSelectedLine(const QPointF& clickPt, vector<LineUI>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedLine(clickPt, line, snapThreshold);
		if (m_slLine.lineSelected) {
			m_slLine.linesToRun = &lines;
			return true;
		}
	}
	return false;
}
bool Diagram::setSelectedLine(const QPointF& clickPt, vector<vector<LineUI>>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedLine(clickPt, line, snapThreshold);
		if (m_slLine.lineSelected) {
			return true;
		}
	}
	return false;
}
bool Diagram::setSelectedLine(const QPointF& clickPt, vector<vector<vector<LineUI>>>& lines, double snapThreshold) {
	for (auto& line : lines) {
		setSelectedLine(clickPt, line, snapThreshold);
		if (m_slLine.lineSelected) {
			return true;
		}
	}
	return false;
}
