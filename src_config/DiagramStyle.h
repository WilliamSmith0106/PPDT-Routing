#pragma once
#include <QPen>
#include <QBrush>
#include <QHash>
#include <vector>

// Diagram style base class
class DiagramStyle {
public:
	virtual ~DiagramStyle() = default;
	bool isVisible() const { return m_visible; };
	void setVisible(bool visible) { m_visible = visible; };

private:
	bool m_visible = true;

public:
	virtual QPen getPen() const = 0;
	virtual QBrush getBrush() const = 0;

};

// m_type selection flag description: m_type defaults to 0
// 0: Not selectable
// 1-999: Fully selectable
// 1001+: Line segment can only select line
// 2001+: Line segment can only select endpoints
// Point/Circle style
class PointStyle : public DiagramStyle {
public:
	QColor m_outlineColor;	// Outline color
	bool m_outlineShow;		// Whether outline is visible
	float m_outlineWidth;	// Outline width
	bool m_fill;			// Fill
	QColor m_fillColor;		// Fill color
	float m_r;				// Radius
	int m_type;				// Classification to determine if capturing
public:
	explicit PointStyle(
		const QColor& outlineColor = Qt::black,
		bool outlineShow = true,
		float outlineWidth = 1.0,
		bool fill = false,
		const QColor& fillColor = Qt::transparent,
		float r = 3.0,
		int type = 0
	) :
		m_outlineColor(outlineColor),
		m_outlineShow(outlineShow),
		m_outlineWidth(outlineWidth),
		m_fill(fill),
		m_fillColor(fillColor),
		m_r(r),
		m_type(type) {
	};

	QPen getPen() const override {
		return QPen(m_outlineColor, m_outlineWidth);
	}

	QBrush getBrush() const override {
		return m_fill ? QBrush(m_fillColor) : QBrush(Qt::NoBrush);
	}

	float radius() const { return m_r; }


};


// Line segment style
class LineStyle : public DiagramStyle {
public:
	QColor m_color;			// Color
	double m_width;			// Line width
	Qt::PenStyle m_style;	// Line style (solid, dashed, etc.)
	int m_type;				// Classification to determine if capturing

public:
	explicit LineStyle(
		const QColor& color = Qt::black,
		float width = 1.0f,
		Qt::PenStyle style = Qt::SolidLine,
		int type = 0
	) :
		m_color(color),
		m_width(width),
		m_style(style),
		m_type(type) {
	}
	QPen getPen() const override {
		return QPen(m_color, m_width, m_style);
	}
	QBrush getBrush() const override { return QBrush(Qt::NoBrush); }

};

// Polygon style
class PolygonStyle : public DiagramStyle {
public:
	QColor m_outlineColor;	// Outline color
	bool m_outlineShow;		// Whether outline is visible
	float m_outlineWidth;	// Outline width
	bool m_fill;			// Fill
	QColor m_fillColor;		// Fill color
	int m_type;
public:
	explicit PolygonStyle(
		const QColor& outlineColor = Qt::black,
		bool outlineShow = true,
		float outlineWidth = 1.0f,
		bool fill = false,
		const QColor& fillColor = Qt::blue,
		int type = 0
	) :
		m_outlineColor(outlineColor),
		m_outlineShow(outlineShow),
		m_outlineWidth(outlineWidth),
		m_fill(fill),
		m_fillColor(fillColor),
		m_type(type) {
	}

	QPen getPen() const override {
		return QPen(m_outlineColor, m_outlineWidth);
	}

	QBrush getBrush() const override {
		return m_fill ? QBrush(m_fillColor) : QBrush(Qt::NoBrush);
	}
};


class DiagramStyleManager {
public:
	DiagramStyleManager() {
		m_pointStyles[""] = PointStyle();
		m_lineStyles[""] = LineStyle();
		m_polygonStyles[""] = PolygonStyle();
	};
	~DiagramStyleManager() = default;

	// Get style
	PointStyle* getPointStyle(const QString& key);
	LineStyle* getLineStyle(const QString& key);
	PolygonStyle* getPolygonStyle(const QString& key);

	// Set style
	void setPointStyle(const QString& key, const PointStyle& style);
	void setPointStyle(const std::vector<QString>& keys, const std::vector<PointStyle>& styles);
	void setLineStyle(const QString& key, const LineStyle& style);
	void setLineStyle(const std::vector<QString>& keys, const std::vector<LineStyle>& styles);
	void setPolygonStyle(const QString& key, const PolygonStyle& style);
	void setPolygonStyle(const std::vector<QString>& keys, const std::vector<PolygonStyle>& styles);

private:
	QHash<QString, PointStyle> m_pointStyles;
	QHash<QString, LineStyle> m_lineStyles;
	QHash<QString, PolygonStyle> m_polygonStyles;

};
