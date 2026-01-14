#pragma once
#include <QObject>
#include <QPen>
#include <QColor>

enum PainState {
	NONE,           // Idle state (push line with left mouse button, move canvas with middle mouse button)
	PUSH_LINE,      // Push line state
	MOVE_CANVAS,    // Move canvas state
	DELETE,         // Delete state
	ROTATE          // Rotate state
};
class ConfigUI : public QObject {
	Q_OBJECT;
public:
	ConfigUI(QObject* parent) :QObject(parent) {};
	~ConfigUI() {}
	//1. Information that needs to be read and written
	QString m_qFilePath = "./data";
	QString m_qFileName = "example.txt";

	//2. Brush-related information
	int m_r = 2;  // Solid point radius
	QPen m_defaultPen = QPen(QColor(65, 105, 225), 2);	// Default brush
	QPen m_blackPen = QPen(Qt::black, 2);
	QPen m_resPen = QPen(Qt::green, 2);	// Default brush

	//3. Status information
	PainState painState = NONE;     // Drawing mode
	//4.1 bool values corresponding to view options
	bool m_flip_left_right = false;	//01. Flip X-axis (left-right)
	bool m_flip_up_down = true;		//02. Flip Y-axis (up-down)
	bool m_rotate_90 = false;		//03. Rotate 90 degrees
	//4.2 bool values corresponding to algorithm options
	bool m_preViaAlctOn = false;	//11. Pre via allocation
	bool m_GNDRouteOn = false;		//12. GND special routing
	bool m_VCCRouteOn = false;		//13. VCC special routing
	bool m_diffRouteOn = false;		//14. Differential routing
	bool m_onGrids = false;			//15. On grids
	//4.3 bool values corresponding to display options
	bool m_showObs1 = true;			//21. Show layer 1 obstacles
	bool m_showObs2 = false;		//22. Show layer 2 obstacles
	bool m_showFlyLines = true;		//23. Show flying lines
	bool m_showPahts = true;		//24. Show routing results
	bool m_showPPs = true;			//55. Show planning points
	bool m_show_vias = false;		//26. Show vias
	bool m_showPinCenters = true;	//27. Show Pin centers
	//4.4 bool values corresponding to other options
	bool m_postOn = false;			//51. Enable post-processing
	bool m_showTrees = true;		//52. Show trees
	bool m_debugFuncOn = false;		//53. Enable debug interrupt

	int m_gridType = 1;				//0 no grid, 1 line grid, 2 point grid
	int m_postMode = 1;
	float m_minimalScreenGridSize = 16;
	float m_gridSize = 50;

	int m_showTreeIndex = -1;

	//5. Custom input data
	QString m_flexibleOpt = "";	// String input from UI interface, can represent different mode options, etc.
	double m_doubleNum1 = 0.0;		// Decimal input from UI interface, can represent coordinates, separated by spaces
	double m_doubleNum2 = 0.0;		// Decimal input from UI interface
	int m_intNum3 = 0;				// Integer input from UI interface

	//5. Configuration information operations
	void configUpdate(const QString& qFilePath, const QString& qFileName);
	bool read_config();
	bool write_config()const;
};


