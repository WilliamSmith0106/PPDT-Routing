#include "GeoDisplay.h"

GeoDisplay::GeoDisplay(QWidget* parent) : QMainWindow(parent) {
	ui.setupUi(this);
	config_init();			// Interface initialization (establish basic variables for subsequent initialization of other modules)
	setupMenuBar();			//1. Set up menu bar
	setupOptionPanel();		//2. Set up option panel
	setupToolBar();			//3. Set up toolbar
	setupTabWidget();		//4. Set up drawing controls (tab pages)
	setup_slots();          // Finally, connect signal slots
	srand(time(0));
}

GeoDisplay::~GeoDisplay() {
	m_config->write_config();
	delete m_optionsPanel;
	delete m_optionsDock;
	delete m_config;
}

void GeoDisplay::config_init() {		// Configuration initialization

	//1. Initial interface layout
	// 1.1 Create central widget and main layout
	m_centralWidget = new QWidget(this);
	m_mainLayout = new QVBoxLayout(m_centralWidget);
	// 1.2 Set layout properties
	m_mainLayout->setContentsMargins(5, 5, 5, 5);  // Margins
	m_mainLayout->setSpacing(10);                 // Control spacing
	// 1.3 Set central widget as main window's central widget
	setCentralWidget(m_centralWidget);

	//2. Read configuration file, set initial file name
	m_config = new ConfigUI(this);	// Create configuration object
	m_config->read_config();			// Read configuration information

	//3. Initial interface configuration
	ui.actionTool_cmd->setCheckable(true);		// Show toolbar
	bool isToolbarVisible = ui.mainToolBar->isVisible();
	ui.actionTool_cmd->setChecked(isToolbarVisible);

	//4. Initialize tab page system
	m_tabWidget = new QTabWidget(this);
	m_tabWidget->setTabsClosable(true);
	setCentralWidget(m_tabWidget);
}

// Finally, connect signal slots
void GeoDisplay::setup_slots() {
	setupMenuBar_slots();               //1. Signal slots - Menu bar
	setupOptionPanel_slots();			//2. Signal slots - Option panel
	setupToolBar_slots();               //3. Signal slots - Toolbar (commands)
}

