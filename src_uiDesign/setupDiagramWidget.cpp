#include "GeoDisplay.h"
#include <QMessageBox>

//4. Set up drawing window (tab pages)
void GeoDisplay::setupDiagramWidget() {
	// Set style
	m_tabWidget->setStyleSheet(
		// Set background for entire tab bar (including empty areas)
		"QTabBar {"
		"    background-color: #CCCCCC;"  // Gray background
		"    border-bottom: 1px solid #999999;" // Bottom border line
		"    min-height: 24px;"           // Set minimum height (controls overall height)
		"    height: 24px;"               // Fixed height (ensure effectiveness)
		"}"

		// Set style for individual tab pages
		"QTabBar::tab {"
		"    background: #FFFFFF;"      // Unselected tab color (white #FFFFFF)
		"    color: #333333;"           // Text color
		"    border: none;"             // Remove all borders (original setting)
		"    border-right: 1px solid #999999;" // Only keep right border
		"    padding: 5px 12px;"
		"    margin-right: 0px;"        // Remove tab spacing (border already acts as separator)
		"    margin-left: -1px;"        // Make tabs stick together (avoid border overlap thickening)
		"}"

		// Selected state
		"QTabBar::tab:selected {"
		"    background: #DDDDDD;"  // Light blue background (white #FFFFFF, light gray #DDDDDD, blue #1565C0, gray blue #778899)
		"    border-top: 3px solid #778899;"  // Thicker dark blue bar
		"}"

		// Hover state
		"QTabBar::tab:hover {"
		"    background: #EEEEEE;"
		"}"

		// Background for tab bar corner controls (like scroll buttons)
		"QTabBar::scroller {"
		"    background: #CCCCCC;"
		"}"

		// Set QTabWidget's pane style (optional)
		"QTabWidget::pane {"
		"    border-top: 1px solid #999999;" // Connect with tab bar border
		"}"
	);

	// Connect signal slots
	connect(m_tabWidget, &QTabWidget::currentChanged, this, &GeoDisplay::onTabChanged);		// Listen for tab page switch
	connect(m_tabWidget, &QTabWidget::tabCloseRequested, this, &GeoDisplay::closeTab);		// Listen for tab page close request
}


// Tab page related slot functions
//1. Three functions managing tab page lifecycle
void GeoDisplay::addDiagramTab(const QString& qFullFileName, const int readType, QWidget* parent) {
	BasicData* newData = new BasicData();
	newData->acquired_ = false;
	m_dp.setData(newData);		// Update data processor
	newData->acquired_ = m_dp.readFile(qFullFileName, readType);		// Parse file, store data in data object as bridge between frontend and backend
	if (newData->acquired_) {
		//1. Create new control
		//1.1 Create new drawing control, set control parameters
		DiagramWidget* newTab = new DiagramWidget(newData, m_config, parent);		// Will trigger drawing
		newTab->m_qFilePath = m_dp.m_qFilePath;
		newTab->m_qFileName = m_dp.m_qFileName;
		newTab->m_data_type = readType;
		newData->runFinished_ = false;		//newTab->m_data and newData are actually the same object

		//2. Create a new tab page, add the drawing control to it
		int tabIndex = m_tabWidget->addTab(newTab, newTab->m_qFileName);	// Create new tab page
		m_tabWidget->setCurrentIndex(tabIndex);		// Set current tab page index
		m_tabWidget->setTabsClosable(true);			// Enable close button
		//3. Add new tab page and set as current page
		m_curTap = newTab;
		//4. Set other parameters of current drawing control
	}
	else {
		delete newData;
		newData = nullptr;
		QMessageBox::warning(this, "错误", "文件解析失败");
	}
	m_config->write_config();
}
void GeoDisplay::onTabChanged(int index) {
	// This function is also triggered when creating a new tab page
	if (index >= 0) {
		m_curTap = qobject_cast<DiagramWidget*>(m_tabWidget->widget(index));
		// Safety check
		if (!m_curTap) {
			qWarning() << "标签页" << index << "不是DiagramWidget类型";
			m_curTap = nullptr;
		}
	}
	else {
		m_curTap = nullptr; // Set to null when no tab pages
	}
}
void GeoDisplay::closeTab(int index) {
	if (QWidget* tab = m_tabWidget->widget(index)) {
		DiagramWidget* diagram = qobject_cast<DiagramWidget*>(tab);
		// Delete data (need extra handling if it's current tab page)
		if (diagram && diagram->m_data) {
			if (diagram == m_curTap) {// If closing the current tab page, need to clear pointer
				m_curTap = nullptr;
			}
			delete diagram->m_data;
			diagram->m_data = nullptr;
		}
		// Remove tab page
		m_tabWidget->removeTab(index);
		tab->deleteLater();
		if (m_tabWidget->count() == 0) {// If no tab pages left, clear pointer
			m_curTap = nullptr;
		}
	}
}
//2. Add a data entry to current tab page
void GeoDisplay::actionAddData(QString& qFullFileName, const int readType) {
	BasicData* curData = m_curTap->m_data;
	curData->acquired_ = false;
	m_dp.setData(curData);
	curData->acquired_ = m_dp.readFile(qFullFileName, readType);
	if (curData->acquired_) {
		m_curTap->m_data_type = readType;
		curData->runFinished_ = false;
		m_curTap->initViewTransform();		// Need to trigger drawing initialization
	}
	else {
		curData->acquired_ = true;
		QMessageBox::warning(this, "错误", "新增文件解析失败");
	}
}