#include "GeoDisplay.h"
#include <QMessageBox>


//4. Set up drawing window style (tab pages)
void GeoDisplay::setupTabWidget() {
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
void GeoDisplay::addTabWidget(const QString& qFullFileName) {
	//1. Create a new drawing control
	TabPage* newTab = TabPageFactory::instance().create(qFullFileName, m_tabWidget);
	if (!newTab) {
		QMessageBox::warning(this, "无法打开文件",
			QString("没有找到支持该文件类型的标签页: %1").arg(qFullFileName));
		return;
	}
	newTab->pageInit(m_config);
	//2. Create a new tab page, add the drawing control to it
	int tabIndex = m_tabWidget->addTab(newTab, newTab->fileName());
	m_tabWidget->setCurrentIndex(tabIndex);
	m_curPage = newTab;
	//3. Connect signal slots (tab page emits signals, option panel executes slot functions)
	connect(m_curPage, &TabPage::requestUpdatePanelUI, m_optionsPanel, &OptionsPanel::updatePanelUI);
}

void GeoDisplay::onTabChanged(int index) {
	// This function is also triggered when creating a new tab page
	if (index >= 0) {
		m_curPage = qobject_cast<TabPage*>(m_tabWidget->widget(index));
		// Safety check
		if (!m_curPage) {
			qWarning() << "标签页" << index << "不是DiagramWidget类型";
			m_curPage = nullptr;
		}
	}
	else {
		m_curPage = nullptr; // Set to null when no tab pages
	}
}
void GeoDisplay::closeTab(int index) {
	QWidget* toBeClosed = m_tabWidget->widget(index);
	if (!toBeClosed) return;

	//1. Record if it's the current page 
	bool isCurrent = (m_curPage == toBeClosed);

	//2. Remove from tab bar and automatically destroy 
	m_tabWidget->removeTab(index);          // Qt will delete toBeClosed
	delete toBeClosed;
	toBeClosed = nullptr;  // Avoid dangling pointer

	//3. Adjust m_curPage 
	if (isCurrent) {                       // The closed page is exactly the current page
		int remaining = m_tabWidget->count();
		if (remaining > 0) {                // There are still pages left
			int newIdx = m_tabWidget->currentIndex(); // Qt has automatically adjusted after removal
			m_curPage = qobject_cast<TabPage*>(m_tabWidget->widget(newIdx));
		}
		else {                              // No more pages left
			m_curPage = nullptr;
		}
	}
	//4. Update configuration file information
	m_config->write_config();
}
//2. Add a data entry to current tab page
void GeoDisplay::actionAddData(QString& qFullFileName, const int readType) {
	//BasicData* curData = m_curPage->m_data;
	//curData->acquired_ = m_dp.readFile<BasicData>(qFullFileName, readType, curData);
	//if (curData->acquired_) {
	//	m_curTap->m_data_type = readType;
	//	curData->runFinished_ = false;
	//		m_curTap->initViewTransform();		// Need to trigger drawing initialization
	//}
	//else {
	//	curData->acquired_ = true;
	//		QMessageBox::warning(this, "Error", "Failed to parse new file")
	//}
}