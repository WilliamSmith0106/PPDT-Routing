#include "GeoDisplay.h"

using namespace std;

//2. Set up option panel
void GeoDisplay::setupOptionPanel() {
	m_optionsPanel = new OptionsPanel(m_config, this);

	// Create dock window and set up option panel
	m_optionsDock = new QDockWidget("选项面板", this);
	m_optionsDock->setWidget(m_optionsPanel);
	m_optionsDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);

	// Add dock window to left side of main window
	addDockWidget(Qt::LeftDockWidgetArea, m_optionsDock);

	// Add a menu item to control panel show/hide
	ui.actionOptionPanel->setCheckable(true);
	ui.actionOptionPanel->setChecked(true);
}

void GeoDisplay::setupOptionPanel_slots() {
	// Option panel emits signals, tab pages respond to signals
	connect(m_optionsPanel, &OptionsPanel::update_request_signal, this, [this]() {			// Request redraw
		if (m_curPage != nullptr)
			m_curPage->onConfigChanged();
		});
	connect(m_optionsPanel, &OptionsPanel::tranform_init_request_signal, this, [this]() {	// Request redraw, update transformation matrix
		if (m_curPage != nullptr)
			m_curPage->tranformChanged();
		});
	connect(m_optionsPanel, &OptionsPanel::pressMSTReset, this, [this]() {					// MST reset, regenerate Steiner tree based on preVia
		if (m_curPage != nullptr)
			m_curPage->onConfigChanged(2);
		});
	connect(m_optionsPanel, &OptionsPanel::open_res_request_signal, this, [this]() {		// Open result data for viewing
		action_open();
		});
	connect(m_optionsPanel, &OptionsPanel::reset_request_signal, this, [this]() {			// Reset
		m_curPage->action_reset();
		});
	connect(m_optionsPanel, &OptionsPanel::random_pin_request_signal, this, [this]() {		// Regenerate random pins
		m_curPage->action_random_pin();
		});
	connect(m_optionsPanel, &OptionsPanel::pin_decrease_request_signal, this, [this]() {	// Decrease pin count
		m_curPage->pin_decrease();
		});
	connect(m_optionsPanel, &OptionsPanel::pin_increase_request_signal, this, [this]() {	// Increase pin count
		m_curPage->action_pin_increase();
		});
	connect(m_optionsPanel, &OptionsPanel::save_case_request_signal, this, [this]() {		// Save case
		m_curPage->action_save_case();
		});
	connect(m_optionsPanel, &OptionsPanel::save_image_request_signal, this, [this]() {		// Save image
		m_curPage->action_save_image();
		});
	connect(m_optionsPanel, &OptionsPanel::write_pre_request_signal, this, [this]() {		// Save preprocessing results
		m_curPage->action_write_pre();
		});
	connect(m_optionsPanel, &OptionsPanel::write_res_request_signal, this, [this]() {		// Append results to preprocessing results
		m_curPage->action_write_res();
		});
   
	connect(m_optionsPanel, &OptionsPanel::PPTT_run_request_signal, this, [this]() {		// Run PPDT
		m_curPage->pressRunButton();
		});
	connect(m_optionsPanel, &OptionsPanel::poly_run_request_signal, this, [this]() {		// Run polygon algorithm
		m_curPage->pressRunButton();
		});
}

