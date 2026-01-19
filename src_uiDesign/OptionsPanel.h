#pragma once

#include <QWidget>
#include "ui_OptionsPanel.h"
#include "../src_config/configUI.h"
#include <QTabWidget>
#include <QCheckBox>
#include <QVBoxLayout>


class OptionsPanel : public QWidget
{
	Q_OBJECT

signals: // Signals
	// Signals sent to parent class
	void update_request_signal();			// Update drawing
	void tranform_init_request_signal();	// Initialize transformation matrix
	void pressMSTReset();					// Regenerate MST

	void open_res_request_signal();			// Open Res file for viewing
	void reset_request_signal();			// Reset
	void random_pin_request_signal();		// Regenerate random pins
	void pin_decrease_request_signal();		// Decrease pin count
	void pin_increase_request_signal();		// Increase pin count
	void save_case_request_signal();		// Save case
	void save_image_request_signal();		// Save image
	void write_pre_request_signal();		// Save preprocessing results
	void write_res_request_signal();		// Append results to preprocessing results
	void PPTT_run_request_signal();			// Run PPDT
	void poly_run_request_signal();			// Run polygon algorithm

public slots:
	void updatePanelUI(int size);

private: // Slot functions
	void paintChanged(int type = 0) {		// Triggered when drawing info changes, requests drawing update
		switch (type) {
		case 0:
			emit update_request_signal();
			break;
		case 1:
			emit tranform_init_request_signal();
			break;
		case 2:
			emit pressMSTReset();
			break;
		default:
			break;
		}
	}

public:
	explicit OptionsPanel(ConfigUI* conf, QWidget* parent);
	~OptionsPanel();

private:
	Ui::OptionsPanelClass ui;
	ConfigUI* m_config;			// Configuration information

private:
	void styleInit();
	void setPanelUI();
	void setup_panelSlots();
};
