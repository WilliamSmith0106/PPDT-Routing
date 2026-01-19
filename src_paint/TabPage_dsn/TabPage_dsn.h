#pragma once
#include <QWidget>
#include "AlgorithmLink/AlgorithmLink_dsn.h"
#include "../../src_baseClasses/TabPage.h"
#include "../../src_config/configUI.h"
#include "../../src_config/DiagramStyle.h"
#include "DataParser_dsn.h"

class TabPage_dsn : public TabPage
{
    Q_OBJECT
public:
    TabPage_dsn(QString qFullName, QString qFilePath, QString qFileName, QWidget* parent)
        : TabPage(qFullName, qFilePath, qFileName, parent) {
		m_data = new Data_dsn();
		m_parser = new DataParser_dsn(m_data);
    }

    ~TabPage_dsn() {
        delete m_algm;
        delete m_parser;
		delete m_data;
	}
private:
    Data_dsn* m_data;;	        // Data object
    DataParser_dsn* m_parser;	// Data parser
    AlgorithmLink_dsn* m_algm = nullptr;

private:
    ConfigUI* m_config = nullptr;	// Configuration information
    void fillData(ConfigUI* conf);
    void refreshUI();
public:
    void action_reset() {};				// Reset (only for reading map-like txt files)
    void action_random_pin() {};			// Regenerate random pins
    void pin_decrease() {};				// Decrease pin count
    void action_pin_increase() {};			// Increase pin count
    void action_save_case() {};			// Save case
    void action_save_image() {};			// Save image
    void action_write_pre() {};			// Save preprocessing results
    void action_write_res() {};			// Append results to preprocessing results
    void pressRunButton();

    void onConfigChanged(int type = 0);
    void tranformChanged();
    void leftPressRun(SelectedTarget* target);
    void leftPressMoveRun(SelectedTarget* target);
    void leftReleaseRun(SelectedTarget* target);

};