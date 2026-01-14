#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_GeoDisplay.h"
#include <QDockWidget>
#include <QVBoxLayout> 
#include <QLabel>
#include "../src_config/configUI.h"
#include "OptionsPanel.h"
#include "../src_baseClasses/TabPage.h"

class OptionsPanel;

class GeoDisplay : public QMainWindow {
	Q_OBJECT

public:
	GeoDisplay(QWidget* parent = nullptr);
	~GeoDisplay();

public:
	QTabWidget* m_tabWidget;		//Tab widget for managing all tab pages
	TabPage* m_curPage;			//Currently active tab page
	//bool noFileIsOpened = true;

private:
	Ui::GeoDisplayClass ui;
	QVBoxLayout* m_mainLayout;  // Main layout
	QWidget* m_centralWidget;   // Central widget (must be layout container via QWidget)
	ConfigUI* m_config;

private: // Major modules
	void setupMenuBar();            //1. Set up menu bar
	void setupOptionPanel();        //2. Set up option panel
	void setupToolBar();			//3. Set up toolbar (commands)
	void setupTabWidget();			//4. Set up drawing window (tab pages)
	void config_init();				// Configuration initialization
	void setup_slots();             // Finally, connect signal slots

private: // Member variables
	//2. Side option panel
	QDockWidget* m_optionsDock;		// Dock window
	OptionsPanel* m_optionsPanel;	// Option panel

	//3. Toolbar (commands)

	//5. Status bar
	QLabel* statusLabel;


private: // Sub functions
	//1. Menu bar
	void setupMenuBar_slots();
	void action_open();				// Open
	void action_add();				// Add
	void action_output(int type);	// Output
	void action_run();				// Run algorithm

	//2. Signal forwarding for side option panel
	void setupOptionPanel_slots();
	//3. Toolbar (commands)
	void setupToolBar_slots();
	//4. Toolbar (drawing)
	void addTabWidget(const QString& qFullFileName);		// New tab page
	void onTabChanged(int index);	// Tab page switch
	void closeTab(int index);		// Close tab page
	void actionAddData(QString& qFullFileName, const int readType);


	//5. Status bar
	//void refreshStateLabel();

};
