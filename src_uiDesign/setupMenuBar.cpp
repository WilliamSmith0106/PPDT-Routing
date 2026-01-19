#include "GeoDisplay.h"
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include "../src_baseClasses/TabPage.h"

using namespace std;

//1. Setup menu bar
void GeoDisplay::setupMenuBar() {
	;
}
void GeoDisplay::setupMenuBar_slots() {
	//File
	connect(ui.actionOpen, &QAction::triggered, this, &GeoDisplay::action_open);		//Open
	connect(ui.actionAdd, &QAction::triggered, this, [this]() {action_add(); });		//Add
	connect(ui.actionSave, &QAction::triggered, this, [this]() {m_curPage->action_save_image(); });		//Save
	connect(ui.actionOutput, &QAction::triggered, this, [this]() {action_output(0); });	//Output
	connect(ui.actionRun, &QAction::triggered, this, [this]() {action_run(); });		//Run algorithm

	//Edit

	//Window (show/hide various windows)
	connect(ui.actionOptionPanel, &QAction::toggled, m_optionsDock, &QDockWidget::setVisible);			//Show/hide option panel
	connect(ui.actionTool_cmd, &QAction::toggled, ui.mainToolBar, &QToolBar::setVisible);				//Show/hide toolbar
	connect(ui.mainToolBar, &QToolBar::visibilityChanged, ui.actionTool_cmd, &QAction::setChecked);		//When toolbar visibility changes, update menu bar (two-way binding)
}

void GeoDisplay::action_open() {
	const QString fileFilter =
		"全部 (*);;"
		"DSN文件 (*.dsn *.DSN);;"
		"文本文件 (*.txt);;"
		"图片 (*.png *.jpg)";
	QString qFullFileName = QFileDialog::getOpenFileName(this, "打开文件", m_config->m_qFilePath, fileFilter);
	if (qFullFileName.isEmpty()) {
		return;
	}
	addTabWidget(qFullFileName);
}
void GeoDisplay::action_add() {
	if (m_tabWidget->count() == 0) {
		action_open();
	}
	else {
		/*
		//1. Read file, get file name, update configuration information
		QString qFullFileName = QFileDialog::getOpenFileName(this, "打开文件", m_config->m_qFilePath, "*");
		if (qFullFileName.isEmpty()) { qDebug() << "Failed to read data!"; return; }
		QFileInfo fileInfo(qFullFileName);
		m_config->m_qFilePath = fileInfo.absolutePath();
		m_config->m_qFileName = fileInfo.fileName();
		//2. Determine file parsing scheme based on file name
		int read_file_type = -1;
		QString qfileName_lower = m_config->m_qFileName.toLower();
		if (qfileName_lower.endsWith(".png") || qfileName_lower.endsWith(".jpg")) {
			read_file_type = 0;		// Parse image format map file
		}
		else if (qfileName_lower.endsWith(".txt")) {
			if (qfileName_lower.startsWith("post")) {
				read_file_type = 1;	// Parse post-processing file
			}
			else if (qfileName_lower.startsWith("map")) {
				read_file_type = 2;	// Parse text format map file
			}
		}
		//3. Unparseable type
		if (read_file_type == 1 && m_curTap->m_data_type == 1) {	// Both are post-processing files
			actionAddData(qFullFileName, read_file_type);
		}
		else {
			QMessageBox::warning(this, "错误!", "添加的数据类型不匹配");
			return;
		}
		*/
	}
}
void GeoDisplay::action_output(int type) {
	//type values: 0, output post-processed data
	switch (type) {
	case 0:
		m_curPage->action_write_res();
		break;
	case 1:
		break;
	default:
		break;
	}
}
void GeoDisplay::action_run() {
	if (m_curPage)
		m_curPage->pressRunButton();
}
