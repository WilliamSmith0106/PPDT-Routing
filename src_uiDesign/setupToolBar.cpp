#include "GeoDisplay.h"
using namespace std;


//3. Set up toolbar (commands)
void GeoDisplay::setupToolBar() {
	ui.mainToolBar->setWindowTitle(tr("命令"));
	// Toolbar commands
	ui.mainToolBar->addAction(ui.actionOpen);
	ui.mainToolBar->addAction(ui.actionAdd);
	ui.mainToolBar->addAction(ui.actionClear);
	ui.mainToolBar->addAction(ui.actionRun);
	ui.mainToolBar->addAction(ui.actionOutput);
}

void GeoDisplay::setupToolBar_slots() {
	;
}
//void GeoDisplay::refreshStateLabel()
//{
//	// Show mouse position in status bar
//	QString str = "(" + QString::number(mouse_x) + "," + QString::number(mouse_y) + ")";
//	statusLabel->setText(state_info + str);
//}