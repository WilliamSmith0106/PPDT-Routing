#include "OptionsPanel.h"
#include <QLabel>
#include <QGroupBox>
#include <QSlider>

#include <QFileDialog>   // File saving
#include <QMessageBox>


void OptionsPanel::updatePanelUI(int size = 0) {
	ui.treeIndex->blockSignals(true);		// Block signals
	// Post-processing mode
	ui.treeIndex->clear();
	ui.treeIndex->addItem("All", -1);
	for (int i = 0; i < size; i++)
		ui.treeIndex->addItem(QString::number(i), i);
	ui.treeIndex->blockSignals(false);		// Restore signals
	int targetIndex = ui.treeIndex->findData(m_config->m_showTreeIndex);
	ui.treeIndex->setCurrentIndex(targetIndex);
}

OptionsPanel::OptionsPanel(ConfigUI* conf, QWidget* parent) : QWidget(parent) {
	ui.setupUi(this);
	m_config = conf;
	styleInit();
	setPanelUI();
	setup_panelSlots();
}

OptionsPanel::~OptionsPanel() {}

void OptionsPanel::styleInit() {

}

void OptionsPanel::setPanelUI() {
	// Post-processing mode
	ui.PostMode->addItem("None", 0);
	ui.PostMode->addItem("Off", 1);
	ui.PostMode->addItem("Minimal", 2);
	ui.PostMode->addItem("Smooth", 3);
	ui.PostMode->addItem("Full", 4);

	// Grid type: 0 no grid, 1 line grid, 2 point grid
	ui.showGridType->addItem("Hide", 0);
	ui.showGridType->addItem("Line", 1);
	ui.showGridType->addItem("Point", 2);
}

void OptionsPanel::setup_panelSlots() {
	//1. Button signals
	connect(ui.openResBt, &QPushButton::clicked, this, [this]() {			// Open Ren file for viewing
		emit open_res_request_signal();
		});
	connect(ui.resetBt, &QPushButton::clicked, this, [this]() {				// Reset
		emit reset_request_signal();
		});
	connect(ui.randPinBt, &QPushButton::clicked, this, [this]() {			// Random pin
		emit random_pin_request_signal();
		});
	connect(ui.pinDecreaseBt, &QPushButton::clicked, this, [this]() {		// Decrease pin
		emit pin_decrease_request_signal();
		});
	connect(ui.pinIncreaseBt, &QPushButton::clicked, this, [this]() {		// Increase pin
		emit pin_increase_request_signal();
		});
	connect(ui.saveCaseBt, &QPushButton::clicked, this, [this]() {			// Save case
		emit save_case_request_signal();
		});
	connect(ui.saveImageBt, &QPushButton::clicked, this, [this]() {			// Save image
		emit save_image_request_signal();
		});
	connect(ui.writePreBt, &QPushButton::clicked, this, [this]() {			// Save preprocessing results
		emit write_pre_request_signal();
		});
	connect(ui.addResBt, &QPushButton::clicked, this, [this]() {			// Append save results
		emit write_res_request_signal();
		});

	//2. View options
	ui.checkFlipTD->setChecked(m_config->m_flip_up_down);
	connect(ui.checkFlipTD, &QCheckBox::checkStateChanged, this, [this]() {			//01 Flip top-bottom
		m_config->m_flip_up_down = ui.checkFlipTD->checkState() == Qt::Checked;
		paintChanged(1);
		});
	ui.checkFlipLR->setChecked(m_config->m_flip_left_right);
	connect(ui.checkFlipLR, &QCheckBox::checkStateChanged, this, [this]() {			//02 Flip left-right
		m_config->m_flip_left_right = ui.checkFlipLR->checkState() == Qt::Checked;
		paintChanged(1);
		});
	ui.checkRotate90->setChecked(m_config->m_rotate_90);
	connect(ui.checkRotate90, &QCheckBox::checkStateChanged, this, [this]() {		//03 Rotate 90 degrees
		m_config->m_rotate_90 = ui.checkRotate90->checkState() == Qt::Checked;
		paintChanged(1);
		});

	//3. Algorithm options
	ui.checkPreViaAlct->setChecked(m_config->m_preViaAlctOn);
	connect(ui.checkPreViaAlct, &QCheckBox::checkStateChanged, this, [this]() {		//11 Enable via pre-allocation
		m_config->m_preViaAlctOn = ui.checkPreViaAlct->checkState() == Qt::Checked;
		paintChanged(2);
		});
	ui.checkGNDRoute->setChecked(m_config->m_GNDRouteOn);
	connect(ui.checkGNDRoute, &QCheckBox::checkStateChanged, this, [this]() {      //12 GND special routing
		m_config->m_GNDRouteOn = ui.checkGNDRoute->checkState() == Qt::Checked;
		});
	ui.checkVCCRoute->setChecked(m_config->m_VCCRouteOn);
	connect(ui.checkVCCRoute, &QCheckBox::checkStateChanged, this, [this]() {      //13 VCC special routing
		m_config->m_VCCRouteOn = ui.checkVCCRoute->checkState() == Qt::Checked;
		});
	ui.checkDiffRoute->setChecked(m_config->m_diffRouteOn);
	connect(ui.checkDiffRoute, &QCheckBox::checkStateChanged, this, [this]() {     //14 Differential routing
		m_config->m_diffRouteOn = ui.checkDiffRoute->checkState() == Qt::Checked;
		});
	ui.checkOnGrid->setChecked(m_config->m_onGrids);
	connect(ui.checkOnGrid, &QCheckBox::checkStateChanged, this, [this]() {			//15 On grid
		m_config->m_onGrids = ui.checkOnGrid->checkState() == Qt::Checked;
		});

	//4. Display options
	ui.showObs1->setChecked(m_config->m_showObs1);
	connect(ui.showObs1, &QCheckBox::checkStateChanged, this, [this]() {		//21 Show layer 1 obstacles
		m_config->m_showObs1 = ui.showObs1->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.showObs2->setChecked(m_config->m_showObs2);
	connect(ui.showObs2, &QCheckBox::checkStateChanged, this, [this]() {		//22 Show layer 2 obstacles
		m_config->m_showObs2 = ui.showObs2->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.showFlyLines->setChecked(m_config->m_showFlyLines);
	connect(ui.showFlyLines, &QCheckBox::toggled, this, [this](bool checked) {	//23 Show flylines
		m_config->m_showFlyLines = checked;
		paintChanged(0);
		});
	ui.showPahts->setChecked(m_config->m_showPahts);
	connect(ui.showPahts, &QCheckBox::checkStateChanged, this, [this]() {		//24 Show routing results
		m_config->m_showPahts = ui.showPahts->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.showPlanningPts->setChecked(m_config->m_showPPs);
	connect(ui.showPlanningPts, &QCheckBox::checkStateChanged, this, [this]() {	//25 Show planning points
		m_config->m_showPPs = ui.showPlanningPts->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.showVias->setChecked(m_config->m_show_vias);
	connect(ui.showVias, &QCheckBox::checkStateChanged, this, [this]() {		//26 Show Vias
		m_config->m_show_vias = ui.showVias->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.showPinCenters->setChecked(m_config->m_showPinCenters);
	connect(ui.showPinCenters, &QCheckBox::checkStateChanged, this, [this]() {    //27 Show Pin centers
		m_config->m_showPinCenters = ui.showPinCenters->checkState() == Qt::Checked;
		paintChanged(0);
		});


	//5. Other optional properties
	ui.checkPostProcess->setChecked(m_config->m_postOn);
	connect(ui.checkPostProcess, &QCheckBox::checkStateChanged, this, [this]() {		//51 Enable post-processing
		m_config->m_postOn = ui.checkPostProcess->checkState() == Qt::Checked;
		paintChanged(0);
		});
	ui.PostMode->setCurrentIndex(m_config->m_postMode);									// Post-processing mode selection
	connect(ui.PostMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
		m_config->m_postMode = index;
		paintChanged(0);
		});
	ui.showGridType->setCurrentIndex(m_config->m_gridType);								// Grid display type selection
	connect(ui.showGridType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
		m_config->m_gridType = index;
		paintChanged(0);
		});
	ui.gridSizeEdit->setText(QString::number(m_config->m_gridSize, 'f', 3));		// Grid size
	connect(ui.gridSizeEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
		bool ok;
		double value = text.toDouble(&ok);
		if (ok) {
			m_config->m_gridSize = value;
			paintChanged(0);
		}
		});
	ui.checkShowTrees->setChecked(m_config->m_showTrees);							//52 Show search trees
	connect(ui.checkShowTrees, &QCheckBox::checkStateChanged, this, [this]() {
		m_config->m_showTrees = ui.checkShowTrees->checkState() == Qt::Checked;
		paintChanged(0);
		});
	connect(ui.treeIndex, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {	//Search tree index selection
		//Displayed tree index selection (initial value set in updatePanelUI)
		m_config->m_showTreeIndex = ui.treeIndex->itemData(index).toInt();
		paintChanged(0);
		});
	ui.flexibleOpt->setText(m_config->m_flexibleOpt);			// Input flexible parameter data
	connect(ui.flexibleOpt, &QLineEdit::textChanged, this, [this]() {	//editingFinished
		m_config->m_flexibleOpt = ui.flexibleOpt->text();
		});
	ui.checkDebugBreak->setChecked(m_config->m_debugFuncOn);
	connect(ui.checkDebugBreak, &QCheckBox::checkStateChanged, this, [this]() {		//53 Enable debug break function
		m_config->m_debugFuncOn = ui.checkDebugBreak->checkState() == Qt::Checked;
		});
	ui.doubleInput->setText(QString::number(m_config->m_doubleNum1, 'f', 2) + " "
		+ QString::number(m_config->m_doubleNum2, 'f', 2) + " "
		+ QString::number(m_config->m_intNum3));
	connect(ui.doubleInput, &QLineEdit::editingFinished, this, [this]() {
		// Input 3 numbers representing coordinates and other data
		QString text = ui.doubleInput->text().trimmed();
		QString normalizedText = text;

		// Unify separators
		normalizedText.replace(QRegularExpression("\\s+"), " ");
		normalizedText.replace(QChar(0xFF0C), QChar(' ')); // Chinese comma
		normalizedText.replace(',', ' ');
		normalizedText.replace('/', ' ');

		QString part1 = normalizedText.section(' ', 0, 0);
		QString part2 = normalizedText.section(' ', 1, 1);
		QString part3 = normalizedText.section(' ', 2, 2);

		bool ok1 = false, ok2 = false, ok3 = false;
		double v1 = part1.toDouble(&ok1);
		double v2 = part2.toDouble(&ok2);
		int    v3 = part3.toInt(&ok3);

		m_config->m_doubleNum1 = ok1 ? v1 : 0.0;
		m_config->m_doubleNum2 = ok2 ? v2 : 0.0;
		m_config->m_intNum3 = ok3 ? v3 : 0;

		QString newText =
			QString::number(m_config->m_doubleNum1, 'f', 2) + " "
			+ QString::number(m_config->m_doubleNum2, 'f', 2) + " "
			+ QString::number(m_config->m_intNum3);

		ui.doubleInput->setText(newText);
		});

}
