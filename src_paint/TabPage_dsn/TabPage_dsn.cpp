#include "TabPage_dsn.h"
#include <QVBoxLayout>
#include <QMessageBox>

using namespace std;

void TabPage_dsn::fillData(ConfigUI* conf) {
	m_config = conf;
	// 2.Write file data to data object
	m_dataAcquired = m_parser->readFile(m_qFullName);
	if (!m_dataAcquired) return;
	m_runFinished = false;
	// 3.Create drawing window Diagram,
	shared_ptr<DiagramData> diagramData = std::static_pointer_cast<DiagramData>(
		std::shared_ptr<Data_dsn>(m_data, [](auto*) {})); // Shared pointer but don't delete
	m_diagram = new Diagram(diagramData, m_config, this);
	m_algm = new AlgorithmLink_dsn(m_data, m_config);
	// 4.Calculate and draw flylines
	if (m_data == nullptr) {
		QMessageBox::warning(this, "提示", "文件读取失败！");
	}
	m_algm->routingRunBegin();
	m_diagram->setData2();
	onConfigChanged();		// Update display/hide and other option information

	// 5.Set layout
	QVBoxLayout* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(m_diagram);
	setLayout(layout);
}
void TabPage_dsn::refreshUI() {
	m_diagram->refresh();
}

// Slot function
void TabPage_dsn::pressRunButton() {
	m_algm->routingRun(m_qFileName);
	m_diagram->setData2();
	m_diagram->setData3();
	emit requestUpdatePanelUI((int)m_data->m_treesLines.size());
	refreshUI();
}
void TabPage_dsn::onConfigChanged(int type) {
	if (type == 0) {
		const Style_dsn& sInfo = m_data->m_styleInfo;
		//1.Update flyline visibility
		m_data->m_styles.getLineStyle(sInfo.flyLineKey)->setVisible(m_config->m_showFlyLines);
		//2.Update path line visibility
		for (const QString& key : sInfo.pathLineKeys) {
			m_data->m_styles.getLineStyle(key)->setVisible(m_config->m_showPahts);
		}
		//3.Update via visibility, consistent with paths
		bool show_vias = m_config->m_showPahts && m_config->m_show_vias;
		m_data->m_styles.getPolygonStyle(sInfo.viaCicleKey)->setVisible(show_vias);

		//4.Update pad visibility
		const QString& obs1key = sInfo.getPadPolyKey(0);
		m_data->m_styles.getPolygonStyle(obs1key)->setVisible(m_config->m_showObs1);
		const QString& obs2key = sInfo.getPadPolyKey(1);
		m_data->m_styles.getPolygonStyle(obs2key)->setVisible(m_config->m_showObs2);
		//5.Update planning point visibility
		m_data->m_styles.getPointStyle(sInfo.planningPtKey)->setVisible(m_config->m_showPPs);
		//6.Update tree line visibility
		m_diagram->setShowTreeIndex(m_config->m_showTreeIndex);
		refreshUI();
	}
	else if (type == 2) {
		if (m_algm) {
			m_algm->setNetMST();
			refreshUI();
		}
	}
}
void TabPage_dsn::tranformChanged() {
	m_diagram->initViewTransform();
	refreshUI();
}
void TabPage_dsn::leftPressRun(SelectedTarget* target) {
	if (target->lineSelected) {
		m_algm->pushLineRunBegin(target->sLine, *target->linesToRun);
	}
	else if (target->pinSelected) {
		m_algm->pushPinRunBegin(target->nearistPt, *target->linesToRun);
	}
}
void TabPage_dsn::leftPressMoveRun(SelectedTarget* target) {
	if (target->lineSelected) {
		m_algm->pushLineRun(target->offset, false, false);
	}
	else if (target->pinSelected) {
		m_algm->pushPinRun(target->offset, false);
	}
}
void TabPage_dsn::leftReleaseRun(SelectedTarget* target) {
	if (target->lineSelected) {
		m_algm->pushLineRun(target->offset, m_config->m_onGrids, true);
	}
	else if (target->pinSelected) {
		m_algm->pushPinRun(target->offset, m_config->m_onGrids);
	}
	m_diagram->setData1();
	m_diagram->setData2();
	refreshUI();
}