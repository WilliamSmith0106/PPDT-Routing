#include "configUI.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QString>
#include <QFileDialog>
#include <QInputDialog>

void ConfigUI::configUpdate(const QString& qFilePath, const QString& qFileName) {
	m_qFilePath = qFilePath;
	m_qFileName = qFileName;
}

bool ConfigUI::read_config() {
	QFile file("src_config/config.config");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QMessageBox::warning(nullptr, tr("警告："), tr("读取配置文件失败！"));
		return false;
	}
	QTextStream in(&file);
	in.setAutoDetectUnicode(true); // Auto-detect UTF-BOM
	while (!in.atEnd()) {
		QString line = in.readLine().trimmed();
		if (line.isEmpty()) continue;	// Skip empty lines

		if (line.startsWith("defaltPath"))
			m_qFilePath = line.section(' ', 1);
		else if (line.startsWith("defaltName"))
			m_qFileName = line.section(' ', 1);
		else if (line.startsWith("viewOpt")) {
			m_flip_left_right = line.section(' ', 1, 1).toInt();;	//01. Flip X-axis (left-right)
			m_flip_up_down = line.section(' ', 2, 2).toInt();;		//02. Flip Y-axis (up-down)
			m_rotate_90 = line.section(' ', 3, 3).toInt();;			//03. Rotate 90 degrees
		}
		else if (line.startsWith("algmOpt")) { 
			m_preViaAlctOn = line.section(' ', 1, 1).toInt();;		//11. Pre via allocation
			m_GNDRouteOn = line.section(' ', 2, 2).toInt();;		//12. GND special routing
			m_VCCRouteOn = line.section(' ', 3, 3).toInt();;		//13. VCC special routing
			m_diffRouteOn = line.section(' ', 4, 4).toInt();;		//14. Differential routing
			m_onGrids = line.section(' ', 5, 5).toInt();;			//15. On grids
		}
		else if (line.startsWith("dispOpt")) {
			m_showObs1 = line.section(' ', 1, 1).toInt();;			//21. Show layer 1 obstacles
			m_showObs2 = line.section(' ', 2, 2).toInt();;			//22. Show layer 2 obstacles
			m_showFlyLines = line.section(' ', 3, 3).toInt();;		//23. Show flying lines
			m_showPahts = line.section(' ', 4, 4).toInt();;			//24. Show routing results
			m_showPPs = line.section(' ', 5, 5).toInt();;			//55. Show planning points
			m_show_vias = line.section(' ', 6, 6).toInt();;			//26. Show vias
			m_showPinCenters = line.section(' ', 7, 7).toInt();;	//27. Show Pin centers
		}
		else if (line.startsWith("boolOpt")) {
			m_postOn = line.section(' ', 1, 1).toInt();;			//51. Enable post-processing
			m_showTrees = line.section(' ', 2, 2).toInt();;			//52. Show trees
			m_debugFuncOn = line.section(' ', 3, 3).toInt();;		//53. Enable debug interrupt
		}
		else if (line.startsWith("m_gridType"))
			m_gridType = line.section(' ', 1).toInt();
		else if (line.startsWith("m_postMode"))
			m_postMode = line.section(' ', 1).toInt();
		else if (line.startsWith("m_minimalScreenGridSize"))
			m_minimalScreenGridSize = line.section(' ', 1).toFloat();
		else if (line.startsWith("m_gridSize"))
			m_gridSize = line.section(' ', 1).toFloat();
		else if(line.startsWith("m_showTreeIndex "))
			m_showTreeIndex = line.section(' ', 1).toInt();
		else if (line.startsWith("m_flexibleOpt"))
			m_flexibleOpt = line.section(' ', 1);
		else if (line.startsWith("m_doubleNum")) {
			m_doubleNum1 = line.section(' ',1, 1).toDouble();
			m_doubleNum2 = line.section(' ',2, 2).toDouble();
			m_intNum3 = line.section(' ', 3, 3).toInt();
		}
		else if (line.startsWith("#")) {
			QMessageBox::warning(nullptr, tr("警告："), tr("配置文件中存在未知的行！"));
			continue;
		}
	}
	file.close();
	return true;
}
bool ConfigUI::write_config()const {
	// Create or overwrite configuration file
	QFile outputFile("src_config/config.config");
	if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QMessageBox::warning(nullptr, "错误", "无法创建配置文件");
		return false;
	}

	QTextStream out(&outputFile);

	// Directly write all configuration items to ensure each item exists
	out << "defaltPath " << m_qFilePath << Qt::endl;
	out << "defaltName " << m_qFileName << Qt::endl;
	out << "viewOpt "
		<< m_flip_left_right << " " //01. Flip X-axis (left-right)
		<< m_flip_up_down << " "    //02. Flip Y-axis (up-down)
		<< m_rotate_90 << " "       //03. Rotate 90 degrees
		<< Qt::endl;
	out << "algmOpt "
		<< m_preViaAlctOn << " "    //11. Pre via allocation
		<< m_GNDRouteOn << " "      //12. GND special routing
		<< m_VCCRouteOn << " "      //13. VCC special routing
		<< m_diffRouteOn << " "     //14. Differential routing
		<< m_onGrids << " "         //15. On grids
		<< Qt::endl;
	out << "dispOpt "
		<< m_showObs1 << " "        //21. Show layer 1 obstacles
		<< m_showObs2 << " "        //22. Show layer 2 obstacles
		<< m_showFlyLines << " "    //23. Show flying lines
		<< m_showPahts << " "       //24. Show routing results
		<< m_showPPs << " "         //25. Show planning points
		<< m_show_vias << " "       //26. Show vias
		<< m_showPinCenters << " "  //27. Show Pin centers
		<< Qt::endl;
	out << "boolOpt "
		<< m_postOn << " "          //51. Enable post-processing
		<< m_showTrees << " "       //52. Show trees
		<< m_debugFuncOn << " "     //53. Enable debug interrupt
		<< Qt::endl;
	out << "m_gridType " << m_gridType << Qt::endl;
	out << "m_postMode " << m_postMode << Qt::endl;
	out << "m_minimalScreenGridSize " << m_minimalScreenGridSize << Qt::endl;
	out << "m_gridSize " << m_gridSize << Qt::endl;
    out << "m_showTreeIndex " << m_showTreeIndex << Qt::endl;
	out << "m_flexibleOpt " << m_flexibleOpt << Qt::endl;
	out << "m_doubleNum " << m_doubleNum1 << " " << m_doubleNum2 << " " << m_intNum3 << Qt::endl;

	outputFile.close();
	return true;
}