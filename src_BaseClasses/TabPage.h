#pragma once
#include <QWidget>
#include "../src_config/configUI.h"
//#include "../src_algorithms/AlgorithmSet.h"
#include "../src_paint/Diagram.h"

class Data;
class DataParser;

class TabPage : public QWidget {
	Q_OBJECT
public:
	explicit TabPage(QString qFullName, QString qFilePath, QString qFileName, QWidget* parent = nullptr)
		: m_qFullName(qFullName), 
		m_qFilePath(qFilePath), 
		m_qFileName(qFileName), 
		QWidget(parent),
		m_diagram(nullptr){
	}
	virtual ~TabPage() = default;

	QString fullName() const { return m_qFullName; };
	QString filePath() const { return m_qFilePath; };
	QString fileName() const { return m_qFileName; };
	//AlgorithmSet* m_alg;

protected:
	QString m_qFullName = "data/example.map";	//Full file path including file name
	QString m_qFilePath = "data";				//File path without file name
	QString m_qFileName = "example.map";		//File name with extension
	QString m_qFullSaveName = "0001.map";
	bool m_dataAcquired = false;				//Records whether data was successfully acquired (true if successful)
	bool m_runFinished = false;
	Diagram* m_diagram;          //Drawing window

public:
	void pageInit(ConfigUI* conf) {
		fillData(conf);
		conf->configUpdate(m_qFilePath, m_qFileName);
		setupConnections();
	}
private:
	void setupConnections() const {
		connect(m_diagram, &Diagram::leftPressed,
			this, &TabPage::leftPressRun);
		connect(m_diagram, &Diagram::leftPressedMove,
			this, &TabPage::leftPressMoveRun);
		connect(m_diagram, &Diagram::leftReleaseed,
			this, &TabPage::leftReleaseRun);
	}
	
public: // Slot functions
	virtual void action_reset() = 0;				// Reset (only for map class txt files)
	virtual void action_random_pin() = 0;			// Regenerate random pins
	virtual void pin_decrease() = 0;				// Decrease pin count
	virtual void action_pin_increase() = 0;		// Increase pin count
	virtual void action_save_case() = 0;			// Save case
	virtual void action_save_image() = 0;			// Save image
	virtual void action_write_pre() = 0;			// Save preprocessing result
	virtual void action_write_res() = 0;			// Append result after preprocessing result

	virtual void pressRunButton() = 0;
	virtual void onConfigChanged(int type = 0) = 0;
	virtual void tranformChanged() = 0;
	virtual void leftPressRun(SelectedTarget* target) = 0;
	virtual void leftPressMoveRun(SelectedTarget* target) = 0;
	virtual void leftReleaseRun(SelectedTarget* target) = 0;
protected:
	virtual void fillData(ConfigUI* conf) = 0;
	virtual void refreshUI() = 0;
signals:
	void requestUpdatePanelUI(int size);
	
};

class TabPageFactory {
public:
	using CreatorFunc = std::function<TabPage* (const QString& qFullName, const QString& qFilePath, const QString& qFileName, QWidget* parent)>;
	static TabPageFactory& instance();
	// Register a TabPage constructor for a specific file extension
	void registerCreator(const QString& suffix, CreatorFunc creator);	
	TabPage* create(const QString& qFullName, QWidget* parent);

private:
	TabPageFactory() = default;
	QHash<QString, CreatorFunc> m_funcs;
};