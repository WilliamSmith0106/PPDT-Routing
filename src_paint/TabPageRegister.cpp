#include "../src_baseClasses/TabPage.h"

//1. Include required header files
#include "TabPage_dsn/TabPage_dsn.h"		//(2) dsn class PCB file

namespace {
	// 2. Define constructor
	static auto fun_dsn = [](	//(3) dsn class PCB file
		const QString& qFullName, const QString& qFilePath, const QString& qFileName, QWidget* parent) {
			return new TabPage_dsn(qFullName, qFilePath, qFileName, parent);
		};

	struct TabPageRegister {
		TabPageRegister() {
			TabPageFactory& factory = TabPageFactory::instance();
			std::vector<QString> suffixes;
			suffixes = { "dsn" };									//(3) dsn class PCB file
			for (const auto& suffix : suffixes) {
				factory.registerCreator(suffix, fun_dsn);
			}
		}
	};
	TabPageRegister registrar_grid;
}
