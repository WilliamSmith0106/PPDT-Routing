#include "src_uiDesign/GeoDisplay.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    GeoDisplay window;
    window.setWindowTitle("PPDT and pushLine");
    window.resize(1000, 600);
    window.show();
    return app.exec();
}
