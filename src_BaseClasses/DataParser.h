#pragma once
#include "Data.h"
#include <QString>
#include <QDir>
#include <QFileInfo>

class DataParser{
public:
	DataParser(){};
	~DataParser() {};

	virtual bool readFile(const QString& qFullName) = 0;
	virtual bool saveFile(const QString& qFullName) = 0;
	virtual bool saveFileAs(const QString& qFullName) = 0;
private:
	bool x_horizontal = false;	//0 means xy not swapped, 1 means xy swapped

protected:
	virtual void setMinMax() = 0;

};

