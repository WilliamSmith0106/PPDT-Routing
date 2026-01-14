#pragma once
#include "dataStructUI.h"
#include "../src_config/DiagramStyle.h"

//Stores data used by frontend and backend
class Data {
public:
	Data() {}
	~Data() {}
public:
	//1. Boundary data
	QPointF margin = QPointF(20, 20);	//Margin from coordinate axes
	QPointF minPt = QPointF(-2, -2);	//Minimum x,y coordinates in the figure
	QPointF maxPt = QPointF(-1, -1);    //Maximum x,y coordinates in the figure
//protected:
	DiagramStyleManager m_styles;

public:
	void clear() {
		margin = QPointF(20, 20);		//Margin from coordinate axes
		minPt = QPointF(-2, -2);	    //Minimum x,y coordinates in the figure
		maxPt = QPointF(-1, -1);        //Maximum x,y coordinates in the figure
	};
};
