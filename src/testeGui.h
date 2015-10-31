/*
 * testeGui.h
 *
 *  Created on: 30 de out de 2015
 *      Author: wagner
 */

#ifndef SRC_TESTEGUI_H_
#define SRC_TESTEGUI_H_

#include "ofMain.h"
//#include "ofxGui.h"

class testeGui {
	public:

	void setup();

	ofParameterGroup guiParameters;
	ofParameterGroup outrosParameters;

	ofParameter<int> nearTresh;
	ofParameter<int> farTresh;
	ofParameter<int> hullPress;

};

#endif /* SRC_TESTEGUI_H_ */
