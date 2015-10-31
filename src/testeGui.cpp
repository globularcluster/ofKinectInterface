/*
 * testeGui.cpp
 *
 *  Created on: 30 de out de 2015
 *      Author: wagner
 */

#include "testeGui.h"

void testeGui::setup(){
	guiParameters.setName("gui controls");
	outrosParameters.setName("outros controles");

	guiParameters.add(nearTresh.set("nearTresh", 255, 0, 255));
	guiParameters.add(farTresh.set("farTresh", 233, 0, 255));
	outrosParameters.add(hullPress.set("hullPress", 20, 0, 100));

}

