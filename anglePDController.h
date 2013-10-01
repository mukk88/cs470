#pragma once
#include "pdController.h"

using namespace std;

class AnglePDController : public PDController{
public:
	AnglePDController() : PDController() {
		kp = .01;
		kd = .5;
	}
};