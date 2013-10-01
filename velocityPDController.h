#pragma once
#include "pdController.h"

using namespace std;

class VelocityPDController : public PDController{
public:
	VelocityPDController() : PDController() {
		kp = .01;
		kd = .5;
	}
};