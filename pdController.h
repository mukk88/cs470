#pragma once

using namespace std;

class PDController {

public:
	PDController(){
		prevError = 1;
	}

	double get_value(double currentValue, double desiredValue){
		double de = (currentValue - desiredValue) / prevError;
		prevError = currentValue - desiredValue;
		return kp * (currentValue - desiredValue) + kd * de;
	}

protected:
	double kp;
	double kd;
	double prevError;
};