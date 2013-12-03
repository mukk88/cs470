#ifndef KALMANAGENT_H
#define KALMANAGENT_H
#include "command.h"

using namespace std;

class KalmanAgent {

public:

	KalmanAgent(int i, BZRC* bzrc) : index(i), commandCenter(bzrc) {

	}
private:
	int index;
	BZRC* commandCenter;
};

#endif