#ifndef KALMANAGENT_H
#define KALMANAGENT_H
#include <vector>
#include <iostream>
#include <iterator>
#include "occgrid.h"
#include "agent.h"
#include "command.h"
#include "grid.h"

using namespace std;

class KalmanAgent : public Agent {

public:

	KalmanAgent(int index, BZRC* bzrc, PotentialField* p, string color): Agent(index, bzrc, p, color) {
	}
};

#endif