#ifndef OBSTACLESEARCH_H
#define OBSTACLESEARCH_H
#include <vector>
#include <iterator>
#include "occgrid.h"
#include "agent.h"
#include "command.h"
#include "grid.h"

using namespace std;

class ObstacleSearchAgent : public Agent {

public:

	ObstacleSearchAgent(int index, BZRC* bzrc, PotentialField* p, string color, 
		bool moveR, bool moveU, Grid* g): Agent(index, bzrc, p, color) {

		moveVertical = true;
		moveUp = moveU;
		moveRight = moveR;
		mainGrid = g;

	}

	~ObstacleSearchAgent() {
	}

	void setTruePositive(double truePositive){
		truePos = truePositive;
	}

	void setTrueNegative(double trueNegative){
		trueNeg = trueNegative;
	}

	bool move(){
		if (missionAccomplished){
			missionAccomplished = false;
			if (moveVertical){
				moveVertical = false;
				goal[1] += (moveUp) ? 775 : -775;
				moveUp = !moveUp;
			}
			else { // move horizontal
				goal[0] += (moveRight) ? 25 : -25;
			}
		}
		Agent::move(true);

		double pos[2];
		pos[0] = get_tank().pos[0];
		pos[1] = get_tank().pos[1];
		/*if (distancePoints(pos, prevPos) < 3){
			pfield->addPoint(pos);
		}*/
		prevPos[0] = pos[0];
		prevPos[1] = pos[1];
	}

	void observe(){
		OccGrid grid;
		commandCenter->get_occ(index, grid);
		for (int i = 0; i < grid.getHeight(); ++i){
			int y = grid.getYStart() + i;
			for (int j = 0; j < grid.getWidth(); ++j){
				int x = grid.getXStart() + j;

				// p(si,j = occupied | oi,j) = p(oi,j | si,j = occupied)p(si,j = occupied) / p(oi,j)
				double occupied_belief = 0;
				double unoccupied_belief = 0;
				if (grid.occupied(y, x)){
					occupied_belief = truePos * mainGrid->getValue(x,y);
					unoccupied_belief = trueNeg * (1-mainGrid->getValue(x,y));
				}
				else {
					occupied_belief = (1-truePos) * mainGrid->getValue(x,y);
					unoccupied_belief = (1-trueNeg) * (1-mainGrid->getValue(x,y));
				}

				mainGrid->setValue(x,y, occupied_belief / (occupied_belief + unoccupied_belief));
			}
		}
	}

private:
	int gridDimension;
	Grid* mainGrid;
	vector<int> bots;
	bool moveVertical;
	bool moveUp;
	bool moveRight;
	double prevPos[2];
	double truePos;
	double trueNeg;

};

#endif