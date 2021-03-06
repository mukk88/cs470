#ifndef OBSTACLESEARCH_H
#define OBSTACLESEARCH_H
#include <vector>
#include <iostream>
#include <iterator>
#include "occgrid.h"
#include "agent.h"
#include "command.h"
#include "grid.h"

using namespace std;

class ObstacleSearchAgent : public Agent {

public:

	ObstacleSearchAgent(int index, BZRC* bzrc, PotentialField* p, string color, 
		bool moveR, bool moveU, Grid* g, bool moveVertFirst): Agent(index, bzrc, p, color) {

		moveVertical = moveVertFirst;
		moveHorizDistance = 25;
		moveVertDistance = 750;
		moveUp = moveU;
		moveRight = moveR;
		mainGrid = g;
		truePos = g->getTruePos();
		trueNeg = g->getTrueNeg();
	}

	~ObstacleSearchAgent() {
	}

	void makeHorizontalMower(){
		int temp = moveHorizDistance;
		moveHorizDistance = moveVertDistance;
		moveVertDistance = temp;
	}

	virtual bool move(){
		if (get_tank().status != "alive")
			return false;

		if (missionAccomplished){
			missionAccomplished = false;
			if (moveVertical){
				moveVertical = false;
				goal[1] += (moveVertDistance * ((moveUp) ? 1 : -1));
				moveUp = !moveUp;
			}
			else { // move horizontal
				goal[0] += (moveHorizDistance * ((moveRight) ? 1 : -1));
				moveVertical = true;
			}
		}
		Agent::move(true);

		observe();


	}

	void observe(){
		OccGrid* grid = new OccGrid();
		bool success = commandCenter->get_occ(index, grid);
		for (int i = 0; i < grid->getHeight(); ++i){
			int y = grid->getYStart() + i;
			for (int j = 0; j < grid->getWidth(); ++j){
				int x = grid->getXStart() + j;

				// p(si,j = occupied | oi,j) = p(oi,j | si,j = occupied)p(si,j = occupied) / p(oi,j)

				double occupied_belief = 0;
				double unoccupied_belief = 0;
				if (grid->occupied(i, j)){
					occupied_belief = truePos * mainGrid->getValue(x,y);
					unoccupied_belief = (1-trueNeg) * (1-mainGrid->getValue(x,y));
				}
				else {
					occupied_belief = (1-truePos) * mainGrid->getValue(x,y);
					unoccupied_belief = trueNeg * (1-mainGrid->getValue(x,y));
				}

				mainGrid->setValue(x,y, occupied_belief / (occupied_belief + unoccupied_belief));
			}
		}
		delete grid;
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
	int moveHorizDistance;
	int moveVertDistance;
};

#endif
