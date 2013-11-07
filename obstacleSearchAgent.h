#ifndef OBSTACLESEARCH_H
#define OBSTACLESEARCH_H
#include <vector>
#include <iterator>
#include "occgrid.h"
#include "agent.h"
#include "command.h"

using namespace std;

class ObstacleSearchAgent : Agent {

public:

	ObstacleSearchAgent(int gd, double truePositive, double trueNegative, moveR, moveU) : moveRight(moveR)
		gridDimension(gd), truePos(truePositive), trueNeg(trueNegative), moveVertical(true), moveUp(moveU) {

		observedObstacles = new double*[gd];
		for(int i = 0; i < gd; ++i) {
		    observedObstacles[i] = new double[gd];
		}
	}

	~ObstacleSearchAgent() {
		for(int i = 0; i < gridDimension; ++i) {
		    delete [] observedObstacles[i];
		}
		delete [] observedObstacles;
	}

private:
	int gridDimension;
	double** observedObstacles;
	vector<int> bots;
	double truePos;
	double trueNeg;
	bool moveVertical;
	bool moveUp;
	const bool moveRight;
	int prevPos[2];
	vector<int[2]> replusionPoints;

	void move(int b){
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
		pfMove(b, false);

		int[] pos = get_tank(index).pos;
		if (distancePoints(pos, prevPos) < 3){
			pfield->addPoint(pos);
		}
		prevPos = pos;
	}

	void observe(int b){
		OccGrid grid;
		commandCenter->get_occ(b, grid);
		for (int i = 0; i < grid.getHeight(); ++i){
			int y = grid.getYStart() + i;
			for (int j = 0; j < grid.getWidth(); ++j){
				int x = grid.getXStart() + j;

				// p(si,j = occupied | oi,j) = p(oi,j | si,j = occupied)p(si,j = occupied) / p(oi,j)
				double occupied_belief = 0;
				double unoccupied_belief = 0;
				if (grid.occupied(y, x)){
					occupied_belief = truePos * observedObstacles[y][x];
					unoccupied_belief = trueNeg * (1-observedObstacles[y][x]);
				}
				else {
					occupied_belief = (1-truePos) * observedObstacles[y][x];
					unoccupied_belief = (1-trueNeg) * (1-observedObstacles[y][x]);
				}

				observedObstacles[y][x] = occupied_belief / (occupied_belief + unoccupied_belief);
			}
		}
	}
};

#endif