#ifndef OBSTACLESEARCH_H
#define OBSTACLESEARCH_H
#include <vector>
#include <iterator>
#include "occgrid.h"

using namespace std;

class ObstacleSearch {

public:

	ObstacleSearch(vector<int> b, int gd, double truePositive, double trueNegative) : 
		bots(b), gridDimension(gd), truePos(truePositive), trueNeg(trueNegative) {

		observedObstacles = new double*[gd];
		for(int i = 0; i < gd; ++i) {
		    observedObstacles[i] = new double[gd];
		}
	}

	~ObstacleSearch() {
		for(int i = 0; i < gridDimension; ++i) {
		    delete [] observedObstacles[i];
		}
		delete [] observedObstacles;
	}

	void run(){
		for (int i = 0; i < bots.size()/2; ++i){
			//bots[i].setGoal();						//TODO
		}

		for (int i = bots.size()/2; i < bots.size(); ++i){
			//bots[i].setGoal();						//TODO
		}

		while(true){
			for (int b = 0; b < bots.size(); ++b){
				move(bots[b]);
				observe(bots[b]);
			}
		}
	}

private:
	int gridDimension;
	double** observedObstacles;
	vector<int> bots;
	double truePos;
	double trueNeg;

	void move(int b){
	}

	void addReplusion(){
	}

	void observe(int b){
		OccGrid grid;
		for (int i = 0; i < grid.getHeight(); ++i){
			int y = grid.getYStart() + i;
			for (int j = 0; j < grid.getWidth(); ++j){
				int x = grid.getXStart() + j;

				//p(si,j = occupied | oi,j) = p(oi,j | si,j = occupied)p(si,j = occupied) / p(oi,j)

			}
		}
	}
};

#endif