
#ifndef RANDOMOBSTACLESEARCH_H
#define RANDOMOBSTACLESEARCH_H

#include <stdlib.h> 
#include <time.h>

class RandomObstacleSearchAgent : public ObstacleSearchAgent{
private:
	int randx;
	int randy;
	int counter;
public:
	RandomObstacleSearchAgent(int index, BZRC* bzrc, PotentialField* p, string color, 
		Grid* g) : ObstacleSearchAgent(index, bzrc, p, color, 0,0,g,0){
		counter = 0;
		srand(time(NULL));
		//updateGoal();
	}

	bool randomMove(){
		counter++;
		if (get_tank().status != "alive")
			return false;

		if (missionAccomplished){
			missionAccomplished = false;
			updateGoal();
		}
		// cout << "called " << counter << endl;
		if(counter > 5){
			updateGoal();
			counter-=5;
		}
		Agent::move(false);
		ObstacleSearchAgent::observe();
	}

	void updateGoal(){
		randx = ( rand() % 799 )- 399;
		randy = ( rand() % 799) - 399;
		goal[0] = randx;
		goal[1] = randy;
	}

};

#endif
