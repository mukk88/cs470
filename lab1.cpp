#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <algorithm>
#include <iterator>
#include <unistd.h>
#include <ctype.h>
#include <vector>
#include "command.h"
#include "agent.h"
#include "potentialfield.h"
#include "obstacleSearchAgent.h"
#include "randomAgent.h"

#include "occgrid.h"

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
bool debugMode = false;

int flagBearer = -1;


//cmd : --default-true-positive=.97 --default-true-negative=.9 --occgrid-width=100 --no-report-obstacles
// void dumbAgents(vector<int> numbots, BZRC& myTeam){
// 	for(int i=0;i<numbots.size();i++){
// 		myTeam.shoot(numbots[i]);
// 		myTeam.speed(numbots[i],10);
// 	}
// 	sleep(2);
// 	for(int i=0;i<numbots.size();i++){
// 		myTeam.shoot(numbots[i]);
// 	}
// 	sleep(2);
// 	for(int i=0;i<numbots.size();i++){
// 		myTeam.speed(numbots[i], 0);
// 		myTeam.shoot(numbots[i]);
// 		myTeam.angvel(numbots[i], 0.5);
// 	}
// 	sleep(2);
// 	for(int i=0;i<numbots.size();i++){
// 		myTeam.shoot(numbots[i]);
// 	}
// 	sleep(2);
// 	for(int i=0;i<numbots.size();i++){
// 		myTeam.angvel(numbots[i], 0);
// 	}
// }

// void pfAgents(vector<int> numbots, BZRC& myTeam, bool shoot = false){
// 	bool goal = false;
// 	if(flagBearer!=-1){
// 		if(!myTeam.pf_move(flagBearer, shoot)){
// 			flagBearer = -1;
// 		}
// 	}
// 	else{
// 		for(int i=0;i<numbots.size();i++){
// 			if(myTeam.pf_move(numbots[i], shoot)){
// 				flagBearer = numbots[i];
// 			}
// 		}
// 	}
// 	sleep(1);
// }

int main(int argc, char *argv[]) {
	const char *pcHost = "127.0.0.1";
	int port = 4000;
	int option;

	while ((option = getopt(argc,argv,"s:p:d")) != -1) {
        switch (option) {
            case 'p':
                port = atoi(optarg);
                break;
            case 's':
                pcHost = optarg;
                break;
            case 'd':
            	debugMode = true;
            	break;
            default:
                cout << "client [-s IP address] [-p port]" << endl;
                exit(EXIT_FAILURE);
        }
    }

	BZRC* myTeam = new BZRC(pcHost, port, false);
	PotentialField* pfield = new PotentialField(myTeam);

	// retrieve constants
	vector<constant_t> constants;
	if (!myTeam->get_constants(&constants)){
		cerr << "unable to retrieve constants" << endl;
		return -1;
	}

	int worldSize = atoi(constants[1].value.c_str());
	cout << worldSize << endl;
	Grid* grid = new Grid(worldSize, worldSize, 0.99, 0.98);
	// Grid* grid = new Grid(worldSize, worldSize, atof(constants.end()->value.c_str()), atof(constants.end()->value.c_str()));

	vector<ObstacleSearchAgent> agents;

	ObstacleSearchAgent a = ObstacleSearchAgent(0, myTeam, pfield, "green", true, true, grid, true);
	a.setGoal(-375, -375);
	agents.push_back(a);
	a = ObstacleSearchAgent(1, myTeam, pfield, "green", false, false, grid, true);
	a.setGoal(375, 375);
	agents.push_back(a);
	a = ObstacleSearchAgent(2, myTeam, pfield, "green", true, false, grid, false);
	a.makeHorizontalMower();
	a.setGoal(-375, 375);
	agents.push_back(a);

	a = ObstacleSearchAgent(3, myTeam, pfield, "green", false, true, grid, false);
	a.makeHorizontalMower();
	a.setGoal(375, -375);
	agents.push_back(a);
	

	vector<RandomObstacleSearchAgent> randomagents;
	for(int i=4;i<10;i++){
		RandomObstacleSearchAgent a = RandomObstacleSearchAgent(i, myTeam, pfield, "green", grid);
		randomagents.push_back(a);

	}
	

	int counter = 0;

	while(true){
		for(int i=0;i<agents.size();i++){
			agents[i].move();
		}
		for(int i=0;i<randomagents.size();i++){
			randomagents[i].randomMove();
		}
		if(counter<2){
			counter++;
		}else{
			counter-=2;
			cout << "writing" << endl;
			grid->print();
		}
	}

	delete myTeam;
	delete pfield;
	delete grid;
	cout << "done" << endl;
	return 0;
}