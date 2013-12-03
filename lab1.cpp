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
#include "kalmanAgent.h"
#include "occgrid.h"

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
bool debugMode = false;

int flagBearer = -1;

//cmd : --default-true-positive=.97 --default-true-negative=.9 --occgrid-width=100 --no-report-obstacles
//cmd: ./bin/bzrflag --world=maps/blank.bzw --blue-tanks=1 --red-tanks=1 --green-tanks=1 --purple-tanks=1 --default-posnoise=5

int grid_filter_lab(BZRC* purple, BZRC* red, BZRC* blue, BZRC* green, PotentialField* pfield){
	//retrieve constants, assume there is purple for now
	vector<constant_t> constants;
	if (!purple||!purple->get_constants(&constants)){
		cerr << "unable to retrieve constants" << endl;
		return -1;
	}

	int worldSize = atoi(constants[1].value.c_str());
	Grid* grid = new Grid(worldSize, worldSize, atof(constants[16].value.c_str()), atof(constants[17].value.c_str()));

	vector<ObstacleSearchAgent> agents;

	ObstacleSearchAgent a = ObstacleSearchAgent(9, red, pfield, "green", true, true, grid, true);
	a.setGoal(-375, -375);
	agents.push_back(a);
	a = ObstacleSearchAgent(9, green, pfield, "green", false, false, grid, true);
	a.setGoal(375, 375);
	agents.push_back(a);
	a = ObstacleSearchAgent(9, blue, pfield, "green", true, false, grid, false);
	a.makeHorizontalMower();
	a.setGoal(-375, 375);
	agents.push_back(a);

	a = ObstacleSearchAgent(9, purple, pfield, "green", false, true, grid, false);
	a.makeHorizontalMower();
	a.setGoal(375, -375);
	agents.push_back(a);
	

	vector<RandomObstacleSearchAgent> purpleagents;
	for(int i=0;i<2;i++){
		RandomObstacleSearchAgent a = RandomObstacleSearchAgent(i, purple, pfield, "green", grid);
			if (i == 0)
				a.setGoal(0, 0);
			else
				a.setGoal(worldSize/4, -worldSize/4);
		purpleagents.push_back(a);
	}
	for (int i = 2; i < 9; i++){
		purple->speed(i, 1);
		blue->speed(i, 1);
		green->speed(i, 1);
		red->speed(i, 1);
	}
	
	vector<RandomObstacleSearchAgent> greenagents;
	if(green)
		for(int i=0;i<2;i++){
			RandomObstacleSearchAgent a = RandomObstacleSearchAgent(i, green, pfield, "green", grid);
			if (i == 0)
				a.setGoal(0, 0);
			else
				a.setGoal(worldSize/4, worldSize/4);
			greenagents.push_back(a);
		}
	vector<RandomObstacleSearchAgent> redagents;
	if(red)
		for(int i=0;i<2;i++){
			RandomObstacleSearchAgent a = RandomObstacleSearchAgent(i, red, pfield, "green", grid);
			if (i == 0)
				a.setGoal(0, 0);
			else
				a.setGoal(-worldSize/4, -worldSize/4);
			redagents.push_back(a);
		}
	vector<RandomObstacleSearchAgent> blueagents;
	if(blue)
		for(int i=0;i<2;i++){
			RandomObstacleSearchAgent a = RandomObstacleSearchAgent(i, blue, pfield, "green", grid);
			if (i == 0)
				a.setGoal(0, 0);
			else
				a.setGoal(-worldSize/4, worldSize/4);
			blueagents.push_back(a);
		}

	vector< vector<RandomObstacleSearchAgent> > randomagents;
	randomagents.push_back(purpleagents);
	randomagents.push_back(blueagents);
	randomagents.push_back(greenagents);
	randomagents.push_back(redagents);

	int counter = 0;

	while(true){
		for(int i=0;i<randomagents.size();i++){
			for(int j=0;j<agents.size();j++){
				agents[j].move();
			}
			grid->print();
			for(int j=0;j<randomagents[i].size();j++){
				randomagents[i][j].randomMove();
			}
			grid->print();
		}
	}
}

int main(int argc, char *argv[]) {
	const char *pcHost = "127.0.0.1";
	int portpurple = 0;
	int portgreen = 0;
	int portblue = 0;
	int portred = 0;
	int option;

	while ((option = getopt(argc,argv,"s:p:g:b:r:d")) != -1) {
        switch (option) {
            case 'p':
                portpurple = atoi(optarg);
                break;
            case 'g':
            	portgreen = atoi(optarg);
            	break;
            case 'b':
            	portblue = atoi(optarg);
            	break;
            case 'r':
            	portred = atoi(optarg);
            	break;	
            case 's':
                pcHost = optarg;
                break;
            case 'd':
            	debugMode = true;
            	break;
            default:
                cout << "client [-s IP address] [-c port], where c is the first char of the color" << endl;
                exit(EXIT_FAILURE);
        }
    }
    BZRC* red = NULL;
    BZRC* blue = NULL;
    BZRC* purple = NULL;
    BZRC* green = NULL;
    if(portred)
    	red = new BZRC(pcHost, portred, false);
    if(portpurple)
    	purple = new BZRC(pcHost, portpurple, false);
    if(portblue)
    	blue = new BZRC(pcHost, portblue, false);
    if(portgreen)
    	green = new BZRC(pcHost, portgreen, false);

	PotentialField* pfield = new PotentialField(purple);

	//sitting duck is red; no movement
	green->speed(0,.5);
	//Agent constantAgent(0, green, pfield, "green");
	KalmanAgent scannerAgent(0, purple, pfield, "purple");
	// Random agent
	//RandomObstacleSearchAgent a = RandomObstacleSearchAgent(0, blue, pfield, "green");

	delete purple;
	if(red)
		delete red;
	if(blue)
		delete blue;
	if(green)
		delete green;
	delete pfield;
	// delete grid;
	cout << "done" << endl;
	return 0;
}