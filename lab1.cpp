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

#include "occgrid.h"

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
bool debugMode = false;

int flagBearer = -1;

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

	OccGrid o;
	myTeam->get_occ(1, o);

	// retrieve constants
	vector<constant_t> constants;
	if (!myTeam->get_constants(&constants)){
		cerr << "unable to retrieve constants" << endl;
		return -1;
	}

	/*ObstacleSearchAgent::initializeGrid(atoi(constants[1].value.c_str()));

	vector<constant_t>::iterator it = constants.end(); 
	ObstacleSearchAgent::setTrueNegative(atoi((*it--).value.c_str()));
	ObstacleSearchAgent::setTruePositive(atoi((*it).value.c_str()));*/

	vector<Agent> agents;
	for(int i=0;i<4;i++){
		//int index, BZRC* bzrc, PotentialField* p, string color, bool moveR, bool moveU
		Agent a = ObstacleSearchAgent(i, myTeam, pfield, "green", true, false);
		agents.push_back(a);
	}
	while(true){
		for(int i=0;i<agents.size();i++){
			agents[i].move(true);
		}
	}

	delete myTeam;
	delete pfield;
	cout << "done" << endl;
	return 0;
}