#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <unistd.h>
#include <ctype.h>
#include <vector>
#include "bot.h"
using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
bool debugMode = false;

void robot_post_update(){
	
}

void world_init(BZRC *my_team){
	if(debugMode) cout << "initializing world\n"; 
}

void dumbAgents(vector<int> numbots, BZRC& myTeam){
	for(int i=0;i<numbots.size();i++){
		myTeam.shoot(numbots[i]);
		myTeam.speed(numbots[i],10);
	}
	sleep(2);
	for(int i=0;i<numbots.size();i++){
		myTeam.shoot(numbots[i]);
	}
	sleep(2);
	for(int i=0;i<numbots.size();i++){
		myTeam.speed(numbots[i], 0);
		myTeam.shoot(numbots[i]);
		myTeam.angvel(numbots[i], 0.5);
	}
	sleep(2);
	for(int i=0;i<numbots.size();i++){
		myTeam.shoot(numbots[i]);
	}
	sleep(2);
	for(int i=0;i<numbots.size();i++){
		myTeam.angvel(numbots[i], 0);
	}
}


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

	BZRC myTeam = BZRC(pcHost, port, false);
	if(!myTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	int botnum = 8;
	if(debugMode) cout << "calling agent code" << endl;

	vector<int> botnums;
	botnums.push_back(2);
	botnums.push_back(6);
	while(true){
		dumbAgents(botnums, myTeam);
	}
	



	// Calling agent code
	// world_init(&MyTeam);
	// for(int i=1; i>0; i++) {
	// 	// robot_pre_update();
	// 	// robot_update();
	// 	robot_post_update();
	// 	sleep(50);
	// }

	// MyTeam.Close();
	// free(&MyTeam);
	return 0;
}