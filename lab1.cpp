#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <unistd.h>
#include <ctype.h>
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

	BZRC MyTeam = BZRC(pcHost, port, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	if(debugMode) cout << "calling agent code" << endl;
	// Calling agent code
	world_init(&MyTeam);
	for(int i=1; i>0; i++) {
		// robot_pre_update();
		// robot_update();
		robot_post_update();
		sleep(50);
	}

	MyTeam.Close();
	// free(&MyTeam);
	return 0;
}