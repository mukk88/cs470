#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <unistd.h>
#include <ctype.h>
#include <vector>
#include "command.h"
#include "kalmanAgent.h"
#include "randomAgent.h"

using namespace std;

//cmd: ./bin/bzrflag --world=maps/blank.bzw --blue-tanks=1 --red-tanks=1 --green-tanks=1 --purple-tanks=1 --default-posnoise=5

int main(int argc, char *argv[]) {
	const char *pcHost = "127.0.0.1";
	int portpurple = 0;
	int portgreen = 0;
	int portblue = 0;
	int portred = 0;
	int option;

	while ((option = getopt(argc,argv,"s:p:g:b:r:")) != -1) {
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
            default:
                cout << "client [-s IP address] [-c port], where c is the first char of the color" << endl;
                exit(EXIT_FAILURE);
        }
    }
    BZRC* blue = NULL;
    BZRC* red = NULL;
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

    green->speed(0,0.1);
    RandomAgent random(0, blue, 10);
    KalmanAgent kalmanGreen(0, purple, .5, .1, "green");
    KalmanAgent kalmanBlue(0, purple, .5, .1, "blue");
    KalmanAgent kalmanRed(0, purple, .5, .1, "red");

    while (true){
        sleep(.5);
        random.move();
        kalmanGreen.update();
        kalmanBlue.update();
        kalmanRed.update();
    }

    if(purple)
        delete purple;
    if(red)
        delete red;
    if(blue)
        delete blue;
    if(green)
        delete green;

	return 0;
}