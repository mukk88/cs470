#include <iostream>
#include <fstream>
#include "bot.h"

using namespace std;

// struct Node{
// 	double x;
// 	double y;
// };


void printObstacleData(vector<obstacle_t>& obstacles, ostream& outfile);
void mainPrint(vector<obstacle_t>& obstacles);
void printLine(Node start, Node end, ofstream& outfile);

void printLine(Node start, Node end, ofstream& outfile){
   char buff[100];
   double sx = start.x;
   double sy = start.y;
   double ex = end.x;
   double ey = end.y;
   sprintf(buff, "set arrow from %f, %f to %f, %f lt %d\n", sx,sy,ex,ey, 1);
   outfile << buff;
}

void printObstacleData(vector<obstacle_t>& obstacles, ofstream & outfile) {
  char buff[100];
  for (int i = 0; i < (int)obstacles.size(); i++) {
     double x1;
     double x2;
     double y1;
     double y2;
     obstacle_t o= obstacles[i];
     for (int j = 0; j < 4; j++) {
         x1 = o.o_corner[j][0];
         y1 = o.o_corner[j][1];
         x2 = o.o_corner[(j+1)%4][0];
         y2 = o.o_corner[(j+1)%4][1];
         sprintf(buff,"set arrow from %f, %f to %f, %f nohead lt 3\n", x1,y1,x2,y2);
         outfile << buff;
     }
  }
}

void printPotentials(ofstream& outfile, BZRC& myTeam){
	Node repulsion[400][400];
	// myTeam.calculate_repulsion(repulsion);
	myTeam.getRepulsion(repulsion);
	for(int i=0;i<400;i+=10){
		for(int j=0;j<400;j+=10){
			Node start, end;
			start.x = i*2-400;
			start.y = j*2-400;
			end.x = start.x + repulsion[i][j].x;
			end.y = start.y + repulsion[i][j].y;
			double curLoc[2] = {start.x, start.y};
			double deltas[2];
			myTeam.calculate_attraction(curLoc, deltas);
			end.x += deltas[0];
			end.y += deltas[1];
			printLine(start,end, outfile);
		}
	}
}

void mainPrint(vector<obstacle_t>& obstacles, BZRC& myTeam){
	ofstream outfile;
	outfile.open("plot2.gpi");
	outfile << "set title \"My Title\"" << endl;
	outfile << "set xrange [-400.0: 400.0]" << endl;
	outfile << "set yrange [-400.0: 400.0]" << endl;
	outfile << "unset key" << endl;
	outfile << "set size square" << endl;

	printObstacleData(obstacles, outfile);

	Node start, end;
	start.x = 100;
	start.y = 100;
	end.x = 200;
	end.y = 200;

	printPotentials(outfile, myTeam);

	outfile << "plot \'-\' with lines\n";
	outfile << "0 0 0 0\ne\n";
	outfile.close();
}
