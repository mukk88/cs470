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
	Node repulsion[40][40];
	myTeam.calculate_repulsion(repulsion);
	for(int i=0;i<40;i++){
		for(int j=0;j<40;j++){
			Node start, end;
			start.x = j*20-400;
			start.y = i*20-400;
			end.x = start.x + repulsion[i][j].x;
			end.y = start.y + repulsion[i][j].y;
			printLine(start, end,outfile);
			// cout << repulsion[j][i].x << "," << repulsion[j][i].y << " ";
		}//cout << endl;
	}
	// for(int i=-390;i<0;i+=20){
	// 	for(int j=-390;j<0;j+=20){
	// 		Node start, end;
	// 		start.x = i;
	// 		start.y = j;
	// 		double curLoc[2] = {i,j};
	// 		double deltas[2];
	// 		// cout << deltas[1] << " " << deltas[2] << endl;
	// 		// myTeam.calculate_attraction(curLoc, deltas);
	// 		end.x = -deltas[0] + i;
	// 		end.y = -deltas[1] + j;
	// 		// cout << end.x << " " << end.y << endl;
	// 		printLine(start, end,outfile);
	// 	}
	// }
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
	// printLine(start, end, outfile);
	printPotentials(outfile, myTeam);

	// double x, y, result;
	// double pi = atan(1)*4;
	// x = -10.0;
	// y = 10.0;
	// result = atan2 (y,x) * 180 / pi;
	// printf ("The arc tangent for (x=%f, y=%f) is %f degrees\n", x, y, result );

	outfile << "plot \'-\' with lines\n";
	outfile << "0 0 0 0\ne\n";
	outfile.close();
}
