#pragma once
#include "command.h"
#include <vector>

#define repulseDistance 35

class PotentialField{
private:

public:
	int attractionSpread;
	double attractionConst;
	int repulsionSpread;
	double repulsionConst;
	Node repulseArray[400][400];
	BZRC*commandCenter;
	vector<Node> replusePoints;

	PotentialField(BZRC* bzrc){
		commandCenter = bzrc;
		attractionSpread = 100;
		attractionConst = -0.2;
		repulsionSpread = 25;
		repulsionConst = -0.8;
		calculate_repulsion();
	}

	void createRepulsion(obstacle_t obs){
		//walk along the sides and create a list of points
		vector<Node> points;
		Node start, end, step;
		for(int j= 0; j<4;j++){
			start.x = obs.o_corner[0+j][0] + 400;
			start.y = obs.o_corner[0+j][1] + 400;
			end.x = obs.o_corner[(1+j)%4][0] + 400;
			end.y = obs.o_corner[(1+j)%4][1] + 400;
			step = stepResult(start,end);
			points.push_back(start);
			int steps = stepsNo(start,end);
			for(int i=0;i<steps;i++){
				start.x += step.x;
				start.y += step.y;
				points.push_back(start);
			}			
		}
		//loop through all points for whole map
		for(int j=0;j<800;j+=2){
			for(int i=0;i<800;i+=2){
				double repulseScalar = 0.7;
				double tanScalar = 0.6;
				Node node, dir, tang;
				double dirX = 0, dirY = 0;
				for(int k=0;k<points.size();k++){
					double repulseMag = repulseScalar * (repulseDistance -distancePoints(i,j,points[k]));
					double tanMag = tanScalar * (repulseMag/repulseScalar);
					dir.x = i - points[k].x;
					dir.y = j - points[k].y;
					norm(dir);
					dirX += dir.x*repulseMag;
					dirY += dir.y*repulseMag;
					tang = perpen(dir);
					dirX += tang.x*tanMag;
					dirY += tang.y*tanMag; 
				}
				repulseArray[i/2][j/2].x += dirX;
				repulseArray[i/2][j/2].y +=dirY; 
			}
		}
	}

	void norm(Node& node){
		double x = node.x;
		double y = node.y;
		double distance = pow(pow(x,2) + pow(y,2),0.5);
		if(distance!=0){
			node.x = x/distance;
			node.y = y/distance;
		}else{
			node.x = 0;
			node.y = 0;
		}
	}

	Node perpen(Node node){
		Node result;
		result.x = node.y;
		result.y = -node.x;
		return result;
	}		

	Node stepResult(Node start, Node end){
		Node result;
		int yDiff = end.y - start.y;
		int xDiff = end.x - start.x;
		int sum = abs(yDiff) + abs(xDiff);
		if(!sum){
			cerr << "error in points" << endl;
			return result;
		}
		int xStep = xDiff/sum;
		int yStep = yDiff/sum;
		result.x = repulseDistance*xStep;
		result.y = repulseDistance*yStep;
		return result;
	}

	int stepsNo(Node start, Node end){
		double dist = distancePoints(start, end);
		return (int)(dist/repulseDistance);
	}

	double distancePoints(Node start, Node end){
		return sqrt( pow(end.x - start.x, 2) + 
			pow(end.y - start.y, 2) );	
	}

	double distancePoints(int startX, int startY, Node end){
		double result =  sqrt( pow(end.x - startX, 2) + 
			pow(end.y - startY, 2) );	
		return result > repulseDistance ? repulseDistance : result;
	}

	bool calculate_repulsion(double pos[], double repulsion[]){

		bool repulsed = false;

		for (int i = 0; i < replusePoints.size(); ++i){

			double distance = distancePoints(pos[0], pos[1], replusePoints[i]);
			
			if (distance < repulsionSpread){
				double coefficent = repulsionConst * (repulsionSpread - distance);
				double pointRepulsion[2];
				double angle = angle_from_tank_to_point(pos, replusePoints[i]);
				
				pointRepulsion[0] = coefficent * cos(angle);
				pointRepulsion[1] = coefficent * sin(angle);

				repulsion[0] += pointRepulsion[0];
				repulsion[1] += pointRepulsion[1];
			}
			repulsed = true;
		}

		return repulsed;
	}


	double angle_from_tank_to_point(double currentLocation[], Node & object){
		return atan2( currentLocation[1] - object.y,
			 currentLocation[0] - object.x );	
	}

	void calculate_repulsion(){
		vector<obstacle_t> obstacles;
		commandCenter->get_obstacles(obstacles);
		for(int i=0;i<400;i++){
			for(int j=0;j<400;j++){
				repulseArray[i][j].x = 0;
				repulseArray[i][j].y = 0;
			}
		}
		for (int i = 0; i < obstacles.size(); ++i){
			createRepulsion(obstacles[i]);
		}
	}

	void getRepulsion(Node repulsion[400][400]){
		for(int i=0;i<400;i++){
			for(int j=0;j<400;j++){
				repulsion[i][j].x = repulseArray[i][j].x;
				repulsion[i][j].y = repulseArray[i][j].y;
			}
		}
	}

	void addPoint(double pos[]){
		Node result;
		result.x = pos[0];
		result.y = pos[1];
		replusePoints.push_back(result);
	}
};