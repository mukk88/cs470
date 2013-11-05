#pragma once
#include "command.h"
#include "potentialfield.h"

class Agent{
private:
	string color;
	double goal[2];
	double home[2];
	map<int, string> goalMapping;
	string HOME;
	bool missionAccomplished;
	BZRC*commandCenter;
	PotentialField* pfield; 
public:
	Agent(BZRC* bzrc, PotentialField* p, string color){
		commandCenter = bzrc;
		pfield = p;
		HOME = "home";
		vector<tank_t> myTanks;
		commandCenter->get_mytanks(myTanks);
		for (int i = 0; i < myTanks.size(); ++i){
			goalMapping.insert( pair<int, string>(i, color) );			
		}
		missionAccomplished = false;
	}

	~Agent(){
	}

	double distancePoints(double currentLocation[], double object[]){
		return sqrt( pow(currentLocation[0] - object[0], 2) + 
			pow(currentLocation[1] - object[1], 2) );	
	}

	double angle_from_tank_to_point(double currentLocation[], double object[]){
		return atan2( currentLocation[1] - object[1],
			 currentLocation[0] - object[0] );	
	}

	void setGoal(double x, double y){
		goal[0] = x;
		goal[1] = y;
	}

	void setGoal(int index){
		string goalName = goalMapping[index];
		if (goalName.compare(HOME) == 0)
		{
			goal[0] = home[0];
			goal[1] = home[1];
		}
		else
		{
			get_flag_location(goalName, goal);
		}
	}

	void calculate_attraction(double currentLocation[], double attraction[]){
		double distance = distancePoints(currentLocation, goal);
		double angle = angle_from_tank_to_point(currentLocation, goal);
		if (distance <= 0){
			attraction[0] = attraction[1] = 0;
		}
		else if (distance > pfield->attractionSpread){
			double coefficent = pfield->attractionConst * pfield->attractionSpread;
			attraction[0] = coefficent * cos(angle);
			attraction[1] = coefficent * sin(angle);
		}
		else {
			double coefficent = pfield->attractionConst * fmax(distance,20);
			attraction[0] = coefficent * cos(angle);
			attraction[1] = coefficent * sin(angle);
		}
	}


	tank_t get_tank(int index){
		vector<tank_t> myTanks;
		commandCenter->get_mytanks(myTanks);
		return myTanks[index];
	}

	void add_values(double a[], double b[], double result[]){
		result[0] = a[0] + b[0];
		result[1] = a[1] + b[1];
	}

	bool calculate_potential_field(int index, double pf[]){
		tank_t tank = get_tank(index);
		double attraction[2];
		calculate_attraction(tank.pos, attraction);
		if (attraction[0] == 0 && attraction[1] == 0)
			return true;

		int tankX = (tank.pos[0] + 400)/2;
		int tankY = (tank.pos[1] + 400)/2;

		double repulsion[2] = {pfield->repulseArray[tankX][tankY].x,pfield->repulseArray[tankX][tankY].y};
		add_values(attraction, repulsion, pf);
		return false;
	}		

	double ratioed(double diff){
		return diff/M_PI*1;
	}

	double calculate_angvel(int index, double target[]){
		tank_t tank = get_tank(index);
		double tankAngle = fmod(tank.angle, M_PI*2);
		double angle = atan2(target[1], target[0]);
		return ratioed(angle - tankAngle);
	} 	

	double calculate_speed(double pf[]){
		double distance = sqrt( pow(pf[0], 2) + pow(pf[1], 2) );
		double speed = distance / 30;
		return fmin(speed, 1); // maximum speed is 1
	} 
	
	bool pf_move(int index, bool shootBullet){
		if (get_tank(index).status != "alive")
			return false;
		
		//setGoal
		if(get_tank(index).flag!="-"){
			goal[0] = home[0];
			goal[1] = home[1];
			missionAccomplished = true;
		}else{
			setGoal(index);
		}

		double pf[2];
		bool mission_accomplished = calculate_potential_field(index, pf);

		double angularvel = calculate_angvel(index, pf);
		commandCenter->angvel(index, angularvel);
		double velocity = calculate_speed(pf);
		commandCenter->speed(index, velocity);

		if (shootBullet)
			commandCenter->shoot(index);

		if(get_tank(index).flag!="-"){
			return true;
		}else{
			return false;
		}
	}

	bool set_home_location() {	
		set<string> colors;
		colors.insert("blue");
		colors.insert("purple");
		colors.insert("green");
		colors.insert("red");
		// Request a list of tanks that aren't our robots.
		commandCenter->SendLine("othertanks");
		commandCenter->ReadAck();
		vector <string> v=commandCenter->ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=commandCenter->ReadArr();
		int i=0;
		while(v.at(0)=="othertank") {	
			if (colors.find( v.at(2) ) != colors.end())	
			{	
	  			colors.erase( colors.find( v.at(2) ) );
	  		}
			// v.clear();
			vector<string> temp;
			v = temp;
			v=commandCenter->ReadArr();
			++i;
		}
		if(v.at(0)!="end") {
			return false;
		}
		for (set<string>::iterator it=colors.begin(); it!=colors.end(); ++it){
    		color = *it;
    		break;
    	}

		return get_flag_location(color, home);
	}

	bool get_flag_location(string desired_color, double home[]){
		commandCenter->SendLine("flags");
		commandCenter->ReadAck();
		vector <string> v=commandCenter->ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=commandCenter->ReadArr();		
		int i=0;
		while(v.at(0)=="flag") {
			if (v.at(1).compare(desired_color) == 0){
				home[0]=atof(v.at(3).c_str());
				home[1]=atof(v.at(4).c_str());
				break;
			}
			v.clear();
			v=commandCenter->ReadArr();
			++i;
		}

		while (v.at(0)!="end") {
			v.clear();
			v=commandCenter->ReadArr();
			++i;
		}
		return false;
	}
};