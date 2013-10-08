#define _USE_MATH_DEFINES
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <set>
#include <map>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <cmath>
#include "velocityPDController.h"
#include "anglePDController.h"

using namespace std;

#define MAX_OBSTACLE_CORNERS 10
const int kBufferSize = 1024;
#define repulseDistance 50

struct Node{
	double x;
	double y;
};

typedef struct team_t {
	string color;
	int count;
	double base_corner[4][2];
} team_t;

typedef struct obstacle_t {
	double o_corner[MAX_OBSTACLE_CORNERS][2];
} obstacle_t;

typedef struct flag_t {
	string color;
	string poss_color;
	double pos[2];
} flag_t;

typedef struct shot_t {
	double pos[2];
	double velocity[2];
} shot_t;

typedef struct tank_t {
	int index;
	string callsign;
	string status;
	int shots_avail;
	double time_to_reload;
	string flag;
	double pos[2];
	double angle;
	double velocity[2];
	double angvel;
} tank_t;

typedef struct otank_t {
	string callsign;
	string color;
	string status;
	string flag;
	double pos[2];
	double angle;
} otank_t;

typedef struct constant_t {
	string name;
	string value;
} constant_t;

class SplitString {
	vector <string> MyVector;
	string MyString;
public:
	SplitString(string str) {
		MyString=str;
		MyVector.begin();
	}

	vector <string> Split(bool skipBlanks = false) {
		MyVector.clear();
		size_t LastLoc = -1;
		size_t CurLoc = MyString.find(" ", 0);
		while (CurLoc != string::npos) {
			string curValue = MyString.substr(LastLoc+1, CurLoc-LastLoc-1);
			if (!skipBlanks || curValue.compare("") != 0)
				MyVector.push_back(curValue);
			LastLoc=CurLoc;
			CurLoc = MyString.find(" ", LastLoc+1);
		}
		MyVector.push_back(MyString.substr(LastLoc+1, MyString.size()-LastLoc));
		return MyVector;
	}
};

class BZRC {
	const char *pcHost;
	int nPort;
	bool debug;
	bool InitStatus;
	char ReplyBuffer[kBufferSize];
	int LineFeedPos;
	int Start;
	int sd;
	//constants
	int attractionSpread;
	double attractionConst;
	int replusionSpread;
	double replusionConst;
	double goal[2];
	vector<AnglePDController*> angleControllers;
	vector<VelocityPDController*> velocityControllers;
	vector<double> latestVelocity;
	vector<double> latestAngVel;
	string color;
	double home[2];
	map<int, string> goalMapping;
	string HOME;
	bool missionAccomplished;
	Node repulseArray[400][400];

	// Initializing connection.
	int Init() {
		ResetReplyBuffer();
		Start=0;

		struct addrinfo *infop = NULL;
		struct addrinfo hint;

		memset(&hint, 0, sizeof(hint));
		hint.ai_family = AF_INET;
		hint.ai_socktype = SOCK_STREAM;

		char port[10];
		snprintf(port, 10, "%d", nPort);

		if (getaddrinfo(pcHost, port, &hint, &infop) != 0) {
			perror("Couldn't lookup host.");
			return 1;
		}

		if ((sd = socket(infop->ai_family, infop->ai_socktype,
						infop->ai_protocol)) < 0) {
			perror("Couldn't create socket.");
			return 1;
		}

		if (connect(sd, infop->ai_addr, infop->ai_addrlen) < 0) {
			perror("Couldn't connect.");
			close(sd);
		}

		freeaddrinfo(infop);

		if(HandShake()==1) {
			cerr << "Handshake failed!" << endl;
			return 1;
		}

		return 0;		
	}

	// Send line to server
	int SendLine(const char *LineText) {
		int Length=(int)strlen(LineText);
		char Command[kBufferSize];
		strcpy(Command, LineText);
		Command[Length]='\n';
		Command[Length+1]='\0';
		if(debug) cout << Command;
		if (send(sd, Command, Length+1, 0) >= 0) {
			return 0;
		}
		else {
			return 1;
		}
	}

	// Read line back from server
	int ReadReply(char *Reply){
		char acReadBuffer[kBufferSize];

		int nNewBytes = recv(sd, acReadBuffer, kBufferSize, 0);
		if (nNewBytes < 0) {
			return -1;
		}
		else if (nNewBytes == 0) {
			cerr << "Connection closed by peer." << endl;
			return 0;
		}
		
		memcpy(Reply, &acReadBuffer, nNewBytes);
		if(nNewBytes!=kBufferSize) {
			Reply[nNewBytes]='\0';
		}

		return nNewBytes;
	}

	// Only read one line of text from ReplyBuffer
	void ReadLine(char *LineText) {
		memset(LineText, '\0', kBufferSize);
		// Only read more from server when done with current ReplyBuffer
		if(strlen(ReplyBuffer)==0) {
			char *Reply;
			Reply = ReplyBuffer;
			ReadReply(Reply);
		}
		int i=0;
		bool done=false;
		while(!done) {
			for(i=LineFeedPos+1; (i<kBufferSize && ReplyBuffer[i]); i++) {
				if(ReplyBuffer[i]=='\n') {
					LineText[i-LineFeedPos-1+Start]='\0';
					LineFeedPos=i;
					Start=0;
					done=true;
					break;
				}
				LineText[i-LineFeedPos-1+Start]=ReplyBuffer[i];
			}
			if(!done) {
					Start = (int)strlen(LineText);
					ResetReplyBuffer();	
					char *Reply;
					Reply = ReplyBuffer;
					ReadReply(Reply);
			}
			else {
				if(ReplyBuffer[i]=='\0') {
					done=true;
					Start=0;
					ResetReplyBuffer();
				}
			}
		}
	}

	// Reset the ReplyBuffer
	void ResetReplyBuffer() {
		memset(ReplyBuffer, '\0', kBufferSize);
		LineFeedPos=-1;
	}

	// Perform HandShake with the server
	int HandShake() {
		char str[kBufferSize];
		char *LineText;
		LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
		if (!strcmp(LineText, "bzrobots 1")) {
			const char * Command="agent 1";
			int temp=SendLine(Command);
			if(temp==1) 
				return 1;
			else
				ResetReplyBuffer();
				return 0;
		}
		else
			return 1;
	}

	// Read line into vector
	vector <string> ReadArr(bool skipBlanks = false) {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(strlen(LineText)!=0) {
			if(debug) cout << LineText << endl;
		}
		while(strlen(LineText)==0) {
			ReadLine(LineText);
			if(debug) cout << LineText << endl;
		}
		SplitString ss=SplitString(LineText);
		return ss.Split(skipBlanks);
	}
	// Read Acknowledgement
	void ReadAck() {
		vector <string> v=ReadArr();
		if(v.at(0)!="ack") {
			cout << "Didn't receive ack! Exit!" << endl;
			exit(1);
		}
	}
	// Read "ok"
	bool ReadBool() {
		vector <string> v=ReadArr();
		if(v.at(0)=="ok") {
			return true;
		}
		else if(v.at(0)=="fail"){
			if(debug) cout << "Received fail. Exiting!" << endl;
			return false;
		}
		else {
			if(debug) cout << "Something went wrong. Exiting!" << endl;
			return false;
		}
	}
	// Receive and print another line
	void PrintLine() {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
	}
public:
	BZRC(const char *host, int port, bool debug_mode) {
		pcHost = host;
		nPort = port;
		debug = debug_mode;
		if(Init()) {
			cout << "BZRC initialization failed." << endl;
			InitStatus=false;
			Close();
		}
		else {
			InitStatus=true;
		}

		//constants 
		attractionSpread = 100;
		attractionConst = -0.2;
		replusionSpread = 50;
		replusionConst = -0.8;

		set_home_location();
		set<string> enemy_colors;
		enemy_colors.insert("green");
		enemy_colors.insert("blue");
		enemy_colors.insert("purple");
		enemy_colors.insert("red");
		enemy_colors.erase(enemy_colors.find(color));
		//create tank controllers
		vector<tank_t> myTanks;
		get_mytanks(myTanks);
		for (int i = 0; i < myTanks.size(); ++i){
			angleControllers.push_back(new AnglePDController());
			velocityControllers.push_back(new VelocityPDController());
			latestVelocity.push_back(0);
			latestAngVel.push_back(0);
			goalMapping.insert( pair<int, string>(i, *(enemy_colors.begin())) );			
		}
		HOME = "home";
		missionAccomplished = false;
	}

	~BZRC(){
		for (int i = 0; i < angleControllers.size(); ++i){
			delete angleControllers[i];
		}
		for (int i = 0; i < velocityControllers.size(); ++i){
			delete velocityControllers[i];
		}
	}

	// Self check
	int GetPort(){return nPort;}
	const char *GetHost() {return pcHost;}
	bool GetStatus() {return InitStatus;}
	// Commands:
	bool shoot(int index) {
		// Perform a shoot request.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="shoot";
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	bool speed(int index, double value) {
		// Set the desired speed to the specified value.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="speed";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			latestVelocity[index] = value;
			return true;
		}
		else {
			return false;
		}
	}

	bool angvel(int index, double value) {
		// Set the desired angular velocity to the specified value.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="angvel";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	bool accelx(int index, double value) {
		// Set the desired accelaration in x axis to the specified value in hovertank mode.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="accelx";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}	
	bool accely(int index, double value) {
		// Set the desired accelaration in x axis to the specified value in hovertank mode.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="accely";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	// Information Request:
	bool get_teams(vector <team_t> *AllTeams) {
		//Request a list of teams.
		SendLine("teams");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="team") {
			team_t MyTeam;
			MyTeam.color=v.at(1);
			MyTeam.count=atoi(v.at(2).c_str());
			MyTeam.base_corner[0][0]=atof(v.at(3).c_str());
			MyTeam.base_corner[0][1]=atof(v.at(4).c_str());
			MyTeam.base_corner[1][0]=atof(v.at(5).c_str());
			MyTeam.base_corner[1][1]=atof(v.at(6).c_str());
			MyTeam.base_corner[2][0]=atof(v.at(7).c_str());
			MyTeam.base_corner[2][1]=atof(v.at(8).c_str());
			MyTeam.base_corner[3][0]=atof(v.at(9).c_str());
			MyTeam.base_corner[3][1]=atof(v.at(10).c_str());
			AllTeams->push_back(MyTeam);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_obstacles(vector <obstacle_t>& AllObstacles) {
		// Request a list of obstacles.
		SendLine("obstacles");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="obstacle") {
			obstacle_t MyObstacle;
			int j=0;
			while(j+2<(int)v.size()) {
				MyObstacle.o_corner[j/2][0]=atof(v.at(j+1).c_str());
				MyObstacle.o_corner[j/2][1]=atof(v.at(j+2).c_str());
				j=j+2;
			}
			AllObstacles.push_back(MyObstacle);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_flags(vector <flag_t> *AllFlags) {
		// Request a list of flags.
		SendLine("flags");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="flag") {
			flag_t MyFlag;
			MyFlag.color=v.at(1);
			MyFlag.poss_color=v.at(2);
			MyFlag.pos[0]=atof(v.at(3).c_str());
			MyFlag.pos[1]=atof(v.at(4).c_str());
			AllFlags->push_back(MyFlag);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool set_home_location() {	
		set<string> colors;
		colors.insert("blue");
		colors.insert("purple");
		colors.insert("green");
		colors.insert("red");
		// Request a list of tanks that aren't our robots.
		SendLine("othertanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="othertank") {	
			if (colors.find( v.at(2) ) != colors.end())	
			{	
	  			colors.erase( colors.find( v.at(2) ) );
	  		}
			// v.clear();
			vector<string> temp;
			v = temp;
			v=ReadArr();
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
		// Request a list of flags.
		SendLine("flags");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();		
		int i=0;
		while(v.at(0)=="flag") {
			if (v.at(1).compare(desired_color) == 0){
				home[0]=atof(v.at(3).c_str());
				home[1]=atof(v.at(4).c_str());
				break;
			}
			v.clear();
			v=ReadArr();
			++i;
		}

		while (v.at(0)!="end") {
			v.clear();
			v=ReadArr();
			++i;
		}
		return false;
	}

	bool get_shots(vector <shot_t> *AllShots) {
		// Request a list of shots.
		SendLine("shots");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="shot") {
			shot_t MyShot;
			MyShot.pos[0]=atof(v.at(1).c_str());
			MyShot.pos[1]=atof(v.at(2).c_str());
			MyShot.velocity[0]=atof(v.at(3).c_str());
			MyShot.velocity[1]=atof(v.at(4).c_str());
			AllShots->push_back(MyShot);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_mytanks(vector <tank_t> &AllMyTanks) {
		// Request a list of our robots.
		SendLine("mytanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr(true);
		int i=0;
		while(v.at(0)=="mytank") {
			tank_t MyTank;
			MyTank.index=atoi(v.at(1).c_str());
			MyTank.callsign=v.at(2);
			MyTank.status=v.at(3);
			MyTank.shots_avail=atoi(v.at(4).c_str());
			MyTank.time_to_reload=atof(v.at(5).c_str());
			MyTank.flag=v.at(6);
			MyTank.pos[0]=atof(v.at(7).c_str());
			MyTank.pos[1]=atof(v.at(8).c_str());
			MyTank.angle=atof(v.at(9).c_str());
			MyTank.velocity[0]=atof(v.at(10).c_str());
			MyTank.velocity[1]=atof(v.at(11).c_str());
			MyTank.angvel=atof(v.at(12).c_str());
			AllMyTanks.push_back(MyTank);
			v.clear();
			v=ReadArr(true);
			i++;
		}
		if(v.at(0)!="end") {
			if(debug) cout << v.at(0) << endl;
			return false;
		}
		return true;
	}

	bool get_othertanks(vector <otank_t> *AllOtherTanks) {
		// Request a list of tanks that aren't our robots.
		SendLine("othertanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="othertank") {
			otank_t OtherTank;
			OtherTank.callsign=v.at(1);
			OtherTank.color=v.at(2);
			OtherTank.status=v.at(3);
			OtherTank.flag=v.at(4);
			OtherTank.pos[0]=atof(v.at(5).c_str());
			OtherTank.pos[1]=atof(v.at(6).c_str());
			OtherTank.angle=atof(v.at(7).c_str());
			AllOtherTanks->push_back(OtherTank);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_constants(vector <constant_t> *AllConstants) {
		// Request a dictionary of game constants.
		SendLine("constants");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="constant") {
			constant_t MyConstant;
			MyConstant.name=v.at(1);
			MyConstant.value=v.at(2);
			AllConstants->push_back(MyConstant);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
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
		else if (distance > attractionSpread){
			double coefficent = attractionConst * attractionSpread;
			attraction[0] = coefficent * cos(angle);
			attraction[1] = coefficent * sin(angle);
		}
		else {
			double coefficent = attractionConst * fmax(distance,20);
			attraction[0] = coefficent * cos(angle);
			attraction[1] = coefficent * sin(angle);
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


	void calculate_repulsion(){
		vector<obstacle_t> obstacles;
		get_obstacles(obstacles);
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
				double repulseScalar = 0.2;
				double tanScalar = 0.3;
				Node node, dir, tang;
				double dirX = 0, dirY = 0;
				for(int k=0;k<points.size();k++){
					double repulseMag = repulseScalar * (repulseDistance -distancePoints(i,j,points[k]));
					double tanMag = tanScalar * (repulseMag/repulseScalar);
					// cout << repulseMag << " " << tanMag << endl;
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
	
	int Close() {
		close(sd);
		return 0;
	}	

	tank_t get_tank(int index){
		vector<tank_t> myTanks;
		get_mytanks(myTanks);
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

		double repulsion[2] = {repulseArray[tankX][tankY].x,repulseArray[tankX][tankY].y};
		add_values(attraction, repulsion, pf);
		return false;
	}		

	double ratioed(double diff){
		return diff/M_PI*0.5;
	}

	double calculate_angvel(int index, double target[]){
		tank_t tank = get_tank(index);
		double tankAngle = fmod(tank.angle, M_PI*2);
		double angle = atan2(target[1], target[0]);
		return ratioed(angle - tankAngle);
	} 	

	double calculate_speed(double pf[]){
		double distance = sqrt( pow(pf[0], 2) + pow(pf[1], 2) );
		double speed = distance / 50;
		return fmin(speed, 1); // maximum speed is 1
	} 
	
	void pf_move(int index, bool shootBullet){
		if (get_tank(index).status != "alive")
			return;
		
		//calculate potential field
		double pf[2];
		bool flag;
		bool mission_accomplished = calculate_potential_field(index, pf);
		if(get_tank(index).flag!="-"){
			flag = true;
		}
		if (flag || missionAccomplished)
		{
			missionAccomplished = true;
			setGoal(home[0], home[1]);
			// goalMapping[index] = HOME;
		}else{
			setGoal(index);
		}

		// calculate angular velocity and velocity
		//double angularvel = angleControllers[index]->get_value(latestAngVel[index], calculate_angvel(index, pf));
		double angularvel = calculate_angvel(index, pf);
		angvel(index, angularvel);
		//double velocity = velocityControllers[index]->get_value(latestVelocity[index], calculate_speed(pf));
		double velocity = calculate_speed(pf);
		speed(index, velocity);

		if (shootBullet && !missionAccomplished)
			shoot(index);
	}
};

// Prototypes
double normalize_angle(double angle);
void world_init(BZRC *my_team);
void robot_pre_update();
void robot_update();
void robot_post_update();
