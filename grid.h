#pragma once
#include <iostream>
#include <vector>
#include <fstream>

class Grid{
private:
	double ** grid;
	int height;
	int width;
	double truePos;
	double trueNeg;

public:
	Grid(int h, int w, double p, double n){
		grid = new double*[h];
		for(int i = 0; i < h; ++i) {
		    grid[i] = new double[w];
		    for(int j= 0;j<w;j++){
		    	grid[i][j] = 0.75;
		    }
		}
		height = h;
		width = w;
		truePos = p;
		trueNeg = n;
	}

	~Grid(){
		for(int i = 0; i < height; ++i) {
		    delete [] grid[i];
		}
		delete [] grid;
	}

	void setValue(int x, int y, double value){
		if (x >= 400 && y >= 400){
			grid[y+399][x+399] = value;
		}
		if (x >= 400){
			grid[y+400][x+399] = value;
		}
		else if (y >= 400){
			grid[y+399][x+400] = value;
		}
		else {
			grid[y+400][x+400] = value;
		}
	}

	double getValue(int x, int y){
		if (x >= 400 && y >= 400){
			return grid[y+399][x+399];
		}
		if (x >= 400){
			return grid[y+400][x+399];
		}
		else if (y >= 400){
			return grid[y+399][x+400];
		}
		else {
			return grid[y+400][x+400];
		}
	}

	double getTruePos() {return truePos;}
	double getTrueNeg() {return trueNeg;}

	void print(){
		ofstream f;
		f.open("output.txt");
		f << height << " " << width << endl;
		for(int i=0;i<height;i++){
			for(int j=0;j<width;j++){
				f << grid[i][j] << " ";
			}f << endl;
		}
		f << "end" << endl;
		f.close();
	}

};