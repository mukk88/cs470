#include <iostream>
#include <vector>

class Grid{
private:
	double ** grid;
	int height;
	int width;

public:
	Grid(int h, int w){
		grid = new double*[h];
		for(int i = 0; i < h; ++i) {
		    grid[i] = new double[w];
		    for(int j= 0;j<w;j++){
		    	grid[i][j] = 0;
		    }
		}
		height = h;
		width = w;
	}

	~Grid(){
		for(int i = 0; i < height; ++i) {
		    delete [] grid[i];
		}
		delete [] grid;
	}

	void updateGrid(int x, int y, double value){
		grid[y][x] = value;
	}

	double getValue(int x, int y){
		return grid[y][x];
	}

	void print(){
		cout << height << " " << width << endl;
		for(int i=0;i<height;i++){
			for(int j=0;j<width;j++){
				cout << grid[i][j] << " ";
			}cout << endl;
		}
		cout << "end" << endl;
	}

};