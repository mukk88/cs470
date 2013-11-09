#ifndef OCCGRID_H
#define OCCGRID_H
#include <string>
#include <assert.h>

using namespace std;

class OccGrid{
public:
	OccGrid() : occupancy(NULL), index(0)  {
		
	}

	void updateGrid(int x, int y, int w, int h){
		height = h;
		xStart = x;
		yStart = y;
		width = w;
		occupancy = new bool*[height];
		for(int i = 0; i < height; ++i) {
		    occupancy[i] = new bool[width];
		}
	}

	~OccGrid() {
		if (occupancy != NULL){
			for(int i = 0; i < height; ++i) {
			    delete [] occupancy[i];
			}
			delete [] occupancy;
		}
	}

	void addLine(string line){
		assert(index < height);
		assert(line.length() == width);
		for (int c = 0; c < line.length(); ++c){
			occupancy[index][c] = (line[c] == '1') ? 1 : 0;
		}
		++index;
	}

	void setXStart(int x) { xStart = x;}

	int getXStart() { return xStart; }
	int getYStart() { return yStart; }
	int getWidth() { return width; }
	int getHeight() { return height; }
	bool occupied(int row, int col) { return occupancy[row][col]; }

private:
	int xStart;
	int yStart;
	int width;
	int height;
	bool** occupancy;
	int index;
};

#endif