#ifndef KALMANAGENT_H
#define KALMANAGENT_H
#include "command.h"
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <string>
#include <iterator>
#include <fstream>
#include <iostream>

using Eigen::MatrixXd;

class KalmanAgent {

public:

	KalmanAgent(int i, BZRC* bzrc, double changeInT, double friction, string c) 
	: index(i), commandCenter(bzrc), color(c) {
		MatrixXd u2(6,1);
		u2 << 0, 0, 0, 0, 0, 0;
		u = u2;
		MatrixXd Sigmat2(6,6);
		Sigmat2 << 100, 0, 0, 0, 0, 0, 
			0, .1, 0, 0, 0, 0, 
			0 , 0, .1, 0, 0, 0, 
			0, 0, 0, 100, 0, 0, 
			0, 0, 0, 0, .1, 0, 
			0, 0, 0, 0, 0, .1;
		Sigmat = Sigmat2;
		MatrixXd F2(6,6);
		F2 << 1, changeInT, (pow(changeInT, 2) / 2), 0, 0, 0,
			0, 1, changeInT, 0, 0, 0,
			0, (-1 * friction), 1, 0, 0, 0,
			0, 0, 0, 1, changeInT, (pow(changeInT, 2) / 2),
			0, 0, 0, 0, 1, changeInT,
			0, 0, 0, 0, (-1 * friction), 1;
		F = F2;
		Ftranspose = F.transpose();
		MatrixXd Sigmax2(6,6);
		Sigmax2 << .1, 0, 0, 0, 0, 0, 
			0, .1, 0, 0, 0, 0, 
			0 , 0, 100, 0, 0, 0, 
			0, 0, 0, .1, 0, 0, 
			0, 0, 0, 0, .1, 0, 
			0, 0, 0, 0, 0, 100;
		Sigmax = Sigmax2;
		MatrixXd H2(2, 6);
		H2 << 1, 0, 0, 0, 0, 0, 
			0, 0, 0, 1, 0, 0;
		H = H2;
		Htranspose = H.transpose();
		MatrixXd Sigmaz2(2, 2);
		Sigmaz2 << 25, 0, 0, 25;
		Sigmaz = Sigmaz2;
	}

	MatrixXd GetKtplus1(){
		//(FΣtFT + Σx)HT(H(FΣtFT + Σx)HT + Σz) − 1
		MatrixXd temp = (F*Sigmat*Ftranspose + Sigmax);
		return temp * Htranspose * 
			(H * temp * Htranspose + Sigmaz).inverse();

	}

	void update(){
		double x, y;
		std::vector <otank_t> otherTanks;
		commandCenter->get_othertanks(&otherTanks);
		for (int i = 0; i < otherTanks.size(); ++i){
			if (color.compare(otherTanks[i].color) == 0){
				x = otherTanks[i].pos[0];
				y = otherTanks[i].pos[1];
				break;
			}
		}
		MatrixXd z(2, 1);
		z << x, y;
		MatrixXd Ktplus1 = GetKtplus1();
		updateMean(Ktplus1, z);
		updateError(Ktplus1);

		ofstream myfile;
		string fileName = "values-" + color + ".dat";
		myfile.open (fileName.c_str());
		//row sigmax sigmay x y
		myfile << 0 << " " << Sigmat(0,0) << " " << Sigmat(3,3) << " " << 
			u(0,0) << " " << u(3,0);
		myfile.close();
	}

	void updateMean(MatrixXd Ktplus1, MatrixXd z){
		//μt + 1 = Fμt + Kt + 1(zt + 1 − HFμt)
		u = F*u + Ktplus1*(z - H*F*u);
	}

	void updateError(MatrixXd Ktplus1){
		//Σt + 1 = (I − Kt + 1H)(FΣtFT + Σx) 
		Sigmat = (Eigen::Matrix<double, 6, 6>::Identity() - 
			Ktplus1*H)*(F*Sigmat*Ftranspose + Sigmax);
	}

private:
	string color;
	int index;
	BZRC* commandCenter;
	MatrixXd Ftranspose;
	MatrixXd Htranspose;
	MatrixXd F;
	MatrixXd Sigmat;
	MatrixXd Sigmax;
	MatrixXd Sigmaz;
	MatrixXd H;
	MatrixXd u;
};

#endif