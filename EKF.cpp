/*
* EKF.cpp
*
*  Created on: 2018/12/18
*      Author: Arsenic
*/

#include "EKF.hpp"
using namespace Eigen;

VectorXf EKF::update(VectorXf z, VectorXf u) {
	VectorXf Xt;
	MatrixXf Pt;

	//Prediction step
	MatrixXf F = jacF(X, u);				//calc jacobian F
	MatrixXf Ftr = F.transpose();
	Xt = f(X, u);							//prediction state
	Pt = F * P * Ftr + Q;					//prediction covariance

	//Correction step
	MatrixXf H = jacH(Xt);					//calc jacobian H
	MatrixXf Htr = H.transpose();
	MatrixXf S = (H*Pt*Htr + R);			//Error covariance
	MatrixXf K = Pt * Htr*S.inverse();		//Kalman gain
	MatrixXf y = z - h(Xt);					//Observation error
	Xt = Xt + K * y;						//correction state
	MatrixXf I = MatrixXf::Identity(X.rows(), X.rows());	//unit matrix
	Pt = (I - K*H)*Pt;						//correction covariance

	X = Xt;
	P = Pt;
	return Xt;
}
