/*
* EKF.h
*
*  Created on: 2018/12/18
*      Author: Arsenic
*/

#ifndef EKF_HPP_
#define EKF_HPP_
#include "Eigen/core"
#include "Eigen/LU"
using namespace Eigen;

class EKF {
private:

public:
	//constructor
	EKF() {
		initialX();
		initialPreP();
		initialQ();
		initialR();
	}

	/**
	* Calculation newest estimation state Xt
	* Matrix z : measurement vector
	* Matrix u : control vector
	* return : estimation state vector
	*/
	VectorXf update(VectorXf z, VectorXf u);

	/* USER DEFINITION */

	//Cycle[s]
	float dt = 0.001f;

	//state X
	VectorXf X;
	void initialX() {	//Define initial state here
		VectorXf m(1);
		m <<
			0;
		X = m;

	}


	//state covariance
	MatrixXf P;
	void initialPreP() {		//Define initial state covariance
		MatrixXf m(1, 1);
		m <<
			0.0;
		P = m;
	}

	//Control covariance
	MatrixXf Q;
	void initialQ() {		//Define Control covariance here
		MatrixXf m(1, 1);
		m <<
			0.0;
		Q = m;
	}

	//Measurement covariance
	MatrixXf R;
	void initialR() {		//Define Measurement covariance here
		MatrixXf m(1, 1);
		m <<
			0.0;
		R = m;
	}

	//Equation of state f
	VectorXf f(VectorXf preX, VectorXf u) {
		VectorXf x;
		return x;
	}

	//Jacobian matrix F
	MatrixXf jacF(VectorXf x, VectorXf u) {
		MatrixXf jF;
		return jF;
	}

	//Equation of output h
	VectorXf h(VectorXf x) {
		VectorXf z;
		return z;
	}

	//Jacobian matrix H
	MatrixXf jacH(VectorXf x) {
		MatrixXf jH;
		return jH;
	}

	/* USER DEFINITION */
};

#endif /* EKF_HPP_ */
