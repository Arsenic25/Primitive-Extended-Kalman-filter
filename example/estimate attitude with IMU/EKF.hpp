/*
* EKF.h
*
*  Created on: 2018/12/18
*      Author: Arsenic
*/

#ifndef EKF_HPP_
#define EKF_HPP_
#define PI 3.1415926535

#include "Eigen/Core.h"
#include "Eigen/LU.h"
#include "math.h"
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
	
	/*---define rotation matrix---*/
	MatrixXf Rx(float dig) {
		float rad = dig*(PI/180.0f);
		MatrixXf m(3, 3);
		m <<
			1.0f, 0.0f, 0.0f,
			0.0f, cos(rad), -sin(rad),
			0.0f, sin(rad), cos(rad);
		return m;
	}
	
	MatrixXf Ry(float dig) {
		float rad = dig*(PI/180.0f);
		MatrixXf m(3, 3);
		m <<
			cos(rad), 0.0f, sin(rad),
			0.0f, 1.0f, 0.0f,
			-sin(rad), 0.0f, cos(rad);
		return m;
	}
	/*---define rotation matrix---*/
	
	/**
	* Calculation newest estimation state Xt
	* Matrix z : measurement vector
	* Matrix u : control vector
	* return : estimation state vector
	*/
	VectorXf update(VectorXf z, VectorXf u);

	/* USER DEFINITION */

	//Cycle[s]
	float dt;
	
	//state X
	VectorXf X;
	void initialX() {	//Define initial state here
		VectorXf m(3);
		m <<
			0.0f,
			0.0f,
			0.0f;
		X = m;

	}

	//state covariance
	MatrixXf P;
	void initialPreP() {		//Define initial state covariance
		MatrixXf m(3, 3);
		m <<
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f;
		P = m;
	}

	//Control covariance
	MatrixXf Q;
	void initialQ() {		//Define Control covariance here
		MatrixXf m(3, 3);
		m <<
			0.001f, 0.0f, 0.0f,
			0.0f, 0.001f, 0.0f,
			0.0f, 0.0f, 0.001f;
		Q = m;
	}

	//Measurement covariance
	MatrixXf R;
	void initialR() {		//Define Measurement covariance here
		MatrixXf m(3, 3);
		m <<
			0.005071526f, 0.0f, 0.0f,
			0.0f, 0.007402357f, 0.0f,
			0.0f, 0.0f, 0.014450039f;
		R = m;
	}

	//Equation of state f
	VectorXf f(VectorXf preX, VectorXf u) {
		VectorXf x;
		x = preX + dt*Ry(preX(1))*Rx(preX(0))*u;
		return x;
	}

	//Jacobian matrix F
	MatrixXf jacF(VectorXf x, VectorXf u) {
		VectorXf rad = (PI/180.0f)*x;
		MatrixXf jF(3, 3);
		jF <<
			1.0f+dt*(cos(rad(0))*sin(rad(1))*u(1)-sin(rad(0))*sin(rad(1))*u(2)), dt*(-sin(rad(1))*u(0)+sin(rad(0))*cos(rad(1))*u(1)+cos(rad(0))*cos(rad(1))*u(2)), 0.0f,
			dt*(-sin(rad(0))*u(1)), 1+dt*(-cos(rad(1))*u(2)), 0.0f,
			dt*(cos(rad(0))*cos(rad(1))*u(1)-sin(rad(0))*cos(rad(1))*u(2)), dt*(-cos(rad(1))*u(0)-sin(rad(0))*sin(rad(1))*u(1)-cos(rad(0))*sin(rad(1))*u(2)), 1.0f;
		return jF;
	}

	//Equation of output h
	VectorXf h(VectorXf x) {
		VectorXf z;
		VectorXf gVec(3);
		gVec <<
			0,
			0,
			1;
		z = (Rx(x(0)).transpose())*(Ry(x(1)).transpose())*gVec;
		return z;
	}

	//Jacobian matrix H
	MatrixXf jacH(VectorXf x) {
		VectorXf rad = (PI/180.0f)*x;
		MatrixXf jH(3, 3);
		jH <<
			0.0f, -cos(rad(1)), 0.0f,
			cos(rad(0))*cos(rad(1)), -sin(rad(0))*sin(rad(1)), 0.0f,
			-sin(rad(0))*cos(rad(1)), -cos(rad(0))*sin(rad(1)), 0.0f;
		return jH;
	}

	/* USER DEFINITION */
};

#endif /* EKF_HPP_ */

