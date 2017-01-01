#include "kalman.h"
#include <iostream>

Kalman::Kalman(int n,
	          const Eigen::MatrixXd& state_transition, 
	          const Eigen::MatrixXd& measurement_noise, 
	          const Eigen::MatrixXd& process_noise,
	          const Eigen::MatrixXd& measurement_jacobian,
	          const Eigen::MatrixXd& control_mat,
	          const Eigen::VectorXd& control_vector) :
_A(state_transition),
_R(measurement_noise),
_Q(process_noise),
_H(measurement_jacobian),
_B(control_mat),
_u(control_vector),
_N(n),
_z(n),
_x(n),
_P(n,n)
{
	//do nothing
}

void Kalman::Predict()
{
    //x = A*x + B*u
	_x = _A*_x + _B*_u;

	//P = A*P*AT + Q
	_P = _A*_P*_A.transpose() + _Q;
}

void Kalman::Correct()
{

	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_N,_N);

	//This update _K
	UpdateKalmanGain();

	//x = x + K*(z-H*x)
	_x = _x + _K * ( _z - _H * _x );

	//P = (I-K*H)*_P
	_P = ( I - _K * _H ) * _P;
}

void Kalman::UpdateKalmanGain()
{
	bool isInvertible = (_H*_P*_H.transpose() + _R).determinant()!=0;

	if(!isInvertible)
		throw std::runtime_error("Unable to invert mat.  Cannot apply Kalman filter.");

	//K = P*HT*(H*P*HT + R)^-1
	_K = _P*_H.transpose()*(_H*_P*_H.transpose() + _R).inverse();	
}

void Kalman::SetMeasurementNoise(const Eigen::VectorXd& noise) 
{ 
	_z=_H*_x + noise;
	std::cout << "Measurement: " << _z << std::endl;
}

void Kalman::SetMeasurement(const Eigen::VectorXd& z) 
{ 
	_z=z;
	std::cout << "Measurement: " << _z << std::endl;
}

void Kalman::SetControlVector(const Eigen::VectorXd& u) 
{ 
	_u=u;
	std::cout << "Control Vector: " << _u << std::endl;
}

void Kalman::Update()
{
	Predict();
	std::cout << "x_: " << _x << std::endl;
	std::cout << "p_: " << _P << std::endl;
	Correct(); 
	std::cout << "K: " << _K << std::endl;
	std::cout << "p+: " << _P << std::endl;
	std::cout << "x+: " << _x << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
}