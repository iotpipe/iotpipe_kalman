#pragma once

#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/vector.hpp"
#include <boost/numeric/ublas/operation.hpp>
#include "helpers.h"

template<typename T>
using vec = boost::numeric::ublas::vector<T>;
template<typename T>
using mat = boost::numeric::ublas::matrix<T>;


using boost::numeric::ublas::axpy_prod;
using boost::numeric::ublas::trans;

template <class T>
class Kalman
{

	public:
		Kalman(int n,
		      const mat<T>& state_transition, 
		      const mat<T>& measurement_noise, 
		      const mat<T>& process_noise,
		      const mat<T>& measurement_jacobian,
		      const mat<T>& control_mat,
		      const vec<T>& control_vector);

		void SetInitialState(vec<T> x0, mat<T> p0) { _x = x0; _P = p0; }

		//Sends a measurement so we can begin prediction process
		void SetMeasurementNoise(const vec<T>& noise);
		void SetMeasurement(const vec<T>& z);
		void SetControlVector(const vec<T>& u);
		
		//Getters
		const vec<T>& GetState() const {return _x;}
		const mat<T>& GetCovariance() const {return _P;}
		const vec<T>& GetMeasurement() const {return _z;}
		const vec<T>& GetControlVector() const {return _u;}

		void Update();
	
	private:

		//Updates x and P with state transition matrices (does not included measurement information or noise)
		void Predict();

		//Corrects x and P taking into account measurements and noise
		void Correct();

		//Computes the kalman gain
		void UpdateKalmanGain();


	private:

		const int _N; 
		const mat<T>& _A;    //State transition
		const mat<T>& _Q;    //Process noise covariance
		const mat<T>& _R;    //Measurement noise covariance
		const mat<T>& _H;    //Measurement Jacobian
		const mat<T>& _B;    //Control mat
		
		vec<T> _x;            //state
		mat<T> _P;            //Covariance
		mat<T> _K;            //Kalman Gain
		vec<T> _z;            //Measurement vector
		vec<T> _u;            //Control vector		
};

template <class T>
Kalman<T>::Kalman(int n,
	          const mat<T>& state_transition, 
	          const mat<T>& measurement_noise, 
	          const mat<T>& process_noise,
	          const mat<T>& measurement_jacobian,
	          const mat<T>& control_mat,
	          const vec<T>& control_vector) :
_A(state_transition),
_R(measurement_noise),
_Q(process_noise),
_H(measurement_jacobian),
_B(control_mat),
_u(control_vector),
_N(n),
_z(vec<T>(n,0)),
_x(vec<T>(n,0)),
_P(mat<T>(n,n,0))
{
	//do nothing
}

template <class T>
void Kalman<T>::Predict()
{
	_x = prod(_A,_x) + prod(_B,_u);

	//P = A*P*AT + Q
	_P = prod(_A,_P);
	_P = prod(_P,trans(_A));
	_P = _P +  _Q;
}

template <class T>
void Kalman<T>::Correct()
{

	boost::numeric::ublas::identity_matrix<double> I (_N);

	//This update _K
	UpdateKalmanGain();

	//x = x + K*(z-H*x)
	_x += prod( _K, _z-prod(_H,_x) );

	//P = (I-K*H)*_P
	_P = prod(I - prod(_K, _H), _P);
}



template <class T>
void Kalman<T>::UpdateKalmanGain()
{
	mat<T> temp1, temp2, temp3, temp4(_H.size1(), _H.size2());

	//K = P*HT*(H*P*HT + R)^-1
	temp1 = prod(_P, trans(_H));
	temp2 = prod(_H,_P);
	temp3 = prod(temp2, trans(_H));
	
	bool success = Helpers::InvertMatrix<T>( temp3 + _R, temp4); // (H*P*HT+R)^-1

	if(!success)
		throw std::runtime_error("Unable to invert mat.  Cannot apply Kalman filter.");
	
	_K =  prod(temp1, temp4);	
}

template <class T>
void Kalman<T>::SetMeasurementNoise(const vec<T>& noise) 
{ 
	_z=prod(_H,_x) + noise;
	std::cout << "Measurement: " << _z << std::endl;
}

template <class T>
void Kalman<T>::SetMeasurement(const vec<T>& z) 
{ 
	_z=z;
	std::cout << "Measurement: " << _z << std::endl;
}

template <class T>
void Kalman<T>::SetControlVector(const vec<T>& u) 
{ 
	_u=u;
	std::cout << "Control Vector: " << _u << std::endl;
}

template <class T>
void Kalman<T>::Update()
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
