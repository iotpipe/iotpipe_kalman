#ifndef KALMAN_H_
#define KALMAN_H_

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
		void SendMeasurement(const vec<T>& z);
		const vec<T>& GetState() {return _x;}
		const mat<T>& GetCovariance() {return _P;}
	
	private:

		//Updates x and P with state transition matrices (does not included measurement information or noise)
		void Predict();

		//Corrects x and P taking into account measurements and noise
		void Correct();

		//Computes the kalman gain
		const mat<T> UpdateKalmanGain() const;


	private:

		const int _N; 
		const mat<T>& _A;    //State transition
		const mat<T>& _Q;    //Process noise covariance
		const mat<T>& _R;    //Measurement noise covariance
		const mat<T>& _H;    //Measurement Jacobian
		const mat<T>& _B;    //Control mat
		const vec<T>& _u;       //Control vector
		
		vec<T> _x;             //state
		mat<T> _P;            //Covariance
		vec<T> _z;            //Measurement vector		
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

	//This update K
	const mat<T> K = UpdateKalmanGain();

	//x = x + K*(z-H*x)
	_x += prod( K, _z-prod(_H,_x) );

	//P = (I-K*H)*_P
	_P = prod(I - prod(K, _H), _P);
}



template <class T>
const mat<T> Kalman<T>::UpdateKalmanGain() const
{
	mat<T> temp1(_N,_N), temp2(_N,_N), temp3(_N,_N), temp4(_N,_N);

	//K = P*HT*(H*P*HT + R)^-1
	temp1 = prod(_P, trans(_H));
	temp2 = prod(_H,_P);
	temp3 = prod(temp2, trans(_H));
	
	bool success = Helpers::InvertMatrix<T>( temp3 + _R, temp4); // (H*P*HT+R)^-1

	if(!success)
		throw std::runtime_error("Unable to invert mat.  Cannot apply Kalman filter.");
	
	return prod(temp1, temp4);	
}

template <class T>
void Kalman<T>::SendMeasurement(const vec<T>& z) 
{ 
	_z=z;
	Predict();
	Correct(); 
}


#endif
