#pragma once

#include <Eigen/Dense>

class Kalman
{

	public:
		Kalman(int n,
		      const Eigen::MatrixXd& state_transition, 
		      const Eigen::MatrixXd& measurement_noise, 
		      const Eigen::MatrixXd& process_noise,
		      const Eigen::MatrixXd& measurement_jacobian,
		      const Eigen::MatrixXd& control_mat,
		      const Eigen::VectorXd& control_vector);

		void SetInitialState(Eigen::VectorXd x0, Eigen::MatrixXd p0) { _x = x0; _P = p0; }

		//Sends a measurement so we can begin prediction process
		void SetMeasurementNoise(const Eigen::VectorXd& noise);
		void SetMeasurement(const Eigen::VectorXd& z);
		void SetControlVector(const Eigen::VectorXd& u);
		
		//Getters
		const Eigen::VectorXd& GetState() const {return _x;}
		const Eigen::MatrixXd& GetCovariance() const {return _P;}
		const Eigen::VectorXd& GetMeasurement() const {return _z;}
		const Eigen::VectorXd& GetControlVector() const {return _u;}

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
		const Eigen::MatrixXd& _A;    //State transition
		const Eigen::MatrixXd& _Q;    //Process noise covariance
		const Eigen::MatrixXd& _R;    //Measurement noise covariance
		const Eigen::MatrixXd& _H;    //Measurement Jacobian
		const Eigen::MatrixXd& _B;    //Control mat
		
		Eigen::VectorXd _x;            //state
		Eigen::MatrixXd _P;            //Covariance
		Eigen::MatrixXd _K;            //Kalman Gain
		Eigen::VectorXd _z;            //Measurement vector
		Eigen::VectorXd _u;            //Control vector		
};