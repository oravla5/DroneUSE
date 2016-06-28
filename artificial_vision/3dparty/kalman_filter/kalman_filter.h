#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
//#include <Eigen/LU>

#define KF_IMAGE_POS_VAR 0.01
#define KF_VEL_NOISE_VAR 0.3  

//Anonymous namespace is used to avoid multiple definition compiler errors when calling KalmanFilter static functions from other files
namespace{
struct KalmanFilter
{
	// State vector: [x (m), y (m), z(m), vx (m/s), vy (m/s), vz(m/s)]
	Eigen::MatrixXd x;
    // Covariance Matrix
	Eigen::MatrixXd P; 
	
	// Last time update
	ros::Time tStamp;
	
	// Updated flag
	bool updated;

	
	// Default constructor
	KalmanFilter(void) : x(6,1), P(6,6) 
	{
		x.setZero(6, 1);
		P.setIdentity(6, 6);
		tStamp = ros::Time::now();
		updated = false;
	}
	
	KalmanFilter(const KalmanFilter &data) : x(6,1), P(6,6)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;
	}
	
	KalmanFilter &operator=(const KalmanFilter &data)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;
		return *this;
	}
	
	// Filter initialization, position in m and velocity on m/s 
	void init(double _x, double _y, double _z, double _vx, double _vy, double _vz, ros::Time _tStamp)
	{

		// Setup state vector
		x.setZero(6, 1);
		x(0,0) = _x;
		x(1,0) = _y;
		x(2,0) = _z;
		x(3,0) = _vx;
		x(4,0) = _vy;
		x(5,0) = _vz;
		
		// Setup cov matrix
		P.setIdentity(6, 6);
		P(0,0) = KF_IMAGE_POS_VAR;
		P(1,1) = KF_IMAGE_POS_VAR;
		P(2,2) = KF_IMAGE_POS_VAR;
		P(3,3) = 1.0*1.0;
		P(4,4) = 1.0*1.0;
		P(5,5) = 1.0*1.0;
		
		// Update time stamp
		tStamp = _tStamp;
		updated = false;
	}

	// State prediction, time in seconds 
	void predict(double _dt)
	{
		// State vector prediction
		x(0,0) += x(3,0)*_dt;
		x(1,0) += x(4,0)*_dt;
		x(2,0) += x(5,0)*_dt;
		
        // State Matrix
		Eigen::Matrix<double, 6, 6> F;
		F.setIdentity(6, 6);
		F(0,3) = _dt;
		F(1,4) = _dt;
		F(2,5) = _dt;
        
        // Estimation Error Matrix
		Eigen::Matrix<double, 6, 6> Q;
		Q.setZero(6, 6);
		Q(3,3) = KF_VEL_NOISE_VAR*_dt*_dt;
		Q(4,4) = KF_VEL_NOISE_VAR*_dt*_dt;
		Q(5,5) = KF_VEL_NOISE_VAR*_dt*_dt;

		// Convariance matrix prediction
		P = F*P*F.transpose() + Q;
		
		updated = false;
	}
	
	// State update for laser data
	void update(double _x, double _y, double _z, ros::Time _t)
	{
		// Update time stamp
		tStamp = _t;
		
		// Compute update jacobian
		Eigen::Matrix<double, 3, 6> H;
		H.setZero(3, 6);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		H(2,2) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 3, 3> R;
		R.setZero(3, 3);
		R(0,0) = KF_IMAGE_POS_VAR;
		R(1,1) = KF_IMAGE_POS_VAR;
		R(2,2) = KF_IMAGE_POS_VAR;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 3, 3> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 6, 3> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 3, 1> y;
		y(0,0) = _x - x(0,0);
		y(1,0) = _y - x(1,0);
		y(2,0) = _z - x(2,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
		P = (I - K*H)*P;
		
		updated = true;
	}
	
};

}
#endif
