#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
//#include <Eigen/LU>

#define KF_IMAGE_POS_VAR 0.1
#define KF_VEL_NOISE_VAR 0.5  

struct KalmanFilter
{
	// State vector: [x (m), y (m), z(m), vx (m/s), vy (m/s), vz(m/s)]
	Eigen::MatrixXd x;
    // Covariance Matrix
	Eigen::MatrixXd P; 
	
	// Last time update
	ros::Time tStamp;
	// Last measurement
	Eigen::Matrix<double,6,1> y_last;	
	ros::Time t_lastMea;
	// Updated flag
	bool updated;

	// Covariances
	float posX_cov;
	float posY_cov;
	float posZ_cov;
	
	float velX_cov;
	float velY_cov;
	float velZ_cov;

	// Default constructor
	KalmanFilter(void) : x(6,1), P(6,6) 
	{
		x.setZero(6, 1);
		P.setIdentity(6, 6);
		tStamp = ros::Time::now();
		updated = false;

		posX_cov = KF_IMAGE_POS_VAR;
		posY_cov = KF_IMAGE_POS_VAR;
		posZ_cov = KF_IMAGE_POS_VAR;
		
		velX_cov = KF_VEL_NOISE_VAR;
		velY_cov = KF_VEL_NOISE_VAR;
		velZ_cov = KF_VEL_NOISE_VAR;
		
	}
	
	KalmanFilter(const KalmanFilter &data) : x(6,1), P(6,6)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;

		posX_cov = KF_IMAGE_POS_VAR;
		posY_cov = KF_IMAGE_POS_VAR;
		posZ_cov = KF_IMAGE_POS_VAR;
		
		velX_cov = KF_VEL_NOISE_VAR;
		velY_cov = KF_VEL_NOISE_VAR;
		velZ_cov = KF_VEL_NOISE_VAR;
	}

	KalmanFilter(float img_pos_cov_x,float img_pos_cov_y,float img_pos_cov_z, float pred_vel_cov_x, float pred_vel_cov_y, float pred_vel_cov_z) : x(6,1), P(6,6)
	{
		x.setZero(6, 1);
		P.setIdentity(6, 6);
		tStamp = ros::Time::now();
		updated = false;

		posX_cov = img_pos_cov_x;
		posY_cov = img_pos_cov_y;
		posZ_cov = img_pos_cov_z;
		
		velX_cov = pred_vel_cov_x;
		velY_cov = pred_vel_cov_y;
		velZ_cov = pred_vel_cov_z;
	}
	
	KalmanFilter &operator=(const KalmanFilter &data)
	{
		x = data.x;
		P = data.P;
		tStamp = data.tStamp;
		updated = data.updated;
		posX_cov = data.posX_cov;
		posY_cov = data.posY_cov;
		posZ_cov = data.posZ_cov;
		velX_cov = data.velX_cov;
		velY_cov = data.velY_cov;
		velZ_cov = data.velZ_cov;
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
		
		y_last(0,0) = _x;
		y_last(1,0) = _y;
		y_last(2,0) = _z;
		
		// Setup cov matrix
		P.setIdentity(6, 6);
		P(0,0) = posX_cov;
		P(1,1) = posY_cov;
		P(2,2) = posZ_cov;
		P(3,3) = 1.0*1.0;
		P(4,4) = 1.0*1.0;
		P(5,5) = 1.0*1.0;
		
		// Update time stamp
		tStamp = _tStamp;
		updated = false;
	}

	// State prediction, time in seconds 
	void predict(ros::Time pred_time)
	{
		double _dt = double(pred_time.toSec() - tStamp.toSec());
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
		Q(3,3) = velX_cov*_dt*_dt;
		Q(4,4) = velY_cov*_dt*_dt;
		Q(5,5) = velZ_cov*_dt*_dt;

		// Convariance matrix prediction
		P = F*P*F.transpose() + Q;
//		tStamp = tStamp + ros::Duration(_dt);
		tStamp = pred_time;
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
		Q(3,3) = velX_cov*_dt*_dt;
		Q(4,4) = velY_cov*_dt*_dt;
		Q(5,5) = velZ_cov*_dt*_dt;

		// Convariance matrix prediction
		P = F*P*F.transpose() + Q;
		tStamp = tStamp + ros::Duration(_dt);
		updated = false;
	}
	
	// State update for laser data
	void update(double _x, double _y, double _z, ros::Time _t)
	{
		
		double _dt = double(_t.toSec() - t_lastMea.toSec());
		double _vx = (_x - y_last(0,0))/(_dt);		
		double _vy = (_y - y_last(1,0))/(_dt);		
		double _vz = (_z - y_last(2,0))/(_dt);		
		
		_vx = (_vx + y_last(3,0))/2;
		_vy = (_vy + y_last(4,0))/2;
		_vz = (_vz + y_last(5,0))/2;

		t_lastMea = _t;
		y_last(0,0) = _x;
		y_last(1,0) = _y;
		y_last(2,0) = _z;
		y_last(3,0) = _vx;
		y_last(4,0) = _vy;
		y_last(5,0) = _vz;

		// Compute update jacobian
		Eigen::Matrix<double, 6, 6> H;
		H.setZero(6, 6);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		H(2,2) = 1.0;
		H(3,3) = 1.0;
		H(4,4) = 1.0;
		H(5,5) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 6, 6> R;
		R.setZero(6, 6);
		R(0,0) = posX_cov;
		R(1,1) = posY_cov;
		R(2,2) = posZ_cov;
		R(3,3) = posZ_cov*15;
		R(4,4) = posZ_cov*15;
		R(5,5) = posZ_cov*15;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 6, 6> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 6, 6> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 6, 1> y;
		y(0,0) = _x - x(0,0);
		y(1,0) = _y - x(1,0);
		y(2,0) = _z - x(2,0);
		y(3,0) = _vx - x(3,0);
		y(4,0) = _vy - x(4,0);
		y(5,0) = _vz - x(5,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
		P = (I - K*H)*P;
		
		tStamp 	= _t;
		updated = true;
	}
	
};

#endif
