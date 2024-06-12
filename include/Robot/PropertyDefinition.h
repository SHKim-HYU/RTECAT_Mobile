/*! 
 *  @file PropertyDefinition.h
 *  @brief header for property definition
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Oct. 26. 2023
 *  @Comm
 */


#pragma once

#include <Eigen/Dense>

#define CONTROL_FREQ 1000
#define ROBOT_DOF 4
#define OFFSET_NUM 1
#define MOBILE_DRIVE_NUM 4
#define MOBILE_DOF_NUM 3

// 3. Mechanical
#define GEAR_RATIO_18 18

#define WHEEL_RADIUS 0.076 // [m]
#define BASE_l 0.41 // [m]
#define BASE_w 0.31 // [m]

#define EFFICIENCY 85.0 // Gear efficiency


// 4. Type Definition
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 4, 4> se3;
typedef Eigen::Matrix<double, 3, 3> so3;
typedef Eigen::Matrix<double, 6, 1> Twist;
typedef Eigen::Matrix<double, 6, 1> Vector6d;   
typedef Eigen::Matrix<float, 6, 1> Vector6f;  
typedef Eigen::Matrix<double, 3, 1> Vector3d;   
typedef Eigen::Matrix<double, 4, 1> Vector4d;  
typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
typedef Eigen::Matrix<double, 3, 3> Matrix3d;  

typedef Eigen::Matrix<double, 3, MOBILE_DRIVE_NUM> Wheel_Jacobian;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM,3 > Wheel_pinvJacobian;
typedef Eigen::Matrix<double, MOBILE_DRIVE_NUM, 1> Wheel_JVec;

typedef Eigen::Matrix<double, MOBILE_DOF_NUM, 1> Mob_JVec;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM, MOBILE_DOF_NUM> Mob_JMat;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM> Mob_Jacobian;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM,6> Mob_pinvJacobian;
typedef Eigen::Matrix<double, 3*MOBILE_DOF_NUM, MOBILE_DOF_NUM> Mob_DerivativeJacobianVec;
typedef Eigen::Matrix<double, 3*MOBILE_DOF_NUM, 1> Mob_vecJVec;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM, MOBILE_DOF_NUM> Mob_Matrixnd;  
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM> Mob_Matrix6xn;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM> Mob_ScrewList;
typedef Eigen::Matrix<double, 6, MOBILE_DOF_NUM+1> Mob_Matrix6xn_1;
typedef Eigen::Matrix<double, MOBILE_DOF_NUM, MOBILE_DOF_NUM> Mob_MassMat;

// Robot Struct
// Mobile
typedef struct WHEEL_STATE{
	Wheel_JVec q;
	Wheel_JVec q_dot;
	Wheel_JVec q_ddot;
	Wheel_JVec tau;
	Wheel_JVec tau_ext;

	SE3 	 T;                           //Task space
	SO3		 R;
	Vector3d x;                           
	Vector3d x_dot;
	Vector3d x_ddot;	
	Vector3d e;
	Vector3d eint;
	Vector3d edot;
	
	Vector3d F;
	Vector3d F_CB;
    Vector3d F_ext;
    
    double s_time;
}wheel_state;

typedef struct WHEEL_MOTOR_INFO{
    double torque_const[MOBILE_DRIVE_NUM];
    double gear_ratio[MOBILE_DRIVE_NUM];
    double rate_current[MOBILE_DRIVE_NUM];
}wheel_Motor_Info;

typedef struct MOB_ROBOT_INFO{
	int Position;
	int q_inc[MOBILE_DRIVE_NUM];
    int dq_inc[MOBILE_DRIVE_NUM];
	int tau_per[MOBILE_DRIVE_NUM];
	int statusword[MOBILE_DRIVE_NUM];
    int modeofop[MOBILE_DRIVE_NUM];

	Wheel_JVec q_target;
	Wheel_JVec qdot_target;
	Wheel_JVec qddot_target;

	Mob_JVec x_target;
	Mob_JVec xdot_target;
	Mob_JVec xddot_target;

	unsigned int idx;

	WHEEL_STATE act;
	WHEEL_STATE des;
	WHEEL_STATE nom;

    WHEEL_MOTOR_INFO motor;
}MOB_ROBOT_INFO;

