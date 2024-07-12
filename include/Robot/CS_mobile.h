#ifndef CS_MOBILE_H
#define CS_MOBILE_H

#include "iostream"
#include "json_loader.h"
// #include "modern_robotics.h"
#include <Eigen/Dense>
// #include <casadi/casadi.hpp>
#include <dlfcn.h>
#include "liegroup_robotics.h"
#include "PropertyDefinition.h"

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

using namespace Eigen;
using namespace std;
using namespace lr;

class CS_Mobile  {
public:
	CS_Mobile();
	
	// ~CS_Mobile(){
	// 	// Free the handle
    // 	dlclose(fd_handle);
    // 	dlclose(M_handle);
    // 	dlclose(Minv_handle);
    // 	dlclose(C_handle);
    // 	dlclose(G_handle);
    // 	dlclose(J_s_handle);
    // 	dlclose(J_b_handle);
    // 	dlclose(FK_handle);
	// };

	// JsonLoader loader_;

	void CSSetup(const string& _modelPath, double _period);
	void setPIDgain(Mob_JVec _Kp, Mob_JVec _Kd, Mob_JVec _Ki);
	void setHinfgain(Mob_JVec _Hinf_Kp, Mob_JVec _Hinf_Kd, Mob_JVec _Hinf_Ki, Mob_JVec _Hinf_K_gamma);
	void setNRICgain(Mob_JVec _NRIC_Kp, Mob_JVec _NRIC_Ki, Mob_JVec _NRIC_K_gamma);

	void updateRobot(Mob_JVec _q, Mob_JVec _dq);

	Mob_JVec computeFD(Mob_JVec _q, Mob_JVec _dq, Mob_JVec _tau);
	void computeRK45(Mob_JVec _q, Mob_JVec _dq, Mob_JVec _tau, Mob_JVec &_q_nom, Mob_JVec &_dq_nom, Mob_JVec &_ddq_nom);

	Mob_MassMat computeM(Mob_JVec _q);
	Mob_MassMat computeMinv(Mob_JVec _q);
	Mob_MassMat computeC(Mob_JVec _q, Mob_JVec _dq);
	Mob_JVec computeG(Mob_JVec _q);

	Mob_Jacobian computeJ_b(Mob_JVec _q);
	Mob_Jacobian computeJ_s(Mob_JVec _q);

	SE3 computeFK(Mob_JVec _q);

	Mob_MassMat getM();
	Mob_MassMat getMinv();
	Mob_MassMat getC();
	Mob_JVec getG();

	SE3 getFK();
	SO3 getRMat();

	Mob_Jacobian getJ_b();
	Mob_Jacobian getJ_s();

	Twist getBodyTwist();

	Mob_JVec ComputedTorqueControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des);
	Mob_JVec ComputedTorqueControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des, Mob_JVec _tau_ext);
	
    Mob_JVec HinfControl(Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des);
	Mob_JVec HinfControl(Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des, Mob_JVec _tau_ext);
	Mob_JVec NRIC(Mob_JVec q_r, Mob_JVec dq_r, Mob_JVec q_n, Mob_JVec dq_n);
	void computeAlpha(Mob_JVec edot, Mob_JVec tau_c);

private:
	Mob_JVec q, dq, ddq;
	Mob_JVec tau, tau_bd, tau_ext, ddq_res;
	Mob_JVec e, eint;
	Mob_MassMat M, Minv, C;
	Mob_JVec G;

	SE3 T_M;
	SE3 T_ee;
	SO3 R_ee;
	Mob_ScrewList Slist, Blist;
	Mob_Jacobian J_b, J_s;

	Twist V_b, V_s;
	Twist V_dot;
	Twist lambda, lambda_dot, lambda_int;



private:
	bool isUpdated = false;
	string robotModel;
	int n_dof;
	double period;

	void* FD_handle;
	void* M_handle;
	void* Minv_handle;
	void* C_handle;
	void* G_handle;
	void* J_s_handle;
	void* J_b_handle;
	void* FK_handle;
	
	eval_t FD_eval;
	eval_t M_eval;
	eval_t Minv_eval;
	eval_t C_eval;
	eval_t G_eval;
	eval_t J_s_eval;
	eval_t J_b_eval;
	eval_t FK_eval;


    Mob_JMat Kp;
    Mob_JMat Kv;
    Mob_JMat Ki;

    Mob_JMat Hinf_Kp;
    Mob_JMat Hinf_Kv;
    Mob_JMat Hinf_Ki;
    Mob_JMat Hinf_K_gamma;

	Mob_JMat NRIC_Kp;
    Mob_JMat NRIC_Ki;
    Mob_JMat NRIC_K_gamma;

	double alpha = 0.0;

};
#endif // CS_Mobile_H