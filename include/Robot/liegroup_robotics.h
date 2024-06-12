/*
 * liegroup_robotics.h
 *
 *  Created on: May 29, 2024
 *      Author: Minchang Sung, Sunhong Kim
 */

#ifndef LIEGROUP_ROBOTICS_H
#define LIEGROUP_ROBOTICS_H
    
#pragma once
#include <iostream>
#include <vector>
#include "PropertyDefinition.h"

namespace lr {

    Matrix6d ad(const Vector6d& V) ;
    Matrix6d Ad(const SE3& T);
	  Matrix6d AdInv(const SE3& T);    
    SE3 TransInv(const SE3& T) ;
    SO3 TransToR(const SE3& T);
    Vector3d TransToP(const SE3& T);
    Vector6d se3ToVec(const se3& T);
    Vector3d so3ToVec(const so3& so3mat);
    so3 VecToso3(const Vector3d& omg);
    se3 VecTose3(const Vector6d& V);
    bool NearZero(const double val) ;
    SO3 MatrixExp3(const so3& so3mat);
    SE3 MatrixExp6(const se3& se3mat);
    so3 MatrixLog3(const SO3& R);
    se3 MatrixLog6(const SE3& T);
    SE3 FKinSpace(const SE3& M, const Mob_ScrewList& Slist, const Mob_JVec& thetaList);
    SE3 FKinBody(const SE3& M, const Mob_ScrewList& Blist, const Mob_JVec& thetaList);
    Mob_Jacobian dJacobianBody(const SE3& M,const Mob_ScrewList& Blist, const Mob_JVec& q ,const Mob_JVec& dq);
    Mob_Jacobian JacobianSpace(const Mob_ScrewList& Slist, const Mob_JVec& thetaList);
    Mob_Jacobian JacobianBody(const Mob_ScrewList& Blist, const Mob_JVec& thetaList) ;
    SE3 RpToTrans(const Matrix3d& R, const Vector3d& p);
    bool IKinBody(const Mob_ScrewList& Blist, const SE3& M, const SE3& T,
		Mob_JVec& thetalist, double eomg, double ev);
    bool IKinSpace(const Mob_ScrewList& Slist, const SE3& M, const SE3& T,
		Mob_JVec& thetalist, double eomg, double ev) ;
    Mob_JVec InverseDynamics(const Mob_JVec& thetalist, const Mob_JVec& dthetalist, const Mob_JVec& ddthetalist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist);
    Mob_JVec InverseDynamics(const Mob_JVec& thetalist, const Mob_JVec& dthetalist, const Mob_JVec& ddthetalist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass);                  
    Mob_JVec GravityForces(const Mob_JVec& thetalist, const Vector3d& g,
                  const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist) ;                                    
    Mob_JVec GravityForces(const Mob_JVec& thetalist, const Vector3d& g,
                  const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass) ;                    
    Mob_MassMat MassMatrix(const Mob_JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist) ;
    Mob_MassMat MassMatrix(const Mob_JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass) ;                          
                                      
    Mob_JVec VelQuadraticForces(const Mob_JVec& thetalist, const Mob_JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist) ;
    Mob_JVec VelQuadraticForces(const Mob_JVec& thetalist, const Mob_JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass) ;
    
    Mob_JVec EndEffectorForces(const Mob_JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist);
    Mob_JVec EndEffectorForces(const Mob_JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass);
    
    Mob_JVec ForwardDynamics(const Mob_JVec& thetalist, const Mob_JVec& dthetalist, const Mob_JVec& taulist,
                  const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
                  const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist);       
               	Mob_JVec ForwardDynamics(const Mob_JVec& thetalist, const Mob_JVec& dthetalist, const Mob_JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const Mob_ScrewList& Slist,double eef_mass);                    

    void EulerStep(Mob_JVec& thetalist, Mob_JVec& dthetalist, const Mob_JVec& ddthetalist, double dt);                                    
    Vector3d QuinticTimeScalingKinematics(double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) ;
    void FKinBody(const SE3& M,const Mob_ScrewList& Blist, const Mob_JVec& q ,const Mob_JVec& dq, SE3 &T, Mob_Jacobian &Jb,Mob_Jacobian& dJb);
    Matrix3d dexp3(const Vector3d& xi);
    Matrix3d dlog3(const Vector3d& xi);
    Matrix6d dexp6(const Vector6d& lambda);
    Matrix3d ddexp3(const Vector3d& xi, const Vector3d& dxi);
    Matrix3d dddexp3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
    Matrix6d ddexp6(const Vector6d& lambda, const Vector6d& lambda_dot);
    Matrix3d skew_sum(const Vector3d& a, const Vector3d& b);
    Matrix3d ddlog3(const Vector3d& xi, const Vector3d& dxi);
    Matrix3d dddlog3(const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy);
    Matrix6d dlog6(const Vector6d& lambda);
    Matrix6d ddlog6(const Vector6d& lambda, const Vector6d& lambda_dot) ;    
    //void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N,std::vector<SE3>&Xd_list,std::vector<Vector6d>&Vd_list,std::vector<Vector6d>&dVd_list);
    void LieScrewTrajectory(const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,double t,SE3& T_des,Vector6d& V_des,Vector6d& V_des_dot);
    void JointTrajectory(const Mob_JVec q0, const Mob_JVec qT, double Tf, double t , int method , Mob_JVec& q_des, Mob_JVec& q_dot_des, Mob_JVec& q_ddot_des) ;
    // SE3 RelFKinSpace(const SE3& M, const RelMob_ScrewList& Slist, const RelMob_JVec& thetaList);
    // SE3 RelFKinBody(const SE3& M, const RelMob_ScrewList& Blist, const RelMob_JVec& thetaList);
    // RelMob_Jacobian RelMob_JacobianSpace(const RelMob_ScrewList& Slist, const RelMob_JVec& thetaList) ;
    // RelMob_Jacobian RelMob_JacobianBody(const RelMob_ScrewList& Blist, const RelMob_JVec& thetaList);
    // RelMob_Jacobian ReldMob_JacobianBody(const SE3& M,const RelMob_ScrewList& Blist, const RelMob_JVec& q ,const RelMob_JVec& dq);
    // bool RelIKinSpace(const RelMob_ScrewList& Slist, const SE3& M, const SE3& T,RelMob_JVec& thetalist, double eomg, double ev);
    // bool RelIKinBody(const RelMob_ScrewList& Blist, const SE3& M, const SE3& T,RelMob_JVec& thetalist, double eomg, double ev);
}


#endif
