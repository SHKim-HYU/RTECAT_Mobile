#ifndef BULLET_MOBILE_SETUP_H
#define BULLET_MOBILE_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "liegroup_robotics.h"
#include "PropertyDefinition.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace Eigen;
using namespace lr;
class Bullet_Mobile
{
	int robotId;
	int actuated_joint_num;
	int eef_num;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
		
public:

	Bullet_Mobile(class b3RobotSimulatorClientAPI* sim, int robotId);
	void set_torque(Mob_JVec  torques ,Mob_JVec  max_torques );
	Mob_JVec get_q();	
	Mob_JVec get_qdot();	
	SE3 get_eef_pose();	
	void reset_q(Mob_JVec q);
	Vector6d get_FT();
	void apply_ext_FT(Mob_JVec FT);
	int get_actuated_joint_num(){
		return this->actuated_joint_num;
	};
	
	virtual ~Bullet_Mobile();
private:
	class b3RobotSimulatorClientAPI* sim;
};
#endif  //BULLET_MOBILE_SETUP_H