#ifndef RTECAT_MOBILEMANIPULATOR_CLIENT_H_
#define RTECAT_MOBILEMANIPULATOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <queue>
#include <sys/mman.h>

#include <sched.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 
#include "xddp_packet.h"
#include "CS_mobile.h"

#undef debug
//QT
// #include <thread>
// #include <pthread.h>
#include <QApplication>
#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QStandardItemModel>
#include <QScreen>

#include "DarkStyle.h"
#include "framelesswindow.h"
#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QResource>
#include <QTextCodec>

#include "Ecat_Master.h"
#include "ServoAxis_Motor.h"
#include <PropertyDefinition.h>
#include <liegroup_robotics.h>

#define NSEC_PER_SEC 			1000000000
unsigned int cycle_ns = 1000000; // 1 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit


// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodLoop = 0, worstLoop = 0;
unsigned int overruns = 0;

double act_max[4] = {0.0,};

// Interface to physical axes
// Mobile
NRMKHelper::ServoAxis_Motor Axis_Motor[MOBILE_DRIVE_NUM];
const int 	 zeroPos_mob[MOBILE_DRIVE_NUM] = {0,0,0,0};
const INT32 	 gearRatio_mob[MOBILE_DRIVE_NUM] = {GEAR_RATIO_18,GEAR_RATIO_18,GEAR_RATIO_18,GEAR_RATIO_18};
const int 	 dirQ_mob[MOBILE_DRIVE_NUM] = {1,1,-1,-1};
const int 	 dirTau_mob[MOBILE_DRIVE_NUM] = {1,1,-1,-1};

NRMKHelper::ServoAxis_Motor Axis_Mobile[MOBILE_DOF_NUM];


// EtherCAT System interface object
Master ecat_master;
Ecat_iServo ecat_iservo[MOBILE_DRIVE_NUM];

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

// Trajectory parameers
double traj_time=0;
int motion=-1;

Mob_JVec des_int;

// Controller Gains
Mob_JVec NRIC_Kp;
Mob_JVec NRIC_Ki;
Mob_JVec NRIC_K_gamma;

Mob_JVec Kp_n;
Mob_JVec Kd_n;
Mob_JVec Ki_n;

// Mobile Jacobian
Wheel_pinvJacobian Jinv_mob;
Wheel_Jacobian J_mob;




#endif  // /* RTECAT_MOBILEMANIPULATOR_CLIENT_H */