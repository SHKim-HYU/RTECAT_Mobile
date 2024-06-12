#include "RTECAT_Mobile_Client.h"

MOB_ROBOT_INFO info_mob;

CS_Mobile cs_hyumob;
CS_Mobile cs_nom_hyumob;

RT_TASK safety_task;
RT_TASK motor_task;
RT_TASK print_task;
RT_TASK xddp_writer;

using namespace std;
using namespace lr;


bool isSlaveInit()
{
    // Mobile Drive Servo on
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
    {
        if(!ecat_iservo[i].isSystemReady())
            return false;
    }

    return true;
}

int initAxes()
{
    // Mobile Drive Init Axes
	for (int i = 0; i < MOBILE_DRIVE_NUM; i++)
	{	
		Axis_Motor[i].setGearRatio(gearRatio_mob[i]);
		Axis_Motor[i].setGearEfficiency(EFFICIENCY);
		Axis_Motor[i].setPulsePerRevolution(ecat_master.SDO_ENCODER_RESOLUTION(i+OFFSET_NUM));
		Axis_Motor[i].setTauRateCur(((double)ecat_master.SDO_RATE_CURRENT(i+OFFSET_NUM))/1000.0);
		Axis_Motor[i].setTauK(((double)ecat_master.SDO_TORQUE_CONSTANT(i+OFFSET_NUM))/1000000.0);
		Axis_Motor[i].setZeroPos(zeroPos_mob[i]);

		Axis_Motor[i].setDirQ(dirQ_mob[i]);
		Axis_Motor[i].setDirTau(dirTau_mob[i]);

		Axis_Motor[i].setConversionConstants();

		Axis_Motor[i].setTrajPeriod(period);
		
		Axis_Motor[i].setTarVelInRPM(0);
		Axis_Motor[i].setTarTorInCnt(0);

        info_mob.des.e = Mob_JVec::Zero();
        info_mob.des.eint = Mob_JVec::Zero();
	}


	// Mobile Init Axes
	for (int i = 0; i < MOBILE_DOF_NUM; i++)
	{	
		Axis_Mobile[i].setTrajPeriod(period);
		
		Axis_Mobile[i].setTarVelInCnt(0);
		Axis_Mobile[i].setTarTorInCnt(0);
	}

	// Mecanum Mobile Base Wheel Jacobian
	Jinv_mob << 1.0, 1.0, -BASE_l-BASE_w,
				1.0, -1.0, -BASE_l-BASE_w,
				1.0, 1.0, BASE_l+BASE_w, 
				1.0, -1.0, BASE_l+BASE_w;
	Jinv_mob = Jinv_mob/WHEEL_RADIUS;

	J_mob << 	1.0, 1.0, 1.0, 1.0,
				1.0, -1.0, 1.0, -1.0,
				 1.0/(-BASE_l-BASE_w), 1.0/(-BASE_l-BASE_w), 1.0/(BASE_l+BASE_w), 1.0/(BASE_l+BASE_w);
	J_mob = J_mob*WHEEL_RADIUS/4.0;


	return 1;
}

void readData()
{
	ecat_master.TxUpdate();
    for(int i=0; i<MOBILE_DRIVE_NUM;i++)
    {

        Axis_Motor[i].setCurrentPosInCnt(ecat_iservo[i].position_);
        Axis_Motor[i].setCurrentVelInRPM(ecat_iservo[i].velocity_);
        Axis_Motor[i].setCurrentTorInCnt(ecat_iservo[i].torque_);
        
        Axis_Motor[i].setCurrentTime(gt);

        info_mob.act.q(i) = Axis_Motor[i].getCurrPosInRad();
		if (fabs(Axis_Motor[i].getCurrVelInRad())<10.0)
		{
			info_mob.act.q_dot(i) = Axis_Motor[i].getCurrVelInRad();
		}
		else
		{
			act_max[i] = fabs(Axis_Motor[i].getCurrVelInRad());
		}
        info_mob.act.tau(i) = Axis_Motor[i].getCurrTorInNm();

        // For Inital target
        if(!system_ready)
        {
            Axis_Motor[i].setTarPosInRad(info_mob.act.q(i));
            Axis_Motor[i].setDesPosInRad(info_mob.act.q(i));
        }

    }

	// Mapping status to Mobile Manipulator
	info_mob.act.x_dot = J_mob * info_mob.act.q_dot;
	info_mob.act.x += info_mob.act.x_dot*period;

	for(int i=0; i<MOBILE_DOF_NUM; i++)
	{
		Axis_Mobile[i].setCurrentPosInConf(info_mob.act.x(i));
		Axis_Mobile[i].setCurrentVelInConf(info_mob.act.x_dot(i));

		Axis_Mobile[i].setCurrentTime(gt);
		
		if(!system_ready)
		{
			Axis_Mobile[i].setTarPosInRad(info_mob.act.x(i));
			Axis_Mobile[i].setDesPosInRad(info_mob.act.x(i));
			info_mob.nom.x = info_mob.act.x;
			info_mob.nom.x_dot = info_mob.act.x_dot;
			info_mob.nom.F = info_mob.act.F;
		}
	}
}

/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis_Mobile[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
			info_mob.x_target(0)=0.0; info_mob.x_target(1)=0.0; info_mob.x_target(2)=0.0;
	    	traj_time = 10;
	    	motion++;
			// motion=1;
	        break;
	    case 2:
			info_mob.x_target(0)=0.3; info_mob.x_target(1)=0.0; info_mob.x_target(2)=0.0;
	    	traj_time = 10;
	    	motion++;
			// motion=1;
	        break;
	    case 3:
			info_mob.x_target(0)=0.0; info_mob.x_target(1)=0.0; info_mob.x_target(2)=0.0;
	    	traj_time = 10;
	    	motion++;
	        break;
	    case 4:
			info_mob.x_target(0)=0.0; info_mob.x_target(1)=0.3; info_mob.x_target(2)=0.0;
	    	traj_time = 10;
			motion++;
	    	break;
		case 5:
			info_mob.x_target(0)=0.0; info_mob.x_target(1)=0.0; info_mob.x_target(2)=0.0;
	    	traj_time = 10;
			motion++;
	    	break;
		case 6:
			info_mob.x_target(0)=0.0; info_mob.x_target(1)=0.0; info_mob.x_target(2)=1.5709;
	    	traj_time = 10;
	    	// motion++;
			motion=1;
	        break;

		default:
			info_mob.x_target(0)=info_mob.act.x(0); info_mob.x_target(1)=info_mob.act.x(1); info_mob.x_target(2)=info_mob.act.x(2);
	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<MOBILE_DOF_NUM;i++)
	{
		if(!Axis_Mobile[i].trajInitialized())
		{
			Axis_Mobile[i].setTrajInitialQuintic();
			Axis_Mobile[i].setTarPosInRad(info_mob.x_target(i));
			Axis_Mobile[i].setTarVelInRad(0);
			Axis_Mobile[i].setTrajTargetQuintic(traj_time);
		}

		Axis_Mobile[i].TrajQuintic();

		info_mob.des.x(i)=Axis_Mobile[i].getDesPosInRad();
		info_mob.des.x_dot(i)=Axis_Mobile[i].getDesVelInRad();
		info_mob.des.x_ddot(i)=Axis_Mobile[i].getDesAccInRad();
	}
}

void compute()
{
	// Update Mobile Manipulator
	cs_hyumob.updateRobot(info_mob.act.x, info_mob.act.x_dot);
	// Update nominal
	cs_nom_hyumob.updateRobot(info_mob.nom.x, info_mob.nom.x_dot);
	
	info_mob.act.T = cs_hyumob.getFK();
	info_mob.act.R = cs_hyumob.getRMat();

	Mob_Jacobian J_b = cs_hyumob.getJ_b();

}

void control()
{

    double Kp = 0.1;
    double Kd = 0.01;
	double Ki = 10;
	double Mb = 100;

	// Mobile Base Controller
    for (int i = 0; i<MOBILE_DOF_NUM; i++)
    {
		info_mob.des.e(i) = info_mob.nom.x(i)-info_mob.act.x(i);
		info_mob.des.edot(i) = info_mob.nom.x_dot(i)-info_mob.act.x_dot(i);
		info_mob.des.eint(i) = info_mob.des.eint(i) + info_mob.des.e(i)*period;
		// info_mob.des.F(i) = info_mob.des.x_dot(i) + 0.1*info_mob.des.edot(i) + 1*info_mob.des.e(i) + info_mob.act.F_ext(i)/Mb;

		info_mob.des.F(i) = info_mob.nom.x_dot(i) + Kd*info_mob.des.edot(i) + Kp*info_mob.des.e(i) + Ki*info_mob.des.eint(i);
    }
	info_mob.des.q_dot = Jinv_mob * info_mob.des.F;
	

	// [CTC]
	info_mob.nom.F = cs_nom_hyumob.ComputedTorqueControl(info_mob.nom.x, info_mob.nom.x_dot, info_mob.des.x, info_mob.des.x_dot, info_mob.des.x_ddot);
	// info_mob.nom.F = cs_nom_hyumob.ComputedTorqueControl(info_mob.nom.x, info_mob.nom.x_dot, info_mob.des.x, info_mob.des.x_dot, info_mob.des.x_ddot, info_mm.act.F_ext);
	

	// [Simulation]
	cs_nom_hyumob.computeRK45(info_mob.nom.x, info_mob.nom.x_dot, info_mob.nom.F, info_mob.nom.x, info_mob.nom.x_dot, info_mob.nom.x_ddot);
}

void writeData()
{
    for(int i=0;i<=MOBILE_DRIVE_NUM;i++){
        ecat_iservo[i].writeVelocity(Axis_Motor[i].getDesVelInRPM(info_mob.des.q_dot(i)));
        
		ecat_master.RxUpdate();
	}

    ecat_master.SyncEcatMaster(rt_timer_read());
}

void motor_run(void *arg)
{
    RTIME beginCycle, endCycle;
	RTIME beginCyclebuf;

	beginCyclebuf = 0;
   
    memset(&info_mob, 0, sizeof(MOB_ROBOT_INFO));

	int ft_init_cnt = 0;

	info_mob.des.x = Mob_JVec::Zero();
	info_mob.des.x_dot = Mob_JVec::Zero();
	info_mob.des.x_ddot = Mob_JVec::Zero();
	info_mob.des.F = Vector3d::Zero();

	// Real
	NRIC_Kp << 50.0, 50.0, 50.0;
	NRIC_Ki << 8.0, 8.0, 8.0;
	NRIC_K_gamma << 700.0, 700.0, 700.0;
	cs_hyumob.setNRICgain(NRIC_Kp, NRIC_Ki, NRIC_K_gamma);
	
	// nominal
	Kp_n << 80.0, 80.0, 80.0;
	Kd_n << 55.0, 55.0, 55.0;
	Ki_n = Mob_JVec::Zero();
	cs_nom_hyumob.setPIDgain(Kp_n, Kd_n, Ki_n);

    for(int j=0; j<MOBILE_DRIVE_NUM; ++j)
	{
		ecat_master.addSlaveiServo(0, j+OFFSET_NUM, &ecat_iservo[j]);
		ecat_iservo[j].mode_of_operation_ = ecat_iservo[j].MODE_CYCLIC_SYNC_VELOCITY;
	}
    
    initAxes();

    // Print Motor parameters
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
    {
        rt_printf("rate current: %lf\n", Axis_Motor[i].getTauRateCur());
        rt_printf("torque constant: %lf\n", Axis_Motor[i].getTauK());
        rt_printf("encoder resol: %d\n", Axis_Motor[i].getPulsePerRevolution());
        rt_printf("motor direction: %d\n", Axis_Motor[i].getDirQ());
    }

    ecat_master.activateWithDC(0, cycle_ns);
    
    for (int i=0; i<MOBILE_DRIVE_NUM; i++)
        ecat_iservo[i].setServoOn();
    
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        beginCycle = rt_timer_read();
        // Read Joints Data
        readData();
        if(system_ready)
        {
            // Trajectory Generation
            trajectory_generation();
            
            // Compute KDL
            compute();	

            
            // Controller
            control();
                    
        }

        // Write Joint Data
        writeData();
        
        endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;
		periodLoop = (unsigned long) beginCycle - beginCyclebuf;

        if(isSlaveInit())
		{
			system_ready=true;;	//all drives have been done
		} 
            
        
		if(system_ready)
		{
			gt+= period;
			if (periodCycle > cycle_ns) overruns++;
			if (periodLoop > worstLoop) worstLoop = periodLoop;
		}

        beginCyclebuf = beginCycle;
		rt_task_wait_period(NULL); //wait for next cycle
    }
}

void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

// Safety task
void safety_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;

	rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		
		if (system_ready)
		{
			for(int i=0;i<MOBILE_DRIVE_NUM;i++)
			{
				if(Axis_Motor[i].isLimitReached())
				{
					for(int i=0;i<MOBILE_DRIVE_NUM;i++)
						ecat_iservo[i].setServoOff();
					rt_printf("Servo Off!!\n");
					break;
				}
			}
		}
	}
}

void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
		
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;

			rt_printf("Time=%0.3lfs, cycle_dt=%lius, worst_cycle=%lius, overrun=%d\n", gt, periodCycle/1000, worstLoop/1000, overruns);
			// /*
            rt_printf("Mobile Data\n");
			rt_printf("Des_x: %lf, Des_y: %lf, Des_th: %lf\n", info_mob.des.x(0), info_mob.des.x(1), info_mob.des.x(2));
			rt_printf("Act_x: %lf, Act_y: %lf, Act_th: %lf\n", info_mob.act.x(0), info_mob.act.x(1), info_mob.act.x(2));
			rt_printf("Nom_x: %lf, Nom_y: %lf, Nom_th: %lf\n", info_mob.nom.x(0), info_mob.nom.x(1), info_mob.nom.x(2));
			rt_printf("Des_Vx: %lf, Des_Vy: %lf, Des_Wz: %lf\n", info_mob.des.x_dot(0), info_mob.des.x_dot(1), info_mob.des.x_dot(2));
			rt_printf("Act_Vx: %lf, Act_Vy: %lf, Act_Wz: %lf\n", info_mob.act.x_dot(0), info_mob.act.x_dot(1), info_mob.act.x_dot(2));
			rt_printf("Nom_Vx: %lf, Nom_Vy: %lf, Nom_Wz: %lf\n", info_mob.nom.x_dot(0), info_mob.nom.x_dot(1), info_mob.nom.x_dot(2));
			
			for(int j=0; j<MOBILE_DRIVE_NUM; ++j){
				rt_printf("ID: %d", j);
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info_mob.des.q[j],info_mob.des.q_dot[j],info_mob.des.q_ddot[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info_mob.act.q(j), info_mob.act.q_dot(j));
				rt_printf("\t TarTor: %lf, ActTor: %lf, ExtTor: %lf \n", info_mob.des.tau(j), info_mob.act.tau(j), info_mob.act.tau_ext(j));
			}
           
			rt_printf("\n");
			// */
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
	}
}


void signal_handler(int signum)
{
    rt_task_delete(&motor_task);
    rt_task_delete(&print_task);
    rt_task_delete(&xddp_writer);
    
    for(int i=0; i<MOBILE_DRIVE_NUM; i++)
        ecat_iservo[i].setServoOff();
        
    ecat_master.deactivate();

    printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGTSTP)
		printf("╔═══════════════[SIGNAL INPUT SIGTSTP]══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	

    exit(1);
}

int main(int argc, char *argv[])
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
	signal(SIGTSTP, signal_handler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    cpu_set_t cpuset_qt, cpuset_rt1, cpuset_rt2;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt1);  
    CPU_ZERO(&cpuset_rt2);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt1);  
    CPU_SET(5, &cpuset_rt2);  

	cs_hyumob=CS_Mobile();
	cs_hyumob.CSSetup("../lib/URDF2CASADI/hyumob/hyumob.json", period);
	cs_nom_hyumob=CS_Mobile();
	cs_nom_hyumob.CSSetup("../lib/URDF2CASADI/hyumob/hyumob.json", period);

    
    // std::thread qtThread(runQtApplication, argc, argv);
    // pthread_t pthread = qtThread.native_handle();
    // int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&safety_task, "safety_task", 0, 93, 0);
    rt_task_set_affinity(&safety_task, &cpuset_rt1);
	rt_task_start(&safety_task, &safety_run, NULL);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

    rt_task_create(&print_task, "print_task", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt1);
    rt_task_start(&print_task, &print_run, NULL);

    // Must pause here
    pause();
    // qtThread.join();

    // Finalize
    signal_handler(0);

    return 0;
}

