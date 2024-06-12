#include "CS_mobile.h"


CS_Mobile::CS_Mobile()
{
    robotModel = "hyumob";
    n_dof = 3;

    this->q.resize(this->n_dof);
    this->dq.resize(this->n_dof);
    this->ddq.resize(this->n_dof);
    this->M.resize(this->n_dof, this->n_dof);
    this->Minv.resize(this->n_dof, this->n_dof);
    this->C.resize(this->n_dof, this->n_dof);
    this->G.resize(this->n_dof);
    this->J_b.resize(6, this->n_dof);
    this->J_s.resize(6, this->n_dof);

    this->Kp.resize(this->n_dof, this->n_dof);
    this->Kv.resize(this->n_dof, this->n_dof);
    this->Ki.resize(this->n_dof, this->n_dof);

    this->Hinf_Kp.resize( this->n_dof, this->n_dof);
    this->Hinf_Kv.resize( this->n_dof, this->n_dof);
    this->Hinf_Ki.resize( this->n_dof, this->n_dof);
    this->Hinf_K_gamma.resize( this->n_dof, this->n_dof);

    Hinf_Kp = Mob_JMat::Zero();
    Hinf_Kv = Mob_JMat::Zero();
    Hinf_Ki = Mob_JMat::Zero();
    Hinf_K_gamma = Mob_JMat::Zero();

    NRIC_Kp = Mob_JMat::Zero();
    NRIC_Ki = Mob_JMat::Zero();
    NRIC_K_gamma = Mob_JMat::Zero();

    Kp = Mob_JMat::Zero();
    Kv = Mob_JMat::Zero();
    Ki = Mob_JMat::Zero();

    e = Mob_JVec::Zero();
    eint = Mob_JVec::Zero();


}

void CS_Mobile::CSSetup(const string& _modelPath, double _period)// : loader_(_modelPath), period[sec]
{
	JsonLoader loader_ = JsonLoader(_modelPath);

	robotModel = loader_.getValue("name").asString();
	n_dof = std::stoi(loader_.getValue("n_dof").asString());
    period = _period;

    this->q.resize(this->n_dof);
    this->dq.resize(this->n_dof);
    this->ddq.resize(this->n_dof);
    this->M.resize(this->n_dof, this->n_dof);
    this->Minv.resize(this->n_dof, this->n_dof);
    this->C.resize(this->n_dof, this->n_dof);
    this->G.resize(this->n_dof);
    this->J_b.resize(6, this->n_dof);
    this->J_s.resize(6, this->n_dof);

    this->Kp.resize(this->n_dof, this->n_dof);
    this->Kv.resize(this->n_dof, this->n_dof);
    this->Ki.resize(this->n_dof, this->n_dof);

    this->Hinf_Kp.resize( this->n_dof, this->n_dof);
    this->Hinf_Kv.resize( this->n_dof, this->n_dof);
    this->Hinf_Ki.resize( this->n_dof, this->n_dof);
    this->Hinf_K_gamma.resize( this->n_dof, this->n_dof);


	
	// Load the shared library
    string casadi_path = "../lib/URDF2CASADI/";
    casadi_path = casadi_path + robotModel + "/" + robotModel;

    string func_path = casadi_path + "_fd.so";
    FD_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FD_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_fd.so");
    }
    func_path = casadi_path + "_M.so";
    M_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (M_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_M.so");
    }
    func_path = casadi_path + "_Minv.so";
    Minv_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (Minv_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_Minv.so");
    }
    func_path = casadi_path + "_C.so";
    C_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (C_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_C.so");
    }
    func_path = casadi_path + "_G.so";
    G_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (G_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_G.so");
    }
    func_path = casadi_path + "_fk.so";
    FK_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (FK_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_fk_ee.so");
    }
    func_path = casadi_path + "_J_b.so";
    J_b_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_b_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_J_b.so");
    }
    func_path = casadi_path + "_J_s.so";
    J_s_handle = dlopen(func_path.c_str(), RTLD_LAZY);
    if (J_s_handle == 0) {
        throw std::runtime_error("Cannot open hyumob_J_s.so");
    }
    
    // Reset error
    dlerror();
    // Function evaluation
    FD_eval = (eval_t)dlsym(FD_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    M_eval = (eval_t)dlsym(M_handle, "M");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    Minv_eval = (eval_t)dlsym(Minv_handle, "Minv");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }

    C_eval = (eval_t)dlsym(C_handle, "coriolis");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    G_eval = (eval_t)dlsym(G_handle, "generalized_gravity");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    FK_eval = (eval_t)dlsym(FK_handle, "fk_T");
    if (dlerror()) {
        throw std::runtime_error("Failed to retrieve \"fk_T\" function.\n");
    }
    J_b_eval = (eval_t)dlsym(J_b_handle, "J_b");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    J_s_eval = (eval_t)dlsym(J_s_handle, "J_s");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    
}

void CS_Mobile::setPIDgain(Mob_JVec _Kp, Mob_JVec _Kd, Mob_JVec _Ki)
{
    Kp = _Kp.asDiagonal();
    Kv = _Kd.asDiagonal();
    Ki = _Ki.asDiagonal();
}

void CS_Mobile::setHinfgain(Mob_JVec _Hinf_Kp, Mob_JVec _Hinf_Kd, Mob_JVec _Hinf_Ki, Mob_JVec _Hinf_K_gamma)
{
    Hinf_Kp = _Hinf_Kp.asDiagonal();
    Hinf_Kv = _Hinf_Kd.asDiagonal();
    Hinf_Ki = _Hinf_Ki.asDiagonal();
    Hinf_K_gamma = _Hinf_K_gamma.asDiagonal();
}

void CS_Mobile::setNRICgain(Mob_JVec _NRIC_Kp, Mob_JVec _NRIC_Ki, Mob_JVec _NRIC_K_gamma)
{
    NRIC_Kp = _NRIC_Kp.asDiagonal();
    NRIC_Ki = _NRIC_Ki.asDiagonal();
    NRIC_K_gamma = _NRIC_K_gamma.asDiagonal();
}

void CS_Mobile::updateRobot(Mob_JVec _q, Mob_JVec _dq)
{
    M = computeM(_q);
    Minv = computeMinv(_q);
    C = computeC(_q, _dq);
    G = computeG(_q); 

    J_b = computeJ_b(_q);

    T_ee = computeFK(_q);

    V_b = J_b*_dq;

    isUpdated=true;   
}

Mob_JVec CS_Mobile::computeFD(Mob_JVec _q, Mob_JVec _dq, Mob_JVec _tau)
{
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[3*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    double input_tau[sz_arg];

    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        input_tau[i] = _tau(i);
        arg[3*i] = &input_pos[i];
        arg[3*i+1] = &input_vel[i];
        arg[3*i+2] = &input_tau[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x1 Vector
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FD_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        // if(!isnan(output_values[i]))
            ddq_res(i) = output_values[i];
        // else
        //     cout<<[ERROR NaN]<<endl;
    }

    // // fd method
    // Minv = computeMinv(_q);
    // C = computeC(_q,_dq);
    // G = computeG(_q);

    // ddq_res = Minv*(_tau-C*_dq-G);
    
    return ddq_res;

}

void CS_Mobile::computeRK45(Mob_JVec _q, Mob_JVec _dq, Mob_JVec _tau, Mob_JVec &_q_nom, Mob_JVec &_dq_nom, Mob_JVec &_ddq_nom)
{
    Mob_JVec k1, k2, k3, k4;

    // state update
    Mob_JVec _q0 = _q;
    // Mob_JVec _q_dot = info.act.q_dot;
    Mob_JVec _q_dot0 = _dq;
    // Mob_JVec _tau = _tau;

    // 1st stage
    k1 = computeFD(_q, _q_dot0, _tau);
    Mob_JVec _q1 = _q + 0.5 * period * _dq;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    Mob_JVec _q_dot1 = _dq + 0.5 * period * k1; // k2(q_dot)
    
    // 2nd stage
    k2 = computeFD(_q1, _q_dot1, _tau);
    Mob_JVec _q2 = _q + 0.5 * period * _q_dot1;
    Mob_JVec _q_dot2 = _dq + 0.5 * period * k2;
    
    // 3th stage
    k3 = computeFD(_q2, _q_dot2, _tau);
    Mob_JVec _q3 = _q + period * _q_dot2;
    Mob_JVec _q_dot3 = _dq + period * k3;
    
    // 4th stage
    k4 = computeFD(_q3, _q_dot3, _tau);
    _q_nom = _q + (period / 6.0) * (_dq + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
    _dq_nom = _dq + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
    _ddq_nom = k1;
}

Mob_MassMat CS_Mobile::computeM(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> M_res = M_cs(arg);

    // return M;
    
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (M_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            M(j,i) = output_values[i * sz_res + j];
        }
    }

    return M;
}

Mob_MassMat CS_Mobile::computeMinv(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> Minv_res = Minv_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (Minv_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            Minv(j,i) = output_values[i * sz_res + j];
        }
    }

    return Minv;
}

Mob_MassMat CS_Mobile::computeC(Mob_JVec _q, Mob_JVec _dq)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // casadi::DM dq_dm = casadi::DM(vector<double>(_dq.data(), _dq.data() + _dq.size()));
    // map<string, casadi::DM> arg;
    // arg["q"] = q_dm; arg["dq"] = dq_dm;
    // map<string, casadi::DM> C_res = C_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[2*sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[sz_arg];
    double input_vel[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_pos[i] = _q(i);
        input_vel[i] = _dq(i);
        arg[2*i] = &input_pos[i];
        arg[2*i+1] = &input_vel[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (C_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            C(j,i) = output_values[i * sz_res + j];
        }
    }

    return C;
}

Mob_JVec CS_Mobile::computeG(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> G_res = G_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (G_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        G(i) = output_values[i];
    }

    return G;
}

SE3 CS_Mobile::computeFK(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> FK_res = FK_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = 4;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[sz_res*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[sz_res*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (FK_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {   
            T_ee(j,i) = output_values[i * sz_res + j];
        }
    }

    R_ee = T_ee.block<3,3>(0,0);

    return T_ee;
}

Mob_Jacobian CS_Mobile::computeJ_b(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_b_res = J_b_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_b_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            J_b(j,i) = output_values[i * 6 + j];
        }
    }

    return J_b;
}


Mob_Jacobian CS_Mobile::computeJ_s(Mob_JVec _q)
{
    // casadi::DM q_dm = casadi::DM(vector<double>(_q.data(), _q.data() + _q.size()));
    // vector<casadi::DM> arg = {q_dm};
    // vector<casadi::DM> J_s_res = J_s_cs(arg);

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = n_dof;
    casadi_int sz_res = n_dof;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[sz_arg];
    double* res[6*sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[sz_arg];
    for (casadi_int i = 0; i < sz_arg; ++i) {
        input_values[i] = _q(i);
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[6*sz_res]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    if (J_s_eval(arg, res, iw, w, mem)) {
        throw std::runtime_error("Function evaluation failed.\n");
    }

    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < 6; ++j) {   
            J_s(j,i) = output_values[i * 6 + j];
        }
    }

    return J_s;
}

Mob_MassMat CS_Mobile::getM()
{
    return M;
}
Mob_MassMat CS_Mobile::getMinv()
{
    return Minv;
}
Mob_MassMat CS_Mobile::getC()
{
    return C;
}
Mob_JVec CS_Mobile::getG()
{
    return G;
}
SE3 CS_Mobile::getFK()
{
    return T_ee;
}
SO3 CS_Mobile::getRMat()
{
    return R_ee;
}
Mob_Jacobian CS_Mobile::getJ_b()
{
    return J_b;
}
Mob_Jacobian CS_Mobile::getJ_s()
{
    return J_s;
}
Twist CS_Mobile::getBodyTwist()
{
    return V_b;
}


Mob_JVec CS_Mobile::ComputedTorqueControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des)
{
    Mob_JVec e = q_des-q;
    Mob_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        Mob_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref + C*dq + G;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        Mob_JVec ddq_ref = ddq_des + Kv*edot + Kp*e;
        tau = M*ddq_ref+C*dq+G;
    }
    return tau;   
}

Mob_JVec CS_Mobile::ComputedTorqueControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des, Mob_JVec _tau_ext)
{
    Mob_JVec e = q_des-q;
    Mob_JVec edot = dq_des-dq;
    double m_gam = 1;
    // double m_gam = 10;
    double m_gam_mob = 0.02;
    // double m_gam_mob = 0.2;
    Mob_MassMat m_mat = Mob_MassMat::Zero();
    m_mat.diagonal()<< m_gam_mob, m_gam_mob, m_gam_mob;
    Mob_JVec K;
    K << 1.0, 1.0, 1.0;
    
    tau_ext = _tau_ext;
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        Mob_JVec ddq_ref = ddq_des + m_mat*Kv*edot + m_mat*Kp*e + m_mat*tau_ext;
        tau = M*ddq_ref + C*dq + G + tau_ext - tau_bd;
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        Mob_JVec ddq_ref = ddq_des + m_mat*Kv*edot + m_mat*Kp*e + m_mat*tau_ext;
        tau = M*ddq_ref+C*dq+G + tau_ext - tau_bd;
    }
    // tau = K.cwiseProduct(tau);
    return tau;   
}

Mob_JVec CS_Mobile::HinfControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des)
{
    Mob_JVec e = q_des-q;
    Mob_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    
    if(isUpdated)
    {
        Mob_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        Mob_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        Mob_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        Mob_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

Mob_JVec CS_Mobile::HinfControl( Mob_JVec q,Mob_JVec dq,Mob_JVec q_des,Mob_JVec dq_des,Mob_JVec ddq_des, Mob_JVec _tau_ext)
{
    Mob_JVec e = q_des-q;
    Mob_JVec edot = dq_des-dq;
    
    eint = eint + e*period;	
    tau_ext = _tau_ext;
    
    if(isUpdated)
    {
        Mob_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        Mob_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
        isUpdated = false;
    }
    else
    {
        M = computeM(q);
        C = computeC(q, dq);
        G = computeG(q);
        Mob_JVec ddq_ref = ddq_des+Hinf_Kv*edot+Hinf_Kp*e;
        Mob_JVec dq_ref = dq_des;
        tau = M*ddq_ref+C*dq_ref+G+(Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
    }
    return tau;
}

Mob_JVec CS_Mobile::NRIC(Mob_JVec q_r, Mob_JVec dq_r, Mob_JVec q_n, Mob_JVec dq_n)
{
    Mob_JVec e = q_r-q_n;
    Mob_JVec edot = dq_r - dq_n;
    eint = eint + e*period;	
    
    tau = NRIC_K_gamma * (edot + NRIC_Kp*e + NRIC_Ki*eint);
    computeAlpha(edot, tau);
    tau_bd = alpha*tau;
    tau = (1-alpha)*tau;


    return tau;
}
void CS_Mobile::computeAlpha(Mob_JVec edot, Mob_JVec tau_c)
{
    double edotc, edotx;
    edotc = edot.transpose() * tau_c;
    edotx = edot.transpose() * tau_ext;
    if(edotc>0 && edotx >0)
    {
        if(edotc<edotx)
            alpha=1.0;
        else
            alpha = edotx/edotc;
    }
    else
        alpha = 0;
}