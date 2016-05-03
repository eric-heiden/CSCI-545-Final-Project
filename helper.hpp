#ifndef HELPER_H
#define HELPER_H

#if SIMULATION
#define SIM_LOG(message) std::cout << message << std::endl;
#else
#define SIM_LOG(message)
#endif

struct Joints
{
    SL_DJstate joints[N_DOFS+1];
    void copyFrom(SL_DJstate const *states)
    {
        for (int i=1; i<=N_DOFS; i++) {
            joints[i] = states[i];
        }
    }
};

// local variables
static Joints target;
static Joints pose_default;

static Joints pose_cog_right;
static Joints pose_cog_middle;
static Joints pose_cog_left;

static Joints pose_left_up;
static Joints pose_right_up;

// variables for COG control
static SL_Cstate cog_target;
static SL_Cstate cog_traj;
static SL_Cstate cog_ref;
static iMatrix stat;
static Matrix Jccogp;
static Matrix NJccog;
static Matrix fc;
static Vector delta_x;
static Vector delta_th;

static void initializeSL()
{
    stat     = my_imatrix(1,N_ENDEFFS,1,2*N_CART);
    Jccogp   = my_matrix(1,N_DOFS,1,N_CART);
    NJccog   = my_matrix(1,N_DOFS,1,N_DOFS+2*N_CART);
    fc       = my_matrix(1,N_ENDEFFS,1,2*N_CART);
    delta_x  = my_vector(1, N_CART); // desired velocity of COG in Cartesian space
    delta_th = my_vector(1, N_DOFS); // joint state velocity for COG IK

    // move to default position
    go_target_wait_ID(joint_default_state);
}



/************************************************************************************
 *                         Minimum Jerk Interpolation                               *
 ************************************************************************************/


/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step
\date  April 2014

\remarks

Given the time to go, the current state is updated to the next state
using min jerk splines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          xt,dxt,ddxt : the current state, vel, acceleration
 \param[in]          xf,dxf,ddxf : the target state, vel, acceleration
 \param[in]          togo   : time to go until target is reached
 \param[in]          t       : time increment
 \param[out]          x_next,xd_next,xdd_next : the next state after dt

 ******************************************************************************/
static int
min_jerk_next_step (double xt, double dxt, double ddxt, double xf, double dxf, double ddxf,
        double togo, double t,
        double *x_next, double *xd_next, double *xdd_next)

{
    *x_next = 0.5*pow(togo, -5.0)*(2*xt*pow(togo, 5.0)
                                 + 2*t*dxt*pow(togo, 5.0)
                                 + pow(t, 2.0)*ddxt*pow(togo, 5.0)
                                 + pow(t, 3.0)*pow(togo, 2.0)
                                 * (20*xf + (-20.0)*xt + togo*((-8.0)*dxf
                                                               + (-12.0)*dxt
                                                               + ddxf*togo
                                                               + (-3.0)*ddxt*togo))
                                 + pow(t, 5.0)*(12*xf + (-12.0)*xt
                                                + togo*((-6.0)*dxf
                                                + (-6.0)*dxt + ddxf*togo
                                                        + (-1.0)*ddxt*togo))
                                 + pow(t, 4.0)*togo*((-30.0)*xf + 30*xt
                                                     + togo*(14*dxf + 16*dxt
                                                             + (-2.0)*ddxf*togo
                                                             + 3*ddxt*togo)));
    *xd_next = 0.5*pow(togo, (-5.0))*(2*dxt*pow(togo, 5.0)
                                    + 2*t*ddxt*pow(togo, 5.0)
                                    + 3*pow(t, 2.0)*pow(togo, 2.0)
                                    * (20*xf + (-20.0)*xt + togo*((-8.0)*dxf
                                                                  + (-12.0)*dxt
                                                                  + ddxf*togo
                                                                  + (-3.0)*ddxt*togo))
                                    + 5*pow(t, 4.0)*(12*xf + (-12.0)*xt
                                                     + togo*((-6.0)*dxf
                                                             + (-6.0)*dxt + ddxf*togo
                                                             + (-1.0)*ddxt*togo))
                                    + 4*pow(t, 3.0)*togo*((-30.0)*xf + 30*xt
                                                          + togo*(14*dxf + 16*dxt
                                                                  + (-2.0)*ddxf*togo
                                                                  + 3*ddxt*togo)));
    *xdd_next = 0.5*pow(togo, (-5.0))*(2*ddxt*pow(togo, 5.0)
                                     + 6*t*pow(togo, 2.0)*(20*xf + (-20.0)*xt
                                                           + togo*((-8.0)*dxf
                                                                   + (-12.0)*dxt
                                                                   + ddxf*togo
                                                                   + (-3.0)*ddxt*togo))
                                     + 20*pow(t, 3.0)*(12*xf + (-12.0)*xt
                                                       + togo*((-6.0)*dxf
                                                               + (-6.0)*dxt + ddxf*togo
                                                               + (-1.0)*ddxt*togo))
                                     + 12*pow(t, 2.0)*togo*((-30.0)*xf + 30*xt
                                                            + togo*(14*dxf + 16*dxt
                                                                    + (-2.0)*ddxf*togo
                                                                    + 3*ddxt*togo)));
    return TRUE;
}

// compute min-jerk increment of entire joint space
static void step_min_jerk_jointspace(double time_to_go)
{
    for (int i = 1; i <= N_DOFS; ++i) {
        SL_DJstate &joint = joint_des_state[i];
        min_jerk_next_step(
            joint.th, joint.thd, joint.thdd,
            target.joints[i].th, 0, 0,
            time_to_go, delta_t,
            &joint.th, &joint.thd, &joint.thdd
        );
    }
}


/************************************************************************************
 *                              Inverse Kinematics                                  *
 ************************************************************************************/

static void set_ik_target(int foot, double xfactor = 0.7,
                          bool leftFootOnGround = true, bool rightFootOnGround = true)
{
    // set target to move center of gravity over right foot
    for (int i=_X_; i <= _Z_; ++i)
    {
        cog_target.x[i] = cart_des_state[foot].x[i];
    }
    cog_target.x[_Y_] += 0.01;
    cog_target.x[_X_] *= xfactor;

    // the structure cog_des has the current position of the COG computed from the
    // joint_des_state of the robot. cog_des should track cog_traj
    for (int i=1; i<=N_CART; ++i)
    {
        cog_traj.x[i] = cog_des.x[i];
    }

    // `stat` tells the COG IK function that the feet are planted on the ground
    for (int i=1; i<=6; ++i)
    {
        stat[RIGHT_FOOT][i] = rightFootOnGround ? TRUE : FALSE;
        stat[LEFT_FOOT][i] = leftFootOnGround ? TRUE : FALSE;
    }
}

static void step_cog_ik(double time_to_go)
{
    double const kp = 0.1;

    // plan the next step of cog movement with min jerk
    for (int i=1; i<=N_CART; ++i)
    {
        min_jerk_next_step(
            cog_traj.x[i], cog_traj.xd[i], cog_traj.xdd[i],
            cog_target.x[i], cog_target.xd[i], cog_target.xdd[i],
            time_to_go, delta_t,
            &(cog_traj.x[i]), &(cog_traj.xd[i]), &(cog_traj.xdd[i]));
    }

    // inverse kinematics: we use a P controller to correct for tracking erros
    for (int i=1; i<=N_CART; ++i)
    {
        cog_ref.xd[i] = kp*(cog_traj.x[i] - cog_des.x[i]) + cog_traj.xd[i];
        delta_x[i] = cog_ref.xd[i];
    }

    // the following code computes the contraint COG Jacobian
    // Jccogp is an N_DOFS x N_CART matrix
    // NJccog is an N_DOFS x N_DOF+2*N_CART matrix -- most likely this is not needed
    compute_cog_kinematics(stat, TRUE, FALSE, TRUE, Jccogp, NJccog);
    mat_vec_mult(Jccogp, delta_x, delta_th);

    // compute the joint_des_state[i].th and joint_des_state[i].thd
    for (int i=1; i<=N_DOFS; ++i)
    {
        joint_des_state[i].th += delta_t * delta_th[i];
        joint_des_state[i].thd = delta_th[i];
        joint_des_state[i].thdd = 0;
        joint_des_state[i].uff  = 0;
    }

    target.copyFrom(joint_des_state);
}


/************************************************************************************
 *                             Movement functions                                   *
 ************************************************************************************/

static void crouch()
{
    // copy the default pose into our struct to make it copyable
    pose_default.copyFrom(joint_default_state);

    target = pose_default;
    // crouch
    target.joints[R_HFE].th = target.joints[L_HFE].th = 0.50;
    target.joints[R_KFE].th = target.joints[L_KFE].th = 1.00;
    target.joints[R_AFE].th = target.joints[L_AFE].th = 0.50;
    pose_cog_middle = target;
}

enum Side
{
    Left,
    Right
};

static void raise_arm_target(Side side)
{
    int const shift = (side == Left) ? (L_HFE - R_HFE) : 0;
    target.joints[R_SAA + shift].th = -1.5;
    target.joints[R_HR + shift].th  =  0.4;
}

static void lower_arm_target(Side side)
{
    int const shift = (side == Left) ? (L_HFE - R_HFE) : 0;
    target.joints[R_SAA + shift].th = joint_default_state[R_SAA + shift].th;
    target.joints[R_HR + shift].th  = joint_default_state[R_HR + shift].th;
}

#endif // HELPER_H
