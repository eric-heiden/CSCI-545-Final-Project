#include <iostream>

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

#define ENDEFF_IK 0
#define SIMULATION 0

#if SIMULATION
#define SIM_LOG(message) std::cout << message << std::endl;
#else
#define SIM_LOG(message)
#endif

// making it easier to copy them
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

//static Joints pose_left_forward;
//static Joints pose_left_down;
static double      delta_t = 0.01;
static double      duration = 7.0;
static double      time_to_go = 0.0;

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

// state machine
enum Steps {
  CROUCH,

  ASSIGN_RAISE_RIGHT_ARM,
  MOVE_RAISE_RIGHT_ARM,

  ASSIGN_COG_RIGHT,
  MOVE_COG_RIGHT,

  ASSIGN_RAISE_LEFT_ARM,
  MOVE_RAISE_LEFT_ARM,

  ASSIGN_COG_LEFT,
  MOVE_COG_LEFT,

  SAVE_COG_LEFT,

  ASSIGN_COG_RIGHT_JOINTS,
  MOVE_COG_RIGHT_JOINTS,

  ASSIGN_COG_LEFT_JOINTS,
  MOVE_COG_LEFT_JOINTS,
};
static Steps which_step;

// macro to decrement time_to_go and transition to next state if ready
// (it's a macro so we can easily use the # operator to stringify the enum)
#define STEP_STATE_MACHINE(next_state, next_dur) { \
    time_to_go -= delta_t; \
    if (time_to_go <= delta_t/2) { \
        SIM_LOG(#next_state) \
        which_step = next_state; \
        time_to_go = (next_dur); \
    } \
}

// compute min-jerk increment for a scalar value
static int min_jerk_next_step (double x, double xd, double xdd,
            double t, double td, double tdd,
            double togo, double dt,
            double *x_next, double *xd_next, double *xdd_next);

// compute min-jerk increment of entire joint space
static void step_min_jerk_jointspace()
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

static void set_ik_target(int foot)
{
    // set target to move center of gravity over right foot
    for (int i=_X_; i <= _Z_; ++i) {
        cog_target.x[i] = cart_des_state[foot].x[i];
    }
    cog_target.x[_Y_] += 0.01;
    cog_target.x[_X_] *= 0.7;

    // the structure cog_des has the current position of the COG computed from the
    // joint_des_state of the robot. cog_des should track cog_traj
    for (int i=1; i<=N_CART; ++i) {
      cog_traj.x[i] = cog_des.x[i];
    }

    // `stat` tells the COG IK function that the feet are planted on the ground
    for (int i=1; i<=6; ++i) {
      stat[RIGHT_FOOT][i] = TRUE;
      stat[LEFT_FOOT][i] = TRUE;
    }
}

static void step_cog_ik()
{
    double const kp = 0.1;

    // plan the next step of cog movement with min jerk
    for (int i=1; i<=N_CART; ++i) {
      min_jerk_next_step(
        cog_traj.x[i], cog_traj.xd[i], cog_traj.xdd[i],
        cog_target.x[i], cog_target.xd[i], cog_target.xdd[i],
        time_to_go, delta_t,
        &(cog_traj.x[i]), &(cog_traj.xd[i]), &(cog_traj.xdd[i]));
    }

    // inverse kinematics: we use a P controller to correct for tracking erros
    for (int i=1; i<=N_CART; ++i) {
      cog_ref.xd[i] = kp*(cog_traj.x[i] - cog_des.x[i]) + cog_traj.xd[i];
      delta_x[i] = cog_ref.xd[i];
    }

    // the following code computes the contraint COG Jacobian
    // Jccogp is an N_DOFS x N_CART matrix
    // NJccog is an N_DOFS x N_DOF+2*N_CART matrix -- most likely this is not needed
    compute_cog_kinematics(stat, TRUE, FALSE, TRUE, Jccogp, NJccog);
    mat_vec_mult(Jccogp, delta_x, delta_th);

    // compute the joint_des_state[i].th and joint_des_state[i].thd
    for (int i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].th += delta_t * delta_th[i];
      joint_des_state[i].thd = delta_th[i];
      joint_des_state[i].thdd = 0;
      joint_des_state[i].uff  = 0;
    }
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

// allocate memory, etc. called once when task starts.
static int
init_shift_task(void)
{
  // copy the default pose into our struct to make it copyable
  pose_default.copyFrom(joint_default_state);

  target = pose_default;
  // crouch
  target.joints[R_HFE].th = target.joints[L_HFE].th = 0.50;
  target.joints[R_KFE].th = target.joints[L_KFE].th = 1.00;
  target.joints[R_AFE].th = target.joints[L_AFE].th = 0.50;
  pose_cog_middle = target;

  static int firsttime = TRUE;
  if (firsttime){
    firsttime = FALSE;

    stat     = my_imatrix(1,N_ENDEFFS,1,2*N_CART);
    Jccogp   = my_matrix(1,N_DOFS,1,N_CART);
    NJccog   = my_matrix(1,N_DOFS,1,N_DOFS+2*N_CART);
    fc       = my_matrix(1,N_ENDEFFS,1,2*N_CART);
    delta_x  = my_vector(1, N_CART); // desired velocity of COG in Cartesian space
    delta_th = my_vector(1, N_DOFS); // joint state velocity for COG IK

    // move to default position
    go_target_wait_ID(joint_default_state);
    STEP_STATE_MACHINE(CROUCH, duration * 2);
  }
  else {
    STEP_STATE_MACHINE(CROUCH, duration * 2);
  }

  return TRUE;
}

static int
run_shift_task(void)
{
  int i, j;

  // switch according to the current state of the state machine
  switch (which_step) {

  case CROUCH:
      step_min_jerk_jointspace();
      STEP_STATE_MACHINE(ASSIGN_RAISE_RIGHT_ARM, 0);
      break;

  case ASSIGN_RAISE_RIGHT_ARM:
      lower_arm_target(Left);
      raise_arm_target(Right);
      STEP_STATE_MACHINE(MOVE_RAISE_RIGHT_ARM, duration/2);
      break;

  case MOVE_RAISE_RIGHT_ARM:
      step_min_jerk_jointspace();
      STEP_STATE_MACHINE(ASSIGN_COG_RIGHT, 0);
      break;


  case ASSIGN_COG_RIGHT:
      set_ik_target(RIGHT_FOOT);
      STEP_STATE_MACHINE(MOVE_COG_RIGHT, duration);
      break;

  case MOVE_COG_RIGHT:
      step_cog_ik();
      STEP_STATE_MACHINE(ASSIGN_RAISE_LEFT_ARM, 0);
      break;

  case ASSIGN_RAISE_LEFT_ARM:
    pose_cog_right.copyFrom(joint_des_state);
    target = pose_cog_right;
    lower_arm_target(Right);
    raise_arm_target(Left);
    STEP_STATE_MACHINE(MOVE_RAISE_LEFT_ARM, duration/2);
    break;

  case MOVE_RAISE_LEFT_ARM:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_COG_LEFT, 0);
      break;


  case ASSIGN_COG_LEFT:
      set_ik_target(LEFT_FOOT);
      STEP_STATE_MACHINE(MOVE_COG_LEFT, duration);
      break;

  case MOVE_COG_LEFT:
      step_cog_ik();
      STEP_STATE_MACHINE(SAVE_COG_LEFT, 0);
      break;

  case SAVE_COG_LEFT:
      pose_cog_left.copyFrom(joint_des_state);
      STEP_STATE_MACHINE(ASSIGN_COG_RIGHT_JOINTS, 0);
      break;

  case ASSIGN_COG_RIGHT_JOINTS:
      target = pose_cog_right;
      STEP_STATE_MACHINE(MOVE_COG_RIGHT_JOINTS, duration);
      break;

  case MOVE_COG_RIGHT_JOINTS:
      step_min_jerk_jointspace();
      STEP_STATE_MACHINE(ASSIGN_COG_LEFT_JOINTS, 0);
      break;

  case ASSIGN_COG_LEFT_JOINTS:
      target = pose_cog_left;
      STEP_STATE_MACHINE(MOVE_COG_LEFT_JOINTS, duration);
      break;

  case MOVE_COG_LEFT_JOINTS:
      step_min_jerk_jointspace();
      STEP_STATE_MACHINE(ASSIGN_COG_RIGHT_JOINTS, 0);
      break;
  }

  // this is the "normal" inverse dynamics used for drawing, etc.
  //SL_InvDynNE(joint_state, joint_des_state, endeff, &base_state, &base_orient);

  // this is a special inverse dynamics computation for a free standing robot
  // BUT... it doesn't seem to work unless both feet are stuck to the ground
  //inverseDynamicsFloat(delta_t, stat, TRUE, joint_des_state, NULL, NULL, fc);

  return TRUE;
}

// it's somehow possible to call this from the console... don't know how yet
static int 
change_shift_task(void)
{
  int    ivar = 0; // default value
  double dvar = 0; // default value

  get_int("This is how to enter an integer variable", ivar, &ivar);
  get_double("This is how to enter a double variable", dvar, &dvar);

  return TRUE;
}

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
  *x_next = 0.5*pow(togo, -5.0)*(2*xt*pow(togo, 5.0) + 2*t*dxt*pow(togo, 5.0) + pow(t, 2.0)*ddxt*pow(togo, 5.0) + pow(t, 3.0)*pow(togo, 2.0)*(20*xf + (-20.0)*xt + togo*((-8.0)*dxf + (-12.0)*dxt + ddxf*togo + (-3.0)*ddxt*togo)) + pow(t, 5.0)*(12*xf + (-12.0)*xt + togo*((-6.0)*dxf + (-6.0)*dxt + ddxf*togo + (-1.0)*ddxt*togo)) + pow(t, 4.0)*togo*((-30.0)*xf + 30*xt + togo*(14*dxf + 16*dxt + (-2.0)*ddxf*togo + 3*ddxt*togo)));
  *xd_next = 0.5*pow(togo, (-5.0))*(2*dxt*pow(togo, 5.0) + 2*t*ddxt*pow(togo, 5.0) + 3*pow(t, 2.0)*pow(togo, 2.0)*(20*xf + (-20.0)*xt + togo*((-8.0)*dxf + (-12.0)*dxt + ddxf*togo + (-3.0)*ddxt*togo)) + 5*pow(t, 4.0)*(12*xf + (-12.0)*xt + togo*((-6.0)*dxf + (-6.0)*dxt + ddxf*togo + (-1.0)*ddxt*togo)) + 4*pow(t, 3.0)*togo*((-30.0)*xf + 30*xt + togo*(14*dxf + 16*dxt + (-2.0)*ddxf*togo + 3*ddxt*togo)));
  *xdd_next = 0.5*pow(togo, (-5.0))*(2*ddxt*pow(togo, 5.0) + 6*t*pow(togo, 2.0)*(20*xf + (-20.0)*xt + togo*((-8.0)*dxf + (-12.0)*dxt + ddxf*togo + (-3.0)*ddxt*togo)) + 20*pow(t, 3.0)*(12*xf + (-12.0)*xt + togo*((-6.0)*dxf + (-6.0)*dxt + ddxf*togo + (-1.0)*ddxt*togo)) + 12*pow(t, 2.0)*togo*((-30.0)*xf + 30*xt + togo*(14*dxf + 16*dxt + (-2.0)*ddxf*togo + 3*ddxt*togo)));
  return TRUE;
}

// declare with C linkage
extern "C" void add_shift_task()
{
  int i, j;
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;

    addTask("Shift Task", init_shift_task,
      run_shift_task, change_shift_task);
  }

}
