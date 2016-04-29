#include <iostream>
#include <vector>

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
#define SIMULATION 1

#if SIMULATION
#define SIM_LOG(message) do { std::cout << message << std::endl; } while (false)
#else
#define SIM_LOG(message) do {} while (false)
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
static Joints pose_cog_left;
static Joints pose_left_up;
static Joints pose_right_up;

//static Joints pose_left_forward;
//static Joints pose_left_down;
static double      delta_t = 0.01;
static double      duration = 20.0;
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

typedef void *(FunctionPtr)(void);

/**
 * @brief Abstracts a step which first assigns a target and makes the required movements.
 */
class Step
{
public:
    Step(std::string name, FunctionPtr &assign, FunctionPtr &move, double tau = duration)
        : name(name), _tau(tau), _assigned(false), _time(0)
    {
        _assign = &assign;
        _move = &move;
    }

    bool hasFinished() const
    {
        return _time <= _tau;
    }

    void execute()
    {
        if (!_assigned)
        {
            _assign();
            _assigned = true;
        }
        else
        {
            _move();
            _time += delta_t;
        }
    }

    std::string name;

    FunctionPtr *_assign;
    FunctionPtr *_move;

private:
    double _tau;
    double _time;
    bool _assigned;
};

/**
 * @brief Executes a sequence of steps.
 */
class StepSequence
{
public:
    StepSequence()
    {
        _active = _steps.begin();
    }

    void add(const Step &step)
    {
        _steps.push_back(step);
    }

    void execute()
    {
        if (_active >= _steps.end())
            return;

        if (_active->hasFinished())
        {
            ++_active;
            if (_active != _steps.end())
            {
                SIM_LOG("Executing step " << _active->name);
            }
            else
            {
                SIM_LOG("Finished execution");
            }
        }

        _active->execute();
    }

private:
    std::vector<Step> _steps;
    std::vector<Step>::iterator _active;
};

// state machine
static StepSequence sequence;
enum Steps {
  RELAX_DYNAMICS,

  ASSIGN_RAISE_RIGHT_ARM,
  MOVE_RAISE_RIGHT_ARM,

  ASSIGN_COG_RIGHT,
  MOVE_COG_RIGHT,

  ASSIGN_COG_RIGHT_2,
  MOVE_COG_RIGHT_2,

    ASSIGN_ADJUST_LEFT,
    MOVE_ADJUST_LEFT,

  ASSIGN_LEFT_LIFT_UP,
  MOVE_LEFT_LIFT_UP,

  ASSIGN_LEFT_DOWN,
  MOVE_LEFT_DOWN,

    ASSIGN_LEFT_DOWN_2,
    MOVE_LEFT_DOWN_2,

  ASSIGN_RAISE_LEFT_ARM,
  MOVE_RAISE_LEFT_ARM,

  ASSIGN_COG_LEFT,
  MOVE_COG_LEFT,

  ASSIGN_RIGHT_LIFT_UP,
  MOVE_RIGHT_LIFT_UP,

  ASSIGN_RIGHT_DOWN,
  MOVE_RIGHT_DOWN,

    ASSIGN_RIGHT_DOWN_2,
    MOVE_RIGHT_DOWN_2,
};
static Steps which_step;

// macro to decrement time_to_go and transition to next state if ready
// (it's a macro so we can easily use the # operator to stringify the enum)
#define STEP_STATE_MACHINE(next_state, next_dur) { \
    time_to_go -= delta_t; \
    if (time_to_go <= 0) { \
        SIM_LOG(#next_state); \
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
    cog_target.x[_X_] *= 0.6;

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
init_step_aerobic_task(void)
{
  // copy the default pose into our struct to make it copyable
  pose_default.copyFrom(joint_default_state);

  target = pose_default;
  // crouch
  target.joints[R_HFE].th = target.joints[L_HFE].th = 0.58;
  target.joints[R_KFE].th = target.joints[L_KFE].th = 1.00;
  target.joints[R_AFE].th = target.joints[L_AFE].th = 0.55;

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
    STEP_STATE_MACHINE(ASSIGN_RAISE_RIGHT_ARM, duration);
  }
  else {
    STEP_STATE_MACHINE(ASSIGN_RAISE_RIGHT_ARM, duration);
  }

  return TRUE;
}

static int
run_step_aerobic_task(void)
{
    sequence.execute();
  int i, j;

  // switch according to the current state of the state machine
  switch (which_step) {

  case RELAX_DYNAMICS:
    // in simulation, the moment you start calling SL_InvDynNE, the dynamics start oscillating like crazy.
    // we need a state that just waits for everything to calm down before proceeding.
    STEP_STATE_MACHINE(ASSIGN_RAISE_RIGHT_ARM, 0);
    break;

  case ASSIGN_RAISE_RIGHT_ARM:
      lower_arm_target(Left);
      raise_arm_target(Right);
      STEP_STATE_MACHINE(MOVE_RAISE_RIGHT_ARM, duration);
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
      STEP_STATE_MACHINE(ASSIGN_LEFT_LIFT_UP, 0);
      break;

  case ASSIGN_LEFT_LIFT_UP: // go down
    // set target to raise left leg in joint space and hold other joints at current angles
    pose_cog_right.copyFrom(joint_des_state);
    target.copyFrom(joint_des_state);
    target.joints[R_HAA].th = -.2;
    target.joints[L_AFE].th = 0.4;
    target.joints[L_AAA].th = 0.23;

    pose_left_up = target;

    STEP_STATE_MACHINE(MOVE_LEFT_LIFT_UP, duration);
    break;

  case MOVE_LEFT_LIFT_UP:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_LEFT_DOWN, 0);
    break;

  case ASSIGN_LEFT_DOWN:
    target = pose_cog_right;
    STEP_STATE_MACHINE(MOVE_LEFT_DOWN, duration);
    break;

  case MOVE_LEFT_DOWN:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_RAISE_LEFT_ARM, 0);
    break;

  case ASSIGN_RAISE_LEFT_ARM:
    lower_arm_target(Right);
    raise_arm_target(Left);
    STEP_STATE_MACHINE(MOVE_RAISE_LEFT_ARM, duration);
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
      STEP_STATE_MACHINE(ASSIGN_RIGHT_LIFT_UP, 0);
      break;

  case ASSIGN_RIGHT_LIFT_UP:
    // store the weight-on-right-foot pose for later
    //pose_cog_left.joints[L_HAA].th -= 0.1;
    //pose_cog_left.joints[L_AAA].th = 0.26;
    pose_cog_left.copyFrom(joint_des_state);

    // set target to raise left leg in joint space and hold other joints at current angles
    target.copyFrom(joint_des_state);
    // set target to raise left leg in joint space and hold other joints at current angles
    //target.copyFrom(joint_des_state);
    target.joints[L_HAA].th = -.2;
    target.joints[R_AFE].th = 0.4;
    target.joints[R_AAA].th = 0.23;

    STEP_STATE_MACHINE(MOVE_RIGHT_LIFT_UP, duration);
      break;

  case MOVE_RIGHT_LIFT_UP:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_RIGHT_DOWN, 0);
      break;


  case ASSIGN_RIGHT_DOWN:
    target = pose_cog_left;
    STEP_STATE_MACHINE(MOVE_RIGHT_DOWN, duration);
    break;

  case MOVE_RIGHT_DOWN:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_RAISE_RIGHT_ARM, 0);
    break;

  /*
  case CROUCH_AND_RAISE_ARM:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_COG_RIGHT, 0.0);
    break;

  case ASSIGN_COG_RIGHT:
    // set target to move center of gravity over right foot
    for (i=_X_; i <= _Z_; ++i) {
        cog_target.x[i] = cart_des_state[RIGHT_FOOT].x[i];
    }

    // the structure cog_des has the current position of the COG computed from the
    // joint_des_state of the robot. cog_des should track cog_traj
    for (i=1; i<=N_CART; ++i) {
      cog_traj.x[i] = cog_des.x[i];
    }

    // `stat` tells the COG IK function that the feet are planted on the ground
    for (i=1; i<=6; ++i) {
      stat[RIGHT_FOOT][i] = TRUE;
      stat[LEFT_FOOT][i] = TRUE;
    }

    STEP_STATE_MACHINE(MOVE_COG_RIGHT, duration);
    break;

  case MOVE_COG_RIGHT:
    step_cog_ik();
    STEP_STATE_MACHINE(ASSIGN_JOINT_TARGET_LIFT_UP, 0.0)
    break;

  case ASSIGN_JOINT_TARGET_LIFT_UP:
    // store the weight-on-right-foot pose for later
    pose_cog_right.copyFrom(joint_des_state);
    // set target to raise left leg in joint space and hold other joints at current angles
    target.copyFrom(joint_des_state);
    target.joints[R_HFE].th =  0.4;
    target.joints[R_HAA].th = -0.12;

    target.joints[L_HFE].th =  0.6;
    target.joints[L_HAA].th = -0.1;

    target.joints[L_KFE].th =  0.85;
    target.joints[L_AFE].th =  0.28;
    target.joints[L_AAA].th =  0.20;
    pose_left_up = target;

    // shift weight more, barely lift up
    // TODO: this causes left foot to slide in as its friction goes to 0
    //target.joints[R_AAA].th = 0.23;
    //target.joints[L_HAA].th = -0.25;
    //target.joints[L_HFE].th =  0.4;
    //target.joints[L_KFE].th =  0.60;
    //target.joints[L_AFE].th =  0.27;
    //target.joints[L_AAA].th = 0.18;

    STEP_STATE_MACHINE(MOVE_JOINT_TARGET_LIFT_UP, duration);
    break;

  case MOVE_JOINT_TARGET_LIFT_UP:
    // do one step of joint-space min-jerk movement to raise left leg
    step_min_jerk_jointspace();

    STEP_STATE_MACHINE(ASSIGN_JOINT_TARGET_FOOT_FORWARD, 0.0);
    break;

  case ASSIGN_JOINT_TARGET_FOOT_FORWARD:

    target.joints[L_AFE].th = 0.0;
    target.joints[L_KFE].th = 0.5;
    target.joints[L_HFE].th = 0.7;
    pose_left_forward = target;

    STEP_STATE_MACHINE(MOVE_JOINT_TARGET_FOOT_FORWARD, duration)

  case MOVE_JOINT_TARGET_FOOT_FORWARD:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_JOINT_TARGET_FOOT_DOWN, 0.0);
    break;

  case ASSIGN_JOINT_TARGET_FOOT_DOWN:
    target.joints[R_HAA].th = -0.08;
    target.joints[R_HFE].th =  0.55;
    target.joints[R_KFE].th =  0.7;
    target.joints[R_AFE].th =  0.3;

    target.joints[L_HFE].th =  0.75;
    target.joints[L_KFE].th =  0.2;
    target.joints[L_AFE].th = -0.2;
    target.joints[L_AAA].th =  0.15;

    pose_left_down = target;

    STEP_STATE_MACHINE(MOVE_JOINT_TARGET_FOOT_DOWN, duration)
    break;

  case MOVE_JOINT_TARGET_FOOT_DOWN:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_TARGET_FOOT_PLANT, 0.0)
    break;

  case ASSIGN_TARGET_FOOT_PLANT:
    lower_arm_target(Right);
    raise_arm_target(Left);
    target.joints[L_HAA].th = target.joints[R_HAA].th = 0;
    target.joints[R_AAA].th =  0.12;
    target.joints[L_AAA].th =  0.07;
    target.joints[L_AFE].th = -0.22;
    STEP_STATE_MACHINE(MOVE_FOOT_PLANT, duration)
    break;

  case MOVE_FOOT_PLANT:
    step_min_jerk_jointspace();
    STEP_STATE_MACHINE(ASSIGN_COG_SHIFT_LEFT, 0.0)
    break;

  case ASSIGN_COG_SHIFT_LEFT:
    for (i=_X_; i <= _Z_; ++i) {
        cog_target.x[i] = cart_des_state[LEFT_FOOT].x[i];
    }
    // the structure cog_des has the current position of the COG computed from the
    // joint_des_state of the robot. cog_des should track cog_traj
    for (i=1; i<=N_CART; ++i) {
      cog_traj.x[i] = cog_des.x[i];
    }
    // `stat` tells the COG IK function that the feet are planted on the ground
    for (i=1; i<=6; ++i) {
      stat[RIGHT_FOOT][i] = TRUE;
      stat[LEFT_FOOT][i] = TRUE;
    }
    STEP_STATE_MACHINE(MOVE_COG_SHIFT_LEFT, duration/2)
    break;

  case MOVE_COG_SHIFT_LEFT:
    // since previous pose wasn't quite touching ground, allow target to move
    for (i=_X_; i <= _Z_; ++i) {
        cog_target.x[i] = cart_des_state[LEFT_FOOT].x[i];
    }
    step_cog_ik();
    STEP_STATE_MACHINE(FREEZE, 0.0)
    break;

  case FREEZE:
    // done!
    freeze();
    return TRUE;
    */
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
change_step_aerobic_task(void)
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
extern "C" void add_step_aerobic_task()
{
  int i, j;
  static int firsttime = TRUE;

  if (firsttime) {
    firsttime = FALSE;

    addTask("Step Aerobic Task", init_step_aerobic_task,
      run_step_aerobic_task, change_step_aerobic_task);
  }

}
