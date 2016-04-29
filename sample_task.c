/*============================================================================
==============================================================================
                      
                              sample_task.c
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

// system headers
#include "SL_system_headers.h"

// SL includes 
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

// defines

// local variables
static double start_time = 0.0;
static double freq;
static double amp;
static SL_DJstate  target[N_DOFS+1];

// global functions

// local functions
static int  init_sample_task(void);
static int  run_sample_task(void);
static int  change_sample_task(void);

/*****************************************************************************
******************************************************************************
Function Name	: add_sample_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_sample_task( void )

{
  int i, j;
  
  addTask("Sample Task", init_sample_task, 
	  run_sample_task, change_sample_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_sample_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_sample_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  if (firsttime){
    firsttime = FALSE;
    freq = 0.1; // frequency
    amp  = 0.5; // amplitude
  }

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
	 start_time, task_servo_time);

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_sample_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_sample_task(void)
{
  int j, i;

  double task_time;
  double omega;
  int    dof;

  // NOTE: all array indices start with 1 in SL

  task_time = task_servo_time - start_time;
  omega     = 2.0*PI*freq;

  // osciallates one DOF 
  dof = 1;
  for (i=dof; i<=dof; ++i) {
    target[i].th   = joint_default_state[i].th +
      amp*sin(omega*task_time);
    target[i].thd   = amp*omega*cos(omega*task_time);
    target[i].thdd  =-amp*omega*omega*sin(omega*task_time);
  }

  // the following variables need to be assigned
  for (i=1; i<=N_DOFS; ++i) {
    joint_des_state[i].th   = target[i].th;
    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].thdd = target[i].thdd;
    joint_des_state[i].uff  = 0.0;
  }

  // compute inverse dynamics torques
  SL_InvDynNE(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_sample_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_sample_task(void)
{
  int    ivar;
  double dvar;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}

