#ifndef SHIFT_TASK_H
#define SHIFT_TASK_H

#include <iostream>

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

#define ENDEFF_IK 0
#define SIMULATION 0

// local variables
static double delta_t = 0.01;
static double duration = 7.0;
static double time_to_go = 0.0;

#include "helper.hpp"
#include "stepsequence.hpp"

static StepSequence sequence;

namespace shift
{
    void raiseRightArm()
    {
        lower_arm_target(Left);
        raise_arm_target(Right);
    }

    void cogRight()
    {
        set_ik_target(RIGHT_FOOT);
    }

    void cogLeft()
    {
        set_ik_target(LEFT_FOOT);
    }

    void raiseLeftArm()
    {
        pose_cog_right.copyFrom(joint_des_state);
        target = pose_cog_right;
        lower_arm_target(Right);
        raise_arm_target(Left);
    }

    void jointCogRight()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            // save current left COG state
            pose_cog_left.copyFrom(joint_des_state);
        }

        target = pose_cog_right;
    }

    void jointCogLeft()
    {
        target = pose_cog_left;
    }
}

// allocate memory, etc. called once when task starts.
static int init_shift_task(void)
{
    static bool firsttime = true;
    if (firsttime)
    {
        firsttime = false;

        initializeSL();
    }

    // define steps
    sequence.add(new Step("Crouch", &crouch, &step_min_jerk_jointspace, duration, delta_t));
    sequence.add(new Step("Raise Right Arm", &shift::raiseRightArm, &step_min_jerk_jointspace, duration/2, delta_t));
    sequence.add(new Step("COG Right", &shift::cogRight, &step_cog_ik, duration, delta_t));
    sequence.add(new Step("Raise Left Arm", &shift::raiseLeftArm, &step_min_jerk_jointspace, duration/2, delta_t));
    sequence.add(new Step("COG Left", &shift::cogLeft, &step_cog_ik, duration, delta_t));

    StepSequence *jointLoop = new StepSequence(true); // cycle
    jointLoop->add(new Step("Joint COG Right", &shift::jointCogRight, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Joint COG Left", &shift::jointCogLeft, &step_min_jerk_jointspace, duration, delta_t));
    sequence.add(jointLoop);

    sequence.initialize();

    return TRUE;
}

static int run_shift_task(void)
{
    sequence.execute();

    // this is the "normal" inverse dynamics used for drawing, etc.
    //SL_InvDynNE(joint_state, joint_des_state, endeff, &base_state, &base_orient);

    // this is a special inverse dynamics computation for a free standing robot
    // BUT... it doesn't seem to work unless both feet are stuck to the ground
    //inverseDynamicsFloat(delta_t, stat, TRUE, joint_des_state, NULL, NULL, fc);

    return TRUE;
}

// it's somehow possible to call this from the console... don't know how yet
static int change_shift_task(void)
{
    int    ivar = 0; // default value
    double dvar = 0; // default value

    get_int("This is how to enter an integer variable", ivar, &ivar);
    get_double("This is how to enter a double variable", dvar, &dvar);

    return TRUE;
}

// declare with C linkage
extern "C" void add_shift_task()
{
    static int firsttime = TRUE;

    if (firsttime) {
        firsttime = FALSE;

        addTask("Shift Task", init_shift_task,
            run_shift_task, change_shift_task);
    }
}

#endif // SHIFT_TASK_H
