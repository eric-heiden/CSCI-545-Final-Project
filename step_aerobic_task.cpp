#ifndef STEP_AEROBIC_TASK_H
#define STEP_AEROBIC_TASK_H

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

//static Joints pose_left_forward;
//static Joints pose_left_down;
static double delta_t = 0.01;
static double duration = 4.0;
static double time_to_go = 0.0;

#include "helper.hpp"
#include "stepsequence.hpp"

static StepSequence sequence;

const double footLiftMultiplier = 1.5;

namespace step_aerobic
{
    void raiseRightArm()
    {
        target = pose_cog_middle;
        lower_arm_target(Left);
        raise_arm_target(Right);
    }

    void raiseLeftArm()
    {
        target = pose_cog_middle;
        lower_arm_target(Right);
        raise_arm_target(Left);
    }

    void cogRight()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            set_ik_target(RIGHT_FOOT);
        }
        else
            target = pose_cog_right;
    }

    void cogLeft()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            set_ik_target(LEFT_FOOT);
        }
        else
            target = pose_cog_left;
    }

    void liftLeftFoot()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            pose_cog_right = target;
        }

        target.copyFrom(joint_des_state);
        target.joints[R_HAA].th = -.2 * footLiftMultiplier;
        target.joints[L_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[L_AAA].th = 0.2 * footLiftMultiplier;
    }

    void liftRightFoot()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            pose_cog_left = target;
        }

        target.copyFrom(joint_des_state);
        target.joints[L_HAA].th = -.2 * footLiftMultiplier;
        target.joints[R_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[R_AAA].th = -0.2 * footLiftMultiplier;
    }
}

// allocate memory, etc. called once when task starts.
static int init_step_aerobic_task(void)
{
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
        initializeSL();
    }

    // define steps
    sequence.add(new Step("Crouch", &crouch, &step_min_jerk_jointspace, duration, delta_t));
    sequence.add(new Step("Raise Right Arm", &step_aerobic::raiseRightArm, &step_min_jerk_jointspace, duration/2, delta_t));
    sequence.add(new Step("COG Right", &step_aerobic::cogRight, &step_cog_ik, duration/2, delta_t));
    sequence.add(new Step("Lift Left Foot", &step_aerobic::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    sequence.add(new Step("Raise Left Arm", &step_aerobic::raiseLeftArm, &step_min_jerk_jointspace, duration/2, delta_t));
    sequence.add(new Step("COG Left", &step_aerobic::cogLeft, &step_cog_ik, duration/2, delta_t));
    sequence.add(new Step("Lift Right Foot", &step_aerobic::liftRightFoot, &step_min_jerk_jointspace, duration/2, delta_t));

    // all in joint space
    StepSequence *jointLoop = new StepSequence(true); // cycle
    jointLoop->add(new Step("Raise Right Arm", &step_aerobic::raiseRightArm, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("COG Right", &step_aerobic::cogRight, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lift Left Foot", &step_aerobic::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Raise Left Arm", &step_aerobic::raiseLeftArm, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("COG Left", &step_aerobic::cogLeft, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lift Right Foot", &step_aerobic::liftRightFoot, &step_min_jerk_jointspace, duration/2, delta_t));

    sequence.add(jointLoop);

    sequence.initialize();

    return TRUE;
}

static int run_step_aerobic_task(void)
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
static int change_step_aerobic_task(void)
{
    int    ivar = 0; // default value
    double dvar = 0; // default value

    get_int("This is how to enter an integer variable", ivar, &ivar);
    get_double("This is how to enter a double variable", dvar, &dvar);

    return TRUE;
}

// declare with C linkage
extern "C" void add_step_aerobic_task()
{
    static bool firsttime = true;

    if (firsttime)
    {
        firsttime = false;

        addTask("Step Aerobic Task", init_step_aerobic_task,
            run_step_aerobic_task, change_step_aerobic_task);
    }
}

#endif // STEP_AEROBIC_TASK_H
