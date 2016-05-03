#ifndef WALKING_TASK_H
#define WALKING_TASK_H

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

//static Joints pose_left_forward;
//static Joints pose_left_down;
static double delta_t = 0.01;
static double duration = 4.0;
static double time_to_go = 0.0;

#include "helper.hpp"
#include "stepsequence.hpp"

static StepSequence sequence;

const double footLiftMultiplier = 1.0;

namespace walking
{
    void slight_crouch()
    {
        // copy the default pose into our struct to make it copyable
        pose_default.copyFrom(joint_default_state);

        target = pose_default;
        // crouch
        target.joints[R_HFE].th = target.joints[L_HFE].th = 0.30;
        target.joints[R_KFE].th = target.joints[L_KFE].th = 0.60;
        target.joints[R_AFE].th = target.joints[L_AFE].th = 0.30;
        pose_cog_middle = target;
    }

    void relax()
    {
        target = pose_cog_middle;
    }

    void leanRight()
    {

    }

    void leanLeft()
    {

    }

    void rightForward()
    {
        target.joints[L_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[L_HAA].th = -.2 * footLiftMultiplier;
        target.joints[R_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[R_AAA].th = -0.2 * footLiftMultiplier;
    }

    void leftForward()
    {
        target.joints[R_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[R_HAA].th = -.2 * footLiftMultiplier;
        target.joints[L_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[L_AAA].th = 0.2 * footLiftMultiplier;
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
static int init_walking_task(void)
{
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
        initializeSL();
    }

    // define steps
    sequence.add(new Step("Crouch", &walking::slight_crouch, &step_min_jerk_jointspace, duration, delta_t));


    // all in joint space
    StepSequence *jointLoop = new StepSequence(true); // cycle
    jointLoop->add(new Step("Lean Right", &walking::leanRight, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Lift Left Foot", &walking::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Left Forward", &walking::leftForward, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Relax", &walking::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lean Left", &walking::leanLeft, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Lift Right Foot", &walking::liftRightFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Right Forward", &walking::rightForward, &step_min_jerk_jointspace, duration/2, delta_t));

    sequence.add(jointLoop);

    sequence.initialize();

    return TRUE;
}

static int run_walking_task(void)
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
static int change_walking_task(void)
{
    int    ivar = 0; // default value
    double dvar = 0; // default value

    get_int("This is how to enter an integer variable", ivar, &ivar);
    get_double("This is how to enter a double variable", dvar, &dvar);

    return TRUE;
}

// declare with C linkage
extern "C" void add_walking_task()
{
    static bool firsttime = true;

    if (firsttime)
    {
        firsttime = false;

        addTask("Walking Task", init_walking_task,
            run_walking_task, change_walking_task);
    }
}

#endif // WALKING_TASK_H
