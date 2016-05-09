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
#define SIMULATION 1

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
    void cogRight()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            set_ik_target(RIGHT_FOOT, 0.7);
            firstRun = false;
        }
        else
            set_ik_target(RIGHT_FOOT, 0.7, 0.05);
    }

    void cogLeftFarOut()
    {
        set_ik_target(LEFT_FOOT, 1.05, 0.02);
        //target = pose_cog_right;
        //mirror_target();
    }

    void cogLeft()
    {
        set_ik_target(LEFT_FOOT);
    }


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

    void leftForward()
    {
        target.joints[R_AFE].th = 0.4 * footLiftMultiplier;
        //target.joints[R_HAA].th = -.2 * footLiftMultiplier;
        target.joints[L_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[L_AAA].th = 0.2 * footLiftMultiplier;
    }

    void rightForward()
    {
        target.joints[L_AFE].th = 0.4 * footLiftMultiplier;
        //target.joints[L_HAA].th = -.2 * footLiftMultiplier;
        target.joints[R_AFE].th = 0.4 * footLiftMultiplier;
        target.joints[R_AAA].th = -0.35 * footLiftMultiplier;

        target.joints[R_HFE].th = 0.80;
        target.joints[R_KFE].th = 0.80;
    }

    void leanLeftForward()
    {
        static bool firstRun = true;

        target.joints[L_FB].th = 0.2 * footLiftMultiplier;
        target.joints[R_FB].th = -.2 * footLiftMultiplier;
        if (firstRun)
        {
            target.joints[R_AFE].th = 0.45 * footLiftMultiplier;
            firstRun = false;
        }
        else
            target.joints[R_AFE].th = 0.3 * footLiftMultiplier;

        target.joints[L_AFE].th = 0.2 * footLiftMultiplier;

        target.joints[R_HFE].th = target.joints[L_HFE].th = 0.3;
    }

    void leanRightForward()
    {
        target.joints[R_FB].th = 0.2 * footLiftMultiplier;
        target.joints[L_FB].th = -.2 * footLiftMultiplier;
        target.joints[L_AFE].th = 0.37 * footLiftMultiplier;

        target.joints[R_AFE].th = 0.25 * footLiftMultiplier;

        target.joints[R_HFE].th = target.joints[L_HFE].th = 0.3;
    }

    void leanLeftForward2()
    {
        target.joints[R_HFE].th = target.joints[L_HFE].th = 0.32;

        target.joints[L_AFE].th = 0.0 * footLiftMultiplier;
        target.joints[R_AFE].th = 0.65 * footLiftMultiplier;

        target.joints[L_KFE].th = 0.35;
        target.joints[R_KFE].th = 0.85;

        target.joints[R_AAA].th = -0.1 * footLiftMultiplier;

        //target.joints[L_HAA].th = -.1 * footLiftMultiplier;
        //target.joints[R_HAA].th = -.2 * footLiftMultiplier;

        target.joints[L_FB].th = 0 * footLiftMultiplier;
        target.joints[R_FB].th = 0 * footLiftMultiplier;
    }

    void leanRightForward2()
    {
        target.joints[R_HFE].th = target.joints[L_HFE].th = 0.32;

        target.joints[R_AFE].th = 0.0 * footLiftMultiplier;
        target.joints[L_AFE].th = 0.55 * footLiftMultiplier;

        target.joints[R_KFE].th = 0.55;
        target.joints[L_KFE].th = 0.75;

        target.joints[L_AAA].th = -0.1 * footLiftMultiplier;

        target.joints[R_HAA].th = -.1 * footLiftMultiplier;
        target.joints[L_HAA].th = -.2 * footLiftMultiplier;

        target.joints[L_FB].th = 0 * footLiftMultiplier;
        target.joints[R_FB].th = 0 * footLiftMultiplier;
    }

    void stabilizeLeft()
    {
        target.joints[L_HAA].th = 0 * footLiftMultiplier;
        target.joints[R_HAA].th = 0 * footLiftMultiplier;


       // target.joints[R_HFE].th = target.joints[L_HFE].th = 0.3;

        //target.joints[R_AFE].th = 0.6 * footLiftMultiplier;
        //target.joints[L_AFE].th = 0.15 * footLiftMultiplier;

        //target.joints[L_HAA].th = 0.1 * footLiftMultiplier;
        //target.joints[R_HAA].th = -.1 * footLiftMultiplier;

        //target.joints[L_AAA].th = -.1 * footLiftMultiplier;
        //target.joints[R_AAA].th = -.1 * footLiftMultiplier;


        //target.joints[L_KFE].th = 0.35;
        //target.joints[R_KFE].th = 0.75;
    }

    void stabilizeRight()
    {
        target.joints[L_HAA].th = 0 * footLiftMultiplier;
        target.joints[R_HAA].th = 0 * footLiftMultiplier;
       // target.joints[R_HFE].th = target.joints[L_HFE].th = 0.3;

        //target.joints[R_AFE].th = 0.6 * footLiftMultiplier;
        //target.joints[L_AFE].th = 0.15 * footLiftMultiplier;

        //target.joints[L_HAA].th = 0.1 * footLiftMultiplier;
        //target.joints[R_HAA].th = -.1 * footLiftMultiplier;

        //target.joints[L_AAA].th = -.1 * footLiftMultiplier;
        //target.joints[R_AAA].th = -.1 * footLiftMultiplier;


        //target.joints[L_KFE].th = 0.35;
        //target.joints[R_KFE].th = 0.75;
    }

    void stabilizeBeforeRight()
    {
        target.joints[R_HFE].th = 0.25;
        target.joints[R_KFE].th = 0.56;

        target.joints[L_AFE].th = 0.16;

        target.joints[R_AFE].th = 0.43;

        target.joints[R_AAA].th = -.31;

        target.joints[L_HFE].th = 0.30;
        target.joints[L_KFE].th = 0.60;
        target.joints[L_AFE].th = 0.30;

        //target.joints[L_AAA] = joint_default_state[L_AAA];

       // target.joints[R_HFE].th = target.joints[L_HFE].th = 0.3;

        //target.joints[R_AFE].th = 0.6 * footLiftMultiplier;
        //target.joints[L_AFE].th = 0.15 * footLiftMultiplier;

        //target.joints[L_HAA].th = 0.1 * footLiftMultiplier;
        //target.joints[R_HAA].th = -.1 * footLiftMultiplier;

        //target.joints[L_AAA].th = -.1 * footLiftMultiplier;
        //target.joints[R_AAA].th = -.1 * footLiftMultiplier;


        //target.joints[L_KFE].th = 0.35;
        //target.joints[R_KFE].th = 0.75;
    }

    void liftLeftFoot()
    {
        static bool firstRun = true;
        if (firstRun)
        {
            firstRun = false;
            pose_cog_right = target;

            target.copyFrom(joint_des_state);
            target.joints[R_HAA].th = -.2 * footLiftMultiplier;
            target.joints[L_AFE].th = 0.3 * footLiftMultiplier;
            target.joints[L_AAA].th = 0.2 * footLiftMultiplier;
        }
        else
        {
            target.joints[R_HAA].th = -.07 * footLiftMultiplier;
            target.joints[L_HFE].th = 0.50;
            target.joints[L_KFE].th = 1.00;
            target.joints[L_AFE].th = 0.50;
        }
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
        target.joints[L_HAA].th = -.07 * footLiftMultiplier;
        //target.joints[R_AFE].th = 0.3 * footLiftMultiplier;
        //target.joints[R_AAA].th = -0.37 * footLiftMultiplier;

        target.joints[R_HFE].th = 0.50;
        target.joints[R_KFE].th = 1.00;
        target.joints[R_AFE].th = 0.50;
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
    sequence.add(new Step("Crouch", &walking::slight_crouch, &step_min_jerk_jointspace, duration/2, delta_t));


    // all in joint space
    StepSequence *jointLoop = new StepSequence(false); // cycle
    jointLoop->add(new Step("COG Right", &walking::cogRight, &step_cog_ik, duration, delta_t));
    jointLoop->add(new Step("Lift Left Foot", &walking::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Left Forward", &walking::leftForward, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lean Left Forward", &walking::leanLeftForward, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lean Left Forward 2", &walking::leanLeftForward2, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Stabilize Left", &walking::stabilizeLeft, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("COG Left", &walking::cogLeftFarOut, &step_cog_ik, duration, delta_t));

    jointLoop->add(new Step("Stabilize Before Right", &walking::stabilizeBeforeRight, &step_min_jerk_jointspace, duration, delta_t));
    //jointLoop->add(new Step("Slight Crouch", &walking::slight_crouch, &step_min_jerk_jointspace, duration, delta_t));
    //jointLoop->add(new Step("COG Left", &walking::cogLeft, &step_cog_ik, duration, delta_t));

    jointLoop->add(new Step("Lift Right Foot", &walking::liftRightFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Right Forward", &walking::rightForward, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Lean Right Forward", &walking::leanRightForward, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Lean Right Forward 2", &walking::leanRightForward2, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Stabilize Right", &walking::stabilizeRight, &step_min_jerk_jointspace, duration/2, delta_t));
    /*jointLoop->add(new Step("Relax", &walking::liftLeftFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Lean Left", &walking::leanLeft, &step_min_jerk_jointspace, duration, delta_t));
    jointLoop->add(new Step("Lift Right Foot", &walking::liftRightFoot, &step_min_jerk_jointspace, duration/2, delta_t));
    jointLoop->add(new Step("Right Forward", &walking::rightForward, &step_min_jerk_jointspace, duration/2, delta_t));*/

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
