/*
* This file provides the limit3_planner() designed by
* John Morris as a refactor of the original limit3.comp
* by John Kasunich.
*
* The code is used by for the hal limit3 component and
* conditionally for single joint or axis jogging and
* homing as enabled by hal pin motion.simple-tp-method
*/

#include "simple_tp.h"
#include "rtapi_math.h"

#define SET_NEXT_STATE(p, _out, _in)           \
    do {                                      \
        *p->L3_out_old    =  *p->L3_curr_pos; \
        *p->L3_curr_pos   =  _out;            \
        *p->L3_in_pos_old =  _in;             \
        \
        *p->L3_curr_vel   =  (*p->L3_curr_pos - *p->L3_out_old)/period; \
        *p->L3_active     =  fabs(*p->L3_curr_pos - *p->L3_pos_cmd)     \
                          >= TINY_DP(*p->L3_max_acc,period);            \
        return;                                                         \
    } while (0)

#define VALID_NEXT(pos) ((pos) <= max_pos && (pos) >= min_pos)

// Distance = avg. velocity * time
#define S_GIVEN_VI_VF_T(vi,vf,t) (((vf) + (vi))/2 * (t))
// Time = chg. velocity / acceleration
#define T_GIVEN_VI_VF_A(vi,vf,a) (((vf) - (vi)) / (a))
// Final velocity = initial velocity + acceleration * time
#define VF_GIVEN_VI_A_T(vi,a,t) ((vi) + (a)*(t))
// A fudge amount for division errors
#define EPSILON 1e-9

struct limit3_parms {
    double *L3_pos_cmd;      // limit3 pin: in
    double *L3_curr_pos;     // limit3 pin: out
    double *L3_min_pos;      // limit3 pin: min
    double *L3_max_pos;      // limit3 pin: max
    double *L3_max_vel;      // limit3 pin: maxv
    double *L3_max_acc;      // limit3 pin: maxa
    int    *L3_smooth_steps; // limit3 pin: smooth_steps
    double *L3_curr_vel;     // limit3 instance variable
    double *L3_in_pos_old;   // limit3 instance variable
    double *L3_out_old;      // limit3 instance variable
    double *L3_out_vel;      // limit3 instance variable
    int    *L3_active;       // limit3 instance variable
};

void limit3_planner(struct limit3_parms *p,double period)
{
//Note: the code design below replicates the Morris code for
//      limit3.comp:FUNCTION(_) using the following
//      capitalized items for all per-instance state
#define OUTPUT       *p->L3_curr_pos
#define INPUT        *p->L3_pos_cmd
#define MINPOS       *p->L3_min_pos
#define MAXPOS       *p->L3_max_pos
#define MAXVEL       *p->L3_max_vel
#define MAXACC       *p->L3_max_acc
#define SMOOTH_STEPS *p->L3_smooth_steps
#define IN_POS_OLD   *p->L3_in_pos_old
#define OUT_OLD      *p->L3_out_old
#define OUT_VEL      *p->L3_out_vel

    double in_vel;
    double min_vel, max_vel, min_pos, max_pos;
    double stop_pos_max, stop_pos_min;
    double stop_time_max, stop_time_min;
    double in_vel_time_max, in_vel_time_min;
    double out_pos_max, out_pos_min, in_pos_max, in_pos_min;
    double ach_pos_min, ach_pos_max;

    OUT_VEL = (OUTPUT-OUT_OLD)/period;
    double goal_pos_min, goal_pos_max, goal_pos_cur;
    double pos_diff, vel_diff, goal_pos_prev;
    double t, ti, a, v, s;

    // Principal of operation:
    //
    // 1. Calculate shortest distance (at max acceleration) to
    //    stop (i.e. reach vel=0) and to match the input velocity
    // 2. Compare our projected positions and choose whether to worry
    //    about the max/min limits or to follow the input signal
    // 3. Adjust acceleration according to decision and return

    // 1.  Calculate distances and times to stop and match input velocity
    //
    // Input and output velocity
    in_vel = (INPUT - IN_POS_OLD) / period;
    OUT_VEL = (OUTPUT - OUT_OLD) / period;
    //
    // Most negative/positive velocity reachable in one period
    min_vel = fmax(VF_GIVEN_VI_A_T(OUT_VEL, -MAXACC, period), -MAXVEL);
    max_vel = fmin(VF_GIVEN_VI_A_T(OUT_VEL,  MAXACC, period),  MAXVEL);
    // Most negative/positive position reachable in one period
    // - cur. pos + (distance to reach min/max vel in one period)
    min_pos = OUTPUT + min_vel * period;
    max_pos = OUTPUT + max_vel * period;
    //
    // Shortest possible distance to stop
    // - time to decel to 0; start from previous period
    stop_time_max = fabs(T_GIVEN_VI_VF_A(max_vel, 0.0, MAXACC)) + period;
    stop_time_min = fabs(T_GIVEN_VI_VF_A(min_vel, 0.0, MAXACC)) + period;
    // - distance to stop from max_pos/min_pos
    stop_pos_max = OUTPUT + S_GIVEN_VI_VF_T(max_vel, 0.0, stop_time_max);
    stop_pos_min = OUTPUT + S_GIVEN_VI_VF_T(min_vel, 0.0, stop_time_min);
    //
    // Shortest possible distance to match input velocity
    // - time to match input velocity from this period; out runs 1 period behind
    in_vel_time_max = fabs(T_GIVEN_VI_VF_A(max_vel, in_vel, MAXACC)) - period;
    in_vel_time_min = fabs(T_GIVEN_VI_VF_A(min_vel, in_vel, MAXACC)) - period;
    // - output position after velocity match
    out_pos_max = max_pos + S_GIVEN_VI_VF_T(max_vel, in_vel, in_vel_time_max);
    out_pos_min = min_pos + S_GIVEN_VI_VF_T(min_vel, in_vel, in_vel_time_min);
    // - input position after velocity match
    in_pos_max = INPUT + in_vel * in_vel_time_max;
    in_pos_min = INPUT + in_vel * in_vel_time_min;

    // 2. Choose the current goal:  input signal, max limit or min limit
    //
    // Min/Max limit:
    // - assume we're stopping at a limit by default
    vel_diff = -OUT_VEL;
    ach_pos_min = stop_pos_min;
    ach_pos_max = stop_pos_max;
    // - are we headed to crash into a min/max limit?
    if (stop_pos_max > MAXPOS + EPSILON && !VALID_NEXT(MAXPOS))
        goal_pos_min = goal_pos_max = goal_pos_cur = goal_pos_prev = MAXPOS;
    else if (stop_pos_min < MINPOS - EPSILON && !VALID_NEXT(MINPOS))
        goal_pos_min = goal_pos_max = goal_pos_cur = goal_pos_prev = MINPOS;
    // - if input is outside min/max limit but heading back in, is
    //   there time to keep heading toward the limit before we need to
    //   start running to meet the input signal?
    else if (INPUT >= MAXPOS && in_pos_max > out_pos_max)
        goal_pos_min = goal_pos_max = goal_pos_cur = goal_pos_prev = MAXPOS;
    else if (INPUT <= MINPOS && in_pos_min < out_pos_min)
        goal_pos_min = goal_pos_max = goal_pos_cur = goal_pos_prev = MINPOS;
    //
    // Input signal:
    // - no min/max constraints; chase the input signal.
    else {
        goal_pos_min = in_pos_min;
        goal_pos_max = in_pos_max;
        goal_pos_cur = INPUT;
        goal_pos_prev = IN_POS_OLD;
        vel_diff = OUT_VEL - in_vel;
        ach_pos_min = out_pos_min;
        ach_pos_max = out_pos_max;
    }

    // 3.  Adjust acceleration
    //
    // - Difference in position, last cycle
    pos_diff = OUTPUT - goal_pos_prev;
    // - Time to reach goal position and velocity with uniform acceleration
    if (fabs(vel_diff) < EPSILON)
        t = 0;
    else
        t = pos_diff / ((vel_diff + 0) / 2); // dp / (avg dv)

    // - If current position and velocity are close enough to reach
    //   goal position in this period, and maintaining goal velocity
    //   in the next period doesn't violate acceleration constraints,
    //   pass the input straight to the output
    if (VALID_NEXT(goal_pos_cur) && fabs(t) <= period)
            SET_NEXT_STATE(p,goal_pos_cur, INPUT);

    // - If no danger of overshoot, accel at max in direction of goal
    if (ach_pos_max < goal_pos_max + EPSILON)
        // Max pos. accel toward goal will still fall short
        SET_NEXT_STATE(p,max_pos, INPUT);
    if (ach_pos_min > goal_pos_min - EPSILON)
        // Max neg. accel toward goal will still fall short
        SET_NEXT_STATE(p,min_pos, INPUT);

    // - If close to reaching goal, try to grease a landing; always
    //   using max acceleration can result in oscillating around the
    //   goal but never quite getting things right to 'lock' onto it
    if (fabs(t) < period * SMOOTH_STEPS) {
        // - Round up the magnitude of time to an integral number of periods
#       define SIGN(n) (((n)>=0) ? 1 : -1)
        ti = (int)((t - EPSILON*SIGN(t)) / period + SIGN(t)) * period;
        // - Uniform acceleration to reach goal in time `ti`
        a = (vel_diff - 0) / ti;
        v = OUT_VEL + a * period;
        s = v * period;
        // - Effect new position, within limits
        SET_NEXT_STATE(p,fmin(max_pos, fmax(min_pos, OUTPUT + s)), INPUT);
    }

    // - If moving toward goal and in danger of overshoot, accelerate
    //   at max in opposite direction of goal
    if (goal_pos_max + EPSILON < ach_pos_max && goal_pos_prev > OUTPUT)
            // Heading up from below
        SET_NEXT_STATE(p,min_pos, INPUT);
    if (goal_pos_min - EPSILON > ach_pos_min && goal_pos_prev < OUTPUT)
            // Heading down from above
        SET_NEXT_STATE(p,max_pos, INPUT);

    // - Shouldn't get here; coast
    SET_NEXT_STATE(p,(min_pos+max_pos)/2, INPUT);
}
