/**
 * \file frne.c
 * \author Peter Corke
 * \brief MEX file body
 *
 *
 *  FRNE     MEX file version of RNE.M
 *
 *  TAU = FRNE(ROBOT, Q, QD, QDD)
 *  TAU = FRNE(ROBOT, [Q QD QDD])
 *
 *  where   Q, QD and QDD are row vectors of the manipulator state; pos,
 *  vel, and accel.
 *
 *  Returns the joint torque required to achieve the specified joint 
 *  position, velocity and acceleration state.  Gravity is taken
 *  from the robot object.
 *
 *  TAU = RNE(ROBOT, Q, QD, QDD, GRAV)
 *  TAU = RNE(ROBOT, [Q QD QDD], GRAV)
 *
 *  GRAV overrides the gravity vector in the robot object.
 *
 *  An external force/moment acting on the end of the manipulator may 
 *  also be specified by a 6-element vector [Fx Fy Fz Mx My Mz].
 *
 *  TAU = RNE(ROBOT, Q, QD, QDD, GRAV, FEXT)
 *  TAU = RNE(ROBOT, [Q QD QDD], GRAV, FEXT)
 *
 */

/*
 * Copyright (C) 1999-2008, by Peter I. Corke
 *
 * This file is part of The Robotics Toolbox for Matlab (RTB).
 * 
 * RTB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RTB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Leser General Public License
 * along with RTB.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mex.h"
#include <math.h>

#include    "frne.h"

/*
#define DEBUG
*/

/* Input Arguments */
#define WAM_INFO_IN    prhs[0]
#define A1_IN       prhs[1]
#define A2_IN       prhs[2]
#define A3_IN       prhs[3]
#define A4_IN       prhs[4]
#define A5_IN       prhs[5]

/* Output Arguments */
#define TAU_OUT plhs[0]

/* Some useful things */
#define NUMROWS(x)  mxGetM(x)
#define NUMCOLS(x)  mxGetN(x)
#define NUMELS(x)   (mxGetN(x)*mxGetM(x))
#define POINTER(x)  mxGetPr(x)

/* forward defines */
static void rot_mat (Link *l, double th, double d, DHType type);
static int mstruct_getfield_number(mxArray *m, char *field);
static int mstruct_getint(mxArray *m, int i, char *field);
static double mstruct_getreal(mxArray *m, int i, char *field);
static double * mstruct_getrealvect(mxArray *m, int i, char *field);
void error(char *s, ...);
void printLink(Link* l,int ind);


/* default values for gravity and external load */

/**
 * MEX function entry point.
 */
void 
mexFunction(
    int     nlhs,
    mxArray     *plhs[],
    int     nrhs,
    const mxArray   *prhs[]
) {
    double  *q, *qd, *qdd;
    double  *tau;
    int j, njoints, p, nq;
    double  *fext = NULL;
    double grav[3] = {0,0,9.81};
    Robot       robot;
    mxArray     *link0;
    mxArray     *mx_robot;
    mxArray     *mx_links;
    mxArray     *mx_arm;
    mxArray*  mx_arm_dh;
    mxArray*  mx_arm_dyn;
    double*   arm_dh_a;
    double*   arm_dh_alpha;
    double*   arm_dh_d;
    double*   arm_dyn_B;
    double*   arm_dyn_I; //???
    double*   arm_dyn_Jm;
    double*   arm_dyn_m;
    double*   arm_dyn_r;
    static int  first_time = 0;
    
    mx_robot = (mxArray *)WAM_INFO_IN;
    
    njoints = 7;
    
    
/***********************************************************************
 * Handle the different calling formats.
 * Setup pointers to q, qd and qdd inputs 
 ***********************************************************************/
    switch (nrhs) {
    case 2:
    /*
     * TAU = RNE(WAM_INFO_IN, [Q QD QDD])
     */ 
        if (NUMCOLS(A1_IN) != 3 * njoints)
            mexErrMsgTxt("RNE too few cols in [Q QD QDD]");
        q = POINTER(A1_IN);
        nq = NUMROWS(A1_IN);
        qd = &q[njoints*nq];
        qdd = &q[2*njoints*nq];
        break;
    default:
        mexErrMsgTxt("RNE wrong number of arguments.");
    }


    /*
     * fill out the robot structure
     */
    robot.njoints = njoints;
    robot.gravity = (Vect *)grav;
    robot.dhtype = 0;
    /* build link structure */
    robot.links = (Link *)mxCalloc((mwSize) njoints, (mwSize) sizeof(Link));


/***********************************************************************
 * Now we have to get pointers to data spread all across a cell-array
 * of Matlab structures.
 *
 * Matlab structure elements can be found by name (slow) or by number (fast).
 * We assume that since the link structures are all created by the same
 * constructor, the index number for each element will be the same for all
 * links.  However we make no assumption about the numbers themselves.
 ***********************************************************************/   
    
    /*
     * Elements of the link structure are:
     *
     *  alpha: 
     *  A:
     *  theta:
     *  D:
     *  offset:
     *  sigma:
     *  mdh:
     *  m:
     *  r:
     *  I:
     *  Jm:
     *  G:
     *  B:
     *  Tc:
     */

    if (first_time == 0) {
        mexPrintf("Fast RNE: (c) Peter Corke 2002-2011\n");
        first_time = 1;
    }
   
    /*
     * copy data from the wamInfo structure into the local links structure
     * to save function calls later
     */
    
    /* Get the kinematic and dynamic parameters from the wamInfo struct */
    mx_arm       = (mxArray*)mxGetField(mx_robot,(mwIndex) 0,"arm");
    mx_arm_dh    = (mxArray*)mxGetField(mx_arm, (mwIndex) 0, "dh");
    mx_arm_dyn   = (mxArray*)mxGetField(mx_arm, (mwIndex) 0, "dyn");
    
    arm_dh_a     = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "a"));
    arm_dh_alpha = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "alpha"));
    arm_dh_d     = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "d"));
    
    arm_dyn_B    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "B"));
    arm_dyn_I    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "I"));
    arm_dyn_Jm   = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "Jm"));
    arm_dyn_m    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "m"));
    arm_dyn_r    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "r"));
    
    /* Assign all of the parametrs to the links in the robot structure */
    for (j=0; j<njoints; j++)
    {
        Link    *l = &robot.links[j];
        double Tc[2] = {0.0,0.0};
        l->alpha =  arm_dh_alpha[j];
        l->A =      arm_dh_a[j];
        l->theta =  0.0;
        l->D =      arm_dh_d[j];
        l->sigma =  0;
        l->offset = 0.0;
        l->m =      arm_dyn_m[j];
        l->rbar =   (Vect *)&(arm_dyn_r[3*j]);
        l->I =      &(arm_dyn_I[9*j]);
        l->Jm =     arm_dyn_Jm[j];
        l->G =      0.0;
        l->B =      arm_dyn_B[j];
        l->Tc =     Tc;
        //printLink(l,j);
    }
    
    /* Create a matrix for the return argument */
    TAU_OUT = mxCreateDoubleMatrix((mwSize) nq, (mwSize) njoints, mxREAL);
    tau = mxGetPr(TAU_OUT);

#define MEL(x,R,C)  (x[(R)+(C)*nq])

    /* for each point in the input trajectory */
    for (p=0; p<nq; p++) {
        /*
         * update all position dependent variables
         */
        for (j = 0; j < njoints; j++) {
            Link    *l = &robot.links[j];
            switch (l->sigma) {
            case REVOLUTE:
                rot_mat(l, MEL(q,p,j)+l->offset, l->D, robot.dhtype);
                break;
            case PRISMATIC:
                rot_mat(l, l->theta, MEL(q,p,j)+l->offset, robot.dhtype);
                break;
            }
#ifdef  DEBUG
            rot_print("R", &l->R);
            vect_print("p*", &l->r);
#endif
        }
        newton_euler(&robot, &tau[p], &qd[p], &qdd[p], fext, nq);
    }
    
    mxFree(robot.links);

}

/*
 *  Written by;
 *
 *      Peter I. Corke
 *      CSIRO Division of Manufacturing Technology
 *      Preston, Melbourne.  Australia. 3072.
 *
 *      pic@mlb.dmt.csiro.au
 *
 *  Permission to use and distribute is granted, provided that this message
 * is retained, and due credit given when the results are incorporated in
 * publised work.
 *
 */

/**
 * Return the link rotation matrix and translation vector.
 *
 * @param l Link object for which R and p* are required.
 * @param th Joint angle, overrides value in link object
 * @param d Link extension, overrides value in link object
 * @param type Kinematic convention.
 */
static void
rot_mat (
    Link    *l,
    double  th,
    double  d,
    DHType  type
) {
    double      st, ct, sa, ca;

#ifdef  sun
    sincos(th, &st, &ct);
    sincos(l->alpha, &sa, &ca);
#else
    st = sin(th);
    ct = cos(th);
    sa = sin(l->alpha);
    ca = cos(l->alpha);
#endif

    switch (type) {
case STANDARD:
    l->R.n.x = ct;      l->R.o.x = -ca*st;  l->R.a.x = sa*st;
    l->R.n.y = st;      l->R.o.y = ca*ct;   l->R.a.y = -sa*ct;
    l->R.n.z = 0.0;     l->R.o.z = sa;      l->R.a.z = ca;

    l->r.x = l->A;
    l->r.y = d * sa;
    l->r.z = d * ca;
    break;
case MODIFIED:
    l->R.n.x = ct;      l->R.o.x = -st;     l->R.a.x = 0.0;
    l->R.n.y = st*ca;   l->R.o.y = ca*ct;   l->R.a.y = -sa;
    l->R.n.z = st*sa;   l->R.o.z = ct*sa;   l->R.a.z = ca;

    l->r.x = l->A;
    l->r.y = -d * sa;
    l->r.z = d * ca;
    break;
    }
}

/*************************************************************************
 * Matlab structure access methods, get the field from joint i
 *************************************************************************/
static mxArray *
mstruct_get_element(mxArray *m, int j, char *field)
{
    mxArray *e;

    if ((e = mxGetProperty(m, (mwIndex)j, field)) != NULL)
        return e;
    else {
        error("No such field as %s", field);
    }
}

static int
mstruct_getfield_number(mxArray *m, char *field)
{
    int f;
    
    if ((f = mxGetFieldNumber(m, field)) < 0)
        error("no element %s in link structure");

    return f;
}

static int
mstruct_getint(mxArray *m, int i, char *field)
{
    mxArray *e;

    e = mstruct_get_element(m, i, field);

    return (int) mxGetScalar(e);
}

static double
mstruct_getreal(mxArray *m, int i, char *field)
{
    mxArray *e;

    e = mstruct_get_element(m, i, field);

    return mxGetScalar(e);
}

static double *
mstruct_getrealvect(mxArray *m, int i, char *field)
{
    mxArray *e;

    e = mstruct_get_element(m, i, field);

    return mxGetPr(e);
}

#include    <stdarg.h>

/**
 * Error message handler.  Takes printf() style format string and variable
 * arguments and sends resultant string to Matlab via \t mexErrMsgTxt().
 *
 * @param s Error message string, \t  printf() style.
 */
void
error(char *s, ...)
{
    char    b[BUFSIZ];

    va_list ap;

    va_start(ap, s);

    vsprintf(b, s, ap);

    mexErrMsgTxt(b);
}

void printLink(Link* l,int ind)
{
  mexPrintf("\n-----------  Link %d  -----------\n",ind);
  mexPrintf(" alpha: %10.4f\n",l->alpha);
  mexPrintf("     A: %10.4f\n",l->A);
  mexPrintf("     D: %10.4f\n",l->D);
  mexPrintf(" theta: %10.4f\n",l->theta);
  mexPrintf("offset: %10.4f\n",l->offset);
  mexPrintf(" sigma: %10.4d\n",l->sigma);
  mexPrintf("  rbar: %10.4f  %10.4f  %10.4f\n",l->rbar[0],l->rbar[1],l->rbar[2]);
  mexPrintf("\n     I: %10.4f  %10.4f  %10.4f\n",l->I[0],l->I[1],l->I[2]);
  mexPrintf("      : %10.4f  %10.4f  %10.4f\n",l->I[3],l->I[4],l->I[5]);
  mexPrintf("      : %10.4f  %10.4f  %10.4f\n\n",l->I[6],l->I[7],l->I[8]);
  mexPrintf("     m: %10.4f\n",l->m);
  mexPrintf("    Jm: %10.4f\n",l->Jm);
  mexPrintf("     G: %10.4f\n",l->G);
  mexPrintf("     B: %10.4f\n",l->B);
  mexPrintf("    Tc: %10.4f %10.4f\n",l->Tc[0],l->Tc[1]);
  mexPrintf("\n---------------------------------\n\n");
}
