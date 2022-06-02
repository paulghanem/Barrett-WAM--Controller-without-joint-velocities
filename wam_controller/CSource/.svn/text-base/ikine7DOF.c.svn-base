#include <math.h>
#include <stdio.h>
#include <mex.h>

/* Robot definitions */

#define JNTS 7         /* Number of degrees of freedom for WAM */
#define LINKS 3        /* Number of links of the WAM */
#define HJNTS 4        /* Number of degrees of freedom for Barrett Hand */
#define FINGERS 3      /* Number of fingers of Barrett Hand */
#define FJNTS 3        /* Number of joints per finger of Barrett Hand */
#define AXES 3         /* Force axes */

/* 
** link lengths and offsets (m)
*/ 
#define WAM_BASE_X          (0.22)   //Base to Arm frame
#define WAM_BASE_Y          (0.14)   //Base to Arm frame
#define WAM_BASE_Z          (0.346)  //Base to Arm frame
#define WAM_J3_OFFSET_X     (0.045)  //Barrett joint 3 X offset
#define WAM_L1              (0.55)   //Barrett first link length
#define WAM_L2              (0.3)    //Barrett second link length (first wrist part)
//#define WAM_L3              (0.061)   //Barrett third link length (second wrist part)
#define WAM_L3              (0.161)   //Barrett third link length (second wrist part)

#define SQ(x) ((x)*(x))
#define SQR(x) ((x)*(x))
#define INFINITE 9999
#define NEGLIGIBLE 0.0000001
#define MPI 3.141592653589793

// Actual Joint Limits for our WAM
//static double wam_theta_min[JNTS] = {-2.62,  -2.01,  -2.97,  -.87,  -4.79,  -1.57,  -3.0};
//static double wam_theta_max[JNTS] = { 2.62,   2.01,   2.97,  3.14,   1.27,   1.57,   3.0};

// Constrained Limits to force more applicable solutions to our workspace.  Also edited joint 7 for when Schunk gripper is attached
static double wam_theta_min[JNTS] = {-1.5708, -1.0472, -2.97, -.87, -4.79, -1.57, -2.2146};
static double wam_theta_max[JNTS] = { 1.5708,    2.01,  2.97, 3.14,  1.27,  1.57,  3.7854};

//static double wam_theta_min[JNTS] = {-2.66, -1.942, -2.704, -0.842, -4.811, -1.633, -2.2};
//static double wam_theta_min[JNTS] = {-2.66, -1.942, -2.704,      0, -4.811, -1.633, -2.2};
//static double wam_theta_max[JNTS] = { 2.66,  1.962,  2.904,  3.117,  1.261,  1.508,  2.2};

void DispRotz(double disp[3], double theta, double trans[4][4]);
void DispRoty(double disp[3], double theta, double trans[4][4]);
void HomTransMult(double trans1[4][4], double trans2[4][4], double result[4][4]);

void WAMForwardKinematics(double theta[JNTS], double x[4][4], double elbwrist[2][4][4]);

int WAMInverseKinematics(double theta2, double x[4][4], double theta_ref[JNTS], double theta[JNTS]);

//run IK to find joint angles to make a Cartesian position
//searches over joint angle 2 (first joint is 0) and solves the rest analytically
//tries to stay close to currentangles
//takes in a 4x4 homogeneous transformation matrix for the hand base (meters) as a 16-vector in row order (rot4)
//resulting angles in resultangles
//returns 1 if solution is found, 0 otherwise
int wam_ik(double rot4[16], double currentangles[7], double resultangles[7]);

//vect mag
double vect_mag(double vect[3]){
	return sqrt(SQ(vect[0])+SQ(vect[1])+SQ(vect[2]));
}

//vector sum
void vect_sum(double vect1[3], double vect2[3], double result[3]){
	int i;
	for(i=0; i<3; i++) result[i] = vect1[i]+vect2[i];
}

//cross product
void vect_cross(double vect1[3], double vect2[3], double result[3]){
	result[0] = vect1[1]*vect2[2] - vect1[2]*vect2[1];
	result[1] = vect1[2]*vect2[0] - vect1[0]*vect2[2];
	result[2] = vect1[0]*vect2[1] - vect1[1]*vect2[0];
}

//rotation error
double rot_diff(double rot1[4][4], double rot2[4][4]){
	double cross[3];
	double error[3] = {0,0,0};
	double vect1[3];
	double vect2[3];
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			vect1[j] = rot1[j][i];
			vect2[j] = rot2[j][i];
		}
		vect_cross(vect1, vect2, cross);
		vect_sum(error, cross, error);
	}
	return vect_mag(error);
}
						 

//check to see how far away the IK result is from the desired pos/rot
double ik_err(double tool[4][4], double resulttool[4][4]){
	int i, j;

	//difference in position
	double poserr = sqrt(SQ(tool[0][3]-resulttool[0][3])+SQ(tool[1][3]-resulttool[1][3])+SQ(tool[2][3]-resulttool[2][3]));

	//difference in rotation
	double roterr = rot_diff(tool, resulttool);

	//weight rotation versus position (1m = 2 radians)
	return poserr + .5*roterr;
}


//tries to find an ik solution, returns 1 if found and 0 otherwise
int find_ik_solution(double theta2, double tool[4][4], double currentangles[7], double resultangles[7]){
	int verbose = 0;
	int err;
	double errmag;
	double resulttool[4][4];

	err = WAMInverseKinematics(theta2, tool, currentangles, resultangles);

	WAMForwardKinematics(resultangles, resulttool, NULL);

	errmag = ik_err(tool, resulttool);

	if(err > 0 && errmag <= .001) return 1;
	return 0;
}


//run IK to find joint angles to make a Cartesian position
//searches over joint angle 2 (first joint is 0) and solves the rest analytically
//tries to stay close to currentangles
//takes in a 4x4 homogeneous transformation matrix for the hand base (meters) as a 16-vector in row order (rot4)
//resulting angles in resultangles
//returns 1 if solution is found, 0 otherwise
int wam_ik(double rot4[16], double currentangles[7], double resultangles[7]){
	int i, j;
	double preferredtheta2 = currentangles[2];
	double diff;
	double theta2;
	double disttolower = fabs(preferredtheta2 - wam_theta_min[2]);
	double disttoupper = fabs(wam_theta_max[2] - preferredtheta2);
	double maxdist = 0;
	int found = 0;

	//convert rot4 to a 4x4 matrix
	double tool[4][4];
	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
			tool[i][j] = rot4[i*4+j];
		}
	}
	
	//search over possible theta2 values
	if(disttolower > disttoupper) maxdist = disttolower;
	else maxdist = disttoupper;
	for(diff = 0; diff < maxdist; diff+=.02){
		theta2 = preferredtheta2 + diff;
		if(theta2 < wam_theta_max[2]){
			found = find_ik_solution(theta2, tool, currentangles, resultangles);
			if(found) break;
		}
		theta2 = preferredtheta2 - diff;
		if(theta2 > wam_theta_min[2]){
			found = find_ik_solution(theta2, tool, currentangles, resultangles);
			if(found) break;
		}
	}
	
	return found;
}

/*======================================================================*
 *  Module .............Kinematics
 *  File ...............WAMKinematics.c
 *  Author .............Manfred Huber
 *  Creation Date ......August 2008
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES: Kinematic functions and definitions for the WAM
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file WAMKinematics.c
   \brief Kinematic functions and definitions for the 7 DOF WAM

*/

void DispRotz(double disp[3], double theta, double trans[4][4])
{ 
  int i;

  trans[0][2] = trans[1][2] = trans[2][0] = trans[2][1] = trans[3][0] = trans[3][1] = trans[3][2] = 0.0;
  trans[2][2] = trans[3][3] = 1.0;

  for (i=0; i<3; i++)
    trans[i][3] = disp[i];

  trans[0][0] = trans[1][1] = cos(theta);
  trans[1][0] = sin(theta);
  trans[0][1] = - trans[1][0];
}


void DispRoty(double disp[3], double theta, double trans[4][4])
{ 
  int i;

  trans[0][1] = trans[1][0] = trans[1][2] = trans[2][1] = trans[3][0] = trans[3][1] = trans[3][2] = 0.0;
  trans[1][1] = trans[3][3] = 1.0;

  for (i=0; i<3; i++)
    trans[i][3] = disp[i];

  trans[0][0] = trans[2][2] = cos(theta);
  trans[0][2] = sin(theta);  
  trans[2][0] = - trans[0][2];
}

void HomTransMult(double trans1[4][4], double trans2[4][4], double result[4][4])
{
  int i, j, k;

  result[3][0] = result[3][1] = result[3][2] = 0.0;
  result[3][3] = 1.0;

  for (i=0; i<3; i++) {
    for (j=0; j<4; j++) {
      result[i][j] = 0.0; 
      for (k=0; k<3; k++) {
	result[i][j] += trans1[i][k]*trans2[k][j];
      }
    }
    result[i][3] += trans1[i][3];
  }
} 

int AngleRangeError(double theta[JNTS])
{
  int err = 0, i;
  
  for (i=0; i<JNTS; i++)
    err += ((theta[i] < wam_theta_min[i]) || (theta[i] > wam_theta_max[i])) << i;

  return(i);
}


void WAMForwardKinematics(double theta[JNTS], double x[4][4], double elbwrist[2][4][4])
{
  static double linkdisp[JNTS][3] = {{0.0, 0.0, 0.0}, 
				     {0.0, 0.0, 0.0}, 
				     {0.0, 0.0, 0.0}, 
				     {WAM_J3_OFFSET_X, 0.0, WAM_L1},
				     {-WAM_J3_OFFSET_X, 0.0, WAM_L2},
				     {0.0, 0.0, 0.0}, 
				     {0.0, 0.0, WAM_L3}};
  double transforms[3][4][4];

  DispRotz(linkdisp[0], theta[0], transforms[0]);
  DispRoty(linkdisp[1], theta[1], transforms[1]);
  HomTransMult(transforms[0], transforms[1], transforms[2]);
  DispRotz(linkdisp[2], theta[2], transforms[0]);
  HomTransMult(transforms[2], transforms[0], transforms[1]);
  DispRoty(linkdisp[3], theta[3], transforms[0]);
  if (elbwrist != NULL) {
    HomTransMult(transforms[1], transforms[0], elbwrist[0]);
    DispRotz(linkdisp[4], theta[4], transforms[0]);
    HomTransMult(elbwrist[0], transforms[0], transforms[1]);
  }
  else {
    HomTransMult(transforms[1], transforms[0], transforms[2]);
    DispRotz(linkdisp[4], theta[4], transforms[0]);
    HomTransMult(transforms[2], transforms[0], transforms[1]);
  }
  DispRoty(linkdisp[5], theta[5], transforms[0]);
  if (elbwrist != NULL) {
    HomTransMult(transforms[1], transforms[0], elbwrist[1]);
    DispRotz(linkdisp[6], theta[6], transforms[0]);
    HomTransMult(elbwrist[1], transforms[0], x);
  }
  else {
    HomTransMult(transforms[1], transforms[0], transforms[2]);

    DispRotz(linkdisp[6], theta[6], transforms[0]);
    HomTransMult(transforms[2], transforms[0], x);
  }
}


int WAMInverseKinematics(double theta2, double x[4][4], double theta_ref[JNTS], double theta[JNTS])
{
  int i, smooth;
  int elsol, basesol, shosol, wrincsol, wrrotsol, toolrotsol;
  int testelsol, testbasesol, testshosol, testwrincsol, testwrrotsol;
  double wristloc[3], wristdist;
  double elboffang[2];
  double elbowthetaprime;
  double elbowang[2], baseang[2], shouldang[2], wrincang[2], wrrotang[2], toolrotang[2];
  double baseoffdist[2], shouldoffdist[2], endoffdist[3], shouldbasedist;
  double baseoffang, shouldoffang, diffang;
  double tmpratio, desang;
  double tmplength[2];
  double transforms[3][4][4], wristpreinctrans[4][4];
  double totwrot, currtotrot, totplay, totrotdist[2], finrotdist[2];
  static double nodisp[3] = {0.0, 0.0, 0.0};
  int soltestcnt = 0;

  static double linkdisp[JNTS][3] = {{0.0, 0.0, 0.0}, 
				     {0.0, 0.0, 0.0}, 
				     {0.0, 0.0, 0.0}, 
				     {WAM_J3_OFFSET_X, 0.0, WAM_L1},
				     {-WAM_J3_OFFSET_X, 0.0, WAM_L2},
				     {0.0, 0.0, 0.0}, 
				     {0.0, 0.0, WAM_L3}};

  /* Computer wrist location */
  for (i=0; i<3; i++) {
    wristloc[i] = x[i][3] - WAM_L3*x[i][2];
  }

  /* Set the given elbow orientation into theta[2] */
  theta[2] = theta2;

  /* Compute the elbow angle */
  elboffang[0] = atan2(WAM_J3_OFFSET_X, WAM_L1);  // Elbow offset angles due to joint displacement
  elboffang[1] = atan2(WAM_J3_OFFSET_X, WAM_L2);
  wristdist = sqrt(SQR(wristloc[0]) + SQR(wristloc[1]) + SQR(wristloc[2])); 
  tmplength[0] = sqrt(SQR(WAM_L1) + SQR(WAM_J3_OFFSET_X));
  tmplength[1] = sqrt(SQR(WAM_L2) + SQR(WAM_J3_OFFSET_X));
  tmpratio = (SQR(tmplength[0]) + SQR(tmplength[1]) - SQR(wristdist))/(2.0*tmplength[0]*tmplength[1]);
  if (tmpratio > 1.0)
    tmpratio = 1.0;
  if (tmpratio < -1.0){
		return -1.0;
  }
  elbowthetaprime = acos(tmpratio);
  elbowang[0] = (MPI - elbowthetaprime) + elboffang[0] + elboffang[1];
  elbowang[1] = -(MPI - elbowthetaprime) + elboffang[0] + elboffang[1]; 

  /* Check how many elbow angle solutions are legal */
  while (elbowang[0] > wam_theta_max[3]) 
    elbowang[0] -= 2.0*MPI;
  while (elbowang[0] < wam_theta_min[3]) 
    elbowang[0] += 2.0*MPI;
  while (elbowang[1] > wam_theta_max[3]) 
    elbowang[1] -= 2.0*MPI;
  while (elbowang[1] < wam_theta_min[3]) 
    elbowang[1] += 2.0*MPI;

  elsol = 0;
  if ((elbowang[1] <= wam_theta_max[3]) && (elbowang[1] >= wam_theta_min[3])) {
    elsol++;
    theta[3] = elbowang[1];
  }
  if ((elbowang[0] <= wam_theta_max[3]) && (elbowang[0] >= wam_theta_min[3])) {
    elsol++;
    theta[3] = elbowang[0];
  }
  switch (elsol) {
  case 1:
    elbowang[0] = theta[3];
    break;
  case 2:
    if (SQR(elbowang[0] - theta_ref[3]) > SQR(elbowang[1] - theta_ref[3])) {
      theta[3] = elbowang[1];
      elbowang[1] = elbowang[0];
      elbowang[0] = theta[3];
    }
    break;
  case 0:
    return(0);
  }
  
  /* For each elbow solution look for a possible solution for the other angles until one is found */
  for (testelsol = 0; testelsol < elsol; testelsol++) { 
    theta[3] = elbowang[testelsol];

    /* Compute the base rotation (theta[0]) for the given elbow orientation (theta[2] = theta2) */
    baseoffdist[0] = -sin(theta2)*(tmplength[1]*sin(elbowang[testelsol]-elboffang[1]) + WAM_J3_OFFSET_X);
    baseoffdist[1] = sqrt(SQR(wristloc[0]) + SQR(wristloc[1]));
    if (fabs(baseoffdist[1]) < NEGLIGIBLE) {
      baseang[0] = theta_ref[0];
      theta[0] = baseang[0];
      basesol = INFINITE;
    }
    else {
      tmpratio = baseoffdist[0]/baseoffdist[1];
      if (tmpratio > 1.0)
	tmpratio = 1.0;
      if (tmpratio < -1.0)
	tmpratio = -1.0;
      baseoffang = asin(tmpratio);
      
      baseang[0] = atan2(wristloc[1], wristloc[0]) + baseoffang;
      baseang[1] = baseang[0] - 2.0*baseoffang + MPI;
      
      while (baseang[0] > wam_theta_max[0])
	baseang[0] -= 2.0*MPI;
      while (baseang[0] < wam_theta_min[0])
	baseang[0] += 2.0*MPI;
      while (baseang[1] > wam_theta_max[0])
	baseang[1] -= 2.0*MPI;
      while (baseang[1] < wam_theta_min[0])
	baseang[1] += 2.0*MPI;
      
      basesol = 0;
      if ((baseang[1] <= wam_theta_max[0]) && (baseang[1] >= wam_theta_min[0])) {
	basesol++;
	theta[0] = baseang[1];
      }
      if ((baseang[0] <= wam_theta_max[0]) && (baseang[0] >= wam_theta_min[0])) {
	basesol++;
	theta[0] = baseang[0];
      }
      switch (basesol) {
      case 1:
	baseang[0] = theta[0];
	break;
      case 2:
	if (SQR(baseang[0] - theta_ref[0]) > SQR(baseang[1] - theta_ref[0])) {
	  theta[0] = baseang[1];
	  baseang[1] = baseang[0];
	  baseang[0] = theta[0];
	}
	break;
      case 0:
	break;
      }
    }
    
    /* For each base/elbow combination look for a solution until one is found */
    for (testbasesol=0; testbasesol<basesol; testbasesol++) {
      theta[0] = baseang[testbasesol];
      /* Compute the shoulder angle (theta[1]) for the fiven elbow orientation (theta[2] = theta2) */
      shouldoffdist[0] = cos(theta2)*(tmplength[1]*sin(elbowang[testelsol]-elboffang[1]) + WAM_J3_OFFSET_X);
      shouldoffdist[1] = WAM_L1+tmplength[1]*cos(elbowang[testelsol]-elboffang[1]);
      shouldoffang = atan2(shouldoffdist[0], shouldoffdist[1]);
      shouldbasedist = cos(baseang[testbasesol])*wristloc[0]+sin(baseang[testbasesol])*wristloc[1];

      if ((fabs(wristloc[2]) < NEGLIGIBLE) && (fabs(shouldbasedist) < NEGLIGIBLE)) {  //Given the link length of the WAM this should never happen
	shosol = INFINITE;
	theta[1] = shouldang[0] = theta_ref[1];
      }
      else {
	shouldang[0] = atan2(shouldbasedist, wristloc[2]) - shouldoffang;
	if (shouldang[0] > wam_theta_max[1])
	  theta[1] = shouldang[0] -= 2.0*MPI;
	if (shouldang[0] < wam_theta_min[1])
	  shosol = 0;
	else
	  shosol = 1;
      }

      /* For each base/elbow/shoulder combination look for a solution until one is found */
      for (testshosol=0; testshosol<shosol; testshosol++) {
	theta[1] = shouldang[testshosol];
	/* Compute the wrist angles */
	/* Compute wrist orientation for current angle solution */
	DispRotz(linkdisp[0], baseang[testbasesol], transforms[0]);
	DispRoty(linkdisp[1], shouldang[testshosol], transforms[1]);
	HomTransMult(transforms[0], transforms[1], transforms[2]);
	DispRotz(linkdisp[2], theta2, transforms[0]);
	HomTransMult(transforms[2], transforms[0], transforms[1]);
	DispRoty(linkdisp[3], elbowang[testelsol], transforms[0]);
	HomTransMult(transforms[1], transforms[0], wristpreinctrans);

	DispRotz(linkdisp[4], 0, transforms[0]);
	HomTransMult(wristpreinctrans, transforms[0], transforms[1]);

	/* Compute wrist inclination */
	endoffdist[2] = wristpreinctrans[0][2]*x[0][2] + wristpreinctrans[1][2]*x[1][2] + wristpreinctrans[2][2]*x[2][2];
	totrotdist[0] = x[0][0]*wristpreinctrans[0][0] + x[1][0]*wristpreinctrans[1][0] + x[2][0]*wristpreinctrans[2][0];
	totrotdist[1] = x[0][0]*wristpreinctrans[0][1] + x[1][0]*wristpreinctrans[1][1] + x[2][0]*wristpreinctrans[2][1];

	wrincang[0] = acos(endoffdist[2]);
	wrincang[1] = - wrincang[0];
	
	wrincsol = 0;
	if (wrincang[1] >= wam_theta_min[5]) {
	  wrincsol++;
	  theta[5] = wrincang[1];
	}
	if (wrincang[0] <= wam_theta_max[5]) {
	  wrincsol++;
	  theta[5] = wrincang[0];
	}
	switch (wrincsol) {
	case 1:
	  wrincang[0] = theta[5];
	  break;
	case 2:
	  if (SQR(wrincang[0] - theta_ref[5]) > SQR(wrincang[1] - theta_ref[5])) {
	    theta[5] = wrincang[1];
	    wrincang[1] = wrincang[0];
	    wrincang[0] = theta[5];
	  }
	  break;
	default:
	  break;
	}

	/* For each base/elbow/shoulder/wrist inclination combination look for a solution until one is found */
	for (testwrincsol=0; testwrincsol<wrincsol; testwrincsol++) {
	  theta[5] = wrincang[testwrincsol];
	  endoffdist[0] = x[0][2]*wristpreinctrans[0][0] + x[1][2]*wristpreinctrans[1][0] + x[2][2]*wristpreinctrans[2][0];
	  endoffdist[1] = x[0][2]*wristpreinctrans[0][1] + x[1][2]*wristpreinctrans[1][1] + x[2][2]*wristpreinctrans[2][1];

	  if ((fabs(endoffdist[0]) < NEGLIGIBLE) && (fabs(endoffdist[1]) < NEGLIGIBLE)) {
	    wrrotsol = INFINITE;
	    theta[4] = wrrotang[0] = theta_ref[4];
	  } 
	  else {
	    wrrotang[0] = atan2(endoffdist[1], endoffdist[0]);
	    if (wrincang[testwrincsol] < 0.0)
	      wrrotang[0] += MPI;
	    if (wrrotang[0] > wam_theta_max[4])
	      wrrotang[0] -= 2.0*MPI;
	    if (wrrotang[0] < wam_theta_min[4])
	      wrrotang[0] += 2.0*MPI;
	    if ((wrrotang[0] <= wam_theta_max[4]) &&(wrrotang[0] >= wam_theta_min[4])) {
	      theta[4] = wrrotang[0];
	      wrrotsol = 1;
	    }
	    else {
	      wrrotsol = 0;
	    }
	  }

	  /* For each base/elbow/shoulder/wrist inc./wrist rot. combination look for a solution until one is found */
	  for (testwrrotsol=0; testwrrotsol<wrrotsol; testwrrotsol++) {
	    theta[4] = wrrotang[testwrrotsol];
	    DispRotz(nodisp, wrrotang[testwrrotsol], transforms[0]);
	    HomTransMult(wristpreinctrans, transforms[0], transforms[1]);
	    DispRoty(nodisp, wrincang[testwrincsol], transforms[0]);
	    HomTransMult(transforms[1], transforms[0], transforms[2]);
	    
	    finrotdist[0] = x[0][0]*transforms[2][0][0] + x[1][0]*transforms[2][1][0] + x[2][0]*transforms[2][2][0];
	    finrotdist[1] = x[0][0]*transforms[2][0][1] + x[1][0]*transforms[2][1][1] + x[2][0]*transforms[2][2][1];
	    toolrotang[0] = atan2(finrotdist[1], finrotdist[0]);

	    while (toolrotang[0] < wam_theta_min[6])
	      toolrotang[0] += 2.0*MPI;
	    while (toolrotang[0] > wam_theta_max[6])
	      toolrotang[0] -= 2.0*MPI;
	    if ((toolrotang[0] >= wam_theta_min[6]) && (toolrotang[0] <= wam_theta_max[6])) {
	      theta[6] = toolrotang[0];
	      soltestcnt++;
	      return(soltestcnt);
	    }
	    else {
	      toolrotsol = 0;
	      /* If there were an infinite number of choices for the wrist rotation, pick one that is 
		 compatible with a legal tool rotation - note, infinite wrist rotations implies 0 wrist
	         inclination and thus straight alignment of the two rotation frames and thus total wrist
	         rotation = theta[4] + theta[6]                                                          */
	      if (wrrotsol == INFINITE) {
		/* If there is also an infinite number of base rotations then the robot is straight up
		   and all roation parameters should be balanced.                                        */
		toolrotsol = INFINITE;
		if (basesol == INFINITE) {
		  totwrot = atan2(x[0][1], x[0][0]);
		  currtotrot = theta[0] + theta_ref[4] + theta_ref[6];
		  diffang = totwrot - currtotrot;
		  while (diffang > MPI)
		    diffang -= 2.0*MPI;
		  while (diffang < -MPI)
		    diffang += 2.0*MPI;
		  if (diffang >= 0.0) {
		    totplay = (wam_theta_max[0] - theta_ref[0]) + (wam_theta_max[4] - theta_ref[4]) + 
		      (wam_theta_max[6] - theta_ref[6]);
		    smooth = (totplay > diffang);
		  }
		  else {
		    totplay = (wam_theta_min[0] - theta_ref[0]) + (wam_theta_min[4] - theta_ref[4]) + 
		      (wam_theta_min[6] - theta_ref[6]);
		    smooth = (totplay < diffang);
		  }
		  diffang /= 3.0;
		  if (smooth) { //No angle jum is necessary
		    theta[6] = toolrotang[0] = theta_ref[6] + diffang;
		    if ((diffang >= 0.0) && (toolrotang[0] > wam_theta_max[6])) {
		      diffang += 0.5*(toolrotang[0] - wam_theta_max[6]);
		      theta[6] = toolrotang[0] = wam_theta_max[6];
		    }
		    else {
		      if (toolrotang[0] < wam_theta_min[6]) {
			diffang += 0.5*(toolrotang[0] - wam_theta_min[6]);
			theta[6] = toolrotang[0] = wam_theta_min[6];
		      }
		    }
		    
		    theta[4] = wrrotang[testwrrotsol] = theta_ref[4] + diffang;
		    if ((diffang >= 0.0) && (wrrotang[testwrrotsol] > wam_theta_max[4])) {
		      diffang += (wrrotang[testwrrotsol] - wam_theta_max[4]);
		      theta[4] = wrrotang[testwrrotsol] = wam_theta_max[4];
		    }
		    else {
		      if (wrrotang[testwrrotsol] < wam_theta_min[4]) {
			diffang += (wrrotang[testwrrotsol] - wam_theta_min[4]);
			theta[4] = wrrotang[testwrrotsol] = wam_theta_min[4];
		      }
		    }
		    
		    theta[0] = baseang[testbasesol] = theta_ref[0] + diffang;
		  }
		  else {
		    /* asign angles evenly from the middle of their range. */
		    theta[0] = 0.5*(wam_theta_max[0] + wam_theta_min[0]) + diffang;
		    theta[4] = 0.5*(wam_theta_max[4] + wam_theta_min[4]) + diffang;
		    theta[6] = 0.5*(wam_theta_max[6] + wam_theta_min[6]) + diffang;
		  }
		  soltestcnt++;
		  return(soltestcnt);
		}
		else {
		  totwrot = atan2(totrotdist[1], totrotdist[0]);
		  currtotrot = theta_ref[4] + theta_ref[6];
		  diffang = totwrot - currtotrot;
		  if (diffang > MPI)
		    diffang -= 2.0*MPI;
		  if (diffang < -MPI)
		    diffang += 2.0*MPI;
		  if (diffang >= 0.0) {
		    totplay = (wam_theta_max[4] - theta_ref[4]) + (wam_theta_max[6] - theta_ref[6]);
		    smooth = (totplay > diffang);
		  }
		  else {
		    totplay = (wam_theta_min[4] - theta_ref[4]) + (wam_theta_min[6] - theta_ref[6]);
		    smooth = (totplay < diffang);
		  }
		  diffang /= 2.0;
		  if (smooth) { //No angle jum is necessary
		    theta[6] = toolrotang[0] = theta_ref[6] + diffang;
		    if ((diffang >= 0.0) && (toolrotang[0] > wam_theta_max[6])) {
		      diffang += toolrotang[0] - wam_theta_max[6];
		      theta[6] = toolrotang[0] = wam_theta_max[6];
		    }
		    else {
		      if (toolrotang[0] < wam_theta_min[6]) {
			diffang += toolrotang[0] - wam_theta_min[6];
			theta[6] = toolrotang[0] = wam_theta_min[6];
		      }
		    }
		    theta[4] = wrrotang[testwrrotsol] = theta_ref[4] + diffang;
		  }
		  else {
		    /* asign angles evenly from the middle of their range. */
		    theta[4] = 0.5*(wam_theta_max[4] + wam_theta_min[4]) + diffang;
		    theta[6] = 0.5*(wam_theta_max[6] + wam_theta_min[6]) + diffang;
		  }
		  soltestcnt++;
		  return(soltestcnt);
		}
	      }
	    }
	    soltestcnt++;
	  }
	  soltestcnt++;
	}
	/* If no solution for the wrist angle was found but there are an infinite number of shoulder inclinations,
	   find one that will work for the wrist. Note: This would really require different link lengths to ever
	   happen.                                                                                                */ 
	if (shosol == INFINITE) {
	  shosol = 0;
	  return(-INFINITE);
	}
	soltestcnt++;
      }
      /* If no solution has been found so far but there are an infinite number of shoulder inclinations,
	 find one that will work for the wrist.                                                            */
      if (basesol == INFINITE) {
	if ((fabs(x[3][0]) < NEGLIGIBLE) && (fabs(x[3][1]) < NEGLIGIBLE)) {   //If this is the case a solution should have been already found
	  basesol = 0;
	  return(-INFINITE);
	}
	else {
	  theta[0] = baseang[1] = atan2(x[3][1], x[3][0]);
	  basesol = 2;
	}
      }
      soltestcnt++;
    }
  }
  return (-soltestcnt);
}

#ifdef  MATLAB_MEX_FILE
// This is used when the MEX file is called.  It is not used when the Simulink model is built
void mexFunction(int nlhs, mxArray *plhs[ ],int nrhs, const mxArray *prhs[ ])
{ 
    int m, n, foundResult;
    double desPos[16]; // Will hold the flattened transformation matrix
    double *indesPos; // Holds the transformation matrix straight from input (4x4 double matrix)
    double* q;       // Will hold the actual joint angles
    double* qCurr;   // Holds the current joint angles
    double* qOut;    // Pointer to output array
    
    // Allocate space for the actual joint angles
    q = mxCalloc(7,sizeof(double));
    
    // Check for the proper number of arguments
    if(nrhs != 2)
    {
        mexErrMsgTxt("Incorrect number of arguments.  Usage: ikine7dof(qCurr,desiredPose) ");
    }
    // Check Dimensionality of first argument
    m = mxGetM(prhs[0]);n = mxGetN(prhs[0]);
    if(m != 1 || n != 7)
    {
        mexErrMsgTxt("Invalid format for argument 1.  Must be a row vector with 7 elements");
    }
    // Check Dimensionality of second argument
    m = mxGetM(prhs[1]); n = mxGetM(prhs[1]);
    if(m != 4 || n != 4)
    {
        mexErrMsgTxt("Invalid format for argument 2.  Must be a 4x4 matrix");
    }
    
    // Get the current joint angles
    qCurr = mxGetPr(prhs[0]);
    // Get the transformation matrix
    indesPos = mxGetPr(prhs[1]);
    
    /*
    mexPrintf("Current Joint Angles: \n" );
    for(m = 0; m < 7; m++)
    {
        mexPrintf("  %f\n",qCurr[m]);
    }
    */
    
    // Flatten the transformation matrix
    for( m = 0; m < 4; m++)
    {
     for(n = 0; n < 4; n++)
     {
       desPos[n*4+m] = indesPos[m*4+n];
     }
    }
    
    // Get pointer to output arrays
    plhs[0] = mxCreateDoubleMatrix(1,7,mxREAL);
    qOut = mxGetPr(plhs[0]);
    
    // Do the inverse kinematics
    foundResult = wam_ik(desPos,qCurr,qOut);
    
    plhs[1] = mxCreateDoubleScalar((double)foundResult);
        
    mxFree(q);
}
#endif
