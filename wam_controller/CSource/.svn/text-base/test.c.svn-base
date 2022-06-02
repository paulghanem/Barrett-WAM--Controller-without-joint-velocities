#include "mex.h"
#include <math.h>
#include <stdio.h>

#define WAM_INFO_IN    prhs[0]

void 
mexFunction( int     nlhs, mxArray     *plhs[], int     nrhs, const mxArray   *prhs[])
{
    mxArray*  mx_wamInfo;
    mxArray*  mx_arm;
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
    
    
    int i,j,k;
    
    mx_wamInfo = (mxArray *)WAM_INFO_IN;
    
    if(mxIsStruct(mx_wamInfo))
    {
        mexPrintf("Structure\n");
        
        mx_arm     = (mxArray*)mxGetField(mx_wamInfo,(mwIndex) 0,"arm");
        mx_arm_dh  = (mxArray*)mxGetField(mx_arm, (mwIndex) 0, "dh");
        mx_arm_dyn = (mxArray*)mxGetField(mx_arm, (mwIndex) 0, "dyn");
        
        arm_dh_a     = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "a"));
        arm_dh_alpha = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "alpha"));
        arm_dh_d     = mxGetPr(mxGetField(mx_arm_dh, (mwIndex) 0, "d"));
        
        arm_dyn_B    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "B"));
        arm_dyn_I    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "I"));
        arm_dyn_Jm   = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "Jm"));
        arm_dyn_m    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "m"));
        arm_dyn_r    = mxGetPr(mxGetField(mx_arm_dyn,(mwIndex) 0, "r"));
        
        mexPrintf("Mark 3\n");
        for(i = 0; i < 9
        j = 0;
        k = 0;
        mexPrintf("J%d::\n  %20.10f \n",i,arm_dyn_I[i+7*j+21*k]);
//        for(i = 0; i < 7; i++)
//        {
//            mexPrintf("J%d::\na:%10.5f b:%10.5f d:%10.5f\n",i,arm_dh_a[i],arm_dh_alpha[i],arm_dh_d[i]);
//            mexPrintf("B: %10.5f m: %10.5f  Jm: %10.5f\n\n",arm_dyn_B[i],arm_dyn_m[i], arm_dyn_Jm[i]);
//        }
            
        
    }
    else
    {
        mexPrintf("Not Struct\n");
    }
    
    
}
