/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[2] = acadoVariables.x[2];
acadoWorkspace.state[3] = acadoVariables.x[3];
acadoWorkspace.state[4] = acadoVariables.x[4];
acadoWorkspace.state[5] = acadoVariables.x[5];
acadoWorkspace.state[54] = acadoVariables.u[0];
acadoWorkspace.state[55] = acadoVariables.u[1];
acadoWorkspace.state[56] = acadoVariables.od[0];
acadoWorkspace.state[57] = acadoVariables.od[1];
acadoWorkspace.state[58] = acadoVariables.od[2];
acadoWorkspace.state[59] = acadoVariables.od[3];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{

acadoWorkspace.state[54] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[55] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[56] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.state[57] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.state[58] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.state[59] = acadoVariables.od[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[lRun1 * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[lRun1 * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[lRun1 * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[lRun1 * 6 + 11] = acadoWorkspace.state[5];

acadoWorkspace.evGx[lRun1 * 36] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 36 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 36 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 36 + 3] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 36 + 4] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 36 + 5] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 36 + 6] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 36 + 7] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 36 + 8] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 36 + 9] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 36 + 10] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 36 + 11] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 36 + 12] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 36 + 13] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 36 + 14] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 36 + 15] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 36 + 16] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 36 + 17] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 36 + 18] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 36 + 19] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 36 + 20] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 36 + 21] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 36 + 22] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 36 + 23] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 36 + 24] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 36 + 25] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 36 + 26] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 36 + 27] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 36 + 28] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 36 + 29] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 36 + 30] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 36 + 31] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 36 + 32] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 36 + 33] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 36 + 34] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 36 + 35] = acadoWorkspace.state[41];

acadoWorkspace.evGu[lRun1 * 12] = acadoWorkspace.state[42];
acadoWorkspace.evGu[lRun1 * 12 + 1] = acadoWorkspace.state[43];
acadoWorkspace.evGu[lRun1 * 12 + 2] = acadoWorkspace.state[44];
acadoWorkspace.evGu[lRun1 * 12 + 3] = acadoWorkspace.state[45];
acadoWorkspace.evGu[lRun1 * 12 + 4] = acadoWorkspace.state[46];
acadoWorkspace.evGu[lRun1 * 12 + 5] = acadoWorkspace.state[47];
acadoWorkspace.evGu[lRun1 * 12 + 6] = acadoWorkspace.state[48];
acadoWorkspace.evGu[lRun1 * 12 + 7] = acadoWorkspace.state[49];
acadoWorkspace.evGu[lRun1 * 12 + 8] = acadoWorkspace.state[50];
acadoWorkspace.evGu[lRun1 * 12 + 9] = acadoWorkspace.state[51];
acadoWorkspace.evGu[lRun1 * 12 + 10] = acadoWorkspace.state[52];
acadoWorkspace.evGu[lRun1 * 12 + 11] = acadoWorkspace.state[53];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = u[0];
out[4] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 4];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 4 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 4 + 2];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acadoWorkspace.objValueIn[6] = acadoVariables.od[40];
acadoWorkspace.objValueIn[7] = acadoVariables.od[41];
acadoWorkspace.objValueIn[8] = acadoVariables.od[42];
acadoWorkspace.objValueIn[9] = acadoVariables.od[43];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5];
dNew[1] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2] + Gx1[9]*dOld[3] + Gx1[10]*dOld[4] + Gx1[11]*dOld[5];
dNew[2] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5];
dNew[3] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5];
dNew[4] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5];
dNew[5] += + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[24] + Gx1[5]*Gx2[30];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[25] + Gx1[5]*Gx2[31];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[20] + Gx1[4]*Gx2[26] + Gx1[5]*Gx2[32];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[27] + Gx1[5]*Gx2[33];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[34];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[35];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[30];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[31];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[8]*Gx2[14] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[32];
Gx3[9] = + Gx1[6]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[21] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[33];
Gx3[10] = + Gx1[6]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[22] + Gx1[10]*Gx2[28] + Gx1[11]*Gx2[34];
Gx3[11] = + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[23] + Gx1[10]*Gx2[29] + Gx1[11]*Gx2[35];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[15]*Gx2[18] + Gx1[16]*Gx2[24] + Gx1[17]*Gx2[30];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[15]*Gx2[19] + Gx1[16]*Gx2[25] + Gx1[17]*Gx2[31];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[15]*Gx2[20] + Gx1[16]*Gx2[26] + Gx1[17]*Gx2[32];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[15]*Gx2[21] + Gx1[16]*Gx2[27] + Gx1[17]*Gx2[33];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[15]*Gx2[22] + Gx1[16]*Gx2[28] + Gx1[17]*Gx2[34];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[15]*Gx2[23] + Gx1[16]*Gx2[29] + Gx1[17]*Gx2[35];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[6] + Gx1[20]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[30];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[7] + Gx1[20]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[31];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[8] + Gx1[20]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[32];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[33];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[34];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[35];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[6] + Gx1[26]*Gx2[12] + Gx1[27]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[29]*Gx2[30];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[7] + Gx1[26]*Gx2[13] + Gx1[27]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[29]*Gx2[31];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[14] + Gx1[27]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[29]*Gx2[32];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[15] + Gx1[27]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[29]*Gx2[33];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[29]*Gx2[34];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[29]*Gx2[35];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[6] + Gx1[32]*Gx2[12] + Gx1[33]*Gx2[18] + Gx1[34]*Gx2[24] + Gx1[35]*Gx2[30];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[7] + Gx1[32]*Gx2[13] + Gx1[33]*Gx2[19] + Gx1[34]*Gx2[25] + Gx1[35]*Gx2[31];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[8] + Gx1[32]*Gx2[14] + Gx1[33]*Gx2[20] + Gx1[34]*Gx2[26] + Gx1[35]*Gx2[32];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[9] + Gx1[32]*Gx2[15] + Gx1[33]*Gx2[21] + Gx1[34]*Gx2[27] + Gx1[35]*Gx2[33];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[16] + Gx1[33]*Gx2[22] + Gx1[34]*Gx2[28] + Gx1[35]*Gx2[34];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[17] + Gx1[33]*Gx2[23] + Gx1[34]*Gx2[29] + Gx1[35]*Gx2[35];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[10];
Gu2[3] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[11];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6] + Gx1[16]*Gu1[8] + Gx1[17]*Gu1[10];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7] + Gx1[16]*Gu1[9] + Gx1[17]*Gu1[11];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[2] + Gx1[20]*Gu1[4] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[10];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[5] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[11];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11];
Gu2[10] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[2] + Gx1[32]*Gu1[4] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[10];
Gu2[11] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[5] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[11];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = (real_t)1.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = (real_t)1.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = 0.0;
;
dNew[1] = 0.0;
;
dNew[2] = 0.0;
;
dNew[3] = +dOld[3];
dNew[4] = +dOld[4];
dNew[5] = +dOld[5];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[3];
RDy1[1] = +Dy1[4];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = 0.0;
;
QDy1[1] = 0.0;
;
QDy1[2] = 0.0;
;
QDy1[3] = +Dy1[0];
QDy1[4] = +Dy1[1];
QDy1[5] = +Dy1[2];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4] + E1[10]*QDy1[5];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4] + E1[11]*QDy1[5];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[6] + E1[4]*Gx1[12] + E1[6]*Gx1[18] + E1[8]*Gx1[24] + E1[10]*Gx1[30];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[7] + E1[4]*Gx1[13] + E1[6]*Gx1[19] + E1[8]*Gx1[25] + E1[10]*Gx1[31];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[8] + E1[4]*Gx1[14] + E1[6]*Gx1[20] + E1[8]*Gx1[26] + E1[10]*Gx1[32];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[9] + E1[4]*Gx1[15] + E1[6]*Gx1[21] + E1[8]*Gx1[27] + E1[10]*Gx1[33];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[10] + E1[4]*Gx1[16] + E1[6]*Gx1[22] + E1[8]*Gx1[28] + E1[10]*Gx1[34];
H101[5] += + E1[0]*Gx1[5] + E1[2]*Gx1[11] + E1[4]*Gx1[17] + E1[6]*Gx1[23] + E1[8]*Gx1[29] + E1[10]*Gx1[35];
H101[6] += + E1[1]*Gx1[0] + E1[3]*Gx1[6] + E1[5]*Gx1[12] + E1[7]*Gx1[18] + E1[9]*Gx1[24] + E1[11]*Gx1[30];
H101[7] += + E1[1]*Gx1[1] + E1[3]*Gx1[7] + E1[5]*Gx1[13] + E1[7]*Gx1[19] + E1[9]*Gx1[25] + E1[11]*Gx1[31];
H101[8] += + E1[1]*Gx1[2] + E1[3]*Gx1[8] + E1[5]*Gx1[14] + E1[7]*Gx1[20] + E1[9]*Gx1[26] + E1[11]*Gx1[32];
H101[9] += + E1[1]*Gx1[3] + E1[3]*Gx1[9] + E1[5]*Gx1[15] + E1[7]*Gx1[21] + E1[9]*Gx1[27] + E1[11]*Gx1[33];
H101[10] += + E1[1]*Gx1[4] + E1[3]*Gx1[10] + E1[5]*Gx1[16] + E1[7]*Gx1[22] + E1[9]*Gx1[28] + E1[11]*Gx1[34];
H101[11] += + E1[1]*Gx1[5] + E1[3]*Gx1[11] + E1[5]*Gx1[17] + E1[7]*Gx1[23] + E1[9]*Gx1[29] + E1[11]*Gx1[35];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 12; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
dNew[5] += + E1[10]*U1[0] + E1[11]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = 0.0;
;
Gx2[1] = 0.0;
;
Gx2[2] = 0.0;
;
Gx2[3] = 0.0;
;
Gx2[4] = 0.0;
;
Gx2[5] = 0.0;
;
Gx2[6] = 0.0;
;
Gx2[7] = 0.0;
;
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = 0.0;
;
Gx2[13] = 0.0;
;
Gx2[14] = 0.0;
;
Gx2[15] = 0.0;
;
Gx2[16] = 0.0;
;
Gx2[17] = 0.0;
;
Gx2[18] = +Gx1[18];
Gx2[19] = +Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = +Gx1[24];
Gx2[25] = +Gx1[25];
Gx2[26] = +Gx1[26];
Gx2[27] = +Gx1[27];
Gx2[28] = +Gx1[28];
Gx2[29] = +Gx1[29];
Gx2[30] = +Gx1[30];
Gx2[31] = +Gx1[31];
Gx2[32] = +Gx1[32];
Gx2[33] = +Gx1[33];
Gx2[34] = +Gx1[34];
Gx2[35] = +Gx1[35];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = 0.0;
;
Gx2[1] = 0.0;
;
Gx2[2] = 0.0;
;
Gx2[3] = 0.0;
;
Gx2[4] = 0.0;
;
Gx2[5] = 0.0;
;
Gx2[6] = 0.0;
;
Gx2[7] = 0.0;
;
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = 0.0;
;
Gx2[13] = 0.0;
;
Gx2[14] = 0.0;
;
Gx2[15] = 0.0;
;
Gx2[16] = 0.0;
;
Gx2[17] = 0.0;
;
Gx2[18] = +Gx1[18];
Gx2[19] = +Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = +Gx1[24];
Gx2[25] = +Gx1[25];
Gx2[26] = +Gx1[26];
Gx2[27] = +Gx1[27];
Gx2[28] = +Gx1[28];
Gx2[29] = +Gx1[29];
Gx2[30] = +Gx1[30];
Gx2[31] = +Gx1[31];
Gx2[32] = +Gx1[32];
Gx2[33] = +Gx1[33];
Gx2[34] = +Gx1[34];
Gx2[35] = +Gx1[35];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = 0.0;
;
Gu2[1] = 0.0;
;
Gu2[2] = 0.0;
;
Gu2[3] = 0.0;
;
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
Gu2[10] = +Gu1[10];
Gu2[11] = +Gu1[11];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = 0.0;
;
Gu2[1] = 0.0;
;
Gu2[2] = 0.0;
;
Gu2[3] = 0.0;
;
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
Gu2[10] = +Gu1[10];
Gu2[11] = +Gu1[11];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 10 */
static const int xBoundIndices[ 10 ] = 
{ 9, 15, 21, 27, 33, 39, 45, 51, 57, 63 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 12 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 108 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 96 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 108 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 156 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 168 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 180 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 204 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 228 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 240 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGx[ 216 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 312 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 324 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGx[ 252 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 348 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 372 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.E[ 396 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 408 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.E[ 420 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.E[ 444 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.E[ 468 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.E[ 492 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.E[ 504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 516 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 528 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 324 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.E[ 552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.E[ 564 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 588 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.E[ 612 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 636 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 648 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 108 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 552 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 564 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 564 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 648 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.g[ 18 ]) );
acadoWorkspace.lb[0] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)6.9999999999999996e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)6.9999999999999996e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.9999999999999996e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.9999999999999996e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)6.9999999999999996e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.9999999999999996e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.9999999999999996e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)6.9999999999999996e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.9999999999999996e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.9999999999999996e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+00 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 6;
lRun4 = ((lRun3) / (6)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (6)) + ((lRun3) % (6));
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 54 ]) );

acadoWorkspace.QDy[60] = 0.0;
;
acadoWorkspace.QDy[61] = 0.0;
;
acadoWorkspace.QDy[62] = 0.0;
;
acadoWorkspace.QDy[63] = +acadoWorkspace.DyN[0];
acadoWorkspace.QDy[64] = +acadoWorkspace.DyN[1];
acadoWorkspace.QDy[65] = +acadoWorkspace.DyN[2];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[1] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[2] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[3] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[4] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[5] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[6] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[7] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[8] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[9] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[10] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[11] += + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[12] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[13] += + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[14] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[15] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[16] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[17] += + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[18] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[19] += + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[5];

tmp = + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[5] + acadoVariables.x[9];
acadoWorkspace.lbA[0] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[5] + acadoVariables.x[15];
acadoWorkspace.lbA[1] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5] + acadoVariables.x[21];
acadoWorkspace.lbA[2] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[5] + acadoVariables.x[27];
acadoWorkspace.lbA[3] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[5] + acadoVariables.x[33];
acadoWorkspace.lbA[4] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[5] + acadoVariables.x[39];
acadoWorkspace.lbA[5] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[5] + acadoVariables.x[45];
acadoWorkspace.lbA[6] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5] + acadoVariables.x[51];
acadoWorkspace.lbA[7] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[5] + acadoVariables.x[57];
acadoWorkspace.lbA[8] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)2.0000000000000000e+01 - tmp;
tmp = + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[5] + acadoVariables.x[63];
acadoWorkspace.lbA[9] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)2.0000000000000000e+01 - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];

acadoVariables.x[6] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5];
acadoVariables.x[7] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[5];
acadoVariables.x[8] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[5];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[5];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[5];
acadoVariables.x[12] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[5];
acadoVariables.x[13] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[5];
acadoVariables.x[14] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[5];
acadoVariables.x[15] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[5];
acadoVariables.x[16] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[5];
acadoVariables.x[17] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[5];
acadoVariables.x[18] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5];
acadoVariables.x[19] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[5];
acadoVariables.x[20] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[5];
acadoVariables.x[21] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5];
acadoVariables.x[22] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5];
acadoVariables.x[23] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[5];
acadoVariables.x[24] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[5];
acadoVariables.x[25] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[5];
acadoVariables.x[26] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5];
acadoVariables.x[27] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[5];
acadoVariables.x[28] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[5];
acadoVariables.x[29] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[5];
acadoVariables.x[30] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5];
acadoVariables.x[31] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[5];
acadoVariables.x[32] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[5];
acadoVariables.x[33] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[5];
acadoVariables.x[34] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[5];
acadoVariables.x[35] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[5];
acadoVariables.x[36] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5];
acadoVariables.x[37] += + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[5];
acadoVariables.x[38] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[5];
acadoVariables.x[39] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[5];
acadoVariables.x[40] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[5];
acadoVariables.x[41] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[5];
acadoVariables.x[42] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5];
acadoVariables.x[43] += + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[5];
acadoVariables.x[44] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[5];
acadoVariables.x[45] += + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[5];
acadoVariables.x[46] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5];
acadoVariables.x[47] += + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[5];
acadoVariables.x[48] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[5];
acadoVariables.x[49] += + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[5];
acadoVariables.x[50] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[5];
acadoVariables.x[51] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5];
acadoVariables.x[52] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[5];
acadoVariables.x[53] += + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[5];
acadoVariables.x[54] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5];
acadoVariables.x[55] += + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[5];
acadoVariables.x[56] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[5];
acadoVariables.x[57] += + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[5];
acadoVariables.x[58] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[5];
acadoVariables.x[59] += + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[5];
acadoVariables.x[60] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[5];
acadoVariables.x[61] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[5];
acadoVariables.x[62] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[5];
acadoVariables.x[63] += + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[5];
acadoVariables.x[64] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[5];
acadoVariables.x[65] += + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[5];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), acadoWorkspace.x, &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), acadoWorkspace.x, &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 60 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 6];
acadoWorkspace.state[1] = acadoVariables.x[index * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 6 + 5];
acadoWorkspace.state[54] = acadoVariables.u[index * 2];
acadoWorkspace.state[55] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[56] = acadoVariables.od[index * 4];
acadoWorkspace.state[57] = acadoVariables.od[index * 4 + 1];
acadoWorkspace.state[58] = acadoVariables.od[index * 4 + 2];
acadoWorkspace.state[59] = acadoVariables.od[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[index * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[index * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[index * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[index * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[index * 6 + 11] = acadoWorkspace.state[5];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[60] = xEnd[0];
acadoVariables.x[61] = xEnd[1];
acadoVariables.x[62] = xEnd[2];
acadoVariables.x[63] = xEnd[3];
acadoVariables.x[64] = xEnd[4];
acadoVariables.x[65] = xEnd[5];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[60];
acadoWorkspace.state[1] = acadoVariables.x[61];
acadoWorkspace.state[2] = acadoVariables.x[62];
acadoWorkspace.state[3] = acadoVariables.x[63];
acadoWorkspace.state[4] = acadoVariables.x[64];
acadoWorkspace.state[5] = acadoVariables.x[65];
if (uEnd != 0)
{
acadoWorkspace.state[54] = uEnd[0];
acadoWorkspace.state[55] = uEnd[1];
}
else
{
acadoWorkspace.state[54] = acadoVariables.u[18];
acadoWorkspace.state[55] = acadoVariables.u[19];
}
acadoWorkspace.state[56] = acadoVariables.od[40];
acadoWorkspace.state[57] = acadoVariables.od[41];
acadoWorkspace.state[58] = acadoVariables.od[42];
acadoWorkspace.state[59] = acadoVariables.od[43];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[60] = acadoWorkspace.state[0];
acadoVariables.x[61] = acadoWorkspace.state[1];
acadoVariables.x[62] = acadoWorkspace.state[2];
acadoVariables.x[63] = acadoWorkspace.state[3];
acadoVariables.x[64] = acadoWorkspace.state[4];
acadoVariables.x[65] = acadoWorkspace.state[5];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acadoWorkspace.objValueIn[6] = acadoVariables.od[40];
acadoWorkspace.objValueIn[7] = acadoVariables.od[41];
acadoWorkspace.objValueIn[8] = acadoVariables.od[42];
acadoWorkspace.objValueIn[9] = acadoVariables.od[43];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1];
tmpDyN[2] = + acadoWorkspace.DyN[2];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

